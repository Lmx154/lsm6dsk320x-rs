//! On-board check of the lsm6dsk320x configuration options no other program touches (U23 on SPI1,
//! INT1 on GPIO7): gyroscope LPF1, low-g LPF2, DRDY mask and fast settling, high-accuracy ODR,
//! latched data-ready, pin polarity, output type and pad drive, status flags.
//!
//! The board lies still on the bench. No SPI transfer is ever cancelled: the only timeouts wrap
//! the wait for an INT1 edge or level. Only the driver's typed methods and raw register reads are
//! used. The pull setting of GPIO7 is restored to `Pull::None` before the final health check.
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Flex, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_7, PWM_SLICE3};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::{
    AccelFullScale, AccelLpf2Bandwidth, Config, DataReady, DataReadyMode, GyroFullScale,
    GyroLpf1Bandwidth, HaodrSet, Info, IntOutput, IntPolarity, IntRouting, Lsm6dsk320x, Odr,
    OperatingMode, PadDrive, PinConfig, RawAxes, reg,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const RUN_LIMIT_S: u64 = 110;
const G: f32 = lsm6dsk320x::sample::G;

static VERIFIED: AtomicU32 = AtomicU32::new(0);

/// Logs a verdict line. The format string starts with a `{}` that receives the tag.
macro_rules! check {
    ($fails:expr, $ok:expr, $fmt:literal $(, $arg:expr)*) => {{
        let ok = $ok;
        if !ok {
            $fails += 1;
        }
        defmt::info!($fmt, if ok { "[PASS]" } else { "[FAIL]" } $(, $arg)*);
    }};
}

fn sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut g = if x > 1.0 { x } else { 1.0 };
    for _ in 0..24 {
        g = 0.5 * (g + x / g);
    }
    g
}

fn abs(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

fn mag(v: [f32; 3]) -> f32 {
    sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

fn in_band(a_g: f32) -> bool {
    (0.9..=1.1).contains(&a_g)
}

fn accel_vec_g(r: RawAxes, fs: AccelFullScale) -> [f32; 3] {
    let k = fs.mg_per_lsb() * 1e-3;
    [r.x as f32 * k, r.y as f32 * k, r.z as f32 * k]
}

fn gyro_dps(r: RawAxes, fs: GyroFullScale) -> [f32; 3] {
    let k = fs.mdps_per_lsb() * 1e-3;
    [r.x as f32 * k, r.y as f32 * k, r.z as f32 * k]
}

fn ms(a: Instant, b: Instant) -> f32 {
    (a - b).as_micros() as f32 / 1000.0
}

/// Polls both futures to completion on the same task. Neither is ever dropped early.
async fn join2<A: Future, B: Future>(a: A, b: B) -> (A::Output, B::Output) {
    let mut a = pin!(a);
    let mut b = pin!(b);
    let (mut ra, mut rb) = (None, None);
    poll_fn(move |cx| {
        if ra.is_none()
            && let Poll::Ready(v) = a.as_mut().poll(cx)
        {
            ra = Some(v);
        }
        if rb.is_none()
            && let Poll::Ready(v) = b.as_mut().poll(cx)
        {
            rb = Some(v);
        }
        if ra.is_some() && rb.is_some() {
            match (ra.take(), rb.take()) {
                (Some(x), Some(y)) => Poll::Ready((x, y)),
                _ => Poll::Pending,
            }
        } else {
            Poll::Pending
        }
    })
    .await
}

fn new_int1<'a>(pin: &'a mut Peri<'static, PIN_7>, pull: Pull) -> Flex<'a> {
    let mut f = Flex::new(pin.reborrow());
    f.set_as_input();
    f.set_pull(pull);
    f
}

/// Waits for a rising edge of INT1 for at most 50 ms. Only the pin wait is under the timeout.
async fn wait_edge(int1: &mut Flex<'_>) -> bool {
    with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
        .await
        .is_ok()
}

async fn verify_after<S: SpiDevice>(
    fails: &mut u32,
    label: &str,
    imu: &mut Lsm6dsk320x<S>,
) -> bool {
    match imu.verify().await {
        Ok(()) => {
            VERIFIED.fetch_add(1, Ordering::Relaxed);
            true
        }
        Err(e) => {
            check!(
                *fails,
                false,
                "{} verify after [{}]: {}",
                label,
                defmt::Debug2Format(&e)
            );
            false
        }
    }
}

/// `configure` then `verify`. Prints a FAIL line (with the error in full) on either failure.
async fn apply<S: SpiDevice>(
    fails: &mut u32,
    label: &str,
    imu: &mut Lsm6dsk320x<S>,
    cfg: &Config,
) -> Option<Info> {
    let info = match imu.configure(&mut Delay, cfg).await {
        Ok(i) => i,
        Err(e) => {
            check!(
                *fails,
                false,
                "{} configure [{}] failed: {}",
                label,
                defmt::Debug2Format(&e)
            );
            return None;
        }
    };
    if verify_after(fails, label, imu).await {
        Some(info)
    } else {
        None
    }
}

fn drdy_accel() -> IntRouting {
    IntRouting::new().with_data_ready(DataReady::new().with_accel(true))
}

/// Accelerometer only (gyro off), +-4 g, INT1 = accel data-ready.
fn accel_cfg(odr: Odr) -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_accel_odr(odr)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_gyro_odr(Odr::Off)
        .with_int1(drdy_accel())
        .with_int2(IntRouting::new())
}

/// Accelerometer and gyroscope at `odr`, +-4 g and +-500 dps, INT1 = accel data-ready.
fn both_cfg(odr: Odr) -> Config {
    accel_cfg(odr).with_gyro_odr(odr)
}

/// As `both_cfg`, with INT1 = gyroscope data-ready instead.
fn both_gyro_cfg(odr: Odr) -> Config {
    both_cfg(odr).with_int1(IntRouting::new().with_data_ready(DataReady::new().with_gyro(true)))
}

fn off_cfg() -> Config {
    accel_cfg(Odr::Off)
}

// ---------------------------------------------------------------------------------------------
// Checks 1 and 2: noise against filter bandwidth
// ---------------------------------------------------------------------------------------------

struct Acc {
    n: i64,
    s: [i64; 3],
    q: [i64; 3],
}

impl Acc {
    fn new() -> Self {
        Self {
            n: 0,
            s: [0; 3],
            q: [0; 3],
        }
    }

    fn add(&mut self, r: RawAxes) {
        let v = [r.x as i64, r.y as i64, r.z as i64];
        self.n += 1;
        for i in 0..3 {
            self.s[i] += v[i];
            self.q[i] += v[i] * v[i];
        }
    }

    fn std(&self) -> [f32; 3] {
        let n = self.n.max(1);
        let mut out = [0f32; 3];
        for i in 0..3 {
            let num = n * self.q[i] - self.s[i] * self.s[i];
            out[i] = sqrt(num.max(0) as f32 / (n * n) as f32);
        }
        out
    }
}

struct Noise {
    std: [f32; 3],
    n: i64,
    errors: u32,
    timeouts: u32,
}

/// One INT1 edge, one `read()`; the first `discard` samples are dropped, the next `n` are used.
async fn noise<S: SpiDevice>(
    imu: &mut Lsm6dsk320x<S>,
    int1: &mut Flex<'_>,
    gyro: bool,
    discard: u32,
    n: u32,
) -> Noise {
    let mut acc = Acc::new();
    let (mut errors, mut timeouts) = (0u32, 0u32);
    let mut got = 0u32;
    let end = Instant::now() + Duration::from_secs(8);
    for _ in 0..(discard + n + 200) {
        if got >= discard + n || Instant::now() > end {
            break;
        }
        if !wait_edge(int1).await {
            timeouts += 1;
            continue;
        }
        match imu.read().await {
            Ok(s) => {
                got += 1;
                if got > discard {
                    acc.add(if gyro { s.gyro } else { s.accel });
                }
            }
            Err(_) => errors += 1,
        }
    }
    Noise {
        std: acc.std(),
        n: acc.n,
        errors,
        timeouts,
    }
}

/// Four settings: widest, middle, narrowest, and a baseline (statistics only).
async fn noise_series<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
    name: &str,
    gyro: bool,
    lsb_unit: f32,
    discard: u32,
    items: [(&str, Config, f32); 4],
) {
    let mut int1 = new_int1(pin7, Pull::None);
    let mut std = [[0f32; 3]; 4];
    let mut ok = true;
    for (i, (label, cfg, bw_hz)) in items.iter().enumerate() {
        if apply(fails, label, imu, cfg).await.is_none() {
            ok = false;
            continue;
        }
        let r = noise(imu, &mut int1, gyro, discard, 2000).await;
        defmt::info!(
            "[STAT] {} {} (bw {} Hz): std {} LSB = {} (n {}, bus errors {}, edge timeouts {})",
            name,
            label,
            bw_hz,
            r.std,
            r.std.map(|v| v * lsb_unit),
            r.n,
            r.errors,
            r.timeouts
        );
        ok &= r.n >= 2000 && r.errors == 0;
        std[i] = r.std;
    }
    let ratio = [0, 1, 2].map(|a| std[0][a] / std[2][a].max(1e-6));
    let expected = sqrt(items[0].2 / items[2].2);
    defmt::info!(
        "[STAT] {} widest/narrowest std ratio {} per axis; sqrt(bandwidth ratio) = {}; widest/middle {}",
        name,
        ratio,
        expected,
        [0, 1, 2].map(|a| std[0][a] / std[1][a].max(1e-6))
    );
    check!(
        *fails,
        ok && (0..3).all(|a| std[0][a] > std[2][a]),
        "{} {} noise falls from widest to narrowest on every axis",
        name
    );
}

async fn check1<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
) {
    // Table 66 (p70) at 960 Hz: code 011 = 310 Hz (widest), 100 = 100 Hz, 111 = 14.4 Hz (narrowest).
    let cfg = |bw: Option<GyroLpf1Bandwidth>| {
        Config::new()
            .with_gyro_fs(GyroFullScale::Dps250)
            .with_gyro_odr(Odr::Hz960)
            .with_accel_odr(Odr::Off)
            .with_gyro_lpf1(bw)
            .with_int1(IntRouting::new().with_data_ready(DataReady::new().with_gyro(true)))
            .with_int2(IntRouting::new())
    };
    let hz = |b: GyroLpf1Bandwidth| b.cutoff_dhz(Odr::Hz960).unwrap_or(0) as f32 / 10.0;
    noise_series(
        fails,
        imu,
        pin7,
        "gyro LPF1",
        true,
        GyroFullScale::Dps250.mdps_per_lsb() * 1e-3,
        200,
        [
            (
                "Bw3 widest",
                cfg(Some(GyroLpf1Bandwidth::Bw3)),
                hz(GyroLpf1Bandwidth::Bw3),
            ),
            (
                "Bw4",
                cfg(Some(GyroLpf1Bandwidth::Bw4)),
                hz(GyroLpf1Bandwidth::Bw4),
            ),
            (
                "Bw7 narrowest",
                cfg(Some(GyroLpf1Bandwidth::Bw7)),
                hz(GyroLpf1Bandwidth::Bw7),
            ),
            ("LPF1 off", cfg(None), 0.0),
        ],
    )
    .await;
}

async fn check2<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
) {
    // Table 71 (p71): ODR/4 widest, ODR/800 narrowest; LPF2 off is ODR/2 (high-performance mode).
    let cfg = |bw: Option<AccelLpf2Bandwidth>| {
        Config::new()
            .with_accel_fs(AccelFullScale::G2)
            .with_accel_odr(Odr::Hz960)
            .with_gyro_odr(Odr::Off)
            .with_accel_lpf2(bw)
            .with_int1(drdy_accel())
            .with_int2(IntRouting::new())
    };
    noise_series(
        fails,
        imu,
        pin7,
        "accel LPF2",
        false,
        AccelFullScale::G2.mg_per_lsb(),
        1000,
        [
            (
                "ODR/4 widest",
                cfg(Some(AccelLpf2Bandwidth::OdrDiv4)),
                960.0 / 4.0,
            ),
            (
                "ODR/20",
                cfg(Some(AccelLpf2Bandwidth::OdrDiv20)),
                960.0 / 20.0,
            ),
            (
                "ODR/800 narrowest",
                cfg(Some(AccelLpf2Bandwidth::OdrDiv800)),
                960.0 / 800.0,
            ),
            ("LPF2 off (ODR/2)", cfg(None), 480.0),
        ],
    )
    .await;
}

// ---------------------------------------------------------------------------------------------
// Check 3: settling
// ---------------------------------------------------------------------------------------------

struct Probe {
    conf_ms: f32,
    edge_ms: Option<f32>,
    first_a: [f32; 5],
    first_g: [[f32; 3]; 5],
    n_first: usize,
    in_band_ms: Option<f32>,
    in_band_idx: u32,
    errors: u32,
    later_a: f32,
    later_g: [f32; 3],
}

/// Configures the sensors off, then times a `configure` to `cfg` from power-down and reads the
/// first samples. The edge is awaited concurrently with the `configure` (join, nothing dropped).
async fn start_probe<S: SpiDevice>(
    fails: &mut u32,
    label: &str,
    imu: &mut Lsm6dsk320x<S>,
    int1: &mut Flex<'_>,
    cfg: &Config,
) -> Option<Probe> {
    apply(fails, label, imu, &off_cfg()).await?;
    Timer::after_millis(30).await;
    let t0 = Instant::now();
    let edge_fut = async {
        with_timeout(Duration::from_millis(500), int1.wait_for_rising_edge())
            .await
            .ok()
            .map(|_| Instant::now())
    };
    let conf_fut = async {
        let r = imu.configure(&mut Delay, cfg).await;
        (r, Instant::now())
    };
    let ((res, conf_end), edge) = join2(conf_fut, edge_fut).await;
    let conf_ms = ms(conf_end, t0);
    if let Err(e) = res {
        check!(
            *fails,
            false,
            "{} configure [{}] failed: {}",
            label,
            defmt::Debug2Format(&e)
        );
        return None;
    }
    let (afs, gfs) = (cfg.accel_fs, cfg.gyro_fs);
    let mut p = Probe {
        conf_ms,
        edge_ms: edge.map(|e| ms(e, t0)),
        first_a: [0.0; 5],
        first_g: [[0.0; 3]; 5],
        n_first: 0,
        in_band_ms: None,
        in_band_idx: 0,
        errors: 0,
        later_a: 0.0,
        later_g: [0.0; 3],
    };
    let mut pre = edge.is_some();
    let mut i = 0u32;
    while i < 700 && t0.elapsed() < Duration::from_millis(1500) {
        i += 1;
        if pre {
            pre = false;
        } else if !wait_edge(int1).await {
            continue;
        }
        let Ok(s) = imu.read().await else {
            p.errors += 1;
            continue;
        };
        let t = Instant::now();
        let a = mag(accel_vec_g(s.accel, afs));
        if p.n_first < 5 {
            p.first_a[p.n_first] = a;
            p.first_g[p.n_first] = gyro_dps(s.gyro, gfs);
            p.n_first += 1;
        }
        if p.in_band_ms.is_none() && in_band(a) {
            p.in_band_ms = Some(ms(t, t0));
            p.in_band_idx = i;
        }
        if p.n_first >= 5 && p.in_band_ms.is_some() {
            break;
        }
    }
    let (mut sa, mut sg, mut n) = ([0f32; 3], [0f32; 3], 0u32);
    for _ in 0..140 {
        if n >= 100 {
            break;
        }
        if !wait_edge(int1).await {
            continue;
        }
        if let Ok(s) = imu.read().await {
            let (a, g) = (accel_vec_g(s.accel, afs), gyro_dps(s.gyro, gfs));
            for k in 0..3 {
                sa[k] += a[k];
                sg[k] += g[k];
            }
            n += 1;
        }
    }
    let nf = n.max(1) as f32;
    p.later_a = mag(sa.map(|v| v / nf));
    p.later_g = sg.map(|v| v / nf);
    if !verify_after(fails, label, imu).await {
        return None;
    }
    Some(p)
}

fn report_probe(label: &str, p: &Probe) {
    let dev = p.first_g.map(|g| [0, 1, 2].map(|k| g[k] - p.later_g[k]));
    let out_a = p.first_a[..p.n_first]
        .iter()
        .filter(|a| !in_band(**a))
        .count();
    let out_g = dev[..p.n_first]
        .iter()
        .filter(|d| d.iter().any(|v| abs(*v) > 3.0))
        .count();
    defmt::info!(
        "[STAT] {}: configure {} ms, first edge {} ms, first in-band at {} ms (read #{}), bus errors {}",
        label,
        p.conf_ms,
        p.edge_ms,
        p.in_band_ms,
        p.in_band_idx,
        p.errors
    );
    defmt::info!(
        "[STAT] {}: first 5 |a| {} g, gyro dev from later mean {} dps; out of band: accel {}, gyro {}; later |a| {} g, later gyro {} dps",
        label,
        p.first_a,
        dev,
        out_a,
        out_g,
        p.later_a,
        p.later_g
    );
}

async fn check3<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
) {
    let mut int1 = new_int1(pin7, Pull::None);
    // Mask on: the first 5 samples must already be valid. Both sensors, then the accelerometer alone.
    for (label, cfg, gyro_on) in [
        ("mask ON  accel+gyro 240Hz", both_cfg(Odr::Hz240), true),
        ("mask ON  accel only 240Hz", accel_cfg(Odr::Hz240), false),
        (
            "mask ON  accel+gyro 240Hz INT1=gyro",
            both_gyro_cfg(Odr::Hz240),
            true,
        ),
    ] {
        let Some(p) = start_probe(fails, label, imu, &mut int1, &cfg).await else {
            continue;
        };
        report_probe(label, &p);
        let a_ok = p.n_first == 5 && p.first_a.iter().all(|a| in_band(*a));
        let g_ok = !gyro_on
            || p.first_g
                .iter()
                .all(|g| (0..3).all(|k| abs(g[k] - p.later_g[k]) <= 3.0));
        check!(
            *fails,
            a_ok && g_ok,
            "{} {}: first 5 samples within the at-rest band (accel 1 g +-10 %: {}; gyro within 3 dps of later mean: {})",
            label,
            a_ok,
            g_ok
        );
    }
    // Mask off: what the mask hides (statistics only).
    for (label, cfg) in [
        (
            "mask OFF accel+gyro 240Hz",
            both_cfg(Odr::Hz240).with_drdy_mask(false),
        ),
        (
            "mask OFF accel+gyro 240Hz INT1=gyro",
            both_gyro_cfg(Odr::Hz240).with_drdy_mask(false),
        ),
        (
            "mask OFF accel only 240Hz",
            accel_cfg(Odr::Hz240).with_drdy_mask(false),
        ),
    ] {
        if let Some(p) = start_probe(fails, label, imu, &mut int1, &cfg).await {
            report_probe(label, &p);
        }
    }
    // Fast settling, narrow LPF2 (ODR/100 at 240 Hz), mask on and off (statistics; assert health).
    for (label, mask, fast) in [
        ("LPF2 /100 mask ON  fast OFF", true, false),
        ("LPF2 /100 mask ON  fast ON ", true, true),
        ("LPF2 /100 mask OFF fast OFF", false, false),
        ("LPF2 /100 mask OFF fast ON ", false, true),
    ] {
        let cfg = accel_cfg(Odr::Hz240)
            .with_accel_lpf2(Some(AccelLpf2Bandwidth::OdrDiv100))
            .with_drdy_mask(mask)
            .with_accel_fast_settling(fast);
        let Some(p) = start_probe(fails, label, imu, &mut int1, &cfg).await else {
            continue;
        };
        report_probe(label, &p);
        check!(
            *fails,
            in_band(p.later_a) && p.errors == 0,
            "{} {}: verify Ok, plausible data afterwards (|a| {} g)",
            label,
            p.later_a
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Check 4: high-accuracy ODR
// ---------------------------------------------------------------------------------------------

/// Measured INT1 rate in Hz (rising edges counted by PWM slice 3, 3 s window) and the `Info`.
async fn measure_rate<S: SpiDevice>(
    fails: &mut u32,
    label: &str,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
    slice3: &mut Peri<'static, PWM_SLICE3>,
    cfg: &Config,
) -> Option<(f32, u32, Info)> {
    let info = apply(fails, label, imu, cfg).await?;
    let counter = Pwm::new_input(
        slice3.reborrow(),
        pin7.reborrow(),
        Pull::None,
        InputMode::RisingEdge,
        pwm::Config::default(),
    );
    Timer::after_millis(150).await;
    let c0 = counter.counter();
    let t0 = Instant::now();
    Timer::after_millis(3000).await;
    let c1 = counter.counter();
    let dt = t0.elapsed().as_micros();
    let edges = c1.wrapping_sub(c0) as u32;
    Some((edges as f32 * 1e6 / dt as f32, edges, info))
}

fn ppm(measured: f32, nominal: f32) -> i32 {
    ((measured / nominal - 1.0) * 1e6) as i32
}

async fn check4<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
    slice3: &mut Peri<'static, PWM_SLICE3>,
) {
    let base = both_cfg(Odr::Hz960);
    let mut normal = 0f32;
    if let Some((rate, edges, info)) =
        measure_rate(fails, "normal 960", imu, pin7, slice3, &base).await
    {
        normal = rate;
        defmt::info!(
            "[STAT] normal 960 Hz: measured {} Hz ({} edges), nominal 960 Hz = {} ppm; info() real rate {} mHz = {} ppm from measured (freq_fine {})",
            rate,
            edges,
            ppm(rate, 960.0),
            info.accel_odr_mhz,
            ppm(info.accel_odr_mhz as f32 / 1000.0, rate),
            info.freq_fine
        );
    }
    for (set, name) in [
        (HaodrSet::Base960, "Base960"),
        (HaodrSet::Base1000, "Base1000"),
        (HaodrSet::Base800, "Base800"),
        (HaodrSet::Base833, "Base833"),
    ] {
        let cfg = base.with_mode(OperatingMode::HighAccuracyOdr(set));
        let Some(nominal_mhz) = Odr::Hz960.haodr_mhz(set) else {
            continue;
        };
        let nominal = nominal_mhz as f32 / 1000.0;
        let Some((rate, edges, info)) = measure_rate(fails, name, imu, pin7, slice3, &cfg).await
        else {
            continue;
        };
        let err = ppm(rate, nominal);
        defmt::info!(
            "[STAT] HAODR {} (code 960): measured {} Hz ({} edges), nominal {} Hz, error {} ppm; info() accel {} mHz gyro {} mHz ({} ppm from measured); vs normal 960 Hz: ratio {} (nominal ratio {})",
            name,
            rate,
            edges,
            nominal,
            err,
            info.accel_odr_mhz,
            info.gyro_odr_mhz,
            ppm(info.accel_odr_mhz as f32 / 1000.0, rate),
            rate / normal.max(1.0),
            nominal / 960.0
        );
        check!(
            *fails,
            edges > 0 && err.abs() <= 15_000,
            "{} HAODR {}: {} Hz vs nominal {} Hz, {} ppm (limit 15000)",
            name,
            rate,
            nominal,
            err
        );
        let info_a = info.accel_odr_mhz as f32 / 1000.0;
        let info_g = info.gyro_odr_mhz as f32 / 1000.0;
        check!(
            *fails,
            abs(info_a - rate) <= 0.015 * rate && abs(info_g - rate) <= 0.015 * rate,
            "{} HAODR {}: info() accel {} Hz and gyro {} Hz within 1.5 % of the measured {} Hz",
            name,
            info_a,
            info_g,
            rate
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Check 5: latched data-ready
// ---------------------------------------------------------------------------------------------

async fn check5<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
) {
    let mut int1 = new_int1(pin7, Pull::None);
    let cfg = accel_cfg(Odr::Hz60).with_data_ready_mode(DataReadyMode::Latched);
    if apply(fails, "latched", imu, &cfg).await.is_some() {
        let got = with_timeout(Duration::from_millis(100), int1.wait_for_high())
            .await
            .is_ok();
        let mut lv = [false; 3];
        for l in lv.iter_mut() {
            *l = int1.is_high();
            Timer::after_millis(20).await;
        }
        let read_ok = imu.read().await.is_ok();
        let low_after = int1.is_low();
        let t = Instant::now();
        let again = with_timeout(Duration::from_millis(100), int1.wait_for_high())
            .await
            .is_ok();
        let dt = ms(Instant::now(), t);
        defmt::info!(
            "[STAT] latched 60 Hz: asserted at start {}, levels over 40 ms without reading {}, after read low {}, asserted again {} after {} ms",
            got,
            lv,
            low_after,
            again,
            dt
        );
        check!(
            *fails,
            got && lv == [true; 3] && read_ok && low_after && again && dt < 25.0,
            "{} latched: line stays asserted until read, deasserts on read, asserts again at next sample"
        );
    }
    let cfg = accel_cfg(Odr::Hz60);
    if apply(fails, "pulsed", imu, &cfg).await.is_some() {
        Timer::after_millis(30).await;
        let mut lv = [true; 3];
        for l in lv.iter_mut() {
            *l = int1.is_high();
            Timer::after_millis(20).await;
        }
        let mut highs = 0u32;
        for _ in 0..200 {
            highs += int1.is_high() as u32;
            Timer::after_millis(1).await;
        }
        let (mut wsum, mut wmin, mut wn) = (0u32, u32::MAX, 0u32);
        for _ in 0..10 {
            if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
                .await
                .is_ok()
            {
                let t = Instant::now();
                while int1.is_high() && t.elapsed() < Duration::from_millis(2) {}
                let w = t.elapsed().as_micros() as u32;
                wsum += w;
                wmin = wmin.min(w);
                wn += 1;
            }
        }
        defmt::info!(
            "[STAT] pulsed 60 Hz: levels {}, high in {} of 200 samples; pulse width seen after edge wake-up (lower bound): min {} us, mean {} us over {} pulses",
            lv,
            highs,
            wmin,
            wsum / wn.max(1),
            wn
        );
        check!(
            *fails,
            lv == [false; 3] && highs <= 4 && wn > 0,
            "{} pulsed: line deasserted between samples ({} of 200 samples high)",
            highs
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Check 6: pin configuration
// ---------------------------------------------------------------------------------------------

async fn polarity_case<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    int1: &mut Flex<'_>,
    pol: IntPolarity,
    name: &str,
) {
    let pins = PinConfig::new().with_polarity(pol);
    let active_high = pol == IntPolarity::ActiveHigh;
    if apply(fails, name, imu, &off_cfg().with_pins(pins))
        .await
        .is_none()
    {
        return;
    }
    Timer::after_millis(20).await;
    let mut idle_high = 0u32;
    for _ in 0..20 {
        idle_high += int1.is_high() as u32;
        Timer::after_millis(1).await;
    }
    let cfg = accel_cfg(Odr::Hz60)
        .with_data_ready_mode(DataReadyMode::Latched)
        .with_pins(pins);
    if apply(fails, name, imu, &cfg).await.is_none() {
        return;
    }
    let asserted = with_timeout(Duration::from_millis(100), async {
        if active_high {
            int1.wait_for_high().await
        } else {
            int1.wait_for_low().await
        }
    })
    .await
    .is_ok();
    let level_asserted = int1.is_high() == active_high;
    let _ = imu.read().await;
    let cleared = int1.is_high() != active_high;
    let edge = with_timeout(Duration::from_millis(100), async {
        if active_high {
            int1.wait_for_rising_edge().await
        } else {
            int1.wait_for_falling_edge().await
        }
    })
    .await
    .is_ok();
    let level_after_edge = int1.is_high() == active_high;
    defmt::info!(
        "[STAT] {}: sensors off, line high in {} of 20 samples; data-ready asserted {}, level high={}; after read still asserted={}; {} edge on next sample {}, level after it high={}",
        name,
        idle_high,
        asserted,
        int1.is_high(),
        !cleared,
        if active_high { "rising" } else { "falling" },
        edge,
        level_after_edge == active_high
    );
    let idle_ok = if active_high {
        idle_high == 0
    } else {
        idle_high == 20
    };
    check!(
        *fails,
        idle_ok && asserted && level_asserted && cleared && edge && level_after_edge,
        "{} {}: idle level {}, asserted level {}, data-ready edge {}",
        name,
        if active_high { "low" } else { "high" },
        if active_high { "high" } else { "low" },
        if active_high { "rising" } else { "falling" }
    );
}

async fn output_case<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    int1: &mut Flex<'_>,
    pol: IntPolarity,
    out: IntOutput,
    name: &str,
) {
    let pins = PinConfig::new().with_polarity(pol).with_output(out);
    if apply(fails, name, imu, &off_cfg().with_pins(pins))
        .await
        .is_none()
    {
        return;
    }
    Timer::after_millis(20).await;
    int1.set_pull(Pull::Up);
    Timer::after_millis(2).await;
    let up = int1.is_high();
    int1.set_pull(Pull::Down);
    Timer::after_millis(2).await;
    let down = int1.is_high();
    int1.set_pull(Pull::None);
    let idle_high = pol == IntPolarity::ActiveLow;
    defmt::info!(
        "[STAT] {} line inactive: pin high with pull-up {}, with pull-down {}",
        name,
        up,
        down
    );
    match (out, pol) {
        (IntOutput::OpenDrain, IntPolarity::ActiveLow) => check!(
            *fails,
            up && !down,
            "{} {}: released line follows the MCU pull (up {}, down {})",
            name,
            up,
            down
        ),
        (IntOutput::PushPull, _) => check!(
            *fails,
            up == idle_high && down == idle_high,
            "{} {}: driven idle level {} does not follow the pull (up {}, down {})",
            name,
            idle_high,
            up,
            down
        ),
        _ => {}
    }
}

async fn check6<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
) {
    {
        let mut int1 = new_int1(pin7, Pull::None);
        polarity_case(
            fails,
            imu,
            &mut int1,
            IntPolarity::ActiveHigh,
            "active-high",
        )
        .await;
        polarity_case(fails, imu, &mut int1, IntPolarity::ActiveLow, "active-low").await;
        for (pol, out, name) in [
            (
                IntPolarity::ActiveLow,
                IntOutput::OpenDrain,
                "open-drain active-low",
            ),
            (
                IntPolarity::ActiveHigh,
                IntOutput::OpenDrain,
                "open-drain active-high",
            ),
            (
                IntPolarity::ActiveHigh,
                IntOutput::PushPull,
                "push-pull active-high",
            ),
            (
                IntPolarity::ActiveLow,
                IntOutput::PushPull,
                "push-pull active-low",
            ),
        ] {
            output_case(fails, imu, &mut int1, pol, out, name).await;
        }
        int1.set_pull(Pull::None);
    }
    // Pad drive strength: the bus must stay clean.
    let afs = AccelFullScale::G4;
    for (drive, name) in [
        (PadDrive::Lowest, "drive lowest"),
        (PadDrive::Intermediate, "drive intermediate"),
        (PadDrive::Highest, "drive highest"),
    ] {
        let cfg = both_cfg(Odr::Hz960).with_pins(PinConfig::new().with_drive(drive));
        if apply(fails, name, imu, &cfg).await.is_none() {
            continue;
        }
        Timer::after_millis(20).await;
        let (mut wrong, mut id_wrong) = (0u32, 0u32);
        for i in 0..2000u32 {
            match imu.read().await {
                Ok(s) => {
                    if !in_band(mag(accel_vec_g(s.accel, afs))) {
                        wrong += 1;
                    }
                }
                Err(_) => wrong += 1,
            }
            if i % 100 == 99
                && imu.read_register(reg::WHO_AM_I).await.ok() != Some(reg::WHO_AM_I_VALUE)
            {
                id_wrong += 1;
            }
        }
        let v = verify_after(fails, name, imu).await;
        check!(
            *fails,
            v && wrong == 0 && id_wrong == 0,
            "{} {}: verify Ok, 2000 reads, {} wrong, {} identity mismatches in 20 checks",
            name,
            wrong,
            id_wrong
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Check 7: status flags
// ---------------------------------------------------------------------------------------------

async fn flags_case<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    name: &str,
    cfg: Config,
    accel_on: bool,
) {
    if apply(fails, name, imu, &cfg).await.is_none() {
        return;
    }
    // A flag left unread by an earlier configuration survives `configure` (see the documentation
    // of `Lsm6dsk320x::status`): one discard read first.
    let _ = imu.read().await;
    let (mut sa, mut sg, mut sh, mut errors) = (0u32, 0u32, 0u32, 0u32);
    for _ in 0..150 {
        match imu.status().await {
            Ok(s) => {
                sa += s.accel as u32;
                sg += s.gyro as u32;
                sh += s.high_g as u32;
            }
            Err(_) => errors += 1,
        }
        Timer::after_micros(700).await;
    }
    let (on, off) = if accel_on { (sa, sg) } else { (sg, sa) };
    check!(
        *fails,
        on > 0 && off == 0 && sh == 0 && errors == 0,
        "{} {}: status() polls (150) with accel flag {}, gyro flag {}, high-g flag {} (errors {})",
        name,
        sa,
        sg,
        sh,
        errors
    );
    if off > 0 {
        let _ = imu.read().await;
        let mut off_after = 0u32;
        for _ in 0..150 {
            if let Ok(s) = imu.status().await {
                off_after += (if accel_on { s.gyro } else { s.accel }) as u32;
            }
            Timer::after_micros(700).await;
        }
        defmt::info!(
            "[STAT] {}: flag of the sensor that is off: set in {} of 150 polls before any read(), {} of 150 after one read()",
            name,
            off,
            off_after
        );
    }
    let flag = |s: &lsm6dsk320x::Status| if accel_on { s.accel } else { s.gyro };
    let mut seen = false;
    for _ in 0..150 {
        if let Ok(s) = imu.status().await
            && flag(&s)
        {
            seen = true;
            break;
        }
        Timer::after_micros(700).await;
    }
    let still = imu.status().await.map(|s| flag(&s)).ok();
    let read_ok = imu.read().await.is_ok();
    let after = imu.status().await.map(|s| flag(&s)).ok();
    let mut again = false;
    for _ in 0..150 {
        if let Ok(s) = imu.status().await
            && flag(&s)
        {
            again = true;
            break;
        }
        Timer::after_micros(700).await;
    }
    defmt::info!(
        "[STAT] {}: flag seen {}, still set on a second status() {}, set after read() {}, set again {}",
        name,
        seen,
        still,
        after,
        again
    );
    check!(
        *fails,
        seen && read_ok && after == Some(false) && again,
        "{} {}: flag clears when its data is read and sets again",
        name
    );
}

/// Finding D: the flag of a sample left unread survives `configure`, also when that sensor is
/// then off; one read of the data clears it for good.
async fn unread_flag_case<S: SpiDevice>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>) {
    let name = "unread flag across configure";
    if apply(fails, name, imu, &both_cfg(Odr::Hz60))
        .await
        .is_none()
    {
        return;
    }
    Timer::after_millis(40).await;
    let gyro_only = both_cfg(Odr::Hz60).with_accel_odr(Odr::Off);
    if apply(fails, name, imu, &gyro_only).await.is_none() {
        return;
    }
    let (mut before, mut after, mut errors) = (0u32, 0u32, 0u32);
    for _ in 0..150 {
        match imu.status().await {
            Ok(s) => before += s.accel as u32,
            Err(_) => errors += 1,
        }
        Timer::after_micros(700).await;
    }
    let read_ok = imu.read().await.is_ok();
    for _ in 0..150 {
        match imu.status().await {
            Ok(s) => after += s.accel as u32,
            Err(_) => errors += 1,
        }
        Timer::after_micros(700).await;
    }
    check!(
        *fails,
        before > 0 && read_ok && after == 0 && errors == 0,
        "{} {}: accel flag of the sample left unread set in {} of 150 polls after configure left accel off, {} of 150 after one read() (read ok {}, errors {})",
        name,
        before,
        after,
        read_ok,
        errors
    );
}

async fn check7<S: SpiDevice>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>) {
    flags_case(fails, imu, "accel on, gyro off", accel_cfg(Odr::Hz60), true).await;
    let gyro_only = both_cfg(Odr::Hz60).with_accel_odr(Odr::Off);
    flags_case(fails, imu, "gyro on, accel off", gyro_only, false).await;
    unread_flag_case(fails, imu).await;

    for (name, cfg, drdy_ok) in [
        ("interrupt_status sensors off", off_cfg(), false),
        (
            "interrupt_status accel+gyro 960 Hz",
            both_cfg(Odr::Hz960),
            true,
        ),
    ] {
        if apply(fails, name, imu, &cfg).await.is_none() {
            continue;
        }
        // Discard read: a flag left unread by an earlier configuration survives `configure`.
        let _ = imu.read().await;
        Timer::after_millis(5).await;
        match imu.interrupt_status().await {
            Ok(st) => {
                defmt::info!("[STAT] {}: {}", name, defmt::Debug2Format(&st));
                let events = !st.timestamp_overflow
                    && !st.high_g_wake_up
                    && !st.high_g_shock
                    && !st.high_g_data_ready
                    && !st.fifo.watermark
                    && !st.fifo.full
                    && !st.fifo.overrun
                    && !st.fifo.overrun_latched;
                let drdy = st.accel_data_ready || st.gyro_data_ready;
                if !drdy_ok && drdy {
                    let _ = imu.read().await;
                    let again = imu.interrupt_status().await.ok();
                    defmt::info!(
                        "[STAT] {}: after one read(): {}",
                        name,
                        defmt::Debug2Format(&again)
                    );
                }
                check!(
                    *fails,
                    events && (drdy_ok || !drdy),
                    "{} {}: Ok, no event or FIFO flag set (data-ready flags {} {})",
                    name,
                    st.accel_data_ready,
                    st.gyro_data_ready
                );
            }
            Err(e) => check!(
                *fails,
                false,
                "{} {}: interrupt_status failed: {}",
                name,
                defmt::Debug2Format(&e)
            ),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Final health check
// ---------------------------------------------------------------------------------------------

async fn final_health<S: SpiDevice>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    pin7: &mut Peri<'static, PIN_7>,
) {
    let mut int1 = new_int1(pin7, Pull::None);
    let cfg = Config::new();
    if let Err(e) = imu.init(&mut Delay, &cfg).await {
        check!(*fails, false, "{} final init: {}", defmt::Debug2Format(&e));
        return;
    }
    check!(*fails, true, "{} final init to the default configuration");
    let v = verify_after(fails, "final", imu).await;
    check!(*fails, v, "{} final verify() Ok");
    let id = imu.read_register(reg::WHO_AM_I).await.ok();
    check!(
        *fails,
        id == Some(reg::WHO_AM_I_VALUE),
        "{} final WHO_AM_I {}",
        id
    );
    let (mut good, mut bad, mut errs) = (0u32, 0u32, 0u32);
    let mut a_sum = 0f32;
    for _ in 0..400 {
        if good + bad >= 200 {
            break;
        }
        if !wait_edge(&mut int1).await {
            errs += 1;
            continue;
        }
        match imu.read().await {
            Ok(s) => {
                let a = mag(accel_vec_g(s.accel, cfg.accel_fs));
                a_sum += a;
                if in_band(a) {
                    good += 1;
                } else {
                    bad += 1;
                }
            }
            Err(_) => errs += 1,
        }
    }
    defmt::info!(
        "[STAT] final: {} plausible, {} implausible samples, {} errors/timeouts, mean |a| {} g",
        good,
        bad,
        errs,
        a_sum / (good + bad).max(1) as f32
    );
    check!(
        *fails,
        good == 200 && bad == 0,
        "{} final 200 plausible samples (|a| 1 g +-10 %)"
    );
    let _ = G;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let start = Instant::now();
    let mut fails = 0u32;

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = SPI_HZ;
    let bus = Spi::new(
        p.SPI1, p.PIN_10, p.PIN_11, p.PIN_8, p.DMA_CH0, p.DMA_CH1, Irqs, spi_cfg,
    );
    let cs = Output::new(p.PIN_9, Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    let mut imu = Lsm6dsk320x::new(dev);
    let mut pin7 = p.PIN_7;
    let mut slice3 = p.PWM_SLICE3;

    let first = imu.init(&mut Delay, &Config::new()).await;
    match first {
        Ok(_) => {
            macro_rules! run {
                ($n:literal, $call:expr) => {
                    if start.elapsed().as_secs() >= RUN_LIMIT_S {
                        fails += 1;
                        defmt::info!("[FAIL] check {} not run: time limit", $n);
                    } else {
                        $call.await;
                    }
                };
            }
            run!(1, check1(&mut fails, &mut imu, &mut pin7));
            run!(2, check2(&mut fails, &mut imu, &mut pin7));
            run!(3, check3(&mut fails, &mut imu, &mut pin7));
            run!(4, check4(&mut fails, &mut imu, &mut pin7, &mut slice3));
            run!(5, check5(&mut fails, &mut imu, &mut pin7));
            run!(6, check6(&mut fails, &mut imu, &mut pin7));
            run!(7, check7(&mut fails, &mut imu));
        }
        Err(e) => check!(
            fails,
            false,
            "{} first init failed: {}",
            defmt::Debug2Format(&e)
        ),
    }
    final_health(&mut fails, &mut imu, &mut pin7).await;
    defmt::info!(
        "[INFO] {} configure+verify pairs Ok; run time {} ms",
        VERIFIED.load(Ordering::Relaxed),
        start.elapsed().as_millis()
    );
    hil_common::exit(fails)
}

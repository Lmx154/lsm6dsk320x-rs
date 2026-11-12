//! On-board check of the lsm6dsk320x driver's two functions run TOGETHER (U23 on SPI1, INT1 on
//! GPIO7, INT2 on GPIO6): the low-g accelerometer, the gyroscope and the high-g accelerometer are
//! all on in every section.
//!
//! D1 every high-g full scale at rest, D2 every high-g output rate against INT1, D3 the
//! high-accuracy ODR mode with the high-g channel on (rates and FIFO, every high-g rate), D4 the timing of `read_all`,
//! then the default configuration. The board lies still on the bench. No SPI transfer is ever
//! cancelled: the only timeouts wrap the wait for an INT1 edge. INT1 edges are counted by PWM slice
//! 3 in hardware, INT2 edges (GPIO6 is the A pin of that slice) by polling the pin.
#![no_std]
#![no_main]

use core::fmt::Debug;

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peripherals, bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::{
    AccelFullScale, BatchRate, Config, DataReady, FifoConfig, FifoFrame, FifoMode, GyroFullScale,
    HaodrSet, HighGConfig, HighGFullScale, HighGOdr, Info, IntRouting, Lsm6dsk320x, Odr,
    OperatingMode, fifo_frames,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>, dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const ACCEL_FS: AccelFullScale = AccelFullScale::G4;
const GYRO_FS: GyroFullScale = GyroFullScale::Dps500;
const G: f32 = lsm6dsk320x::sample::G;
const DEG: f32 = 180.0 / core::f32::consts::PI;
/// Sanity bound of the high-g minus low-g difference per axis, in g. The datasheet gives only a
/// TYPICAL high-g zero-g offset, +-250 mg (Table 3, p12), not a guaranteed limit; this chip
/// measures 0.05 / -0.15 / 0.30 g on X / Y / Z at every full scale (log
/// `hil/logs/lsm6dsk320x_dual-20261111-183224.log`). The application calibrates it out with the
/// high-g offset registers; the check only catches a wrong scale or axis.
const OFFSET_LIMIT_G: f32 = 0.5;
/// Tolerance of the same physical value across high-g full scales, per axis, in g.
const SCALE_TOL_G: f32 = 0.10;
/// Tolerance of the low-g vector across high-g settings, per axis, in g.
const LOWG_TOL_G: f32 = 0.03;

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

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn max_abs(v: [f32; 3]) -> f32 {
    let mut m = 0.0;
    for x in v {
        if abs(x) > m {
            m = abs(x);
        }
    }
    m
}

fn cyc() -> u32 {
    DWT::cycle_count()
}

fn dwt_on() {
    // SAFETY: only the trace enable and the cycle counter of the calling core are touched.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}

fn clk_hz() -> f32 {
    embassy_rp::clocks::clk_sys_freq() as f32
}

fn bus_cfg(hz: u32, mode3: bool) -> spi::Config {
    let mut c = spi::Config::default();
    c.frequency = hz;
    if mode3 {
        c.phase = spi::Phase::CaptureOnSecondTransition;
        c.polarity = spi::Polarity::IdleHigh;
    }
    c
}

/// Low-g and gyro at `low`, high-g on, all three channels in one configuration. INT2 carries the
/// high-g data-ready, INT1 the low-g one (or the gyro one with `gyro_on_int1`).
fn cfg(
    low: Odr,
    hodr: HighGOdr,
    hfs: HighGFullScale,
    mode: OperatingMode,
    gyro_on_int1: bool,
) -> Config {
    let dr = if gyro_on_int1 {
        DataReady::new().with_gyro(true)
    } else {
        DataReady::new().with_accel(true)
    };
    Config::new()
        .with_accel_fs(ACCEL_FS)
        .with_accel_odr(low)
        .with_gyro_fs(GYRO_FS)
        .with_gyro_odr(low)
        .with_mode(mode)
        .with_high_g(Some(HighGConfig::new().with_odr(hodr).with_full_scale(hfs)))
        .with_int1(IntRouting::new().with_data_ready(dr))
        .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)))
}

/// `configure`, `verify()`, then 150 ms to settle. `None` after a FAIL line.
async fn apply<S>(
    fails: &mut u32,
    label: &str,
    imu: &mut Lsm6dsk320x<S>,
    cfg: &Config,
) -> Option<Info>
where
    S: SpiDevice,
    S::Error: Debug,
{
    let info = match imu.configure(&mut Delay, cfg).await {
        Ok(i) => i,
        Err(e) => {
            check!(
                *fails,
                false,
                "{} {}: configure failed: {}",
                label,
                defmt::Debug2Format(&e)
            );
            return None;
        }
    };
    match imu.verify().await {
        Ok(()) => check!(*fails, true, "{} {}: configure + verify() Ok", label),
        Err(e) => {
            check!(
                *fails,
                false,
                "{} {}: verify() rejected the setting: {}",
                label,
                defmt::Debug2Format(&e)
            );
            return None;
        }
    }
    Timer::after_millis(150).await;
    Some(info)
}

/// Sums of `read_all` samples, one per INT1 edge.
struct Acc {
    n: u32,
    errors: u32,
    timeouts: u32,
    a: [f32; 3],
    g: [f32; 3],
    h: [f32; 3],
    amin: f32,
    amax: f32,
    gpeak: f32,
}

impl Acc {
    fn means(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        let n = self.n.max(1) as f32;
        (
            self.a.map(|v| v / n),
            self.g.map(|v| v / n),
            self.h.map(|v| v / n),
        )
    }

    /// The board moved during the window.
    fn disturbed(&self) -> bool {
        self.gpeak > 20.0 || self.amax - self.amin > 0.15
    }
}

/// `want` samples of `read_all`, each after an INT1 rising edge. Low-g and high-g in g, gyro in dps.
async fn collect<S>(imu: &mut Lsm6dsk320x<S>, int1: &mut Input<'_>, cfg: &Config, want: u32) -> Acc
where
    S: SpiDevice,
{
    let mut acc = Acc {
        n: 0,
        errors: 0,
        timeouts: 0,
        a: [0.0; 3],
        g: [0.0; 3],
        h: [0.0; 3],
        amin: 1e9,
        amax: 0.0,
        gpeak: 0.0,
    };
    let Some(hfs) = cfg.high_g.map(|h| h.full_scale) else {
        return acc;
    };
    let mut tries = 0;
    while acc.n < want && tries < 2 * want {
        tries += 1;
        if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
            .await
            .is_err()
        {
            acc.timeouts += 1;
            continue;
        }
        match imu.read_all().await {
            Ok(all) => {
                let a = all.accel.accel_ms2(cfg.accel_fs).map(|v| v / G);
                let g = all.gyro.gyro_rads(cfg.gyro_fs).map(|v| v * DEG);
                let h = all.high_g.high_g_ms2(hfs).map(|v| v / G);
                for i in 0..3 {
                    acc.a[i] += a[i];
                    acc.g[i] += g[i];
                    acc.h[i] += h[i];
                }
                let m = mag(a);
                acc.amin = if m < acc.amin { m } else { acc.amin };
                acc.amax = if m > acc.amax { m } else { acc.amax };
                let gp = max_abs(g);
                acc.gpeak = if gp > acc.gpeak { gp } else { acc.gpeak };
                acc.n += 1;
            }
            Err(_) => acc.errors += 1,
        }
    }
    acc
}

fn describe(label: &str, acc: &Acc) {
    let (a, g, h) = acc.means();
    defmt::info!(
        "[STAT] {}: {} samples, bus errors {}, edge timeouts {}; low-g {} g |a| {} g; gyro {} dps (peak {}); high-g {} g |h| {} g; low-g |a| spread {} g",
        label,
        acc.n,
        acc.errors,
        acc.timeouts,
        a,
        mag(a),
        g,
        acc.gpeak,
        h,
        mag(h),
        acc.amax - acc.amin
    );
}

/// Plausible data on all three channels, or a note that the board moved.
fn plausible(fails: &mut u32, label: &str, acc: &Acc, want: u32) {
    describe(label, acc);
    if acc.disturbed() {
        defmt::info!(
            "[INFO] {}: window DISTURBED (gyro peak {} dps, low-g |a| spread {} g): plausibility not judged",
            label,
            acc.gpeak,
            acc.amax - acc.amin
        );
        return;
    }
    let (a, g, h) = acc.means();
    check!(
        *fails,
        acc.n >= want * 9 / 10
            && acc.errors == 0
            && (0.9..=1.1).contains(&mag(a))
            && max_abs(g) <= 5.0
            && (0.5..=1.5).contains(&mag(h)),
        "{} {}: plausible data: low-g |a| {} g (0.9..1.1), gyro max {} dps (<= 5), high-g |h| {} g (0.5..1.5), {} of {} samples, errors {}",
        label,
        mag(a),
        max_abs(g),
        mag(h),
        acc.n,
        want,
        acc.errors
    );
}

/// Counts INT1 rising edges (PWM slice 3) and INT2 rising edges (polled) for `cycles` CPU cycles.
/// Returns (INT1 edges, INT2 edges, cycles elapsed).
fn edge_window(int2: &Input<'_>, counter: &Pwm<'_>, cycles: u32) -> (u32, u32, u32) {
    let mut prev = int2.is_high();
    let c0 = counter.counter();
    let t0 = cyc();
    let mut n2 = 0u32;
    let mut now;
    loop {
        let lv = int2.is_high();
        if lv && !prev {
            n2 += 1;
        }
        prev = lv;
        now = cyc().wrapping_sub(t0);
        if now >= cycles {
            break;
        }
    }
    let c1 = counter.counter();
    (c1.wrapping_sub(c0) as u32, n2, now)
}

fn hz_of(edges: u32, cycles: u32) -> f32 {
    edges as f32 * clk_hz() / cycles as f32
}

fn pct(measured: f32, expected: f32) -> f32 {
    (measured / expected - 1.0) * 100.0
}

// ---------------------------------------------------------------------------------------------
// D1
// ---------------------------------------------------------------------------------------------

async fn d1<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>, int1: &mut Input<'_>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] D1: every high-g full scale, low-g + gyro + high-g all at 960 Hz");
    let scales = [
        (HighGFullScale::G32, "32 g"),
        (HighGFullScale::G64, "64 g"),
        (HighGFullScale::G128, "128 g"),
        (HighGFullScale::G256, "256 g"),
        (HighGFullScale::G320, "320 g"),
    ];
    let mut base: Option<([f32; 3], [f32; 3])> = None;
    for (fs, name) in scales {
        let c = cfg(
            Odr::Hz960,
            HighGOdr::Hz960,
            fs,
            OperatingMode::HighPerformance,
            false,
        );
        if apply(fails, name, imu, &c).await.is_none() {
            continue;
        }
        let acc = collect(imu, int1, &c, 400).await;
        describe(name, &acc);
        let (a, g, h) = acc.means();
        defmt::info!(
            "[STAT] D1 {}: mean high-g {} g, low-g {} g, difference {} g",
            name,
            h,
            a,
            sub(h, a)
        );
        if acc.disturbed() {
            defmt::info!(
                "[INFO] D1 {}: window DISTURBED (gyro peak {} dps, low-g |a| spread {} g): checks (a) (b) (c) not judged",
                name,
                acc.gpeak,
                acc.amax - acc.amin
            );
            continue;
        }
        let d = sub(h, a);
        check!(
            *fails,
            max_abs(d) <= OFFSET_LIMIT_G,
            "{} D1 {} (a): high-g minus low-g per axis {} g, worst {} g, sanity bound {} g",
            name,
            d,
            max_abs(d),
            OFFSET_LIMIT_G
        );
        match base {
            None => {
                base = Some((h, a));
                defmt::info!(
                    "[INFO] D1 {}: reference for (b) and the low-g comparison",
                    name
                );
            }
            Some((bh, ba)) => {
                let dh = sub(h, bh);
                defmt::info!(
                    "[STAT] D1 {}: |h| {} g vs {} g at 32 g = {}%",
                    name,
                    mag(h),
                    mag(bh),
                    pct(mag(h), mag(bh))
                );
                check!(
                    *fails,
                    max_abs(dh) <= SCALE_TOL_G,
                    "{} D1 {} (b): high-g vector minus the 32 g one {} g, worst {} g, tolerance {} g per axis",
                    name,
                    dh,
                    max_abs(dh),
                    SCALE_TOL_G
                );
                let da = sub(a, ba);
                check!(
                    *fails,
                    max_abs(da) <= LOWG_TOL_G,
                    "{} D1 {} (c): low-g vector minus the 32 g setting's {} g, worst {} g, tolerance {} g",
                    name,
                    da,
                    max_abs(da),
                    LOWG_TOL_G
                );
            }
        }
        check!(
            *fails,
            acc.n >= 360 && acc.errors == 0 && (0.9..=1.1).contains(&mag(a)) && max_abs(g) <= 5.0,
            "{} D1 {} (c): low-g |a| {} g (0.9..1.1), gyro max {} dps (<= 5), samples {} of 400, errors {}",
            name,
            mag(a),
            max_abs(g),
            acc.n,
            acc.errors
        );
    }
}

// ---------------------------------------------------------------------------------------------
// D2
// ---------------------------------------------------------------------------------------------

async fn d2<S>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    int1: &mut Input<'_>,
    int2: &Input<'_>,
    counter: &Pwm<'_>,
) where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] D2: every high-g output rate with low-g + gyro at 7680 Hz");
    let rates = [
        (HighGOdr::Hz480, "high-g 480 Hz"),
        (HighGOdr::Hz960, "high-g 960 Hz"),
        (HighGOdr::Hz1920, "high-g 1920 Hz"),
        (HighGOdr::Hz3840, "high-g 3840 Hz"),
        (HighGOdr::Hz7680, "high-g 7680 Hz"),
    ];
    for (hodr, name) in rates {
        let c = cfg(
            Odr::Hz7680,
            hodr,
            HighGFullScale::G32,
            OperatingMode::HighPerformance,
            false,
        );
        if apply(fails, name, imu, &c).await.is_none() {
            continue;
        }
        let (n1, n2, cycles) = edge_window(int2, counter, 2 * clk_hz() as u32);
        let (r1, r2) = (hz_of(n1, cycles), hz_of(n2, cycles));
        let nominal = hodr.hz() / 7680.0;
        let ratio = n2 as f32 / n1.max(1) as f32;
        let err = pct(ratio, nominal);
        defmt::info!(
            "[STAT] D2 {}: INT1 {} edges = {} Hz, INT2 {} edges = {} Hz, ratio {} (nominal {}); INT2 {}% from {} Hz nominal",
            name,
            n1,
            r1,
            n2,
            r2,
            ratio,
            nominal,
            pct(r2, hodr.hz()),
            hodr.hz()
        );
        check!(
            *fails,
            n1 > 1000 && n2 > 100 && abs(err) <= 0.5,
            "{} D2 {}: INT2/INT1 edge ratio {} vs nominal {}, {}% (limit 0.5)",
            name,
            ratio,
            nominal,
            err
        );
        let acc = collect(imu, int1, &c, 200).await;
        plausible(fails, name, &acc, 200);
    }
}

// ---------------------------------------------------------------------------------------------
// D3
// ---------------------------------------------------------------------------------------------

struct FifoCount {
    words: u32,
    gyro: u32,
    accel: u32,
    high_g: u32,
    other: u32,
    unknown: u32,
    trailing: u32,
    overruns: u32,
    errors: u32,
}

impl FifoCount {
    fn feed(&mut self, bytes: &[u8]) {
        for f in fifo_frames(bytes) {
            match f {
                FifoFrame::Gyro(_) => self.gyro += 1,
                FifoFrame::Accel(_) => self.accel += 1,
                FifoFrame::HighG(_) => self.high_g += 1,
                FifoFrame::Unknown(_) => self.unknown += 1,
                FifoFrame::Trailing(_) => {
                    self.trailing += 1;
                    continue;
                }
                _ => self.other += 1,
            }
            self.words += 1;
        }
    }
}

/// Flushes the FIFO, drains it for `secs` seconds and returns the counts and the elapsed
/// microseconds between the flush and the last status read.
async fn fifo_run<S>(imu: &mut Lsm6dsk320x<S>, secs: u64) -> (FifoCount, u64)
where
    S: SpiDevice,
{
    let mut n = FifoCount {
        words: 0,
        gyro: 0,
        accel: 0,
        high_g: 0,
        other: 0,
        unknown: 0,
        trailing: 0,
        overruns: 0,
        errors: 0,
    };
    let mut buf = [0u8; 100 * 7];
    if imu.fifo_flush().await.is_err() {
        n.errors += 1;
        return (n, 1);
    }
    let t0 = Instant::now();
    let mut t_last;
    loop {
        Timer::after_millis(20).await;
        loop {
            t_last = Instant::now();
            match imu.fifo_read(&mut buf).await {
                Ok(r) => {
                    n.feed(&buf[..r.bytes]);
                    if r.flags.is_some_and(|f| f.any_overrun()) {
                        n.overruns += 1;
                    }
                    if r.unread.unwrap_or(0) == 0 {
                        break;
                    }
                }
                Err(_) => {
                    n.errors += 1;
                    break;
                }
            }
        }
        if t_last - t0 >= Duration::from_secs(secs) {
            break;
        }
    }
    (n, (t_last - t0).as_micros())
}

async fn d3<S>(
    fails: &mut u32,
    imu: &mut Lsm6dsk320x<S>,
    int1: &mut Input<'_>,
    int2: &Input<'_>,
    counter: &Pwm<'_>,
) where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!(
        "[INFO] D3: high-accuracy ODR mode with the high-g channel on (960 Hz low-g and gyro, every high-g rate Config::validate accepts)"
    );
    let sets = [
        (HaodrSet::Base960, "HAODR set 960"),
        (HaodrSet::Base1000, "HAODR set 1000"),
        (HaodrSet::Base800, "HAODR set 800"),
        (HaodrSet::Base833, "HAODR set 833"),
    ];
    let hodrs = [
        HighGOdr::Hz960,
        HighGOdr::Hz480,
        HighGOdr::Hz1920,
        HighGOdr::Hz3840,
        HighGOdr::Hz7680,
    ];
    for (set, name) in sets {
        let mode = OperatingMode::HighAccuracyOdr(set);
        for hodr in hodrs {
            let main = matches!(hodr, HighGOdr::Hz960);
            let hz = hodr.hz();
            for gyro_pin in [false, true] {
                if gyro_pin && !main {
                    continue;
                }
                let c = cfg(Odr::Hz960, hodr, HighGFullScale::G32, mode, gyro_pin);
                if c.validate().is_err() {
                    defmt::info!(
                        "[INFO] D3 {} high-g {} Hz: Config::validate rejects it, not run",
                        name,
                        hz
                    );
                    continue;
                }
                let Some(info) = apply(fails, name, imu, &c).await else {
                    continue;
                };
                let window = if main { 2 } else { 1 };
                let (n1, n2, cycles) = edge_window(int2, counter, window * clk_hz() as u32);
                let (r1, r2) = (hz_of(n1, cycles), hz_of(n2, cycles));
                let (what, reported_mhz) = if gyro_pin {
                    ("gyro", info.gyro_odr_mhz)
                } else {
                    ("low-g", info.accel_odr_mhz)
                };
                let reported = reported_mhz as f32 / 1000.0;
                let table = Odr::Hz960.haodr_mhz(set).unwrap_or(0) as f32 / 1000.0;
                let hg = info.high_g_odr_mhz as f32 / 1000.0;
                defmt::info!(
                    "[STAT] D3 {} high-g {} Hz: INT1 ({} data-ready) {} edges = {} Hz, driver reports {} Hz (Table 22 {} Hz); INT2 (high-g) {} edges = {} Hz, Info::high_g_odr_mhz {} mHz; INT2/INT1 ratio {}",
                    name,
                    hz,
                    what,
                    n1,
                    r1,
                    reported,
                    table,
                    n2,
                    r2,
                    info.high_g_odr_mhz,
                    n2 as f32 / n1.max(1) as f32
                );
                check!(
                    *fails,
                    n1 > 100 && abs(pct(r1, reported)) <= 0.5,
                    "{} D3 {} high-g {} Hz: {} data-ready rate {} Hz vs driver-reported {} Hz, {}% (limit 0.5)",
                    name,
                    hz,
                    what,
                    r1,
                    reported,
                    pct(r1, reported)
                );
                if gyro_pin {
                    continue;
                }
                check!(
                    *fails,
                    n2 > 100 && hg > 0.0 && abs(pct(r2, hg)) <= 0.5,
                    "{} D3 {} high-g {} Hz: high-g data-ready rate {} Hz vs Info::high_g_odr_mhz {} Hz, {}% (limit 0.5)",
                    name,
                    hz,
                    r2,
                    hg,
                    pct(r2, hg)
                );
                if main {
                    let acc = collect(imu, int1, &c, 200).await;
                    plausible(fails, name, &acc, 200);
                }

                // The FIFO with gyro, low-g and high-g batched.
                let fc = c.with_fifo(
                    FifoConfig::new()
                        .with_mode(FifoMode::Continuous)
                        .with_gyro(BatchRate::Hz960)
                        .with_accel(BatchRate::Hz960)
                        .with_high_g(true),
                );
                let Some(finfo) = apply(fails, name, imu, &fc).await else {
                    continue;
                };
                let (n, us) = fifo_run(imu, window as u64).await;
                let secs = us as f32 / 1e6;
                let rate = n.words as f32 / secs;
                let want = finfo.fifo_words_per_second as f32;
                defmt::info!(
                    "[STAT] D3 {} high-g {} Hz FIFO: {} words in {} s = {} words/s (gyro {} accel {} high-g {} other {} unknown {} trailing {}), fifo_words_per_second {}, overruns {}, bus errors {}; per kind Hz: gyro {} accel {} high-g {}",
                    name,
                    hz,
                    n.words,
                    secs,
                    rate,
                    n.gyro,
                    n.accel,
                    n.high_g,
                    n.other,
                    n.unknown,
                    n.trailing,
                    finfo.fifo_words_per_second,
                    n.overruns,
                    n.errors,
                    n.gyro as f32 / secs,
                    n.accel as f32 / secs,
                    n.high_g as f32 / secs
                );
                check!(
                    *fails,
                    n.words > 500 && n.errors == 0 && abs(pct(rate, want)) <= 1.0,
                    "{} D3 {} high-g {} Hz: FIFO words/s {} vs fifo_words_per_second {}, {}% (limit 1)",
                    name,
                    hz,
                    rate,
                    want,
                    pct(rate, want)
                );
                check!(
                    *fails,
                    n.unknown == 0 && n.trailing == 0 && n.other == 0 && n.overruns == 0,
                    "{} D3 {} high-g {} Hz: FIFO words: unknown {}, trailing {}, other kinds {}, overruns {}",
                    name,
                    hz,
                    n.unknown,
                    n.trailing,
                    n.other,
                    n.overruns
                );
            }
        }
    }
}

async fn bus_part(p: &mut Peripherals, fails: &mut u32) {
    let bus = Spi::new(
        p.SPI1.reborrow(),
        p.PIN_10.reborrow(),
        p.PIN_11.reborrow(),
        p.PIN_8.reborrow(),
        p.DMA_CH0.reborrow(),
        p.DMA_CH1.reborrow(),
        Irqs,
        bus_cfg(SPI_HZ, false),
    );
    let cs = Output::new(p.PIN_9.reborrow(), Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    let mut imu = Lsm6dsk320x::new(dev);

    // SAFETY: the Input only awaits edges and the PWM slice only counts; neither reconfigures the pad.
    let pwm_pin = unsafe { p.PIN_7.clone_unchecked() };
    let mut int1 = Input::new(p.PIN_7.reborrow(), Pull::None);
    let counter = Pwm::new_input(
        p.PWM_SLICE3.reborrow(),
        pwm_pin,
        Pull::None,
        InputMode::RisingEdge,
        pwm::Config::default(),
    );
    let int2 = Input::new(p.PIN_6.reborrow(), Pull::None);

    let c0 = cfg(
        Odr::Hz960,
        HighGOdr::Hz960,
        HighGFullScale::G32,
        OperatingMode::HighPerformance,
        false,
    );
    match imu.init(&mut Delay, &c0).await {
        Ok(_) => check!(*fails, true, "{} init (low-g + gyro + high-g)"),
        Err(e) => {
            check!(*fails, false, "{} init: {}", defmt::Debug2Format(&e));
            return;
        }
    }
    d1(fails, &mut imu, &mut int1).await;
    d2(fails, &mut imu, &mut int1, &int2, &counter).await;
    d3(fails, &mut imu, &mut int1, &int2, &counter).await;
    let s = imu.stats();
    defmt::info!("[STAT] driver counters after D1-D3: {}", s);
}

// ---------------------------------------------------------------------------------------------
// D4
// ---------------------------------------------------------------------------------------------

async fn timing_part(p: &mut Peripherals, fails: &mut u32, hz: u32, mode3: bool, label: &str) {
    let bus = Spi::new(
        p.SPI1.reborrow(),
        p.PIN_10.reborrow(),
        p.PIN_11.reborrow(),
        p.PIN_8.reborrow(),
        p.DMA_CH0.reborrow(),
        p.DMA_CH1.reborrow(),
        Irqs,
        bus_cfg(hz, mode3),
    );
    let cs = Output::new(p.PIN_9.reborrow(), Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    let mut imu = Lsm6dsk320x::new(dev);
    let c = cfg(
        Odr::Hz960,
        HighGOdr::Hz960,
        HighGFullScale::G32,
        OperatingMode::HighPerformance,
        false,
    );
    if let Err(e) = imu.init(&mut Delay, &c).await {
        check!(
            *fails,
            false,
            "{} D4 {}: init: {}",
            label,
            defmt::Debug2Format(&e)
        );
        return;
    }
    let v = imu.verify().await;
    check!(*fails, v.is_ok(), "{} D4 {}: init + verify() Ok", label);
    Timer::after_millis(150).await;
    let us = |cycles: f32| cycles / (clk_hz() / 1e6);
    const N: u32 = 1000;
    let (mut sum, mut worst, mut errors) = (0u64, 0u32, 0u32);
    for _ in 0..N {
        let t = cyc();
        let r = imu.read_all().await;
        let d = cyc().wrapping_sub(t);
        sum += d as u64;
        worst = worst.max(d);
        errors += r.is_err() as u32;
    }
    defmt::info!(
        "[STAT] D4 {}: read_all mean {} us, worst {} us over {} calls",
        label,
        us(sum as f32 / N as f32),
        us(worst as f32),
        N
    );
    check!(
        *fails,
        errors == 0,
        "{} D4 {}: read_all bus errors {}",
        label,
        errors
    );
    let (mut sum, mut worst, mut errors) = (0u64, 0u32, 0u32);
    for _ in 0..N {
        let t = cyc();
        let r1 = imu.read().await;
        let r2 = imu.read_high_g().await;
        let d = cyc().wrapping_sub(t);
        sum += d as u64;
        worst = worst.max(d);
        errors += (r1.is_err() || r2.is_err()) as u32;
    }
    defmt::info!(
        "[STAT] D4 {}: read() + read_high_g() mean {} us, worst {} us over {} pairs",
        label,
        us(sum as f32 / N as f32),
        us(worst as f32),
        N
    );
    check!(
        *fails,
        errors == 0,
        "{} D4 {}: read + read_high_g bus errors {}",
        label,
        errors
    );
    let v = imu.verify().await;
    check!(
        *fails,
        v.is_ok(),
        "{} D4 {}: verify() Ok after the timing runs",
        label
    );
}

// ---------------------------------------------------------------------------------------------
// End
// ---------------------------------------------------------------------------------------------

async fn end_part(p: &mut Peripherals, fails: &mut u32) {
    let bus = Spi::new(
        p.SPI1.reborrow(),
        p.PIN_10.reborrow(),
        p.PIN_11.reborrow(),
        p.PIN_8.reborrow(),
        p.DMA_CH0.reborrow(),
        p.DMA_CH1.reborrow(),
        Irqs,
        bus_cfg(SPI_HZ, false),
    );
    let cs = Output::new(p.PIN_9.reborrow(), Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    let mut imu = Lsm6dsk320x::new(dev);
    let mut int1 = Input::new(p.PIN_7.reborrow(), Pull::None);
    let c = Config::new();
    if let Err(e) = imu.init(&mut Delay, &c).await {
        check!(*fails, false, "{} final init: {}", defmt::Debug2Format(&e));
        return;
    }
    check!(*fails, true, "{} final init to the default configuration");
    let v = imu.verify().await;
    check!(
        *fails,
        v.is_ok(),
        "{} final verify() Ok: {}",
        defmt::Debug2Format(&v)
    );
    let (mut good, mut bad, mut errs) = (0u32, 0u32, 0u32);
    let mut a_sum = 0f32;
    for _ in 0..400 {
        if good + bad >= 200 {
            break;
        }
        if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
            .await
            .is_err()
        {
            errs += 1;
            continue;
        }
        match imu.read().await {
            Ok(s) => {
                let a = mag(s.accel.accel_ms2(c.accel_fs)) / G;
                let g = s.gyro.gyro_rads(c.gyro_fs).map(|v| v * DEG);
                a_sum += a;
                if (0.9..=1.1).contains(&a) && max_abs(g) <= 5.0 {
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
        "{} final 200 plausible samples (|a| 1 g +-10 %, gyro within 5 dps)"
    );
    let s = imu.stats();
    defmt::info!("[INFO] health: driver counters {}", s);
    check!(
        *fails,
        s.bus_errors == 0 && s.fifo_overruns == 0,
        "{} health: bus errors {}, fifo overruns {}",
        s.bus_errors,
        s.fifo_overruns
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    dwt_on();
    let start = Instant::now();
    let mut fails = 0u32;
    bus_part(&mut p, &mut fails).await;
    timing_part(
        &mut p,
        &mut fails,
        9_375_000,
        true,
        "SPI mode 3 at 9.375 MHz",
    )
    .await;
    timing_part(
        &mut p,
        &mut fails,
        7_500_000,
        false,
        "SPI mode 0 at 7.5 MHz",
    )
    .await;
    end_part(&mut p, &mut fails).await;
    defmt::info!("[INFO] run time {} ms", start.elapsed().as_millis());
    hil_common::exit(fails)
}

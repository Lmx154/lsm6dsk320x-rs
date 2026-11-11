//! On-board check of the lsm6dsk320x driver (U23 on SPI1, INT1 on GPIO7, INT2 on GPIO6).
//!
//! The driver is exercised through embassy-rp's DMA SPI and through its blocking SPI. The board
//! lies still on the bench.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peripherals, bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::{
    AccelFullScale, Config, DataReady, GyroFullScale, HighGConfig, HighGFullScale, HighGOdr,
    IntRouting, Odr, blocking, reg,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const ACCEL_FS: AccelFullScale = AccelFullScale::G4;
const GYRO_FS: GyroFullScale = GyroFullScale::Dps500;
const HIGH_G_FS: HighGFullScale = HighGFullScale::G32;
const G: f32 = lsm6dsk320x::sample::G;
const DEG: f32 = 180.0 / core::f32::consts::PI;

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

fn accel_config(odr: Odr) -> Config {
    Config::new()
        .with_accel_fs(ACCEL_FS)
        .with_accel_odr(odr)
        .with_gyro_fs(GYRO_FS)
        .with_gyro_odr(odr)
        .with_high_g(None)
        .with_int1(IntRouting::new().with_data_ready(DataReady::new().with_accel(true)))
        .with_int2(IntRouting::new())
}

fn high_g_config(odr: HighGOdr) -> Config {
    accel_config(Odr::Hz960)
        .with_high_g(Some(
            HighGConfig::new().with_odr(odr).with_full_scale(HIGH_G_FS),
        ))
        .with_int1(IntRouting::new())
        .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)))
}

/// What one timed window of reads, one per data-ready edge, saw.
struct Window {
    reads: u32,
    errors: u32,
    timeouts: u32,
    edges: Option<u32>,
    elapsed_us: u64,
    sum_us: u64,
    worst_us: u32,
}

impl Window {
    fn rate(&self, n: u32) -> f32 {
        n as f32 * 1e6 / self.elapsed_us as f32
    }

    fn mean_us(&self) -> f32 {
        self.sum_us as f32 / self.reads.max(1) as f32
    }
}

/// Runs `op` once per rising edge of `int` for `ms` milliseconds. The edges are also counted by
/// the PWM slice in hardware, independently of the reads, when `pwm` is given.
async fn window(
    int: &mut Input<'_>,
    pwm: Option<&Pwm<'_>>,
    ms: u64,
    mut op: impl AsyncFnMut() -> bool,
) -> Window {
    let mut w = Window {
        reads: 0,
        errors: 0,
        timeouts: 0,
        edges: None,
        elapsed_us: 0,
        sum_us: 0,
        worst_us: 0,
    };
    let c0 = pwm.map(|p| p.counter());
    let start = Instant::now();
    let end = start + Duration::from_millis(ms);
    while Instant::now() < end {
        if with_timeout(Duration::from_millis(50), int.wait_for_rising_edge())
            .await
            .is_err()
        {
            w.timeouts += 1;
            continue;
        }
        let t = Instant::now();
        let ok = op().await;
        let us = t.elapsed().as_micros() as u32;
        w.reads += 1;
        w.errors += !ok as u32;
        w.sum_us += us as u64;
        w.worst_us = w.worst_us.max(us);
    }
    w.elapsed_us = start.elapsed().as_micros();
    if let (Some(p), Some(c0)) = (pwm, c0) {
        w.edges = Some(p.counter().wrapping_sub(c0) as u32);
    }
    w
}

fn report(fails: &mut u32, label: &str, nominal: f32, w: &Window) {
    let rate = w.rate(w.reads);
    let err = (rate / nominal - 1.0) * 100.0;
    defmt::info!(
        "[INFO] {} nominal {} Hz: reads {}, bus errors {}, edge timeouts {}, elapsed {} us",
        label,
        nominal,
        w.reads,
        w.errors,
        w.timeouts,
        w.elapsed_us
    );
    check!(
        *fails,
        abs(err) <= 5.0 && w.errors == 0 && w.timeouts == 0,
        "{} {} rate {} Hz, {}% from nominal (limit 5)",
        label,
        rate,
        err
    );
    defmt::info!(
        "[STAT] {} read: mean {} us, worst {} us",
        label,
        w.mean_us(),
        w.worst_us
    );
    match w.edges {
        Some(edges) => {
            let missed = edges as i64 - w.reads as i64;
            defmt::info!(
                "[STAT] {} INT edge counter: {} edges = {} Hz",
                label,
                edges,
                w.rate(edges)
            );
            check!(
                *fails,
                missed <= 1,
                "{} {} data-ready edges not read: {} (edges {}, reads {})",
                label,
                missed,
                edges,
                w.reads
            );
        }
        None => defmt::info!(
            "[INFO] {} edges not counted independently (INT2 is the A pin of PWM slice 3)",
            label
        ),
    }
}

async fn async_part(p: &mut Peripherals, fails: &mut u32) {
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = SPI_HZ;
    let bus = Spi::new(
        p.SPI1.reborrow(),
        p.PIN_10.reborrow(),
        p.PIN_11.reborrow(),
        p.PIN_8.reborrow(),
        p.DMA_CH0.reborrow(),
        p.DMA_CH1.reborrow(),
        Irqs,
        spi_cfg,
    );
    let cs = Output::new(p.PIN_9.reborrow(), Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    let mut imu = lsm6dsk320x::Lsm6dsk320x::new(dev);

    // INT1: the GPIO edge interrupt waits and a PWM slice 3 B input counts, on the same pad.
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
    let mut int2 = Input::new(p.PIN_6.reborrow(), Pull::None);

    // 1. init and identity.
    if let Err(e) = imu.init(&mut Delay, &accel_config(Odr::Hz960)).await {
        check!(*fails, false, "{} init: {}", defmt::Debug2Format(&e));
        return;
    }
    check!(*fails, true, "{} init");
    let id = imu.read_register(reg::WHO_AM_I).await;
    check!(
        *fails,
        id.as_ref().ok() == Some(&reg::WHO_AM_I_VALUE),
        "{} WHO_AM_I {}",
        defmt::Debug2Format(&id)
    );

    // 2. gyro and low-g accel per INT1 edge.
    for odr in [Odr::Hz960, Odr::Hz7680] {
        if let Err(e) = imu.init(&mut Delay, &accel_config(odr)).await {
            check!(*fails, false, "{} init: {}", defmt::Debug2Format(&e));
            continue;
        }
        let w = window(&mut int1, Some(&counter), 2000, async || {
            imu.read().await.is_ok()
        })
        .await;
        report(fails, "gyro+low-g", odr.hz(), &w);
    }

    // 3. high-g per INT2 edge, then read_all timing.
    for odr in [HighGOdr::Hz960, HighGOdr::Hz7680] {
        if let Err(e) = imu.init(&mut Delay, &high_g_config(odr)).await {
            check!(*fails, false, "{} init: {}", defmt::Debug2Format(&e));
            continue;
        }
        let w = window(&mut int2, None, 1000, async || {
            imu.read_high_g().await.is_ok()
        })
        .await;
        report(fails, "high-g", odr.hz(), &w);
    }

    if let Err(e) = imu.init(&mut Delay, &high_g_config(HighGOdr::Hz960)).await {
        check!(
            *fails,
            false,
            "{} init before read_all: {}",
            defmt::Debug2Format(&e)
        );
        return;
    }
    const N: u32 = 500;
    let (mut sum_us, mut worst_us, mut errors) = (0u64, 0u32, 0u32);
    let (mut temp, mut gyro, mut accel, mut high_g) = (0f32, [0f32; 3], [0f32; 3], [0f32; 3]);
    for _ in 0..N {
        let t = Instant::now();
        let r = imu.read_all().await;
        let us = t.elapsed().as_micros() as u32;
        sum_us += us as u64;
        worst_us = worst_us.max(us);
        match r {
            Ok(all) => {
                temp += all.temperature_c();
                let (g, a, h) = (
                    all.gyro.gyro_rads(GYRO_FS),
                    all.accel.accel_ms2(ACCEL_FS),
                    all.high_g.high_g_ms2(HIGH_G_FS),
                );
                for i in 0..3 {
                    gyro[i] += g[i];
                    accel[i] += a[i];
                    high_g[i] += h[i];
                }
            }
            Err(_) => errors += 1,
        }
    }
    defmt::info!(
        "[STAT] read_all: mean {} us, worst {} us over {} calls",
        sum_us as f32 / N as f32,
        worst_us,
        N
    );
    check!(*fails, errors == 0, "{} read_all bus errors {}", errors);

    // 4. plausibility at rest, on the means of the read_all samples.
    let n = (N - errors).max(1) as f32;
    let (temp, gyro, accel, high_g) = (
        temp / n,
        gyro.map(|v| v / n),
        accel.map(|v| v / n),
        high_g.map(|v| v / n),
    );
    let a = mag(accel) / G;
    check!(
        *fails,
        (0.9..=1.1).contains(&a),
        "{} low-g |a| {} g (0.9..1.1)",
        a
    );
    let h = mag(high_g) / G;
    check!(
        *fails,
        (0.5..=1.5).contains(&h),
        "{} high-g |a| {} g (0.5..1.5)",
        h
    );
    let gd = gyro.map(|v| v * DEG);
    check!(
        *fails,
        gd.iter().all(|v| abs(*v) <= 5.0),
        "{} gyro {} dps (each within 5)",
        gd
    );
    check!(
        *fails,
        (10.0..=50.0).contains(&temp),
        "{} temperature {} C (10..50)",
        temp
    );
    defmt::info!("[INFO] high-g axes {} g", high_g.map(|v| v / G));
}

fn blocking_part(p: &mut Peripherals, fails: &mut u32) {
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = SPI_HZ;
    let bus = Spi::new_blocking(
        p.SPI1.reborrow(),
        p.PIN_10.reborrow(),
        p.PIN_11.reborrow(),
        p.PIN_8.reborrow(),
        spi_cfg,
    );
    let cs = Output::new(p.PIN_9.reborrow(), Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    let mut imu = blocking::Lsm6dsk320x::new(dev);
    if let Err(e) = imu.init(&mut Delay, &accel_config(Odr::Hz960)) {
        check!(
            *fails,
            false,
            "{} blocking init: {}",
            defmt::Debug2Format(&e)
        );
        return;
    }
    let sample = match imu.read() {
        Ok(s) => s,
        Err(e) => {
            check!(
                *fails,
                false,
                "{} blocking read: {}",
                defmt::Debug2Format(&e)
            );
            return;
        }
    };
    let a = mag(sample.accel.accel_ms2(ACCEL_FS)) / G;
    let gd = sample.gyro.gyro_rads(GYRO_FS).map(|v| v * DEG);
    check!(
        *fails,
        (0.9..=1.1).contains(&a) && gd.iter().all(|v| abs(*v) <= 5.0),
        "{} blocking init + read: |a| {} g, gyro {} dps",
        a,
        gd
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut fails = 0u32;
    async_part(&mut p, &mut fails).await;
    blocking_part(&mut p, &mut fails);
    hil_common::exit(fails)
}

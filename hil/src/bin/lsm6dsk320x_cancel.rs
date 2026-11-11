//! On-board cancel sweep of contract rule A4d for the lsm6dsk320x driver (U23 on SPI1).
//!
//! Only `read()` is ever cancelled, through `ExclusiveDevice` at the smoke test's clock, and
//! `recover()` follows every cancel. The board lies still on the bench.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::{Config, Error, Lsm6dsk320x, RawSample};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const G: f32 = lsm6dsk320x::sample::G;
const DEG: f32 = 180.0 / core::f32::consts::PI;

const ACCEL_LIMIT_MG: f32 = 60.0;
const GYRO_LIMIT_DPS: f32 = 3.0;
const TEMP_LIMIT_C: f32 = 3.0;

const MAX_CANCELS: u32 = 2500;
const PHASE1_CANCELS: u32 = 100;
/// Attempts in phase 3. About a third of them really cancel a read (139 to 208 of 500 in the first
/// runs), so 750 attempts keep the count of real cancels above the 150 the phase asks for.
const PHASE3_CANCELS: u32 = 750;
const BLOCK: u32 = 25;
/// Blocks of `BLOCK` cancels per delay step of the phase 2 sweep.
const SWEEP_BLOCKS: u32 = 2;
/// The phase 2 sweep runs from 0 to the median read time plus this, in microseconds.
const SWEEP_PAST_T_US: u32 = 2;
const RUN_LIMIT_S: u64 = 50;

type Dev = ExclusiveDevice<
    Spi<'static, embassy_rp::peripherals::SPI1, spi::Async>,
    Output<'static>,
    Delay,
>;
type E = Error<<Dev as embedded_hal::spi::ErrorType>::Error>;

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

fn abs(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

struct Reference {
    accel_mg: [f32; 3],
    gyro_dps: [f32; 3],
    temp_c: f32,
}

#[derive(Default)]
struct Sweep {
    cancelled: u32,
    completed: u32,
    recover_failures: u32,
    wrong: u32,
    verify_failures: u32,
    recover_n: u32,
    recover_sum_us: u64,
    recover_worst_us: u32,
}

enum Cut {
    Cancelled,
    Done(bool),
}

struct Ctx {
    imu: Lsm6dsk320x<Dev>,
    int1: Input<'static>,
    cfg: Config,
    reference: Reference,
    fails: u32,
    cancels: u32,
    sensor_fault: bool,
    time_up: bool,
    start: Instant,
}

impl Ctx {
    fn stopped(&self) -> bool {
        self.sensor_fault || self.time_up
    }

    fn check_time(&mut self) -> bool {
        if self.start.elapsed() > Duration::from_secs(RUN_LIMIT_S) {
            self.time_up = true;
            defmt::error!(
                "[FAIL] run time limit of {} s reached, stopping",
                RUN_LIMIT_S
            );
        }
        self.time_up
    }

    fn wrong(&self, r: &Result<RawSample, E>) -> bool {
        match r {
            Err(_) => true,
            Ok(s) => {
                let a = s.accel.accel_ms2(self.cfg.accel_fs).map(|v| v / G * 1000.0);
                let w = s.gyro.gyro_rads(self.cfg.gyro_fs).map(|v| v * DEG);
                (0..3).any(|i| {
                    abs(a[i] - self.reference.accel_mg[i]) > ACCEL_LIMIT_MG
                        || abs(w[i] - self.reference.gyro_dps[i]) > GYRO_LIMIT_DPS
                })
            }
        }
    }

    fn show_sample(&self, what: &str, d: u32, r: &Result<RawSample, E>) {
        match r {
            Ok(s) => defmt::error!(
                "[FAIL] {} at d={} us: accel raw {} {} {}, gyro raw {} {} {}",
                what,
                d,
                s.accel.x,
                s.accel.y,
                s.accel.z,
                s.gyro.x,
                s.gyro.y,
                s.gyro.z
            ),
            Err(e) => defmt::error!("[FAIL] {} at d={} us: {}", what, d, defmt::Debug2Format(e)),
        }
    }

    fn show_err(&self, what: &str, d: u32, e: &E) {
        match e {
            Error::ConfigMismatch(m) => defmt::error!(
                "[FAIL] {} at d={} us: register 0x{=u8:x} mask 0x{=u8:x} expected 0x{=u8:x} found 0x{=u8:x}",
                what,
                d,
                m.register,
                m.mask,
                m.expected,
                m.found
            ),
            _ => defmt::error!("[FAIL] {} at d={} us: {}", what, d, defmt::Debug2Format(e)),
        }
    }

    fn sensor_stop(&mut self, sw: &Sweep) {
        self.sensor_fault = true;
        self.fails += 1;
        defmt::error!(
            "[FAIL] first sensor-side failure, all cancel phases stop: cancels so far {}, phase counters: cancelled {}, completed {}, recover failures {}, wrong reads {}, verify failures {}",
            self.cancels,
            sw.cancelled,
            sw.completed,
            sw.recover_failures,
            sw.wrong,
            sw.verify_failures
        );
    }

    /// Cancels exactly one `read()` after `d` microseconds.
    async fn cut(&mut self, d: u32) -> Cut {
        self.cancels += 1;
        let r = with_timeout(Duration::from_micros(d as u64), self.imu.read()).await;
        match r {
            Err(_) => Cut::Cancelled,
            Ok(r) => {
                let w = self.wrong(&r);
                if w {
                    self.show_sample("completed read wrong", d, &r);
                }
                Cut::Done(w)
            }
        }
    }

    /// `recover()` at once, timed. False on failure (state already stopped).
    async fn recover(&mut self, d: u32, sw: &mut Sweep) -> bool {
        let t = Instant::now();
        let r = self.imu.recover().await;
        let us = t.elapsed().as_micros() as u32;
        sw.recover_n += 1;
        sw.recover_sum_us += us as u64;
        sw.recover_worst_us = sw.recover_worst_us.max(us);
        if let Err(e) = r {
            sw.recover_failures += 1;
            self.show_err("recover", d, &e);
            self.sensor_stop(sw);
            return false;
        }
        true
    }

    /// Two uncancelled reads, both must be right. False on failure (state already stopped).
    async fn two_reads(&mut self, d: u32, sw: &mut Sweep) -> bool {
        for _ in 0..2 {
            let r = self.imu.read().await;
            if self.wrong(&r) {
                sw.wrong += 1;
                self.show_sample("wrong read after recover", d, &r);
                self.sensor_stop(sw);
                return false;
            }
        }
        true
    }

    /// One cancelled read, `recover()`, two checked reads. False when a sensor failure stopped it.
    async fn cycle(&mut self, d: u32, sw: &mut Sweep) -> bool {
        match self.cut(d).await {
            Cut::Done(w) => {
                sw.completed += 1;
                if w {
                    sw.wrong += 1;
                    self.sensor_stop(sw);
                    return false;
                }
                true
            }
            Cut::Cancelled => {
                sw.cancelled += 1;
                self.recover(d, sw).await && self.two_reads(d, sw).await
            }
        }
    }

    /// `verify()` and one temperature check. False when a sensor failure stopped it.
    async fn block_check(&mut self, d: u32, sw: &mut Sweep) -> bool {
        if let Err(e) = self.imu.verify().await {
            sw.verify_failures += 1;
            self.show_err("verify", d, &e);
            self.sensor_stop(sw);
            return false;
        }
        match self.imu.read_all().await {
            Ok(all) if abs(all.temperature_c() - self.reference.temp_c) <= TEMP_LIMIT_C => true,
            r => {
                sw.wrong += 1;
                match r {
                    Ok(all) => defmt::error!(
                        "[FAIL] temperature wrong at d={} us: {} C against {}",
                        d,
                        all.temperature_c(),
                        self.reference.temp_c
                    ),
                    Err(e) => defmt::error!(
                        "[FAIL] read_all failed at d={} us: {}",
                        d,
                        defmt::Debug2Format(&e)
                    ),
                }
                self.sensor_stop(sw);
                false
            }
        }
    }

    fn print_sweep(&self, name: &str, sw: &Sweep) {
        let mean = sw.recover_sum_us as f32 / sw.recover_n.max(1) as f32;
        defmt::info!(
            "[STAT] {}: cancelled {}, completed before timeout {}, recover failures {}, wrong reads after recover {}, verify failures {}",
            name,
            sw.cancelled,
            sw.completed,
            sw.recover_failures,
            sw.wrong,
            sw.verify_failures
        );
        defmt::info!(
            "[STAT] {}: recover() mean {} us, worst {} us over {} calls; cancels so far {}",
            name,
            mean,
            sw.recover_worst_us,
            sw.recover_n,
            self.cancels
        );
    }

    /// The reference and T. Returns the median T in microseconds, or None.
    async fn phase0(&mut self) -> Option<u32> {
        if let Err(e) = self.imu.init(&mut Delay, &self.cfg).await {
            check!(
                self.fails,
                false,
                "{} phase 0 init: {}",
                defmt::Debug2Format(&e)
            );
            return None;
        }
        let v = self.imu.verify().await;
        check!(
            self.fails,
            v.is_ok(),
            "{} phase 0 verify after init: {}",
            defmt::Debug2Format(&v)
        );
        if v.is_err() {
            return None;
        }

        let mut times = [0u16; 1000];
        let mut errors = 0u32;
        for t in times.iter_mut() {
            let s = Instant::now();
            let r = self.imu.read().await;
            *t = s.elapsed().as_micros().min(65535) as u16;
            errors += r.is_err() as u32;
        }
        times.sort_unstable();
        let median = times[500] as u32;
        let worst = times[999] as u32;
        defmt::info!(
            "[STAT] phase 0: read() duration T median {} us, worst {} us, bus errors {}",
            median,
            worst,
            errors
        );
        if errors != 0 {
            check!(
                self.fails,
                false,
                "{} phase 0 timing reads had {} errors",
                errors
            );
            return None;
        }

        let (mut a, mut w) = ([0f32; 3], [0f32; 3]);
        let mut n = 0f32;
        for _ in 0..400 {
            let Ok(s) = self.imu.read().await else {
                check!(self.fails, false, "{} phase 0 reference read failed");
                return None;
            };
            let sa = s.accel.accel_ms2(self.cfg.accel_fs).map(|v| v / G * 1000.0);
            let sg = s.gyro.gyro_rads(self.cfg.gyro_fs).map(|v| v * DEG);
            for i in 0..3 {
                a[i] += sa[i];
                w[i] += sg[i];
            }
            n += 1.0;
        }
        let mut temp = 0f32;
        for _ in 0..400 {
            let Ok(all) = self.imu.read_all().await else {
                check!(self.fails, false, "{} phase 0 reference read_all failed");
                return None;
            };
            temp += all.temperature_c();
        }
        self.reference = Reference {
            accel_mg: a.map(|v| v / n),
            gyro_dps: w.map(|v| v / n),
            temp_c: temp / 400.0,
        };
        defmt::info!(
            "[STAT] phase 0 reference: accel {} mg, gyro {} dps, temperature {} C",
            self.reference.accel_mg,
            self.reference.gyro_dps,
            self.reference.temp_c
        );

        let mut wrong = 0u32;
        for _ in 0..1000 {
            let r = self.imu.read().await;
            wrong += self.wrong(&r) as u32;
        }
        check!(
            self.fails,
            wrong == 0,
            "{} phase 0: wrong reads in 1000 uncancelled: {}",
            wrong
        );
        if wrong != 0 {
            return None;
        }
        Some(median)
    }

    async fn phase1(&mut self, t: u32) {
        let mut sw = Sweep::default();
        let (mut with_wrong, mut worst_wrong, mut unsettled) = (0u32, 0u32, 0u32);
        for i in 0..PHASE1_CANCELS {
            if self.check_time() {
                return;
            }
            let d = i * (t + 10) / (PHASE1_CANCELS - 1);
            match self.cut(d).await {
                Cut::Done(w) => {
                    sw.completed += 1;
                    sw.wrong += w as u32;
                }
                Cut::Cancelled => {
                    sw.cancelled += 1;
                    let (mut wrong, mut streak) = (0u32, 0u32);
                    for _ in 0..8 {
                        let r = self.imu.read().await;
                        if self.wrong(&r) {
                            wrong += 1;
                            streak = 0;
                        } else {
                            streak += 1;
                            if streak == 2 {
                                break;
                            }
                        }
                    }
                    if streak < 2 {
                        unsettled += 1;
                    }
                    with_wrong += (wrong > 0) as u32;
                    worst_wrong = worst_wrong.max(wrong);
                }
            }
            if !self.recover(d, &mut sw).await {
                break;
            }
        }
        defmt::info!(
            "[STAT] phase 1 (control, no recover between cancel and reads): really cancelled {}, completed first {}, cancels followed by at least one wrong read {}, largest wrong reads after one cancel {}, not settled in 8 reads {}",
            sw.cancelled,
            sw.completed,
            with_wrong,
            worst_wrong,
            unsettled
        );
        self.print_sweep("phase 1", &sw);
        if self.sensor_fault {
            return;
        }
        let v = self.imu.verify().await;
        let ok = v.is_ok();
        check!(
            self.fails,
            ok && sw.recover_failures == 0,
            "{} phase 1: every recover() Ok and verify() after the phase Ok ({})",
            defmt::Debug2Format(&v)
        );
        if let Err(e) = v {
            self.show_err("phase 1 verify", 0, &e);
            self.sensor_fault = true;
        }
    }

    async fn phase2(&mut self, t: u32) {
        let mut sw = Sweep::default();
        // Only delays shorter than one read cancel it (the first run cancelled 247 of 975 attempts
        // over 0..=T+10), so the range ends just past T and every delay gets SWEEP_BLOCKS blocks,
        // each followed by verify().
        'sweep: for d in 0..=(t + SWEEP_PAST_T_US) {
            for _ in 0..SWEEP_BLOCKS {
                for _ in 0..BLOCK {
                    if self.check_time() {
                        break 'sweep;
                    }
                    if !self.cycle(d, &mut sw).await {
                        break 'sweep;
                    }
                }
                if !self.block_check(d, &mut sw).await {
                    break 'sweep;
                }
            }
        }
        self.print_sweep("phase 2", &sw);
        let ok = sw.recover_failures == 0
            && sw.wrong == 0
            && sw.verify_failures == 0
            && sw.cancelled >= 300
            && !self.time_up;
        check!(
            self.fails,
            ok,
            "{} phase 2: the three failure counters are 0 and at least 300 really cancelled ({})",
            sw.cancelled
        );
    }

    async fn phase3(&mut self, t: u32) {
        let mut sw = Sweep::default();
        let span = t + 11;
        let mut edge_timeouts = 0u32;
        for i in 0..PHASE3_CANCELS {
            if self.check_time() {
                break;
            }
            if with_timeout(Duration::from_millis(50), self.int1.wait_for_rising_edge())
                .await
                .is_err()
            {
                edge_timeouts += 1;
                if edge_timeouts > 20 {
                    break;
                }
                continue;
            }
            let d = i % span;
            if !self.cycle(d, &mut sw).await {
                break;
            }
            if i % BLOCK == BLOCK - 1 && !self.block_check(d, &mut sw).await {
                break;
            }
        }
        defmt::info!("[STAT] phase 3: data-ready edge timeouts {}", edge_timeouts);
        self.print_sweep("phase 3", &sw);
        let ok = sw.recover_failures == 0
            && sw.wrong == 0
            && sw.verify_failures == 0
            && sw.cancelled >= 150
            && edge_timeouts <= 20
            && !self.time_up;
        check!(
            self.fails,
            ok,
            "{} phase 3: the three failure counters are 0 and at least 150 really cancelled ({})",
            sw.cancelled
        );
    }

    /// One repair attempt after a sensor-side failure. True when the sensor answers again.
    async fn repair(&mut self) -> bool {
        Timer::after_millis(1).await;
        let r = self.imu.recover().await;
        defmt::info!("[INFO] repair: recover {}", defmt::Debug2Format(&r));
        if r.is_err() {
            return false;
        }
        let c = self.imu.configure(&mut Delay, &self.cfg).await;
        defmt::info!("[INFO] repair: configure {}", defmt::Debug2Format(&c));
        if c.is_err() {
            return false;
        }
        let v = self.imu.verify().await;
        defmt::info!("[INFO] repair: verify {}", defmt::Debug2Format(&v));
        v.is_ok()
    }

    async fn phase4(&mut self) {
        let init = self.imu.init(&mut Delay, &self.cfg).await;
        let verify = if init.is_ok() {
            self.imu.verify().await
        } else {
            Err(Error::NotConfigured)
        };
        let mut wrong = 0u32;
        if init.is_ok() && verify.is_ok() {
            for _ in 0..200 {
                let r = self.imu.read().await;
                wrong += self.wrong(&r) as u32;
            }
        } else {
            wrong = 200;
        }
        check!(
            self.fails,
            init.is_ok() && verify.is_ok() && wrong == 0,
            "{} sensor normal at the end: init {}, verify {}, wrong of 200 reads {}",
            defmt::Debug2Format(&init.err()),
            defmt::Debug2Format(&verify.err()),
            wrong
        );
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = SPI_HZ;
    let bus = Spi::new(
        p.SPI1, p.PIN_10, p.PIN_11, p.PIN_8, p.DMA_CH0, p.DMA_CH1, Irqs, spi_cfg,
    );
    let cs = Output::new(p.PIN_9, Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    let int1 = Input::new(p.PIN_7, Pull::None);

    let mut c = Ctx {
        imu: Lsm6dsk320x::new(dev),
        int1,
        cfg: Config::default(),
        reference: Reference {
            accel_mg: [0.0; 3],
            gyro_dps: [0.0; 3],
            temp_c: 0.0,
        },
        fails: 0,
        cancels: 0,
        sensor_fault: false,
        time_up: false,
        start: Instant::now(),
    };

    let t = c.phase0().await;
    check!(c.fails, t.is_some(), "{} phase 0: reference");
    if let Some(t) = t {
        let planned =
            PHASE1_CANCELS + (t + SWEEP_PAST_T_US + 1) * BLOCK * SWEEP_BLOCKS + PHASE3_CANCELS;
        if planned > MAX_CANCELS {
            check!(
                c.fails,
                false,
                "{} planned cancels {} exceed the limit of 2500, none made",
                planned
            );
        } else {
            defmt::info!(
                "[INFO] planned cancels at most {} of {}",
                planned,
                MAX_CANCELS
            );
            c.phase1(t).await;
            if !c.stopped() {
                c.phase2(t).await;
            }
            if !c.stopped() {
                c.phase3(t).await;
            }
        }
    }

    let mut bus_ok = true;
    if c.sensor_fault {
        bus_ok = c.repair().await;
        check!(
            c.fails,
            false,
            "{} repair attempt ended, run fails (repair ok: {})",
            bus_ok
        );
    }
    if bus_ok {
        c.phase4().await;
    } else {
        check!(
            c.fails,
            false,
            "{} sensor normal at the end: NO, repair did not bring it back, final init not attempted"
        );
    }
    defmt::info!("[STAT] total cancels {}", c.cancels);
    hil_common::exit(c.fails)
}

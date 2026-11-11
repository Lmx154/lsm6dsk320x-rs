//! On-board check of the user-offset registers of the lsm6dsk320x driver (U23 on SPI1, INT1 on
//! GPIO7): which offsets USR_OFF_W scales, the weights, the signs, and `set_offsets`, `configure`,
//! `init` and `recover` on the real part. The board lies still on the bench.
//!
//! No raw register access, no cancelled bus transfer. The run ends within 60 s and always puts the
//! offsets back to zero (`init`) and checks `verify()` before the verdict.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peripherals, bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::calc::Offsets;
use lsm6dsk320x::{
    AccelFullScale, Channel, Config, ConfigError, DataReady, Error, GyroFullScale, HighGConfig,
    HighGFullScale, HighGOdr, IntRouting, Lsm6dsk320x, Odr, OffsetWeight,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const ACCEL_FS: AccelFullScale = AccelFullScale::G4;
const GYRO_FS: GyroFullScale = GyroFullScale::Dps500;
const HIGH_G_FS: HighGFullScale = HighGFullScale::G32;
const DISCARD: u32 = 100;
const SAMPLES: u32 = 400;
const DEADLINE_S: u64 = 48;
/// Noise limits for "did not move", on the mean of 400 samples.
const LOW_G_NOISE_MG: f32 = 3.0;
const GYRO_NOISE_DPS: f32 = 0.5;
const HIGH_G_NOISE_MG: f32 = 20.0;
const NONE: usize = usize::MAX;

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

fn base_cfg() -> Config {
    Config::new()
        .with_accel_fs(ACCEL_FS)
        .with_accel_odr(Odr::Hz960)
        .with_gyro_fs(GYRO_FS)
        .with_gyro_odr(Odr::Hz960)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz960)
                .with_full_scale(HIGH_G_FS),
        ))
        .with_int1(IntRouting::new().with_data_ready(DataReady::new().with_accel(true)))
        .with_int2(IntRouting::new())
}

fn cfg_with(weight: Option<OffsetWeight>, hg_offset: bool) -> Config {
    base_cfg().with_accel_offset(weight).with_high_g(Some(
        HighGConfig::new()
            .with_odr(HighGOdr::Hz960)
            .with_full_scale(HIGH_G_FS)
            .with_user_offset(hg_offset),
    ))
}

fn low(v: [i8; 3]) -> Offsets {
    Offsets::new().with_low_g(v)
}

fn high(v: [i8; 3]) -> Offsets {
    Offsets::new().with_high_g(v)
}

fn axis(i: usize, v: i8) -> [i8; 3] {
    let mut a = [0; 3];
    a[i] = v;
    a
}

/// Means of the low-g axes (mg), the gyro axes (dps) and the high-g axes (mg), or their changes.
#[derive(Clone, Copy)]
struct M {
    lo: [f32; 3],
    gy: [f32; 3],
    hg: [f32; 3],
}

impl M {
    fn minus(&self, b: &M) -> M {
        let mut d = *self;
        for i in 0..3 {
            d.lo[i] -= b.lo[i];
            d.gy[i] -= b.gy[i];
            d.hg[i] -= b.hg[i];
        }
        d
    }

    /// True when every channel except low-g axis `skip_lo` and high-g axis `skip_hg` is quiet.
    fn quiet(&self, skip_lo: usize, skip_hg: usize) -> bool {
        (0..3).all(|i| {
            (i == skip_lo || abs(self.lo[i]) <= LOW_G_NOISE_MG)
                && abs(self.gy[i]) <= GYRO_NOISE_DPS
                && (i == skip_hg || abs(self.hg[i]) <= HIGH_G_NOISE_MG)
        })
    }
}

fn within(value: f32, nominal: f32, pct: f32) -> bool {
    abs(abs(value) - abs(nominal)) <= abs(nominal) * pct / 100.0
}

fn dir(v: f32) -> &'static str {
    if v > 0.0 { "raises" } else { "lowers" }
}

struct Rig<'a, S: SpiDevice> {
    imu: Lsm6dsk320x<S>,
    int1: Input<'a>,
    fails: u32,
    start: Instant,
    done: bool,
}

impl<S: SpiDevice> Rig<'_, S> {
    /// Mean of `SAMPLES` read_all samples on data-ready after `DISCARD` discarded ones.
    async fn meas(&mut self) -> Option<M> {
        if self.start.elapsed() > Duration::from_secs(DEADLINE_S) {
            defmt::error!("[INFO] deadline reached, remaining measurements skipped");
            return None;
        }
        let (mut lo, mut gy, mut hg) = ([0i32; 3], [0i32; 3], [0i32; 3]);
        for n in 0..DISCARD + SAMPLES {
            if with_timeout(Duration::from_millis(50), self.int1.wait_for_rising_edge())
                .await
                .is_err()
            {
                defmt::error!("[INFO] data-ready edge timeout");
                return None;
            }
            let all = match self.imu.read_all().await {
                Ok(a) => a,
                Err(e) => {
                    defmt::error!("[INFO] read_all: {}", defmt::Debug2Format(&e));
                    return None;
                }
            };
            if n < DISCARD {
                continue;
            }
            lo[0] += all.accel.x as i32;
            lo[1] += all.accel.y as i32;
            lo[2] += all.accel.z as i32;
            gy[0] += all.gyro.x as i32;
            gy[1] += all.gyro.y as i32;
            gy[2] += all.gyro.z as i32;
            hg[0] += all.high_g.x as i32;
            hg[1] += all.high_g.y as i32;
            hg[2] += all.high_g.z as i32;
        }
        let n = SAMPLES as f32;
        let (a, g, h) = (
            ACCEL_FS.mg_per_lsb(),
            GYRO_FS.mdps_per_lsb() * 1e-3,
            HIGH_G_FS.mg_per_lsb(),
        );
        Some(M {
            lo: lo.map(|v| v as f32 / n * a),
            gy: gy.map(|v| v as f32 / n * g),
            hg: hg.map(|v| v as f32 / n * h),
        })
    }

    async fn set(&mut self, o: &Offsets) -> bool {
        let r = self.imu.set_offsets(o).await;
        check!(
            self.fails,
            r.is_ok(),
            "{} set_offsets {} -> {}",
            *o,
            defmt::Debug2Format(&r)
        );
        r.is_ok()
    }

    async fn verify(&mut self, what: &str) {
        let r = self.imu.verify().await;
        check!(
            self.fails,
            r.is_ok(),
            "{} verify after {}: {}",
            what,
            defmt::Debug2Format(&r)
        );
    }

    /// `configure` (or `init`) with `c`.
    async fn apply(&mut self, c: &Config, init: bool) -> bool {
        let r = if init {
            self.imu.init(&mut Delay, c).await
        } else {
            self.imu.configure(&mut Delay, c).await
        };
        let what = if init { "init" } else { "configure" };
        check!(
            self.fails,
            r.is_ok(),
            "{} {}: {}",
            what,
            defmt::Debug2Format(&r.as_ref().err())
        );
        r.is_ok()
    }

    /// Sets `o`, measures and returns the change against `base`, printed.
    async fn step(&mut self, label: &str, o: &Offsets, base: &M) -> Option<M> {
        if !self.set(o).await {
            return None;
        }
        let m = self.meas().await?;
        let d = m.minus(base);
        defmt::info!(
            "[STAT] {}: dlow {} mg, dgyro {} dps, dhigh {} mg",
            label,
            d.lo,
            d.gy,
            d.hg
        );
        Some(d)
    }

    /// A zero-offset reference in the configuration just applied, and its drift from `base`.
    async fn reference(&mut self, label: &str, base: &M) -> Option<M> {
        let m = self.meas().await?;
        let d = m.minus(base);
        defmt::info!(
            "[STAT] {} reference drift from the c1 baseline: dlow {} mg, dgyro {} dps, dhigh {} mg",
            label,
            d.lo,
            d.gy,
            d.hg
        );
        Some(m)
    }

    async fn checks(&mut self) {
        let off_cfg = cfg_with(None, false);
        let fine = cfg_with(Some(OffsetWeight::Fine), false);
        let coarse = cfg_with(Some(OffsetWeight::Coarse), false);

        // 1. Baseline, registers alone do nothing.
        if !self.apply(&off_cfg, true).await {
            return;
        }
        self.verify("c1 init").await;
        let Some(base) = self.meas().await else {
            return;
        };
        defmt::info!(
            "[STAT] c1 baseline: low {} mg, gyro {} dps, high {} mg",
            base.lo,
            base.gy,
            base.hg
        );
        let set1 = Offsets::new()
            .with_low_g([64, -32, 16])
            .with_high_g([4, -4, 2]);
        let Some(d) = self
            .step("c1 registers set, not applied", &set1, &base)
            .await
        else {
            return;
        };
        check!(
            self.fails,
            d.quiet(NONE, NONE),
            "{} c1 means unchanged with offsets not applied (limits {} mg low, {} dps, {} mg high)",
            LOW_G_NOISE_MG,
            GYRO_NOISE_DPS,
            HIGH_G_NOISE_MG
        );
        self.verify("c1 set_offsets").await;
        check!(
            self.fails,
            self.imu.offsets() == Some(set1),
            "{} c1 offsets() returns what was set: {}",
            self.imu.offsets()
        );
        if !self.set(&Offsets::new()).await {
            return;
        }

        // 2. Low-g fine weight, +-64 per axis.
        if !self.apply(&fine, false).await {
            return;
        }
        let Some(ref2) = self.reference("c2", &base).await else {
            return;
        };
        for i in 0..3 {
            let (Some(dp), Some(dn), Some(dz)) = (
                self.step("c2 +64", &low(axis(i, 64)), &ref2).await,
                self.step("c2 -64", &low(axis(i, -64)), &ref2).await,
                self.step("c2 0", &Offsets::new(), &ref2).await,
            ) else {
                return;
            };
            defmt::info!(
                "[STAT] c2 low-g axis {}: +64 {} the output by {} mg, -64 {} it by {} mg",
                i,
                dir(dp.lo[i]),
                dp.lo[i],
                dir(dn.lo[i]),
                dn.lo[i]
            );
            check!(
                self.fails,
                within(dp.lo[i], 62.5, 10.0)
                    && within(dn.lo[i], 62.5, 10.0)
                    && dp.lo[i] * dn.lo[i] < 0.0
                    && dp.quiet(i, NONE)
                    && dn.quiet(i, NONE)
                    && dz.quiet(NONE, NONE),
                "{} c2 axis {}: +64 -> {} mg, -64 -> {} mg (62.5 +-10 %, opposite, others quiet, 0 returns)",
                i,
                dp.lo[i],
                dn.lo[i]
            );
        }
        self.verify("c2").await;

        // 3. Low-g coarse weight, +8 on X; the offsets survive configure.
        let o3 = low(axis(0, 8));
        if !self.set(&o3).await {
            return;
        }
        if !self.apply(&coarse, false).await {
            return;
        }
        check!(
            self.fails,
            self.imu.offsets() == Some(o3),
            "{} c3 offsets() after configure: {}",
            self.imu.offsets()
        );
        let Some(ref3) = self
            .reference("c3 (offset +8 loaded, not zero)", &base)
            .await
        else {
            return;
        };
        let _ = ref3;
        if !self.set(&Offsets::new()).await {
            return;
        }
        let Some(ref3) = self.reference("c3", &base).await else {
            return;
        };
        if !self.set(&o3).await {
            return;
        }
        let Some(m) = self.meas().await else {
            return;
        };
        let d = m.minus(&ref3);
        defmt::info!(
            "[STAT] c3 coarse +8 on X: dlow {} mg, dgyro {} dps, dhigh {} mg",
            d.lo,
            d.gy,
            d.hg
        );
        check!(
            self.fails,
            within(d.lo[0], 125.0, 10.0) && d.quiet(0, NONE),
            "{} c3 coarse +8 on X moves {} mg (125 +-10 %)",
            d.lo[0]
        );
        if let Some(d) = self.step("c3 0", &Offsets::new(), &ref3).await {
            check!(
                self.fails,
                d.quiet(NONE, NONE),
                "{} c3 0 returns to baseline"
            );
        }
        self.verify("c3").await;

        // 4. High-g offsets.
        let hg_fine = cfg_with(Some(OffsetWeight::Fine), true);
        let hg_coarse = cfg_with(Some(OffsetWeight::Coarse), true);
        if !self.apply(&hg_fine, false).await {
            return;
        }
        let Some(ref4) = self.reference("c4", &base).await else {
            return;
        };
        let mut x_plus = 0.0;
        for i in 0..3 {
            let (Some(dp), Some(dn), Some(dz)) = (
                self.step("c4 +4", &high(axis(i, 4)), &ref4).await,
                self.step("c4 -4", &high(axis(i, -4)), &ref4).await,
                self.step("c4 0", &Offsets::new(), &ref4).await,
            ) else {
                return;
            };
            if i == 0 {
                x_plus = dp.hg[0];
            }
            let nominal = lsm6dsk320x::calc::high_g_offset_mg(1, HIGH_G_FS) as f32;
            defmt::info!(
                "[STAT] c4 high-g axis {}: +4 {} the output by {} mg = {} mg/LSB, -4 {} by {} mg = {} mg/LSB (crate {})",
                i,
                dir(dp.hg[i]),
                dp.hg[i],
                dp.hg[i] / 4.0,
                dir(dn.hg[i]),
                dn.hg[i],
                dn.hg[i] / -4.0,
                nominal
            );
            check!(
                self.fails,
                within(dp.hg[i], 4.0 * nominal, 15.0)
                    && within(dn.hg[i], 4.0 * nominal, 15.0)
                    && dp.hg[i] * dn.hg[i] < 0.0
                    && dp.quiet(NONE, i)
                    && dn.quiet(NONE, i)
                    && dz.quiet(NONE, NONE),
                "{} c4 axis {}: +4 -> {} mg, -4 -> {} mg (crate weight, +-15 %)",
                i,
                dp.hg[i],
                dn.hg[i]
            );
        }
        if !self.apply(&hg_coarse, false).await {
            return;
        }
        let Some(ref4c) = self.reference("c4 coarse", &base).await else {
            return;
        };
        if let Some(d) = self
            .step("c4 coarse bit, +4 on X", &high(axis(0, 4)), &ref4c)
            .await
        {
            defmt::info!(
                "[STAT] c4 high-g X +4: fine bit {} mg, coarse bit {} mg",
                x_plus,
                d.hg[0]
            );
            check!(
                self.fails,
                within(d.hg[0], x_plus, 10.0) && d.hg[0] * x_plus > 0.0,
                "{} c4 USR_OFF_W does not change the high-g offset ({} mg fine, {} mg coarse)",
                x_plus,
                d.hg[0]
            );
        }
        if !self.set(&Offsets::new()).await {
            return;
        }
        self.verify("c4").await;

        // 5. Field edges on low-g X with the fine weight.
        if !self.apply(&fine, false).await {
            return;
        }
        let Some(ref5) = self.reference("c5", &base).await else {
            return;
        };
        if let (Some(dp), Some(dn)) = (
            self.step("c5 +127", &low(axis(0, 127)), &ref5).await,
            self.step("c5 -127", &low(axis(0, -127)), &ref5).await,
        ) {
            check!(
                self.fails,
                within(dp.lo[0], 124.0, 10.0)
                    && within(dn.lo[0], 124.0, 10.0)
                    && dp.lo[0] * dn.lo[0] < 0.0
                    && dp.quiet(0, NONE)
                    && dn.quiet(0, NONE),
                "{} c5 +127 -> {} mg, -127 -> {} mg (124 +-10 %, opposite)",
                dp.lo[0],
                dn.lo[0]
            );
        }
        if !self.set(&Offsets::new()).await {
            return;
        }
        let stored = self.imu.offsets();
        let r = self.imu.set_offsets(&low(axis(0, -128))).await;
        check!(
            self.fails,
            matches!(
                r,
                Err(Error::InvalidConfig(ConfigError::OffsetOutOfRange {
                    channel: Channel::LowG,
                    ..
                }))
            ) && self.imu.offsets() == stored,
            "{} c5 set_offsets(-128) -> {}, offsets() unchanged: {}",
            defmt::Debug2Format(&r),
            self.imu.offsets() == stored
        );
        self.verify("c5").await;

        // 6. configure keeps the offsets, init clears them.
        let o6 = low(axis(0, 64));
        if !self.apply(&fine, false).await {
            return;
        }
        let Some(ref6) = self.reference("c6", &base).await else {
            return;
        };
        if !self.set(&o6).await {
            return;
        }
        if !self.apply(&fine, false).await {
            return;
        }
        check!(
            self.fails,
            self.imu.offsets() == Some(o6),
            "{} c6 offsets() after configure: {}",
            self.imu.offsets()
        );
        if let Some(m) = self.meas().await {
            let d = m.minus(&ref6);
            defmt::info!(
                "[STAT] c6 after configure: dlow {} mg, dgyro {} dps, dhigh {} mg",
                d.lo,
                d.gy,
                d.hg
            );
            check!(
                self.fails,
                within(d.lo[0], 62.5, 10.0) && d.quiet(0, NONE),
                "{} c6 offset still acts after configure: {} mg",
                d.lo[0]
            );
        }
        self.verify("c6 configure").await;
        if !self.apply(&fine, true).await {
            return;
        }
        check!(
            self.fails,
            self.imu.offsets() == Some(Offsets::new()),
            "{} c6 offsets() after init: {}",
            self.imu.offsets()
        );
        if let Some(m) = self.meas().await {
            let d = m.minus(&ref6);
            defmt::info!(
                "[STAT] c6 after init: dlow {} mg, dgyro {} dps, dhigh {} mg",
                d.lo,
                d.gy,
                d.hg
            );
            check!(
                self.fails,
                d.quiet(NONE, NONE),
                "{} c6 offset is gone after init"
            );
        }
        self.verify("c6 init").await;

        // Control: the offset block off again, against the first baseline (drift over the run).
        if self.apply(&off_cfg, true).await {
            let _ = self.reference("control, offset block off", &base).await;
        }

        // 7. recover on a healthy bus.
        let t = Instant::now();
        let r = self.imu.recover().await;
        let us = t.elapsed().as_micros();
        check!(
            self.fails,
            r.is_ok(),
            "{} c7 recover: {} in {} us",
            defmt::Debug2Format(&r),
            us
        );
        self.verify("c7 recover").await;
        self.done = true;
    }

    /// Offsets back to zero and the configuration known, on every path.
    async fn cleanup(&mut self) {
        let c = cfg_with(None, false);
        let mut ok = self.imu.init(&mut Delay, &c).await.is_ok();
        if !ok {
            let _ = self.imu.recover().await;
            ok = self.imu.init(&mut Delay, &c).await.is_ok();
        }
        check!(self.fails, ok, "{} cleanup init");
        check!(
            self.fails,
            self.imu.offsets() == Some(Offsets::new()),
            "{} cleanup offsets() zero: {}",
            self.imu.offsets()
        );
        self.verify("cleanup").await;
    }
}

async fn run(p: &mut Peripherals) -> u32 {
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
    let mut rig = Rig {
        imu: Lsm6dsk320x::new(dev),
        int1: Input::new(p.PIN_7.reborrow(), Pull::None),
        fails: 0,
        start: Instant::now(),
        done: false,
    };
    rig.checks().await;
    check!(rig.fails, rig.done, "{} all seven checks ran to the end");
    rig.cleanup().await;
    let s = rig.start.elapsed().as_secs();
    defmt::info!("[STAT] run time {} s", s);
    check!(rig.fails, s < 60, "{} run ends within 60 s");
    rig.fails
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let fails = run(&mut p).await;
    hil_common::exit(fails)
}

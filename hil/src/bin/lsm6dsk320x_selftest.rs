//! On-board check of the built-in self-test of the lsm6dsk320x driver (U23 on SPI1). The board
//! lies still on the bench.
//!
//! 1. `self_test` with the defaults, five runs. 2. Negative stimulus and each channel alone.
//! 3. Settling time of the stimulus, per channel (one raw write on, one off, only in this check).
//! 4. Margins to the datasheet windows. 5. Shortened settings. 6. Nothing left behind.
//!
//! No driver call is cancelled. The run ends within 90 s with `init` to the default configuration.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peripherals, bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::{
    AccelFullScale, Channel, Config, Error, GyroFullScale, HighGConfig, HighGFullScale, HighGOdr,
    IntRouting, Lsm6dsk320x, Odr, RawAxes, SelfTestConfig, Stimulus, calc, reg,
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
const LATE_S: u64 = 72;

const CH: [Channel; 3] = [Channel::LowG, Channel::Gyro, Channel::HighG];
const NAME: [&str; 3] = ["lowg", "gyro", "highg"];
const UNIT: [&str; 3] = ["ug", "mdps", "ug"];
const AX: [&str; 3] = ["X", "Y", "Z"];
/// Datasheet windows in ug, mdps, ug (the driver's own constants, converted).
const WIN: [(u32, u32); 3] = [
    (
        calc::SELF_TEST_LOW_G_MG.min * 1000,
        calc::SELF_TEST_LOW_G_MG.max * 1000,
    ),
    (
        calc::SELF_TEST_GYRO_250_DPS_MDPS.min,
        calc::SELF_TEST_GYRO_250_DPS_MDPS.max,
    ),
    (
        calc::SELF_TEST_HIGH_G_MG.min * 1000,
        calc::SELF_TEST_HIGH_G_MG.max * 1000,
    ),
];
const MARKS_MS: [u32; 7] = [5, 10, 20, 50, 100, 200, 300];
const BUF: usize = 192;

type Smp = (u32, [i32; 3]);

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
}

/// The nearest `Config` to the driver's sequence of channel `ci` (all else off).
fn channel_cfg(ci: usize) -> Config {
    let c = Config::new()
        .with_int1(IntRouting::new())
        .with_int2(IntRouting::new())
        .with_drdy_mask(false)
        .with_accel_odr(Odr::Off)
        .with_gyro_odr(Odr::Off)
        .with_high_g(None);
    match ci {
        0 => c.with_accel_fs(AccelFullScale::G4).with_accel_odr(Odr::Hz60),
        1 => c
            .with_gyro_fs(GyroFullScale::Dps250)
            .with_gyro_odr(Odr::Hz240),
        _ => c.with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz480)
                .with_full_scale(HighGFullScale::G32),
        )),
    }
}

fn phys(ci: usize, raw: i16) -> i32 {
    let r = raw as i32;
    match ci {
        0 => r * 122,
        1 => r * 35 / 4,
        _ => r * 976,
    }
}

fn selected(t: &SelfTestConfig, ci: usize) -> bool {
    [t.low_g, t.gyro, t.high_g][ci]
}

fn tail_mean(s: &[Smp], a: usize) -> i32 {
    let k = (s.len() / 5).clamp(3, 20).min(s.len());
    if k == 0 {
        return 0;
    }
    let sum: i64 = s[s.len() - k..].iter().map(|x| x.1[a] as i64).sum();
    (sum / k as i64) as i32
}

/// Time of the first sample after the last one outside the band around `fin`; 0 when none is
/// outside, `u32::MAX` when the last sample is.
fn settle_us(s: &[Smp], a: usize, fin: i32, band: i32, smooth: bool) -> u32 {
    let mut last_bad: Option<usize> = None;
    for i in 0..s.len() {
        let v = if smooth {
            let lo = i.saturating_sub(2);
            let sum: i64 = s[lo..=i].iter().map(|x| x.1[a] as i64).sum();
            (sum / (i - lo + 1) as i64) as i32
        } else {
            s[i].1[a]
        };
        if (v - fin).abs() > band {
            last_bad = Some(i);
        }
    }
    match last_bad {
        None => 0,
        Some(i) if i + 1 < s.len() => s[i + 1].0,
        Some(_) => u32::MAX,
    }
}

/// Width of the windows the verdict's settle time is measured in.
const BLOCK_US: u32 = 50_000;

/// Start of the first `BLOCK_US` window from which the mean of every later window stays inside the
/// band around `fin`; 0 when every window is inside, `u32::MAX` when the last one is not. A single
/// noisy sample cannot move this (the high-g channel's sample noise is as large as 2 % of its
/// step), which the per-sample measures above cannot claim.
fn settle_block_us(s: &[Smp], a: usize, fin: i32, band: i32) -> u32 {
    let mut last_bad: Option<u32> = None;
    let mut last_block = 0u32;
    let mut i = 0usize;
    while i < s.len() {
        let block = s[i].0 / BLOCK_US;
        let (mut sum, mut n) = (0i64, 0i64);
        while i < s.len() && s[i].0 / BLOCK_US == block {
            sum += s[i].1[a] as i64;
            n += 1;
            i += 1;
        }
        if ((sum / n) as i32 - fin).abs() > band {
            last_bad = Some(block);
        }
        last_block = block;
    }
    match last_bad {
        None => 0,
        Some(b) if b < last_block => (b + 1) * BLOCK_US,
        Some(_) => u32::MAX,
    }
}

/// Prints the samples at the marks and returns (raw settle, smoothed settle, noise-aware settle,
/// settle of the 50 ms window means) in us.
fn analyse(
    ci: usize,
    phase: &str,
    a: usize,
    s: &[Smp],
    init: i32,
    fin: i32,
) -> (u32, u32, u32, u32) {
    let step = fin - init;
    let band = (step.abs() / 50).max(1);
    let mut vals = [0i32; 7];
    for (m, mark) in MARKS_MS.iter().enumerate() {
        let mut best = 0usize;
        let mut bd = u32::MAX;
        for (i, x) in s.iter().enumerate() {
            let d = x.0.abs_diff(mark * 1000);
            if d < bd {
                bd = d;
                best = i;
            }
        }
        vals[m] = s.get(best).map_or(0, |x| x.1[a]);
    }
    let raw = settle_us(s, a, fin, band, false);
    let sm = settle_us(s, a, fin, band, true);
    let (mut sq, mut cnt) = (0i64, 0i64);
    for x in s.iter().filter(|x| x.0 >= 200_000) {
        let d = (x.1[a] - fin) as i64;
        sq += d * d;
        cnt += 1;
    }
    let rms = if cnt > 0 { sqrt((sq / cnt) as f32) } else { 0.0 };
    let nband = band.max((3.0 * rms) as i32);
    let nb = settle_us(s, a, fin, nband, false);
    // Window means of at least three samples: their noise is at most rms / sqrt(3), so 2 rms is a
    // 3.5 sigma band.
    let bband = band.max((2.0 * rms) as i32);
    let blk = settle_block_us(s, a, fin, bband);
    defmt::info!(
        "[STAT] settle {} {} {}: 50 ms window means inside max(2%, 2 rms) = {} {} from {} us on",
        NAME[ci],
        phase,
        AX[a],
        bband,
        UNIT[ci],
        blk
    );
    defmt::info!(
        "[STAT] settle {} {} {}: from {} to {} {} (step {}); at 5/10/20/50/100/200/300 ms {}; within 2% ({} {}) after: single sample {} us, 3-sample mean {} us; noise rms {} over t>=200 ms, band max(2%, 3 rms) {}: single sample {} us",
        NAME[ci],
        phase,
        AX[a],
        init,
        fin,
        UNIT[ci],
        step,
        vals,
        band,
        UNIT[ci],
        raw,
        sm,
        rms,
        nband,
        nb
    );
    (raw, sm, nb, blk)
}

fn marks_line(ci: usize, phase: &str, s: &[Smp]) {
    let mut t = [0u32; 7];
    for (m, mark) in MARKS_MS.iter().enumerate() {
        let mut bd = u32::MAX;
        for x in s {
            let d = x.0.abs_diff(mark * 1000);
            if d < bd {
                bd = d;
                t[m] = x.0;
            }
        }
    }
    let period = if s.len() > 1 {
        (s[s.len() - 1].0 - s[0].0) / (s.len() as u32 - 1)
    } else {
        0
    };
    defmt::info!(
        "[STAT] settle {} {}: {} samples, mean period {} us, times of the sample nearest the marks {} us",
        NAME[ci],
        phase,
        s.len() as u32,
        period,
        t
    );
}

#[derive(Clone, Copy)]
struct Acc {
    n: u32,
    sum: i64,
    min: u32,
    max: u32,
}

impl Acc {
    const NEW: Acc = Acc {
        n: 0,
        sum: 0,
        min: u32::MAX,
        max: 0,
    };
    fn add(&mut self, signed: i32) {
        self.n += 1;
        self.sum += signed as i64;
        let m = signed.unsigned_abs();
        self.min = self.min.min(m);
        self.max = self.max.max(m);
    }
    fn mean(&self) -> i32 {
        if self.n == 0 {
            0
        } else {
            (self.sum / self.n as i64) as i32
        }
    }
}

struct Rig<S: SpiDevice> {
    imu: Lsm6dsk320x<S>,
    fails: u32,
    start: Instant,
    bus_errs: u32,
    /// [sign: 0 positive, 1 negative][channel][axis]
    acc: [[[Acc; 3]; 3]; 2],
    call_us: (u64, u64),
}

impl<S: SpiDevice> Rig<S> {
    fn too_late(&self) -> bool {
        self.start.elapsed().as_secs() > LATE_S || self.bus_errs > 20
    }

    async fn restore(&mut self) {
        let c = base_cfg();
        if self.imu.configure(&mut Delay, &c).await.is_err() {
            let _ = self.imu.recover().await;
            if self.imu.init(&mut Delay, &c).await.is_err() {
                self.bus_errs += 5;
            }
        }
    }

    async fn verify(&mut self, label: &str) {
        let r = self.imu.verify().await;
        check!(
            self.fails,
            r.is_ok(),
            "{} {} verify: {}",
            label,
            defmt::Debug2Format(&r)
        );
    }

    /// `n` read_all samples at rest: means within the smoke test's plausibility limits.
    async fn plausible(&mut self, label: &str, n: u32) {
        let (mut gyro, mut accel, mut hg) = ([0f32; 3], [0f32; 3], [0f32; 3]);
        let mut errors = 0u32;
        for _ in 0..n {
            match self.imu.read_all().await {
                Ok(all) => {
                    let g = all.gyro.gyro_rads(GYRO_FS);
                    let a = all.accel.accel_ms2(ACCEL_FS);
                    let h = all.high_g.high_g_ms2(HIGH_G_FS);
                    for i in 0..3 {
                        gyro[i] += g[i];
                        accel[i] += a[i];
                        hg[i] += h[i];
                    }
                }
                Err(_) => errors += 1,
            }
            Timer::after_micros(1100).await;
        }
        let k = (n - errors).max(1) as f32;
        let gd = gyro.map(|v| v * DEG / k);
        let a = sqrt(accel.iter().map(|v| (v / k) * (v / k)).sum()) / G;
        let h = sqrt(hg.iter().map(|v| (v / k) * (v / k)).sum()) / G;
        let ok = errors == 0
            && (0.9..=1.1).contains(&a)
            && (0.5..=1.5).contains(&h)
            && gd.iter().all(|v| abs(*v) <= 5.0);
        check!(
            self.fails,
            ok,
            "{} {} {} samples at rest: |a_lowg| {} g, |a_highg| {} g, gyro {} dps, bus errors {}",
            label,
            n,
            a,
            h,
            gd,
            errors
        );
    }

    /// One `self_test` with everything around it. `enforce`: window result is a PASS/FAIL line,
    /// otherwise statistics only. `rec`: sign index to record the differences under.
    async fn st_run(&mut self, label: &str, t: &SelfTestConfig, enforce: bool, rec: Option<usize>) {
        if self.too_late() {
            check!(self.fails, false, "{} {} skipped: run deadline or bus failures", label);
            return;
        }
        let before = self.imu.config().copied();
        let t0 = Instant::now();
        let r = self.imu.self_test(&mut Delay, t).await;
        let us = t0.elapsed().as_micros();
        let rep = match r {
            Ok(rep) => rep,
            Err(e) => {
                if matches!(e, Error::Bus(_)) {
                    self.bus_errs += 1;
                }
                check!(
                    self.fails,
                    false,
                    "{} {} self_test error {}",
                    label,
                    defmt::Debug2Format(&e)
                );
                self.restore().await;
                return;
            }
        };
        defmt::info!("[STAT] {} self_test took {} us", label, us);
        self.call_us = (self.call_us.0.min(us), self.call_us.1.max(us));
        let mut all = true;
        for ci in 0..3 {
            if !selected(t, ci) {
                if rep.channel(CH[ci]).is_some() {
                    all = false;
                    defmt::info!("[STAT] {} {} reported but not selected", label, NAME[ci]);
                }
                continue;
            }
            let Some(res) = rep.channel(CH[ci]) else {
                all = false;
                defmt::info!("[STAT] {} {} missing from the report", label, NAME[ci]);
                continue;
            };
            for a in 0..3 {
                let x = res.axes[a];
                let signed = x.stimulated - x.baseline;
                if !x.pass {
                    all = false;
                }
                if let Some(s) = rec {
                    self.acc[s][ci][a].add(signed);
                }
                defmt::info!(
                    "[STAT] {} {} {}: baseline {} stimulated {} diff {} ({} {}) window {}..{} {}",
                    label,
                    NAME[ci],
                    AX[a],
                    x.baseline,
                    x.stimulated,
                    signed,
                    x.difference,
                    UNIT[ci],
                    WIN[ci].0,
                    WIN[ci].1,
                    if x.pass { "pass" } else { "FAIL" }
                );
            }
        }
        if enforce {
            check!(
                self.fails,
                all && rep.passed(),
                "{} {} every selected axis inside its window",
                label
            );
        } else {
            defmt::info!("[STAT] {} windows all met: {}", label, all);
        }
        self.verify(label).await;
        let same = self.imu.config().copied() == before && before.is_some();
        check!(self.fails, same, "{} {} config() is the one applied before", label);
        self.plausible(label, 100).await;
    }

    async fn stim(&mut self, ci: usize, on: bool) -> Option<Instant> {
        let code = if on { reg::SELF_TEST_POSITIVE } else { 0 };
        if ci == 2 {
            let t0 = Instant::now();
            let r = self
                .imu
                .write_register(reg::CTRL2_XL_HG, code << reg::CTRL2_XL_HG_ST_SHIFT)
                .await;
            return r.ok().map(|_| t0);
        }
        let (mask, shift) = if ci == 0 {
            (reg::CTRL10_ST_XL_MASK, reg::CTRL10_ST_XL_SHIFT)
        } else {
            (reg::CTRL10_ST_G_MASK, reg::CTRL10_ST_G_SHIFT)
        };
        let cur = self.imu.read_register(reg::CTRL10).await.ok()?;
        let t0 = Instant::now();
        let r = self
            .imu
            .write_register(reg::CTRL10, (cur & !mask) | (code << shift))
            .await;
        r.ok().map(|_| t0)
    }

    /// Every data-ready sample of channel `ci` for `ms` milliseconds after `t0`.
    async fn window(&mut self, ci: usize, t0: Instant, ms: u64, buf: &mut [Smp; BUF]) -> usize {
        let (bit, first) = match ci {
            0 => (reg::STATUS_XLDA, reg::OUTX_L_A),
            1 => (reg::STATUS_GDA, reg::OUTX_L_G),
            _ => (reg::STATUS_XLHGDA, reg::UI_OUTX_L_A_OIS_HG),
        };
        let end = t0 + Duration::from_millis(ms);
        let mut n = 0;
        while Instant::now() < end && n < BUF {
            let st = match self.imu.read_register(reg::STATUS_REG).await {
                Ok(v) => v,
                Err(_) => {
                    self.bus_errs += 1;
                    break;
                }
            };
            if st & bit == 0 {
                continue;
            }
            let t = t0.elapsed().as_micros() as u32;
            let mut b = [0u8; 6];
            if self.imu.read_registers(first, &mut b).await.is_err() {
                self.bus_errs += 1;
                break;
            }
            let r = RawAxes::from_bytes(&b);
            buf[n] = (t, [phys(ci, r.x), phys(ci, r.y), phys(ci, r.z)]);
            n += 1;
        }
        n
    }

    async fn settling(&mut self, ci: usize) {
        if self.too_late() {
            check!(self.fails, false, "{} settle {} skipped: run deadline", NAME[ci]);
            return;
        }
        let cfg = channel_cfg(ci);
        if let Err(e) = self.imu.configure(&mut Delay, &cfg).await {
            check!(
                self.fails,
                false,
                "{} settle {} configure: {}",
                NAME[ci],
                defmt::Debug2Format(&e)
            );
            self.restore().await;
            return;
        }
        Timer::after_millis(200).await;
        let mut pre = [(0u32, [0i32; 3]); BUF];
        let mut on = [(0u32, [0i32; 3]); BUF];
        let mut off = [(0u32, [0i32; 3]); BUF];
        let t = Instant::now();
        let npre = self.window(ci, t, 150, &mut pre).await;
        let mut ok = true;
        let non = match self.stim(ci, true).await {
            Some(t0) => self.window(ci, t0, 300, &mut on).await,
            None => {
                ok = false;
                0
            }
        };
        let noff = match self.stim(ci, false).await {
            Some(t0) => self.window(ci, t0, 300, &mut off).await,
            None => {
                ok = false;
                0
            }
        };
        // Whatever happened, the stimulus is off and the device restored before anything else.
        let _ = self.stim(ci, false).await;
        self.restore().await;
        let (pre, on, off) = (&pre[..npre], &on[..non], &off[..noff]);
        defmt::info!(
            "[STAT] settle {}: samples pre {} on {} off {}",
            NAME[ci],
            npre as u32,
            non as u32,
            noff as u32
        );
        marks_line(ci, "on", on);
        marks_line(ci, "off", off);
        let mut worst = 0u32;
        let mut worst_nb = 0u32;
        let mut worst_blk = 0u32;
        for a in 0..3 {
            let init = tail_mean(pre, a);
            let fon = tail_mean(on, a);
            let foff = tail_mean(off, a);
            let (_, s_on, n_on, b_on) = analyse(ci, "ON", a, on, init, fon);
            let (_, s_off, n_off, b_off) = analyse(ci, "OFF", a, off, fon, foff);
            worst = worst.max(s_on).max(s_off);
            worst_nb = worst_nb.max(n_on).max(n_off);
            worst_blk = worst_blk.max(b_on).max(b_off);
        }
        defmt::info!(
            "[STAT] settle {}: worst noise-aware (band max(2%, 3 rms), single sample) settle over axes and both edges {} us",
            NAME[ci],
            worst_nb
        );
        check!(
            self.fails,
            ok && npre > 3 && non > 3 && noff > 3 && worst_blk <= 100_000,
            "{} settle {}: 50 ms window means settled by 100 ms on every axis and both edges (worst {} us; worst 3-sample-mean within 2% {} us, statistics only: it trips on single noisy samples)",
            NAME[ci],
            worst_blk,
            worst
        );
        self.verify("settle restore").await;
    }

    fn margins(&self) {
        for ci in 0..3 {
            for a in 0..3 {
                let p = self.acc[0][ci][a];
                let n = self.acc[1][ci][a];
                let (lo, hi) = WIN[ci];
                let min = p.min.min(n.min);
                let max = p.max.max(n.max);
                let m_lo = (min as f32 - lo as f32) / lo as f32 * 100.0;
                let m_hi = (hi as f32 - max as f32) / hi as f32 * 100.0;
                let ratio = if p.mean() != 0 {
                    abs(n.mean() as f32 / p.mean() as f32) * 100.0
                } else {
                    0.0
                };
                defmt::info!(
                    "[STAT] margin {} {}: positive mean {} (n {}, |min| {}, |max| {}), negative mean {} (n {}, |min| {}, |max| {}) {}; |neg|/|pos| {}%; window {}..{}; distance to lower edge {}%, to upper edge {}%, smallest {}%",
                    NAME[ci],
                    AX[a],
                    p.mean(),
                    p.n,
                    p.min,
                    p.max,
                    n.mean(),
                    n.n,
                    n.min,
                    n.max,
                    UNIT[ci],
                    ratio,
                    lo,
                    hi,
                    m_lo,
                    m_hi,
                    if m_lo < m_hi { m_lo } else { m_hi }
                );
            }
        }
    }

    async fn checks(&mut self) {
        let c = base_cfg();
        match self.imu.init(&mut Delay, &c).await {
            Ok(_) => check!(self.fails, true, "{} init"),
            Err(e) => {
                check!(self.fails, false, "{} init: {}", defmt::Debug2Format(&e));
                self.bus_errs += 100;
                return;
            }
        }
        self.verify("start").await;
        self.plausible("start", 100).await;

        // 1.
        let t = SelfTestConfig::new();
        for i in 0..5 {
            let l = ["c1 run1", "c1 run2", "c1 run3", "c1 run4", "c1 run5"][i];
            self.st_run(l, &t, true, Some(0)).await;
        }
        // 2.
        let neg = SelfTestConfig::new().with_stimulus(Stimulus::Negative);
        self.st_run("c2 neg1", &neg, true, Some(1)).await;
        self.st_run("c2 neg2", &neg, true, Some(1)).await;
        let alone = [
            ("c2 alone lowg", SelfTestConfig::new().with_gyro(false).with_high_g(false)),
            ("c2 alone gyro", SelfTestConfig::new().with_low_g(false).with_high_g(false)),
            ("c2 alone highg", SelfTestConfig::new().with_low_g(false).with_gyro(false)),
        ];
        for (l, tc) in alone {
            self.st_run(l, &tc, true, Some(0)).await;
        }
        // 3.
        for ci in 0..3 {
            self.settling(ci).await;
        }
        // 4.
        self.margins();
        defmt::info!(
            "[STAT] self_test call time: shortest {} us, longest {} us",
            self.call_us.0,
            self.call_us.1
        );
        // 5.
        let d1 = SelfTestConfig::new().with_settle_ms(10).with_samples(5);
        let d2 = SelfTestConfig::new().with_settle_ms(100).with_samples(1);
        self.st_run("c5 settle10 samples5", &d1, false, None).await;
        self.st_run("c5 settle100 samples1", &d2, false, None).await;
        for ci in 0..3 {
            for a in 0..3 {
                defmt::info!(
                    "[STAT] c5 reference {} {}: default-run mean difference {} {}",
                    NAME[ci],
                    AX[a],
                    self.acc[0][ci][a].mean(),
                    UNIT[ci]
                );
            }
        }
        // 6.
        self.verify("c6 after all").await;
        match self.imu.read_register(reg::CTRL10).await {
            Ok(cur) => {
                let w = (cur & !reg::CTRL10_ST_XL_MASK) | reg::SELF_TEST_POSITIVE;
                let wr = self.imu.write_register(reg::CTRL10, w).await;
                check!(self.fails, wr.is_ok(), "{} c6 raw write of ST_XL: {}", defmt::Debug2Format(&wr));
            }
            Err(_) => {
                self.bus_errs += 1;
                check!(self.fails, false, "{} c6 read of CTRL10");
            }
        }
        let r = self.imu.verify().await;
        let named = match r {
            Err(Error::ConfigMismatch(m)) => {
                defmt::info!(
                    "[STAT] c6 mismatch register {} mask {} expected {} found {}",
                    m.register,
                    m.mask,
                    m.expected,
                    m.found
                );
                m.register == reg::CTRL10
            }
            _ => false,
        };
        check!(
            self.fails,
            named,
            "{} c6 verify names CTRL10 (0x19) with ST_XL set: {}",
            defmt::Debug2Format(&r)
        );
        self.restore().await;
        self.verify("c6 after configure").await;
    }

    async fn finish(&mut self) {
        let c = base_cfg();
        let mut ok = self.imu.init(&mut Delay, &c).await.is_ok();
        if !ok {
            let _ = self.imu.recover().await;
            ok = self.imu.init(&mut Delay, &c).await.is_ok();
        }
        check!(self.fails, ok, "{} final init");
        self.verify("final").await;
        self.plausible("final", 200).await;
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
        fails: 0,
        start: Instant::now(),
        bus_errs: 0,
        acc: [[[Acc::NEW; 3]; 3]; 2],
        call_us: (u64::MAX, 0),
    };
    rig.checks().await;
    rig.finish().await;
    let s = rig.start.elapsed().as_millis();
    defmt::info!("[STAT] run time {} ms", s as u32);
    check!(rig.fails, s < 90_000, "{} run ends within 90 s");
    rig.fails
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let fails = run(&mut p).await;
    hil_common::exit(fails)
}

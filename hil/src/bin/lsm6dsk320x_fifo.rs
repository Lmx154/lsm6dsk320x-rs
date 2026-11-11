//! On-board check of the lsm6dsk320x FIFO and `verify()` (U23 on SPI1, INT1 on GPIO7).
//!
//! Settles on the real sensor what the host tests could only assume: the burst over many FIFO
//! words, the layouts of the temperature, timestamp and configuration-change words, the FIFO
//! depth, overrun and flush, the watermark interrupt path, high-g words, and `verify()`.
//! The board lies still on the bench. No SPI transfer is ever cancelled: the only timeout wraps
//! the wait for an INT1 edge.
#![no_std]
#![no_main]

use core::fmt::Debug;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::fifo::timestamp_delta_ticks;
use lsm6dsk320x::sample::temperature_c;
use lsm6dsk320x::{
    AccelFullScale, BatchRate, Config, Error, FifoConfig, FifoFrame, FifoMode, GyroFullScale,
    HighGConfig, HighGFullScale, HighGOdr, Info, IntRouting, Lsm6dsk320x, Odr, TempBatchRate,
    TimestampBatch, calc, fifo_frames, reg,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const G: f32 = lsm6dsk320x::sample::G;
const DEG: f32 = 180.0 / core::f32::consts::PI;
const WORD: usize = 7;
const BUF_WORDS: usize = 260;
const RUN_LIMIT_S: u64 = 80;

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

/// Unwraps a driver result; on error logs a FAIL line and returns from the calling check.
macro_rules! need {
    ($fails:expr, $what:literal, $e:expr) => {
        match $e {
            Ok(v) => v,
            Err(e) => {
                check!($fails, false, "{} {} failed: {}", $what, defmt::Debug2Format(&e));
                return;
            }
        }
    };
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

fn mean3(s: [f32; 3], n: u32) -> [f32; 3] {
    let n = n.max(1) as f32;
    [s[0] / n, s[1] / n, s[2] / n]
}

fn add3(s: &mut [f32; 3], v: [f32; 3]) {
    for i in 0..3 {
        s[i] += v[i];
    }
}

// ---------------------------------------------------------------------------------------------
// Configurations
// ---------------------------------------------------------------------------------------------

fn cfg_a() -> Config {
    Config::new()
}

fn cfg_b() -> Config {
    Config::new().with_high_g(Some(
        HighGConfig::new()
            .with_odr(HighGOdr::Hz7680)
            .with_full_scale(HighGFullScale::G32),
    ))
}

fn fifo_c() -> FifoConfig {
    FifoConfig::new()
        .with_mode(FifoMode::Continuous)
        .with_watermark(28)
        .with_gyro(BatchRate::Hz960)
        .with_accel(BatchRate::Hz960)
        .with_temperature(TempBatchRate::Hz60)
        .with_timestamp(TimestampBatch::Dec8)
}

fn cfg_c() -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_accel_odr(Odr::Hz960)
        .with_gyro_odr(Odr::Hz960)
        .with_timestamp(true)
        .with_fifo(fifo_c())
        .with_int1(IntRouting::new().with_fifo_threshold(true))
        .with_int2(IntRouting::new())
}

fn cfg_high_g(with_low: bool) -> Config {
    let rate = if with_low {
        BatchRate::Hz960
    } else {
        BatchRate::Off
    };
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz960)
                .with_full_scale(HighGFullScale::G320),
        ))
        .with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Continuous)
                .with_gyro(rate)
                .with_accel(rate)
                .with_high_g(true),
        )
        .with_int1(IntRouting::new())
        .with_int2(IntRouting::new())
}

// ---------------------------------------------------------------------------------------------
// FIFO word statistics
// ---------------------------------------------------------------------------------------------

struct Scan {
    words: u32,
    gyro: u32,
    accel: u32,
    high_g: u32,
    temp: u32,
    ts: u32,
    cfg: u32,
    unknown: u32,
    trailing: u32,
    gsum: [f32; 3],
    asum: [f32; 3],
    hsum: [f32; 3],
    hraw: [i32; 3],
    tsum: f32,
    last_ts: u32,
    d_n: u32,
    d_min: u32,
    d_max: u32,
    d_sum: u64,
    d_bad: u32,
    exp_ticks: f32,
    sensors: [u8; 16],
    slots: [u8; 16],
    cfg_idx: [u32; 4],
    cfg_data: [[u8; 6]; 4],
    gcnt: u8,
    acnt: u8,
    g_jumps: u32,
    a_jumps: u32,
    temp_rest: [u8; 4],
    ts_rest: [u8; 2],
}

impl Scan {
    /// `exp_ticks`: expected timestamp delta in ticks, 0 when not checked.
    fn new(exp_ticks: f32) -> Self {
        Self {
            words: 0,
            gyro: 0,
            accel: 0,
            high_g: 0,
            temp: 0,
            ts: 0,
            cfg: 0,
            unknown: 0,
            trailing: 0,
            gsum: [0.0; 3],
            asum: [0.0; 3],
            hsum: [0.0; 3],
            hraw: [0; 3],
            tsum: 0.0,
            last_ts: 0,
            d_n: 0,
            d_min: u32::MAX,
            d_max: 0,
            d_sum: 0,
            d_bad: 0,
            exp_ticks,
            sensors: [0xFF; 16],
            slots: [0xFF; 16],
            cfg_idx: [0; 4],
            cfg_data: [[0; 6]; 4],
            gcnt: 0xFF,
            acnt: 0xFF,
            g_jumps: 0,
            a_jumps: 0,
            temp_rest: [0; 4],
            ts_rest: [0; 2],
        }
    }

    fn feed(&mut self, bytes: &[u8], c: &Config) {
        let hfs = c.high_g.map(|h| h.full_scale);
        for f in fifo_frames(bytes) {
            let i = self.words as usize;
            let tag = match &f {
                FifoFrame::Gyro(s) | FifoFrame::Accel(s) | FifoFrame::HighG(s) => Some(s.tag),
                FifoFrame::Temperature(t) => Some(t.tag),
                FifoFrame::Timestamp(t) => Some(t.tag),
                FifoFrame::ConfigChange(t) => Some(t.tag),
                FifoFrame::Unknown(u) => Some(u.tag()),
                _ => None,
            };
            if let Some(t) = tag
                && i < 16
            {
                self.sensors[i] = t.sensor;
                self.slots[i] = t.slot;
            }
            match f {
                FifoFrame::Gyro(s) => {
                    if self.gcnt != 0xFF && s.tag.slot != (self.gcnt + 1) & 3 {
                        self.g_jumps += 1;
                    }
                    self.gcnt = s.tag.slot;
                    self.gyro += 1;
                    add3(&mut self.gsum, s.axes.gyro_rads(c.gyro_fs));
                }
                FifoFrame::Accel(s) => {
                    if self.acnt != 0xFF && s.tag.slot != (self.acnt + 1) & 3 {
                        self.a_jumps += 1;
                    }
                    self.acnt = s.tag.slot;
                    self.accel += 1;
                    add3(&mut self.asum, s.axes.accel_ms2(c.accel_fs));
                }
                FifoFrame::HighG(s) => {
                    self.high_g += 1;
                    if let Some(fs) = hfs {
                        add3(&mut self.hsum, s.axes.high_g_ms2(fs));
                    }
                    self.hraw[0] += i32::from(s.axes.x);
                    self.hraw[1] += i32::from(s.axes.y);
                    self.hraw[2] += i32::from(s.axes.z);
                }
                FifoFrame::Temperature(t) => {
                    if self.temp == 0 {
                        self.temp_rest = t.rest;
                    }
                    self.temp += 1;
                    self.tsum += temperature_c(t.raw);
                }
                FifoFrame::Timestamp(t) => {
                    if self.ts > 0 {
                        let d = timestamp_delta_ticks(self.last_ts, t.ticks);
                        self.d_n += 1;
                        self.d_min = self.d_min.min(d);
                        self.d_max = self.d_max.max(d);
                        self.d_sum += u64::from(d);
                        if self.exp_ticks > 0.0 && abs(d as f32 / self.exp_ticks - 1.0) > 0.05 {
                            self.d_bad += 1;
                        }
                    }
                    if self.ts == 0 {
                        self.ts_rest = t.rest;
                    }
                    self.last_ts = t.ticks;
                    self.ts += 1;
                }
                FifoFrame::ConfigChange(t) => {
                    let k = self.cfg as usize;
                    if k < 4 {
                        self.cfg_idx[k] = self.words;
                        self.cfg_data[k] = t.payload;
                    }
                    self.cfg += 1;
                }
                FifoFrame::Trailing(_) => {
                    self.trailing += 1;
                    continue;
                }
                _ => self.unknown += 1,
            }
            self.words += 1;
        }
    }

    fn print(&self, label: &str) {
        defmt::info!(
            "[STAT] {}: words {} gyro {} accel {} high-g {} temp {} ts {} cfg {} unknown {} trailing {}",
            label,
            self.words,
            self.gyro,
            self.accel,
            self.high_g,
            self.temp,
            self.ts,
            self.cfg,
            self.unknown,
            self.trailing
        );
    }

    fn print_tags(&self, label: &str) {
        defmt::info!(
            "[STAT] {} TAG_CNT jumps (not +1 mod 4): gyro {}, accel {}; first temperature rest bytes {}, first timestamp rest bytes {}",
            label,
            self.g_jumps,
            self.a_jumps,
            self.temp_rest,
            self.ts_rest
        );
        defmt::info!("[STAT] {} TAG_SENSOR of first 16 words {}", label, self.sensors);
        defmt::info!("[STAT] {} TAG_CNT of first 16 words    {}", label, self.slots);
    }

    /// Delta statistics in microseconds: (min, max, mean).
    fn delta_us(&self, tick_ns: u32) -> (f32, f32, f32) {
        let t = tick_ns as f32 * 1e-3;
        (
            self.d_min as f32 * t,
            self.d_max as f32 * t,
            self.d_sum as f32 / self.d_n.max(1) as f32 * t,
        )
    }

    /// Clean decode and the batching pattern for the configuration (c) rates.
    fn pattern_ok(&self) -> bool {
        let g = self.gyro;
        let balanced = g.abs_diff(self.accel) <= 1;
        let ts_ok = self.ts + 1 >= g / 8 && self.ts <= g / 8 + 2;
        let temp_ok = self.temp + 2 >= g / 16 && self.temp <= g / 16 + 2;
        self.unknown == 0 && self.trailing == 0 && self.cfg == 0 && g > 0 && balanced && ts_ok && temp_ok
    }
}

/// Expected timestamp delta in ticks: 8 gyro periods at the real rate, from the trim.
fn exp_ticks(info: &Info) -> f32 {
    let period_us = 1e9 / info.gyro_odr_mhz.max(1) as f32;
    8.0 * period_us * 1000.0 / calc::timestamp_tick_ns(info.freq_fine) as f32
}

fn exp_delta_us(info: &Info) -> f32 {
    8.0 * 1e9 / info.gyro_odr_mhz.max(1) as f32
}

// ---------------------------------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------------------------------

async fn verify_is_ok<S>(fails: &mut u32, label: &str, imu: &mut Lsm6dsk320x<S>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    match imu.verify().await {
        Ok(()) => check!(*fails, true, "{} verify after {}: Ok", label),
        Err(Error::ConfigMismatch(m)) => check!(
            *fails,
            false,
            "{} verify after {}: ConfigMismatch reg 0x{=u8:x} mask 0x{=u8:x} expected 0x{=u8:x} found 0x{=u8:x}",
            label,
            m.register,
            m.mask,
            m.expected,
            m.found
        ),
        Err(e) => check!(
            *fails,
            false,
            "{} verify after {}: {}",
            label,
            defmt::Debug2Format(&e)
        ),
    }
}

async fn check1<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] check 1: verify() on the real chip");
    let mut d = Delay;
    let steps: [(&str, Config, bool); 7] = [
        ("init(a)", cfg_a(), true),
        ("init(b)", cfg_b(), true),
        ("init(c)", cfg_c(), true),
        ("configure(c->a)", cfg_a(), false),
        ("configure(a->b)", cfg_b(), false),
        ("configure(b->c)", cfg_c(), false),
        ("configure(c->b)", cfg_b(), false),
    ];
    for (label, cfg, is_init) in steps {
        let r = if is_init {
            imu.init(&mut d, &cfg).await
        } else {
            imu.configure(&mut d, &cfg).await
        };
        match r {
            Ok(_) => verify_is_ok(fails, label, imu).await,
            Err(e) => check!(*fails, false, "{} {} failed: {}", label, defmt::Debug2Format(&e)),
        }
    }
}

async fn check2<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] check 2: positive control for verify()");
    let mut d = Delay;
    need!(*fails, "init(a)", imu.init(&mut d, &cfg_a()).await);
    verify_is_ok(fails, "baseline", imu).await;
    let v = need!(*fails, "read CTRL6", imu.read_register(reg::CTRL6).await);
    let w = v & !reg::CTRL6_FS_G_MASK;
    need!(*fails, "write CTRL6", imu.write_register(reg::CTRL6, w).await);
    let back = need!(*fails, "read CTRL6", imu.read_register(reg::CTRL6).await);
    defmt::info!("[STAT] CTRL6 was 0x{=u8:x}, written 0x{=u8:x}, read back 0x{=u8:x}", v, w, back);
    match imu.verify().await {
        Err(Error::ConfigMismatch(m)) => {
            defmt::info!(
                "[STAT] mismatch reg 0x{=u8:x} mask 0x{=u8:x} expected 0x{=u8:x} found 0x{=u8:x}",
                m.register,
                m.mask,
                m.expected,
                m.found
            );
            check!(
                *fails,
                m.register == reg::CTRL6 && m.found == back,
                "{} verify reports exactly CTRL6 (0x{=u8:x})",
                m.register
            );
        }
        other => check!(
            *fails,
            false,
            "{} verify after the raw write should report CTRL6, got {}",
            defmt::Debug2Format(&other)
        ),
    }
    need!(*fails, "configure(a)", imu.configure(&mut d, &cfg_a()).await);
    verify_is_ok(fails, "restore by configure(a)", imu).await;
}

fn eval4(fails: &mut u32, label: &str, s: &Scan, info: &Info, cfg: &Config, reg_temp: f32) {
    let tick = calc::timestamp_tick_ns(info.freq_fine);
    defmt::info!(
        "[STAT] {} freq_fine {} tick {} ns, gyro odr {} mHz, expected delta {} us",
        label,
        info.freq_fine,
        tick,
        info.gyro_odr_mhz,
        exp_delta_us(info)
    );
    let t = s.tsum / s.temp.max(1) as f32;
    defmt::info!(
        "[STAT] {} temperature: FIFO mean {} C over {} words, register {} C",
        label,
        t,
        s.temp,
        reg_temp
    );
    check!(
        *fails,
        s.temp > 0 && abs(t - reg_temp) <= 3.0,
        "{} {} FIFO temperature agrees with register within 3 C",
        label
    );
    let (mn, mx, mean) = s.delta_us(tick);
    let exp = exp_delta_us(info);
    defmt::info!(
        "[STAT] {} timestamp delta over {} intervals: min {} us, max {} us, mean {} us",
        label,
        s.d_n,
        mn,
        mx,
        mean
    );
    let within = |v: f32| abs(v / exp - 1.0) <= 0.05;
    check!(
        *fails,
        s.d_n > 0 && within(mn) && within(mx) && within(mean),
        "{} {} timestamp deltas equal 8 output periods within 5 %",
        label
    );
    let a = mean3(s.asum, s.accel);
    let g = mean3(s.gsum, s.gyro);
    let am = mag(a) / G;
    let gd = g.map(|v| v * DEG);
    defmt::info!("[STAT] {} accel mean {} g axes, |a| {} g; gyro mean {} dps", label, a.map(|v| v / G), am, gd);
    let _ = cfg;
    check!(
        *fails,
        s.accel > 0 && (0.9..=1.1).contains(&am),
        "{} {} FIFO |a| {} g (0.9..1.1)",
        label,
        am
    );
    check!(
        *fails,
        s.gyro > 0 && gd.iter().all(|v| abs(*v) <= 5.0),
        "{} {} FIFO gyro {} dps (each within 5)",
        label,
        gd
    );
}

async fn check34<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] check 3: one burst over many words, against word by word");
    let mut d = Delay;
    let cfg = cfg_c();
    let info = need!(*fails, "init(c)", imu.init(&mut d, &cfg).await);
    let exp = exp_ticks(&info);
    need!(*fails, "fifo_flush", imu.fifo_flush().await);
    Timer::after_millis(100).await;
    let mut buf = [0u8; BUF_WORDS * WORD];
    let r = need!(*fails, "fifo_read", imu.fifo_read(&mut buf).await);
    defmt::info!(
        "[STAT] burst: read {} words in one burst, unread after {}, flags {}",
        r.words,
        r.unread.unwrap_or(0xFFFF),
        r.flags
    );
    let mut burst = Scan::new(exp);
    burst.feed(&buf[..r.bytes], &cfg);
    burst.print("burst");
    burst.print_tags("burst");
    check!(
        *fails,
        burst.words > 100 && burst.unknown == 0 && burst.trailing == 0 && burst.pattern_ok(),
        "{} burst read of {} words decodes cleanly and follows the batching pattern",
        burst.words
    );
    if burst.unknown > 0 {
        let off = usize::try_from(burst.words).unwrap_or(0).min(8);
        defmt::info!("[STAT] burst first 3 words raw {=[u8]:x}", &buf[..3 * WORD]);
        let _ = off;
    }

    need!(*fails, "fifo_flush", imu.fifo_flush().await);
    Timer::after_millis(40).await;
    let mut single = [0u8; 56 * WORD];
    let mut errs = 0u32;
    for i in 0..56 {
        if imu.fifo_read_exact(&mut single[i * WORD..], 1).await.is_err() {
            errs += 1;
        }
    }
    let mut one = Scan::new(exp);
    one.feed(&single, &cfg);
    one.print("word-by-word");
    one.print_tags("word-by-word");
    check!(
        *fails,
        errs == 0 && one.words == 56 && one.pattern_ok(),
        "{} word-by-word read of 56 words (bus errors {}) decodes cleanly and follows the pattern",
        errs
    );

    defmt::info!("[INFO] check 4: word layouts");
    let reg_temp = need!(*fails, "read_all", imu.read_all().await).temperature_c();
    if burst.unknown == 0 && burst.trailing == 0 {
        eval4(fails, "burst", &burst, &info, &cfg, reg_temp);
    } else {
        defmt::info!("[INFO] burst is not clean: check 4 evaluated on the word-by-word frames");
        eval4(fails, "word-by-word", &one, &info, &cfg, reg_temp);
    }
}

async fn check5<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] check 5: configuration-change word");
    let mut d = Delay;
    let mut before = cfg_c();
    before.fifo = before.fifo.with_odr_change(true);
    let after = before
        .with_gyro_odr(Odr::Hz480)
        .with_accel_odr(Odr::Hz480)
        .with_fifo(
            before
                .fifo
                .with_gyro(BatchRate::Hz480)
                .with_accel(BatchRate::Hz480),
        );
    let info = need!(*fails, "init", imu.init(&mut d, &before).await);
    need!(*fails, "fifo_flush", imu.fifo_flush().await);
    Timer::after_millis(20).await;
    let mut buf = [0u8; BUF_WORDS * WORD];
    let r = need!(*fails, "fifo_read", imu.fifo_read(&mut buf).await);
    let mut pre = Scan::new(exp_ticks(&info));
    pre.feed(&buf[..r.bytes], &before);
    pre.print("before the change");
    Timer::after_millis(20).await;
    need!(*fails, "configure", imu.configure(&mut d, &after).await);
    Timer::after_millis(30).await;
    let r = need!(*fails, "fifo_read", imu.fifo_read(&mut buf).await);
    let mut post = Scan::new(0.0);
    post.feed(&buf[..r.bytes], &after);
    post.print("after the change");
    post.print_tags("after the change");
    for (name, s) in [("before", &pre), ("after", &post)] {
        for k in 0..(s.cfg as usize).min(4) {
            defmt::info!(
                "[STAT] {} cfg-change word {} at stream index {}, data bytes {}",
                name,
                k,
                s.cfg_idx[k],
                s.cfg_data[k]
            );
        }
    }
    defmt::info!(
        "[STAT] configuration-change words seen: before {}, after {} (configure puts the FIFO in bypass first)",
        pre.cfg,
        post.cfg
    );
    check!(
        *fails,
        pre.unknown == 0 && post.unknown == 0 && pre.trailing == 0 && post.trailing == 0,
        "{} no unknown words around the configuration change (unknown {} / {})",
        pre.unknown,
        post.unknown
    );
}

async fn check6<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] check 6: depth, overrun, flush");
    let mut d = Delay;
    let cfg = cfg_c();
    need!(*fails, "init(c)", imu.init(&mut d, &cfg).await);
    need!(*fails, "fifo_flush", imu.fifo_flush().await);
    let ov0 = imu.stats().fifo_overruns;
    Timer::after_millis(400).await;
    let (mut max_words, mut full, mut ovr, mut latched, mut wtm) = (0u16, false, false, false, false);
    let mut first = None;
    for _ in 0..6 {
        let st = need!(*fails, "fifo_status", imu.fifo_status().await);
        if first.is_none() {
            first = Some(st);
        }
        max_words = max_words.max(st.words);
        full |= st.flags.full;
        ovr |= st.flags.overrun;
        latched |= st.flags.overrun_latched;
        wtm |= st.flags.watermark;
    }
    let ov1 = imu.stats().fifo_overruns;
    if let Some(f) = first {
        defmt::info!("[STAT] first status after 400 ms: words {}, flags {}", f.words, f.flags);
    }
    defmt::info!(
        "[STAT] largest unread count {} words; any: wtm {} full {} overrun {} latched {}; stats overruns {} -> {}",
        max_words,
        wtm,
        full,
        ovr,
        latched,
        ov0,
        ov1
    );
    check!(
        *fails,
        max_words >= 200,
        "{} FIFO depth as reported: {} words (datasheet: 214 or 219)",
        max_words
    );
    check!(
        *fails,
        ovr || latched,
        "{} overrun flags seen (overrun {}, latched {})",
        ovr,
        latched
    );
    check!(
        *fails,
        ov1 > ov0,
        "{} stats() counted the overrun ({} -> {})",
        ov0,
        ov1
    );
    need!(*fails, "fifo_flush", imu.fifo_flush().await);
    let st = need!(*fails, "fifo_status", imu.fifo_status().await);
    defmt::info!("[STAT] status right after flush: words {}, flags {}", st.words, st.flags);
    check!(
        *fails,
        st.words <= 3 && !st.flags.any_overrun(),
        "{} flush leaves {} words (max 3), no overrun",
        st.words
    );
    verify_is_ok(fails, "overrun and flush", imu).await;
    Timer::after_millis(30).await;
    let mut buf = [0u8; BUF_WORDS * WORD];
    let r = need!(*fails, "fifo_read", imu.fifo_read(&mut buf).await);
    let mut s = Scan::new(0.0);
    s.feed(&buf[..r.bytes], &cfg);
    s.print("after flush");
    check!(
        *fails,
        s.words > 0 && s.unknown == 0 && s.trailing == 0,
        "{} words read after the flush decode cleanly ({})",
        s.words
    );
    need!(*fails, "fifo_flush", imu.fifo_flush().await);
    Timer::after_millis(300).await;
    let r = need!(*fails, "fifo_read", imu.fifo_read(&mut buf).await);
    defmt::info!("[STAT] overrun FIFO read in one burst: {} words, unread after {}, flags {}", r.words, r.unread.unwrap_or(0xFFFF), r.flags);
    let mut s = Scan::new(0.0);
    s.feed(&buf[..r.bytes], &cfg);
    s.print("full FIFO");
    s.print_tags("full FIFO");
    check!(
        *fails,
        r.words >= 214 && s.unknown == 0 && s.trailing == 0,
        "{} a full, overrun FIFO read in one burst decodes cleanly ({} words)",
        r.words
    );
}

async fn check7<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>, int1: &mut Input<'_>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] check 7: watermark interrupt path");
    let mut d = Delay;
    let cfg = cfg_c();
    let info = need!(*fails, "init(c)", imu.init(&mut d, &cfg).await);
    need!(*fails, "fifo_flush", imu.fifo_flush().await);
    let mut scan = Scan::new(exp_ticks(&info));
    let mut buf = [0u8; 28 * WORD];
    let (mut edges, mut errors, mut sum_us, mut worst_us) = (0u32, 0u32, 0u64, 0u32);
    let mut timed_out = false;
    for _ in 0..200 {
        if with_timeout(Duration::from_millis(200), int1.wait_for_rising_edge())
            .await
            .is_err()
        {
            timed_out = true;
            break;
        }
        edges += 1;
        let t = Instant::now();
        let r = imu.fifo_read_exact(&mut buf, 28).await;
        let us = t.elapsed().as_micros() as u32;
        sum_us += u64::from(us);
        worst_us = worst_us.max(us);
        match r {
            Ok(_) => scan.feed(&buf, &cfg),
            Err(_) => errors += 1,
        }
    }
    check!(
        *fails,
        !timed_out && edges == 200,
        "{} INT1 edges {} of 200, timeout {}",
        edges,
        timed_out
    );
    scan.print("watermark loop");
    scan.print_tags("watermark loop");
    defmt::info!(
        "[STAT] fifo_read_exact(28): mean {} us, worst {} us over {} calls, bus errors {}",
        sum_us as f32 / edges.max(1) as f32,
        worst_us,
        edges,
        errors
    );
    let (mn, mx, mean) = scan.delta_us(calc::timestamp_tick_ns(info.freq_fine));
    defmt::info!(
        "[STAT] timestamp deltas over {} intervals: min {} us, max {} us, mean {} us, outside 5 %: {}",
        scan.d_n,
        mn,
        mx,
        mean,
        scan.d_bad
    );
    check!(
        *fails,
        errors == 0 && scan.words == 5600 && scan.unknown == 0 && scan.trailing == 0 && scan.d_bad == 0 && scan.d_n > 100,
        "{} 5600 words, none unknown ({}), timestamps without a gap (bad deltas {})",
        scan.unknown,
        scan.d_bad
    );
    match imu.fifo_status().await {
        Ok(st) => defmt::info!("[STAT] fifo_status after the loop: words {}, flags {}", st.words, st.flags),
        Err(e) => check!(*fails, false, "{} fifo_status failed: {}", defmt::Debug2Format(&e)),
    }
}

async fn check8<S>(fails: &mut u32, imu: &mut Lsm6dsk320x<S>)
where
    S: SpiDevice,
    S::Error: Debug,
{
    defmt::info!("[INFO] check 8: high-g words");
    let mut d = Delay;
    for with_low in [false, true] {
        let label = if with_low { "high-g+gyro+accel" } else { "high-g alone" };
        let cfg = cfg_high_g(with_low);
        need!(*fails, "init", imu.init(&mut d, &cfg).await);
        need!(*fails, "fifo_flush", imu.fifo_flush().await);
        let mut s = Scan::new(0.0);
        let mut buf = [0u8; BUF_WORDS * WORD];
        for _ in 0..20 {
            if s.words >= 200 {
                break;
            }
            Timer::after_millis(50).await;
            let r = need!(*fails, "fifo_read", imu.fifo_read(&mut buf).await);
            s.feed(&buf[..r.bytes], &cfg);
        }
        s.print(label);
        let share = s.high_g as f32 / s.words.max(1) as f32;
        let want = if with_low { 1.0 / 3.0 } else { 1.0 };
        let h = mean3(s.hsum, s.high_g);
        let hm = mag(h) / G;
        defmt::info!(
            "[STAT] {} high-g share {} (expected {}); mean raw counts {}, mean {} g axes, |a| {} g",
            label,
            share,
            want,
            s.hraw.map(|v| v as f32 / s.high_g.max(1) as f32),
            h.map(|v| v / G),
            hm
        );
        check!(
            *fails,
            s.words >= 200 && s.unknown == 0 && s.trailing == 0 && abs(share - want) <= 0.05,
            "{} {} 200 words, high-g share {} as expected",
            label,
            share
        );
        check!(
            *fails,
            s.high_g > 0 && abs(hm - 1.0) <= 3.0,
            "{} {} high-g |a| {} g at rest (within 3 g of 1 g)",
            label,
            hm
        );
    }
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
    let mut int1 = Input::new(p.PIN_7, Pull::None);

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
    run!(1, check1(&mut fails, &mut imu));
    run!(2, check2(&mut fails, &mut imu));
    run!(3, check34(&mut fails, &mut imu));
    run!(5, check5(&mut fails, &mut imu));
    run!(6, check6(&mut fails, &mut imu));
    run!(7, check7(&mut fails, &mut imu, &mut int1));
    run!(8, check8(&mut fails, &mut imu));
    defmt::info!("[INFO] run time {} ms", start.elapsed().as_millis());
    hil_common::exit(fails)
}

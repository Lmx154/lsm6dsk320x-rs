//! On-board soak of the lsm6dsk320x driver (U23 on SPI1, INT1 on GPIO7): data integrity over a long
//! run, with a CPU spinner and a DMA memory-copy loop running in thread mode the whole time.
//!
//! Three phases of equal length (`SOAK_SECONDS` / 3), gyro, low-g and high-g on in all of them:
//! A1 (7.68 kHz, INT1 = low-g data-ready, counted; INT2 = high-g data-ready, routed, nothing waits
//! on it): after each INT1 edge ONE `read_all()`, the way firmware reads; the edge count read at
//! least G = 25 us after the read returned must equal the count read before it (else a "late
//! read"; at most 0.5 % allowed). A2 (3.84 kHz, same routing): a PAIR of `read_all()` per edge,
//! byte-identical on gyro, low-g and high-g (18 bytes). The registers change up to 21.75 us BEFORE
//! the edge, so a pair counts only if the edge count read before the first read equals the count
//! read at least G after the second read returned, G from the lead measurement at 3.84 kHz
//! (CONTRACT.md, amendment of 2026-11-11, night); skipped pairs (edge, late) at most 1 %.
//! B: FIFO at 960 Hz with gyro, low-g and high-g words, 8 of each between two timestamp words.
//! Motion (a bump) is a statistic, never a failure. Nothing is added to a flight start-up: this is a
//! one-time bench test. No SPI transfer is ever cancelled: the only timeouts wrap the wait for an
//! INT1 edge. Only the driver's typed methods and its raw register reads.
//!
//! Pauses: only the sampling task delays itself, only inside a declared window (identity read
//! every 100 reads or pairs, the minute `verify()` with the progress line printed inside it), and
//! only when no edge waits for its read. Nothing is logged outside a window during a timed
//! section; thread mode only reads atomics. Each phase is reported when it ends.
//!
//! A1 has no identity read every 100 reads: at 7.68 kHz that window ended about 30 us before the
//! next edge and an edge falling at its end was lost. In A1 the identity read is the first part of
//! the minute window (identity, `verify()`, progress line); A2 (265 us period) keeps it every 100
//! pairs.
//!
//! Window ends: the wait for the next edge is armed before the read's second count and so before
//! any window; the count read at the end of a window (or at the second count) is the window's
//! "after" count, `last`; every edge up to it is served or paused. A wake whose count has not
//! advanced beyond `last` is stale (its edge was counted already): no read is served, a fresh wait
//! is armed, its count is read, and the loop waits again (counted in `stale`).
//!
//! Edge bookkeeping (phases A1, A2): `last` is the hardware edge count of the edge serviced last.
//! When a read or pair starts with count `c`, `c - last - 1` edges came and went unserviced:
//! `missed`. A read or pair that an edge interrupted (before or within G) is "skipped" (late) and
//! is NOT credited with serving that edge: the interrupting edge is serviced next, at once. So
//! every edge has one read or pair and `lost = edges - reads - paused` must be exactly 0.
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::hint::black_box;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::{Context, Poll, Waker};

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::interrupt::{self, InterruptExt, Priority};
use embassy_rp::peripherals::{
    DMA_CH0, DMA_CH1, DMA_CH2, PIN_7, PIN_8, PIN_9, PIN_10, PIN_11, PWM_SLICE3, SPI1,
};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::fifo::timestamp_delta_ticks;
use lsm6dsk320x::sample::G;
use lsm6dsk320x::{
    AccelFullScale, BatchRate, Config, DataReady, Error, FifoConfig, FifoFrame, FifoMode,
    GyroFullScale, HighGConfig, HighGFullScale, HighGOdr, IntRouting, Lsm6dsk320x, Odr, RawAll,
    RawAxes, TempBatchRate, TimestampBatch, fifo_frames, reg,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

type Dev = ExclusiveDevice<Spi<'static, SPI1, spi::Async>, Output<'static>, Delay>;

const fn parse_secs(s: Option<&str>) -> u32 {
    let Some(s) = s else { return 1800 };
    let b = s.as_bytes();
    let mut n = 0u32;
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c < b'0' || c > b'9' {
            return 1800;
        }
        n = n * 10 + (c - b'0') as u32;
        i += 1;
    }
    if n == 0 { 1800 } else { n }
}

const fn is_one(s: Option<&str>) -> bool {
    match s {
        Some(s) => s.len() == 1 && s.as_bytes()[0] == b'1',
        None => false,
    }
}

const TOTAL_S: u32 = parse_secs(option_env!("SOAK_SECONDS"));
const P_S: u32 = TOTAL_S / 3;
const REQ_SOAK_HZ: u32 = 9_500_000; // gives 9.375 MHz, SPI mode 3
const REQ_SMOKE_HZ: u32 = 8_000_000; // gives 7.5 MHz, SPI mode 0, as the smoke test
const CPU_MHZ: u32 = 150;
/// The guard time G, from a read's return to the second edge count: 1.1 x the largest lead of an
/// output register before its data-ready edge, whole us (CONTRACT.md amendment of 2026-11-11,
/// night). A1 (7.68 kHz): largest lead 21.75 us (gyro X, 0.5 us bins, lag 0), G = 25 us.
const G1_US: u32 = 25;
/// A2 (3.84 kHz): lead measured with `LEAD_HZ3840=1` (lsm6dsk320x_drdy_lead.rs, logs
/// lsm6dsk320x_drdy_lead-20261111-184537.log and -184823.log): largest 22.25 us (gyro X, 0.5 us
/// bins, upper bin edge 22.5 us; 21.75 us in the first run), smallest lead 4.25 us, so lag 0;
/// 1.1 x 22.5 us = 24.75 us, rounded up = 25 us.
const G2_US: u32 = 25;
/// Negative control: with `SOAK_INJECT=1` at build time (any other value is off), one read of
/// phase A and one FIFO word of phase B are corrupted in RAM, and the run must fail with both
/// recorded. The report says so.
const INJECT: bool = is_one(option_env!("SOAK_INJECT"));
const WORD: usize = 7;
const WM: usize = 28;
const DEG: f32 = 180.0 / core::f32::consts::PI;

macro_rules! check {
    ($fails:expr, $ok:expr, $fmt:literal $(, $arg:expr)*) => {{
        let ok = $ok;
        if !ok {
            $fails += 1;
        }
        defmt::info!($fmt, if ok { "[PASS]" } else { "[FAIL]" } $(, $arg)*);
    }};
}

// ---------------------------------------------------------------------------------------------
// Shared with thread mode (atomics only: the verdict and the aggressor counters)
// ---------------------------------------------------------------------------------------------

static FAILS: AtomicU32 = AtomicU32::new(0);
static DONE: AtomicBool = AtomicBool::new(false);
static STOP: AtomicBool = AtomicBool::new(false);
static SPIN_COUNT: AtomicU32 = AtomicU32::new(0);
static DMA_COUNT: AtomicU32 = AtomicU32::new(0);
static DMA_BAD: AtomicU32 = AtomicU32::new(0);

fn cyc() -> u32 {
    DWT::cycle_count()
}

fn dwt_on() {
    // SAFETY: only the trace enable and the cycle counter of the calling core are touched.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}

fn spi_hz() -> u32 {
    let p = embassy_rp::pac::SPI1;
    let div = p.cpsr().read().cpsdvsr() as u32 * (1 + p.cr0().read().scr() as u32);
    embassy_rp::clocks::clk_peri_freq() / div
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

// ---------------------------------------------------------------------------------------------
// Record
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Ev {
    idx: u32,
    t_us: u32,
    edges: u32,
    a: [i16; 9],
    b: [i16; 9],
    w: [u8; 21],
    exp: u32,
    found: u32,
    flags: u32,
    gap: u32,
    d1: u32,
    d2: u32,
    since_us: u32,
}

const EV0: Ev = Ev {
    idx: 0,
    t_us: 0,
    edges: 0,
    a: [0; 9],
    b: [0; 9],
    w: [0; 21],
    exp: 0,
    found: 0,
    flags: 0,
    gap: 0,
    d1: 0,
    d2: 0,
    since_us: 0,
};

#[derive(Clone, Copy)]
struct Win {
    start_us: u32,
    len: u32,
    min: f32,
    max: f32,
}

const WIN0: Win = Win {
    start_us: 0,
    len: 0,
    min: 0.0,
    max: 0.0,
};
const KINDS: usize = 14;
const NAMES: [&str; KINDS] = [
    "mismatch",
    "identity",
    "verify A",
    "bus error A",
    "stuck",
    "malformed word",
    "timestamp step",
    "overrun",
    "verify B",
    "bus error B",
    "missed edge A",
    "missed edge B",
    "verify window",
    "words per interval",
];
const K_MISMATCH: usize = 0;
const K_IDENT: usize = 1;
const K_VER_A: usize = 2;
const K_BUS_A: usize = 3;
const K_STUCK: usize = 4;
const K_MALFORMED: usize = 5;
const K_TS: usize = 6;
const K_OVERRUN: usize = 7;
const K_VER_B: usize = 8;
const K_BUS_B: usize = 9;
const K_MISSED_A: usize = 10;
const K_MISSED_B: usize = 11;
const K_VERWIN: usize = 12;
const K_INTERVAL: usize = 13;

struct Rec {
    n: [u32; KINDS],
    ev: [[Ev; 16]; KINDS],
    ring: [[i16; 9]; 32],
    ring_i: usize,
    ring_n: u32,
    snap: [[[i16; 9]; 32]; 4],
    snap_n: usize,
    snap_kind: [u8; 4],
    snap_idx: [u32; 4],
    win: [Win; 16],
    hist: [u32; 2048],
    dur: [u32; 512],
    tsd: [u32; 1000],
}

static mut REC: Rec = Rec {
    n: [0; KINDS],
    ev: [[EV0; 16]; KINDS],
    ring: [[0; 9]; 32],
    ring_i: 0,
    ring_n: 0,
    snap: [[[0; 9]; 32]; 4],
    snap_n: 0,
    snap_kind: [0; 4],
    snap_idx: [0; 4],
    win: [WIN0; 16],
    hist: [0; 2048],
    dur: [0; 512],
    tsd: [0; 1000],
};

/// The compared channels (18 bytes) of a read: gyro, low-g, high-g, x y z each. Temperature and
/// 0x2E-0x33 are not compared.
fn flat(s: &RawAll) -> [i16; 9] {
    [
        s.gyro.x, s.gyro.y, s.gyro.z, s.accel.x, s.accel.y, s.accel.z, s.high_g.x, s.high_g.y,
        s.high_g.z,
    ]
}

impl Rec {
    fn push_read(&mut self, s: &RawAll) {
        self.ring[self.ring_i] = flat(s);
        self.ring_i = (self.ring_i + 1) % 32;
        self.ring_n = self.ring_n.saturating_add(1);
    }

    fn event(&mut self, kind: usize, ev: Ev) {
        let k = self.n[kind];
        self.n[kind] = k.saturating_add(1);
        if k < 16 {
            self.ev[kind][k as usize] = ev;
        }
        if (kind <= K_STUCK || kind == K_MISSED_A) && self.snap_n < 4 {
            let have = (self.ring_n as usize).min(32);
            let first = if have < 32 { 0 } else { self.ring_i };
            let mut out = [[0i16; 9]; 32];
            for (i, o) in out.iter_mut().take(have).enumerate() {
                *o = self.ring[(first + i) % 32];
            }
            self.snap[self.snap_n] = out;
            self.snap_kind[self.snap_n] = kind as u8;
            self.snap_idx[self.snap_n] = ev.idx;
            self.snap_n += 1;
        }
    }
}

#[derive(Default)]
struct S {
    // phase A
    pairs: u32,
    counted: u32,
    skipped_edge: u32,
    skipped_late: u32,
    mismatch: u32,
    mm_gyro: u32,
    mm_low: u32,
    mm_high: u32,
    g_short: u32,
    pair_min: u32,
    pair_sum: u64,
    serve_min: u32,
    serve_sum: u64,
    serve_max: u32,
    ident_wrong: u32,
    ver_a: u32,
    bus_a: u32,
    stuck: [u32; 3],
    timeouts_a: u32,
    missed: u32,
    hw_a: u32,
    paused_a: u32,
    ctrl_flagged: u32,
    motion: u32,
    windows: u32,
    longest: u32,
    peak_gyro: i32,
    a_ran_us: u32,
    worst_cyc: u32,
    // phase B
    words: u32,
    reads_b: u32,
    reads_ok_b: u32,
    malformed: u32,
    ts_bad: u32,
    overrun: u32,
    ver_b: u32,
    bus_b: u32,
    timeouts_b: u32,
    missed_b: u32,
    hw_b: u32,
    paused_b: u32,
    ts_n: u32,
    ts_med: u32,
    max_unread: u32,
    cfg_words: u32,
    fifo_starts: u32,
    ready_now: u32,
    pend_start: u32,
    words_g: u32,
    words_l: u32,
    words_h: u32,
    words_t: u32,
    intervals: u32,
    intervals_bad: u32,
    ts_min: u32,
    ts_max: u32,
    ts_set: bool,
    rate_mhz: u32,
    edges_loop: u64,
    t_loop_us: u64,
    first_rel: u64,
    end_gap: u64,
    ver_paused: u32,
    stale: u32,
    // verify windows, both phases
    verify_n: u32,
    id_n: u32,
    id_max_us: u32,
    id_max_paused: u32,
    verwin_bad: u32,
    ver_max_us: u32,
    ver_max_paused: u32,
    // current phase scratch
    hw: u32,
    paused: u32,
    last_p0: u64,
    post_cyc: [u32; 8],
    post_worst: u32,
}

/// The INT1 edges, counted by PWM slice 3 (16 bit), summed so that it never wraps.
struct Hw<'a> {
    pwm: &'a Pwm<'static>,
    last: u16,
    acc: u64,
}

impl Hw<'_> {
    fn sync(&mut self) -> u16 {
        let c = self.pwm.counter();
        self.acc += u64::from(c.wrapping_sub(self.last));
        self.last = c;
        c
    }
}

/// How the hardware edge count classifies a pair: unchanged until G after the second read
/// returned (counted, compared), changed before the second read returned (skipped, edge), or
/// changed only during G (skipped, late).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Class {
    Counted,
    SkipEdge,
    SkipLate,
}

/// Bit i set if compared value i differs: 0-2 gyro x y z, 3-5 low-g x y z, 6-8 high-g x y z.
fn diff_mask(a: &RawAll, b: &RawAll) -> u32 {
    let (fa, fb) = (flat(a), flat(b));
    let mut m = 0u32;
    for i in 0..9 {
        m |= u32::from(fa[i] != fb[i]) << i;
    }
    m
}

/// The one place where a pair is counted: skipped (by kind), or counted and compared. The
/// positive control goes through here too, on a scratch `S`. A difference on any compared channel
/// is a mismatch; the event carries the differing axes in `flags` bits 4-12 (`diff_mask`).
fn count_pair(rec: &mut Rec, s: &mut S, a: &RawAll, b: &RawAll, class: Class, mut ev: Ev) {
    match class {
        Class::SkipEdge => {
            s.skipped_edge += 1;
            return;
        }
        Class::SkipLate => {
            s.skipped_late += 1;
            return;
        }
        Class::Counted => {}
    }
    s.counted += 1;
    let m = diff_mask(a, b);
    if m != 0 {
        s.mismatch += 1;
        s.mm_gyro += u32::from(m & 0b000_000_111 != 0);
        s.mm_low += u32::from(m & 0b000_111_000 != 0);
        s.mm_high += u32::from(m & 0b111_000_000 != 0);
        ev.flags |= m << 4;
        rec.event(K_MISMATCH, ev);
    }
}

/// The window rule: at most 100 ms, and paused edges at most duration x rate + 2.
fn win_check(rec: &mut Rec, s: &mut S, ev: Ev, dur_us: u32, paused: u32, rate_hz: u32) {
    let bound = (u64::from(dur_us) * u64::from(rate_hz) / 1_000_000) as u32 + 2;
    if dur_us > 100_000 || paused > bound {
        s.verwin_bad += 1;
        let mut e = ev;
        e.exp = dur_us;
        e.found = paused;
        rec.event(K_VERWIN, e);
    }
}

/// Called when the wait for an edge returns with the hardware count `acc0`: edges that came and
/// went since the last serviced one are missed. Continues from `acc0`.
fn edge_check(missed: &mut u32, rec: &mut Rec, kind: usize, acc0: u64, last: &mut u64, mut ev: Ev) {
    if acc0 > *last + 1 {
        let late = acc0 - *last - 1;
        *missed = missed.saturating_add(late as u32);
        ev.found = late as u32;
        rec.event(kind, ev);
    }
    if acc0 > *last {
        *last = acc0;
    }
}

/// The one place where a FIFO status is judged for an overrun.
fn note_status(s: &mut S, rec: &mut Rec, words: u16, overrun: bool, latched: bool, mut ev: Ev) {
    s.max_unread = s.max_unread.max(u32::from(words));
    if overrun || latched {
        s.overrun += 1;
        ev.exp = u32::from(words);
        ev.found = u32::from(overrun) | u32::from(latched) << 1;
        rec.event(K_OVERRUN, ev);
    }
}

// ---------------------------------------------------------------------------------------------
// Bus, configurations
// ---------------------------------------------------------------------------------------------

fn mkdev(mode3: bool, hz: u32) -> Dev {
    let mut c = spi::Config::default();
    c.frequency = hz;
    if mode3 {
        c.phase = spi::Phase::CaptureOnSecondTransition;
        c.polarity = spi::Polarity::IdleHigh;
    }
    // SAFETY: the main task starts this once; nothing else uses the SPI, its pins or channels.
    let (spi1, clk, mosi, miso, cs, d0, d1) = unsafe {
        (
            SPI1::steal(),
            PIN_10::steal(),
            PIN_11::steal(),
            PIN_8::steal(),
            PIN_9::steal(),
            DMA_CH0::steal(),
            DMA_CH1::steal(),
        )
    };
    let bus = Spi::new(spi1, clk, mosi, miso, d0, d1, Irqs, c);
    // SAFETY: enables the DMA interrupt in the NVIC.
    unsafe { interrupt::DMA_IRQ_0.enable() };
    let cs = Output::new(cs, Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    dev
}

fn mkint() -> (Input<'static>, Pwm<'static>) {
    // SAFETY: the input only awaits edges and the PWM slice only counts them.
    let (p7a, p7b, slice) = unsafe { (PIN_7::steal(), PIN_7::steal(), PWM_SLICE3::steal()) };
    let int1 = Input::new(p7a, Pull::None);
    let pwm = Pwm::new_input(
        slice,
        p7b,
        Pull::None,
        InputMode::RisingEdge,
        pwm::Config::default(),
    );
    (int1, pwm)
}

fn drdy_accel() -> IntRouting {
    IntRouting::new().with_data_ready(DataReady::new().with_accel(true))
}

fn cfg_a() -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_accel_odr(Odr::Hz7680)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_gyro_odr(Odr::Hz7680)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz7680)
                .with_full_scale(HighGFullScale::G32),
        ))
        .with_int1(drdy_accel())
        .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)))
}

/// Phase A2: `cfg_a` at half the rate, all three channels at 3.84 kHz.
fn cfg_a2() -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_accel_odr(Odr::Hz3840)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_gyro_odr(Odr::Hz3840)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz3840)
                .with_full_scale(HighGFullScale::G32),
        ))
        .with_int1(drdy_accel())
        .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)))
}

/// The continuous-mode configuration of lsm6dsk320x_fifo.rs (its `cfg_c`) with the high-g words
/// batched as its `cfg_high_g` does (high-g ODR 960 Hz).
fn cfg_b() -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_accel_odr(Odr::Hz960)
        .with_gyro_odr(Odr::Hz960)
        .with_timestamp(true)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz960)
                .with_full_scale(HighGFullScale::G320),
        ))
        .with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Continuous)
                .with_watermark(WM as u8)
                .with_gyro(BatchRate::Hz960)
                .with_accel(BatchRate::Hz960)
                .with_temperature(TempBatchRate::Hz60)
                .with_high_g(true)
                .with_timestamp(TimestampBatch::Dec8),
        )
        .with_int1(IntRouting::new().with_fifo_threshold(true))
        .with_int2(IntRouting::new())
}

fn m2_g(s: &RawAxes) -> f32 {
    let a = s.accel_ms2(AccelFullScale::G4);
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (G * G)
}

fn hist_add(rec: &mut Rec, m2: f32) {
    let b = (m2 * 100.0) as usize;
    rec.hist[b.min(2047)] += 1;
}

// ---------------------------------------------------------------------------------------------
// Aggressors, in thread mode
// ---------------------------------------------------------------------------------------------

#[embassy_executor::task]
async fn spinner() {
    let mut x = 1u32;
    loop {
        for _ in 0..1000 {
            x = black_box(x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223));
        }
        SPIN_COUNT.fetch_add(1, Ordering::Relaxed);
        let mut y = false;
        poll_fn(|cx| {
            if y {
                Poll::Ready(())
            } else {
                y = true;
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
        .await;
    }
}

static mut SRC: [u32; 1024] = [0; 1024];
static mut DST: [u32; 1024] = [0; 1024];

#[embassy_executor::task]
async fn dma_copy(ch: Peri<'static, DMA_CH2>) {
    let mut ch = dma::Channel::new(ch, Irqs);
    // SAFETY: these two statics are used only by this task.
    let (src, dst) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(SRC),
            &mut *core::ptr::addr_of_mut!(DST),
        )
    };
    for (i, w) in src.iter_mut().enumerate() {
        *w = (i as u32).wrapping_mul(0x9E37_79B9) ^ 0xA5A5_5A5A;
    }
    let mut n = 0u32;
    loop {
        dst[0] = !src[0];
        // SAFETY: both slices are in RAM, 4 KiB each.
        unsafe { ch.copy(&src[..], &mut dst[..]) }.await;
        n = n.wrapping_add(1);
        DMA_COUNT.store(n, Ordering::Relaxed);
        if n % 16 == 0 {
            let bad = src.iter().zip(dst.iter()).filter(|(a, b)| a != b).count() as u32;
            if bad > 0 {
                DMA_BAD.fetch_add(bad, Ordering::Relaxed);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The soak, on the interrupt executor
// ---------------------------------------------------------------------------------------------

/// The progress line, printed by the sampling task inside the minute window (never from thread
/// mode). `s.hw` is the edge count at the last pair's end.
fn progress(s: &S, start: Instant, ph: u8) {
    let t = start.elapsed().as_secs() as u32;
    if ph == 2 {
        let lost = s.hw as i32 - s.reads_b as i32 - s.paused as i32;
        defmt::info!(
            "[STAT] {} s phase B: reads {} words {} intervals {} not 8/8/8 {} bad {} lost {} missed {} | spinner {} dma {}",
            t,
            s.reads_b,
            s.words,
            s.intervals,
            s.intervals_bad,
            s.malformed + s.ts_bad + s.overrun + s.ver_b + s.bus_b,
            lost,
            s.missed_b,
            SPIN_COUNT.load(Ordering::Relaxed),
            DMA_COUNT.load(Ordering::Relaxed)
        );
    } else {
        let lost = s.hw as i32 - s.pairs as i32 - s.paused as i32;
        defmt::info!(
            "[STAT] {} s phase {}: reads or pairs {} (mean {} max {} us) mismatch {} skipped edge {} late {} lost {} missed {} motion samples {} | spinner {} dma {}",
            t,
            if ph == 0 { "A1" } else { "A2" },
            s.pairs,
            s.pair_sum as f32 / CPU_MHZ as f32 / s.pairs.max(1) as f32,
            s.worst_cyc as f32 / CPU_MHZ as f32,
            s.mismatch,
            s.skipped_edge,
            s.skipped_late,
            lost,
            s.missed,
            s.motion,
            SPIN_COUNT.load(Ordering::Relaxed),
            DMA_COUNT.load(Ordering::Relaxed)
        );
    }
}

/// `verify()` and the progress line, in one window: the edges inside it counted as paused, its
/// duration and paused edges checked against the edge rate `rate_hz` of the phase. `before` is the
/// edge count at which the caller knows no edge was waiting (None: counted at the start of the
/// window). `last` continues after the window.
#[allow(clippy::too_many_arguments)]
async fn verify_region(
    imu: &mut Lsm6dsk320x<Dev>,
    hw: &mut Hw<'_>,
    rec: &mut Rec,
    s: &mut S,
    start: Instant,
    idx: u32,
    ph: u8,
    rate_hz: u32,
    last: &mut u64,
    before: Option<u16>,
    ident: bool,
) {
    let b = ph == 2;
    let t_us = start.elapsed().as_micros() as u32;
    let t0 = Instant::now();
    let p0 = match before {
        Some(c) => c,
        None => hw.sync(),
    };
    s.last_p0 = hw.acc;
    let mut ev = EV0;
    ev.idx = idx;
    ev.t_us = t_us;
    if ident {
        let ri = imu.read_register(reg::WHO_AM_I).await;
        s.id_n += 1;
        match ri {
            Ok(v) if v == reg::WHO_AM_I_VALUE => {}
            Ok(v) => {
                s.ident_wrong += 1;
                ev.exp = u32::from(reg::WHO_AM_I_VALUE);
                ev.found = u32::from(v);
                rec.event(K_IDENT, ev);
            }
            Err(_) => {
                s.bus_a += 1;
                ev.found = 4;
                rec.event(K_BUS_A, ev);
            }
        }
    }
    let r = imu.verify().await;
    progress(s, start, ph);
    let dur_us = t0.elapsed().as_micros() as u32;
    let p1 = hw.sync();
    *last = hw.acc;
    let paused = u32::from(p1.wrapping_sub(p0));
    s.paused += paused;
    ev = EV0;
    ev.idx = idx;
    ev.t_us = t_us;
    ev.edges = hw.acc as u32;
    s.verify_n += 1;
    s.ver_max_us = s.ver_max_us.max(dur_us);
    s.ver_max_paused = s.ver_max_paused.max(paused);
    win_check(rec, s, ev, dur_us, paused, rate_hz);
    match r {
        Ok(()) => {}
        Err(Error::ConfigMismatch(m)) => {
            ev.exp = u32::from(m.register) << 16 | u32::from(m.mask) << 8 | u32::from(m.expected);
            ev.found = u32::from(m.found);
            if b {
                s.ver_b += 1;
                rec.event(K_VER_B, ev);
            } else {
                s.ver_a += 1;
                rec.event(K_VER_A, ev);
            }
        }
        Err(_) => {
            ev.found = 5;
            if b {
                s.bus_b += 1;
                rec.event(K_BUS_B, ev);
            } else {
                s.bus_a += 1;
                rec.event(K_BUS_A, ev);
            }
        }
    }
}

/// Arms a wait for an edge: its first poll clears stale edges and enables the edge interrupt;
/// an edge from then on is kept. True if it was already ready.
fn arm<F: Future<Output = ()>>(next: &mut core::pin::Pin<&mut F>) -> bool {
    let mut cx = Context::from_waker(Waker::noop());
    next.as_mut().poll(&mut cx).is_ready()
}

fn word_ctx(buf: &[u8], j: usize) -> [u8; 21] {
    let mut w = [0u8; 21];
    for (k, jj) in [j.checked_sub(1), Some(j), Some(j + 1)]
        .into_iter()
        .enumerate()
    {
        if let Some(jj) = jj {
            let lo = jj * WORD;
            if lo + WORD <= buf.len() {
                w[k * WORD..k * WORD + WORD].copy_from_slice(&buf[lo..lo + WORD]);
            }
        }
    }
    w
}

fn dump(rec: &Rec) {
    for k in 0..KINDS {
        defmt::info!("[STAT] event {}: {} events", NAMES[k], rec.n[k]);
        for e in rec.ev[k].iter().take((rec.n[k] as usize).min(16)) {
            if k == K_MALFORMED || k == K_TS {
                defmt::info!(
                    "[STAT] event   {} read {} t {} us edges {} word-1,word,word+1 {=[u8]:x} expected {} found {}",
                    NAMES[k],
                    e.idx,
                    e.t_us,
                    e.edges,
                    &e.w[..],
                    e.exp,
                    e.found
                );
            } else if k == K_VER_A || k == K_VER_B {
                defmt::info!(
                    "[STAT] event   {} read {} t {} us edges {} reg 0x{=u8:x} mask 0x{=u8:x} expected 0x{=u8:x} found 0x{=u8:x}",
                    NAMES[k],
                    e.idx,
                    e.t_us,
                    e.edges,
                    (e.exp >> 16) as u8,
                    (e.exp >> 8) as u8,
                    e.exp as u8,
                    e.found as u8
                );
            } else if k == K_INTERVAL {
                defmt::info!(
                    "[STAT] event   {} read {} t {} us edges {} gyro {} low-g {} high-g {} words before timestamp word number {} (8 each expected)",
                    NAMES[k],
                    e.idx,
                    e.t_us,
                    e.edges,
                    e.exp >> 16,
                    (e.exp >> 8) & 0xff,
                    e.exp & 0xff,
                    e.found
                );
            } else if k >= K_MISSED_A {
                defmt::info!(
                    "[STAT] event   {} read {} t {} us edges {} expected {} found {} (missed: found = edges that came and went; verify window: expected = duration us, found = paused edges)",
                    NAMES[k],
                    e.idx,
                    e.t_us,
                    e.edges,
                    e.exp,
                    e.found
                );
            } else if k == K_MISMATCH {
                defmt::info!(
                    "[STAT] event   {} read {} t {} us edges {} first {} second {} (gyro xyz, low-g xyz, high-g xyz) | differing axes mask (bit 0-2 gyro xyz, 3-5 low-g xyz, 6-8 high-g xyz) {=u32:b} | ready_at_once {} after_interrupted_pair {} | gap previous pair end to first read {} us, first read {} us, second read {} us, since init {} us | first read equals previous pair's second read {}",
                    NAMES[k],
                    e.idx,
                    e.t_us,
                    e.edges,
                    &e.a[..],
                    &e.b[..],
                    (e.flags >> 4) & 0x1ff,
                    e.flags & 1 != 0,
                    e.flags & 2 != 0,
                    e.gap as f32 / CPU_MHZ as f32,
                    e.d1 as f32 / CPU_MHZ as f32,
                    e.d2 as f32 / CPU_MHZ as f32,
                    e.since_us,
                    e.flags & 8 != 0
                );
            } else {
                defmt::info!(
                    "[STAT] event   {} read {} t {} us edges {} first {} second {} expected {} found {}",
                    NAMES[k],
                    e.idx,
                    e.t_us,
                    e.edges,
                    &e.a[..],
                    &e.b[..],
                    e.exp,
                    e.found
                );
            }
        }
    }
    for i in 0..rec.snap_n {
        defmt::info!(
            "[STAT] event ring before event {} ({}) at read {}, oldest first:",
            i as u32,
            NAMES[rec.snap_kind[i] as usize],
            rec.snap_idx[i]
        );
        for r in rec.snap[i].iter() {
            defmt::info!("[STAT] event   {}", &r[..]);
        }
    }
}

/// Phase A, counted section, one phase: `single` is A1 (one `read_all()` per edge, the way
/// firmware reads), otherwise A2 (a pair of `read_all()` per edge, compared). The pair rule's
/// classification (`Class`) is the same for both: for a single read, "skipped" means a late read.
#[allow(unused_assignments, clippy::too_many_arguments)]
async fn run_a(
    imu: &mut Lsm6dsk320x<Dev>,
    hw: &mut Hw<'_>,
    int1: &mut Input<'static>,
    rec: &mut Rec,
    s: &mut S,
    start: Instant,
    init_us: u32,
    rate_hz: u32,
    single: bool,
    g_us: u32,
) {
    let ph: u8 = if single { 0 } else { 1 };
    let g_cyc = g_us * CPU_MHZ;
    let a_end = Instant::now() + Duration::from_secs(u64::from(P_S));
    rec.dur = [0; 512];
    let a_started = Instant::now();
    hw.sync();
    let hw0 = hw.acc;
    let mut last = hw.acc;
    let mut last_end = hw.acc;
    let mut pending = false;
    let mut ready_next = false;
    let mut t_prev_end = 0u32;
    let mut prev_second: Option<[i16; 9]> = None;
    let mut id_due = false;
    let mut first = true;
    let mut injected = false;
    s.pair_min = u32::MAX;
    s.serve_min = u32::MAX;
    let mut next_verify = Instant::now() + Duration::from_secs(60);
    let mut prev: [Option<[i16; 3]>; 3] = [None; 3];
    let mut stuck_run = [0u32; 3];
    let (mut win_open, mut w_start, mut w_len, mut w_min, mut w_max) =
        (false, 0u32, 0u32, 0f32, 0f32);
    // the first edge: the same fresh-wait logic as after a stale wake, counted from hw0
    let mut have_edge = true;
    let mut rewait = true;
    while have_edge {
        if rewait {
            rewait = false;
            // a stale wake: arm a fresh wait, read the count, wait again; an edge counted
            // since is served at once
            loop {
                if Instant::now() >= a_end || STOP.load(Ordering::Relaxed) {
                    have_edge = false;
                    break;
                }
                let mut n2 = core::pin::pin!(int1.wait_for_rising_edge());
                let rdy = arm(&mut n2);
                hw.sync();
                if hw.acc > last {
                    break;
                }
                if rdy {
                    s.stale += 1;
                    continue;
                }
                if with_timeout(Duration::from_millis(50), n2.as_mut())
                    .await
                    .is_ok()
                {
                    hw.sync();
                    if hw.acc > last {
                        break;
                    }
                    s.stale += 1;
                } else {
                    s.timeouts_a += 1;
                    hw.sync();
                }
            }
            if !have_edge {
                break;
            }
        }
        let t_serve0 = cyc();
        let was_pending = pending;
        let was_ready = ready_next;
        s.ready_now += u32::from(was_ready);
        s.pend_start += u32::from(was_pending);
        let t_us = start.elapsed().as_micros() as u32;
        // the first edge count, before the first transfer starts
        let c0 = hw.sync();
        let acc0 = hw.acc;
        let mut ev = EV0;
        ev.idx = s.pairs + 1;
        ev.t_us = t_us;
        ev.edges = (acc0 - hw0) as u32;
        if first {
            first = false;
            s.first_rel = acc0 - hw0;
        }
        // edges that came and went since the last serviced one are missed; recorded below,
        // inside G, so that recording costs no sample
        let late_edges = if acc0 > last + 1 { acc0 - last - 1 } else { 0 };
        if acc0 > last {
            last = acc0;
        }
        s.missed = s.missed.saturating_add(late_edges as u32);
        let t0 = cyc();
        let r1 = imu.read_all().await;
        let t_mid = cyc();
        let r2 = if single {
            None
        } else {
            Some(imu.read_all().await)
        };
        let t_pair_end = cyc();
        // the count at the second read's return: a change here is "skipped (edge)"
        let c_ret = hw.sync();
        let dc = t_pair_end.wrapping_sub(t0);
        let gap = if s.pairs == 0 {
            0
        } else {
            t0.wrapping_sub(t_prev_end)
        };
        t_prev_end = t_pair_end;
        s.pairs += 1;
        let idx = s.pairs;
        s.worst_cyc = s.worst_cyc.max(dc);
        s.pair_min = s.pair_min.min(dc);
        s.pair_sum += u64::from(dc);
        rec.dur[((dc / CPU_MHZ) as usize).min(511)] += 1;
        // Arm the wait for the next edge now, before the processing, before the second count and
        // before any window: an edge from here on is kept by the wait. Edges are judged by the
        // hardware count, not by the wake: a wake with no edge beyond `last` is stale.
        let mut next = core::pin::pin!(int1.wait_for_rising_edge());
        let armed_ready = arm(&mut next);
        // ---- the pair's own processing, inside G
        let pair = match (&r1, &r2) {
            (Ok(a), Some(Ok(b))) => Some((*a, *b)),
            (Ok(a), None) => Some((*a, *a)),
            _ => None,
        };
        if let Some((a, b)) = &pair {
            ev.a = flat(a);
            ev.b = flat(b);
            rec.push_read(a);
            rec.push_read(b);
            ev.flags = u32::from(was_ready)
                | u32::from(was_pending) << 1
                | u32::from(prev_second == Some(flat(a))) << 3;
            ev.gap = gap;
            ev.d1 = t_mid.wrapping_sub(t0);
            ev.d2 = t_pair_end.wrapping_sub(t_mid);
            ev.since_us = t_us.wrapping_sub(init_us);
            prev_second = Some(flat(b));
            // stuck data, per function
            let f = flat(a);
            for k in 0..3 {
                let v = [f[3 * k], f[3 * k + 1], f[3 * k + 2]];
                if prev[k] == Some(v) {
                    stuck_run[k] += 1;
                    if stuck_run[k] >= 1000 {
                        s.stuck[k] += 1;
                        let mut e = ev;
                        e.found = k as u32;
                        rec.event(K_STUCK, e);
                        stuck_run[k] = 0;
                    }
                } else {
                    stuck_run[k] = 0;
                }
                prev[k] = Some(v);
            }
            // motion: a statistic
            let m2 = m2_g(&a.accel);
            hist_add(rec, m2);
            for v in [a.gyro.x, a.gyro.y, a.gyro.z] {
                s.peak_gyro = s.peak_gyro.max(i32::from(v).abs());
            }
            if !(0.81..=1.21).contains(&m2) {
                s.motion += 1;
                if !win_open {
                    win_open = true;
                    w_start = t_us;
                    w_len = 0;
                    w_min = m2;
                    w_max = m2;
                }
                w_len += 1;
                w_min = w_min.min(m2);
                w_max = w_max.max(m2);
            } else if win_open {
                win_open = false;
                if (s.windows as usize) < 16 {
                    rec.win[s.windows as usize] = Win {
                        start_us: w_start,
                        len: w_len,
                        min: w_min,
                        max: w_max,
                    };
                }
                s.windows += 1;
                s.longest = s.longest.max(w_len);
            }
        } else {
            s.bus_a += 1;
            if let Ok(a) = &r1 {
                ev.a = flat(a);
            }
            if let Some(Ok(b)) = &r2 {
                ev.b = flat(b);
            }
            ev.found = (r1.is_err() as u32) | (matches!(r2, Some(Err(_))) as u32) << 1;
            rec.event(K_BUS_A, ev);
        }
        if late_edges > 0 {
            let mut e = ev;
            e.found = late_edges as u32;
            rec.event(K_MISSED_A, e);
        }
        let t_proc = cyc();
        let post = t_proc.wrapping_sub(t_pair_end);
        if idx <= 8 {
            s.post_cyc[idx as usize - 1] = post;
        } else {
            s.post_worst = s.post_worst.max(post);
        }
        // ---- the second count, no earlier than G after the second read returned
        let mut spins = 0u32;
        while cyc().wrapping_sub(t_pair_end) < g_cyc && spins < 100_000 {
            spins += 1;
        }
        if cyc().wrapping_sub(t_pair_end) < g_cyc {
            s.g_short += 1;
        }
        let c1 = hw.sync();
        let t_serve1 = cyc();
        last_end = hw.acc;
        s.hw = (hw.acc - hw0) as u32;
        let serve = t_serve1.wrapping_sub(t_serve0);
        s.serve_min = s.serve_min.min(serve);
        s.serve_sum += u64::from(serve);
        s.serve_max = s.serve_max.max(serve);
        // An edge before the second count: the pair is skipped and the edge is serviced next,
        // at once, by a pair of its own.
        pending = c1 != c0;
        let class = if c_ret != c0 {
            Class::SkipEdge
        } else if c1 != c0 {
            Class::SkipLate
        } else {
            Class::Counted
        };
        if let Some((a, mut b)) = pair {
            if INJECT && !single && !injected && class == Class::Counted && idx >= 5000 {
                injected = true;
                b.accel.x ^= 1;
                ev.b = flat(&b);
            }
            count_pair(rec, s, &a, &b, class, ev);
        }
        // identity, and verify: never while an interrupting edge waits for its pair
        if !single && idx % 100 == 0 {
            id_due = true;
        }
        if id_due && !pending {
            id_due = false;
            let tw = Instant::now();
            let p0 = c1;
            let r = imu.read_register(reg::WHO_AM_I).await;
            let p1 = hw.sync();
            let dur_us = tw.elapsed().as_micros() as u32;
            let paused = u32::from(p1.wrapping_sub(p0));
            s.paused += paused;
            last = hw.acc;
            last_end = hw.acc;
            s.id_n += 1;
            s.id_max_us = s.id_max_us.max(dur_us);
            s.id_max_paused = s.id_max_paused.max(paused);
            win_check(rec, s, ev, dur_us, paused, rate_hz);
            match r {
                Ok(v) if v == reg::WHO_AM_I_VALUE => {}
                Ok(v) => {
                    s.ident_wrong += 1;
                    ev.exp = u32::from(reg::WHO_AM_I_VALUE);
                    ev.found = u32::from(v);
                    rec.event(K_IDENT, ev);
                }
                Err(_) => {
                    s.bus_a += 1;
                    ev.found = 4;
                    rec.event(K_BUS_A, ev);
                }
            }
        }
        if !pending && Instant::now() >= next_verify {
            next_verify += Duration::from_secs(60);
            let before = Some(hw.last);
            verify_region(
                imu, hw, rec, s, start, idx, ph, rate_hz, &mut last, before, single,
            )
            .await;
            last_end = hw.acc;
        }
        s.hw = (hw.acc - hw0) as u32;
        if !((Instant::now() < a_end && !STOP.load(Ordering::Relaxed)) || pending) {
            break;
        }
        ready_next = !pending && armed_ready;
        have_edge = true;
        if !pending {
            have_edge = loop {
                if armed_ready
                    || with_timeout(Duration::from_millis(50), next.as_mut())
                        .await
                        .is_ok()
                {
                    hw.sync();
                    if hw.acc <= last {
                        s.stale += 1;
                        rewait = true;
                    }
                    break true;
                }
                s.timeouts_a += 1;
                hw.sync();
                if Instant::now() >= a_end || STOP.load(Ordering::Relaxed) {
                    break false;
                }
            };
        }
    }
    // the edges since the last count (c1 of the last read or pair) belong to the final window
    let p_end = hw.last;
    hw.sync();
    s.edges_loop = hw.acc - hw0;
    s.t_loop_us = a_started.elapsed().as_micros() as u64;
    if win_open {
        if (s.windows as usize) < 16 {
            rec.win[s.windows as usize] = Win {
                start_us: w_start,
                len: w_len,
                min: w_min,
                max: w_max,
            };
        }
        s.windows += 1;
        s.longest = s.longest.max(w_len);
    }
    let idx = s.pairs;
    let paused_before = s.paused;
    verify_region(
        imu,
        hw,
        rec,
        s,
        start,
        idx,
        ph,
        rate_hz,
        &mut last,
        Some(p_end),
        single,
    )
    .await;
    s.ver_paused = s.paused - paused_before;
    s.end_gap = s.last_p0 - last_end;
    s.hw = (hw.acc - hw0) as u32;
    s.hw_a = s.hw;
    s.paused_a = s.paused;
    s.a_ran_us = a_started.elapsed().as_micros() as u32;
}

/// The pair-rule control, in the A2 configuration, before its timed section, on scratch counters:
/// 400 pairs, each started a busy delay d after its edge, d stepped evenly from 0 to one period.
/// No counted pair may differ, and at least one pair must be skipped.
async fn ctrl_pair(
    imu: &mut Lsm6dsk320x<Dev>,
    hw: &mut Hw<'_>,
    int1: &mut Input<'static>,
    rec: &mut Rec,
    fails: &mut u32,
    rate_mhz: u32,
    g_us: u32,
) -> bool {
    let saved_n = rec.n;
    let saved_snap = rec.snap_n;
    let g_cyc = g_us * CPU_MHZ;
    let period_cyc = (u64::from(CPU_MHZ) * 1_000_000_000 / u64::from(rate_mhz.max(1))) as u32;
    let mut sc = S::default();
    let (mut sk_diff, mut tmo, mut bus) = (0u32, 0u32, 0u32);
    for i in 0..400u32 {
        if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
            .await
            .is_err()
        {
            tmo += 1;
            continue;
        }
        let d = (u64::from(i) * u64::from(period_cyc) / 400) as u32;
        let t = cyc();
        while cyc().wrapping_sub(t) < d {}
        let c0 = hw.sync();
        let r1 = imu.read_all().await;
        let r2 = imu.read_all().await;
        let t_end = cyc();
        let c_ret = hw.sync();
        while cyc().wrapping_sub(t_end) < g_cyc {}
        let c1 = hw.sync();
        let class = if c_ret != c0 {
            Class::SkipEdge
        } else if c1 != c0 {
            Class::SkipLate
        } else {
            Class::Counted
        };
        match (r1, r2) {
            (Ok(a), Ok(b)) => {
                if class != Class::Counted && diff_mask(&a, &b) != 0 {
                    sk_diff += 1;
                }
                count_pair(rec, &mut sc, &a, &b, class, EV0);
            }
            _ => bus += 1,
        }
    }
    let skipped = sc.skipped_edge + sc.skipped_late;
    defmt::info!(
        "[STAT] pair-rule control (A2 configuration, G {} us): 400 pairs started d = 0 .. one period ({} cycles) after their edge: counted {} (mismatch among them {}), skipped (edge) {}, skipped (late) {}, of the skipped ones that differed (the old rule's false alarms) {}, bus errors {}, edge timeouts {}; scratch counters, not added to the totals",
        g_us,
        period_cyc,
        sc.counted,
        sc.mismatch,
        sc.skipped_edge,
        sc.skipped_late,
        sk_diff,
        bus,
        tmo
    );
    let ok = sc.mismatch == 0 && skipped >= 1 && sc.counted >= 1 && bus == 0 && tmo == 0;
    check!(
        *fails,
        ok,
        "{} pair-rule control: counted {} with {} mismatches (need 0), skipped {} (need at least 1), bus errors {}, timeouts {}",
        sc.counted,
        sc.mismatch,
        skipped,
        bus,
        tmo
    );
    rec.n = saved_n;
    rec.snap_n = saved_snap;
    ok
}

/// Phase B: set-up, the overrun control, and the counted section. False if it cannot go on.
#[allow(unused_assignments, clippy::too_many_arguments)]
async fn phase_b(
    imu: &mut Lsm6dsk320x<Dev>,
    hw: &mut Hw<'_>,
    int1: &mut Input<'static>,
    rec: &mut Rec,
    s: &mut S,
    start: Instant,
    fails: &mut u32,
    ctrl_b_over: &mut u32,
) -> bool {
    let info_b = match imu.init(&mut Delay, &cfg_b()).await {
        Ok(i) => i,
        Err(e) => {
            check!(
                *fails,
                false,
                "{} phase B init: {}",
                defmt::Debug2Format(&e)
            );
            return false;
        }
    };
    let rate_b_hz = info_b.fifo_words_per_second / WM as u32 + 2;
    let mut ev = EV0;
    if imu.verify().await.is_err() || imu.fifo_flush().await.is_err() {
        s.bus_b += 1;
        rec.event(K_BUS_B, ev);
    }

    // ---- positive control B: no read for 200 ms, the FIFO overruns
    let saved_n = rec.n;
    let saved_snap = rec.snap_n;
    let mut sc3 = S::default();
    Timer::after_millis(200).await;
    let st = imu.fifo_status().await;
    let ok3 = match &st {
        Ok(st) => {
            note_status(
                &mut sc3,
                rec,
                st.words,
                st.flags.overrun,
                st.flags.overrun_latched,
                EV0,
            );
            defmt::info!(
                "[STAT] positive control B: no read for 200 ms (the 214-word FIFO fills in about {} ms at {} words/s); status after it: words {}, flags {}, counted as overruns by note_status: {}",
                214_000 / info_b.fifo_words_per_second.max(1),
                info_b.fifo_words_per_second,
                st.words,
                st.flags,
                sc3.overrun
            );
            true
        }
        Err(_) => false,
    };
    *ctrl_b_over = sc3.overrun;
    check!(
        *fails,
        ok3 && *ctrl_b_over >= 1,
        "{} positive control B: the overrun flag was set and counted ({})",
        *ctrl_b_over
    );
    rec.n = saved_n;
    rec.snap_n = saved_snap;
    if !(ok3 && *ctrl_b_over >= 1) {
        return false;
    }
    if imu.fifo_flush().await.is_err() {
        s.bus_b += 1;
        rec.event(K_BUS_B, ev);
    }
    s.fifo_starts = 1;

    // ---- phase B, counted
    let b_end = Instant::now() + Duration::from_secs(u64::from(P_S));
    hw.sync();
    let hw0 = hw.acc;
    let mut last = hw.acc;
    let mut last_end = hw.acc;
    let mut next_verify = Instant::now() + Duration::from_secs(60);
    let mut buf = [0u8; WM * WORD];
    let mut last_ts: Option<u32> = None;
    // words between two timestamp words; counting starts at the first timestamp word after a start
    let (mut n_g, mut n_l, mut n_h, mut iv_on) = (0u32, 0u32, 0u32, false);
    // the first edge: the same fresh-wait logic as after a stale wake, counted from hw0
    let mut have_edge = true;
    let mut rewait = true;
    while have_edge {
        if rewait {
            rewait = false;
            loop {
                if Instant::now() >= b_end || STOP.load(Ordering::Relaxed) {
                    have_edge = false;
                    break;
                }
                let mut n2 = core::pin::pin!(int1.wait_for_rising_edge());
                let rdy = arm(&mut n2);
                hw.sync();
                if hw.acc > last {
                    break;
                }
                if rdy {
                    s.stale += 1;
                    continue;
                }
                if with_timeout(Duration::from_millis(200), n2.as_mut())
                    .await
                    .is_ok()
                {
                    hw.sync();
                    if hw.acc > last {
                        break;
                    }
                    s.stale += 1;
                } else {
                    s.timeouts_b += 1;
                    hw.sync();
                }
            }
            if !have_edge {
                break;
            }
        }
        let t_us = start.elapsed().as_micros() as u32;
        hw.sync();
        ev = EV0;
        ev.idx = s.reads_b + 1;
        ev.t_us = t_us;
        ev.edges = (hw.acc - hw0) as u32;
        edge_check(&mut s.missed_b, rec, K_MISSED_B, hw.acc, &mut last, ev);
        // armed before the read, the status window and any verify window: the count read at the
        // end of the status window is its "after" count
        let mut next = core::pin::pin!(int1.wait_for_rising_edge());
        let nrdy = arm(&mut next);
        'serve: {
            let r = imu.fifo_read_exact(&mut buf, WM).await;
            s.reads_b += 1;
            let idx = s.reads_b;
            if r.is_err() {
                s.bus_b += 1;
                rec.event(K_BUS_B, ev);
                iv_on = false;
                hw.sync();
                last = hw.acc;
                last_end = hw.acc;
                break 'serve;
            }
            s.reads_ok_b += 1;
            if INJECT && idx == 100 {
                buf[3 * WORD] = 0;
            }
            for (j, f) in fifo_frames(&buf).enumerate() {
                s.words += 1;
                ev.w = word_ctx(&buf, j);
                match f {
                    FifoFrame::Gyro(_) => {
                        s.words_g += 1;
                        n_g += 1;
                    }
                    FifoFrame::HighG(_) => {
                        s.words_h += 1;
                        n_h += 1;
                    }
                    FifoFrame::Temperature(_) => s.words_t += 1,
                    FifoFrame::ConfigChange(_) => s.cfg_words += 1,
                    FifoFrame::Accel(smp) => {
                        s.words_l += 1;
                        n_l += 1;
                        let a = smp.axes.accel_ms2(AccelFullScale::G4);
                        hist_add(rec, (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (G * G));
                    }
                    FifoFrame::Timestamp(t) => {
                        if iv_on {
                            s.intervals += 1;
                            if !(n_g == 8 && n_l == 8 && n_h == 8) {
                                s.intervals_bad += 1;
                                let mut e = ev;
                                e.exp = n_g << 16 | n_l << 8 | n_h;
                                e.found = s.intervals;
                                rec.event(K_INTERVAL, e);
                            }
                        }
                        iv_on = true;
                        (n_g, n_l, n_h) = (0, 0, 0);
                        if let Some(p) = last_ts {
                            let d = timestamp_delta_ticks(p, t.ticks);
                            if s.ts_n < 1000 {
                                rec.tsd[s.ts_n as usize] = d;
                                s.ts_n += 1;
                                if s.ts_n == 1000 {
                                    rec.tsd.sort_unstable();
                                    s.ts_med = rec.tsd[500];
                                    s.ts_min = rec.tsd[0];
                                    s.ts_max = rec.tsd[999];
                                    s.ts_set = true;
                                }
                            } else if d.abs_diff(s.ts_med) > 1 {
                                s.ts_bad += 1;
                                ev.exp = s.ts_med;
                                ev.found = d;
                                rec.event(K_TS, ev);
                            }
                        }
                        last_ts = Some(t.ticks);
                    }
                    _ => {
                        s.malformed += 1;
                        rec.event(K_MALFORMED, ev);
                    }
                }
            }
            // the FIFO status after every watermark read
            let p0 = hw.sync();
            let st = imu.fifo_status().await;
            let p1 = hw.sync();
            last = hw.acc;
            last_end = hw.acc;
            s.paused += u32::from(p1.wrapping_sub(p0));
            match st {
                Ok(st) => note_status(
                    s,
                    rec,
                    st.words,
                    st.flags.overrun,
                    st.flags.overrun_latched,
                    ev,
                ),
                Err(_) => {
                    s.bus_b += 1;
                    rec.event(K_BUS_B, ev);
                }
            }
            if Instant::now() >= next_verify {
                next_verify += Duration::from_secs(60);
                s.hw = (hw.acc - hw0) as u32;
                verify_region(
                    imu,
                    hw,
                    rec,
                    s,
                    start,
                    idx,
                    2,
                    rate_b_hz,
                    &mut last,
                    Some(hw.last),
                    false,
                )
                .await;
                last_end = hw.acc;
            }
            s.hw = (hw.acc - hw0) as u32;
        }
        s.hw = (hw.acc - hw0) as u32;
        if !(Instant::now() < b_end && !STOP.load(Ordering::Relaxed)) {
            break;
        }
        have_edge = loop {
            if nrdy
                || with_timeout(Duration::from_millis(200), next.as_mut())
                    .await
                    .is_ok()
            {
                hw.sync();
                if hw.acc <= last {
                    s.stale += 1;
                    rewait = true;
                }
                break true;
            }
            s.timeouts_b += 1;
            hw.sync();
            if Instant::now() >= b_end || STOP.load(Ordering::Relaxed) {
                break false;
            }
        };
    }
    // once more at the end: the status, then verify; the edges since the last count are paused
    let p0 = hw.last;
    let st = imu.fifo_status().await;
    let p1 = hw.sync();
    last = hw.acc;
    last_end = hw.acc;
    s.paused += u32::from(p1.wrapping_sub(p0));
    match st {
        Ok(st) => note_status(
            s,
            rec,
            st.words,
            st.flags.overrun,
            st.flags.overrun_latched,
            ev,
        ),
        Err(_) => {
            s.bus_b += 1;
            rec.event(K_BUS_B, ev);
        }
    }
    let idx = s.reads_b;
    let paused_before = s.paused;
    verify_region(
        imu,
        hw,
        rec,
        s,
        start,
        idx,
        2,
        rate_b_hz,
        &mut last,
        Some(p1),
        false,
    )
    .await;
    s.ver_paused = s.paused - paused_before;
    s.end_gap = s.last_p0 - last_end;
    s.hw = (hw.acc - hw0) as u32;
    s.hw_b = s.hw;
    s.paused_b = s.paused;
    true
}

fn pct(n: u32, d: u32) -> f32 {
    n as f32 * 100.0 / d.max(1) as f32
}

/// The report and the verdicts of one phase A (A1: `single`, A2: pairs).
fn report_a(fails: &mut u32, rec: &Rec, s: &S, single: bool, hz: u32, g_us: u32) {
    let nm = if single { "A1" } else { "A2" };
    let lost = s.hw_a as i32 - s.pairs as i32 - s.paused_a as i32;
    let (med_us, worst_us) = {
        let n: u32 = rec.dur.iter().sum();
        let (mut acc, mut med) = (0u32, 0u32);
        for (i, c) in rec.dur.iter().enumerate() {
            acc += c;
            if acc * 2 >= n.max(1) {
                med = i as u32;
                break;
            }
        }
        (med, s.worst_cyc as f32 / CPU_MHZ as f32)
    };
    let a_s = s.a_ran_us as f32 / 1e6;
    let rate_meas = s.edges_loop as f32 * 1e6 / (s.t_loop_us.max(1)) as f32;
    let rate_info = s.rate_mhz as f32 / 1000.0;
    if INJECT {
        defmt::info!(
            "[STAT] *** INJECTION RUN (SOAK_INJECT=1): the results of {} below are of a deliberately corrupted run ***",
            nm
        );
    }
    let skipped = s.skipped_edge + s.skipped_late;
    let stuck_all = s.stuck[0] + s.stuck[1] + s.stuck[2];
    defmt::info!(
        "[STAT] integrity {}: reads or pairs={} counted={} mismatch={} (with a difference in gyro {} low-g {} high-g {}) skipped_edge={}({}%) skipped_late={}({}%) skipped_total={}({}%){} identity_wrong={} verify_fail={} bus_err={} lost={} missed={} stuck={} (gyro {} low-g {} high-g {}) g_short={}",
        nm,
        s.pairs,
        s.counted,
        s.mismatch,
        s.mm_gyro,
        s.mm_low,
        s.mm_high,
        s.skipped_edge,
        pct(s.skipped_edge, s.pairs),
        s.skipped_late,
        pct(s.skipped_late, s.pairs),
        skipped,
        pct(skipped, s.pairs),
        if single {
            " (these are the late reads)"
        } else {
            ""
        },
        s.ident_wrong,
        s.ver_a,
        s.bus_a,
        lost,
        s.missed,
        stuck_all,
        s.stuck[0],
        s.stuck[1],
        s.stuck[2],
        s.g_short
    );
    let cyc_us = |c: u64, n: u32| c as f32 / CPU_MHZ as f32 / n.max(1) as f32;
    defmt::info!(
        "[STAT] {} {} at {} Hz, SPI mode 3: min {} mean {} max {} us over {}; serve (task start after the edge wake-up, first edge count, to the second edge count, G = {} us included; the edge itself has no clock): min {} mean {} max {} us; processing inside G (read end to the start of the wait): first 8 {} worst of the rest {} us; edge period 131.9 us at 7.68 kHz, about 265 us at 3.84 kHz",
        nm,
        if single {
            "read_all single read (start to return)"
        } else {
            "read_all pair (first read start to second read return)"
        },
        hz,
        s.pair_min as f32 / CPU_MHZ as f32,
        cyc_us(s.pair_sum, s.pairs),
        worst_us,
        s.pairs,
        g_us,
        s.serve_min as f32 / CPU_MHZ as f32,
        cyc_us(s.serve_sum, s.pairs),
        s.serve_max as f32 / CPU_MHZ as f32,
        s.post_cyc.map(|c| c as f32 / CPU_MHZ as f32),
        s.post_worst as f32 / CPU_MHZ as f32
    );
    let peak = RawAxes {
        x: s.peak_gyro.min(32767) as i16,
        y: 0,
        z: 0,
    }
    .gyro_rads(GyroFullScale::Dps500)[0]
        * DEG;
    defmt::info!(
        "[STAT] motion {}: samples={} windows={} longest={} peak_gyro={} dps",
        nm,
        s.motion,
        s.windows,
        s.longest,
        peak
    );
    for i in 0..(s.windows as usize).min(16) {
        let w = &rec.win[i];
        defmt::info!(
            "[STAT] motion {} window {}: start {} us, {} samples, |a|^2 min {} max {} g^2",
            nm,
            i as u32,
            w.start_us,
            w.len,
            w.min,
            w.max
        );
    }
    defmt::info!(
        "[STAT] rate {}: {} edges in {} s = {} Hz measured; the driver's info() says {} Hz; {} reads or pairs served; time median {} us, worst {} us; started on the ready-at-once path {} of {} ({} %); started right after an interrupted one: {}",
        nm,
        s.edges_loop as u32,
        s.t_loop_us as f32 / 1e6,
        rate_meas,
        rate_info,
        s.pairs,
        med_us,
        worst_us,
        s.ready_now,
        s.pairs,
        pct(s.ready_now, s.pairs),
        s.pend_start
    );
    defmt::info!(
        "[STAT] windows {}: verify (with the progress line) {} calls, longest {} us, most paused edges {}; identity reads {}, longest {} us, most paused edges {}; windows over their bound or over 100 ms {}",
        nm,
        s.verify_n,
        s.ver_max_us,
        s.ver_max_paused,
        s.id_n,
        s.id_max_us,
        s.id_max_paused,
        s.verwin_bad
    );
    defmt::info!(
        "[STAT] diag {}: first service at edge {} (expect 1); counted {}; edges {}, paused {} (final verify {}), edges between the last one and the final verify {}, lost {}; wall {} s; stale wakes discarded {}",
        nm,
        s.first_rel as u32,
        s.counted,
        s.hw_a,
        s.paused_a,
        s.ver_paused,
        s.end_gap as u32,
        lost,
        a_s,
        s.stale
    );
    if single {
        check!(
            *fails,
            s.pairs > 0 && skipped * 200 <= s.pairs,
            "{} A1 late reads {} (edge {}, late {}) of {} reads (at most 0.5 %)",
            skipped,
            s.skipped_edge,
            s.skipped_late,
            s.pairs
        );
    } else {
        check!(
            *fails,
            s.counted > 0 && s.mismatch == 0,
            "{} A2 mismatch = {} among {} counted pairs ({} pairs)",
            s.mismatch,
            s.counted,
            s.pairs
        );
        check!(
            *fails,
            skipped * 100 <= s.pairs,
            "{} A2 skipped {} (edge {}, late {}) of {} pairs (at most 1 %)",
            skipped,
            s.skipped_edge,
            s.skipped_late,
            s.pairs
        );
    }
    check!(
        *fails,
        s.g_short == 0,
        "{} {} the guard time G = {} us was waited out before every second count (short waits {})",
        nm,
        g_us,
        s.g_short
    );
    check!(
        *fails,
        s.ident_wrong == 0,
        "{} {} identity_wrong = {}",
        nm,
        s.ident_wrong
    );
    check!(*fails, s.ver_a == 0, "{} {} verify_fail = {}", nm, s.ver_a);
    check!(*fails, s.bus_a == 0, "{} {} bus_err = {}", nm, s.bus_a);
    check!(
        *fails,
        lost == 0 && s.timeouts_a == 0,
        "{} {} lost = {} (must be exactly 0; a negative value is a bookkeeping error), edge timeouts {}",
        nm,
        lost,
        s.timeouts_a
    );
    check!(
        *fails,
        s.missed == 0,
        "{} {} missed = {} (edges that came and went unserviced)",
        nm,
        s.missed
    );
    check!(
        *fails,
        stuck_all == 0,
        "{} {} stuck = {} (gyro {} low-g {} high-g {})",
        nm,
        stuck_all,
        s.stuck[0],
        s.stuck[1],
        s.stuck[2]
    );
    check!(
        *fails,
        rate_info > 0.0 && (rate_meas / rate_info - 1.0).abs() <= 0.05,
        "{} {} edge rate {} Hz within 5 % of the info() rate {} Hz",
        nm,
        rate_meas,
        rate_info
    );
    check!(
        *fails,
        s.pairs as u64 * 100 >= s.edges_loop * 99 && s.edges_loop > 0,
        "{} {} reads or pairs {} are at least 99 % of the {} edges",
        nm,
        s.pairs,
        s.edges_loop as u32
    );
    check!(
        *fails,
        s.verwin_bad == 0 && s.verify_n >= 1 + (P_S - 1) / 60,
        "{} {} windows: {} verify calls (at least {}), longest {} us (limit 100000), most paused {}; {} identity reads, longest {} us, most paused {}; bad windows {}",
        nm,
        s.verify_n,
        1 + (P_S - 1) / 60,
        s.ver_max_us,
        s.ver_max_paused,
        s.id_n,
        s.id_max_us,
        s.id_max_paused,
        s.verwin_bad
    );
}

/// The report and the verdicts of phase B.
fn report_b(fails: &mut u32, s: &S) {
    let lost_b = s.hw_b as i32 - s.reads_b as i32 - s.paused_b as i32;
    if INJECT {
        defmt::info!(
            "[STAT] *** INJECTION RUN (SOAK_INJECT=1): the results of B below are of a deliberately corrupted run ***"
        );
    }
    defmt::info!(
        "[STAT] integrity B: words={} (gyro {} low-g {} high-g {} temperature {}) intervals checked={} not 8/8/8={} malformed={} timestamp_bad={} overrun={} verify_fail={} lost={} missed={} bus_err={} edge_timeouts={}",
        s.words,
        s.words_g,
        s.words_l,
        s.words_h,
        s.words_t,
        s.intervals,
        s.intervals_bad,
        s.malformed,
        s.ts_bad,
        s.overrun,
        s.ver_b,
        lost_b,
        s.missed_b,
        s.bus_b,
        s.timeouts_b
    );
    defmt::info!(
        "[STAT] diag B: reads {} of {} edges, paused {} (final verify {}), edges between the last read and the final verify {}, lost {}, config-change words {} (FIFO starts and flushes {}), max unread {}, timestamp steps in the first 1000: median {} min {} max {} ticks (set {}), words {} (reads x {} = {})",
        s.reads_b,
        s.hw_b,
        s.paused_b,
        s.ver_paused,
        s.end_gap as u32,
        lost_b,
        s.cfg_words,
        s.fifo_starts,
        s.max_unread,
        s.ts_med,
        s.ts_min,
        s.ts_max,
        s.ts_set,
        s.words,
        WM as u32,
        s.reads_ok_b * WM as u32
    );
    check!(
        *fails,
        s.words > 0
            && s.malformed == 0
            && s.ts_bad == 0
            && s.overrun == 0
            && s.ver_b == 0
            && s.bus_b == 0
            && lost_b == 0
            && s.missed_b == 0
            && s.timeouts_b == 0,
        "{} phase B all zero: words {} malformed {} timestamp_bad {} overrun {} verify_fail {} bus_err {} lost {} missed {} timeouts {}",
        s.words,
        s.malformed,
        s.ts_bad,
        s.overrun,
        s.ver_b,
        s.bus_b,
        lost_b,
        s.missed_b,
        s.timeouts_b
    );
    check!(
        *fails,
        s.intervals > 0 && s.intervals_bad == 0 && s.words_h > 0,
        "{} phase B: {} intervals between timestamp words checked, {} not exactly 8 gyro + 8 low-g + 8 high-g words; high-g words {}",
        s.intervals,
        s.intervals_bad,
        s.words_h
    );
    check!(
        *fails,
        s.ts_set,
        "{} phase B: the median timestamp step was found ({} steps seen, 1000 needed)",
        s.ts_n
    );
    check!(
        *fails,
        s.words == s.reads_ok_b * WM as u32,
        "{} phase B: words {} equal reads {} x watermark {}",
        s.words,
        s.reads_ok_b,
        WM as u32
    );
    // Two per start or flush: the documentation of `FifoConfigChange` (src/fifo.rs) records, from the
    // board on 2026-11-10, a configuration-change word as the first word after every FIFO start or
    // restart, and "a second one ... seen once a few words later". A healthy sensor can show both.
    check!(
        *fails,
        s.cfg_words <= 2 * s.fifo_starts,
        "{} phase B: configuration-change words {} at most two per FIFO start or flush ({} starts and flushes)",
        s.cfg_words,
        s.fifo_starts
    );
}

#[allow(unused_assignments)]
#[embassy_executor::task]
async fn soak(start: Instant) {
    dwt_on();
    // SAFETY: this task is the only user of REC while it runs; thread mode never touches it.
    let rec = unsafe { &mut *core::ptr::addr_of_mut!(REC) };
    let (mut s1, mut s2, mut s3) = (S::default(), S::default(), S::default());
    let mut fails = 0u32;
    let mut run_ok = true;
    let (mut ctrl_a2, mut ctrl_b_over) = (0u32, 0u32);

    if INJECT {
        defmt::info!(
            "[STAT] *** INJECTION RUN (SOAK_INJECT=1): one pair of phase A2 and one FIFO word of phase B are corrupted on purpose; this is NOT a real result ***"
        );
    }
    let dev = mkdev(true, REQ_SOAK_HZ);
    let hz = spi_hz();
    let (mut int1, pwm) = mkint();
    let mut hw = Hw {
        pwm: &pwm,
        last: pwm.counter(),
        acc: 0,
    };
    defmt::info!(
        "[INFO] soak {} s: phases A1, A2 and B of {} s each; SPI mode 3 at {} Hz (requested {}); clk_peri {} Hz; G A1 {} us, G A2 {} us",
        TOTAL_S,
        P_S,
        hz,
        REQ_SOAK_HZ,
        embassy_rp::clocks::clk_peri_freq(),
        G1_US,
        G2_US
    );
    if hz > 10_000_000 {
        check!(
            fails,
            false,
            "{} SPI clock {} Hz is above 10 MHz: no sensor access",
            hz
        );
        run_ok = false;
    }
    let mut imu = Lsm6dsk320x::new(dev);

    'run: {
        if !run_ok {
            break 'run;
        }
        // ---- phase A1: set-up
        let info_a = match imu.init(&mut Delay, &cfg_a()).await {
            Ok(i) => i,
            Err(e) => {
                check!(
                    fails,
                    false,
                    "{} phase A1 init: {}",
                    defmt::Debug2Format(&e)
                );
                run_ok = false;
                break 'run;
            }
        };
        s1.rate_mhz = info_a.accel_odr_mhz;
        let init_us = start.elapsed().as_micros() as u32;
        let rate_hz = s1.rate_mhz / 1000 + 1;
        if let Err(e) = imu.verify().await {
            check!(
                fails,
                false,
                "{} phase A1 verify after init: {}",
                defmt::Debug2Format(&e)
            );
            run_ok = false;
            break 'run;
        }
        // ---- positive controls A, before the counted section, on scratch counters
        let saved_n = rec.n;
        let saved_snap = rec.snap_n;
        let mut sc = S::default();
        let mut got = false;
        let mut tries = 0u32;
        for _ in 0..40 {
            if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
                .await
                .is_err()
            {
                continue;
            }
            let c0 = hw.sync();
            let r1 = imu.read_all().await;
            let mut b = [0u8; RawAll::LEN];
            let r2 = imu.read_registers(reg::OUT_TEMP_L + 1, &mut b).await;
            let c1 = hw.sync();
            tries += 1;
            if let (Ok(a), Ok(())) = (r1, r2) {
                if c0 == c1 {
                    count_pair(
                        rec,
                        &mut sc,
                        &a,
                        &RawAll::from_bytes(&b),
                        Class::Counted,
                        EV0,
                    );
                    got = true;
                    break;
                }
            }
        }
        s1.ctrl_flagged = (got && sc.mismatch == 1) as u32;
        defmt::info!(
            "[STAT] positive control A1: read_all() against a raw read one register later (OUT_TEMP_L+1), counted pair in {} tries, through count_pair on a scratch struct: mismatches {} (expected exactly 1; not added to the totals)",
            tries,
            sc.mismatch
        );
        check!(
            fails,
            s1.ctrl_flagged == 1,
            "{} positive control A1: pair mismatch flagged exactly once ({})",
            s1.ctrl_flagged
        );
        if s1.ctrl_flagged != 1 {
            run_ok = false;
            break 'run;
        }
        let mut sc2 = S::default();
        hw.sync();
        let mut lastc = hw.acc;
        let mut ok2 = true;
        for step in 0..2 {
            if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
                .await
                .is_err()
            {
                ok2 = false;
                break;
            }
            hw.sync();
            edge_check(&mut sc2.missed, rec, K_MISSED_A, hw.acc, &mut lastc, EV0);
            let r1 = imu.read_all().await;
            let r2 = imu.read_all().await;
            ok2 &= r1.is_ok() && r2.is_ok();
            if step == 0 {
                Timer::after_millis(2).await;
            }
        }
        ctrl_a2 = sc2.missed;
        defmt::info!(
            "[STAT] positive control A2: a plain 2 ms timer wait between two pairs (edge period about 133 us): missed rose by {} (needs at least 5; not added to the totals)",
            ctrl_a2
        );
        check!(
            fails,
            ok2 && ctrl_a2 >= 5,
            "{} positive control A2: missed rose by {} (at least 5)",
            ctrl_a2
        );
        rec.n = saved_n;
        rec.snap_n = saved_snap;
        if !(ok2 && ctrl_a2 >= 5) {
            run_ok = false;
            break 'run;
        }

        // ---- phase A1, counted
        run_a(
            &mut imu, &mut hw, &mut int1, rec, &mut s1, start, init_us, rate_hz, true, G1_US,
        )
        .await;
        report_a(&mut fails, rec, &s1, true, hz, G1_US);

        // ---- phase A2: set-up, the pair-rule control, counted
        let info_a2 = match imu.init(&mut Delay, &cfg_a2()).await {
            Ok(i) => i,
            Err(e) => {
                check!(
                    fails,
                    false,
                    "{} phase A2 init: {}",
                    defmt::Debug2Format(&e)
                );
                run_ok = false;
                break 'run;
            }
        };
        s2.rate_mhz = info_a2.accel_odr_mhz;
        let init2_us = start.elapsed().as_micros() as u32;
        let rate2_hz = s2.rate_mhz / 1000 + 1;
        if let Err(e) = imu.verify().await {
            check!(
                fails,
                false,
                "{} phase A2 verify after init: {}",
                defmt::Debug2Format(&e)
            );
            run_ok = false;
            break 'run;
        }
        if !ctrl_pair(
            &mut imu,
            &mut hw,
            &mut int1,
            rec,
            &mut fails,
            s2.rate_mhz,
            G2_US,
        )
        .await
        {
            run_ok = false;
            break 'run;
        }
        run_a(
            &mut imu, &mut hw, &mut int1, rec, &mut s2, start, init2_us, rate2_hz, false, G2_US,
        )
        .await;
        report_a(&mut fails, rec, &s2, false, hz, G2_US);

        // ---- phase B
        if !phase_b(
            &mut imu,
            &mut hw,
            &mut int1,
            rec,
            &mut s3,
            start,
            &mut fails,
            &mut ctrl_b_over,
        )
        .await
        {
            run_ok = false;
            break 'run;
        }
        report_b(&mut fails, &s3);
    }
    let _ = run_ok;

    // ---- end of the timed sections
    defmt::info!(
        "[STAT] positive controls: A1 mismatch flagged {}, A2 missed rose by {}, B overruns counted {}",
        s1.ctrl_flagged,
        ctrl_a2,
        ctrl_b_over
    );
    let (dma_n, dma_bad) = (
        DMA_COUNT.load(Ordering::Relaxed),
        DMA_BAD.load(Ordering::Relaxed),
    );
    defmt::info!(
        "[STAT] aggressors: thread-mode spinner blocks {}, DMA 4 KiB copies {} (words wrong when compared, every 16th copy: {})",
        SPIN_COUNT.load(Ordering::Relaxed),
        dma_n,
        dma_bad
    );
    dump(rec);

    // median magnitude over the whole run (histogram of |a|^2, 0.01 g^2 bins)
    let n: u32 = rec.hist.iter().sum();
    let mut acc = 0u32;
    let mut mb = 0usize;
    for (i, c) in rec.hist.iter().enumerate() {
        acc += c;
        if acc * 2 >= n.max(1) {
            mb = i;
            break;
        }
    }
    let med_mag = sqrt((mb as f32 + 0.5) / 100.0);
    if INJECT {
        defmt::info!(
            "[STAT] *** RESULT SECTION OF AN INJECTION RUN (SOAK_INJECT=1): expected to FAIL ***"
        );
    }
    check!(
        fails,
        n > 0 && (0.9..=1.1).contains(&med_mag),
        "{} median |a| over the run {} g (0.9 to 1.1), {} samples",
        med_mag,
        n
    );
    check!(
        fails,
        SPIN_COUNT.load(Ordering::Relaxed) > 0 && dma_n > 0 && dma_bad == 0,
        "{} aggressors ran: spinner {} blocks, DMA {} copies, wrong words {}",
        SPIN_COUNT.load(Ordering::Relaxed),
        dma_n,
        dma_bad
    );

    // ---- end: the default configuration in the smoke test's bus setting, INT1 routed to the
    // accelerometer's data-ready (the default routes nothing) so that samples are read per edge
    let mut dev = imu.release();
    let mut c0 = spi::Config::default();
    c0.frequency = REQ_SMOKE_HZ;
    dev.bus_mut().set_config(&c0);
    let hz0 = spi_hz();
    let mut imu = Lsm6dsk320x::new(dev);
    let cfg_end = Config::new().with_int1(drdy_accel());
    match imu.init(&mut Delay, &cfg_end).await {
        Err(e) => check!(fails, false, "{} final init: {}", defmt::Debug2Format(&e)),
        Ok(_) => {
            let ver = imu.verify().await;
            let (mut served, mut good, mut bad, mut tmo) = (0u32, 0u32, 0u32, 0u32);
            for _ in 0..400 {
                if served >= 200 {
                    break;
                }
                if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
                    .await
                    .is_err()
                {
                    tmo += 1;
                    continue;
                }
                match imu.read_all().await {
                    Ok(a) => {
                        served += 1;
                        let ac = a.accel.accel_ms2(AccelFullScale::G8);
                        let m2 = ac[0] * ac[0] + ac[1] * ac[1] + ac[2] * ac[2];
                        let gy = a.gyro.gyro_rads(GyroFullScale::Dps2000);
                        let gmax = gy.iter().fold(0f32, |m, v| m.max(v.abs())) * DEG;
                        let t = a.temperature_c();
                        if m2 >= 0.81 * G * G
                            && m2 <= 1.21 * G * G
                            && gmax <= 20.0
                            && (10.0..=50.0).contains(&t)
                        {
                            good += 1;
                        }
                    }
                    Err(_) => bad += 1,
                }
            }
            let id = imu.read_register(reg::WHO_AM_I).await;
            // A bump must not fail this: bus errors and missing edges do, implausible samples
            // only when more than half of them are.
            check!(
                fails,
                ver.is_ok()
                    && bad == 0
                    && tmo == 0
                    && served == 200
                    && good >= 100
                    && id.ok() == Some(reg::WHO_AM_I_VALUE),
                "{} sensor at the end ({} Hz, mode 0): default config with INT1 on the accelerometer data-ready, verify ok {}, samples served on edges {} of 200, plausible {} (at least 100), bus errors {}, edge timeouts {}",
                hz0,
                ver.is_ok(),
                served,
                good,
                bad,
                tmo
            );
        }
    }
    FAILS.store(fails, Ordering::Relaxed);
    DONE.store(true, Ordering::Release);
}

static EXEC_HI: InterruptExecutor = InterruptExecutor::new();

#[embassy_rp::pac::interrupt]
unsafe fn SWI_IRQ_1() {
    // SAFETY: the interrupt executor is started on this interrupt.
    unsafe { EXEC_HI.on_interrupt() }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    dwt_on();
    let start = Instant::now();
    if INJECT {
        defmt::info!(
            "[STAT] *** INJECTION RUN (SOAK_INJECT=1): this is not a real soak; it must FAIL ***"
        );
    }
    defmt::info!(
        "[INFO] clk_sys {} Hz; SOAK_SECONDS {}",
        embassy_rp::clocks::clk_sys_freq(),
        TOTAL_S
    );
    spawner.spawn(spinner().unwrap());
    spawner.spawn(dma_copy(p.DMA_CH2).unwrap());
    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let hi = EXEC_HI.start(interrupt::SWI_IRQ_1);
    hi.spawn(soak(start).unwrap());

    let mut hung = false;
    loop {
        Timer::after_millis(250).await;
        if DONE.load(Ordering::Acquire) {
            break;
        }
        let s = start.elapsed().as_secs();
        if s >= u64::from(TOTAL_S) + 15 {
            STOP.store(true, Ordering::Relaxed);
        }
        if s >= u64::from(TOTAL_S) + 28 {
            hung = true;
            break;
        }
    }
    let mut fails = FAILS.load(Ordering::Relaxed);
    if hung {
        fails += 1;
        defmt::info!("[FAIL] the soak did not finish within SOAK_SECONDS + 28 s");
    }
    if INJECT {
        defmt::info!(
            "[STAT] *** INJECTION RUN (SOAK_INJECT=1): the verdict below is of a deliberately corrupted run ***"
        );
    }
    defmt::info!("[INFO] run time {} ms", start.elapsed().as_millis() as u32);
    hil_common::exit(fails)
}

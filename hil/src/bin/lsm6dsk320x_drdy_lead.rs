//! On-board measurement (U23 on SPI1, INT1 on GPIO7, INT2 on GPIO6): when do the output registers
//! of the three channels change relative to the INT1 data-ready edge seen at the MCU pin?
//!
//! Both functions of the chip run: gyro + low-g accelerometer at 7.68 kHz and the high-g
//! accelerometer at 7.68 kHz; INT1 = low-g data-ready, INT2 = high-g data-ready. SPI mode 3,
//! 9.375 MHz. Nothing here is a pass criterion except 2000 classified edges per channel and a
//! healthy sensor at the end; the answers are `[STAT]` lines.
//!
//! Q1 method (phase scan, interrupts masked during each cycle): spin on the INT1 level, timestamp
//! rising edge N with the DWT counter, read one channel's six output bytes twice (sample N, the
//! cycle is dropped when they differ), busy-wait to an offset after edge N, do ONE probe read of
//! the same six bytes straight on the SPI registers (this measures the device, not the driver),
//! keep polling the pins during the probe and until edge N+1 is seen (the pin is sampled
//! every few cycles, also while bytes are clocked, so edge N+1 is measured, never estimated), then
//! read twice again (sample N+1). Every probe byte that differs between N and N+1 is classified
//! old/new/neither and binned by the time at which that byte was clocked, relative to edge N+1.
//! The clock time of a byte is the cycle count before the first data-register write plus a
//! calibrated latency to the first falling SCLK edge (SCLK is read through SIO GPIO_IN) plus whole
//! bytes; the sensor is assumed to latch data byte k at the last rising SCLK edge of the byte before.
//! Q2/Q4: a polling loop timestamps both pins' rising edges for 2 s. Q3: one read of all nine
//! axes 50 us after every INT1 edge (far from the change), consecutive samples compared.
#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::interrupt::InterruptExt;
use embassy_rp::peripherals::{
    DMA_CH0, DMA_CH1, PIN_6, PIN_7, PIN_8, PIN_9, PIN_10, PIN_11, PWM_SLICE3, SPI1,
};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{bind_interrupts, dma, pac};
use embassy_time::{Delay, Duration, Instant, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::sample::G;
use lsm6dsk320x::{
    AccelFullScale, Config, DataReady, GyroFullScale, HighGConfig, HighGFullScale, HighGOdr,
    IntRouting, Lsm6dsk320x, Odr, reg,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

type Dev = ExclusiveDevice<Spi<'static, SPI1, spi::Async>, Output<'static>, Delay>;

const fn is_one(s: Option<&str>) -> bool {
    match s {
        Some(s) => s.len() == 1 && s.as_bytes()[0] == b'1',
        None => false,
    }
}

const fn is_zero(s: Option<&str>) -> bool {
    match s {
        Some(s) => s.len() == 1 && s.as_bytes()[0] == b'0',
        None => false,
    }
}

/// `LEAD_HIGH_G=0` at build time (any other value or none is on) runs the soak's old configuration
/// (`cfg_a` of lsm6dsk320x_soak.rs: high-g off, INT2 unrouted) and skips the high-g channel and Q2.
const HIGH_G: bool = !is_zero(option_env!("LEAD_HIGH_G"));
/// `LEAD_HZ3840=1` at build time (any other value or none is off) runs every channel that is on at
/// 3.84 kHz instead of 7.68 kHz (the soak's phase A2); the edge period is then about 265 us.
const SLOW: bool = is_one(option_env!("LEAD_HZ3840"));
const ODR: Odr = if SLOW { Odr::Hz3840 } else { Odr::Hz7680 };
const HG_ODR: HighGOdr = if SLOW {
    HighGOdr::Hz3840
} else {
    HighGOdr::Hz7680
};
const NCH: usize = if HIGH_G { 3 } else { 2 };
const NAX: usize = 3 * NCH;
const REQ_SOAK_HZ: u32 = 9_500_000; // gives 9.375 MHz, SPI mode 3
const REQ_SMOKE_HZ: u32 = 8_000_000; // gives 7.5 MHz, SPI mode 0, as the smoke test
const CPU_MHZ: u32 = 150;
const DEG: f32 = 180.0 / core::f32::consts::PI;

const INT1: u32 = 1 << 7;
const INT2: u32 = 1 << 6;
const SCLK: u32 = 1 << 10;
const CSB: u32 = 1 << 9;

const US: u32 = CPU_MHZ;
const PRE_DELAY: u32 = 8 * US;
const POST_DELAY: u32 = 20 * US;
const REST_DELAY: u32 = 50 * US;
const WAIT_EDGE_TO: u32 = 500 * US;
const EDGE_TO: u32 = if SLOW { 430 * US } else { 215 * US };
const MIN_P: u32 = if SLOW { 180 * US } else { 90 * US };
const MAX_P: u32 = if SLOW { 400 * US } else { 200 * US };
const MIN_EDGES_2S: u32 = if SLOW { 5_000 } else { 10_000 };
const PER_N: usize = if SLOW { 4000 } else { 2000 };

/// Q1 histogram: 0.5 us bins from REL_MIN_US to REL_MAX_US relative to edge N+1.
const REL_MIN_US: i32 = -110;
const REL_MAX_US: i32 = 40;
const BIN_CYC: i32 = (US / 2) as i32;
const NB: usize = ((REL_MAX_US - REL_MIN_US) * 2) as usize;
/// Probe targets (centre of the six data bytes relative to edge N+1), 1 us steps.
const SCAN_LO_US: i32 = -95;
const SCAN_STEPS: u32 = 116;
const Q1_TARGET: u32 = 15_000;
const Q1_MAX_S: u64 = 60;
const MIN_CLASSIFIED: u32 = 2000;
const Q3_SAMPLES: u32 = 8000;

const NAMES: [&str; 3] = ["gyro", "low-g", "high-g"];
const BASES: [u8; 3] = [reg::OUTX_L_G, reg::OUTX_L_A, reg::UI_OUTX_L_A_OIS_HG];
const AXN: [&str; 9] = ["gx", "gy", "gz", "ax", "ay", "az", "hx", "hy", "hz"];

macro_rules! check {
    ($fails:expr, $ok:expr, $fmt:literal $(, $arg:expr)*) => {{
        let ok = $ok;
        if !ok {
            $fails += 1;
        }
        defmt::info!($fmt, if ok { "[PASS]" } else { "[FAIL]" } $(, $arg)*);
    }};
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

/// SSPCLK cycles per SPI bit.
fn spi_div() -> u32 {
    let p = pac::SPI1;
    p.cpsr().read().cpsdvsr() as u32 * (1 + p.cr0().read().scr() as u32)
}

fn spi_hz() -> u32 {
    embassy_rp::clocks::clk_peri_freq() / spi_div()
}

fn gin() -> u32 {
    pac::SIO.gpio_in(0).read()
}

fn cs_low() {
    pac::SIO.gpio_out(0).value_clr().write_value(CSB);
}

fn cs_high() {
    pac::SIO.gpio_out(0).value_set().write_value(CSB);
}

fn us_f(c: i32) -> f32 {
    c as f32 / US as f32
}

// ---------------------------------------------------------------------------------------------
// Bus, configuration (copied from the soak)
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
    unsafe { embassy_rp::interrupt::DMA_IRQ_0.enable() };
    let cs = Output::new(cs, Level::High);
    let Ok(dev) = ExclusiveDevice::new(bus, cs, Delay);
    dev
}

fn mkint() -> (Input<'static>, Pwm<'static>, Input<'static>) {
    // SAFETY: the input only awaits edges and the PWM slice only counts them.
    let (p7a, p7b, slice, p6) = unsafe {
        (
            PIN_7::steal(),
            PIN_7::steal(),
            PWM_SLICE3::steal(),
            PIN_6::steal(),
        )
    };
    let int1 = Input::new(p7a, Pull::None);
    let pwm = Pwm::new_input(
        slice,
        p7b,
        Pull::None,
        InputMode::RisingEdge,
        pwm::Config::default(),
    );
    let int2 = Input::new(p6, Pull::None);
    (int1, pwm, int2)
}

fn drdy_accel() -> IntRouting {
    IntRouting::new().with_data_ready(DataReady::new().with_accel(true))
}

/// The soak's `cfg_a` with the high-g channel on at 7.68 kHz (as lsm6dsk320x_fifo.rs `cfg_b`),
/// INT1 = low-g data-ready, INT2 = high-g data-ready.
fn cfg_lead() -> Config {
    if !HIGH_G {
        return Config::new()
            .with_accel_fs(AccelFullScale::G4)
            .with_accel_odr(ODR)
            .with_gyro_fs(GyroFullScale::Dps500)
            .with_gyro_odr(ODR)
            .with_high_g(None)
            .with_int1(drdy_accel())
            .with_int2(IntRouting::new());
    }
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_accel_odr(ODR)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_gyro_odr(ODR)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HG_ODR)
                .with_full_scale(HighGFullScale::G32),
        ))
        .with_int1(drdy_accel())
        .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)))
}

// ---------------------------------------------------------------------------------------------
// Raw SPI transfer with pin polling
// ---------------------------------------------------------------------------------------------

/// Rising-edge detector over INT1 and INT2, fed by `poll`.
struct Ev {
    prev: u32,
    seen1: bool,
    t1: u32,
}

impl Ev {
    fn new() -> Self {
        Ev {
            prev: gin(),
            seen1: false,
            t1: 0,
        }
    }

    #[inline(always)]
    fn poll(&mut self) {
        let now = gin();
        let t = cyc();
        if now & !self.prev & INT1 != 0 && !self.seen1 {
            self.seen1 = true;
            self.t1 = t;
        }
        self.prev = now;
    }
}

/// One read transaction of `out.len()` bytes from `addr`, straight on the SPI1 registers, the pins
/// polled all the while. Returns the cycle count taken just before the address byte was written.
fn xfer(addr: u8, out: &mut [u8], ev: &mut Ev) -> u32 {
    let spi = pac::SPI1;
    let len = out.len() + 1;
    cs_low();
    let t0 = cyc();
    spi.dr().write(|w| w.set_data((addr | 0x80) as u16));
    let mut sent = 1;
    let mut got = 0;
    while got < len {
        let sr = spi.sr().read();
        if sent < len && sr.tnf() {
            spi.dr().write(|w| w.set_data(0));
            sent += 1;
        }
        if sr.rne() {
            let v = spi.dr().read().data() as u8;
            if got > 0 {
                out[got - 1] = v;
            }
            got += 1;
        }
        ev.poll();
    }
    while spi.sr().read().bsy() {}
    cs_high();
    t0
}

/// Cycles from the cycle count taken before the first register write to the first falling SCLK
/// edge: (median, min, max) of 255 two-byte reads of WHO_AM_I; `None` if SCLK never moved.
fn calibrate() -> Option<(u32, u32, u32)> {
    let spi = pac::SPI1;
    let mut v = [0u32; 255];
    for slot in v.iter_mut() {
        let r = cortex_m::interrupt::free(|_| {
            cs_low();
            let t0 = cyc();
            spi.dr()
                .write(|w| w.set_data((reg::WHO_AM_I | 0x80) as u16));
            spi.dr().write(|w| w.set_data(0));
            let mut lat = None;
            loop {
                let g = gin();
                let t = cyc().wrapping_sub(t0);
                if g & SCLK == 0 {
                    lat = Some(t);
                    break;
                }
                if t > 4000 {
                    break;
                }
            }
            let mut got = 0;
            while got < 2 {
                if spi.sr().read().rne() {
                    let _ = spi.dr().read();
                    got += 1;
                }
            }
            while spi.sr().read().bsy() {}
            cs_high();
            lat
        });
        *slot = r?;
    }
    for i in 1..v.len() {
        let mut j = i;
        while j > 0 && v[j - 1] > v[j] {
            v.swap(j - 1, j);
            j -= 1;
        }
    }
    Some((v[127], v[0], v[254]))
}

fn wait_rise(to: u32) -> Option<u32> {
    let s = cyc();
    while gin() & INT1 != 0 {
        if cyc().wrapping_sub(s) > to {
            return None;
        }
    }
    loop {
        let g = gin();
        let t = cyc();
        if g & INT1 != 0 {
            return Some(t);
        }
        if t.wrapping_sub(s) > to {
            return None;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Small statistics helpers
// ---------------------------------------------------------------------------------------------

struct Hist<const N: usize> {
    bins: [u32; N],
    n: u32,
    min: i32,
    max: i32,
}

impl<const N: usize> Hist<N> {
    const fn new() -> Self {
        Hist {
            bins: [0; N],
            n: 0,
            min: i32::MAX,
            max: i32::MIN,
        }
    }

    fn add(&mut self, v: i32, off: i32, w: i32) {
        self.n += 1;
        self.min = self.min.min(v);
        self.max = self.max.max(v);
        let b = (v - off).div_euclid(w).clamp(0, N as i32 - 1) as usize;
        self.bins[b] += 1;
    }

    /// Centre of the bin holding the median, in the unit of `v`.
    fn median(&self, off: i32, w: i32) -> i32 {
        let half = self.n.div_ceil(2);
        let mut c = 0;
        for (i, b) in self.bins.iter().enumerate() {
            c += *b;
            if c >= half {
                return off + i as i32 * w + w / 2;
            }
        }
        off
    }
}

// ---------------------------------------------------------------------------------------------
// Q2 and Q4: both pins' edges for 2 s
// ---------------------------------------------------------------------------------------------

const HALF_CYC: i32 = if SLOW { 132 } else { 66 } * US as i32;

struct Mon {
    prev: u32,
    have1: bool,
    last1: u32,
    pend2: Option<u32>,
    n1: u32,
    n2: u32,
    paired: u32,
    before: u32,
    per: Hist<PER_N>,
    del: Hist<1400>,
}

impl Mon {
    fn run(&mut self, chunk: u32) {
        let s = cyc();
        while cyc().wrapping_sub(s) < chunk {
            let now = gin();
            let t = cyc();
            let r = now & !self.prev;
            self.prev = now;
            if r & INT1 != 0 {
                self.n1 += 1;
                if self.have1 {
                    self.per
                        .add(t.wrapping_sub(self.last1) as i32, 0, (US / 10) as i32);
                }
                self.have1 = true;
                self.last1 = t;
                if let Some(p) = self.pend2.take() {
                    self.del
                        .add(p.wrapping_sub(t) as i32, -70 * US as i32, (US / 10) as i32);
                    self.before += 1;
                }
            }
            if r & INT2 != 0 {
                self.n2 += 1;
                if self.have1 {
                    let d = t.wrapping_sub(self.last1) as i32;
                    if d <= HALF_CYC {
                        self.del.add(d, -70 * US as i32, (US / 10) as i32);
                        self.paired += 1;
                    } else {
                        self.pend2 = Some(t);
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Q1
// ---------------------------------------------------------------------------------------------

struct Cyc {
    pre: [u8; 6],
    post: [u8; 6],
    probe: [u8; 6],
    t0: u32,
    t1: u32,
}

enum Out {
    NoEdgeN,
    PreBad,
    NoEdge,
    BadEdge,
    PostBad,
    Ok(Cyc),
}

fn one_cycle(addr: u8, d: u32) -> Out {
    cortex_m::interrupt::free(|_| {
        let Some(tn) = wait_rise(WAIT_EDGE_TO) else {
            return Out::NoEdgeN;
        };
        while cyc().wrapping_sub(tn) < PRE_DELAY {}
        let (mut a1, mut a2) = ([0u8; 6], [0u8; 6]);
        let mut dummy = Ev::new();
        xfer(addr, &mut a1, &mut dummy);
        xfer(addr, &mut a2, &mut dummy);
        if a1 != a2 {
            return Out::PreBad;
        }
        let mut ev = Ev::new();
        while cyc().wrapping_sub(tn) < d {
            ev.poll();
        }
        let mut probe = [0u8; 6];
        let t0 = xfer(addr, &mut probe, &mut ev);
        while !ev.seen1 {
            ev.poll();
            if cyc().wrapping_sub(tn) > EDGE_TO {
                return Out::NoEdge;
            }
        }
        let t1 = ev.t1;
        let per = t1.wrapping_sub(tn);
        if !(MIN_P..=MAX_P).contains(&per) {
            return Out::BadEdge;
        }
        while cyc().wrapping_sub(t1) < POST_DELAY {}
        let (mut b1, mut b2) = ([0u8; 6], [0u8; 6]);
        xfer(addr, &mut b1, &mut dummy);
        xfer(addr, &mut b2, &mut dummy);
        if b1 != b2 {
            return Out::PostBad;
        }
        Out::Ok(Cyc {
            pre: a1,
            post: b1,
            probe,
            t0,
            t1,
        })
    })
}

#[derive(Clone, Copy, Default)]
struct Torn {
    rel_us: i32,
    a: [u8; 6],
    b: [u8; 6],
    p: [u8; 6],
}

struct Q1 {
    old: [[[u32; NB]; 3]; 3],
    new: [[[u32; NB]; 3]; 3],
    cyc_n: [u32; 3],
    classified: [u32; 3],
    same: [u32; 3],
    no_edge_n: [u32; 3],
    pre_bad: [u32; 3],
    no_edge: [u32; 3],
    bad_edge: [u32; 3],
    post_bad: [u32; 3],
    rd_old: [u32; 3],
    rd_new: [u32; 3],
    rd_torn: [u32; 3],
    split: [u32; 3],
    torn_byte: [u32; 3],
    /// [channel][axis state code]: state of X + 4 * Y + 16 * Z, each 0 old, 1 new, 2 equal in N and N+1, 3 neither
    pat: [[u32; 64]; 3],
    /// [channel][axis][old, new, same, neither]
    axis: [[[u32; 4]; 3]; 3],
    torn: [Torn; 8],
    torn_stored: usize,
}

impl Q1 {
    fn classify(&mut self, ch: usize, c: &Cyc, lat: u32, div: u32) {
        let (a, b, p) = (&c.pre, &c.post, &c.probe);
        if a == b {
            self.same[ch] += 1;
            return;
        }
        self.classified[ch] += 1;
        let byte_cyc = 8 * div;
        let mut axis_state = [0u8; 3];
        for ax in 0..3 {
            let (i, j) = (2 * ax, 2 * ax + 1);
            let pa = [p[i], p[j]] == [a[i], a[j]];
            let pb = [p[i], p[j]] == [b[i], b[j]];
            let ab = [a[i], a[j]] == [b[i], b[j]];
            let s = if ab {
                2
            } else if pa {
                0
            } else if pb {
                1
            } else {
                3
            };
            axis_state[ax] = s;
            self.axis[ch][ax][s as usize] += 1;
        }
        self.pat[ch][(axis_state[0] + 4 * axis_state[1] + 16 * axis_state[2]) as usize] += 1;
        let mut lo_hi_old = false;
        let mut lo_hi_new = false;
        for k in 0..6 {
            if a[k] == b[k] {
                continue;
            }
            let t = c.t0.wrapping_add(lat + byte_cyc * (k as u32 + 1) - div / 2);
            let rel = t.wrapping_sub(c.t1) as i32;
            let bin = (rel - REL_MIN_US * US as i32).div_euclid(BIN_CYC);
            let bin = bin.clamp(0, NB as i32 - 1) as usize;
            if p[k] == a[k] {
                self.old[ch][k / 2][bin] += 1;
                lo_hi_old = true;
            } else if p[k] == b[k] {
                self.new[ch][k / 2][bin] += 1;
                lo_hi_new = true;
            } else {
                self.torn_byte[ch] += 1;
            }
        }
        if lo_hi_old && lo_hi_new {
            // At least one byte old and at least one new: the pair of bytes of one axis may be split.
            for ax in 0..3 {
                let (i, j) = (2 * ax, 2 * ax + 1);
                if a[i] != b[i] && a[j] != b[j] {
                    let (si, sj) = (p[i] == b[i], p[j] == b[j]);
                    let (oi, oj) = (p[i] == a[i], p[j] == a[j]);
                    if (si && oj) || (oi && sj) {
                        self.split[ch] += 1;
                    }
                }
            }
        }
        if p == a {
            self.rd_old[ch] += 1;
        } else if p == b {
            self.rd_new[ch] += 1;
        } else {
            self.rd_torn[ch] += 1;
            if self.torn_stored < self.torn.len() {
                self.torn[self.torn_stored] = Torn {
                    rel_us: (c.t0.wrapping_sub(c.t1) as i32) / US as i32,
                    a: *a,
                    b: *b,
                    p: *p,
                };
                self.torn_stored += 1;
            }
        }
    }
}

fn bin_us(i: usize) -> f32 {
    REL_MIN_US as f32 + (i as f32 + 0.5) * 0.5
}

/// First bin (ascending time) with at least 3 observations of which `num/den` or more are new.
fn crossing(old: &[u32; NB], new: &[u32; NB], num: u32, den: u32) -> Option<usize> {
    (0..NB).find(|&i| {
        let t = old[i] + new[i];
        t >= 3 && new[i] * den >= num * t
    })
}

const AX_LABEL: [&str; 3] = ["axis X", "axis Y", "axis Z"];
const TAB_LO: usize = ((-30 - REL_MIN_US) * 2) as usize;
const TAB_HI: usize = TAB_LO + 68;

fn lead_line(name: &str, what: &str, old: &[u32; NB], new: &[u32; NB]) {
    let first_new = (0..NB).find(|&i| new[i] > 0);
    let last_old = (0..NB).rev().find(|&i| old[i] > 0);
    let p5 = crossing(old, new, 5, 100);
    let p50 = crossing(old, new, 50, 100);
    let p95 = crossing(old, new, 95, 100);
    let f = |o: Option<usize>| match o {
        Some(i) => -bin_us(i),
        None => f32::NAN,
    };
    // Registers change at rel time T (negative = before the edge); "new" seen at t means T < t.
    // Lead = -T (positive = registers change BEFORE the edge).
    defmt::info!(
        "[STAT] Q1 {} {}: registers change before the INT1 edge by (positive = before): min {} us, median {} us, max {} us; 5%..95% points {} .. {} us; bin 0.5 us",
        name,
        what,
        f(last_old),
        f(p50),
        f(first_new),
        f(p95),
        f(p5)
    );
}

fn report_q1(ch: usize, q: &Q1) {
    let name = NAMES[ch];
    defmt::info!(
        "[STAT] Q1 {}: cycles {}, classified {} (N equal N+1, dropped: {}), dropped: no edge N {} / pre pair differs {} / no edge N+1 {} / implausible period {} / post pair differs {}",
        name,
        q.cyc_n[ch],
        q.classified[ch],
        q.same[ch],
        q.no_edge_n[ch],
        q.pre_bad[ch],
        q.no_edge[ch],
        q.bad_edge[ch],
        q.post_bad[ch]
    );
    let mut old = [0u32; NB];
    let mut new = [0u32; NB];
    for ax in 0..3 {
        for i in 0..NB {
            old[i] += q.old[ch][ax][i];
            new[i] += q.new[ch][ax][i];
        }
    }
    lead_line(name, "all axes", &old, &new);
    for ax in 0..3 {
        lead_line(name, AX_LABEL[ax], &q.old[ch][ax], &q.new[ch][ax]);
    }
    defmt::info!(
        "[STAT] Q1 {}: probe reads old {} new {} torn {} (equal neither N nor N+1); byte observations torn {}, axes split between low and high byte {}",
        name,
        q.rd_old[ch],
        q.rd_new[ch],
        q.rd_torn[ch],
        q.torn_byte[ch],
        q.split[ch]
    );
    for ax in 0..3 {
        let a = q.axis[ch][ax];
        defmt::info!(
            "[STAT] Q1 {} axis {}: matched old {} new {} (N equals N+1 on this axis {}) neither {}",
            name,
            ax,
            a[0],
            a[1],
            a[2],
            a[3]
        );
    }
    for (code, n) in q.pat[ch].iter().enumerate() {
        if *n > 0 {
            let l = |s: usize| ['o', 'n', 's', 'x'][s & 3];
            defmt::info!(
                "[STAT] Q1 {} probe read axis pattern X{}Y{}Z{} (o old, n new, s N equals N+1 on that axis, x neither): {} reads",
                name,
                l(code),
                l(code >> 2),
                l(code >> 4),
                *n
            );
        }
    }
    for ax in 0..4 {
        let (o, n) = if ax == 3 {
            (&old, &new)
        } else {
            (&q.old[ch][ax], &q.new[ch][ax])
        };
        let mut i0 = TAB_LO;
        while i0 < TAB_HI {
            defmt::info!(
                "[STAT] Q1 {} {} bins of 0.5 us from {} us to the edge: old {=[?]} new {=[?]}",
                name,
                if ax == 3 { "all axes" } else { AX_LABEL[ax] },
                bin_us(i0) - 0.25,
                &o[i0..i0 + 17],
                &n[i0..i0 + 17]
            );
            i0 += 17;
        }
    }
    let (mut o_all, mut n_all) = (0u32, 0u32);
    for i in 0..NB {
        o_all += old[i];
        n_all += new[i];
    }
    defmt::info!(
        "[STAT] Q1 {}: byte observations old {} new {} in total; scan covered {} us to {} us from the edge",
        name,
        o_all,
        n_all,
        SCAN_LO_US,
        SCAN_LO_US + SCAN_STEPS as i32
    );
}

// ---------------------------------------------------------------------------------------------
// Q3
// ---------------------------------------------------------------------------------------------

struct AxisStat {
    n: u32,
    same: u32,
    ge4: u32,
    max: u32,
    h: [u32; 64],
}

fn rest_sample(buf: &mut [u8; 18]) -> Option<(u32, bool)> {
    cortex_m::interrupt::free(|_| {
        let tn = wait_rise(WAIT_EDGE_TO)?;
        while cyc().wrapping_sub(tn) < REST_DELAY {}
        let mut ev = Ev::new();
        let mut b2 = [0u8; 18];
        xfer(reg::OUTX_L_G, &mut buf[..12], &mut ev);
        if HIGH_G {
            xfer(reg::UI_OUTX_L_A_OIS_HG, &mut buf[12..], &mut ev);
        }
        xfer(reg::OUTX_L_G, &mut b2[..12], &mut ev);
        if HIGH_G {
            xfer(reg::UI_OUTX_L_A_OIS_HG, &mut b2[12..], &mut ev);
        }
        Some((tn, *buf == b2))
    })
}

fn axes(b: &[u8; 18]) -> [i32; 9] {
    let mut o = [0i32; 9];
    for i in 0..NAX {
        o[i] = i16::from_le_bytes([b[2 * i], b[2 * i + 1]]) as i32;
    }
    o
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    dwt_on();
    let mut fails = 0u32;
    defmt::info!(
        "[INFO] clk_sys {} Hz, clk_peri {} Hz",
        embassy_rp::clocks::clk_sys_freq(),
        embassy_rp::clocks::clk_peri_freq()
    );
    let dev = mkdev(true, REQ_SOAK_HZ);
    let (mut int1, pwm, _int2) = mkint();
    let mut imu = Lsm6dsk320x::new(dev);
    match imu.init(&mut Delay, &cfg_lead()).await {
        Ok(_) if !HIGH_G => defmt::info!(
            "[PASS] init (LEAD_HIGH_G=0, the soak cfg_a): gyro + low-g 7.68 kHz, high-g off, INT1 low-g, INT2 unrouted"
        ),
        Ok(_) => defmt::info!(
            "[PASS] init: gyro + low-g 7.68 kHz, high-g 7.68 kHz, INT1 low-g, INT2 high-g"
        ),
        Err(e) => {
            defmt::info!("[FAIL] init: {}", defmt::Debug2Format(&e));
            hil_common::exit(1)
        }
    }
    defmt::info!(
        "[INFO] LEAD_HZ3840 {}: every channel that is on runs at {} Hz",
        SLOW,
        if SLOW { 3840u32 } else { 7680u32 }
    );
    let div = spi_div();
    defmt::info!(
        "[INFO] SPI1 {} Hz (SSPCLK cycles per bit {}), mode 3",
        spi_hz(),
        div
    );
    // Let the output rate settle, then check that both channels deliver edges at all.
    let _ = with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge()).await;

    // ---- SCLK latency
    let Some((lat, lat_min, lat_max)) = calibrate() else {
        defmt::info!("[FAIL] SCLK was never seen low through GPIO_IN: byte timing not calibrated");
        hil_common::exit(1)
    };
    defmt::info!(
        "[INFO] first SCLK fall after the first data-register write: median {} cycles ({} us), min {}, max {}",
        lat,
        us_f(lat as i32),
        lat_min,
        lat_max
    );

    // ---- Q2, Q4
    let mut mon = Mon {
        prev: gin(),
        have1: false,
        last1: 0,
        pend2: None,
        n1: 0,
        n2: 0,
        paired: 0,
        before: 0,
        per: Hist::new(),
        del: Hist::new(),
    };
    let c0 = pwm.counter();
    let start = cyc();
    let mut gaps = 0u32;
    let mut last_end = cyc();
    while cyc().wrapping_sub(start) < 2 * CPU_MHZ * 1_000_000 {
        cortex_m::interrupt::free(|_| {
            let g = cyc().wrapping_sub(last_end);
            if g > 2 * MIN_P {
                gaps += 1;
            }
            mon.run(300_000);
            last_end = cyc();
        });
    }
    let c1 = pwm.counter();
    let pwm_edges = c1.wrapping_sub(c0) as u32;
    let p_med = mon.per.median(0, (US / 10) as i32);
    defmt::info!(
        "[STAT] Q4 INT1 period: min {} us, median {} us, max {} us over {} periods (bin 0.1 us; polled edge timestamps, 6.7 ns tick)",
        us_f(mon.per.min),
        us_f(p_med),
        us_f(mon.per.max),
        mon.per.n
    );
    if HIGH_G {
        defmt::info!(
            "[STAT] Q2 edges over 2 s: INT1 {} (hardware counter PWM slice 3: {}), INT2 {} (polled; GPIO6 is the A pin of the slice and cannot be counted in hardware); polling gaps over 2 periods between chunks {}",
            mon.n1,
            pwm_edges,
            mon.n2,
            gaps
        );
        defmt::info!(
            "[STAT] Q2 INT1 rising edge -> INT2 rising edge (negative = INT2 first): min {} us, median {} us, max {} us over {} pairs ({} with INT2 before INT1; bin 0.1 us)",
            us_f(mon.del.min),
            us_f(mon.del.median(-70 * US as i32, (US / 10) as i32)),
            us_f(mon.del.max),
            mon.del.n,
            mon.before
        );
        check!(
            fails,
            mon.n1 > MIN_EDGES_2S && mon.n2 > MIN_EDGES_2S && p_med > 0,
            "{} both interrupt lines run: INT1 {} edges, INT2 {} edges in 2 s",
            mon.n1,
            mon.n2
        );
    } else {
        defmt::info!(
            "[STAT] high-g off (LEAD_HIGH_G=0): INT1 edges over 2 s {} (hardware counter PWM slice 3: {}); polling gaps {}; Q2 skipped",
            mon.n1,
            pwm_edges,
            gaps
        );
        check!(
            fails,
            mon.n1 > MIN_EDGES_2S && p_med > 0,
            "{} INT1 runs: {} edges in 2 s",
            mon.n1
        );
    }
    let period = p_med.max(100 * US as i32) as u32;

    // ---- Q3
    let mut st: [AxisStat; 9] = core::array::from_fn(|_| AxisStat {
        n: 0,
        same: 0,
        ge4: 0,
        max: 0,
        h: [0; 64],
    });
    let (mut prev, mut prev_t): (Option<[i32; 9]>, u32) = (None, 0);
    let (mut q3_ok, mut q3_bad, mut q3_miss, mut q3_gap) = (0u32, 0u32, 0u32, 0u32);
    while q3_ok < Q3_SAMPLES {
        let mut buf = [0u8; 18];
        let Some((tn, good)) = rest_sample(&mut buf) else {
            q3_miss += 1;
            prev = None;
            continue;
        };
        if !good {
            q3_bad += 1;
            prev = None;
            continue;
        }
        let cur = axes(&buf);
        let gap = tn.wrapping_sub(prev_t);
        prev_t = tn;
        if let Some(pv) = prev {
            if gap > period + period / 2 {
                q3_gap += 1;
            } else {
                q3_ok += 1;
                for i in 0..NAX {
                    let s = (cur[i] - pv[i]).unsigned_abs();
                    let a = &mut st[i];
                    a.n += 1;
                    if s == 0 {
                        a.same += 1;
                    }
                    if s >= 4 {
                        a.ge4 += 1;
                    }
                    a.max = a.max.max(s);
                    a.h[(s as usize).min(63)] += 1;
                }
            }
        }
        prev = Some(cur);
    }
    for i in 0..NAX {
        let a = &st[i];
        let (mut c, mut med) = (0u32, 0u32);
        for (s, n) in a.h.iter().enumerate() {
            c += *n;
            if c >= a.n.div_ceil(2) {
                med = s as u32;
                break;
            }
        }
        defmt::info!(
            "[STAT] Q3 {} at rest: consecutive samples {}, identical {} per mille, median |step| {} LSB, steps of 4 LSB or more {} per mille, largest step {} LSB",
            AXN[i],
            a.n,
            a.same * 1000 / a.n.max(1),
            med,
            a.ge4 * 1000 / a.n.max(1),
            a.max
        );
    }
    defmt::info!(
        "[STAT] Q3 reads: usable {}, repeated read differed {}, no edge {}, discontinuity (edge missed) {}",
        q3_ok,
        q3_bad,
        q3_miss,
        q3_gap
    );

    // ---- Q1
    let mut q = alloc_q1();
    let t_start = Instant::now();
    let mut k = 0u32;
    loop {
        let ch = (k % NCH as u32) as usize;
        let step = (k / NCH as u32) % SCAN_STEPS;
        k += 1;
        let rel = SCAN_LO_US + step as i32;
        let byte_cyc = 8 * div;
        // The centre of the six data bytes is byte index 2.5.
        let d = (period as i32 + rel * US as i32 - lat as i32 - (byte_cyc * 7 / 2) as i32
            + (div / 2) as i32)
            .max((20 * US) as i32) as u32;
        q.cyc_n[ch] += 1;
        match one_cycle(BASES[ch], d) {
            Out::NoEdgeN => q.no_edge_n[ch] += 1,
            Out::PreBad => q.pre_bad[ch] += 1,
            Out::NoEdge => q.no_edge[ch] += 1,
            Out::BadEdge => q.bad_edge[ch] += 1,
            Out::PostBad => q.post_bad[ch] += 1,
            Out::Ok(c) => q.classify(ch, &c, lat, div),
        }
        if k % 300 == 0 {
            // Interrupts are on again here for a moment; the executor has nothing to do.
            let done = q.classified[..NCH].iter().all(|&n| n >= Q1_TARGET);
            if done || t_start.elapsed().as_secs() >= Q1_MAX_S {
                break;
            }
        }
    }
    defmt::info!(
        "[INFO] Q1 scan finished: {} cycles in {} ms",
        k,
        t_start.elapsed().as_millis() as u32
    );
    for ch in 0..NCH {
        report_q1(ch, &q);
        check!(
            fails,
            q.classified[ch] >= MIN_CLASSIFIED,
            "{} {}: classified edges {} (at least {})",
            NAMES[ch],
            q.classified[ch],
            MIN_CLASSIFIED
        );
    }
    for t in &q.torn[..q.torn_stored] {
        defmt::info!(
            "[STAT] torn probe read, started {} us from edge N+1: N {:x} N+1 {:x} probe {:x}",
            t.rel_us,
            t.a,
            t.b,
            t.p
        );
    }

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
    hil_common::exit(fails)
}

fn alloc_q1() -> Q1 {
    Q1 {
        old: [[[0; NB]; 3]; 3],
        new: [[[0; NB]; 3]; 3],
        cyc_n: [0; 3],
        classified: [0; 3],
        same: [0; 3],
        no_edge_n: [0; 3],
        pre_bad: [0; 3],
        no_edge: [0; 3],
        bad_edge: [0; 3],
        post_bad: [0; 3],
        rd_old: [0; 3],
        rd_new: [0; 3],
        rd_torn: [0; 3],
        split: [0; 3],
        torn_byte: [0; 3],
        axis: [[[0; 4]; 3]; 3],
        pat: [[0; 64]; 3],
        torn: [Torn::default(); 8],
        torn_stored: 0,
    }
}

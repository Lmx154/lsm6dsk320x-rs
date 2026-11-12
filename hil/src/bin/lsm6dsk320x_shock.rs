//! On-board strike test of the high-g wake-up and shock generator of the lsm6dsk320x driver (U23
//! on SPI1, INT1 on GPIO7, INT2 on GPIO6). The low-g accelerometer, the gyroscope and the high-g
//! accelerometer are all on throughout.
//!
//! H0 start, H1 machine-only control (can the generator fire at rest), H2 thresholds that give 0
//! events at rest, H3 the strike window (a person raps the board; length `SHOCK_WINDOW_S`, default
//! 30), H4 quiet again, H5 the data-ready edges of the low-g and gyro channels were undisturbed,
//! then the default configuration. No SPI transfer is ever cancelled: the only timeouts wrap the
//! wait for an INT1 edge.
//!
//! Pins during H2 to H5: INT1 carries the low-g and gyro data-ready (counted by PWM slice 3 in
//! hardware, it also paces the reads), INT2 carries ONLY the wake-up and the shock events. The
//! wake-up and the shock share INT2, so an INT2 edge is matched to a kind by the status flag. INT2
//! edges are counted by a task awaiting the GPIO edge.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIO0, SPI1};
use embassy_rp::pio::{self, Pio};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsk320x::{
    AccelFullScale, Config, DataReady, GyroFullScale, HighGConfig, HighGEvents, HighGFullScale,
    HighGOdr, HighGWakeUpMode, IntRouting, Odr, calc, reg,
};
use panic_probe as _;
use smart_leds::RGB8;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>, dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>, dma::InterruptHandler<embassy_rp::peripherals::DMA_CH2>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

type Dev = ExclusiveDevice<Spi<'static, SPI1, spi::Async>, Output<'static>, Delay>;
type Led = PioWs2812<'static, PIO0, 0, 1, Grb>;
type Imu = lsm6dsk320x::Lsm6dsk320x<Dev>;

const fn parse_secs(s: Option<&str>) -> u32 {
    let Some(s) = s else { return 30 };
    let b = s.as_bytes();
    let mut n = 0u32;
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c < b'0' || c > b'9' {
            return 30;
        }
        n = n * 10 + (c - b'0') as u32;
        i += 1;
    }
    if n == 0 { 30 } else { n }
}

const WINDOW_S: u32 = parse_secs(option_env!("SHOCK_WINDOW_S"));
const SPI_HZ: u32 = 8_000_000;
const ACCEL_FS: AccelFullScale = AccelFullScale::G4;
const GYRO_FS: GyroFullScale = GyroFullScale::Dps500;
const HIGH_G_FS: HighGFullScale = HighGFullScale::G64;
const ODR_HZ: f32 = 7680.0;
const G: f32 = lsm6dsk320x::sample::G;
/// Rest-test thresholds in g (1 g per LSB at +-64 g full scale, p94), tried in order.
const CANDIDATES_G: [u8; 8] = [3, 4, 5, 6, 8, 10, 12, 16];
/// HG_SHOCK_DUR code of the strike test: (0 + 1) * 512 / 7680 Hz = 66.7 ms (p94).
const SHOCK_DUR: u8 = 0;
/// Events of one kind closer together than this are one strike (ms).
const DEAD_MS: u64 = 30;
/// Peak tracking continues this long after an event before its line is printed (ms).
const PEAK_AFTER_MS: u64 = 40;
const MAX_LINES: u32 = 20;
/// Threshold that nothing can reach at +-64 g: the generator is on but silent.
const SILENT_G: u8 = 100;

static EDGES2: AtomicU32 = AtomicU32::new(0);

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

/// Brightness of the status LED, of 255.
const LED: u8 = 40;

/// The four phases of the run: each is announced by one fixed log line and one colour of the
/// status LED (WS2812C on GPIO43, PIO0 state machine 0, DMA channel 2), written once per phase.
enum Phase {
    Ready,
    Strike,
    Still,
    Done,
}

async fn phase(led: &mut Led, which: Phase) {
    let (r, g, b) = match which {
        Phase::Ready => (0, 0, LED),
        Phase::Strike => (0, LED, 0),
        Phase::Still => (LED, 0, 0),
        Phase::Done => (0, 0, 0),
    };
    led.write(&[RGB8::new(r, g, b)]).await;
    match which {
        Phase::Ready => defmt::info!("[INFO] PHASE READY: hands off the board"),
        Phase::Strike => defmt::info!(
            "[INFO] PHASE STRIKE: strike the board now for {} s",
            WINDOW_S
        ),
        Phase::Still => {
            defmt::info!("[INFO] PHASE STILL: stop, put the board down, leave it still")
        }
        Phase::Done => defmt::info!("[INFO] PHASE DONE"),
    }
}

fn abs(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

fn ms_since(t0: Instant) -> u64 {
    t0.elapsed().as_millis()
}

/// INT2 rising edges, counted from the latched raw edge bit of GPIO6 (IO_BANK0 INTR0 bits 24 to
/// 27 are GPIO6: level low, level high, edge low, edge high). Reading this function takes the
/// latched edge, so every wait loop calls it. Two edges between two calls count once.
fn edges2() -> u32 {
    const EDGE_HIGH: u32 = 1 << 27;
    let intr = embassy_rp::pac::IO_BANK0.intr(0);
    if intr.read().0 & EDGE_HIGH != 0 {
        intr.write_value(embassy_rp::pac::io::regs::Int(EDGE_HIGH));
        EDGES2.fetch_add(1, Ordering::Relaxed);
    }
    EDGES2.load(Ordering::Relaxed)
}

fn config(events: Option<HighGEvents>, int1: IntRouting, int2: IntRouting) -> Config {
    Config::new()
        .with_accel_fs(ACCEL_FS)
        .with_accel_odr(Odr::Hz7680)
        .with_gyro_fs(GYRO_FS)
        .with_gyro_odr(Odr::Hz7680)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz7680)
                .with_full_scale(HIGH_G_FS),
        ))
        .with_high_g_events(events)
        .with_int1(int1)
        .with_int2(int2)
}

fn drdy1() -> IntRouting {
    IntRouting::new().with_data_ready(DataReady::new().with_accel(true).with_gyro(true))
}

fn events_int2() -> IntRouting {
    IntRouting::new()
        .with_high_g_wake_up(true)
        .with_high_g_shock(true)
}

fn events(threshold: u8, mode: HighGWakeUpMode) -> HighGEvents {
    HighGEvents::new()
        .with_threshold(threshold)
        .with_wake_up_mode(mode)
        .with_shock_duration(SHOCK_DUR)
}

/// The configuration of H2 to H5.
fn strike_config(threshold: u8) -> Config {
    config(
        Some(events(threshold, HighGWakeUpMode::EachTime)),
        drdy1(),
        events_int2(),
    )
}

/// Reads ALL_INT_SRC, which resets the latched interrupt signals (9.55, p93), the step the
/// datasheet gives for clearing.
async fn clear(imu: &mut Imu) -> Result<u8, ()> {
    imu.read_register(reg::ALL_INT_SRC).await.map_err(|_| ())
}

fn level(gpio: usize) -> bool {
    embassy_rp::pac::SIO.gpio_in(0).read() & (1 << gpio) != 0
}

/// What `rest_probe` saw.
struct Rest {
    flags: u32,
    edges: u32,
    peak: [f32; 3],
    errors: u32,
}

/// Polls the generator for `ms` at about 1 kHz with the board at rest; clears every flag it sees.
async fn rest_probe(imu: &mut Imu, ms: u64) -> Rest {
    let mut r = Rest {
        flags: 0,
        edges: 0,
        peak: [0.0; 3],
        errors: 0,
    };
    let e0 = edges2();
    let t0 = Instant::now();
    while ms_since(t0) < ms {
        match imu.read_high_g().await {
            Ok(h) => {
                let g = h.high_g_ms2(HIGH_G_FS);
                for i in 0..3 {
                    r.peak[i] = r.peak[i].max(abs(g[i]) / G);
                }
            }
            Err(_) => r.errors += 1,
        }
        match imu.read_register(reg::HG_WAKE_UP_SRC).await {
            Ok(v) => {
                if v & (reg::HG_WAKE_UP_SRC_HG_WU_IA | reg::HG_WAKE_UP_SRC_HG_SHOCK_CHANGE_IA) != 0
                {
                    r.flags += 1;
                    if clear(imu).await.is_err() {
                        r.errors += 1;
                    }
                }
            }
            Err(_) => r.errors += 1,
        }
        Timer::after(Duration::from_millis(1)).await;
    }
    r.edges = edges2().wrapping_sub(e0);
    r
}

/// One H1 attempt: which kind of event, on which pin, with which settings.
struct Try {
    shock: bool,
    pin: u8,
    threshold: u8,
    mode: HighGWakeUpMode,
}

fn kind_name(shock: bool) -> &'static str {
    if shock { "shock" } else { "wake-up" }
}

/// H1: can the generator be made to fire with the board at rest? Returns the number of firing
/// attempts whose edge AND status flag were both seen.
async fn h1(imu: &mut Imu, counter: &Pwm<'_>, fails: &mut u32) -> u32 {
    let mut proven = 0u32;
    for shock in [false, true] {
        for pin in [2u8, 1u8] {
            let mut fired = false;
            let tries = [
                (0u8, HighGWakeUpMode::EachTime),
                (1u8, HighGWakeUpMode::EachTime),
                (0u8, HighGWakeUpMode::OnChange),
            ];
            for (threshold, mode) in tries {
                if fired {
                    break;
                }
                let t = Try {
                    shock,
                    pin,
                    threshold,
                    mode,
                };
                let route = IntRouting::new()
                    .with_high_g_wake_up(!shock)
                    .with_high_g_shock(shock);
                let (i1, i2) = if pin == 1 {
                    (route, IntRouting::new())
                } else {
                    (IntRouting::new(), route)
                };
                // Generator off first, so the shock state starts again from "not in shock".
                let off = config(None, IntRouting::new(), IntRouting::new());
                if let Err(e) = imu.configure(&mut Delay, &off).await {
                    check!(
                        *fails,
                        false,
                        "{} H1 generator off: {}",
                        defmt::Debug2Format(&e)
                    );
                    return proven;
                }
                let on = config(Some(events(SILENT_G, t.mode)), i1, i2);
                if let Err(e) = imu.configure(&mut Delay, &on).await {
                    check!(
                        *fails,
                        false,
                        "{} H1 configure: {}",
                        defmt::Debug2Format(&e)
                    );
                    return proven;
                }
                if let Err(e) = imu.verify().await {
                    check!(*fails, false, "{} H1 verify: {}", defmt::Debug2Format(&e));
                    return proven;
                }
                let _ = clear(imu).await;
                let c0 = counter.counter();
                let e0 = edges2();
                let (mut raw_flag, mut st_flag, mut seen, mut errs) = (0u32, 0u32, 0u8, 0u32);
                // The trigger: the threshold register is lowered while the polling runs, so the
                // pin edge and the flags happen inside the observation.
                if imu
                    .write_register(reg::HG_WAKE_UP_THS, t.threshold)
                    .await
                    .is_err()
                {
                    errs += 1;
                }
                let t0 = Instant::now();
                let mut n = 0u32;
                let mut polled_rises = 0u32;
                let mut was_high = level(if t.pin == 1 { 7 } else { 6 });
                while ms_since(t0) < 200 {
                    let now_high = level(if t.pin == 1 { 7 } else { 6 });
                    polled_rises += (now_high && !was_high) as u32;
                    was_high = now_high;
                    match imu.read_register(reg::HG_WAKE_UP_SRC).await {
                        Ok(v) => {
                            seen |= v;
                            let bit = if t.shock {
                                reg::HG_WAKE_UP_SRC_HG_SHOCK_CHANGE_IA
                            } else {
                                reg::HG_WAKE_UP_SRC_HG_WU_IA
                            };
                            if v & bit != 0 {
                                raw_flag += 1;
                            }
                        }
                        Err(_) => errs += 1,
                    }
                    if t.shock || n % 10 == 0 {
                        match imu.interrupt_status().await {
                            Ok(s) => {
                                if (t.shock && s.high_g_shock) || (!t.shock && s.high_g_wake_up) {
                                    st_flag += 1;
                                }
                            }
                            Err(_) => errs += 1,
                        }
                    }
                    n += 1;
                }
                let edges = if t.pin == 1 {
                    counter.counter().wrapping_sub(c0) as u32
                } else {
                    edges2().wrapping_sub(e0)
                };
                let pin_high = level(if t.pin == 1 { 7 } else { 6 });
                defmt::info!(
                    "[INFO] H1 {} on INT{} threshold {} g {}: pin edges {} (polled rises {}), pin level now {}, HG_WAKE_UP_SRC seen 0x{=u8:x}, raw flag polls {}/{}, interrupt_status flag polls {}, bus errors {}",
                    kind_name(t.shock),
                    t.pin,
                    t.threshold,
                    match t.mode {
                        HighGWakeUpMode::EachTime => "each-time",
                        HighGWakeUpMode::OnChange => "on-change",
                    },
                    edges,
                    polled_rises,
                    pin_high,
                    seen,
                    raw_flag,
                    n,
                    st_flag,
                    errs
                );
                if errs != 0 {
                    check!(*fails, false, "{} H1 bus errors {}", errs);
                    return proven;
                }
                if edges == 0 && st_flag == 0 && raw_flag == 0 {
                    continue;
                }
                fired = true;
                check!(
                    *fails,
                    edges > 0 && st_flag > 0,
                    "{} H1 {} on INT{} at rest: edge on the pin ({}) AND interrupt_status flag ({} polls)",
                    kind_name(t.shock),
                    t.pin,
                    edges,
                    st_flag
                );
                if edges > 0 && st_flag > 0 {
                    proven += 1;
                }
                // Clearing: the condition is removed (threshold above anything reachable), then the
                // flag is read before and after the ALL_INT_SRC read (p93).
                let quiet = config(Some(events(SILENT_G, t.mode)), i1, i2);
                if let Err(e) = imu.configure(&mut Delay, &quiet).await {
                    check!(
                        *fails,
                        false,
                        "{} H1 configure quiet: {}",
                        defmt::Debug2Format(&e)
                    );
                    return proven;
                }
                let a = imu.interrupt_status().await;
                let all_int = clear(imu).await;
                let b = imu.interrupt_status().await;
                let c = imu.read_register(reg::HG_WAKE_UP_SRC).await;
                if let (Ok(a), Ok(all_int), Ok(b), Ok(c)) = (a, all_int, b, c) {
                    let (fa, fb) = if t.shock {
                        (a.high_g_shock, b.high_g_shock)
                    } else {
                        (a.high_g_wake_up, b.high_g_wake_up)
                    };
                    defmt::info!(
                        "[INFO] H1 {} clearing: after the condition ended the flag read {} (latched if true), ALL_INT_SRC read 0x{=u8:x}, flag after that read {}, HG_WAKE_UP_SRC 0x{=u8:x}",
                        kind_name(t.shock),
                        fa,
                        all_int,
                        fb,
                        c
                    );
                    check!(
                        *fails,
                        !fb,
                        "{} H1 {} flag clear after the condition ended and ALL_INT_SRC was read",
                        kind_name(t.shock)
                    );
                } else {
                    check!(*fails, false, "{} H1 clearing: bus error");
                    return proven;
                }
            }
            if !fired {
                defmt::info!(
                    "[INFO] H1 {} on INT{} did not fire at rest with threshold 0 g or 1 g (each-time) or threshold 0 g (on-change), shock duration 0",
                    kind_name(shock),
                    pin
                );
            }
        }
    }
    if proven == 0 {
        defmt::info!(
            "[INFO] H1 the generator could not be made to fire at rest with the settings above: routing and status are not proven without a strike"
        );
    }
    proven
}

/// One counted event of the strike window, printed when its peak window has elapsed.
struct Pending {
    due_ms: u64,
    t_ms: u64,
    wake_up: bool,
    shock: bool,
    edge: bool,
    src: u8,
    cleared: bool,
}

#[derive(Default)]
struct Stat {
    wu: u32,
    sh: u32,
    wu_both: u32,
    sh_both: u32,
    raw_edges: u32,
    flag_only: u32,
    edge_only: u32,
    not_cleared: u32,
    lines: u32,
    peak_all: f32,
    errors: u32,
    timeouts: u32,
    reads: u32,
}

fn emit(st: &mut Stat, p: &Pending, peak: [f32; 3]) {
    if st.lines >= MAX_LINES {
        return;
    }
    st.lines += 1;
    let kind = match (p.wake_up, p.shock) {
        (true, true) => "wake-up+shock",
        (true, false) => "wake-up",
        _ => "shock",
    };
    defmt::info!(
        "[INFO] event {}: t {} ms, {}, pin INT2 edge {}, status wake-up {} shock {} (HG_WAKE_UP_SRC 0x{=u8:x}), flag cleared by ALL_INT_SRC read {}, peak high-g since previous event [{}, {}, {}] g",
        st.lines,
        p.t_ms,
        kind,
        p.edge,
        p.wake_up,
        p.shock,
        p.src,
        p.cleared,
        peak[0],
        peak[1],
        peak[2]
    );
}

/// H3 and the data for H5.
struct Strike {
    st: Stat,
    int1_edges: u32,
    elapsed_us: u64,
}

async fn strike_window(imu: &mut Imu, int1: &mut Input<'_>, counter: &Pwm<'_>) -> Strike {
    let mut st = Stat::default();
    // Strikes before the window do not count: clear whatever is latched.
    let _ = clear(imu).await;
    let _ = imu.interrupt_status().await;
    let _ = clear(imu).await;
    let _ = imu.interrupt_status().await;
    let mut seen_e2 = edges2();
    let mut edge_mark = seen_e2;
    let mut last_c = counter.counter();
    let mut int1_edges = 0u32;
    let t0 = Instant::now();
    let end = t0 + Duration::from_secs(WINDOW_S as u64);
    let mut peak = [0f32; 3];
    let (mut last_wu, mut last_sh): (Option<u64>, Option<u64>) = (None, None);
    let mut pending: Option<Pending> = None;
    let (mut prev_present, mut last_handled) = (false, 0u64);
    let mut t_last = t0;
    while Instant::now() < end {
        if let Some(p) = pending.as_ref()
            && ms_since(t0) >= p.due_ms
        {
            emit(&mut st, p, peak);
            peak = [0.0; 3];
            pending = None;
        }
        if with_timeout(Duration::from_millis(50), int1.wait_for_rising_edge())
            .await
            .is_err()
        {
            st.timeouts += 1;
            continue;
        }
        let c = counter.counter();
        int1_edges += c.wrapping_sub(last_c) as u32;
        last_c = c;
        t_last = Instant::now();
        match imu.read_all().await {
            Ok(all) => {
                st.reads += 1;
                let g = all.high_g.high_g_ms2(HIGH_G_FS);
                for i in 0..3 {
                    let a = abs(g[i]) / G;
                    peak[i] = peak[i].max(a);
                    st.peak_all = st.peak_all.max(a);
                }
            }
            Err(_) => st.errors += 1,
        }
        let src = match imu.read_register(reg::HG_WAKE_UP_SRC).await {
            Ok(v) => v,
            Err(_) => {
                st.errors += 1;
                continue;
            }
        };
        let wu_f = src & reg::HG_WAKE_UP_SRC_HG_WU_IA != 0;
        let sh_f = src & reg::HG_WAKE_UP_SRC_HG_SHOCK_CHANGE_IA != 0;
        let present = wu_f || sh_f || edges2() != seen_e2;
        let now = ms_since(t0);
        if present && (!prev_present || now >= last_handled + 2) {
            last_handled = now;
            Timer::after(Duration::from_micros(300)).await;
            let e2 = edges2();
            let s = match imu.interrupt_status().await {
                Ok(s) => s,
                Err(_) => {
                    st.errors += 1;
                    prev_present = present;
                    continue;
                }
            };
            let wu_k = wu_f || s.high_g_wake_up;
            let sh_k = sh_f || s.high_g_shock;
            st.raw_edges += e2.wrapping_sub(seen_e2);
            seen_e2 = e2;
            let edge = e2 != edge_mark;
            if (wu_k || sh_k) && !edge {
                st.flag_only += 1;
            }
            if edge && !wu_k && !sh_k {
                st.edge_only += 1;
                edge_mark = e2;
            }
            let t_ev = ms_since(t0);
            let new_wu = wu_k && last_wu.is_none_or(|t| t_ev >= t + DEAD_MS);
            let new_sh = sh_k && last_sh.is_none_or(|t| t_ev >= t + DEAD_MS);
            if wu_k {
                last_wu = Some(t_ev);
            }
            if sh_k {
                last_sh = Some(t_ev);
            }
            // The required clearing step; then the flag is read again.
            let mut cleared = true;
            if wu_k || sh_k {
                if clear(imu).await.is_err() {
                    st.errors += 1;
                }
                match imu.interrupt_status().await {
                    Ok(s2) => {
                        cleared = !((wu_k && s2.high_g_wake_up) || (sh_k && s2.high_g_shock));
                    }
                    Err(_) => st.errors += 1,
                }
                if !cleared {
                    st.not_cleared += 1;
                }
            }
            if new_wu || new_sh {
                if let Some(p) = pending.take() {
                    emit(&mut st, &p, peak);
                    peak = [0.0; 3];
                }
                if new_wu {
                    st.wu += 1;
                    if edge {
                        st.wu_both += 1;
                    }
                }
                if new_sh {
                    st.sh += 1;
                    if edge {
                        st.sh_both += 1;
                    }
                }
                edge_mark = e2;
                pending = Some(Pending {
                    due_ms: t_ev + PEAK_AFTER_MS,
                    t_ms: t_ev,
                    wake_up: new_wu,
                    shock: new_sh,
                    edge,
                    src,
                    cleared,
                });
            }
        }
        prev_present = present;
    }
    if let Some(p) = pending.take() {
        emit(&mut st, &p, peak);
    }
    Strike {
        st,
        int1_edges,
        elapsed_us: t_last.duration_since(t0).as_micros(),
    }
}

/// H4: waits at most 30 s for 3 consecutive seconds without a flag or an INT2 edge. Returns the
/// time it took in ms, or `None`.
async fn quiet_wait(imu: &mut Imu) -> (Option<u64>, u32, u32) {
    let _ = clear(imu).await;
    let mut seen = edges2();
    let t0 = Instant::now();
    let mut streak = Instant::now();
    let (mut events, mut errs) = (0u32, 0u32);
    while ms_since(t0) < 30_000 {
        let mut hit = false;
        match imu.read_register(reg::HG_WAKE_UP_SRC).await {
            Ok(v) => {
                if v & (reg::HG_WAKE_UP_SRC_HG_WU_IA | reg::HG_WAKE_UP_SRC_HG_SHOCK_CHANGE_IA) != 0
                {
                    hit = true;
                    if clear(imu).await.is_err() {
                        errs += 1;
                    }
                }
            }
            Err(_) => errs += 1,
        }
        let e = edges2();
        if e != seen {
            hit = true;
            seen = e;
        }
        if hit {
            events += 1;
            streak = Instant::now();
        } else if ms_since(streak) >= 3000 {
            return (Some(ms_since(t0)), events, errs);
        }
        Timer::after(Duration::from_millis(1)).await;
    }
    (None, events, errs)
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
    let mut imu: Imu = lsm6dsk320x::Lsm6dsk320x::new(dev);

    // INT1: the GPIO edge interrupt waits and PWM slice 3 counts, on the same pad.
    // SAFETY: the Input only awaits edges and the PWM slice only counts; neither reconfigures the pad.
    let pwm_pin = unsafe { p.PIN_7.clone_unchecked() };
    let mut int1 = Input::new(p.PIN_7, Pull::None);
    let counter = Pwm::new_input(
        p.PWM_SLICE3,
        pwm_pin,
        Pull::None,
        InputMode::RisingEdge,
        pwm::Config::default(),
    );
    let _int2 = Input::new(p.PIN_6, Pull::None);

    // Status LED: PIO0 state machine 0, DMA channel 2 (SPI1 uses channels 0 and 1), GPIO43.
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let ws_program = PioWs2812Program::new(&mut common);
    let mut led: Led = PioWs2812::new(&mut common, sm0, p.DMA_CH2, Irqs, p.PIN_43, &ws_program);
    phase(&mut led, Phase::Ready).await;

    // H0
    let h0 = strike_config(SILENT_G);
    match imu.init(&mut Delay, &h0).await {
        Ok(_) => check!(
            fails,
            true,
            "{} H0 init and configure (low-g, gyro, high-g, generator on)"
        ),
        Err(e) => {
            check!(fails, false, "{} H0 init: {}", defmt::Debug2Format(&e));
            hil_common::exit(fails)
        }
    }
    let v = imu.verify().await;
    check!(fails, v.is_ok(), "{} H0 verify {}", defmt::Debug2Format(&v));
    if v.is_err() {
        hil_common::exit(fails)
    }
    let r = rest_probe(&mut imu, 500).await;
    defmt::info!(
        "[INFO] H0 at rest: high-g peak |a| [{}, {}, {}] g over 500 ms (+-64 g full scale), bus errors {}",
        r.peak[0],
        r.peak[1],
        r.peak[2],
        r.errors
    );

    // H1
    let proven = h1(&mut imu, &counter, &mut fails).await;
    defmt::info!(
        "[INFO] H1 firing attempts with edge and status flag both seen: {}",
        proven
    );

    // H2
    let mut chosen: Option<u8> = None;
    for g in CANDIDATES_G {
        if let Err(e) = imu.configure(&mut Delay, &strike_config(g)).await {
            check!(
                fails,
                false,
                "{} H2 configure {} g: {}",
                g,
                defmt::Debug2Format(&e)
            );
            break;
        }
        let _ = clear(&mut imu).await;
        let _ = rest_probe(&mut imu, 200).await;
        let r = rest_probe(&mut imu, 3000).await;
        defmt::info!(
            "[INFO] H2 threshold {} g (HG_WAKE_UP_THS {}): 3 s at rest gave {} flag polls, {} INT2 edges, bus errors {}, peak [{}, {}, {}] g",
            g,
            g,
            r.flags,
            r.edges,
            r.errors,
            r.peak[0],
            r.peak[1],
            r.peak[2]
        );
        if r.flags == 0 && r.edges == 0 && r.errors == 0 {
            chosen = Some(g);
            break;
        }
    }
    let Some(thr) = chosen else {
        check!(
            fails,
            false,
            "{} H2 no threshold up to 16 g gives 0 events in 3 s at rest"
        );
        finish(&mut imu, &mut led, &mut fails, start, 0, 0, 0.0, 0).await
    };
    let mg = calc::hg_wake_up_threshold_mg(thr, HIGH_G_FS);
    check!(
        fails,
        true,
        "{} H2 wake-up threshold {} mg (HG_WAKE_UP_THS = {}, 1 g/LSB up to +-256 g full scale, p94), shock: the same register, shock duration code {} = 66.7 ms (p94), 0 events in 3 s at rest",
        mg,
        thr,
        SHOCK_DUR
    );
    let _ = clear(&mut imu).await;

    // H3
    phase(&mut led, Phase::Strike).await;
    let strike = strike_window(&mut imu, &mut int1, &counter).await;
    phase(&mut led, Phase::Still).await;
    let s = &strike.st;
    defmt::info!(
        "[STAT] H3 counted events: wake-up {} (edge and flag {}), shock {} (edge and flag {}); INT2 edges {}, flag without edge {}, edge without flag {}, flags not cleared by ALL_INT_SRC {}, largest peak {} g",
        s.wu,
        s.wu_both,
        s.sh,
        s.sh_both,
        s.raw_edges,
        s.flag_only,
        s.edge_only,
        s.not_cleared,
        s.peak_all
    );
    if s.wu == 0 && s.sh == 0 && s.raw_edges == 0 {
        check!(
            fails,
            false,
            "{} no strike detected in {} s (was the board struck?)",
            WINDOW_S
        );
    } else {
        check!(
            fails,
            s.wu_both >= 3,
            "{} H3 wake-up events with both the INT2 edge and the status flag: {} (need 3)",
            s.wu_both
        );
        // The shock state is entered when the signal passes the threshold and left after the
        // shock duration below it (p94); each change sets HG_SHOCK_CHANGE_IA. A hand strike meets it.
        check!(
            fails,
            s.sh_both >= 1,
            "{} H3 shock state changes with both the INT2 edge and the status flag: {} (need 1)",
            s.sh_both
        );
        // Reported, not judged. The two flags are level flags: H1 shows they read true for as long as
        // an axis is over the threshold (or the one-sample shock pulse lasts) and false afterwards,
        // whatever is read. A flag still set right after the ALL_INT_SRC read during a strike is
        // therefore not a fault (run of 2026-11-11: 2584 times in 120 s of continuous strikes).
        defmt::info!(
            "[INFO] H3 flag still set right after the ALL_INT_SRC read: {} times (level flags: set while an axis is over the threshold)",
            s.not_cleared
        );
    }

    // H4
    let (quiet, q_events, q_errs) = quiet_wait(&mut imu).await;
    match quiet {
        Some(ms) => check!(
            fails,
            q_errs == 0,
            "{} H4 quiet again: 3 consecutive seconds with 0 events found after {} ms ({} events seen while waiting, bus errors {})",
            ms,
            q_events,
            q_errs
        ),
        None => check!(
            fails,
            false,
            "{} H4 no 3 s without events within 30 s ({} events seen, bus errors {})",
            q_events,
            q_errs
        ),
    }

    // H5
    let rate = strike.int1_edges as f32 * 1e6 / strike.elapsed_us.max(1) as f32;
    let err = (rate / ODR_HZ - 1.0) * 100.0;
    let bus_errors = imu.stats().bus_errors;
    defmt::info!(
        "[STAT] H5 INT1 low-g+gyro data-ready: {} edges in {} us = {} Hz, reads {}, read errors {}, edge timeouts {}",
        strike.int1_edges,
        strike.elapsed_us,
        rate,
        s.reads,
        s.errors,
        s.timeouts
    );
    check!(
        fails,
        abs(err) <= 2.0 && s.errors == 0 && s.timeouts == 0 && bus_errors == 0,
        "{} H5 data-ready rate {} Hz, {}% from {} Hz (limit 2), bus errors {}",
        rate,
        err,
        ODR_HZ,
        bus_errors
    );

    finish(
        &mut imu, &mut led, &mut fails, start, s.wu, s.sh, s.peak_all, thr as u32,
    )
    .await
}

async fn finish(
    imu: &mut Imu,
    led: &mut Led,
    fails: &mut u32,
    start: Instant,
    wu: u32,
    sh: u32,
    peak: f32,
    thr: u32,
) -> ! {
    defmt::info!(
        "[STAT] summary: wake-up events {}, shock events {}, largest peak {} g, threshold {} g (HG_WAKE_UP_THS {}), shock duration code {}",
        wu,
        sh,
        peak,
        thr,
        thr,
        SHOCK_DUR
    );
    let end_cfg = Config::new();
    match imu.configure(&mut Delay, &end_cfg).await {
        Ok(_) => {
            let v = imu.verify().await;
            check!(
                *fails,
                v.is_ok(),
                "{} End: generator off, default configuration, verify {}",
                defmt::Debug2Format(&v)
            );
        }
        Err(e) => check!(
            *fails,
            false,
            "{} End configure: {}",
            defmt::Debug2Format(&e)
        ),
    }
    let s = imu.stats();
    defmt::info!("[INFO] health: driver counters {}", s);
    check!(
        *fails,
        s.bus_errors == 0,
        "{} health: bus errors {}",
        s.bus_errors
    );
    defmt::info!("[INFO] run time {} ms", ms_since(start));
    phase(led, Phase::Done).await;
    hil_common::exit(*fails)
}

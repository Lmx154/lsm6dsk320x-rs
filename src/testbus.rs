//! Host test harness: a fake SPI device over a 256-byte register file, a fake delay, a counter of
//! await points, and a minimal executor. One copy per driver repository, on purpose.
//!
//! The chip, the bus and the delay share one state so that a test can inspect the traffic after a
//! future was dropped. Awaits are counted in one sequence across bus and delay; a pend at await
//! `k` returns `Poll::Pending` once, before the operation takes effect, and completes afterwards.

use core::future::Future;
use core::pin::{Pin, pin};
use core::task::{Context, Poll, Waker};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};
use std::vec::Vec;

use embedded_hal::spi::{ErrorKind, ErrorType, Operation};

use crate::reg;

/// The bus error the fake raises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BusFault;

impl embedded_hal::spi::Error for BusFault {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// What one operation of a transaction put on the bus and got back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Op {
    /// `Operation::Write`: the bytes sent.
    Write(Vec<u8>),
    /// `Operation::Read`: the bytes returned.
    Read(Vec<u8>),
    /// `Operation::TransferInPlace`: the bytes sent and the bytes returned.
    InPlace(Vec<u8>, Vec<u8>),
}

/// One logged event, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    /// A completed transaction.
    Spi(Vec<Op>),
    /// A delay, in nanoseconds.
    Delay(u64),
}

/// The self-test stimulus model: while the stimulus bits of a channel are set (ST_XL and ST_G in
/// CTRL10, XL_HG_ST in CTRL2_XL_HG), its output registers return `rest` plus (positive sign) or
/// minus (negative sign) `delta`, per axis; otherwise `rest`. STATUS_REG then shows the data-ready
/// bit of every channel except those in `hold`. Channels are indexed low-g, gyroscope, high-g.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct StModel {
    pub rest: [[i16; 3]; 3],
    pub delta: [[i16; 3]; 3],
    pub hold: [bool; 3],
}

impl StModel {
    const BASES: [u8; 3] = [reg::OUTX_L_A, reg::OUTX_L_G, reg::UI_OUTX_L_A_OIS_HG];
    const READY: [u8; 3] = [reg::STATUS_XLDA, reg::STATUS_GDA, reg::STATUS_XLHGDA];

    fn status(&self) -> u8 {
        (0..3)
            .filter(|&c| !self.hold[c])
            .fold(0, |v, c| v | Self::READY[c])
    }

    /// The byte at register `a`, if it is an output register of a channel.
    fn byte(&self, regs: &[u8; 256], a: usize) -> Option<u8> {
        let c = (0..3)
            .find(|&c| (Self::BASES[c] as usize..Self::BASES[c] as usize + 6).contains(&a))?;
        let code = match c {
            0 => regs[reg::CTRL10 as usize] & reg::CTRL10_ST_XL_MASK,
            1 => (regs[reg::CTRL10 as usize] & reg::CTRL10_ST_G_MASK) >> reg::CTRL10_ST_G_SHIFT,
            _ => regs[reg::CTRL2_XL_HG as usize] & reg::CTRL2_XL_HG_ST_MASK,
        };
        let i = a - Self::BASES[c] as usize;
        let (rest, delta) = (self.rest[c][i / 2], self.delta[c][i / 2]);
        let v = match code {
            reg::SELF_TEST_POSITIVE => rest.wrapping_add(delta),
            reg::SELF_TEST_NEGATIVE => rest.wrapping_sub(delta),
            _ => rest,
        };
        Some(v.to_le_bytes()[i % 2])
    }
}

pub(crate) struct Chip {
    /// The self-test stimulus model; `None` leaves the output registers as plain memory.
    pub model: Option<StModel>,
    pub regs: [u8; 256],
    pub log: Vec<Event>,
    /// Transactions started since the last `arm` (failed ones included).
    pub transactions: usize,
    /// Awaits reached since the last `arm`.
    pub awaits: u32,
    /// Fail this transaction (1-based) with `BusFault`; it has no effect on the register file.
    pub fail_at: Option<usize>,
    /// Return `Pending` once at this await (1-based).
    pub pend_at: Option<u32>,
    pub pended: bool,
    /// Bus rules the driver broke: a read without a preceding address byte, a read done by an
    /// equal-length transfer (first byte has the read bit), `Operation::DelayNs`.
    pub violations: usize,
    /// Flip these bits in the first byte read by the `Operation::Read` of this transaction
    /// (1-based, counted like `fail_at`), once; the register file is not changed.
    pub glitch: Option<(usize, u8)>,
    /// CTRL3 SW_RESET never clears.
    pub reset_stuck: bool,
    /// The FIFO as the bytes of its words in order, 7 per word (6.5.8, p46). A read of
    /// FIFO_DATA_OUT_TAG to FIFO_DATA_OUT_Z_H pops one byte per register; the address returns from
    /// 0x7E to 0x78 inside a burst, so a burst runs across word boundaries. An empty FIFO reads 0
    /// (tag 0, "FIFO empty").
    pub fifo: VecDeque<u8>,
    /// FIFO_FULL_IA, set by the test.
    pub fifo_full: bool,
    /// FIFO_OVR_IA, set by the test.
    pub fifo_overrun: bool,
    /// FIFO_OVR_LATCHED, set by the test and cleared when FIFO_STATUS2 is read (p75).
    pub fifo_overrun_latched: bool,
}

impl Chip {
    fn new() -> Self {
        let mut regs = [0u8; 256];
        regs[reg::WHO_AM_I as usize] = reg::WHO_AM_I_VALUE;
        regs[reg::CTRL3 as usize] = reg::CTRL3_RESET;
        regs[reg::INTERNAL_FREQ_FINE as usize] = 0xFD;
        // Distinct non-trivial bytes over the whole output block and the status register.
        for (i, r) in regs[reg::STATUS_REG as usize..=0x3F].iter_mut().enumerate() {
            *r = (i as u8).wrapping_mul(37).wrapping_add(11);
        }
        Self {
            model: None,
            regs,
            log: Vec::new(),
            transactions: 0,
            awaits: 0,
            fail_at: None,
            pend_at: None,
            pended: false,
            violations: 0,
            glitch: None,
            reset_stuck: false,
            fifo: VecDeque::new(),
            fifo_full: false,
            fifo_overrun: false,
            fifo_overrun_latched: false,
        }
    }

    /// Unread whole words: DIFF_FIFO (9 bits).
    pub fn fifo_words(&self) -> usize {
        self.fifo.len() / reg::FIFO_WORD_LEN
    }

    /// FIFO_STATUS2 as derived from the queue and the flags; the watermark flag is set when the
    /// threshold in FIFO_CTRL1 is not 0 and the unread words reach it.
    fn fifo_status2(&self) -> u8 {
        let words = self.fifo_words();
        let wtm = usize::from(self.regs[reg::FIFO_CTRL1 as usize]);
        let mut v = ((words >> 8) & 1) as u8;
        if wtm > 0 && words >= wtm {
            v |= reg::FIFO_STATUS2_FIFO_WTM_IA;
        }
        if self.fifo_full {
            v |= reg::FIFO_STATUS2_FIFO_FULL_IA;
        }
        if self.fifo_overrun {
            v |= reg::FIFO_STATUS2_FIFO_OVR_IA;
        }
        if self.fifo_overrun_latched {
            v |= reg::FIFO_STATUS2_FIFO_OVR_LATCHED;
        }
        v
    }

    fn read_register_byte(&mut self, a: usize) -> u8 {
        match a {
            x if x == reg::FIFO_STATUS1 as usize => (self.fifo_words() & 0xFF) as u8,
            x if x == reg::FIFO_STATUS2 as usize => {
                let v = self.fifo_status2();
                self.fifo_overrun_latched = false;
                v
            }
            x if (reg::FIFO_DATA_OUT_TAG as usize..=reg::FIFO_DATA_OUT_Z_H as usize)
                .contains(&x) =>
            {
                self.fifo.pop_front().unwrap_or(0)
            }
            x if x == reg::STATUS_REG as usize && self.model.is_some() => {
                self.model.map_or(0, |m| m.status())
            }
            _ => match self.model.and_then(|m| m.byte(&self.regs, a)) {
                Some(v) => v,
                None => self.regs[a],
            },
        }
    }

    /// The chip's framing: bit 7 of the first byte is read, the rest is the address, and the
    /// address increments after every data byte.
    fn clock(&mut self, header: &mut Option<u8>, addr: &mut u8, byte: u8) -> u8 {
        let Some(h) = *header else {
            *header = Some(byte);
            *addr = byte & 0x7F;
            return 0;
        };
        let a = *addr as usize;
        *addr = if a == reg::FIFO_DATA_OUT_Z_H as usize {
            reg::FIFO_DATA_OUT_TAG
        } else {
            addr.wrapping_add(1)
        };
        if h & reg::SPI_READ != 0 {
            self.read_register_byte(a)
        } else {
            if a == reg::FIFO_CTRL4 as usize
                && (byte ^ self.regs[a]) & reg::FIFO_CTRL4_FIFO_MODE_MASK != 0
            {
                self.fifo.clear();
                self.fifo_full = false;
                self.fifo_overrun = false;
                self.fifo_overrun_latched = false;
            }
            self.regs[a] = if a == reg::CTRL3 as usize && !self.reset_stuck {
                byte & !reg::CTRL3_SW_RESET
            } else {
                byte
            };
            0
        }
    }

    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), BusFault> {
        self.transactions += 1;
        if self.fail_at == Some(self.transactions) {
            return Err(BusFault);
        }
        let (mut header, mut addr) = (None, 0u8);
        let mut ops = Vec::new();
        for op in operations {
            match op {
                Operation::Write(w) => {
                    for &b in w.iter() {
                        self.clock(&mut header, &mut addr, b);
                    }
                    ops.push(Op::Write(w.to_vec()));
                }
                Operation::Read(r) => {
                    if header.is_none() {
                        self.violations += 1;
                    }
                    for b in r.iter_mut() {
                        *b = self.clock(&mut header, &mut addr, 0);
                    }
                    if let (Some((n, xor)), Some(first)) = (self.glitch, r.first_mut())
                        && n == self.transactions
                    {
                        *first ^= xor;
                        self.glitch = None;
                    }
                    ops.push(Op::Read(r.to_vec()));
                }
                Operation::TransferInPlace(t) => {
                    if t.first().is_some_and(|b| b & reg::SPI_READ != 0) {
                        self.violations += 1;
                    }
                    let sent = t.to_vec();
                    for b in t.iter_mut() {
                        *b = self.clock(&mut header, &mut addr, *b);
                    }
                    ops.push(Op::InPlace(sent, t.to_vec()));
                }
                Operation::Transfer(r, w) => {
                    if w.first().is_some_and(|b| b & reg::SPI_READ != 0) {
                        self.violations += 1;
                    }
                    for (o, i) in r.iter_mut().zip(w.iter()) {
                        *o = self.clock(&mut header, &mut addr, *i);
                    }
                }
                Operation::DelayNs(_) => self.violations += 1,
            }
        }
        self.log.push(Event::Spi(ops));
        Ok(())
    }
}

/// The shared state; clone it to keep a handle after the bus has moved into the driver.
#[derive(Clone)]
pub(crate) struct Shared(Arc<Mutex<Chip>>);

impl Shared {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(Chip::new())))
    }

    pub fn chip(&self) -> MutexGuard<'_, Chip> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn bus(&self) -> TestBus {
        TestBus(self.clone())
    }

    pub fn delay(&self) -> TestDelay {
        TestDelay(self.clone())
    }

    /// Clears the log and the counters and sets the injections; the register file stays.
    pub fn arm(&self, fail_at: Option<usize>, pend_at: Option<u32>) {
        let mut c = self.chip();
        c.log.clear();
        c.transactions = 0;
        c.awaits = 0;
        c.fail_at = fail_at;
        c.pend_at = pend_at;
        c.pended = false;
        c.violations = 0;
        c.glitch = None;
    }

    /// Queues one FIFO word (7 bytes) at the end of the FIFO. The FIFO holds
    /// [`crate::FIFO_DEPTH_WORDS`] words (256, measured on the board, 2026-11-10); a test that
    /// queues more is wrong.
    pub fn push_word(&self, word: [u8; reg::FIFO_WORD_LEN]) {
        let mut c = self.chip();
        assert!(
            c.fifo_words() < usize::from(crate::FIFO_DEPTH_WORDS),
            "the fake FIFO is full"
        );
        c.fifo.extend(word);
    }

    /// Queues `n` distinct words and returns them as one byte string. Word `i` has the gyroscope
    /// tag (0x01 << 3) and six bytes derived from `seed` and `i`.
    pub fn push_words(&self, n: usize, seed: u8) -> Vec<u8> {
        let mut all = Vec::new();
        for i in 0..n {
            let b = seed.wrapping_add(i as u8).wrapping_mul(7);
            let word = [
                0x01 << 3,
                b,
                b.wrapping_add(1),
                b.wrapping_add(2),
                b.wrapping_add(3),
                b.wrapping_add(4),
                (i >> 8) as u8 ^ b.wrapping_add(5),
            ];
            self.push_word(word);
            all.extend(word);
        }
        all
    }

    /// The unread words in the fake FIFO.
    pub fn fifo_words(&self) -> usize {
        self.chip().fifo_words()
    }

    /// Sets the FIFO_FULL_IA, FIFO_OVR_IA and FIFO_OVR_LATCHED flags the status register shows.
    pub fn set_fifo_flags(&self, full: bool, overrun: bool, latched: bool) {
        let mut c = self.chip();
        c.fifo_full = full;
        c.fifo_overrun = overrun;
        c.fifo_overrun_latched = latched;
    }

    /// Switches the self-test stimulus model on with these resting values and stimulus deltas.
    pub fn model(&self, rest: [[i16; 3]; 3], delta: [[i16; 3]; 3]) {
        self.chip().model = Some(StModel {
            rest,
            delta,
            hold: [false; 3],
        });
    }

    /// Holds back (or releases) the data-ready bit of channel `c` (0 low-g, 1 gyroscope, 2 high-g).
    pub fn hold_ready(&self, c: usize, hold: bool) {
        if let Some(m) = self.chip().model.as_mut()
            && let Some(h) = m.hold.get_mut(c)
        {
            *h = hold;
        }
    }

    /// Corrupts one read once: flips `xor` in the first byte read in transaction `n` (1-based).
    pub fn glitch(&self, n: usize, xor: u8) {
        self.chip().glitch = Some((n, xor));
    }

    pub fn log(&self) -> Vec<Event> {
        self.chip().log.clone()
    }

    pub fn awaits(&self) -> u32 {
        self.chip().awaits
    }

    pub fn transactions(&self) -> usize {
        self.chip().transactions
    }

    pub fn violations(&self) -> usize {
        self.chip().violations
    }
}

/// An await point: counts itself, and pends once if it is the chosen one.
struct Tick {
    shared: Shared,
    counted: bool,
}

impl Future for Tick {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.counted {
            return Poll::Ready(());
        }
        self.counted = true;
        let mut c = self.shared.chip();
        c.awaits += 1;
        if c.pend_at == Some(c.awaits) && !c.pended {
            c.pended = true;
            return Poll::Pending;
        }
        Poll::Ready(())
    }
}

fn tick(shared: &Shared) -> Tick {
    Tick {
        shared: shared.clone(),
        counted: false,
    }
}

/// The fake SPI device. Both the async and the blocking trait are implemented over one state.
pub(crate) struct TestBus(Shared);

impl ErrorType for TestBus {
    type Error = BusFault;
}

impl embedded_hal_async::spi::SpiDevice for TestBus {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), BusFault> {
        tick(&self.0).await;
        self.0.chip().transaction(operations)
    }
}

impl embedded_hal::spi::SpiDevice for TestBus {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), BusFault> {
        self.0.chip().transaction(operations)
    }
}

/// The fake delay; it records and does not wait.
pub(crate) struct TestDelay(Shared);

impl embedded_hal_async::delay::DelayNs for TestDelay {
    async fn delay_ns(&mut self, ns: u32) {
        tick(&self.0).await;
        self.0.chip().log.push(Event::Delay(u64::from(ns)));
    }
}

impl embedded_hal::delay::DelayNs for TestDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.chip().log.push(Event::Delay(u64::from(ns)));
    }
}

/// Runs a future to completion with a no-op waker, polling again after every `Pending`.
pub(crate) fn run<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(v) = future.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// How a future that was polled until its first `Pending` ended.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Polled<T> {
    /// It completed without ever pending: the chosen await is beyond its last.
    Done(T),
    /// It returned `Pending` and was dropped there.
    Cancelled,
}

/// Polls until the first `Pending`, then drops the future.
pub(crate) fn poll_then_drop<F: Future>(future: F) -> Polled<F::Output> {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    match future.as_mut().poll(&mut cx) {
        Poll::Ready(v) => Polled::Done(v),
        Poll::Pending => Polled::Cancelled,
    }
}

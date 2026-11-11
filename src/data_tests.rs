//! Host tests of the data path: `fifo_read`, `fifo_read_exact`, `fifo_status`, `fifo_flush`,
//! `interrupt_status`, `stats` and `info`, on the fake bus with its FIFO.

use std::vec;
use std::vec::Vec;

use proptest::prelude::{ProptestConfig, prop_assert, prop_assert_eq, proptest};

use crate::driver_tests::{Imu, Res, configured, rig, w};
use crate::testbus::{BusFault, Event, Op, Polled, Shared, TestDelay, poll_then_drop, run};
use crate::{
    BatchRate, Config, Error, FifoConfig, FifoFlags, FifoFrame, FifoMode, FifoRead, FifoStatus,
    HighGConfig, HighGEvents, IntRouting, InterruptStatus, Stats, blocking, fifo_frames, reg,
};

fn fifo_config() -> Config {
    Config::new().with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::Continuous)
            .with_watermark(10)
            .with_gyro(BatchRate::Hz480),
    )
}

/// FIFO on, and a high-g wake-up routed with its events enabled: `interrupt_status` reads
/// HG_WAKE_UP_SRC.
fn events_config() -> Config {
    fifo_config()
        .with_high_g(Some(HighGConfig::new()))
        .with_int1(IntRouting::new().with_high_g_wake_up(true))
        .with_high_g_events(Some(HighGEvents::new()))
}

fn rig_with(c: &Config) -> (Shared, Imu, TestDelay) {
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, c)).unwrap();
    (shared, imu, delay)
}

/// A read of `bytes.len()` registers from `reg`: `[Write(reg | 0x80), Read(bytes)]`.
fn rd(reg: u8, bytes: &[u8]) -> Event {
    Event::Spi(vec![Op::Write(vec![reg | 0x80]), Op::Read(bytes.to_vec())])
}

// ---------------------------------------------------------------------------------------------
// Golden traces (D2, E1)
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_read_golden_trace_two_transactions_then_one_when_empty() {
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    let words = shared.push_words(5, 1);
    let mut buf = [0u8; 100];
    shared.arm(None, None);
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    // FIFO_STATUS1 (0x1B, 9.25, p75) and FIFO_STATUS2 (0x1C, 9.26, p75): 5 words, no flag (the
    // watermark is 10). FIFO_DATA_OUT_TAG (0x78, Table 26, p55): 35 bytes in one burst (6.5.8, p46).
    assert_eq!(
        shared.log(),
        vec![rd(0x1B, &[5, 0x00]), rd(0x78, &words)],
        "two transactions"
    );
    assert_eq!(shared.transactions(), 2);
    assert_eq!(shared.awaits(), 2);
    assert_eq!(
        got,
        FifoRead {
            bytes: 35,
            words: 5,
            unread: Some(0),
            flags: Some(FifoFlags::default()),
        }
    );
    assert_eq!(buf[..35], words[..]);
    assert_eq!(imu.stats().fifo_words, 5);

    // Nothing waiting: the status only.
    shared.arm(None, None);
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    assert_eq!(shared.log(), vec![rd(0x1B, &[0, 0x00])]);
    assert_eq!(shared.transactions(), 1);
    assert_eq!(got.bytes, 0);
    assert_eq!(got.words, 0);
    assert_eq!(got.unread, Some(0));
    assert_eq!(imu.stats().fifo_words, 5);
    assert_eq!(shared.violations(), 0);
}

#[test]
fn fifo_read_reads_only_whole_words_that_fit_and_reports_the_rest() {
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    let words = shared.push_words(12, 9);
    let mut buf = [0xEEu8; 34]; // room for 4 words and 6 spare bytes
    shared.arm(None, None);
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    // 12 words are at or above the watermark of 10: FIFO_WTM_IA (bit 7, p75).
    assert_eq!(
        shared.log(),
        vec![rd(0x1B, &[12, 0x80]), rd(0x78, &words[..28])]
    );
    assert_eq!(got.bytes, 28);
    assert_eq!(got.words, 4);
    assert_eq!(got.unread, Some(8));
    assert!(got.flags.is_some_and(|f| f.watermark));
    assert_eq!(buf[28..], [0xEE; 6]);
    assert_eq!(shared.fifo_words(), 8);
    // The next call goes on with the next word.
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    assert_eq!(buf[..28], words[28..56]);
    assert_eq!((got.words, got.unread), (4, Some(4)));
}

#[test]
fn fifo_read_exact_golden_trace_is_one_transaction() {
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    let words = shared.push_words(4, 2);
    let mut buf = [0u8; 40];
    shared.arm(None, None);
    let got = run(imu.fifo_read_exact(&mut buf, 3)).unwrap();
    // One burst of 3 words from FIFO_DATA_OUT_TAG (0x78, Table 26, p55; 6.5.8, p46); no status.
    assert_eq!(shared.log(), vec![rd(0x78, &words[..21])]);
    assert_eq!(shared.transactions(), 1);
    assert_eq!(shared.awaits(), 1);
    assert_eq!(
        got,
        FifoRead {
            bytes: 21,
            words: 3,
            unread: None,
            flags: None,
        }
    );
    assert_eq!(buf[..21], words[..21]);
    assert_eq!(shared.fifo_words(), 1);
    assert_eq!(imu.stats().fifo_words, 3);
    assert_eq!(shared.violations(), 0);
}

#[test]
fn fifo_status_golden_trace_is_one_transaction_with_nine_bit_count() {
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    shared.push_words(256, 0);
    shared.arm(None, None);
    let got = run(imu.fifo_status()).unwrap();
    // DIFF_FIFO[8:0] = 256 = 0x100, a full FIFO (measured on the board, 2026-11-10): FIFO_STATUS1
    // 0x00, FIFO_STATUS2 bit 0 (p75); watermark set.
    assert_eq!(shared.log(), vec![rd(0x1B, &[0x00, 0x81])]);
    assert_eq!(shared.transactions(), 1);
    assert_eq!(got.words, 256);
    assert!(got.flags.watermark);
}

#[test]
fn interrupt_status_golden_traces_one_transaction_or_three() {
    // Without high-g events: FIFO_STATUS1, FIFO_STATUS2, ALL_INT_SRC, STATUS_REG (0x1B to 0x1E,
    // p75-p77) in one read.
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    shared.chip().regs[reg::STATUS_REG as usize] = 0b1000_1011;
    shared.chip().regs[reg::ALL_INT_SRC as usize] = 0xFF;
    shared.push_words(10, 0);
    shared.set_fifo_flags(false, true, false);
    shared.arm(None, None);
    let got = run(imu.interrupt_status()).unwrap();
    assert_eq!(shared.log(), vec![rd(0x1B, &[10, 0xC0, 0xFF, 0b1000_1011])]);
    assert_eq!(shared.transactions(), 1);
    // STATUS_REG (p77): XLDA, GDA, XLHGDA (bit 3) and TIMESTAMP_ENDCOUNT (bit 7).
    assert_eq!(
        got,
        InterruptStatus {
            accel_data_ready: true,
            gyro_data_ready: true,
            high_g_data_ready: true,
            timestamp_overflow: true,
            fifo: FifoFlags {
                watermark: true,
                overrun: true,
                ..FifoFlags::default()
            },
            high_g_wake_up: false,
            high_g_shock: false,
        }
    );
    assert_eq!(imu.stats().fifo_overruns, 1);

    // With high-g events: FIFO_STATUS (2), HG_WAKE_UP_SRC 0x4C (p90) and STATUS_REG 0x1E; the
    // ALL_INT_SRC register is not read (p93).
    let (shared, mut imu, _d) = rig_with(&events_config());
    shared.chip().regs[reg::STATUS_REG as usize] = 0b0000_0010;
    shared.chip().regs[reg::HG_WAKE_UP_SRC as usize] = 0b0100_1000;
    shared.arm(None, None);
    let got = run(imu.interrupt_status()).unwrap();
    assert_eq!(
        shared.log(),
        vec![
            rd(0x1B, &[0, 0x00]),
            rd(0x4C, &[0b0100_1000]),
            rd(0x1E, &[0b0000_0010])
        ]
    );
    assert_eq!(shared.transactions(), 3);
    assert!(got.high_g_wake_up && got.high_g_shock && got.gyro_data_ready);
    assert!(!got.accel_data_ready && !got.timestamp_overflow);
}

#[test]
fn fifo_flush_golden_trace_empties_the_fifo_and_restores_the_mode() {
    let c = fifo_config();
    let (shared, mut imu, _d) = rig_with(&c);
    shared.push_words(20, 0);
    shared.set_fifo_flags(true, true, true);
    let image = c.fifo_ctrl4();
    assert_eq!(image, 0x06); // FIFO_MODE = 110 continuous (9.8, p61)
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_flush()), Ok(()));
    // Bypass resets the FIFO (6.5.1, 6.5.2, p45), then the applied mode is written back.
    assert_eq!(shared.log(), vec![w(0x0A, 0x00), w(0x0A, 0x06)]);
    assert_eq!(shared.transactions(), 2);
    assert_eq!(shared.fifo_words(), 0);
    assert_eq!(shared.chip().regs[reg::FIFO_CTRL4 as usize], image);
    assert_eq!(run(imu.verify()), Ok(()));
    // Back in its mode: new words are kept and read.
    let words = shared.push_words(2, 5);
    let mut buf = [0u8; 14];
    assert_eq!(run(imu.fifo_read(&mut buf)).unwrap().words, 2);
    assert_eq!(buf[..], words[..]);
    assert_eq!(
        run(imu.fifo_status()).unwrap().flags,
        FifoFlags::default(),
        "the flags went with the words"
    );
}

#[test]
fn fifo_flush_interrupted_after_the_first_write_is_found_by_verify_and_put_right_by_a_repeat() {
    let c = fifo_config();
    let (shared, mut imu, _d) = rig_with(&c);
    shared.push_words(3, 0);
    shared.arm(Some(2), None);
    assert_eq!(run(imu.fifo_flush()), Err(Error::Bus(BusFault)));
    shared.arm(None, None);
    assert!(matches!(
        run(imu.verify()),
        Err(Error::ConfigMismatch(m)) if m.register == reg::FIFO_CTRL4
    ));
    assert_eq!(run(imu.fifo_flush()), Ok(()));
    assert_eq!(run(imu.verify()), Ok(()));
}

// ---------------------------------------------------------------------------------------------
// Words (C1): proptest
// ---------------------------------------------------------------------------------------------

proptest! {
    #![proptest_config(ProptestConfig::with_cases(600))]

    #[test]
    fn fifo_read_never_splits_a_word_or_exceeds_the_buffer(
        waiting in 0usize..=usize::from(crate::FIFO_DEPTH_WORDS),
        len in 0usize..=2000,
        seed in proptest::prelude::any::<u8>(),
    ) {
        let (shared, mut imu, _d) = rig_with(&fifo_config());
        let words = shared.push_words(waiting, seed);
        let mut buf = vec![0xEEu8; len];
        shared.arm(None, None);
        let res = run(imu.fifo_read(&mut buf));
        if len < 7 {
            prop_assert_eq!(res, Err(Error::BufferTooSmall));
            prop_assert_eq!(shared.transactions(), 0);
        } else {
            let got = res.unwrap();
            let n = waiting.min(len / 7);
            prop_assert_eq!(got.words, n);
            prop_assert_eq!(got.bytes, 7 * n);
            prop_assert!(got.bytes <= len);
            prop_assert_eq!(got.bytes % 7, 0);
            prop_assert_eq!(got.unread, Some((waiting - n) as u16));
            prop_assert_eq!(shared.transactions(), if n == 0 { 1 } else { 2 });
            prop_assert_eq!(&buf[..got.bytes], &words[..7 * n]);
            prop_assert!(buf[got.bytes..].iter().all(|&b| b == 0xEE));
            prop_assert_eq!(shared.fifo_words(), waiting - n);
            let frames: Vec<_> = fifo_frames(&buf[..got.bytes]).collect();
            let expected: Vec<_> = fifo_frames(&words[..7 * n]).collect();
            prop_assert_eq!(frames.len(), n);
            prop_assert!(frames.iter().all(|f| !matches!(f, FifoFrame::Trailing(_))));
            prop_assert_eq!(frames, expected);
            prop_assert_eq!(imu.stats().fifo_words, n as u32);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Flags and stats (C4)
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_status_reports_every_flag_and_the_stats_count_an_overrun_once_per_call() {
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    // Watermark: 10 words reach the threshold of 10 (p75).
    shared.push_words(10, 0);
    let s = run(imu.fifo_status()).unwrap();
    assert_eq!(s.words, 10);
    assert_eq!(
        s.flags,
        FifoFlags {
            watermark: true,
            ..FifoFlags::default()
        }
    );
    // Full.
    shared.set_fifo_flags(true, false, false);
    assert_eq!(
        run(imu.fifo_status()).unwrap().flags,
        FifoFlags {
            watermark: true,
            full: true,
            ..FifoFlags::default()
        }
    );
    assert_eq!(imu.stats().fifo_overruns, 0);
    // Overrun now: counted by each call that sees it.
    shared.set_fifo_flags(false, true, false);
    let f = run(imu.fifo_status()).unwrap().flags;
    assert!(f.overrun && !f.overrun_latched && f.any_overrun());
    run(imu.fifo_status()).unwrap();
    assert_eq!(imu.stats().fifo_overruns, 2);
    // Latched overrun: the read clears it (p75), so it is reported and counted once.
    shared.set_fifo_flags(false, false, true);
    let f = run(imu.fifo_status()).unwrap().flags;
    assert!(f.overrun_latched && !f.overrun);
    assert_eq!(imu.stats().fifo_overruns, 3);
    let f = run(imu.fifo_status()).unwrap().flags;
    assert!(!f.any_overrun());
    assert_eq!(imu.stats().fifo_overruns, 3);
    // fifo_read counts too, and a latched overrun seen by interrupt_status is not lost.
    shared.set_fifo_flags(false, false, true);
    let mut buf = [0u8; 7];
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    assert!(got.flags.is_some_and(|f| f.overrun_latched));
    assert_eq!(imu.stats().fifo_overruns, 4);
    shared.set_fifo_flags(false, false, true);
    assert!(run(imu.interrupt_status()).unwrap().fifo.overrun_latched);
    assert_eq!(imu.stats().fifo_overruns, 5);
    // The words of the one fifo_read.
    assert_eq!(imu.stats().fifo_words, 1);
}

#[test]
fn fifo_read_counts_an_overrun_even_when_the_data_transaction_fails() {
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    shared.push_words(3, 0);
    shared.set_fifo_flags(false, false, true);
    shared.arm(Some(2), None);
    let mut buf = [0u8; 21];
    assert_eq!(run(imu.fifo_read(&mut buf)), Err(Error::Bus(BusFault)));
    assert_eq!(
        imu.stats().fifo_overruns,
        1,
        "the status read cleared the latch"
    );
    assert_eq!(imu.stats().fifo_words, 0);
    assert_eq!(imu.stats().bus_errors, 1);
}

#[test]
fn stats_start_at_zero_count_bus_errors_and_saturate() {
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    assert_eq!(*imu.stats(), Stats::default());
    assert_eq!(Stats::new(), Stats::default());
    shared.arm(Some(1), None);
    assert_eq!(run(imu.read()), Err(Error::Bus(BusFault)));
    shared.arm(None, None);
    assert_eq!(imu.stats().bus_errors, 1);
    // Nothing clears them: not a new init either.
    shared.push_words(2, 0);
    let mut buf = [0u8; 14];
    run(imu.fifo_read(&mut buf)).unwrap();
    let mut delay = shared.delay();
    run(imu.init(&mut delay, &fifo_config())).unwrap();
    assert_eq!(imu.stats().fifo_words, 2);
    assert_eq!(imu.stats().bus_errors, 1);

    // Saturation: each counter at its maximum stays there.
    *imu.stats_mut() = Stats {
        fifo_overruns: u32::MAX,
        fifo_words: u32::MAX,
        bus_errors: u32::MAX,
    };
    shared.push_words(1, 0);
    shared.set_fifo_flags(false, true, false);
    run(imu.fifo_read(&mut buf)).unwrap();
    assert_eq!(imu.stats().fifo_overruns, u32::MAX);
    assert_eq!(imu.stats().fifo_words, u32::MAX);
    shared.arm(Some(1), None);
    assert_eq!(run(imu.fifo_status()), Err(Error::Bus(BusFault)));
    assert_eq!(imu.stats().bus_errors, u32::MAX);
}

#[test]
fn info_is_none_until_configured_and_then_what_init_returned() {
    let (_shared, mut imu, mut delay) = rig();
    assert_eq!(imu.info(), None);
    let info = run(imu.init(&mut delay, &fifo_config())).unwrap();
    assert_eq!(imu.info(), Some(info));
    assert_eq!(info.freq_fine, -3);
    let info = run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_eq!(imu.info(), Some(info));
    assert_eq!(info.fifo_words_per_second, 0);
}

// ---------------------------------------------------------------------------------------------
// Errors before any bus access (D3)
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_read_errors_before_any_bus_access() {
    let mut buf = [0u8; 100];
    let mut small = [0u8; 6];
    // Not configured.
    let (shared, mut imu, _d) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_read(&mut buf)), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
    // FIFO off in the applied configuration.
    let (shared, mut imu, _d) = configured();
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_read(&mut buf)), Err(Error::FifoOff));
    assert_eq!(shared.transactions(), 0);
    // Buffer shorter than one word.
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    shared.push_words(3, 0);
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_read(&mut small)), Err(Error::BufferTooSmall));
    assert_eq!(run(imu.fifo_read(&mut [])), Err(Error::BufferTooSmall));
    assert_eq!(shared.transactions(), 0);
    assert_eq!(shared.fifo_words(), 3);
}

#[test]
fn fifo_read_exact_errors_before_any_bus_access() {
    let mut buf = [0u8; 20];
    let (shared, mut imu, _d) = rig();
    shared.arm(None, None);
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, 1)),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, _d) = configured();
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_read_exact(&mut buf, 1)), Err(Error::FifoOff));
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, _d) = rig_with(&fifo_config());
    shared.push_words(5, 0);
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_read_exact(&mut buf, 0)), Err(Error::ZeroWords));
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, 3)),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, usize::MAX)),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, usize::MAX / 7)),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(shared.transactions(), 0);
    assert_eq!(shared.fifo_words(), 5);
    // Two words fit in 20 bytes.
    assert_eq!(run(imu.fifo_read_exact(&mut buf, 2)).unwrap().words, 2);
}

#[test]
fn fifo_status_fifo_flush_and_interrupt_status_error_before_any_bus_access() {
    let (shared, mut imu, _d) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_status()), Err(Error::NotConfigured));
    assert_eq!(run(imu.fifo_flush()), Err(Error::NotConfigured));
    assert_eq!(run(imu.interrupt_status()), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, _d) = configured();
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_status()), Err(Error::FifoOff));
    assert_eq!(run(imu.fifo_flush()), Err(Error::FifoOff));
    assert_eq!(shared.transactions(), 0);
    // interrupt_status does not need the FIFO.
    assert!(run(imu.interrupt_status()).is_ok());
    assert_eq!(shared.transactions(), 1);
}

#[test]
fn any_overrun_is_either_flag() {
    assert!(!FifoFlags::default().any_overrun());
    let a = FifoFlags {
        overrun: true,
        ..FifoFlags::default()
    };
    let b = FifoFlags {
        overrun_latched: true,
        ..FifoFlags::default()
    };
    assert!(a.any_overrun() && b.any_overrun());
    assert_eq!(FifoStatus::default().words, 0);
}

// ---------------------------------------------------------------------------------------------
// Bus error and cancel at every point (D3, D4, A2)
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum M {
    Read,
    Exact,
    Status,
    Flush,
    IntPlain,
    IntEvents,
}

const METHODS: [M; 6] = [
    M::Read,
    M::Exact,
    M::Status,
    M::Flush,
    M::IntPlain,
    M::IntEvents,
];

#[derive(Debug, PartialEq, Eq)]
enum Out {
    Read(FifoRead, Vec<u8>),
    Status(FifoStatus),
    Done,
    Int(InterruptStatus),
}

fn setup(m: M) -> (Shared, Imu, TestDelay) {
    let c = match m {
        M::IntEvents => events_config(),
        _ => fifo_config(),
    };
    rig_with(&c)
}

/// The same device state before every attempt: 5 words, watermark flag off, nothing latched.
fn prepare(shared: &Shared) {
    shared.chip().fifo.clear();
    shared.push_words(5, 3);
    shared.set_fifo_flags(false, false, false);
    shared.chip().regs[reg::STATUS_REG as usize] = 0b0000_0011;
    shared.chip().regs[reg::HG_WAKE_UP_SRC as usize] = 0b0000_1000;
}

async fn call(imu: &mut Imu, m: M) -> Res<Out> {
    Ok(match m {
        M::Read => {
            let mut b = [0u8; 100];
            let f = imu.fifo_read(&mut b).await?;
            Out::Read(f, b[..f.bytes].to_vec())
        }
        M::Exact => {
            let mut b = [0u8; 21];
            let f = imu.fifo_read_exact(&mut b, 3).await?;
            Out::Read(f, b[..f.bytes].to_vec())
        }
        M::Status => Out::Status(imu.fifo_status().await?),
        M::Flush => {
            imu.fifo_flush().await?;
            Out::Done
        }
        M::IntPlain | M::IntEvents => Out::Int(imu.interrupt_status().await?),
    })
}

/// The device state a finished call leaves: for the flush, an empty FIFO and the applied image.
fn check_end_state(shared: &Shared, imu: &mut Imu, m: M) {
    if matches!(m, M::Flush) {
        assert_eq!(shared.fifo_words(), 0);
        let image = imu.config().map(Config::fifo_ctrl4);
        assert_eq!(Some(shared.chip().regs[reg::FIFO_CTRL4 as usize]), image);
        assert_eq!(run(imu.verify()), Ok(()));
    }
}

/// Words whose transaction was cancelled or failed BEFORE it started are still in the device: the
/// fake takes a word out only in a transaction that completes, so the same call again returns the
/// same words. A real device that fails in the middle of the burst has already shifted words out
/// and they are lost to the caller; the driver documents that on `fifo_read`, and cannot get them
/// back. Nothing in the driver state changes in any case.
#[test]
fn fifo_read_fifo_read_exact_fifo_status_fifo_flush_and_interrupt_status_bus_error_at_every_transaction()
 {
    for m in METHODS {
        let (shared, mut imu, _d) = setup(m);
        prepare(&shared);
        shared.arm(None, None);
        let reference = run(call(&mut imu, m)).unwrap();
        let n = shared.transactions();
        check_end_state(&shared, &mut imu, m);
        let expected = match m {
            M::Read | M::Flush => 2,
            M::Exact | M::Status | M::IntPlain => 1,
            M::IntEvents => 3,
        };
        assert_eq!(n, expected, "{m:?}");
        for i in 1..=n {
            prepare(&shared);
            let before = imu.stats().bus_errors;
            shared.arm(Some(i), None);
            assert_eq!(
                run(call(&mut imu, m)),
                Err(Error::Bus(BusFault)),
                "{m:?} transaction {i}"
            );
            assert_eq!(imu.stats().bus_errors, before + 1);
            assert!(imu.config().is_some());
            shared.arm(None, None);
            assert_eq!(
                run(call(&mut imu, m)).unwrap(),
                reference,
                "{m:?} after {i}"
            );
            check_end_state(&shared, &mut imu, m);
            assert_eq!(shared.violations(), 0);
        }
    }
}

#[test]
fn fifo_read_fifo_read_exact_fifo_status_fifo_flush_and_interrupt_status_cancel_at_every_await() {
    for m in METHODS {
        let (shared, mut imu, _d) = setup(m);
        prepare(&shared);
        shared.arm(None, None);
        let reference = run(call(&mut imu, m)).unwrap();
        let total = shared.awaits();
        let mut cancelled = 0;
        for k in 1..=100 {
            prepare(&shared);
            shared.arm(None, Some(k));
            match poll_then_drop(call(&mut imu, m)) {
                Polled::Done(res) => {
                    assert_eq!(res.unwrap(), reference, "{m:?}");
                    break;
                }
                Polled::Cancelled => cancelled += 1,
            }
            assert!(imu.config().is_some(), "{m:?} cancelled at await {k}");
            shared.arm(None, None);
            assert_eq!(
                run(call(&mut imu, m)).unwrap(),
                reference,
                "{m:?} cancelled at await {k}"
            );
            check_end_state(&shared, &mut imu, m);
            assert_eq!(shared.violations(), 0);
        }
        assert_eq!(cancelled, total, "{m:?}");
        assert_eq!(imu.stats().bus_errors, 0);
    }
}

// ---------------------------------------------------------------------------------------------
// Blocking front-end (A11) and Send (A5)
// ---------------------------------------------------------------------------------------------

#[test]
fn blocking_fifo_read_fifo_read_exact_fifo_status_fifo_flush_and_interrupt_status_match_async() {
    let c = events_config();
    let (sa, mut a, _da) = rig_with(&c);
    let sb = Shared::new();
    let mut b = blocking::Lsm6dsk320x::new(sb.bus());
    let mut db = sb.delay();
    b.init(&mut db, &c).unwrap();
    for s in [&sa, &sb] {
        prepare(s);
        s.set_fifo_flags(true, true, true);
        s.arm(None, None);
    }
    let (mut ba, mut bb) = ([0u8; 30], [0u8; 30]);
    assert_eq!(run(a.fifo_read(&mut ba)), b.fifo_read(&mut bb));
    assert_eq!(ba, bb);
    assert_eq!(run(a.fifo_status()), b.fifo_status());
    for s in [&sa, &sb] {
        s.push_words(4, 7);
    }
    let (mut ea, mut eb) = ([0u8; 21], [0u8; 21]);
    assert_eq!(
        run(a.fifo_read_exact(&mut ea, 3)),
        b.fifo_read_exact(&mut eb, 3)
    );
    assert_eq!(ea, eb);
    assert_eq!(run(a.interrupt_status()), b.interrupt_status());
    assert_eq!(run(a.fifo_flush()), b.fifo_flush());
    assert_eq!(run(a.fifo_read(&mut [0u8; 3])), b.fifo_read(&mut [0u8; 3]));
    assert_eq!(
        run(a.fifo_read_exact(&mut ea, 0)),
        b.fifo_read_exact(&mut eb, 0)
    );
    assert_eq!(sa.log(), sb.log());
    assert_eq!(sa.chip().regs, sb.chip().regs);
    assert_eq!(a.stats(), b.stats());
    assert_eq!(a.info(), b.info());
    assert!(a.stats().fifo_overruns > 0 && a.stats().fifo_words == 7);

    let (_s, mut a, _d) = rig();
    let s = Shared::new();
    let mut b = blocking::Lsm6dsk320x::new(s.bus());
    assert_eq!(run(a.fifo_status()), b.fifo_status());
    assert_eq!(b.fifo_flush(), Err(Error::NotConfigured));
    assert_eq!(b.info(), None);
}

fn assert_send<T: Send>(_: &T) {}

#[test]
fn fifo_read_and_the_other_new_futures_are_send() {
    let (_shared, mut imu, _d) = rig();
    let mut buf = [0u8; 14];
    assert_send(&imu.fifo_read(&mut buf));
    assert_send(&imu.fifo_status());
    assert_send(&imu.fifo_flush());
    assert_send(&imu.interrupt_status());
    let mut buf = [0u8; 14];
    assert_send(&imu.fifo_read_exact(&mut buf, 2));
    assert_send(&Stats::default());
}

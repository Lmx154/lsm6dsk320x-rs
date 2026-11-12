//! Host tests of the self-test: the sequences, the pure evaluation at the datasheet limits (DS15060
//! Rev 1, Table 3, p13, notes p14), and the driver method on the fake with its stimulus model.
//!
//! Expected values are numbers worked from the datasheet, not computed from the code under test.

use std::vec;
use std::vec::Vec;

use crate::calc::Offsets;
use crate::driver_tests::{Imu, Res, rig, w};
use crate::plan::{Apply, Step};
use crate::selftest::{MAX_STEPS, Phase, evaluate, mean};
use crate::testbus::{Event, Op, Polled, Shared, TestDelay, poll_then_drop, run};
use crate::{
    AccelFullScale, Channel, Config, Error, HighGConfig, OffsetWeight, SelfTestConfig,
    SelfTestReport, SelfTestStep, Stimulus, blocking, reg,
};

// Channel indices of the fake: low-g, gyroscope, high-g.
const CH: [Channel; 3] = [Channel::LowG, Channel::Gyro, Channel::HighG];
const REST: [[i16; 3]; 3] = [[10, -20, 8192], [30, -40, 5], [7, -9, 1000]];
const PASS_DELTA: [[i16; 3]; 3] = [[1000; 3], [5000; 3], [1000; 3]];
const FAIL_DELTA: [[i16; 3]; 3] = [[10; 3], [100; 3], [10; 3]];

fn only(c: usize) -> SelfTestConfig {
    SelfTestConfig::new()
        .with_low_g(c == 0)
        .with_gyro(c == 1)
        .with_high_g(c == 2)
}

fn applied_config() -> Config {
    Config {
        accel_fs: AccelFullScale::G4,
        high_g: Some(HighGConfig::new()),
        accel_offset: Some(OffsetWeight::Fine),
        ..Config::new()
    }
}

fn offsets() -> Offsets {
    Offsets::new()
        .with_low_g([1, -2, 3])
        .with_high_g([4, 5, -6])
}

/// A driver configured with a non-default configuration and offsets, and the stimulus model on.
fn rig_st(delta: [[i16; 3]; 3]) -> (Shared, Imu, TestDelay) {
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, &applied_config())).unwrap();
    run(imu.set_offsets(&offsets())).unwrap();
    shared.model(REST, delta);
    (shared, imu, delay)
}

fn st_bits(shared: &Shared) -> (u8, u8) {
    let c = shared.chip();
    (
        c.regs[reg::CTRL10 as usize] & 0x0F,
        c.regs[reg::CTRL2_XL_HG as usize] & 0x03,
    )
}

fn restored(shared: &Shared, imu: &mut Imu) {
    assert_eq!(st_bits(shared), (0, 0));
    assert_eq!(imu.config(), Some(&applied_config()));
    assert_eq!(imu.offsets(), Some(offsets()));
    assert_eq!(run(imu.verify()), Ok(()));
}

fn lsb_bytes(v: [i16; 3]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

// ---------------------------------------------------------------------------------------------
// The sequences
// ---------------------------------------------------------------------------------------------

fn bus(reg: u8, value: u8) -> SelfTestStep {
    SelfTestStep::Bus(Step::Write { reg, value })
}

fn delay_100ms() -> SelfTestStep {
    SelfTestStep::Bus(Step::DelayUs(100_000))
}

fn modify_ctrl10(value: u8) -> SelfTestStep {
    SelfTestStep::Bus(Step::Modify {
        reg: 0x19,
        mask: 0x0F,
        value,
    })
}

#[test]
fn sequences_low_g_gyro_high_g_as_step_lists() {
    let t = SelfTestConfig::new();
    // Low-g: others off, filters and offsets off (CTRL9), +-4 g (CTRL8 01, p71), 60 Hz (CTRL1 0101,
    // p65), ST_XL positive (CTRL10 bits 1:0 = 01, p74).
    let low = t.sequence(Channel::LowG).unwrap();
    assert_eq!(
        low.steps(),
        &[
            bus(0x4E, 0x00),
            bus(0x11, 0x00),
            bus(0x10, 0x00),
            bus(0x18, 0x00),
            bus(0x17, 0x01),
            bus(0x10, 0x05),
            delay_100ms(),
            SelfTestStep::Collect(Phase::Baseline),
            modify_ctrl10(0x01),
            delay_100ms(),
            SelfTestStep::Collect(Phase::Stimulated),
            modify_ctrl10(0x00),
        ]
    );
    // Gyroscope: LPF1 off and routing off (CTRL7), +-250 dps (CTRL6 FS_G 001 and bit 3, p69),
    // 240 Hz (CTRL2 0111, p66), ST_G positive (CTRL10 bits 3:2 = 01).
    let gyro = t.sequence(Channel::Gyro).unwrap();
    assert_eq!(
        gyro.steps(),
        &[
            bus(0x4E, 0x00),
            bus(0x10, 0x00),
            bus(0x11, 0x00),
            bus(0x16, 0x00),
            bus(0x15, 0x09),
            bus(0x11, 0x07),
            delay_100ms(),
            SelfTestStep::Collect(Phase::Baseline),
            modify_ctrl10(0x04),
            delay_100ms(),
            SelfTestStep::Collect(Phase::Stimulated),
            modify_ctrl10(0x00),
        ]
    );
    // High-g: REGOUT_EN, 480 Hz (ODR_XL_HG 011), +-32 g (FS 000), no offset (CTRL1_XL_HG 0x98,
    // p91); XL_HG_ST positive (CTRL2_XL_HG bits 1:0 = 01).
    let high = t.sequence(Channel::HighG).unwrap();
    assert_eq!(
        high.steps(),
        &[
            bus(0x4E, 0x00),
            bus(0x10, 0x00),
            bus(0x11, 0x00),
            bus(0x4E, 0x98),
            delay_100ms(),
            SelfTestStep::Collect(Phase::Baseline),
            bus(0x4D, 0x01),
            delay_100ms(),
            SelfTestStep::Collect(Phase::Stimulated),
            bus(0x4D, 0x00),
        ]
    );
    assert_eq!(
        (low.steps().len(), gyro.steps().len(), high.steps().len()),
        (12, 12, 10)
    );
    assert_eq!(MAX_STEPS, 12);
    assert_eq!(low.steps().last(), Some(&modify_ctrl10(0x00)));
    assert_eq!(gyro.steps().last(), Some(&modify_ctrl10(0x00)));
    assert_eq!(high.steps().last(), Some(&bus(0x4D, 0x00)));
}

#[test]
fn sequence_negative_stimulus_and_settle_time() {
    let t = SelfTestConfig::new()
        .with_stimulus(Stimulus::Negative)
        .with_settle_ms(7);
    let low = t.sequence(Channel::LowG).unwrap();
    assert_eq!(low.steps().get(8), Some(&modify_ctrl10(0x02)));
    assert_eq!(
        low.steps().get(6),
        Some(&SelfTestStep::Bus(Step::DelayUs(7000)))
    );
    assert_eq!(
        t.sequence(Channel::Gyro).unwrap().steps().get(8),
        Some(&modify_ctrl10(0x08))
    );
    assert_eq!(
        t.sequence(Channel::HighG).unwrap().steps().get(6),
        Some(&bus(0x4D, 0x02))
    );
}

#[test]
fn config_defaults_builders_and_validity() {
    let t = SelfTestConfig::default();
    assert_eq!(
        (t.low_g, t.gyro, t.high_g, t.settle_ms, t.samples, t.discard),
        (true, true, true, 100, 5, 1)
    );
    assert_eq!(t.stimulus, Stimulus::Positive);
    assert!(t.is_valid());
    assert!(!t.with_samples(0).is_valid());
    assert!(
        !t.with_low_g(false)
            .with_gyro(false)
            .with_high_g(false)
            .is_valid()
    );
    assert!(t.with_discard(0).with_settle_ms(0).is_valid());
    assert_eq!(
        t.with_gyro(false).channels(),
        [Some(Channel::LowG), None, Some(Channel::HighG)]
    );
}

// ---------------------------------------------------------------------------------------------
// The evaluation
// ---------------------------------------------------------------------------------------------

/// Pass flag of the X axis for a raw change of `d` counts from `base`, in both directions.
fn x_pass(channel: Channel, base: i16, d: i16) -> bool {
    let up = evaluate(channel, [base, 0, 0], [base + d, 0, 0]).axes[0].pass;
    let down = evaluate(channel, [base, 0, 0], [base - d, 0, 0]).axes[0].pass;
    assert_eq!(up, down);
    up
}

#[test]
fn evaluation_at_each_limit_and_one_lsb_outside() {
    // Low-g, 0.122 mg per LSB at +-4 g: 50 mg is 409.8 LSB, 1700 mg is 13934.4 LSB (p13, p12).
    assert!(!x_pass(Channel::LowG, 0, 409)); // 49 898 ug
    assert!(x_pass(Channel::LowG, 0, 410)); // 50 020 ug
    assert!(x_pass(Channel::LowG, 0, 13934)); // 1 699 948 ug
    assert!(!x_pass(Channel::LowG, 0, 13935)); // 1 700 070 ug
    // Gyroscope, 8.75 mdps per LSB at +-250 dps: 20 dps is 2285.7 LSB, 80 dps is 9142.9 LSB.
    assert!(!x_pass(Channel::Gyro, 0, 2285)); // 19 994 mdps
    assert!(x_pass(Channel::Gyro, 0, 2286)); // 20 003 mdps
    assert!(x_pass(Channel::Gyro, 0, 9142)); // 79 993 mdps
    assert!(!x_pass(Channel::Gyro, 0, 9143)); // 80 001 mdps
    // High-g, 0.976 mg per LSB at +-32 g: 150 mg is 153.7 LSB, 2500 mg is 2561.5 LSB.
    assert!(!x_pass(Channel::HighG, 0, 153)); // 149 328 ug
    assert!(x_pass(Channel::HighG, 0, 154)); // 150 304 ug
    assert!(x_pass(Channel::HighG, 0, 2561)); // 2 499 536 ug
    assert!(!x_pass(Channel::HighG, 0, 2562)); // 2 500 512 ug
    // A resting value does not move the window.
    assert!(x_pass(Channel::LowG, 8000, 410) && !x_pass(Channel::LowG, 8000, 409));
    assert!(x_pass(Channel::Gyro, -5000, 9142) && !x_pass(Channel::Gyro, -5000, 9143));
}

#[test]
fn evaluation_reports_units_and_each_axis_alone() {
    let r = evaluate(Channel::LowG, [10, -20, 8192], [1010, -20, 8192 - 1000]);
    assert_eq!(r.axes[0].baseline, 1220);
    assert_eq!(r.axes[0].stimulated, 123_220);
    assert_eq!(r.axes[0].difference, 122_000);
    assert!(r.axes[0].pass);
    assert_eq!(r.axes[1].baseline, -2440);
    assert_eq!(r.axes[1].difference, 0);
    assert!(!r.axes[1].pass);
    assert_eq!(r.axes[2].baseline, 999_424);
    assert_eq!(r.axes[2].difference, 122_000);
    assert!(!r.passed());
    // Gyroscope: 1000 LSB = 8750 mdps, -3 LSB = -26.25 -> -26 mdps.
    let g = evaluate(Channel::Gyro, [-3, 0, 0], [997, 0, 0]);
    assert_eq!(
        (
            g.axes[0].baseline,
            g.axes[0].stimulated,
            g.axes[0].difference
        ),
        (-26, 8724, 8750)
    );
}

#[test]
fn mean_rounds_to_nearest_halves_away_from_zero() {
    assert_eq!(mean(10, 5), 2);
    assert_eq!(mean(7, 5), 1);
    assert_eq!(mean(-7, 5), -1);
    assert_eq!(mean(5, 2), 3);
    assert_eq!(mean(-5, 2), -3);
    assert_eq!(mean(i32::from(i16::MAX) * 255, 255), i16::MAX);
    assert_eq!(mean(i32::from(i16::MIN) * 255, 255), i16::MIN);
}

#[test]
fn report_passed_needs_a_channel_and_all_axes() {
    assert!(!SelfTestReport::default().passed());
    let ok = evaluate(Channel::LowG, [0; 3], [1000; 3]);
    let bad = evaluate(Channel::LowG, [0; 3], [1000, 1000, 1]);
    let mut r = SelfTestReport {
        low_g: Some(ok),
        ..SelfTestReport::default()
    };
    assert!(r.passed());
    r.gyro = Some(bad);
    assert!(!r.passed());
    assert_eq!(r.channel(Channel::Gyro), Some(&bad));
    assert_eq!(r.channel(Channel::HighG), None);
}

// ---------------------------------------------------------------------------------------------
// The plan clears the stimulus bits
// ---------------------------------------------------------------------------------------------

#[test]
fn configure_clears_st_xl_st_g_and_xl_hg_st_and_keeps_the_other_ctrl10_bits() {
    // The plans clear ST_XL and ST_G (p74) with a read-modify-write after CTRL3, EMB_FUNC_DEBUG
    // (bit 6) stays; the in-place plan also writes CTRL2_XL_HG.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().regs[reg::CTRL10 as usize] = 0x4F;
    shared.chip().regs[reg::CTRL2_XL_HG as usize] = 0x03;
    shared.arm(None, None);
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_eq!(shared.chip().regs[reg::CTRL10 as usize], 0x40);
    assert_eq!(shared.chip().regs[reg::CTRL2_XL_HG as usize], 0x00);
    let log = shared.log();
    assert_eq!(log.get(1), Some(&w(0x12, 0x44)));
    assert_eq!(log.get(2), Some(&rd(0x19, 0x4F)));
    assert_eq!(log.get(3), Some(&w(0x19, 0x40)));
}

#[test]
fn a_stimulus_left_on_is_reported_by_verify_and_cleared_by_configure() {
    // (register, bit set behind the driver's back, mask the plan owns): low-g, gyroscope, high-g.
    for (address, bits, mask) in [
        (reg::CTRL10, 0x01u8, 0x0Fu8),
        (reg::CTRL10, 0x04, 0x0F),
        (reg::CTRL10, 0x08, 0x0F),
        (reg::CTRL2_XL_HG, 0x01, 0x13),
    ] {
        let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
        shared.chip().regs[address as usize] |= bits;
        let found = shared.chip().regs[address as usize];
        shared.arm(None, None);
        assert_eq!(
            run(imu.verify()),
            Err(Error::ConfigMismatch(crate::Mismatch {
                register: address,
                mask,
                expected: 0,
                found,
            })),
            "{address:#04x} {bits:#04x}"
        );
        run(imu.configure(&mut delay, &applied_config())).unwrap();
        assert_eq!(st_bits(&shared), (0, 0));
        restored(&shared, &mut imu);
    }
}

// ---------------------------------------------------------------------------------------------
// The driver on the fake
// ---------------------------------------------------------------------------------------------

fn status(v: u8) -> Event {
    Event::Spi(vec![Op::Write(vec![0x9E]), Op::Read(vec![v])])
}

fn out(address: u8, v: [i16; 3]) -> Event {
    Event::Spi(vec![
        Op::Write(vec![address | 0x80]),
        Op::Read(lsb_bytes(v)),
    ])
}

fn rd(reg: u8, got: u8) -> Event {
    Event::Spi(vec![Op::Write(vec![reg | 0x80]), Op::Read(vec![got])])
}

#[test]
fn golden_trace_of_the_low_g_test() {
    let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
    shared.arm(None, None);
    let report = run(imu.self_test(&mut delay, &only(0))).unwrap();
    let stim = [1010, 980, 9192];
    let mut expected = vec![
        w(0x4E, 0),
        w(0x11, 0),
        w(0x10, 0),
        w(0x18, 0),
        w(0x17, 0x01),
        w(0x10, 0x05),
        Event::Delay(100_000_000),
    ];
    // 1 discarded and 5 averaged samples, each after one STATUS_REG poll (XLDA, GDA, XLHGDA set).
    for _ in 0..6 {
        expected.push(status(0x0B));
        expected.push(out(0xA8 & 0x7F, REST[0]));
    }
    expected.extend([rd(0x19, 0), w(0x19, 0x01), Event::Delay(100_000_000)]);
    for _ in 0..6 {
        expected.push(status(0x0B));
        expected.push(out(0x28, stim));
    }
    expected.extend([rd(0x19, 0x01), w(0x19, 0x00)]);
    let log = shared.log();
    assert_eq!(log.get(..expected.len()), Some(&expected[..]));
    // Then the in-place plan of the applied configuration and the two offset bursts.
    let plan = applied_config().plan(Apply::InPlace).unwrap();
    let plan_txn: usize = plan
        .steps()
        .iter()
        .map(|s| match s {
            Step::Write { .. } => 1,
            Step::Modify { .. } => 2,
            _ => 0,
        })
        .sum();
    assert_eq!(shared.transactions(), 34 + plan_txn + 2);
    assert_eq!(
        log.last(),
        Some(&Event::Spi(vec![Op::Write(vec![0x73, 1, 0xFE, 3])]))
    );
    let axes = report.low_g.unwrap().axes;
    assert_eq!(axes[0].difference, 122_000);
    assert!(report.passed() && report.gyro.is_none() && report.high_g.is_none());
    restored(&shared, &mut imu);
}

#[test]
fn transactions_per_channel_with_the_defaults() {
    for (c, n) in [(0usize, 34usize), (1, 34), (2, 30)] {
        let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
        shared.arm(None, None);
        run(imu.self_test(&mut delay, &only(c))).unwrap();
        let plan = applied_config().plan(Apply::InPlace).unwrap();
        let plan_txn: usize = plan
            .steps()
            .iter()
            .map(|s| {
                if matches!(s, Step::Modify { .. }) {
                    2
                } else {
                    usize::from(!matches!(s, Step::DelayUs(_)))
                }
            })
            .sum();
        assert_eq!(shared.transactions(), n + plan_txn + 2, "channel {c}");
        assert_eq!(shared.violations(), 0);
    }
}

#[test]
fn a_pass_and_a_fail_per_channel_through_the_driver() {
    for (c, &channel) in CH.iter().enumerate() {
        let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
        let r = run(imu.self_test(&mut delay, &only(c))).unwrap();
        assert!(r.passed(), "pass {c}");
        let res = r.channel(channel).unwrap();
        assert!(res.axes.iter().all(|a| a.pass));
        restored(&shared, &mut imu);

        let (shared, mut imu, mut delay) = rig_st(FAIL_DELTA);
        let r = run(imu.self_test(&mut delay, &only(c))).unwrap();
        assert!(!r.passed(), "fail {c}");
        assert!(r.channel(channel).unwrap().axes.iter().all(|a| !a.pass));
        restored(&shared, &mut imu);
    }
}

#[test]
fn all_three_channels_and_the_negative_stimulus() {
    let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
    let t = SelfTestConfig::new().with_stimulus(Stimulus::Negative);
    let r = run(imu.self_test(&mut delay, &t)).unwrap();
    assert!(r.passed());
    let lo = r.low_g.unwrap().axes[0];
    assert_eq!(
        (lo.baseline, lo.stimulated, lo.difference),
        (1220, -120_780, 122_000)
    );
    let hi = r.high_g.unwrap().axes[2];
    assert_eq!(
        (hi.baseline, hi.stimulated, hi.difference),
        (976_000, 0, 976_000)
    );
    let g = r.gyro.unwrap().axes[0];
    assert_eq!(g.difference, 43_750);
    restored(&shared, &mut imu);
}

#[test]
fn data_ready_timeout_names_the_channel_and_configure_recovers() {
    for (c, &channel) in CH.iter().enumerate() {
        let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
        shared.hold_ready(c, true);
        shared.arm(None, None);
        let e = run(imu.self_test(&mut delay, &only(c)));
        assert_eq!(e, Err(Error::DataReadyTimeout { channel }));
        assert_eq!(imu.config(), None);
        assert_eq!(imu.offsets(), None);
        assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        let waits = shared
            .log()
            .iter()
            .filter(|e| **e == Event::Delay(500_000))
            .count();
        assert_eq!(waits, 399);
        assert_eq!(st_bits(&shared), (0, 0));
        shared.hold_ready(c, false);
        run(imu.configure(&mut delay, &applied_config())).unwrap();
        restored(&shared, &mut imu);
    }
}

#[test]
fn not_configured_and_invalid_use_no_bus_and_keep_the_state() {
    let (shared, mut imu, mut delay) = rig();
    assert_eq!(
        run(imu.self_test(&mut delay, &SelfTestConfig::new())),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
    shared.arm(None, None);
    for bad in [
        SelfTestConfig::new().with_samples(0),
        only(0).with_low_g(false),
    ] {
        assert_eq!(
            run(imu.self_test(&mut delay, &bad)),
            Err(Error::InvalidSelfTest)
        );
    }
    assert_eq!(shared.transactions(), 0);
    restored(&shared, &mut imu);
}

#[test]
fn the_validity_state_is_cleared_before_the_first_await() {
    let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
    shared.arm(None, Some(1));
    let t = SelfTestConfig::new();
    {
        let mut f = core::pin::pin!(imu.self_test(&mut delay, &t));
        let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
        let _ = core::future::Future::poll(f.as_mut(), &mut cx);
    }
    assert_eq!(imu.config(), None);
}

#[test]
fn bus_error_at_every_transaction_leaves_not_configured_stimulus_off_and_configure_recovers() {
    let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
    shared.arm(None, None);
    run(imu.self_test(&mut delay, &SelfTestConfig::new())).unwrap();
    let total = shared.transactions();
    assert!(total > 100);
    for k in 1..=total {
        let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
        shared.arm(Some(k), None);
        let e = run(imu.self_test(&mut delay, &SelfTestConfig::new()));
        assert!(matches!(e, Err(Error::Bus(_))), "k {k}: {e:?}");
        assert_eq!(imu.config(), None, "k {k}");
        // The best-effort switch-off reaches the chip, which fails once only.
        assert_eq!(st_bits(&shared), (0, 0), "k {k}");
        shared.arm(None, None);
        run(imu.configure(&mut delay, &applied_config())).unwrap();
        restored(&shared, &mut imu);
    }
}

#[test]
fn bus_error_during_a_test_switches_the_stimulus_off() {
    // Fail the first STATUS_REG poll of the stimulated phase of the low-g test: transaction 7
    // (six writes), 12 for the first phase, 2 for the read-modify-write, then the poll.
    let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
    shared.arm(Some(6 + 12 + 2 + 1), None);
    let e = run(imu.self_test(&mut delay, &only(0)));
    assert!(matches!(e, Err(Error::Bus(_))));
    assert_eq!(st_bits(&shared), (0, 0));
    assert!(shared.log().contains(&w(0x19, 0x01)));
}

#[test]
fn cancel_at_every_await_leaves_not_configured_and_configure_recovers() {
    let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
    shared.arm(None, None);
    run(imu.self_test(&mut delay, &SelfTestConfig::new())).unwrap();
    let awaits = shared.awaits();
    assert!(awaits > 100);
    let mut left_on = 0;
    for k in 1..=awaits {
        let (shared, mut imu, mut delay) = rig_st(PASS_DELTA);
        shared.arm(None, Some(k));
        let t = SelfTestConfig::new();
        let polled = poll_then_drop(imu.self_test(&mut delay, &t));
        assert_eq!(polled, Polled::Cancelled, "await {k}");
        assert_eq!(imu.config(), None, "await {k}");
        assert_eq!(imu.offsets(), None);
        if st_bits(&shared) != (0, 0) {
            left_on += 1;
        }
        shared.arm(None, None);
        run(imu.configure(&mut delay, &applied_config())).unwrap();
        restored(&shared, &mut imu);
    }
    // Negative control: a cancel in the stimulated phase does leave the stimulus on.
    assert!(left_on > 10, "left_on {left_on}");
}

#[test]
fn blocking_self_test_matches_the_async_driver() {
    let (a_shared, mut a, mut a_delay) = rig_st(PASS_DELTA);
    let b_shared = Shared::new();
    let mut b = blocking::Lsm6dsk320x::new(b_shared.bus());
    let mut b_delay = b_shared.delay();
    b.init(&mut b_delay, &applied_config()).unwrap();
    b.set_offsets(&offsets()).unwrap();
    b_shared.model(REST, PASS_DELTA);
    a_shared.arm(None, None);
    b_shared.arm(None, None);
    let t = SelfTestConfig::new();
    let ra: Res<SelfTestReport> = run(a.self_test(&mut a_delay, &t));
    let rb = b.self_test(&mut b_delay, &t);
    assert_eq!(ra, rb);
    assert_eq!(a_shared.log(), b_shared.log());
    assert_eq!(a_shared.chip().regs, b_shared.chip().regs);
    assert_eq!(b.config(), Some(&applied_config()));
    assert_eq!(b.verify(), Ok(()));
    assert_eq!(
        b.self_test(&mut b_delay, &SelfTestConfig::new().with_samples(0)),
        Err(Error::InvalidSelfTest)
    );
}

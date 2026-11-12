//! Host tests of the driver methods on the fake bus of [`crate::testbus`]: golden traces,
//! transaction counts, the validity state, bus errors and cancels at every point, the blocking
//! front-end against the async one, and `Send`.

use std::vec;
use std::vec::Vec;

use proptest::prelude::{ProptestConfig, prop_assert_eq, proptest};

use crate::calc::Offsets;
use crate::image::OWNED_COUNT;
use crate::plan::{Apply, Step};
use crate::testbus::{
    BusFault, Event, Op, Polled, Shared, TestBus, TestDelay, poll_then_drop, run,
};
use crate::tests::valid_config_s;
use crate::{
    Channel, Config, ConfigError, Error, Info, Lsm6dsk320x, Mismatch, Odr, RawAll, RawAxes,
    RawSample, Status, blocking, reg,
};

pub(crate) type Imu = Lsm6dsk320x<TestBus>;
pub(crate) type Res<T> = Result<T, Error<BusFault>>;

pub(crate) fn rig() -> (Shared, Imu, TestDelay) {
    let shared = Shared::new();
    let imu = Lsm6dsk320x::new(shared.bus());
    let delay = shared.delay();
    (shared, imu, delay)
}

pub(crate) fn configured() -> (Shared, Imu, TestDelay) {
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, &Config::new())).unwrap();
    (shared, imu, delay)
}

pub(crate) fn w(reg: u8, value: u8) -> Event {
    Event::Spi(vec![Op::Write(vec![reg, value])])
}

fn r(reg: u8, got: u8) -> Event {
    Event::Spi(vec![Op::Write(vec![reg | 0x80]), Op::Read(vec![got])])
}

/// Transactions of a plan run: a write is one, a read-modify-write is two.
fn plan_transactions(config: &Config, apply: Apply) -> usize {
    config
        .plan(apply)
        .unwrap()
        .steps()
        .iter()
        .map(|s| match s {
            Step::Write { .. } => 1,
            Step::Modify { .. } => 2,
            _ => 0,
        })
        .sum()
}

fn plan_delays(config: &Config, apply: Apply) -> usize {
    config
        .plan(apply)
        .unwrap()
        .steps()
        .iter()
        .filter(|s| matches!(s, Step::DelayUs(_)))
        .count()
}

#[derive(Clone, Copy, Debug)]
enum Method {
    Configure,
    Read,
    ReadHighG,
    ReadAll,
    Status,
}

const ALL: [Method; 5] = [
    Method::Configure,
    Method::Read,
    Method::ReadHighG,
    Method::ReadAll,
    Method::Status,
];

#[derive(Debug, PartialEq, Eq)]
enum Out {
    Info(Info),
    Sample(RawSample),
    High(RawAxes),
    All(RawAll),
    Status(Status),
}

async fn call(imu: &mut Imu, delay: &mut TestDelay, m: Method) -> Res<Out> {
    Ok(match m {
        Method::Configure => Out::Info(imu.configure(delay, &Config::new()).await?),
        Method::Read => Out::Sample(imu.read().await?),
        Method::ReadHighG => Out::High(imu.read_high_g().await?),
        Method::ReadAll => Out::All(imu.read_all().await?),
        Method::Status => Out::Status(imu.status().await?),
    })
}

fn reg_bytes<const N: usize>(shared: &Shared, from: u8) -> [u8; N] {
    let c = shared.chip();
    let mut out = [0u8; N];
    out.copy_from_slice(&c.regs[from as usize..from as usize + N]);
    out
}

fn expected_read(shared: &Shared) -> RawSample {
    RawSample::from_bytes(&reg_bytes::<12>(shared, reg::OUTX_L_G))
}

// ---------------------------------------------------------------------------------------------
// Golden traces (D2) and transaction counts (E1)
// ---------------------------------------------------------------------------------------------

#[test]
fn init_default_golden_trace() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    let info = run(imu.init(&mut delay, &Config::new())).unwrap();
    // DS15060 Rev 1, printed pages. Address byte: bit 7 set reads (reg::SPI_READ).
    let expected: Vec<Event> = vec![
        // CTRL3 (0x12, p67): BDU | IF_INC | SW_RESET = 0x45.
        w(0x12, 0x45),
        // 100 us, then CTRL3 read; SW_RESET has cleared (p67).
        Event::Delay(100_000),
        r(0x12, 0x44),
        // WHO_AM_I (0x0F, p64) = 0x75.
        r(0x0F, 0x75),
        // CTRL3 (0x12, p67): BDU | IF_INC = 0x44, the first step of the plan.
        w(0x12, 0x44),
        // CTRL10 (0x19, p74): ST_XL and ST_G to 0 under the mask 0x0F, a read-modify-write.
        r(0x19, 0),
        w(0x19, 0),
        // IF_CFG (0x03, p58): SPI 4-wire, auto-increment.
        w(0x03, 0x01),
        // PIN_CTRL (0x02, p57): reset value 0x23, written because a reset does not restore it.
        w(0x02, 0x23),
        // CTRL4 (0x13, p68): DRDY_MASK on.
        w(0x13, 0x0A),
        // INT1_CTRL (0x0D, p63), INT2_CTRL (0x0E, p64): nothing routed.
        w(0x0D, 0x01),
        w(0x0E, 0x00),
        // CTRL7 (0x16, p70), CTRL8 (0x17, p71), CTRL6 (0x15, p69).
        w(0x16, 0x00),
        w(0x17, 0x02),
        w(0x15, 0x0C),
        // CTRL1_XL_HG (0x4E, p91): high-g off.
        w(0x4E, 0x00),
        // CTRL1 (0x10, p65): low-g 960 Hz high-performance; CTRL2 (0x11, p66): gyro 960 Hz.
        w(0x10, 0x09),
        w(0x11, 0x09),
        // Gyroscope turn-on, 70 ms (GYRO_TURN_ON_US; Table 4, p15 gives 40 ms typical).
        Event::Delay(70_000_000),
        // The user offsets cleared: XL_HG_{X,Y,Z}_OFS_USR (0x6C to 0x6E, p108) and
        // {X,Y,Z}_OFS_USR (0x73 to 0x75, p113), one burst each.
        Event::Spi(vec![Op::Write(vec![0x6C, 0, 0, 0])]),
        Event::Spi(vec![Op::Write(vec![0x73, 0, 0, 0])]),
        // INTERNAL_FREQ_FINE (0x4F, p92).
        r(0x4F, 0xFD),
    ];
    assert_eq!(shared.log(), expected);
    assert_eq!(shared.violations(), 0);
    assert_eq!(shared.transactions(), 20);
    assert_eq!(shared.awaits(), 22);
    assert_eq!(info.who_am_i, 0x75);
    assert_eq!(info.freq_fine, -3);
    // 960 Hz * (1 + 0.0013 * -3) (p92).
    assert_eq!(info.gyro_odr_mhz, 956_256);
    assert_eq!(info.accel_odr_mhz, 956_256);
    assert_eq!(info.high_g_odr_mhz, 0);
    assert_eq!(info.fifo_words_per_second, 0);
    assert_eq!(imu.config(), Some(&Config::new()));
}

/// Table 22 (p34): in the high-accuracy ODR mode `Info` reports the table rate of the set, with no
/// trim (the chip's INTERNAL_FREQ_FINE is -3 here, which would give 956 256 at the 960 setting).
#[test]
fn init_reports_the_table_rate_without_trim_in_haodr_mode() {
    for (set, mhz) in [
        (crate::HaodrSet::Base960, 960_000),
        (crate::HaodrSet::Base1000, 1_000_000),
        (crate::HaodrSet::Base800, 800_000),
        (crate::HaodrSet::Base833, 833_000),
    ] {
        let (_shared, mut imu, mut delay) = rig();
        let config = Config::new().with_mode(crate::OperatingMode::HighAccuracyOdr(set));
        let info = run(imu.init(&mut delay, &config)).unwrap();
        assert_eq!(info.freq_fine, -3);
        assert_eq!(info.gyro_odr_mhz, mhz);
        assert_eq!(info.accel_odr_mhz, mhz);
    }
}

/// Measured on the MARV V2 board, 2026-11-11 (`hil/logs/lsm6dsk320x_dual-20261111-184219.log`,
/// section D3): in the high-accuracy ODR mode the high-g data-ready rate follows the ODR set like
/// the low-g one (INT2 / INT1 = high-g setting / 960 in every set). `Info` reports the table rate
/// of the set (Table 22, p34), within 0.5 % of each measured rate.
#[test]
fn init_reports_the_high_g_rate_of_the_haodr_set() {
    // (set, measured Hz for the high-g settings 480, 960, 1920, 3840, 7680)
    let measured = [
        (
            crate::HaodrSet::Base960,
            [479.0, 957.5, 1915.0, 3830.0, 7660.0],
        ),
        (
            crate::HaodrSet::Base1000,
            [500.0, 999.5, 1999.0, 3999.0, 7999.0],
        ),
        (
            crate::HaodrSet::Base800,
            [400.0, 800.5, 1602.0, 3203.0, 6406.0],
        ),
        (
            crate::HaodrSet::Base833,
            [417.0, 834.0, 1668.0, 3335.0, 6671.0],
        ),
    ];
    let rates = [
        crate::HighGOdr::Hz480,
        crate::HighGOdr::Hz960,
        crate::HighGOdr::Hz1920,
        crate::HighGOdr::Hz3840,
        crate::HighGOdr::Hz7680,
    ];
    for (set, hz) in measured {
        for (odr, want) in rates.into_iter().zip(hz) {
            let (_shared, mut imu, mut delay) = rig();
            let config = Config::new()
                .with_mode(crate::OperatingMode::HighAccuracyOdr(set))
                .with_high_g(Some(crate::HighGConfig::new().with_odr(odr)));
            let info = run(imu.init(&mut delay, &config)).unwrap();
            let got = info.high_g_odr_mhz as f32 / 1000.0;
            assert!(
                (got / want - 1.0).abs() <= 0.005,
                "{set:?} {odr:?}: reported {got} Hz, measured {want} Hz"
            );
        }
    }
    let (_shared, mut imu, mut delay) = rig();
    let config = Config::new()
        .with_mode(crate::OperatingMode::HighAccuracyOdr(
            crate::HaodrSet::Base833,
        ))
        .with_high_g(Some(crate::HighGConfig::new()));
    let info = run(imu.init(&mut delay, &config)).unwrap();
    assert_eq!(info.high_g_odr_mhz, 833_000);
}

/// With the low-g accelerometer off, only the gyroscope has a rate; the off sensor reports 0.
#[test]
fn init_reports_zero_for_an_off_sensor_in_haodr_mode() {
    let (_shared, mut imu, mut delay) = rig();
    let config = Config::new()
        .with_mode(crate::OperatingMode::HighAccuracyOdr(
            crate::HaodrSet::Base1000,
        ))
        .with_accel_odr(Odr::Off);
    let info = run(imu.init(&mut delay, &config)).unwrap();
    assert_eq!(info.accel_odr_mhz, 0);
    assert_eq!(info.gyro_odr_mhz, 1_000_000);
}

/// The one transaction of a read: `[Write(address | 0x80), Read(n bytes)]`, nothing else.
fn assert_read_shape(shared: &Shared, address: u8, n: usize) {
    let log = shared.log();
    assert_eq!(log.len(), 1);
    let Event::Spi(ops) = &log[0] else { panic!() };
    assert_eq!(ops.len(), 2);
    assert_eq!(ops[0], Op::Write(vec![address | 0x80]));
    let Op::Read(got) = &ops[1] else { panic!() };
    assert_eq!(got.len(), n);
    assert_eq!(shared.transactions(), 1);
    assert_eq!(shared.awaits(), 1);
    assert_eq!(shared.violations(), 0);
}

#[test]
fn read_is_one_transaction_write_one_byte_then_read_12_bytes() {
    let (shared, mut imu, _d) = configured();
    shared.arm(None, None);
    let sample = run(imu.read()).unwrap();
    assert_eq!(sample, expected_read(&shared));
    // OUTX_L_G (0x22, p73) | read, then the 12 output bytes, in the same transaction (A4a).
    assert_read_shape(&shared, 0x22, 12);
    let Event::Spi(ops) = &shared.log()[0] else {
        panic!()
    };
    assert_eq!(ops[0], Op::Write(vec![0xA2]));
}

#[test]
fn read_high_g_read_all_and_status_are_one_write_then_read_transaction_each() {
    let (shared, mut imu, _d) = configured();
    shared.arm(None, None);
    let high = run(imu.read_high_g()).unwrap();
    assert_eq!(
        high,
        RawAxes::from_bytes(&reg_bytes::<6>(&shared, reg::UI_OUTX_L_A_OIS_HG))
    );
    assert_read_shape(&shared, reg::UI_OUTX_L_A_OIS_HG, 6);
    shared.arm(None, None);
    let all = run(imu.read_all()).unwrap();
    assert_eq!(
        all,
        RawAll::from_bytes(&reg_bytes::<{ RawAll::LEN }>(&shared, reg::OUT_TEMP_L))
    );
    assert_read_shape(&shared, reg::OUT_TEMP_L, RawAll::LEN);
    shared.arm(None, None);
    let status = run(imu.status()).unwrap();
    assert_eq!(
        status,
        Status::from_bits(shared.chip().regs[reg::STATUS_REG as usize])
    );
    assert_eq!(
        shared.log(),
        vec![r(
            reg::STATUS_REG,
            shared.chip().regs[reg::STATUS_REG as usize]
        )]
    );
    assert_read_shape(&shared, reg::STATUS_REG, 1);
}

#[test]
fn testbus_counts_a_read_done_by_an_equal_length_transfer_as_a_violation() {
    use embedded_hal_async::spi::{Operation, SpiDevice};
    let shared = Shared::new();
    let mut bus = shared.bus();
    shared.arm(None, None);
    let mut b = [0xA2u8, 0, 0];
    run(bus.transaction(&mut [Operation::TransferInPlace(&mut b)])).unwrap();
    assert_eq!(shared.violations(), 1);
    let (mut rd, wr) = ([0u8; 3], [0xA2u8, 0, 0]);
    run(bus.transaction(&mut [Operation::Transfer(&mut rd, &wr)])).unwrap();
    assert_eq!(shared.violations(), 2);
    let mut w = [0x12u8, 0x44];
    run(bus.transaction(&mut [Operation::TransferInPlace(&mut w)])).unwrap();
    assert_eq!(shared.violations(), 2);
}

#[test]
fn configure_default_golden_prefix_and_counts() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    let info = run(imu.configure(&mut delay, &Config::new())).unwrap();
    let log = shared.log();
    // WHO_AM_I first (p64), then the plan: CTRL3 = BDU | IF_INC (p67), ST_XL and ST_G cleared in
    // CTRL10 by a read-modify-write (p74), the power-down block: FIFO_CTRL4 bypass (p61, 6.5.2
    // p45), CTRL1, CTRL2 (p65, p66) and CTRL1_XL_HG (p91) to 0; no reset (no CTRL3 write with
    // SW_RESET). At the end the two offset bursts (stored value, zero here) and the clock trim.
    assert_eq!(
        log[..8],
        [
            r(0x0F, 0x75),
            w(0x12, 0x44),
            r(0x19, 0),
            w(0x19, 0),
            w(0x0A, 0),
            w(0x10, 0),
            w(0x11, 0),
            w(0x4E, 0)
        ]
    );
    assert_eq!(
        log[log.len() - 3..],
        [
            Event::Spi(vec![Op::Write(vec![0x6C, 0, 0, 0])]),
            Event::Spi(vec![Op::Write(vec![0x73, 0, 0, 0])]),
            r(0x4F, 0xFD)
        ]
    );
    assert!(!log.contains(&w(0x12, 0x45)));
    let txns = plan_transactions(&Config::new(), Apply::InPlace) + 4;
    assert_eq!(shared.transactions(), txns);
    assert_eq!(
        shared.awaits() as usize,
        txns + plan_delays(&Config::new(), Apply::InPlace)
    );
    assert_eq!(info.freq_fine, -3);
    assert_eq!(imu.config(), Some(&Config::new()));
    assert_eq!(shared.violations(), 0);
}

// ---------------------------------------------------------------------------------------------
// Errors (D3) and the validity state (A3)
// ---------------------------------------------------------------------------------------------

#[test]
fn read_before_init_is_not_configured() {
    let (shared, mut imu, _d) = rig();
    shared.arm(None, None);
    assert_eq!(imu.config(), None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    assert_eq!(run(imu.read_high_g()), Err(Error::NotConfigured));
    assert_eq!(run(imu.read_all()), Err(Error::NotConfigured));
    assert_eq!(run(imu.status()), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
    // Raw access is outside the state.
    assert_eq!(run(imu.read_register(reg::WHO_AM_I)), Ok(0x75));
}

#[test]
fn init_error_variants() {
    let (shared, mut imu, mut delay) = rig();
    shared.chip().regs[reg::WHO_AM_I as usize] = 0x00;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::WrongId { found: 0 })
    );
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));

    let (shared, mut imu, mut delay) = rig();
    shared.chip().reset_stuck = true;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::ResetTimeout)
    );
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));

    let (shared, mut imu, mut delay) = configured();
    let bad = Config::new()
        .with_mode(crate::OperatingMode::HighAccuracyOdr(
            crate::HaodrSet::Base960,
        ))
        .with_accel_odr(Odr::Hz7_5);
    shared.arm(None, None);
    let e = run(imu.init(&mut delay, &bad));
    assert!(matches!(
        e,
        Err(Error::InvalidConfig(ConfigError::HaodrOdrReserved { .. }))
    ));
    assert_eq!(shared.transactions(), 0);
    assert_eq!(imu.config(), None);
    assert!(matches!(
        run(imu.configure(&mut delay, &bad)),
        Err(Error::InvalidConfig(_))
    ));
    assert_eq!(shared.transactions(), 0);
}

#[test]
fn init_bus_error_at_every_transaction_leaves_not_configured() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    run(imu.init(&mut delay, &Config::new())).unwrap();
    let n = shared.transactions();
    assert_eq!(n, 20);
    for i in 1..=n {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(Some(i), None);
        assert_eq!(
            run(imu.init(&mut delay, &Config::new())),
            Err(Error::Bus(BusFault)),
            "transaction {i}"
        );
        assert_eq!(imu.config(), None);
        shared.arm(None, None);
        assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        assert_eq!(shared.transactions(), 0);
        run(imu.init(&mut delay, &Config::new())).unwrap();
        assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
    }
}

#[test]
fn init_cancel_at_every_await_leaves_not_configured() {
    let mut cancelled = 0;
    for k in 1..=1000 {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(None, Some(k));
        match poll_then_drop(imu.init(&mut delay, &Config::new())) {
            Polled::Done(res) => {
                res.unwrap();
                assert_eq!(shared.awaits(), 22);
                assert_eq!(cancelled, 22);
                return;
            }
            Polled::Cancelled => cancelled += 1,
        }
        assert_eq!(imu.config(), None, "await {k}");
        shared.arm(None, None);
        assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        run(imu.init(&mut delay, &Config::new())).unwrap();
        assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
    }
    panic!("the sweep did not reach the end of init");
}

#[test]
fn configure_bus_error_and_cancel_at_every_point_leave_not_configured() {
    let n = plan_transactions(&Config::new(), Apply::InPlace) + 4;
    for i in 1..=n {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(Some(i), None);
        assert_eq!(
            run(imu.configure(&mut delay, &Config::new())),
            Err(Error::Bus(BusFault)),
            "transaction {i}"
        );
        assert_eq!(imu.config(), None);
        assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
    }
    let mut cancelled = 0;
    for k in 1..=1000 {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(None, Some(k));
        match poll_then_drop(imu.configure(&mut delay, &Config::new())) {
            Polled::Done(res) => {
                res.unwrap();
                assert_eq!(cancelled, shared.awaits());
                return;
            }
            Polled::Cancelled => cancelled += 1,
        }
        assert_eq!(imu.config(), None, "await {k}");
        shared.arm(None, None);
        assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
    }
    panic!("the sweep did not reach the end of configure");
}

#[test]
fn configure_in_place_with_other_settings_ends_with_the_new_config() {
    let (shared, mut imu, mut delay) = configured();
    let c = Config::new()
        .with_gyro_odr(Odr::Off)
        .with_accel_odr(Odr::Hz240);
    shared.arm(None, None);
    let info = run(imu.configure(&mut delay, &c)).unwrap();
    assert_eq!(imu.config(), Some(&c));
    assert_eq!(info.gyro_odr_mhz, 0);
    assert!(
        !shared
            .log()
            .iter()
            .any(|e| matches!(e, Event::Delay(70_000_000)))
    );
}

#[test]
fn methods_bus_error_at_every_transaction_returns_err_and_next_call_works() {
    for m in ALL {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(None, None);
        let reference = run(call(&mut imu, &mut delay, m)).unwrap();
        let n = shared.transactions();
        assert!(n >= 1);
        for i in 1..=n {
            shared.arm(Some(i), None);
            assert_eq!(
                run(call(&mut imu, &mut delay, m)),
                Err(Error::Bus(BusFault)),
                "{m:?} transaction {i}"
            );
            shared.arm(None, None);
            if matches!(m, Method::Configure) {
                assert_eq!(imu.config(), None);
            } else {
                assert!(imu.config().is_some());
            }
            assert_eq!(run(call(&mut imu, &mut delay, m)).unwrap(), reference);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Cancel safety (A2, D4)
// ---------------------------------------------------------------------------------------------

#[test]
fn methods_cancel_at_every_await_then_the_same_call_gives_the_right_result() {
    for m in ALL {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(None, None);
        let reference = run(call(&mut imu, &mut delay, m)).unwrap();
        let total = shared.awaits();
        let mut cancelled = 0;
        for k in 1..=1000 {
            shared.arm(None, Some(k));
            match poll_then_drop(call(&mut imu, &mut delay, m)) {
                Polled::Done(res) => {
                    assert_eq!(res.unwrap(), reference);
                    break;
                }
                Polled::Cancelled => cancelled += 1,
            }
            if matches!(m, Method::Configure) {
                assert_eq!(imu.config(), None);
                assert_eq!(run(imu.read()), Err(Error::NotConfigured));
            } else {
                assert!(imu.config().is_some(), "{m:?} cancelled at await {k}");
            }
            shared.arm(None, None);
            assert_eq!(
                run(call(&mut imu, &mut delay, m)).unwrap(),
                reference,
                "{m:?} cancelled at await {k}"
            );
            assert_eq!(shared.violations(), 0);
        }
        assert_eq!(cancelled, total, "{m:?}");
        if !matches!(m, Method::Configure) {
            assert_eq!(total, 1, "{m:?}");
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Blocking front-end (A11) and Send (A5)
// ---------------------------------------------------------------------------------------------

#[test]
fn blocking_init_configure_read_match_the_async_driver() {
    let cfg = Config::new().with_gyro_odr(Odr::Hz240);

    let (sa, mut a, mut da) = rig();
    let sb = Shared::new();
    let mut b = blocking::Lsm6dsk320x::new(sb.bus());
    let mut db = sb.delay();

    sa.arm(None, None);
    sb.arm(None, None);
    let ia = run(a.init(&mut da, &cfg));
    let ib = b.init(&mut db, &cfg);
    assert_eq!(ia, ib);
    assert!(ia.is_ok());
    assert_eq!(sa.log(), sb.log());
    assert_eq!(a.config(), b.config());

    sa.arm(None, None);
    sb.arm(None, None);
    assert_eq!(run(a.configure(&mut da, &cfg)), b.configure(&mut db, &cfg));
    assert_eq!(sa.log(), sb.log());

    sa.arm(None, None);
    sb.arm(None, None);
    assert_eq!(run(a.read()), b.read());
    assert_eq!(sa.log(), sb.log());
    assert_eq!(sa.log().len(), 1);
    assert_eq!(sa.chip().regs, sb.chip().regs);

    let (_s, mut a, _d) = rig();
    let s = Shared::new();
    let mut b = blocking::Lsm6dsk320x::new(s.bus());
    assert_eq!(run(a.read()), b.read());
    assert_eq!(b.read(), Err(Error::NotConfigured));
    assert_eq!(b.config(), None);
}

fn assert_send<T: Send>(_: &T) {}

#[test]
fn driver_and_futures_are_send() {
    let (_shared, mut imu, mut delay) = rig();
    let cfg = Config::new();
    assert_send(&imu);
    assert_send(&imu.init(&mut delay, &cfg));
    assert_send(&imu.configure(&mut delay, &cfg));
    assert_send(&imu.read());
    assert_send(&imu.read_high_g());
    assert_send(&imu.read_all());
    assert_send(&imu.status());
    assert_send(&imu.verify());
    assert_send(&imu.recover());
    assert_send(&imu.self_test(&mut delay, &crate::SelfTestConfig::new()));
}

// ---------------------------------------------------------------------------------------------
// verify and recover (contract A4c)
// ---------------------------------------------------------------------------------------------

type Rig = (Shared, Imu, TestDelay);

/// A driver configured with `c` by `init` (`by_init`) or by `configure`.
fn applied(c: &Config, by_init: bool) -> Rig {
    let (shared, mut imu, mut delay) = rig();
    if by_init {
        run(imu.init(&mut delay, c)).unwrap();
    } else {
        run(imu.configure(&mut delay, c)).unwrap();
    }
    (shared, imu, delay)
}

async fn vr(imu: &mut Imu, recover: bool) -> Res<()> {
    if recover {
        imu.recover().await
    } else {
        imu.verify().await
    }
}

/// The transactions of a successful `verify` for the default `Config`: (address, register value).
const DEFAULT_VERIFY: [(u8, u8); OWNED_COUNT] = [
    (0x12, 0x44), // CTRL3 (p67): BDU and IF_INC
    (0x19, 0x00), // CTRL10 (p74): ST_XL and ST_G off
    (0x03, 0x01), // IF_CFG (p58)
    (0x02, 0x23), // PIN_CTRL (p57)
    (0x13, 0x0A), // CTRL4 (p68)
    (0x0D, 0x01), // INT1_CTRL (p63)
    (0x0E, 0x00), // INT2_CTRL (p64)
    (0x5F, 0x00), // MD2_CFG (p103)
    (0x50, 0x00), // FUNCTIONS_ENABLE (p93)
    (0x53, 0x00), // HG_WAKE_UP_THS (p94)
    (0x55, 0x00), // INACTIVITY_THS (p96)
    (0x4D, 0x00), // CTRL2_XL_HG (p91)
    (0x52, 0x00), // HG_FUNCTIONS_ENABLE (p94)
    (0x16, 0x00), // CTRL7 (p70)
    (0x17, 0x02), // CTRL8 (p71)
    (0x18, 0x00), // CTRL9 (p72)
    (0x15, 0x0C), // CTRL6 (p69)
    (0x62, 0x00), // HAODR_CFG (p103)
    (0x07, 0x00), // FIFO_CTRL1 (p59)
    (0x0B, 0x00), // COUNTER_BDR_REG1 (p62)
    (0x08, 0x00), // FIFO_CTRL2 (p59)
    (0x09, 0x00), // FIFO_CTRL3 (p60)
    (0x0A, 0x00), // FIFO_CTRL4 (p61)
    (0x4E, 0x00), // CTRL1_XL_HG (p91)
    (0x10, 0x09), // CTRL1 (p65)
    (0x11, 0x09), // CTRL2 (p66)
    (0x6C, 0x00), // XL_HG_X_OFS_USR (p108)
    (0x6D, 0x00), // XL_HG_Y_OFS_USR (p108)
    (0x6E, 0x00), // XL_HG_Z_OFS_USR (p108)
    (0x73, 0x00), // X_OFS_USR (p113)
    (0x74, 0x00), // Y_OFS_USR (p113)
    (0x75, 0x00), // Z_OFS_USR (p113)
];

fn default_verify_events() -> Vec<Event> {
    DEFAULT_VERIFY.iter().map(|&(a, v)| r(a, v)).collect()
}

#[test]
fn verify_default_golden_trace() {
    for by_init in [true, false] {
        let (shared, mut imu, _d) = applied(&Config::new(), by_init);
        shared.arm(None, None);
        assert_eq!(run(imu.verify()), Ok(()));
        assert_eq!(shared.log(), default_verify_events());
        assert_eq!(shared.transactions(), 32);
        assert_eq!(shared.awaits(), 32);
        assert_eq!(shared.violations(), 0);
    }
}

#[test]
fn recover_default_golden_trace() {
    let (shared, mut imu, _d) = configured();
    shared.arm(None, None);
    assert_eq!(run(imu.recover()), Ok(()));
    let mut expected = vec![
        // One frame, one byte: the read address of WHO_AM_I (0x0F | 0x80, p64); nothing else.
        Event::Spi(vec![Op::Write(vec![0x8F])]),
        r(0x0F, 0x75),
    ];
    expected.extend(default_verify_events());
    assert_eq!(shared.log(), expected);
    assert_eq!(shared.transactions(), 34);
    assert_eq!(shared.awaits(), 34);
    assert_eq!(shared.violations(), 0);
}

#[test]
fn verify_not_configured_does_not_touch_the_bus() {
    let (shared, mut imu, _d) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(200))]

    #[test]
    fn verify_is_ok_after_init_and_after_configure_in_the_documented_count(c in valid_config_s()) {
        for by_init in [true, false] {
            let (shared, mut imu, _d) = applied(&c, by_init);
            shared.arm(None, None);
            prop_assert_eq!(run(imu.verify()), Ok(()));
            prop_assert_eq!(shared.transactions(), OWNED_COUNT);
            prop_assert_eq!(shared.awaits() as usize, OWNED_COUNT);
            prop_assert_eq!(shared.violations(), 0);
            prop_assert_eq!(imu.config(), Some(&c));
            shared.arm(None, None);
            prop_assert_eq!(run(imu.recover()), Ok(()));
            prop_assert_eq!(shared.transactions(), OWNED_COUNT + 2);
        }
    }

    #[test]
    fn verify_detects_every_masked_bit_for_valid_configs(c in valid_config_s()) {
        flip_every_bit(&c);
    }
}

#[test]
fn verify_ok_for_several_unlike_configs() {
    let configs = [
        Config::new(),
        Config::new()
            .with_gyro_odr(Odr::Off)
            .with_accel_odr(Odr::Hz240),
        Config::new().with_mode(crate::OperatingMode::HighAccuracyOdr(
            crate::HaodrSet::Base1000,
        )),
        Config::new().with_high_g(Some(crate::HighGConfig::new())),
    ];
    for c in configs {
        for by_init in [true, false] {
            let (shared, mut imu, _d) = applied(&c, by_init);
            shared.arm(None, None);
            assert_eq!(run(imu.verify()), Ok(()), "{c:?}");
            assert_eq!(shared.transactions(), OWNED_COUNT);
        }
    }
}

/// Flips every bit of every register of the fake, one at a time, and runs `verify`.
fn flip_every_bit(c: &Config) {
    let owned = c.owned_registers(&crate::calc::Offsets::new());
    for by_init in [true, false] {
        let (shared, mut imu, _d) = applied(c, by_init);
        for address in 0..=0xFFu8 {
            for bit in 0..8 {
                let m = 1u8 << bit;
                let before = shared.chip().regs[address as usize];
                shared.chip().regs[address as usize] = before ^ m;
                shared.arm(None, None);
                let out = run(imu.verify());
                match owned
                    .iter()
                    .position(|o| o.reg == address && o.mask & m != 0)
                {
                    Some(i) => {
                        assert_eq!(
                            out,
                            Err(Error::ConfigMismatch(Mismatch {
                                register: address,
                                mask: owned[i].mask,
                                expected: owned[i].expected,
                                found: before ^ m,
                            })),
                            "register {address:#04x} bit {bit}"
                        );
                        // Reads up to and including the register, and the one re-read.
                        assert_eq!(shared.transactions(), i + 2);
                    }
                    None => {
                        assert_eq!(out, Ok(()), "register {address:#04x} bit {bit}");
                        assert_eq!(shared.transactions(), OWNED_COUNT);
                    }
                }
                assert_eq!(imu.config(), Some(c));
                shared.chip().regs[address as usize] = before;
            }
        }
    }
}

#[test]
fn verify_detects_every_masked_bit_and_ignores_the_others() {
    flip_every_bit(&Config::new());
    // At least one owned bit of each register is in a mask.
    for o in Config::new().owned_registers(&crate::calc::Offsets::new()) {
        assert_ne!(o.mask, 0);
        assert_eq!(o.expected & !o.mask, 0);
    }
}

#[test]
fn verify_one_off_glitch_is_ok_with_one_extra_transaction() {
    for by_init in [true, false] {
        let c = Config::new();
        for (i, o) in c
            .owned_registers(&crate::calc::Offsets::new())
            .iter()
            .enumerate()
        {
            let (shared, mut imu, _d) = applied(&c, by_init);
            shared.arm(None, None);
            shared.glitch(i + 1, o.mask & o.mask.wrapping_neg());
            assert_eq!(run(imu.verify()), Ok(()), "register {:#04x}", o.reg);
            assert_eq!(shared.transactions(), OWNED_COUNT + 1);
            assert_eq!(shared.chip().glitch, None);
            assert_eq!(imu.config(), Some(&c));
        }
    }
}

#[test]
fn verify_and_recover_keep_the_config_on_mismatch_and_the_next_read_works() {
    let (shared, mut imu, _d) = configured();
    shared.chip().regs[reg::CTRL8 as usize] ^= 0x01;
    let expected = Err(Error::ConfigMismatch(Mismatch {
        register: reg::CTRL8,
        mask: 0xE3,
        expected: 0x02,
        found: 0x03,
    }));
    for recover in [false, true] {
        shared.arm(None, None);
        assert_eq!(run(vr(&mut imu, recover)), expected);
        assert_eq!(imu.config(), Some(&Config::new()));
        assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
        // Nothing was written.
        assert!(shared.log().iter().all(|e| match e {
            Event::Spi(ops) => !matches!(&ops[..], [Op::Write(w)] if w.len() == 2),
            Event::Delay(_) => false,
        }));
    }
}

#[test]
fn verify_and_recover_bus_error_at_every_transaction_then_the_same_call_works() {
    for recover in [false, true] {
        let (shared, mut imu, _d) = configured();
        let regs = shared.chip().regs;
        shared.arm(None, None);
        assert_eq!(run(vr(&mut imu, recover)), Ok(()));
        let n = shared.transactions();
        assert_eq!(n, if recover { 34 } else { 32 });
        for i in 1..=n {
            shared.arm(Some(i), None);
            assert_eq!(
                run(vr(&mut imu, recover)),
                Err(Error::Bus(BusFault)),
                "recover {recover} transaction {i}"
            );
            assert_eq!(imu.config(), Some(&Config::new()));
            shared.arm(None, None);
            assert_eq!(run(vr(&mut imu, recover)), Ok(()));
            assert_eq!(shared.chip().regs, regs);
            assert_eq!(shared.violations(), 0);
        }
    }
}

#[test]
fn verify_and_recover_cancel_at_every_await_then_the_same_call_works() {
    for recover in [false, true] {
        let (shared, mut imu, _d) = configured();
        shared.arm(None, None);
        assert_eq!(run(vr(&mut imu, recover)), Ok(()));
        let total = shared.awaits();
        assert_eq!(total, if recover { 34 } else { 32 });
        let mut cancelled = 0;
        for k in 1..=1000 {
            shared.arm(None, Some(k));
            match poll_then_drop(vr(&mut imu, recover)) {
                Polled::Done(res) => {
                    assert_eq!(res, Ok(()));
                    break;
                }
                Polled::Cancelled => cancelled += 1,
            }
            assert_eq!(imu.config(), Some(&Config::new()), "cancelled at await {k}");
            shared.arm(None, None);
            assert_eq!(run(vr(&mut imu, recover)), Ok(()), "cancelled at await {k}");
            assert_eq!(shared.violations(), 0);
        }
        assert_eq!(cancelled, total);
    }
}

#[test]
fn recover_not_configured_writes_and_checks_identity_then_says_so() {
    let (shared, mut imu, _d) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.recover()), Err(Error::NotConfigured));
    assert_eq!(
        shared.log(),
        vec![Event::Spi(vec![Op::Write(vec![0x8F])]), r(0x0F, 0x75)]
    );
    assert_eq!(imu.config(), None);

    shared.chip().regs[reg::WHO_AM_I as usize] = 0x12;
    shared.arm(None, None);
    assert_eq!(run(imu.recover()), Err(Error::WrongId { found: 0x12 }));
    assert_eq!(shared.transactions(), 2);

    // Configured: a wrong identity stops before verify and leaves the state alone.
    let (shared, mut imu, _d) = configured();
    shared.chip().regs[reg::WHO_AM_I as usize] = 0x00;
    shared.arm(None, None);
    assert_eq!(run(imu.recover()), Err(Error::WrongId { found: 0 }));
    assert_eq!(shared.transactions(), 2);
    assert_eq!(imu.config(), Some(&Config::new()));
}

#[test]
fn verify_and_recover_through_blocking_match_the_async_driver() {
    for scenario in 0..4 {
        let (sa, mut a, _da) = configured();
        let sb = Shared::new();
        let mut b = blocking::Lsm6dsk320x::new(sb.bus());
        let mut db = sb.delay();
        b.init(&mut db, &Config::new()).unwrap();
        for s in [&sa, &sb] {
            match scenario {
                1 => s.chip().regs[reg::CTRL8 as usize] ^= 0x01,
                2 => s.chip().regs[reg::WHO_AM_I as usize] = 0x00,
                _ => {}
            }
        }
        sa.arm(None, None);
        sb.arm(None, None);
        assert_eq!(run(a.verify()), b.verify());
        assert_eq!(sa.log(), sb.log());
        sa.arm(None, None);
        sb.arm(None, None);
        let ra = run(a.recover());
        assert_eq!(ra, b.recover());
        assert_eq!(sa.log(), sb.log());
        if scenario == 0 {
            assert_eq!(ra, Ok(()));
        }
        assert_eq!(a.config(), b.config());
        assert_eq!(sa.chip().regs, sb.chip().regs);
    }
    let (sa, mut a, _da) = rig();
    let sb = Shared::new();
    let mut b = blocking::Lsm6dsk320x::new(sb.bus());
    sa.arm(None, None);
    sb.arm(None, None);
    assert_eq!(run(a.verify()), b.verify());
    assert_eq!(run(a.recover()), b.recover());
    assert_eq!(b.recover(), Err(Error::NotConfigured));
    assert_eq!(sa.log()[..2], sb.log()[..2]);
}

#[test]
fn verify_masks_are_the_documented_ones() {
    // Reserved bits left out, with the pages in `Config::owned_registers`.
    let masks: [u8; OWNED_COUNT] = [
        0x44, 0x0F, 0xFD, 0xE3, 0x1F, 0x7B, 0xFF, 0x01, 0x40, 0xFF, 0xC0, 0x13, 0xFF, 0xC1, 0xE3,
        0x7B, 0x7F, 0x03, 0xFF, 0x6B, 0xD6, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF,
    ];
    let owned = Config::new().owned_registers(&crate::calc::Offsets::new());
    for (o, (m, (a, _))) in owned.iter().zip(masks.iter().zip(DEFAULT_VERIFY)) {
        assert_eq!((o.reg, o.mask), (a, *m));
    }
}

// ---------------------------------------------------------------------------------------------
// User offsets
// ---------------------------------------------------------------------------------------------

const OFFSET_REGS: [u8; 6] = [
    reg::XL_HG_X_OFS_USR,
    reg::XL_HG_Y_OFS_USR,
    reg::XL_HG_Z_OFS_USR,
    reg::X_OFS_USR,
    reg::Y_OFS_USR,
    reg::Z_OFS_USR,
];

fn offsets_a() -> Offsets {
    Offsets::new()
        .with_low_g([1, -2, 127])
        .with_high_g([-127, 3, 4])
}

fn offsets_b() -> Offsets {
    Offsets::new()
        .with_low_g([-5, 6, -7])
        .with_high_g([8, -9, 10])
}

fn offset_bytes(shared: &Shared) -> [u8; 6] {
    let c = shared.chip();
    OFFSET_REGS.map(|a| c.regs[a as usize])
}

fn mismatch_of(o: &Offsets, index: usize, found: u8) -> Error<BusFault> {
    let [hx, hy, hz] = o.high_g_bytes();
    let [lx, ly, lz] = o.low_g_bytes();
    let expected = [hx, hy, hz, lx, ly, lz][index];
    Error::ConfigMismatch(Mismatch {
        register: OFFSET_REGS[index],
        mask: 0xFF,
        expected,
        found,
    })
}

#[test]
fn set_offsets_golden_trace_two_bursts_and_state() {
    let (shared, mut imu, mut delay) = rig();
    assert_eq!(imu.offsets(), None);
    run(imu.init(&mut delay, &Config::new())).unwrap();
    assert_eq!(imu.offsets(), Some(Offsets::new()));
    shared.arm(None, None);
    assert_eq!(run(imu.set_offsets(&offsets_a())), Ok(()));
    // XL_HG_{X,Y,Z}_OFS_USR (0x6C to 0x6E, p108) first, then {X,Y,Z}_OFS_USR (0x73 to 0x75, p113):
    // two's complement bytes, one write burst each; registers 0x6F to 0x72 are not touched.
    assert_eq!(
        shared.log(),
        vec![
            Event::Spi(vec![Op::Write(vec![0x6C, 0x81, 0x03, 0x04])]),
            Event::Spi(vec![Op::Write(vec![0x73, 0x01, 0xFE, 0x7F])]),
        ]
    );
    assert_eq!(shared.transactions(), 2);
    assert_eq!(shared.awaits(), 2);
    assert_eq!(shared.violations(), 0);
    assert_eq!(imu.offsets(), Some(offsets_a()));
    assert_eq!(offset_bytes(&shared), [0x81, 3, 4, 1, 0xFE, 0x7F]);
    // The getter has no bus access.
    shared.arm(None, None);
    assert_eq!(imu.offsets(), Some(offsets_a()));
    assert_eq!(shared.transactions(), 0);
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(shared.transactions(), 32);

    // `configure` writes the stored offsets back: the registers and the driver hold them.
    let other = Config::new().with_accel_odr(Odr::Hz240);
    run(imu.configure(&mut delay, &other)).unwrap();
    assert_eq!(imu.offsets(), Some(offsets_a()));
    assert_eq!(offset_bytes(&shared), [0x81, 3, 4, 1, 0xFE, 0x7F]);
    assert_eq!(run(imu.verify()), Ok(()));
    // A failed `configure` leaves them too.
    shared.arm(Some(3), None);
    assert_eq!(
        run(imu.configure(&mut delay, &other)),
        Err(Error::Bus(BusFault))
    );
    assert_eq!(imu.offsets(), None);
    shared.arm(None, None);
    run(imu.configure(&mut delay, &other)).unwrap();
    assert_eq!(imu.offsets(), Some(offsets_a()));
    assert_eq!(run(imu.verify()), Ok(()));
}

#[test]
fn set_offsets_not_configured_and_out_of_range_use_no_bus() {
    let (shared, mut imu, _d) = rig();
    shared.arm(None, None);
    assert_eq!(
        run(imu.set_offsets(&offsets_a())),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.transactions(), 0);
    assert_eq!(imu.offsets(), None);

    let (shared, mut imu, _d) = configured();
    run(imu.set_offsets(&offsets_a())).unwrap();
    let regs = shared.chip().regs;
    for (bad, channel) in [
        (offsets_b().with_low_g([0, -128, 0]), Channel::LowG),
        (offsets_b().with_high_g([0, 0, -128]), Channel::HighG),
        (
            Offsets::new().with_low_g([-128; 3]).with_high_g([-128; 3]),
            Channel::LowG,
        ),
    ] {
        shared.arm(None, None);
        assert_eq!(
            run(imu.set_offsets(&bad)),
            Err(Error::InvalidConfig(ConfigError::OffsetOutOfRange {
                channel
            }))
        );
        assert_eq!(shared.transactions(), 0);
        assert_eq!(shared.awaits(), 0);
        assert_eq!(imu.offsets(), Some(offsets_a()));
        assert_eq!(shared.chip().regs, regs);
    }
    // The extremes of the range are accepted.
    let edge = Offsets::new().with_low_g([127; 3]).with_high_g([-127; 3]);
    assert_eq!(run(imu.set_offsets(&edge)), Ok(()));
    assert_eq!(imu.offsets(), Some(edge));
    assert_eq!(run(imu.verify()), Ok(()));
}

#[test]
fn set_offsets_bus_error_at_every_transaction_then_the_same_call_puts_it_right() {
    for i in 1..=2usize {
        let (shared, mut imu, _d) = configured();
        run(imu.set_offsets(&offsets_a())).unwrap();
        shared.arm(Some(i), None);
        assert_eq!(
            run(imu.set_offsets(&offsets_b())),
            Err(Error::Bus(BusFault)),
            "transaction {i}"
        );
        // Stored state unchanged; the failed transaction has no effect, so only the first
        // transaction's bytes can have reached the registers.
        assert_eq!(imu.offsets(), Some(offsets_a()));
        shared.arm(None, None);
        if i == 1 {
            assert_eq!(run(imu.verify()), Ok(()));
        } else {
            assert_eq!(
                run(imu.verify()),
                Err(mismatch_of(&offsets_a(), 0, offsets_b().high_g[0] as u8))
            );
        }
        assert_eq!(run(imu.set_offsets(&offsets_b())), Ok(()));
        assert_eq!(imu.offsets(), Some(offsets_b()));
        assert_eq!(run(imu.verify()), Ok(()));
        assert_eq!(imu.stats().bus_errors, 1);
    }
}

#[test]
fn set_offsets_cancel_at_every_await_then_the_same_call_puts_it_right() {
    let mut cancelled = 0;
    for k in 1..=10u32 {
        let (shared, mut imu, _d) = configured();
        run(imu.set_offsets(&offsets_a())).unwrap();
        shared.arm(None, Some(k));
        match poll_then_drop(imu.set_offsets(&offsets_b())) {
            Polled::Done(res) => {
                assert_eq!(res, Ok(()));
                assert_eq!(k, 3);
                break;
            }
            Polled::Cancelled => cancelled += 1,
        }
        assert_eq!(imu.offsets(), Some(offsets_a()), "cancelled at await {k}");
        shared.arm(None, None);
        if k == 1 {
            assert_eq!(run(imu.verify()), Ok(()));
        } else {
            assert_eq!(
                run(imu.verify()),
                Err(mismatch_of(&offsets_a(), 0, offsets_b().high_g[0] as u8)),
                "cancelled at await {k}"
            );
        }
        assert_eq!(run(imu.set_offsets(&offsets_b())), Ok(()));
        assert_eq!(imu.offsets(), Some(offsets_b()));
        assert_eq!(run(imu.verify()), Ok(()));
        assert_eq!(shared.violations(), 0);
    }
    assert_eq!(cancelled, 2);
}

#[test]
fn verify_detects_a_flipped_bit_in_each_offset_register() {
    let (shared, mut imu, _d) = configured();
    run(imu.set_offsets(&offsets_a())).unwrap();
    let applied_bytes = offset_bytes(&shared);
    for (index, &address) in OFFSET_REGS.iter().enumerate() {
        for bit in 0..8 {
            let found = applied_bytes[index] ^ (1 << bit);
            shared.chip().regs[address as usize] = found;
            shared.arm(None, None);
            assert_eq!(
                run(imu.verify()),
                Err(mismatch_of(&offsets_a(), index, found)),
                "register {address:#04x} bit {bit}"
            );
            // Reads up to and including the register, and the one re-read.
            assert_eq!(shared.transactions(), OWNED_COUNT - 6 + index + 2);
            shared.chip().regs[address as usize] = applied_bytes[index];
        }
    }
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(imu.offsets(), Some(offsets_a()));
}

#[test]
fn init_clears_the_offsets_in_the_driver_and_in_the_registers() {
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&offsets_a())).unwrap();
    assert_eq!(imu.offsets(), Some(offsets_a()));
    // The fake's software reset does not clear registers (it only clears SW_RESET), so the zero
    // bytes below come from the explicit zero writes of `init`, which this test proves.
    run(imu.init(&mut delay, &Config::new())).unwrap();
    assert_eq!(imu.offsets(), Some(Offsets::new()));
    assert_eq!(offset_bytes(&shared), [0; 6]);
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), Ok(()));
    // A failed `init` is not configured, with no offsets to report.
    shared.arm(Some(2), None);
    assert!(run(imu.init(&mut delay, &Config::new())).is_err());
    assert_eq!(imu.offsets(), None);
}

/// The state a successful `configure` must leave after anything: the driver stores `want`, the
/// registers hold it, and `verify` agrees.
fn assert_holds(shared: &Shared, imu: &mut Imu, want: &Offsets, what: &str) {
    assert_eq!(imu.offsets(), Some(*want), "{what}");
    let [hx, hy, hz] = want.high_g_bytes();
    let [lx, ly, lz] = want.low_g_bytes();
    assert_eq!(offset_bytes(shared), [hx, hy, hz, lx, ly, lz], "{what}");
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), Ok(()), "{what}");
}

#[test]
fn a_failed_init_at_every_transaction_keeps_the_stored_offsets_and_configure_restores_them() {
    for i in 1..=20usize {
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&offsets_a())).unwrap();
        shared.arm(Some(i), None);
        assert_eq!(
            run(imu.init(&mut delay, &Config::new())),
            Err(Error::Bus(BusFault)),
            "transaction {i}"
        );
        assert_eq!(imu.offsets(), None);
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_holds(
            &shared,
            &mut imu,
            &offsets_a(),
            &std::format!("init failed at {i}"),
        );
    }
}

#[test]
fn an_init_cancelled_at_every_await_keeps_the_stored_offsets_and_configure_restores_them() {
    let mut cancelled = 0;
    for k in 1..=1000u32 {
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&offsets_a())).unwrap();
        shared.arm(None, Some(k));
        match poll_then_drop(imu.init(&mut delay, &Config::new())) {
            Polled::Done(res) => {
                res.unwrap();
                break;
            }
            Polled::Cancelled => cancelled += 1,
        }
        assert_eq!(imu.offsets(), None);
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_holds(
            &shared,
            &mut imu,
            &offsets_a(),
            &std::format!("init cancelled at {k}"),
        );
    }
    assert_eq!(cancelled, 22);
}

#[test]
fn an_init_with_an_invalid_config_leaves_the_stored_offsets() {
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&offsets_a())).unwrap();
    let bad = Config::new()
        .with_mode(crate::OperatingMode::HighAccuracyOdr(
            crate::HaodrSet::Base960,
        ))
        .with_accel_odr(Odr::Hz7_5);
    shared.arm(None, None);
    assert!(matches!(
        run(imu.init(&mut delay, &bad)),
        Err(Error::InvalidConfig(_))
    ));
    assert_eq!(shared.transactions(), 0);
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_holds(&shared, &mut imu, &offsets_a(), "invalid init");
}

#[test]
fn configure_on_a_new_driver_clears_offsets_another_program_left() {
    let (shared, mut imu, mut delay) = rig();
    for (address, value) in OFFSET_REGS.iter().zip([0x81u8, 3, 4, 1, 0xFE, 0x7F]) {
        shared.chip().regs[*address as usize] = value;
    }
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_eq!(offset_bytes(&shared), [0; 6]);
    assert_holds(&shared, &mut imu, &Offsets::new(), "new driver, configure");
}

#[test]
fn a_set_offsets_cut_off_after_its_first_write_is_put_right_by_configure() {
    // Bus error at the second transaction.
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&offsets_a())).unwrap();
    shared.arm(Some(2), None);
    assert_eq!(
        run(imu.set_offsets(&offsets_b())),
        Err(Error::Bus(BusFault))
    );
    let [bx, by, bz] = offsets_b().high_g_bytes();
    let [lx, ly, lz] = offsets_a().low_g_bytes();
    assert_eq!(offset_bytes(&shared), [bx, by, bz, lx, ly, lz]);
    shared.arm(None, None);
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_holds(
        &shared,
        &mut imu,
        &offsets_a(),
        "bus error after the first write",
    );

    // Dropped after the first write completed (await 2 is the second burst).
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&offsets_a())).unwrap();
    shared.arm(None, Some(2));
    assert_eq!(
        poll_then_drop(imu.set_offsets(&offsets_b())),
        Polled::Cancelled
    );
    assert_eq!(offset_bytes(&shared), [bx, by, bz, lx, ly, lz]);
    shared.arm(None, None);
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_holds(
        &shared,
        &mut imu,
        &offsets_a(),
        "dropped after the first write",
    );
}

#[test]
fn set_offsets_and_offsets_through_blocking_match_the_async_driver() {
    let (sa, mut a, _da) = configured();
    let sb = Shared::new();
    let mut b = blocking::Lsm6dsk320x::new(sb.bus());
    let mut db = sb.delay();
    assert_eq!(b.offsets(), None);
    assert_eq!(b.set_offsets(&offsets_a()), Err(Error::NotConfigured));
    b.init(&mut db, &Config::new()).unwrap();
    for o in [
        offsets_a(),
        offsets_b().with_low_g([-128, 0, 0]),
        offsets_b(),
        Offsets::new(),
    ] {
        sa.arm(None, None);
        sb.arm(None, None);
        assert_eq!(run(a.set_offsets(&o)), b.set_offsets(&o));
        assert_eq!(sa.log(), sb.log());
        assert_eq!(a.offsets(), b.offsets());
        assert_eq!(run(a.verify()), b.verify());
        assert_eq!(sa.chip().regs, sb.chip().regs);
    }
    assert_eq!(b.offsets(), Some(Offsets::new()));
    sb.arm(Some(2), None);
    assert_eq!(b.set_offsets(&offsets_a()), Err(Error::Bus(BusFault)));
    assert_eq!(b.offsets(), Some(Offsets::new()));
}

//! Driver for the ST LSM6DSK320X: a six-axis IMU (gyroscope and low-g accelerometer) with a
//! high-g accelerometer, over four-wire SPI.
//!
//! The crate depends on `embedded-hal` and `embedded-hal-async` only. It does not own interrupt
//! pins, timers or tasks: [`Lsm6dsk320x::init`] configures the chip's INT1 and INT2 outputs and
//! the application waits on them. Device knowledge ([`reg`], [`Config`], the encoders and the
//! decoders in [`sample`]) is plain code with no bus access.
//!
//! The built-in self-test is [`Lsm6dsk320x::self_test`] (see [`selftest`]). User offsets are written by
//! [`Lsm6dsk320x::set_offsets`]. FIFO words are read by
//! [`Lsm6dsk320x::fifo_read`] and decoded by the pure [`fifo_frames`]. Not covered at all: embedded
//! functions (FSM, MLC, step counter, tap, free-fall), I2C/I3C, sensor hub, low-power modes.
//! The register map, the [`Config`] family with its validation, the register write plan
//! ([`Config::plan`]) and the pure helpers in [`calc`] are in place and need no bus.
//!
//! # Units
//!
//! Conversions give acceleration in m/s², angular rate in rad/s and temperature in °C, as `f32`,
//! from the raw counts and the full scale that was configured.
//!
//! # Async
//!
//! ```ignore
//! use lsm6dsk320x::{Config, Lsm6dsk320x};
//!
//! let mut imu = Lsm6dsk320x::new(spi_device);
//! let config = Config::default();
//! imu.init(&mut delay, &config).await?;
//! loop {
//!     int1.wait_for_rising_edge().await;
//!     let raw = imu.read().await?;
//!     let accel = raw.accel.accel_ms2(config.accel_fs);
//!     let gyro = raw.gyro.gyro_rads(config.gyro_fs);
//! }
//! ```
//!
//! # Blocking
//!
//! ```ignore
//! use lsm6dsk320x::{blocking, Config};
//!
//! let mut imu = blocking::Lsm6dsk320x::new(spi_device);
//! let config = Config::default();
//! imu.init(&mut delay, &config)?;
//! let raw = imu.read()?;
//! ```
#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::{Operation, SpiDevice};

use crate::image::{OWNED_COUNT, Owned};

pub mod blocking;
pub mod calc;
pub mod config;
mod data;
pub mod fifo;
mod image;
pub mod plan;
pub mod reg;
pub mod sample;
pub mod selftest;

pub use calc::Offsets;
pub use config::{
    AccelFullScale, AccelLpf2Bandwidth, BatchRate, Channel, Config, ConfigError, DataReady,
    DataReadyMode, FIFO_DEPTH_WORDS, FifoConfig, FifoMode, GyroFullScale, GyroLpf1Bandwidth,
    HaodrSet, HighGConfig, HighGEvents, HighGFullScale, HighGOdr, HighGWakeUpMode, IntOutput,
    IntPolarity, IntRouting, Odr, OffsetWeight, OperatingMode, PadDrive, PinConfig, TempBatchRate,
    TimestampBatch,
};
pub use data::{FifoFlags, FifoRead, FifoStatus, InterruptStatus, Stats};
pub use fifo::{FifoFrame, FifoFrames, fifo_frames};
pub use plan::{Apply, Plan, Step};
pub use sample::{RawAll, RawAxes, RawSample, Status};
pub use selftest::{
    AxisResult, ChannelResult, SelfTestConfig, SelfTestReport, SelfTestSequence, SelfTestStep,
    Stimulus,
};

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod data_tests;
#[cfg(test)]
mod driver_tests;
#[cfg(test)]
mod selftest_tests;
#[cfg(test)]
mod testbus;
#[cfg(test)]
mod tests;

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<E> {
    /// The SPI bus failed.
    Bus(E),
    /// WHO_AM_I did not read 0x75.
    WrongId {
        /// The value read.
        found: u8,
    },
    /// SW_RESET did not clear in time.
    ResetTimeout,
    /// No configuration is applied: `init` or `configure` has not completed since the driver was
    /// created or since the last one that failed or was dropped.
    NotConfigured,
    /// The [`Config`] is not valid. Found before any bus access.
    InvalidConfig(ConfigError),
    /// A register the applied configuration owns did not read back as configured, twice in a
    /// row. Returned by [`Lsm6dsk320x::verify`] and [`Lsm6dsk320x::recover`]; the driver is
    /// unchanged and still configured.
    ConfigMismatch(Mismatch),
    /// The buffer is shorter than the words asked for, or shorter than one FIFO word (7 bytes).
    /// Found before any bus access.
    BufferTooSmall,
    /// `fifo_read_exact` was asked for 0 words. Found before any bus access.
    ZeroWords,
    /// The applied configuration has the FIFO off (mode bypass), so the FIFO methods have
    /// nothing to work on. Found before any bus access.
    FifoOff,
    /// A data-ready bit did not come within the bound while `self_test` waited for a sample of
    /// this channel (200 ms at 500 µs steps). The driver is not configured: call `init` or
    /// `configure`.
    DataReadyTimeout {
        /// The channel being tested.
        channel: Channel,
    },
    /// The [`SelfTestConfig`] selects no channel or 0 samples. Found before any bus access.
    InvalidSelfTest,
}

/// A register that differs from what the applied configuration put there.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Mismatch {
    /// Register address.
    pub register: u8,
    /// The bits that were compared.
    pub mask: u8,
    /// The value expected under `mask`; bits outside `mask` are 0.
    pub expected: u8,
    /// The value read, whole register (the second of the two reads).
    pub found: u8,
}

/// What `init` and `configure` learned and applied.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Info {
    /// WHO_AM_I as read (0x75 for this chip).
    pub who_am_i: u8,
    /// INTERNAL_FREQ_FINE as read: the signed factory trim of the internal clock (9.54, p92).
    pub freq_fine: i8,
    /// Real gyroscope output rate in millihertz, 0 when it is off (see [`calc::real_odr_mhz`]).
    pub gyro_odr_mhz: u32,
    /// Real low-g accelerometer output rate in millihertz, 0 when it is off.
    pub accel_odr_mhz: u32,
    /// Real high-g accelerometer output rate in millihertz, 0 when it is off.
    pub high_g_odr_mhz: u32,
    /// Expected FIFO words per second, rounded up (see [`fifo::fifo_words_per_second`]).
    pub fifo_words_per_second: u32,
}

/// Polls of CTRL3 after a software reset, 100 µs apart (the reset has been measured at about 230 µs).
const RESET_POLLS: u32 = 1000;
const RESET_POLL_US: u32 = 100;

/// The configuration the chip is known to hold, and the trim read with it.
#[derive(Clone, Copy)]
struct Applied {
    config: Config,
    /// What `init` or `configure` computed, the clock trim included; [`Lsm6dsk320x::info`].
    info: Info,
}

/// Async driver over an [`embedded_hal_async::spi::SpiDevice`].
///
/// # Validity
///
/// The driver is "configured" only between the successful end of [`init`](Self::init) or
/// [`configure`](Self::configure) and the start of the next one. Both clear it before their first
/// bus access, so a failure or a dropped future leaves the driver not configured, and `read`,
/// `read_high_g`, `read_all` and `status` return [`Error::NotConfigured`] until a call completes.
/// The raw register methods ([`read_register`](Self::read_register),
/// [`write_register`](Self::write_register), [`read_registers`](Self::read_registers)) are outside
/// this: they neither check nor change the state.
///
/// # Transactions
///
/// `read`, `read_high_g`, `read_all` and `status` are one SPI transaction each; so are
/// `fifo_status` and `fifo_read_exact`. `fifo_read` is two, or one when nothing is read, and
/// `fifo_flush` is two writes ([`fifo_read`](Self::fifo_read) has the rest). Every register
/// read the driver issues is one transaction of the form `[Write(address | 0x80), Read(buffer)]`,
/// never an equal-length transfer: on embassy-rp 0.10 only this shape drains the SPI receive
/// FIFO, so it heals after a cancelled transfer (contract A4a). Writes are one `Write`.
///
/// An SPI transfer in flight must not be cancelled on embassy-rp 0.10: put a timeout around the
/// wait for data-ready, not around a bus call. After a cancel call [`recover`](Self::recover).
pub struct Lsm6dsk320x<SPI> {
    spi: SPI,
    applied: Option<Applied>,
    /// The user offsets the driver has written and the registers hold. Changed only after the
    /// device was written: set to 0 by `init` when it marks the driver configured, replaced by
    /// `set_offsets` after its last write succeeded. `configure` writes this value to the
    /// registers and leaves it. A failed or dropped call leaves it as it was. Meaningful while
    /// `applied` is `Some`.
    offsets: Offsets,
    stats: Stats,
}

impl<SPI: SpiDevice> Lsm6dsk320x<SPI> {
    /// Wraps the SPI device. Nothing is sent. The driver starts not configured.
    pub const fn new(spi: SPI) -> Self {
        Self {
            spi,
            applied: None,
            offsets: Offsets::new(),
            stats: Stats::new(),
        }
    }

    /// Gives the SPI device back.
    pub fn release(self) -> SPI {
        self.spi
    }

    /// The applied configuration; `None` while the driver is not configured.
    pub fn config(&self) -> Option<&Config> {
        self.applied.as_ref().map(|a| &a.config)
    }

    /// Resets the chip, checks WHO_AM_I and applies `config` ([`Apply::AfterReset`] plan).
    ///
    /// The driver is not configured from the start of this call until it returns `Ok`. When the
    /// gyroscope is enabled the plan includes its 40 ms turn-on time (datasheet p15), so `read()`
    /// gives valid data once `init` returns.
    ///
    /// The user offsets ([`set_offsets`](Self::set_offsets)) are cleared: the plan is followed by
    /// two writes of 0 to the six offset registers (see below), and [`offsets`](Self::offsets)
    /// reads all zero once `init` returns. The stored offsets change only when `init` marks the
    /// driver configured, so an `init` that fails or is dropped leaves them as they were, and
    /// [`configure`](Self::configure) then writes those values back. The datasheet says of SW_RESET that it "resets all
    /// control registers to their default value" (9.16, p67) and lists 0 as the default of the
    /// offset registers (Table 26, p54), but does not name the offset registers; they are written
    /// anyway so that [`verify`](Self::verify) cannot report a healthy device as changed. That is
    /// two transactions more than the plan and the reset: `[Write(0x6C), 0, 0, 0]` and
    /// `[Write(0x73), 0, 0, 0]`, after the last plan step and before the read of the clock trim.
    ///
    /// The plan switches the self-test stimulus off (ST_XL and ST_G in CTRL10, one read-modify-write
    /// of the mask 0x0F; the plan has XL_HG_ST in CTRL2_XL_HG) whatever the reset did.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<SPI::Error>> {
        self.applied = None;
        let plan = config
            .plan(Apply::AfterReset)
            .map_err(Error::InvalidConfig)?;

        self.write_register(
            reg::CTRL3,
            reg::CTRL3_BDU | reg::CTRL3_IF_INC | reg::CTRL3_SW_RESET,
        )
        .await?;
        let mut polls = 0;
        loop {
            delay.delay_us(RESET_POLL_US).await;
            if self.read_register(reg::CTRL3).await? & reg::CTRL3_SW_RESET == 0 {
                break;
            }
            polls += 1;
            if polls >= RESET_POLLS {
                return Err(Error::ResetTimeout);
            }
        }

        let who_am_i = self.check_identity().await?;
        self.apply(delay, config, &plan, who_am_i, Offsets::new())
            .await
    }

    /// Applies `config` to a chip that is not reset ([`Apply::InPlace`] plan: FIFO to bypass and
    /// the sensors to power-down first, then every register).
    ///
    /// WHO_AM_I is checked first. The driver is not configured from the start of this call until
    /// it returns `Ok`.
    ///
    /// The plan switches the self-test stimulus off: ST_XL and ST_G (CTRL10, p74) by one
    /// read-modify-write of the mask 0x0F that leaves the other bits, and XL_HG_ST by the
    /// CTRL2_XL_HG write (p91). A [`self_test`](Self::self_test) that was cut short may have left
    /// the stimulus on; [`verify`](Self::verify) reports it.
    ///
    /// The user offsets are calibration, not configuration, and this restores them: after the plan
    /// and before it marks the driver configured it writes all six offset registers from the
    /// stored value ([`offsets`](Self::offsets); the two bursts of
    /// [`set_offsets`](Self::set_offsets)), which is two transactions more than the plan, the
    /// identity check and the read of the clock trim. So after any successful `configure` the
    /// registers hold what the driver stores, whatever came before: an `init` that failed, a
    /// `set_offsets` that was cut off, a driver created on a chip that another program
    /// calibrated (it stores zero, so the registers are cleared). To change the offsets, call
    /// [`set_offsets`](Self::set_offsets).
    pub async fn configure(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<SPI::Error>> {
        self.applied = None;
        let plan = config.plan(Apply::InPlace).map_err(Error::InvalidConfig)?;
        let who_am_i = self.check_identity().await?;
        let offsets = self.offsets;
        self.apply(delay, config, &plan, who_am_i, offsets).await
    }

    async fn check_identity(&mut self) -> Result<u8, Error<SPI::Error>> {
        let found = self.read_register(reg::WHO_AM_I).await?;
        if found != reg::WHO_AM_I_VALUE {
            return Err(Error::WrongId { found });
        }
        Ok(found)
    }

    /// Runs `plan`, writes `offsets` to the offset registers, reads the clock trim and only then
    /// stores `offsets` and marks the driver configured.
    async fn apply(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
        plan: &Plan,
        who_am_i: u8,
        offsets: Offsets,
    ) -> Result<Info, Error<SPI::Error>> {
        self.run_plan(delay, plan).await?;
        self.write_offset_registers(&offsets).await?;
        // The trim is a signed byte; the cast keeps the bits.
        let freq_fine = self.read_register(reg::INTERNAL_FREQ_FINE).await? as i8;
        let info = Info {
            who_am_i,
            freq_fine,
            gyro_odr_mhz: calc::real_odr_mhz(config.gyro_odr, freq_fine),
            accel_odr_mhz: calc::real_odr_mhz(config.accel_odr, freq_fine),
            high_g_odr_mhz: match config.high_g {
                Some(h) => calc::scale_rate_mhz(h.odr.nominal_mhz(), freq_fine),
                None => 0,
            },
            fifo_words_per_second: fifo::fifo_words_per_second(config, freq_fine),
        };
        self.offsets = offsets;
        self.applied = Some(Applied {
            config: *config,
            info,
        });
        Ok(info)
    }

    async fn run_plan(
        &mut self,
        delay: &mut impl DelayNs,
        plan: &Plan,
    ) -> Result<(), Error<SPI::Error>> {
        for step in plan {
            self.run_step(delay, step).await?;
        }
        Ok(())
    }

    async fn run_step(
        &mut self,
        delay: &mut impl DelayNs,
        step: &Step,
    ) -> Result<(), Error<SPI::Error>> {
        match *step {
            Step::Write { reg, value } => self.write_register(reg, value).await?,
            Step::Modify { reg, mask, value } => {
                let current = self.read_register(reg).await?;
                self.write_register(reg, Step::modified(current, mask, value))
                    .await?;
            }
            Step::DelayUs(us) => delay.delay_us(us).await,
        }
        Ok(())
    }

    fn ensure_configured(&self) -> Result<(), Error<SPI::Error>> {
        if self.applied.is_some() {
            Ok(())
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// The user offsets the driver has written and the registers should hold; `None` while the
    /// driver is not configured. No bus access. All zero after [`init`](Self::init);
    /// [`configure`](Self::configure) writes them back to the registers and leaves them;
    /// [`set_offsets`](Self::set_offsets) changes them.
    pub fn offsets(&self) -> Option<Offsets> {
        self.applied.as_ref().map(|_| self.offsets)
    }

    /// Writes the user offset registers: XL_HG_X_OFS_USR to XL_HG_Z_OFS_USR (0x6C to 0x6E, p108)
    /// and X_OFS_USR to Z_OFS_USR (0x73 to 0x75, p113). Whether the offsets are applied to the
    /// output is set by the [`Config`] ([`Config::accel_offset`] for low-g,
    /// [`HighGConfig::user_offset`] for high-g).
    ///
    /// [`Error::NotConfigured`] while the driver is not configured, and
    /// [`Error::InvalidConfig`] holding [`ConfigError::OffsetOutOfRange`] for a value of -128
    /// ([`Offsets::validate`]), both before any bus access.
    ///
    /// Transactions: 2 writes, the fewest the register layout allows: registers 0x6F to 0x72 lie
    /// between the two groups and are not ours, so the groups cannot be one burst. First
    /// `[Write(0x6C), x, y, z]` for the high-g group, then `[Write(0x73), x, y, z]` for the
    /// low-g group, each a burst that relies on IF_INC, which `init` and `configure` set.
    ///
    /// The offsets are stored as applied ([`offsets`](Self::offsets)) only after the last write
    /// succeeded. If a write fails or the future is dropped before that, the stored offsets are
    /// unchanged, so [`verify`](Self::verify) reports the registers that already changed (as an
    /// [`Error::ConfigMismatch`] naming one of them) and calling this again with the same or any
    /// other offsets puts it right.
    pub async fn set_offsets(&mut self, offsets: &Offsets) -> Result<(), Error<SPI::Error>> {
        self.ensure_configured()?;
        offsets.validate().map_err(Error::InvalidConfig)?;
        self.write_offset_registers(offsets).await?;
        self.offsets = *offsets;
        Ok(())
    }

    /// The two bursts of [`set_offsets`](Self::set_offsets), high-g group first.
    async fn write_offset_registers(&mut self, offsets: &Offsets) -> Result<(), Error<SPI::Error>> {
        let [hx, hy, hz] = offsets.high_g_bytes();
        let sent = self
            .spi
            .write(&[reg::XL_HG_X_OFS_USR & !reg::SPI_READ, hx, hy, hz])
            .await;
        self.bus(sent)?;
        let [lx, ly, lz] = offsets.low_g_bytes();
        let sent = self
            .spi
            .write(&[reg::X_OFS_USR & !reg::SPI_READ, lx, ly, lz])
            .await;
        self.bus(sent)
    }

    /// Gyroscope and low-g accelerometer in one SPI transaction. [`Error::NotConfigured`] while
    /// the driver is not configured.
    pub async fn read(&mut self) -> Result<RawSample, Error<SPI::Error>> {
        self.ensure_configured()?;
        let mut b = [0u8; 12];
        self.read_registers(reg::OUTX_L_G, &mut b).await?;
        Ok(RawSample::from_bytes(&b))
    }

    /// High-g accelerometer axes in one SPI transaction.
    pub async fn read_high_g(&mut self) -> Result<RawAxes, Error<SPI::Error>> {
        self.ensure_configured()?;
        let mut b = [0u8; 6];
        self.read_registers(reg::UI_OUTX_L_A_OIS_HG, &mut b).await?;
        Ok(RawAxes::from_bytes(&b))
    }

    /// Temperature, gyroscope, low-g and high-g in one burst over the whole output block.
    pub async fn read_all(&mut self) -> Result<RawAll, Error<SPI::Error>> {
        self.ensure_configured()?;
        let mut b = [0u8; RawAll::LEN];
        self.read_registers(reg::OUT_TEMP_L, &mut b).await?;
        Ok(RawAll::from_bytes(&b))
    }

    /// STATUS_REG.
    pub async fn status(&mut self) -> Result<Status, Error<SPI::Error>> {
        self.ensure_configured()?;
        Ok(Status::from_bits(
            self.read_register(reg::STATUS_REG).await?,
        ))
    }

    /// Reads back the registers the applied configuration owns and compares them with what the
    /// configuration puts there.
    ///
    /// [`Error::NotConfigured`] while the driver is not configured, with no bus access. Otherwise
    /// one read transaction per owned register, 32 whatever the [`Config`] and the offsets, in this
    /// order: CTRL3, CTRL10, IF_CFG, PIN_CTRL, CTRL4, INT1_CTRL, INT2_CTRL, MD2_CFG, FUNCTIONS_ENABLE,
    /// HG_WAKE_UP_THS, INACTIVITY_THS, CTRL2_XL_HG, HG_FUNCTIONS_ENABLE, CTRL7, CTRL8, CTRL9, CTRL6,
    /// HAODR_CFG, FIFO_CTRL1, COUNTER_BDR_REG1, FIFO_CTRL2, FIFO_CTRL3, FIFO_CTRL4, CTRL1_XL_HG,
    /// CTRL1, CTRL2, then the six user offset registers against [`offsets`](Self::offsets):
    /// XL_HG_X_OFS_USR, XL_HG_Y_OFS_USR, XL_HG_Z_OFS_USR, X_OFS_USR, Y_OFS_USR, Z_OFS_USR (all
    /// eight bits compared). No awaits other than those 32 transactions when every register
    /// matches.
    ///
    /// A register that differs under its mask is read once more; only if it differs again does the
    /// call stop with [`Error::ConfigMismatch`] (so a mismatch costs one extra transaction, and a
    /// one-off glitch costs one and still returns `Ok`). The masks leave out the reserved bits;
    /// the registers' images are those of the register plan.
    /// CTRL10 is compared under the mask 0x0F (ST_XL and ST_G) against 0, and CTRL2_XL_HG includes
    /// XL_HG_ST, so a self-test stimulus left on is reported.
    ///
    /// Confirmed on the real chip on 2026-11-10: `Ok` after `init` and after `configure` for three
    /// unlike configurations, and the right register named after a deliberate change.
    ///
    /// Nothing is written and the driver's state and [`config`](Self::config) are never changed:
    /// the caller decides what to do after a mismatch ([`configure`](Self::configure), keep
    /// reading, or switch sensors). Cancelling between transactions is safe; call it again.
    pub async fn verify(&mut self) -> Result<(), Error<SPI::Error>> {
        let config = match self.applied.as_ref() {
            Some(a) => a.config,
            None => return Err(Error::NotConfigured),
        };
        let owned: [Owned; OWNED_COUNT] = config.owned_registers(&self.offsets);
        for o in owned {
            let found = self.read_register(o.reg).await?;
            if (found ^ o.expected) & o.mask == 0 {
                continue;
            }
            let found = self.read_register(o.reg).await?;
            if (found ^ o.expected) & o.mask != 0 {
                return Err(Error::ConfigMismatch(Mismatch {
                    register: o.reg,
                    mask: o.mask,
                    expected: o.expected,
                    found,
                }));
            }
        }
        Ok(())
    }

    /// Brings the bus back in step after a cancelled or failed transfer, then checks the chip.
    ///
    /// 1. One transaction holding a single one-byte `Write`: the read address of WHO_AM_I
    ///    (0x8F). It empties the HAL's receive FIFO and re-frames the bus; it writes no device
    ///    register.
    /// 2. The identity check: one read of WHO_AM_I, [`Error::WrongId`] when it is not 0x75.
    /// 3. [`verify`](Self::verify) when configured; [`Error::NotConfigured`] otherwise.
    ///
    /// Transactions: 34 when configured and every register matches, 2 when not configured. The
    /// state and [`config`](Self::config) are never changed. Cancelling between transactions is
    /// safe; call it again.
    pub async fn recover(&mut self) -> Result<(), Error<SPI::Error>> {
        let sent = self.spi.write(&[reg::WHO_AM_I | reg::SPI_READ]).await;
        self.bus(sent)?;
        self.check_identity().await?;
        self.verify().await
    }

    /// Reads one register.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<SPI::Error>> {
        let mut v = [0u8];
        self.read_registers(reg, &mut v).await?;
        Ok(v[0])
    }

    /// Writes one register.
    pub async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        let sent = self.spi.write(&[reg & !reg::SPI_READ, value]).await;
        self.bus(sent)
    }

    /// Reads `buf.len()` registers from `reg` on, using the address auto-increment.
    pub async fn read_registers(
        &mut self,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<SPI::Error>> {
        let done = self
            .spi
            .transaction(&mut [
                Operation::Write(&[reg | reg::SPI_READ]),
                Operation::Read(buf),
            ])
            .await;
        self.bus(done)
    }

    /// Turns a bus result into the driver's, counting an error in [`Stats::bus_errors`].
    fn bus<T>(&mut self, result: Result<T, SPI::Error>) -> Result<T, Error<SPI::Error>> {
        result.map_err(|e| {
            self.stats.bus_errors = self.stats.bus_errors.saturating_add(1);
            Error::Bus(e)
        })
    }
}

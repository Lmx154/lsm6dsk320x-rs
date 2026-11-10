//! Driver for the ST LSM6DSK320X: a six-axis IMU (gyroscope and low-g accelerometer) with a
//! high-g accelerometer, over four-wire SPI.
//!
//! The crate depends on `embedded-hal` and `embedded-hal-async` only. It does not own interrupt
//! pins, timers or tasks: [`Lsm6dsk320x::init`] configures the chip's INT1 and INT2 outputs and
//! the application waits on them. Device knowledge ([`reg`], [`Config`], the encoders and the
//! decoders in [`sample`]) is plain code with no bus access.
//!
//! Not covered: FIFO, embedded functions (FSM, MLC, step counter, tap, free-fall), self-test,
//! I2C/I3C, sensor hub, low-power modes.
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

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::{Operation, SpiDevice};

pub mod blocking;
pub mod config;
pub mod reg;
pub mod sample;

pub use config::{
    AccelFullScale, Config, DataReady, GyroFullScale, HighGConfig, HighGFullScale, HighGOdr, Odr,
};
pub use sample::{RawAll, RawAxes, RawSample, Status};

#[cfg(test)]
extern crate std;

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
}

/// Polls of CTRL3 after a software reset, 100 µs apart (the reset has been measured at about 230 µs).
const RESET_POLLS: u32 = 1000;
const RESET_POLL_US: u32 = 100;
/// Gyroscope turn-on time (p15).
const GYRO_TURN_ON_MS: u32 = 40;

/// Async driver over an [`embedded_hal_async::spi::SpiDevice`].
pub struct Lsm6dsk320x<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice> Lsm6dsk320x<SPI> {
    /// Wraps the SPI device. Nothing is sent.
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Gives the SPI device back.
    pub fn release(self) -> SPI {
        self.spi
    }

    /// Resets the chip, checks WHO_AM_I and writes `config`.
    ///
    /// When the gyroscope is enabled this includes its 40 ms turn-on time (datasheet p15), so
    /// `read()` gives valid data once `init` returns.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<(), Error<SPI::Error>> {
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

        let found = self.read_register(reg::WHO_AM_I).await?;
        if found != reg::WHO_AM_I_VALUE {
            return Err(Error::WrongId { found });
        }

        self.write_register(reg::IF_CFG, config.if_cfg()).await?;
        self.write_register(reg::CTRL4, config.ctrl4()).await?;
        self.write_register(reg::INT1_CTRL, config.int1_ctrl())
            .await?;
        self.write_register(reg::INT2_CTRL, config.int2_ctrl())
            .await?;
        self.write_register(reg::CTRL7, config.ctrl7()).await?;
        self.write_register(reg::CTRL8, config.ctrl8()).await?;
        self.write_register(reg::CTRL6, config.ctrl6()).await?;
        self.write_register(reg::CTRL1_XL_HG, config.ctrl1_xl_hg())
            .await?;
        self.write_register(reg::CTRL1, config.ctrl1()).await?;
        self.write_register(reg::CTRL2, config.ctrl2()).await?;
        if config.gyro_odr != Odr::Off {
            delay.delay_ms(GYRO_TURN_ON_MS).await;
        }
        Ok(())
    }

    /// Gyroscope and low-g accelerometer in one SPI transaction.
    pub async fn read(&mut self) -> Result<RawSample, Error<SPI::Error>> {
        let mut b = [0u8; 13];
        b[0] = reg::OUTX_L_G | reg::SPI_READ;
        self.spi
            .transfer_in_place(&mut b)
            .await
            .map_err(Error::Bus)?;
        Ok(RawSample::from_bytes(&payload(&b)))
    }

    /// High-g accelerometer axes in one SPI transaction.
    pub async fn read_high_g(&mut self) -> Result<RawAxes, Error<SPI::Error>> {
        let mut b = [0u8; 7];
        b[0] = reg::UI_OUTX_L_A_OIS_HG | reg::SPI_READ;
        self.spi
            .transfer_in_place(&mut b)
            .await
            .map_err(Error::Bus)?;
        Ok(RawAxes::from_bytes(&payload(&b)))
    }

    /// Temperature, gyroscope, low-g and high-g in one burst over the whole output block.
    pub async fn read_all(&mut self) -> Result<RawAll, Error<SPI::Error>> {
        let mut b = [0u8; RawAll::LEN + 1];
        b[0] = reg::OUT_TEMP_L | reg::SPI_READ;
        self.spi
            .transfer_in_place(&mut b)
            .await
            .map_err(Error::Bus)?;
        Ok(RawAll::from_bytes(&payload(&b)))
    }

    /// STATUS_REG.
    pub async fn status(&mut self) -> Result<Status, Error<SPI::Error>> {
        Ok(Status::from_bits(
            self.read_register(reg::STATUS_REG).await?,
        ))
    }

    /// Reads one register.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<SPI::Error>> {
        let mut v = [0u8];
        self.read_registers(reg, &mut v).await?;
        Ok(v[0])
    }

    /// Writes one register.
    pub async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        self.spi
            .write(&[reg & !reg::SPI_READ, value])
            .await
            .map_err(Error::Bus)
    }

    /// Reads `buf.len()` registers from `reg` on, using the address auto-increment.
    pub async fn read_registers(
        &mut self,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<SPI::Error>> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[reg | reg::SPI_READ]),
                Operation::Read(buf),
            ])
            .await
            .map_err(Error::Bus)
    }
}

/// The bytes of a transfer buffer after the address byte.
fn payload<const N: usize>(b: &[u8]) -> [u8; N] {
    let mut out = [0u8; N];
    for (o, i) in out.iter_mut().zip(b.iter().skip(1)) {
        *o = *i;
    }
    out
}

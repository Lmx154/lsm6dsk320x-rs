//! Blocking front-end over `embedded_hal` 1.0 traits, with the same API as the async driver.
//!
//! There is one implementation. A private adapter presents the blocking SPI device and delay as
//! async ones whose futures never pend, and each method here polls the async method to completion.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

use crate::{Config, Error, RawAll, RawAxes, RawSample, Status};

struct SpiAdapter<SPI>(SPI);

impl<SPI: ErrorType> ErrorType for SpiAdapter<SPI> {
    type Error = SPI::Error;
}

impl<SPI: SpiDevice> embedded_hal_async::spi::SpiDevice for SpiAdapter<SPI> {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.0.transaction(operations)
    }
}

struct DelayAdapter<'a, D>(&'a mut D);

impl<D: DelayNs> embedded_hal_async::delay::DelayNs for DelayAdapter<'_, D> {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.delay_ns(ns);
    }
}

fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
    }
}

/// Blocking driver over an [`embedded_hal::spi::SpiDevice`].
pub struct Lsm6dsk320x<SPI> {
    inner: crate::Lsm6dsk320x<SpiAdapter<SPI>>,
}

impl<SPI: SpiDevice> Lsm6dsk320x<SPI> {
    /// Wraps the SPI device. Nothing is sent.
    pub const fn new(spi: SPI) -> Self {
        Self {
            inner: crate::Lsm6dsk320x::new(SpiAdapter(spi)),
        }
    }

    /// Gives the SPI device back.
    pub fn release(self) -> SPI {
        self.inner.release().0
    }

    /// See [`crate::Lsm6dsk320x::init`].
    pub fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.init(&mut DelayAdapter(delay), config))
    }

    /// See [`crate::Lsm6dsk320x::read`].
    pub fn read(&mut self) -> Result<RawSample, Error<SPI::Error>> {
        block_on(self.inner.read())
    }

    /// See [`crate::Lsm6dsk320x::read_high_g`].
    pub fn read_high_g(&mut self) -> Result<RawAxes, Error<SPI::Error>> {
        block_on(self.inner.read_high_g())
    }

    /// See [`crate::Lsm6dsk320x::read_all`].
    pub fn read_all(&mut self) -> Result<RawAll, Error<SPI::Error>> {
        block_on(self.inner.read_all())
    }

    /// See [`crate::Lsm6dsk320x::status`].
    pub fn status(&mut self) -> Result<Status, Error<SPI::Error>> {
        block_on(self.inner.status())
    }

    /// See [`crate::Lsm6dsk320x::read_register`].
    pub fn read_register(&mut self, reg: u8) -> Result<u8, Error<SPI::Error>> {
        block_on(self.inner.read_register(reg))
    }

    /// See [`crate::Lsm6dsk320x::write_register`].
    pub fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.write_register(reg, value))
    }

    /// See [`crate::Lsm6dsk320x::read_registers`].
    pub fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.read_registers(reg, buf))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reg;
    use crate::{
        AccelFullScale, DataReady, GyroFullScale, HighGConfig, HighGFullScale, HighGOdr, Odr,
    };
    use core::convert::Infallible;
    use std::vec::Vec;

    /// Register file with the chip's framing: bit 7 of the first byte is read, the rest of it is
    /// the address, and the address increments after every data byte.
    struct FakeChip {
        regs: [u8; 256],
        writes: Vec<(u8, u8)>,
    }

    impl FakeChip {
        fn new() -> Self {
            let mut regs = [0u8; 256];
            regs[reg::WHO_AM_I as usize] = reg::WHO_AM_I_VALUE;
            regs[reg::CTRL3 as usize] = 0x44;
            for i in 0..12 {
                regs[reg::OUTX_L_G as usize + i] = i as u8 + 1;
            }
            Self {
                regs,
                writes: Vec::new(),
            }
        }

        fn clock(&mut self, header: &mut Option<u8>, addr: &mut u8, byte: u8) -> u8 {
            let Some(h) = *header else {
                *header = Some(byte);
                *addr = byte & 0x7F;
                return 0;
            };
            let a = *addr as usize;
            *addr = addr.wrapping_add(1);
            if h & 0x80 != 0 {
                self.regs[a]
            } else {
                self.regs[a] = if a == reg::CTRL3 as usize {
                    byte & !reg::CTRL3_SW_RESET
                } else {
                    byte
                };
                self.writes.push((a as u8, byte));
                0
            }
        }
    }

    impl ErrorType for FakeChip {
        type Error = Infallible;
    }

    impl SpiDevice for FakeChip {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Infallible> {
            let (mut header, mut addr) = (None, 0);
            for op in operations {
                match op {
                    Operation::Write(w) => {
                        for &b in w.iter() {
                            self.clock(&mut header, &mut addr, b);
                        }
                    }
                    Operation::Read(r) => {
                        for b in r.iter_mut() {
                            *b = self.clock(&mut header, &mut addr, 0);
                        }
                    }
                    Operation::TransferInPlace(t) => {
                        for b in t.iter_mut() {
                            *b = self.clock(&mut header, &mut addr, *b);
                        }
                    }
                    Operation::Transfer(r, w) => {
                        for (o, i) in r.iter_mut().zip(w.iter()) {
                            *o = self.clock(&mut header, &mut addr, *i);
                        }
                    }
                    Operation::DelayNs(_) => {}
                }
            }
            Ok(())
        }
    }

    struct NoDelay;

    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn init_then_read() {
        let config = Config {
            accel_fs: AccelFullScale::G4,
            accel_odr: Odr::Hz960,
            gyro_fs: GyroFullScale::Dps500,
            gyro_odr: Odr::Hz240,
            high_g: Some(HighGConfig {
                odr: HighGOdr::Hz960,
                full_scale: HighGFullScale::G64,
            }),
            int1: DataReady {
                accel: true,
                gyro: false,
                high_g: false,
            },
            int2: DataReady {
                accel: false,
                gyro: false,
                high_g: true,
            },
        };
        let mut imu = Lsm6dsk320x::new(FakeChip::new());
        imu.init(&mut NoDelay, &config).unwrap();
        let sample = imu.read().unwrap();
        let chip = imu.release();

        assert_eq!(
            chip.writes,
            std::vec![
                (reg::CTRL3, 0x45),
                (reg::IF_CFG, 0x01),
                (reg::CTRL4, 0x02),
                (reg::INT1_CTRL, 0x01),
                (reg::INT2_CTRL, 0x00),
                (reg::CTRL7, 0x40),
                (reg::CTRL8, 0x01),
                (reg::CTRL6, 0x0A),
                (reg::CTRL1_XL_HG, 0x80 | (4 << 3) | 1),
                (reg::CTRL1, 0x09),
                (reg::CTRL2, 0x07),
            ]
        );
        assert_eq!(
            sample.gyro,
            RawAxes {
                x: 0x0201,
                y: 0x0403,
                z: 0x0605
            }
        );
        assert_eq!(
            sample.accel,
            RawAxes {
                x: 0x0807,
                y: 0x0A09,
                z: 0x0C0B
            }
        );
    }
}

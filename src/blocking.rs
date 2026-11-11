//! Blocking front-end over `embedded_hal` 1.0 traits, with the same API as the async driver.
//!
//! There is one implementation. A private adapter presents the blocking SPI device and delay as
//! async ones whose futures never pend, and each method here polls the async method to completion.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

use crate::calc::Offsets;
use crate::{
    Config, Error, FifoRead, FifoStatus, Info, InterruptStatus, RawAll, RawAxes, RawSample,
    SelfTestConfig, SelfTestReport, Stats, Status,
};

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
    ) -> Result<Info, Error<SPI::Error>> {
        block_on(self.inner.init(&mut DelayAdapter(delay), config))
    }

    /// See [`crate::Lsm6dsk320x::configure`].
    pub fn configure(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<SPI::Error>> {
        block_on(self.inner.configure(&mut DelayAdapter(delay), config))
    }

    /// See [`crate::Lsm6dsk320x::config`].
    pub fn config(&self) -> Option<&Config> {
        self.inner.config()
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

    /// See [`crate::Lsm6dsk320x::fifo_read`].
    pub fn fifo_read(&mut self, buf: &mut [u8]) -> Result<FifoRead, Error<SPI::Error>> {
        block_on(self.inner.fifo_read(buf))
    }

    /// See [`crate::Lsm6dsk320x::fifo_read_exact`].
    pub fn fifo_read_exact(
        &mut self,
        buf: &mut [u8],
        words: usize,
    ) -> Result<FifoRead, Error<SPI::Error>> {
        block_on(self.inner.fifo_read_exact(buf, words))
    }

    /// See [`crate::Lsm6dsk320x::fifo_status`].
    pub fn fifo_status(&mut self) -> Result<FifoStatus, Error<SPI::Error>> {
        block_on(self.inner.fifo_status())
    }

    /// See [`crate::Lsm6dsk320x::fifo_flush`].
    pub fn fifo_flush(&mut self) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.fifo_flush())
    }

    /// See [`crate::Lsm6dsk320x::interrupt_status`].
    pub fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<SPI::Error>> {
        block_on(self.inner.interrupt_status())
    }

    /// See [`crate::Lsm6dsk320x::stats`].
    pub const fn stats(&self) -> &Stats {
        self.inner.stats()
    }

    /// See [`crate::Lsm6dsk320x::info`].
    pub fn info(&self) -> Option<Info> {
        self.inner.info()
    }

    /// See [`crate::Lsm6dsk320x::verify`].
    pub fn verify(&mut self) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.verify())
    }

    /// See [`crate::Lsm6dsk320x::recover`].
    pub fn recover(&mut self) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.recover())
    }

    /// See [`crate::Lsm6dsk320x::set_offsets`].
    pub fn set_offsets(&mut self, offsets: &Offsets) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.set_offsets(offsets))
    }

    /// See [`crate::Lsm6dsk320x::offsets`].
    pub fn offsets(&self) -> Option<Offsets> {
        self.inner.offsets()
    }

    /// See [`crate::Lsm6dsk320x::self_test`].
    pub fn self_test(
        &mut self,
        delay: &mut impl DelayNs,
        test: &SelfTestConfig,
    ) -> Result<SelfTestReport, Error<SPI::Error>> {
        block_on(self.inner.self_test(&mut DelayAdapter(delay), test))
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

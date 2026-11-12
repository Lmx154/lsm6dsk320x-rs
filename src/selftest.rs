//! The built-in self-test of the low-g accelerometer, the gyroscope and the high-g accelerometer:
//! three register sequences as data, a pure evaluation of the result, and
//! [`Lsm6dsk320x::self_test`], which executes them.
//!
//! # Sources of the procedure
//!
//! ST publishes no self-test sequence for the LSM6DSK320X or for the LSM6DSV320X. The only
//! published ST procedure in the family is the LSM6DSV16X example in ST's driver examples
//! (`lsm6dsv16x_self_test.c`, BSD-3-Clause). It is used here as facts only; no code is copied.
//! This module takes the SHAPE of that procedure for all three channels and takes every limit and
//! every bit from the datasheet of THIS part (ST DS15060 Rev 1). The waits are parameters; the first
//! runs on the real sensor are under "Measured" below.
//!
//! Per channel: the other channels are put in power-down and the filters and offsets of the
//! channel are set to their reset state, so that nothing but the stimulus changes the output; the
//! channel is set to its test rate and full scale; wait `settle`; wait for data-ready, discard
//! `discard` samples, average `samples` samples per axis (the baseline); switch the stimulus on;
//! wait `settle`; wait for data-ready, discard `discard` samples, average `samples` samples per
//! axis; the absolute difference per axis must lie in the datasheet window; switch the stimulus
//! off. Afterwards the driver re-applies the [`Config`](crate::Config) and the offsets it had.
//!
//! | Channel | Rate and full scale | Window of the absolute difference | Source |
//! |---|---|---|---|
//! | low-g | 60 Hz, +-4 g (the family example's choice) | 50 to 1700 mg, full-scale independent | Table 3, p13, notes 11 to 13, p14 |
//! | gyroscope | 240 Hz (the family example's choice), +-250 dps | 20 to 80 dps | Table 3, p13, notes 14 and 15, p14 |
//! | high-g | 480 Hz, +-32 g | 150 to 2500 mg, full-scale independent | Table 3, p13, notes 11 to 13, p14 |
//!
//! The family example uses +-2000 dps for the gyroscope; Table 3 has rows for +-250 dps and
//! +-4000 dps only, so the lower of the two is used (finer steps, lower noise). The stimulus bits
//! are ST_XL and ST_G in CTRL10 (Table 75, p74) and XL_HG_ST in CTRL2_XL_HG (Table 146, p91).
//!
//! The high-g procedure is by analogy with the family procedure. The datasheet gives no self-test sequence, no settling time and no rate restriction for
//! it; 480 Hz and +-32 g are the lowest rate and full scale of Tables 149 and 150 (p91), chosen for
//! the lowest noise.
//!
//! # Measured
//!
//! First runs on the real sensor, measured on the MARV V2 board, 2026-11-10:
//!
//! - The stimulus settles within 10 ms (low-g accelerometer), 36 ms (gyroscope) and 5 ms (high-g
//!   accelerometer), against the 100 ms the driver waits by default.
//! - One full self-test takes 0.85 s.
//! - The high-g procedure, which the manufacturer does not publish and which was built by analogy,
//!   produced responses inside the datasheet's windows.
//! - The positive and negative responses of the accelerometers are not mirror images of each
//!   other (the pass window is on the absolute value).
//! - Only the gyroscope test drops the interrupt routing while it runs.
//!
//! # Units
//!
//! Low-g and high-g values are in micro-g, gyroscope values in millidegrees per second, computed
//! from the raw averages with the sensitivity of the test full scale (Table 3, p12) in integers.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

use crate::calc::{
    self, SELF_TEST_GYRO_250_DPS_MDPS, SELF_TEST_HIGH_G_MG, SELF_TEST_LOW_G_MG, SelfTestRange,
    SelfTestSign,
};
use crate::config::{
    AccelFullScale, Channel, ConfigError, GyroFullScale, HighGFullScale, HighGOdr, Odr,
};
use crate::plan::{Apply, Step};
use crate::sample::RawAxes;
use crate::{Error, Lsm6dsk320x, reg};

/// Rate of the low-g accelerometer during its test.
const LOW_G_ODR: Odr = Odr::Hz60;
/// Rate of the gyroscope during its test.
const GYRO_ODR: Odr = Odr::Hz240;
/// Rate of the high-g accelerometer during its test.
const HIGH_G_ODR: HighGOdr = HighGOdr::Hz480;

/// Low-g sensitivity at +-4 g, micro-g per LSB (0.122 mg, Table 3, p12).
const LOW_G_UG_PER_LSB: i32 = 122;
/// High-g sensitivity at +-32 g, micro-g per LSB (0.976 mg, Table 3, p12).
const HIGH_G_UG_PER_LSB: i32 = 976;
/// Gyroscope sensitivity at +-250 dps, millidegrees per second per LSB: 35/4 (8.75, Table 3, p12).
const GYRO_MDPS_NUM: i32 = 35;
const GYRO_MDPS_DEN: i32 = 4;

/// Polls of STATUS_REG for one sample, 500 µs apart: 200 ms, more than ten periods of the slowest
/// test rate.
const DRDY_POLLS: u32 = 400;
const DRDY_POLL_US: u32 = 500;

/// Most steps of a sequence.
pub const MAX_STEPS: usize = 12;

/// Which sign of the stimulus is applied.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Stimulus {
    /// Positive sign self-test (code 01).
    #[default]
    Positive,
    /// Negative sign self-test (code 10).
    Negative,
}

impl Stimulus {
    const fn sign(self) -> SelfTestSign {
        match self {
            Self::Positive => SelfTestSign::Positive,
            Self::Negative => SelfTestSign::Negative,
        }
    }
}

/// What [`Lsm6dsk320x::self_test`] tests and with which waits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SelfTestConfig {
    /// Test the low-g accelerometer.
    pub low_g: bool,
    /// Test the gyroscope.
    pub gyro: bool,
    /// Test the high-g accelerometer.
    pub high_g: bool,
    /// Sign of the stimulus.
    pub stimulus: Stimulus,
    /// Wait after a channel is set up and after the stimulus is switched, in milliseconds.
    pub settle_ms: u16,
    /// Samples averaged per phase and axis, 1 to 255.
    pub samples: u8,
    /// Samples read and dropped before the averaged ones, per phase.
    pub discard: u8,
}

impl SelfTestConfig {
    /// All three channels, positive stimulus, 100 ms settle, 5 samples, 1 discarded (the family
    /// example's values).
    pub const fn new() -> Self {
        Self {
            low_g: true,
            gyro: true,
            high_g: true,
            stimulus: Stimulus::Positive,
            settle_ms: 100,
            samples: 5,
            discard: 1,
        }
    }

    /// Sets [`Self::low_g`].
    #[must_use]
    pub const fn with_low_g(mut self, v: bool) -> Self {
        self.low_g = v;
        self
    }

    /// Sets [`Self::gyro`].
    #[must_use]
    pub const fn with_gyro(mut self, v: bool) -> Self {
        self.gyro = v;
        self
    }

    /// Sets [`Self::high_g`].
    #[must_use]
    pub const fn with_high_g(mut self, v: bool) -> Self {
        self.high_g = v;
        self
    }

    /// Sets [`Self::stimulus`].
    #[must_use]
    pub const fn with_stimulus(mut self, v: Stimulus) -> Self {
        self.stimulus = v;
        self
    }

    /// Sets [`Self::settle_ms`].
    #[must_use]
    pub const fn with_settle_ms(mut self, v: u16) -> Self {
        self.settle_ms = v;
        self
    }

    /// Sets [`Self::samples`].
    #[must_use]
    pub const fn with_samples(mut self, v: u8) -> Self {
        self.samples = v;
        self
    }

    /// Sets [`Self::discard`].
    #[must_use]
    pub const fn with_discard(mut self, v: u8) -> Self {
        self.discard = v;
        self
    }

    /// True when at least one channel is selected and `samples` is not 0.
    pub const fn is_valid(&self) -> bool {
        (self.low_g || self.gyro || self.high_g) && self.samples != 0
    }

    /// The selected channels in the order they run: low-g, gyroscope, high-g.
    pub const fn channels(&self) -> [Option<Channel>; 3] {
        [
            if self.low_g {
                Some(Channel::LowG)
            } else {
                None
            },
            if self.gyro { Some(Channel::Gyro) } else { None },
            if self.high_g {
                Some(Channel::HighG)
            } else {
                None
            },
        ]
    }

    /// The register sequence that tests `channel` with these settings.
    ///
    /// # Errors
    ///
    /// [`ConfigError::PlanOverflow`] if the steps do not fit [`MAX_STEPS`]. Not reachable today:
    /// the low-g and gyroscope lists have exactly 12 steps and the high-g list 10; reported so that
    /// a step is never dropped silently.
    pub fn sequence(&self, channel: Channel) -> Result<SelfTestSequence, ConfigError> {
        let settle = self.settle_ms as u32 * 1000;
        let sign = self.stimulus.sign();
        let mut s = SelfTestSequence::empty(channel);
        s.write(reg::CTRL1_XL_HG, 0)?;
        match channel {
            Channel::LowG => {
                s.write(reg::CTRL2, 0)?;
                s.write(reg::CTRL1, 0)?;
                s.write(reg::CTRL9, 0)?;
                s.write(reg::CTRL8, AccelFullScale::G4.code())?;
                s.write(reg::CTRL1, LOW_G_ODR.code())?;
            }
            Channel::Gyro => {
                s.write(reg::CTRL1, 0)?;
                s.write(reg::CTRL2, 0)?;
                s.write(reg::CTRL7, 0)?;
                s.write(
                    reg::CTRL6,
                    reg::CTRL6_MUST_BE_ONE | GyroFullScale::Dps250.code(),
                )?;
                s.write(reg::CTRL2, GYRO_ODR.code())?;
            }
            Channel::HighG => {
                s.write(reg::CTRL1, 0)?;
                s.write(reg::CTRL2, 0)?;
                s.write(
                    reg::CTRL1_XL_HG,
                    reg::CTRL1_XL_HG_REGOUT_EN
                        | (HIGH_G_ODR.code() << reg::CTRL1_XL_HG_ODR_SHIFT)
                        | HighGFullScale::G32.code(),
                )?;
            }
        }
        s.push(SelfTestStep::Bus(Step::DelayUs(settle)))?;
        s.push(SelfTestStep::Collect(Phase::Baseline))?;
        s.stimulus(sign)?;
        s.push(SelfTestStep::Bus(Step::DelayUs(settle)))?;
        s.push(SelfTestStep::Collect(Phase::Stimulated))?;
        s.stimulus(SelfTestSign::Off)?;
        Ok(s)
    }
}

impl Default for SelfTestConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Which of the two averages a [`SelfTestStep::Collect`] produces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Phase {
    /// Stimulus off.
    Baseline,
    /// Stimulus on.
    Stimulated,
}

/// One step of a self-test sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SelfTestStep {
    /// A register write, a read-modify-write or a wait, as in a [`Plan`](crate::Plan).
    Bus(Step),
    /// For each of `discard + samples` samples: poll STATUS_REG until the channel's data-ready bit
    /// is set (bounded), read the channel's output registers; average the last `samples`.
    Collect(Phase),
}

/// The steps that test one channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestSequence {
    channel: Channel,
    steps: [SelfTestStep; MAX_STEPS],
    len: usize,
}

impl SelfTestSequence {
    const fn empty(channel: Channel) -> Self {
        Self {
            channel,
            steps: [SelfTestStep::Bus(Step::DelayUs(0)); MAX_STEPS],
            len: 0,
        }
    }

    /// Appends a step. [`ConfigError::PlanOverflow`] when the list already holds [`MAX_STEPS`]
    /// steps; the step is then not added and the list is unchanged.
    fn push(&mut self, step: SelfTestStep) -> Result<(), ConfigError> {
        let slot = self
            .steps
            .get_mut(self.len)
            .ok_or(ConfigError::PlanOverflow)?;
        *slot = step;
        self.len += 1;
        Ok(())
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), ConfigError> {
        self.push(SelfTestStep::Bus(Step::Write { reg, value }))
    }

    /// Sets the stimulus of the channel: ST_XL or ST_G in CTRL10 (read-modify-write, the other
    /// fields stay), XL_HG_ST in CTRL2_XL_HG (a write; the rest of that register is re-applied
    /// afterwards).
    fn stimulus(&mut self, sign: SelfTestSign) -> Result<(), ConfigError> {
        self.push(SelfTestStep::Bus(stimulus_step(self.channel, sign)))
    }

    /// The channel tested.
    pub const fn channel(&self) -> Channel {
        self.channel
    }

    /// The steps, in order.
    pub fn steps(&self) -> &[SelfTestStep] {
        self.steps.get(..self.len).unwrap_or(&[])
    }
}

/// The step that sets (or, with [`SelfTestSign::Off`], clears) the stimulus of `channel`.
pub(crate) const fn stimulus_step(channel: Channel, sign: SelfTestSign) -> Step {
    match channel {
        Channel::LowG => Step::Modify {
            reg: reg::CTRL10,
            mask: reg::CTRL10_ST_XL_MASK | reg::CTRL10_ST_G_MASK,
            value: calc::ctrl10_self_test(sign, SelfTestSign::Off),
        },
        Channel::Gyro => Step::Modify {
            reg: reg::CTRL10,
            mask: reg::CTRL10_ST_XL_MASK | reg::CTRL10_ST_G_MASK,
            value: calc::ctrl10_self_test(SelfTestSign::Off, sign),
        },
        Channel::HighG => Step::Write {
            reg: reg::CTRL2_XL_HG,
            value: calc::ctrl2_xl_hg_self_test(sign),
        },
    }
}

/// The result for one axis. Units: micro-g for the accelerometers, millidegrees per second for the
/// gyroscope.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct AxisResult {
    /// Average without the stimulus.
    pub baseline: i32,
    /// Average with the stimulus.
    pub stimulated: i32,
    /// Absolute difference of the two raw averages, converted.
    pub difference: u32,
    /// The difference lies inside the datasheet window, limits included.
    pub pass: bool,
}

/// The result for one channel, axes X, Y, Z.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ChannelResult {
    /// X, Y and Z.
    pub axes: [AxisResult; 3],
}

impl ChannelResult {
    /// All three axes passed.
    pub fn passed(&self) -> bool {
        self.axes.iter().all(|a| a.pass)
    }
}

/// The result of [`Lsm6dsk320x::self_test`]: one entry per channel that was run.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SelfTestReport {
    /// Low-g accelerometer.
    pub low_g: Option<ChannelResult>,
    /// Gyroscope.
    pub gyro: Option<ChannelResult>,
    /// High-g accelerometer.
    pub high_g: Option<ChannelResult>,
}

impl SelfTestReport {
    /// The result of `channel`, if it was run.
    pub const fn channel(&self, channel: Channel) -> Option<&ChannelResult> {
        match channel {
            Channel::LowG => self.low_g.as_ref(),
            Channel::Gyro => self.gyro.as_ref(),
            Channel::HighG => self.high_g.as_ref(),
        }
    }

    /// At least one channel was run and every axis of every channel run passed.
    pub fn passed(&self) -> bool {
        let all = [self.low_g, self.gyro, self.high_g];
        all.iter().any(Option::is_some) && all.iter().flatten().all(ChannelResult::passed)
    }
}

/// Raw counts to the report unit: `raw * num / den`, rounded to nearest, halves away from zero.
const fn convert(raw: i32, num: i32, den: i32) -> i32 {
    // bound: |raw| <= 65535 and num <= 976, so the product fits i32.
    let n = raw * num;
    if n >= 0 {
        (n + den / 2) / den
    } else {
        (n - den / 2) / den
    }
}

/// The window of `channel` in the report unit.
const fn window(channel: Channel) -> SelfTestRange {
    match channel {
        Channel::LowG => SelfTestRange {
            min: SELF_TEST_LOW_G_MG.min * 1000,
            max: SELF_TEST_LOW_G_MG.max * 1000,
        },
        Channel::HighG => SelfTestRange {
            min: SELF_TEST_HIGH_G_MG.min * 1000,
            max: SELF_TEST_HIGH_G_MG.max * 1000,
        },
        Channel::Gyro => SelfTestRange {
            min: SELF_TEST_GYRO_250_DPS_MDPS.min,
            max: SELF_TEST_GYRO_250_DPS_MDPS.max,
        },
    }
}

const fn axis(channel: Channel, baseline: i16, stimulated: i16) -> AxisResult {
    let (num, den) = match channel {
        Channel::LowG => (LOW_G_UG_PER_LSB, 1),
        Channel::HighG => (HIGH_G_UG_PER_LSB, 1),
        Channel::Gyro => (GYRO_MDPS_NUM, GYRO_MDPS_DEN),
    };
    let diff = (stimulated as i32 - baseline as i32).abs();
    // bound: diff <= 65535, so the converted value is positive and below 2^27.
    let difference = convert(diff, num, den) as u32;
    AxisResult {
        baseline: convert(baseline as i32, num, den),
        stimulated: convert(stimulated as i32, num, den),
        difference,
        pass: window(channel).contains(difference),
    }
}

/// Pure evaluation: the raw averages of one channel without and with the stimulus (X, Y, Z) to the
/// per-axis result against the datasheet window.
pub const fn evaluate(channel: Channel, baseline: [i16; 3], stimulated: [i16; 3]) -> ChannelResult {
    let [bx, by, bz] = baseline;
    let [sx, sy, sz] = stimulated;
    ChannelResult {
        axes: [
            axis(channel, bx, sx),
            axis(channel, by, sy),
            axis(channel, bz, sz),
        ],
    }
}

/// Mean of `n` (not 0) samples with sum `sum`, rounded to nearest, halves away from zero.
pub(crate) const fn mean(sum: i32, n: u8) -> i16 {
    let n = n as i32;
    let m = if sum >= 0 {
        (sum + n / 2) / n
    } else {
        (sum - n / 2) / n
    };
    if m > i16::MAX as i32 {
        i16::MAX
    } else if m < i16::MIN as i32 {
        i16::MIN
    } else {
        m as i16
    }
}

impl<SPI: SpiDevice> Lsm6dsk320x<SPI> {
    /// Runs the built-in self-test of the selected channels, one after another, on a configured
    /// driver ([`SelfTestConfig`], the module documentation has the procedure and its sources).
    ///
    /// [`Error::NotConfigured`] while the driver is not configured and [`Error::InvalidSelfTest`]
    /// for a [`SelfTestConfig`] with no channel or 0 samples, both before any bus access and with
    /// the driver's state unchanged; so is [`Error::InvalidConfig`] holding
    /// [`ConfigError::PlanOverflow`] if a self-test step list does not fit [`MAX_STEPS`] (not
    /// reachable today, see [`SelfTestConfig::sequence`]). From there on the driver is not configured until the call
    /// completes: it is cleared before the first await, and set again only after the stimulus is
    /// off, the applied [`Config`](crate::Config) is re-applied with the in-place plan and the
    /// applied offsets are written back. The call takes about one second with the defaults, and
    /// the outputs and the interrupt pins are not meaningful while it runs.
    ///
    /// A self-test that completes but misses a limit is `Ok(report)` with
    /// [`SelfTestReport::passed`] false. A bus error or [`Error::DataReadyTimeout`] ends the call
    /// with the stimulus switched off when the bus still works (best effort); a dropped future
    /// cannot. In every such case the driver stays NOT configured and the device may still have
    /// the stimulus on or the configuration half applied: the caller must call
    /// [`init`](Self::init) or [`configure`](Self::configure), both of which clear ST_XL and ST_G
    /// (CTRL10) and XL_HG_ST (CTRL2_XL_HG).
    ///
    /// Transactions with the defaults and every data-ready flag set at the first poll: 34 for
    /// low-g, 34 for the gyroscope and 30 for high-g, then the in-place plan and the two writes
    /// of the offset registers. A flag that is not set costs another STATUS_REG read and 500 µs.
    pub async fn self_test(
        &mut self,
        delay: &mut impl DelayNs,
        test: &SelfTestConfig,
    ) -> Result<SelfTestReport, Error<SPI::Error>> {
        let saved = self.applied.ok_or(Error::NotConfigured)?;
        if !test.is_valid() {
            return Err(Error::InvalidSelfTest);
        }
        let plan = saved
            .config
            .plan(Apply::InPlace)
            .map_err(Error::InvalidConfig)?;
        let mut sequences = [None; 3];
        for (slot, channel) in sequences.iter_mut().zip(test.channels()) {
            if let Some(channel) = channel {
                *slot = Some(test.sequence(channel).map_err(Error::InvalidConfig)?);
            }
        }
        self.applied = None;

        let mut report = SelfTestReport::default();
        for sequence in sequences.into_iter().flatten() {
            let channel = sequence.channel();
            match self.run_sequence(delay, test, &sequence).await {
                Ok(result) => match channel {
                    Channel::LowG => report.low_g = Some(result),
                    Channel::Gyro => report.gyro = Some(result),
                    Channel::HighG => report.high_g = Some(result),
                },
                Err(e) => {
                    let off = stimulus_step(channel, SelfTestSign::Off);
                    // Best effort: the error being reported is the earlier one.
                    let _ = self.run_step(delay, &off).await;
                    return Err(e);
                }
            }
        }

        self.run_plan(delay, &plan).await?;
        let offsets = self.offsets;
        self.write_offset_registers(&offsets).await?;
        self.applied = Some(saved);
        Ok(report)
    }

    async fn run_sequence(
        &mut self,
        delay: &mut impl DelayNs,
        test: &SelfTestConfig,
        sequence: &SelfTestSequence,
    ) -> Result<ChannelResult, Error<SPI::Error>> {
        let mut baseline = [0i16; 3];
        let mut stimulated = [0i16; 3];
        for step in sequence.steps() {
            match *step {
                SelfTestStep::Bus(s) => self.run_step(delay, &s).await?,
                SelfTestStep::Collect(phase) => {
                    let m = self.collect(delay, test, sequence.channel()).await?;
                    match phase {
                        Phase::Baseline => baseline = m,
                        Phase::Stimulated => stimulated = m,
                    }
                }
            }
        }
        Ok(evaluate(sequence.channel(), baseline, stimulated))
    }

    /// `discard + samples` samples of `channel`, each after its own data-ready; the mean of the
    /// last `samples`.
    async fn collect(
        &mut self,
        delay: &mut impl DelayNs,
        test: &SelfTestConfig,
        channel: Channel,
    ) -> Result<[i16; 3], Error<SPI::Error>> {
        let (mut sx, mut sy, mut sz) = (0i32, 0i32, 0i32);
        let total = u16::from(test.discard) + u16::from(test.samples);
        for i in 0..total {
            self.wait_data_ready(delay, channel).await?;
            let s = self.read_output(channel).await?;
            if i >= u16::from(test.discard) {
                sx += i32::from(s.x);
                sy += i32::from(s.y);
                sz += i32::from(s.z);
            }
        }
        Ok([
            mean(sx, test.samples),
            mean(sy, test.samples),
            mean(sz, test.samples),
        ])
    }

    async fn wait_data_ready(
        &mut self,
        delay: &mut impl DelayNs,
        channel: Channel,
    ) -> Result<(), Error<SPI::Error>> {
        let bit = match channel {
            Channel::LowG => reg::STATUS_XLDA,
            Channel::Gyro => reg::STATUS_GDA,
            Channel::HighG => reg::STATUS_XLHGDA,
        };
        let mut polls = 0;
        while self.read_register(reg::STATUS_REG).await? & bit == 0 {
            polls += 1;
            if polls >= DRDY_POLLS {
                return Err(Error::DataReadyTimeout { channel });
            }
            delay.delay_us(DRDY_POLL_US).await;
        }
        Ok(())
    }

    async fn read_output(&mut self, channel: Channel) -> Result<RawAxes, Error<SPI::Error>> {
        let first = match channel {
            Channel::LowG => reg::OUTX_L_A,
            Channel::Gyro => reg::OUTX_L_G,
            Channel::HighG => reg::UI_OUTX_L_A_OIS_HG,
        };
        let mut b = [0u8; 6];
        self.read_registers(first, &mut b).await?;
        Ok(RawAxes::from_bytes(&b))
    }
}

#[cfg(test)]
mod overflow_tests {
    use super::*;
    use crate::config::ConfigError;

    #[test]
    fn push_past_capacity_is_an_error_and_keeps_the_steps() {
        let mut s = SelfTestSequence::empty(Channel::LowG);
        for i in 0..MAX_STEPS {
            assert_eq!(s.write(reg::CTRL1, i as u8), Ok(()));
        }
        assert_eq!(s.steps().len(), MAX_STEPS);
        assert_eq!(
            s.push(SelfTestStep::Collect(Phase::Baseline)),
            Err(ConfigError::PlanOverflow)
        );
        assert_eq!(s.write(reg::CTRL2, 0xFF), Err(ConfigError::PlanOverflow));
        assert_eq!(
            s.stimulus(SelfTestSign::Off),
            Err(ConfigError::PlanOverflow)
        );
        assert_eq!(s.steps().len(), MAX_STEPS);
        for (i, step) in s.steps().iter().enumerate() {
            assert_eq!(
                *step,
                SelfTestStep::Bus(Step::Write {
                    reg: reg::CTRL1,
                    value: i as u8
                })
            );
        }
    }
}

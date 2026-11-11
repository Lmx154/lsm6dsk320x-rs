//! The register write plan: the ordered steps that configure the device from a [`Config`].
//!
//! The plan is data. It is built without a bus and without allocation, and a driver only executes
//! it: [`Step::Write`] is one register write, [`Step::Modify`] a read-modify-write of the masked
//! bits, [`Step::DelayUs`] a wait.
//!
//! # Order
//!
//! 0. CTRL3: block data update and register auto-increment (p67), in both plans. Then the
//!    self-test stimulus of the low-g and gyroscope channels off (CTRL10, a read-modify-write).
//! 1. In place only: the FIFO goes to bypass mode, which empties it (6.5.1, 6.5.2, p45), and the
//!    three sensors go to power-down. The operating mode is changed in power-down (6.1.4, p34) and
//!    the gyroscope full scale is set in power-down (CTRL6 note, p69).
//! 2. The interface and the data-ready style (IF_CFG, PIN_CTRL, CTRL4), the interrupt routing, the
//!    timestamp counter and the high-g generator.
//! 3. The filters and full scales (CTRL7, CTRL8, CTRL9, CTRL6), the ODR set of the high-accuracy
//!    ODR mode and the operating mode, while the sensors are still in power-down.
//! 4. The FIFO, the timestamp counter being on already.
//! 5. The ODRs: high-g, low-g, gyroscope. Writing a nonzero ODR turns the sensor on.
//! 6. With the gyroscope on, 70 ms for its turn-on time ([`GYRO_TURN_ON_US`]).
//!
//! The turn-on time of the accelerometers and the filter settling times are not in the datasheet,
//! which refers to the application note (6, p33; CTRL4 note, p68): the plan adds no delay for them.
//! `DRDY_MASK` in [`Config::drdy_mask`] hides the data-ready signals until the filters settle.

use crate::config::{Config, ConfigError};
use crate::reg;

/// Wait after starting the gyroscope, in microseconds: 70 ms. The datasheet's turn-on time is 40 ms
/// in the typical column, "not guaranteed" (Table 4, p15). Measured on the MARV V2 board,
/// 2026-11-11: gyroscope samples were clean from about 65 ms after the sensor was started and off
/// by up to 6.6 dps before that (see [`Config::drdy_mask`]); 70 ms is the measured figure plus
/// margin.
pub const GYRO_TURN_ON_US: u32 = 70_000;

/// Most steps a plan can hold. The longest plan, in place with everything on, has 33.
pub const MAX_STEPS: usize = 34;

/// One step of a plan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Step {
    /// Write `value` to register `reg`.
    Write {
        /// Register address.
        reg: u8,
        /// The byte to write.
        value: u8,
    },
    /// Read register `reg`, replace the bits in `mask` by those of `value`, write it back. Used
    /// where the register also holds fields this crate does not own (functions it does not
    /// configure), so that they are left as they are.
    Modify {
        /// Register address.
        reg: u8,
        /// The bits to replace.
        mask: u8,
        /// The new value of those bits; bits outside `mask` are 0.
        value: u8,
    },
    /// Wait this many microseconds.
    DelayUs(u32),
}

impl Step {
    /// The byte to write for a [`Step::Modify`] when the register currently reads `current`.
    pub const fn modified(current: u8, mask: u8, value: u8) -> u8 {
        (current & !mask) | (value & mask)
    }
}

/// Where the plan starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Apply {
    /// Right after a software reset: the control registers are at their reset values (CTRL3 SW_RESET,
    /// p67), so a register whose image equals its reset value is not written. IF_CFG, PIN_CTRL and
    /// the other registers `init` has always written are always written. PIN_CTRL and IF_CFG are
    /// not reset by a software reset (p57-58).
    AfterReset,
    /// On a device in an unknown state, without a reset: every register is written, after a
    /// power-down block.
    InPlace,
}

/// A register write plan. Iterate it with [`Plan::steps`] or `for step in &plan`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Plan {
    steps: [Step; MAX_STEPS],
    len: usize,
}

impl Plan {
    const fn empty() -> Self {
        Self {
            steps: [Step::DelayUs(0); MAX_STEPS],
            len: 0,
        }
    }

    /// The steps, in order.
    pub fn steps(&self) -> &[Step] {
        self.steps.get(..self.len).unwrap_or(&[])
    }

    /// An iterator over the steps.
    pub fn iter(&self) -> core::slice::Iter<'_, Step> {
        self.steps().iter()
    }

    /// Number of steps.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// True when there is no step.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    fn push(&mut self, step: Step) -> Result<(), ConfigError> {
        let slot = self
            .steps
            .get_mut(self.len)
            .ok_or(ConfigError::PlanOverflow)?;
        *slot = step;
        self.len += 1;
        Ok(())
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), ConfigError> {
        self.push(Step::Write { reg, value })
    }

    /// A write that `AfterReset` skips when `value` is the reset value.
    fn write_unless_reset(
        &mut self,
        apply: Apply,
        reg: u8,
        value: u8,
        reset: u8,
    ) -> Result<(), ConfigError> {
        if apply == Apply::InPlace || value != reset {
            self.write(reg, value)?;
        }
        Ok(())
    }

    /// A read-modify-write that `AfterReset` skips when the new bits are 0 (the reset value).
    fn modify_unless_reset(
        &mut self,
        apply: Apply,
        reg: u8,
        mask: u8,
        value: u8,
    ) -> Result<(), ConfigError> {
        if apply == Apply::InPlace || value != 0 {
            self.push(Step::Modify { reg, mask, value })?;
        }
        Ok(())
    }
}

impl<'a> IntoIterator for &'a Plan {
    type Item = &'a Step;
    type IntoIter = core::slice::Iter<'a, Step>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl Config {
    /// Validates the configuration and builds the register write plan for it. A plan that would not
    /// fit in [`MAX_STEPS`] is [`ConfigError::PlanOverflow`], never a plan with a step dropped.
    ///
    /// For `Config::default()` and [`Apply::AfterReset`] the plan is the writes `init` has always
    /// made after the identity check (CTRL3 as 0x44, IF_CFG, CTRL4, INT1_CTRL, INT2_CTRL, CTRL7, CTRL8, CTRL6,
    /// CTRL1_XL_HG, CTRL1, CTRL2, then the 70 ms gyroscope turn-on) plus PIN_CTRL, which is always
    /// written because a software reset does not restore it (p57).
    pub fn plan(&self, apply: Apply) -> Result<Plan, ConfigError> {
        self.validate()?;
        let mut p = Plan::empty();

        // Always written, in both plans: BDU and IF_INC are what every multi-byte read relies on,
        // and `verify` compares them (CTRL3, p67). After a reset it equals the reset value 0x44.
        p.write(reg::CTRL3, self.ctrl3())?;
        // The self-test switches off in both plans, whatever a reset or an earlier program left:
        // ST_XL and ST_G, the other bits of CTRL10 are left alone. XL_HG_ST is in CTRL2_XL_HG.
        p.push(Step::Modify {
            reg: reg::CTRL10,
            mask: Self::CTRL10_SELF_TEST_MASK,
            value: 0,
        })?;

        if apply == Apply::InPlace {
            // Bypass empties the FIFO (6.5.2, p45); power-down for the writes below (p34, p69).
            p.write(reg::FIFO_CTRL4, 0)?;
            p.write(reg::CTRL1, 0)?;
            p.write(reg::CTRL2, 0)?;
            p.write(reg::CTRL1_XL_HG, 0)?;
        }

        // Always written, as `init` always has. IF_CFG is not reset by a software reset.
        p.write(reg::IF_CFG, self.if_cfg())?;
        // Not reset by a software reset (p57), so written whatever an earlier program left.
        p.write(reg::PIN_CTRL, self.pin_ctrl())?;
        p.write(reg::CTRL4, self.ctrl4())?;
        p.write(reg::INT1_CTRL, self.int1_ctrl())?;
        p.write(reg::INT2_CTRL, self.int2_ctrl())?;
        p.modify_unless_reset(apply, reg::MD2_CFG, Self::MD2_CFG_MASK, self.md2_cfg())?;
        p.modify_unless_reset(
            apply,
            reg::FUNCTIONS_ENABLE,
            Self::FUNCTIONS_ENABLE_MASK,
            self.functions_enable(),
        )?;
        // The generator is enabled last, after its threshold and routing are in place.
        p.write_unless_reset(apply, reg::HG_WAKE_UP_THS, self.hg_wake_up_ths(), 0)?;
        p.modify_unless_reset(
            apply,
            reg::INACTIVITY_THS,
            Self::INACTIVITY_THS_MASK,
            self.inactivity_ths(),
        )?;
        p.write_unless_reset(apply, reg::CTRL2_XL_HG, self.ctrl2_xl_hg(), 0)?;
        p.write_unless_reset(
            apply,
            reg::HG_FUNCTIONS_ENABLE,
            self.hg_functions_enable(),
            0,
        )?;

        p.write(reg::CTRL7, self.ctrl7())?;
        p.write(reg::CTRL8, self.ctrl8())?;
        p.write_unless_reset(apply, reg::CTRL9, self.ctrl9(), 0)?;
        p.write(reg::CTRL6, self.ctrl6())?;

        // The high-accuracy ODR mode is enabled while the sensors are in power-down (p34).
        p.write_unless_reset(apply, reg::HAODR_CFG, self.haodr_cfg(), 0)?;
        if self.mode.is_haodr() {
            p.write(reg::CTRL1, self.ctrl1_mode_only())?;
            p.write(reg::CTRL2, self.ctrl2_mode_only())?;
        }

        p.write_unless_reset(apply, reg::FIFO_CTRL1, self.fifo_ctrl1(), 0)?;
        p.write_unless_reset(apply, reg::COUNTER_BDR_REG1, self.counter_bdr_reg1(), 0)?;
        p.write_unless_reset(apply, reg::FIFO_CTRL2, self.fifo_ctrl2(), 0)?;
        p.write_unless_reset(apply, reg::FIFO_CTRL3, self.fifo_ctrl3(), 0)?;
        p.write_unless_reset(apply, reg::FIFO_CTRL4, self.fifo_ctrl4(), 0)?;

        p.write(reg::CTRL1_XL_HG, self.ctrl1_xl_hg())?;
        p.write(reg::CTRL1, self.ctrl1())?;
        p.write(reg::CTRL2, self.ctrl2())?;
        if self.gyro_odr != crate::config::Odr::Off {
            p.push(Step::DelayUs(GYRO_TURN_ON_US))?;
        }
        Ok(p)
    }
}

#[cfg(test)]
mod overflow_tests {
    use super::*;

    #[test]
    fn plan_push_past_capacity_is_an_error_and_keeps_the_steps() {
        let mut p = Plan::empty();
        for i in 0..MAX_STEPS {
            assert_eq!(p.write(reg::CTRL1, i as u8), Ok(()));
        }
        assert_eq!(p.len(), MAX_STEPS);
        assert_eq!(p.write(reg::CTRL2, 0xFF), Err(ConfigError::PlanOverflow));
        assert_eq!(p.push(Step::DelayUs(1)), Err(ConfigError::PlanOverflow));
        assert_eq!(p.len(), MAX_STEPS);
        assert_eq!(
            p.steps().last(),
            Some(&Step::Write {
                reg: reg::CTRL1,
                value: (MAX_STEPS - 1) as u8
            })
        );
    }
}

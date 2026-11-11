//! Register images: the value of each configuration register as a pure function of a [`Config`].
//!
//! Every image sets only the bits of fields that `Config` owns; the bits the datasheet requires to
//! be 0 stay 0, and CTRL6 bit 3 (which must be 1) is set. [`Config::validate`] decides whether the
//! combination is allowed; the images are total functions that do not check it.

use crate::calc::Offsets;
use crate::config::{Config, DataReadyMode, IntOutput, IntPolarity};
use crate::reg;

const fn bit(cond: bool, mask: u8) -> u8 {
    if cond { mask } else { 0 }
}

impl Config {
    /// CTRL3, the bits this crate owns (p67): BDU and IF_INC, both on. Every multi-byte read of the
    /// driver needs IF_INC, and BDU keeps the two bytes of a sample from the same conversion.
    pub const fn ctrl3(&self) -> u8 {
        reg::CTRL3_BDU | reg::CTRL3_IF_INC
    }

    /// The bits of CTRL3 that [`Self::ctrl3`] owns: BOOT (bit 7) and SW_RESET (bit 0) clear
    /// themselves, bits 5:3 and 1 are reserved and must be 0 (p67).
    pub const CTRL3_MASK: u8 = reg::CTRL3_BDU | reg::CTRL3_IF_INC;

    /// The bits of CTRL10 that the plan owns: ST_XL (1:0) and ST_G (3:2), the self-test switches
    /// of the low-g accelerometer and the gyroscope (p74). Their image is 0.
    pub const CTRL10_SELF_TEST_MASK: u8 = reg::CTRL10_ST_XL_MASK | reg::CTRL10_ST_G_MASK;

    /// IF_CFG (p58): SPI only (I2C and I3C disabled), plus the interrupt output type and level.
    pub const fn if_cfg(&self) -> u8 {
        reg::IF_CFG_I2C_I3C_DISABLE
            | bit(
                matches!(self.pins.output, IntOutput::OpenDrain),
                reg::IF_CFG_PP_OD,
            )
            | bit(
                matches!(self.pins.polarity, IntPolarity::ActiveLow),
                reg::IF_CFG_H_LACTIVE,
            )
    }

    /// PIN_CTRL (p57): IBHR_POR_EN kept at its reset value 1, and the pad drive strength.
    pub const fn pin_ctrl(&self) -> u8 {
        reg::PIN_CTRL_IBHR_POR_EN | (self.pins.drive.code() << reg::PIN_CTRL_IO_PAD_STRENGTH_SHIFT)
    }

    /// INT1_CTRL (p63).
    pub const fn int1_ctrl(&self) -> u8 {
        self.int1.data_ready.int_ctrl() | self.int1.fifo_ctrl_bits()
    }

    /// INT2_CTRL (p64).
    pub const fn int2_ctrl(&self) -> u8 {
        self.int2.data_ready.int_ctrl() | self.int2.fifo_ctrl_bits()
    }

    /// MD2_CFG, the bits this crate owns (p103): the timestamp overflow routing.
    pub const fn md2_cfg(&self) -> u8 {
        bit(self.int2.timestamp_overflow, reg::MD2_CFG_INT2_TIMESTAMP)
    }

    /// The bits of MD2_CFG that [`Self::md2_cfg`] owns.
    pub const MD2_CFG_MASK: u8 = reg::MD2_CFG_INT2_TIMESTAMP;

    /// CTRL1: the operating mode and the low-g accelerometer ODR (p65).
    pub const fn ctrl1(&self) -> u8 {
        (self.mode.xl_code() << reg::CTRL1_OP_MODE_XL_SHIFT) | self.accel_odr.code()
    }

    /// CTRL1 with the operating mode and the ODR in power-down: the mode is set while the sensor
    /// is off (6.1.4, p34).
    pub const fn ctrl1_mode_only(&self) -> u8 {
        self.mode.xl_code() << reg::CTRL1_OP_MODE_XL_SHIFT
    }

    /// CTRL2: the operating mode and the gyroscope ODR (p66).
    pub const fn ctrl2(&self) -> u8 {
        (self.mode.g_code() << reg::CTRL2_OP_MODE_G_SHIFT) | self.gyro_odr.code()
    }

    /// CTRL2 with the operating mode and the ODR in power-down.
    pub const fn ctrl2_mode_only(&self) -> u8 {
        self.mode.g_code() << reg::CTRL2_OP_MODE_G_SHIFT
    }

    /// CTRL4: pulsed or latched data-ready and DRDY_MASK (p68).
    pub const fn ctrl4(&self) -> u8 {
        bit(
            matches!(self.data_ready_mode, DataReadyMode::Pulsed),
            reg::CTRL4_DRDY_PULSED,
        ) | bit(self.drdy_mask, reg::CTRL4_DRDY_MASK)
    }

    /// CTRL6: the gyroscope full scale, LPF1 bandwidth and the mandatory bit 3 (p69).
    pub const fn ctrl6(&self) -> u8 {
        reg::CTRL6_MUST_BE_ONE
            | self.gyro_fs.code()
            | match self.gyro_lpf1 {
                Some(bw) => bw.code() << reg::CTRL6_LPF1_G_BW_SHIFT,
                None => 0,
            }
    }

    /// CTRL7: gyroscope LPF1 enable and the high-g data-ready routing (p70).
    pub const fn ctrl7(&self) -> u8 {
        bit(self.gyro_lpf1.is_some(), reg::CTRL7_LPF1_G_EN)
            | bit(self.int1.data_ready.high_g, reg::CTRL7_INT1_DRDY_XL_HG)
            | bit(self.int2.data_ready.high_g, reg::CTRL7_INT2_DRDY_XL_HG)
    }

    /// CTRL8: the low-g accelerometer full scale and LPF2 cutoff (p71).
    pub const fn ctrl8(&self) -> u8 {
        self.accel_fs.code()
            | match self.accel_lpf2 {
                Some(bw) => bw.code() << reg::CTRL8_HP_LPF2_XL_BW_SHIFT,
                None => 0,
            }
    }

    /// CTRL9: LPF2 enable, fast settling and the low-g user offset (p72).
    pub const fn ctrl9(&self) -> u8 {
        bit(self.accel_lpf2.is_some(), reg::CTRL9_LPF2_XL_EN)
            | bit(self.accel_fast_settling, reg::CTRL9_XL_FASTSETTL_MODE)
            | bit(self.accel_offset.is_some(), reg::CTRL9_USR_OFF_ON_OUT)
            | match self.accel_offset {
                Some(crate::config::OffsetWeight::Coarse) => reg::CTRL9_USR_OFF_W,
                _ => 0,
            }
    }

    /// CTRL1_XL_HG: output enabled with ODR, full scale and user offset, or 0 (power-down) when off
    /// (p91).
    pub const fn ctrl1_xl_hg(&self) -> u8 {
        match self.high_g {
            Some(h) => {
                reg::CTRL1_XL_HG_REGOUT_EN
                    | bit(h.user_offset, reg::CTRL1_XL_HG_HG_USR_OFF_ON_OUT)
                    | (h.odr.code() << reg::CTRL1_XL_HG_ODR_SHIFT)
                    | h.full_scale.code()
            }
            None => 0,
        }
    }

    /// CTRL2_XL_HG: the user offset on the high-g wake-up and shock data (p91).
    pub const fn ctrl2_xl_hg(&self) -> u8 {
        match self.high_g_events {
            Some(e) => bit(e.user_offset, reg::CTRL2_XL_HG_HG_USR_OFF_ON_WU),
            None => 0,
        }
    }

    /// FUNCTIONS_ENABLE, the bit this crate owns (p93): the timestamp counter.
    pub const fn functions_enable(&self) -> u8 {
        bit(self.timestamp, reg::FUNCTIONS_ENABLE_TIMESTAMP_EN)
    }

    /// The bits of FUNCTIONS_ENABLE that [`Self::functions_enable`] owns.
    pub const FUNCTIONS_ENABLE_MASK: u8 = reg::FUNCTIONS_ENABLE_TIMESTAMP_EN;

    /// HG_FUNCTIONS_ENABLE (p94): generator enable, wake-up type, routing and shock duration.
    pub const fn hg_functions_enable(&self) -> u8 {
        match self.high_g_events {
            Some(e) => {
                reg::HG_FUNCTIONS_ENABLE_HG_INTERRUPTS_ENABLE
                    | bit(
                        matches!(e.wake_up_mode, crate::config::HighGWakeUpMode::OnChange),
                        reg::HG_FUNCTIONS_ENABLE_HG_WU_CHANGE_INT_SEL,
                    )
                    | bit(
                        self.int2.high_g_wake_up,
                        reg::HG_FUNCTIONS_ENABLE_INT2_HG_WU,
                    )
                    | bit(
                        self.int1.high_g_wake_up,
                        reg::HG_FUNCTIONS_ENABLE_INT1_HG_WU,
                    )
                    | (e.shock_duration & reg::HG_FUNCTIONS_ENABLE_HG_SHOCK_DUR_MASK)
            }
            None => 0,
        }
    }

    /// HG_WAKE_UP_THS (p94).
    pub const fn hg_wake_up_ths(&self) -> u8 {
        match self.high_g_events {
            Some(e) => e.threshold,
            None => 0,
        }
    }

    /// INACTIVITY_THS, the bits this crate owns (p96): the high-g shock routing.
    pub const fn inactivity_ths(&self) -> u8 {
        bit(
            self.int1.high_g_shock,
            reg::INACTIVITY_THS_INT1_HG_SHOCK_CHANGE,
        ) | bit(
            self.int2.high_g_shock,
            reg::INACTIVITY_THS_INT2_HG_SHOCK_CHANGE,
        )
    }

    /// The bits of INACTIVITY_THS that [`Self::inactivity_ths`] owns. Bits 5:0 are the
    /// activity/inactivity threshold.
    pub const INACTIVITY_THS_MASK: u8 =
        reg::INACTIVITY_THS_INT1_HG_SHOCK_CHANGE | reg::INACTIVITY_THS_INT2_HG_SHOCK_CHANGE;

    /// HAODR_CFG: the ODR set of the high-accuracy ODR mode (p103).
    pub const fn haodr_cfg(&self) -> u8 {
        self.mode.haodr_sel() << reg::HAODR_CFG_HAODR_SEL_SHIFT
    }

    /// FIFO_CTRL1: the watermark (p59).
    pub const fn fifo_ctrl1(&self) -> u8 {
        self.fifo.watermark
    }

    /// FIFO_CTRL2: STOP_ON_WTM and ODR_CHG_EN; compression stays off (p59).
    pub const fn fifo_ctrl2(&self) -> u8 {
        bit(self.fifo.stop_on_wtm, reg::FIFO_CTRL2_STOP_ON_WTM)
            | bit(self.fifo.odr_change, reg::FIFO_CTRL2_ODR_CHG_EN)
    }

    /// FIFO_CTRL3: the gyroscope and low-g batch data rates (p60).
    pub const fn fifo_ctrl3(&self) -> u8 {
        (self.fifo.gyro.code() << reg::FIFO_CTRL3_BDR_GY_SHIFT)
            | (self.fifo.accel.code() << reg::FIFO_CTRL3_BDR_XL_SHIFT)
    }

    /// FIFO_CTRL4: timestamp decimation, temperature batch rate and the FIFO mode (p61).
    pub const fn fifo_ctrl4(&self) -> u8 {
        (self.fifo.timestamp.code() << reg::FIFO_CTRL4_DEC_TS_BATCH_SHIFT)
            | (self.fifo.temperature.code() << reg::FIFO_CTRL4_ODR_T_BATCH_SHIFT)
            | (self.fifo.mode.code() << reg::FIFO_CTRL4_FIFO_MODE_SHIFT)
    }

    /// COUNTER_BDR_REG1: high-g batching; the counter trigger and threshold stay 0 (p62).
    pub const fn counter_bdr_reg1(&self) -> u8 {
        bit(self.fifo.high_g, reg::COUNTER_BDR_REG1_XL_HG_BATCH_EN)
    }
}

/// A register the applied configuration owns: the bits `mask`, which hold `expected` (already
/// ANDed with `mask`) once the configuration is applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Owned {
    pub(crate) reg: u8,
    pub(crate) mask: u8,
    pub(crate) expected: u8,
}

/// How many registers [`Config::owned_registers`] lists, whatever the configuration.
pub(crate) const OWNED_COUNT: usize = 32;

impl Config {
    /// The registers the applied configuration owns, in the order `verify` reads them (the order
    /// of the plan), each with the bits to compare and their expected value. The values are the
    /// images above, the ones the plan writes; the list is the same after [`Apply::AfterReset`]
    /// and after [`Apply::InPlace`] (a register the after-reset plan skips holds its reset value 0,
    /// which is the image).
    ///
    /// A mask leaves out the bits the datasheet reserves ("must be set to 0", and no field of
    /// [`Config`] owns them): it does not say that they read back as written. A register the plan
    /// only modifies is owned under the plan's mask. No owned register has a bit that reports
    /// status. CTRL3 has two bits that clear themselves, SW_RESET and BOOT (p67): its mask leaves
    /// them out and compares BDU and IF_INC only.
    ///
    /// The last six are the user offset registers, against `offsets` (the driver's applied
    /// offsets; they are not part of the [`Config`] and no plan step writes them), all eight bits.
    ///
    /// [`Apply::AfterReset`]: crate::plan::Apply::AfterReset
    /// [`Apply::InPlace`]: crate::plan::Apply::InPlace
    pub(crate) const fn owned_registers(&self, offsets: &Offsets) -> [Owned; OWNED_COUNT] {
        const fn o(reg: u8, mask: u8, image: u8) -> Owned {
            Owned {
                reg,
                mask,
                expected: image & mask,
            }
        }
        [
            // BOOT and SW_RESET clear themselves, bits 5:3 and 1 reserved (p67).
            o(reg::CTRL3, Self::CTRL3_MASK, self.ctrl3()),
            // ST_XL and ST_G, the self-test switches: owned with the stimulus off (p74).
            o(reg::CTRL10, Self::CTRL10_SELF_TEST_MASK, 0),
            // Bit 1 reserved (p58).
            o(reg::IF_CFG, 0xFD, self.if_cfg()),
            // Bits 4:2 reserved (p57).
            o(reg::PIN_CTRL, 0xE3, self.pin_ctrl()),
            // Bits 7:5 reserved (p68).
            o(reg::CTRL4, 0x1F, self.ctrl4()),
            // Bits 7 and 2 reserved (p63).
            o(reg::INT1_CTRL, 0x7B, self.int1_ctrl()),
            o(reg::INT2_CTRL, 0xFF, self.int2_ctrl()),
            o(reg::MD2_CFG, Self::MD2_CFG_MASK, self.md2_cfg()),
            o(
                reg::FUNCTIONS_ENABLE,
                Self::FUNCTIONS_ENABLE_MASK,
                self.functions_enable(),
            ),
            o(reg::HG_WAKE_UP_THS, 0xFF, self.hg_wake_up_ths()),
            o(
                reg::INACTIVITY_THS,
                Self::INACTIVITY_THS_MASK,
                self.inactivity_ths(),
            ),
            // Bits 7:5 and 3:2 reserved (p91); XL_HG_ST (1:0) is in the mask and its image is 0.
            o(reg::CTRL2_XL_HG, 0x13, self.ctrl2_xl_hg()),
            o(reg::HG_FUNCTIONS_ENABLE, 0xFF, self.hg_functions_enable()),
            // Bits 5:1 reserved (p70).
            o(reg::CTRL7, 0xC1, self.ctrl7()),
            // Bits 4:2 reserved (p71).
            o(reg::CTRL8, 0xE3, self.ctrl8()),
            // Bits 7 and 2 reserved (p72).
            o(reg::CTRL9, 0x7B, self.ctrl9()),
            // Bit 7 reserved; bit 3 reads 1 (p69).
            o(reg::CTRL6, 0x7F, self.ctrl6()),
            // Bits 7:2 reserved (p103).
            o(reg::HAODR_CFG, 0x03, self.haodr_cfg()),
            o(reg::FIFO_CTRL1, 0xFF, self.fifo_ctrl1()),
            // Bits 7, 4 and 2 reserved (p62).
            o(reg::COUNTER_BDR_REG1, 0x6B, self.counter_bdr_reg1()),
            // Bits 5, 3 and 0 reserved (p59).
            o(reg::FIFO_CTRL2, 0xD6, self.fifo_ctrl2()),
            o(reg::FIFO_CTRL3, 0xFF, self.fifo_ctrl3()),
            o(reg::FIFO_CTRL4, 0xFF, self.fifo_ctrl4()),
            o(reg::CTRL1_XL_HG, 0xFF, self.ctrl1_xl_hg()),
            // Bit 7 reserved (p65, p66).
            o(reg::CTRL1, 0x7F, self.ctrl1()),
            o(reg::CTRL2, 0x7F, self.ctrl2()),
            o(reg::XL_HG_X_OFS_USR, 0xFF, offsets.high_g[0] as u8),
            o(reg::XL_HG_Y_OFS_USR, 0xFF, offsets.high_g[1] as u8),
            o(reg::XL_HG_Z_OFS_USR, 0xFF, offsets.high_g[2] as u8),
            o(reg::X_OFS_USR, 0xFF, offsets.low_g[0] as u8),
            o(reg::Y_OFS_USR, 0xFF, offsets.low_g[1] as u8),
            o(reg::Z_OFS_USR, 0xFF, offsets.low_g[2] as u8),
        ]
    }
}

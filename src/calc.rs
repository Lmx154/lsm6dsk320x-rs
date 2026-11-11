//! Pure helpers with no bus access: the real output and timestamp rates, the self-test limits,
//! the self-test register images, and the user-offset registers with their conversions.
//!
//! Rates are integers (millihertz, nanoseconds). The offset and threshold conversions use integers
//! too; none of the functions here uses floating point.

use crate::config::{Channel, ConfigError, HighGFullScale, Odr, OffsetWeight};
use crate::reg;

const PPM_DENOMINATOR: u64 = 1_000_000;

/// `1 + 0.0013 * freq_fine` in parts per million (9.54, p92). Always positive for an `i8`.
const fn freq_fine_factor_ppm(freq_fine: i8) -> u64 {
    // bound: freq_fine >= -128, so the factor is at least 1_000_000 - 128 * 1300 = 833_600.
    (PPM_DENOMINATOR as i64 + freq_fine as i64 * reg::INTERNAL_FREQ_FINE_STEP_PPM as i64) as u64
}

/// ODRcoeff of an ODR (Table 153, p92); `None` for power-down.
pub const fn odr_coeff(odr: Odr) -> Option<u32> {
    match odr {
        Odr::Off => None,
        Odr::Hz7_5 => Some(1024),
        Odr::Hz15 => Some(512),
        Odr::Hz30 => Some(256),
        Odr::Hz60 => Some(128),
        Odr::Hz120 => Some(64),
        Odr::Hz240 => Some(32),
        Odr::Hz480 => Some(16),
        Odr::Hz960 => Some(8),
        Odr::Hz1920 => Some(4),
        Odr::Hz3840 => Some(2),
        Odr::Hz7680 => Some(1),
    }
}

/// Scales a nominal rate by `1 + 0.0013 * freq_fine`, rounded to the nearest millihertz.
///
/// `freq_fine` is the signed INTERNAL_FREQ_FINE value (9.54, p92).
pub const fn scale_rate_mhz(nominal_mhz: u32, freq_fine: i8) -> u32 {
    let scaled = nominal_mhz as u64 * freq_fine_factor_ppm(freq_fine);
    // bound: nominal <= 8_000_000 mHz and the factor <= 1_166_400 ppm, so the quotient fits u32.
    ((scaled + PPM_DENOMINATOR / 2) / PPM_DENOMINATOR) as u32
}

/// The real output data rate in millihertz for a nominal ODR and the INTERNAL_FREQ_FINE value:
/// `ODR_actual = 7680 Hz * (1 + 0.0013 * FREQ_FINE) / ODRcoeff` (9.54, p92). 0 for power-down.
///
/// The formula is the datasheet's for the standard rates. In the high-accuracy ODR mode the trim
/// does not apply: measured on the MARV V2 board, 2026-11-11, the rates followed the table values
/// of [`Odr::haodr_mhz`] to within 0.3 %, not the trim; [`Info`](crate::Info) reports the table
/// value there.
pub const fn real_odr_mhz(odr: Odr, freq_fine: i8) -> u32 {
    match odr_coeff(odr) {
        // The coefficients are powers of two up to 1024 and divide 7_680_000 mHz exactly.
        Some(coeff) => scale_rate_mhz(7_680_000 / coeff, freq_fine),
        None => 0,
    }
}

/// The real timestamp tick in nanoseconds for the INTERNAL_FREQ_FINE value:
/// `t_actual = 1 / (46080 Hz * (1 + 0.0013 * FREQ_FINE))` (9.54, p92), rounded to the nearest
/// nanosecond. 21 701 ns at `freq_fine = 0` (the datasheet's 21.7 us, p85).
pub const fn timestamp_tick_ns(freq_fine: i8) -> u32 {
    // 1e9 ns * 1e6 ppm / (46080 * factor_ppm)
    let denominator = 46_080 * freq_fine_factor_ppm(freq_fine);
    // bound: the denominator is at least 3.8e10, so the quotient is at most 2.7e4.
    ((1_000_000_000u64 * PPM_DENOMINATOR + denominator / 2) / denominator) as u32
}

/// An inclusive range of a self-test output change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestRange {
    /// Smallest allowed change.
    pub min: u32,
    /// Largest allowed change.
    pub max: u32,
}

impl SelfTestRange {
    /// True when `value` is inside the range.
    pub const fn contains(&self, value: u32) -> bool {
        value >= self.min && value <= self.max
    }
}

/// Low-g self-test output change, in mg, full-scale independent (Table 3, p13, notes 11-13, p14).
pub const SELF_TEST_LOW_G_MG: SelfTestRange = SelfTestRange { min: 50, max: 1700 };

/// High-g self-test output change, in mg, full-scale independent (Table 3, p13, notes 11-13).
pub const SELF_TEST_HIGH_G_MG: SelfTestRange = SelfTestRange {
    min: 150,
    max: 2500,
};

/// Gyroscope self-test output change at +-250 dps, in mdps (Table 3, p13, notes 14-15, p14).
pub const SELF_TEST_GYRO_250_DPS_MDPS: SelfTestRange = SelfTestRange {
    min: 20_000,
    max: 80_000,
};

/// Gyroscope self-test output change at +-4000 dps, in mdps (Table 3, p13, notes 14-15, p14).
pub const SELF_TEST_GYRO_4000_DPS_MDPS: SelfTestRange = SelfTestRange {
    min: 150_000,
    max: 700_000,
};

/// Self-test sign, the codes of ST_XL, ST_G and XL_HG_ST (p74, p91).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SelfTestSign {
    /// Normal mode.
    #[default]
    Off,
    /// Positive sign self-test.
    Positive,
    /// Negative sign self-test.
    Negative,
}

impl SelfTestSign {
    /// The 2-bit code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Off => reg::SELF_TEST_OFF,
            Self::Positive => reg::SELF_TEST_POSITIVE,
            Self::Negative => reg::SELF_TEST_NEGATIVE,
        }
    }
}

/// CTRL10 with the low-g and gyroscope self-test selected (9.23, Tables 74-75, p74).
pub const fn ctrl10_self_test(low_g: SelfTestSign, gyro: SelfTestSign) -> u8 {
    (low_g.code() << reg::CTRL10_ST_XL_SHIFT) | (gyro.code() << reg::CTRL10_ST_G_SHIFT)
}

/// The self-test field of CTRL2_XL_HG (9.52, Tables 145-146, p91). The other bit of that register,
/// HG_USR_OFF_ON_WU, is left 0; use [`crate::Config::ctrl2_xl_hg`] and OR the two to keep it.
pub const fn ctrl2_xl_hg_self_test(high_g: SelfTestSign) -> u8 {
    high_g.code() << reg::CTRL2_XL_HG_ST_SHIFT
}

/// Low-g user offset to micro-g: `raw * weight`, rounded to the nearest micro-g, halves away from
/// zero (Tables 225-229, p113; CTRL9 USR_OFF_W, p72).
pub const fn low_g_offset_ug(raw: i8, weight: OffsetWeight) -> i32 {
    // 2^-10 g = 976.5625 ug = 15625/16; 2^-6 g = 15625 ug.
    let r = raw as i32;
    match weight {
        OffsetWeight::Fine => {
            let n = r * 15_625;
            if n >= 0 { (n + 8) / 16 } else { (n - 8) / 16 }
        }
        OffsetWeight::Coarse => r * 15_625,
    }
}

/// High-g user offset to mg: 250 mg per LSB up to +-256 g full scale, 330 mg per LSB at +-320 g
/// (Tables 209-213, p108).
pub const fn high_g_offset_mg(raw: i8, full_scale: HighGFullScale) -> i32 {
    let step = match full_scale {
        HighGFullScale::G320 => 330,
        HighGFullScale::G32 | HighGFullScale::G64 | HighGFullScale::G128 | HighGFullScale::G256 => {
            250
        }
    };
    raw as i32 * step
}

/// High-g wake-up threshold to mg: 1 g per LSB up to +-256 g full scale, 1.25 g per LSB at
/// +-320 g (Table 159, p94).
pub const fn hg_wake_up_threshold_mg(raw: u8, full_scale: HighGFullScale) -> u32 {
    let step = match full_scale {
        HighGFullScale::G320 => 1250,
        HighGFullScale::G32 | HighGFullScale::G64 | HighGFullScale::G128 | HighGFullScale::G256 => {
            1000
        }
    };
    raw as u32 * step
}

/// The user offset registers, raw LSB per axis in the order X, Y, Z.
///
/// The low-g weight is set by [`crate::Config::accel_offset`], the high-g weight by the full
/// scale; convert with [`low_g_offset_ug`] and [`high_g_offset_mg`]. Each value must be in
/// -127..=127 (p108, p113). Whether an offset is applied is set in the [`crate::Config`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Offsets {
    /// X_OFS_USR, Y_OFS_USR, Z_OFS_USR (0x73 to 0x75).
    pub low_g: [i8; 3],
    /// XL_HG_X_OFS_USR, XL_HG_Y_OFS_USR, XL_HG_Z_OFS_USR (0x6C to 0x6E).
    pub high_g: [i8; 3],
}

impl Offsets {
    /// All zero (the reset state).
    pub const fn new() -> Self {
        Self {
            low_g: [0; 3],
            high_g: [0; 3],
        }
    }

    /// Sets [`Self::low_g`].
    #[must_use]
    pub const fn with_low_g(mut self, v: [i8; 3]) -> Self {
        self.low_g = v;
        self
    }

    /// Sets [`Self::high_g`].
    #[must_use]
    pub const fn with_high_g(mut self, v: [i8; 3]) -> Self {
        self.high_g = v;
        self
    }

    /// Rejects the value -128, which the registers do not accept (p108, p113).
    pub fn validate(&self) -> Result<(), ConfigError> {
        let bad = |v: &[i8; 3]| v.iter().any(|&x| x < reg::OFS_USR_MIN);
        if bad(&self.low_g) {
            return Err(ConfigError::OffsetOutOfRange {
                channel: Channel::LowG,
            });
        }
        if bad(&self.high_g) {
            return Err(ConfigError::OffsetOutOfRange {
                channel: Channel::HighG,
            });
        }
        Ok(())
    }

    /// The three bytes for X_OFS_USR (0x73), Y_OFS_USR, Z_OFS_USR, written as one burst.
    pub const fn low_g_bytes(&self) -> [u8; 3] {
        let [x, y, z] = self.low_g;
        [x as u8, y as u8, z as u8]
    }

    /// The three bytes for XL_HG_X_OFS_USR (0x6C), XL_HG_Y_OFS_USR, XL_HG_Z_OFS_USR.
    pub const fn high_g_bytes(&self) -> [u8; 3] {
        let [x, y, z] = self.high_g;
        [x as u8, y as u8, z as u8]
    }
}

//! Typed configuration: the `Config` family, its validation, and the code tables of the fields.
//!
//! Everything here is plain code with no bus access. The register images built from a [`Config`]
//! are in `image.rs` (methods of [`Config`]), the register write plan is in [`crate::plan`].
//!
//! Every settings struct is `#[non_exhaustive]` with public fields, `Default`, a `const fn new()`
//! equal to the default and a `const fn with_<field>(self, v) -> Self` per field. Build one with
//! `Config::new().with_accel_odr(..)`. Enums that copy a complete datasheet table are exhaustive.

use crate::reg;

/// Defines `const fn with_<field>(self, v) -> Self` setters.
macro_rules! setters {
    ($($(#[$doc:meta])* $field:ident, $name:ident: $ty:ty;)*) => {
        $(
            $(#[$doc])*
            #[must_use]
            pub const fn $name(mut self, v: $ty) -> Self {
                self.$field = v;
                self
            }
        )*
    };
}

/// Low-g accelerometer full scale, CTRL8 FS_XL (p71).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelFullScale {
    /// +-2 g.
    G2,
    /// +-4 g.
    G4,
    /// +-8 g.
    G8,
    /// +-16 g.
    G16,
}

impl AccelFullScale {
    /// FS_XL code.
    pub const fn code(self) -> u8 {
        match self {
            Self::G2 => reg::FS_XL_2_G,
            Self::G4 => reg::FS_XL_4_G,
            Self::G8 => reg::FS_XL_8_G,
            Self::G16 => reg::FS_XL_16_G,
        }
    }

    /// Sensitivity in mg per LSB (Table 3, p12).
    pub const fn mg_per_lsb(self) -> f32 {
        match self {
            Self::G2 => 0.061,
            Self::G4 => 0.122,
            Self::G8 => 0.244,
            Self::G16 => 0.488,
        }
    }
}

/// Gyroscope full scale, CTRL6 FS_G (p69).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroFullScale {
    /// +-250 dps.
    Dps250,
    /// +-500 dps.
    Dps500,
    /// +-1000 dps.
    Dps1000,
    /// +-2000 dps.
    Dps2000,
    /// +-4000 dps.
    Dps4000,
}

impl GyroFullScale {
    /// FS_G code. The reset value 000 is reserved, so this is always written.
    pub const fn code(self) -> u8 {
        match self {
            Self::Dps250 => reg::FS_G_250_DPS,
            Self::Dps500 => reg::FS_G_500_DPS,
            Self::Dps1000 => reg::FS_G_1000_DPS,
            Self::Dps2000 => reg::FS_G_2000_DPS,
            Self::Dps4000 => reg::FS_G_4000_DPS,
        }
    }

    /// Sensitivity in mdps per LSB (Table 3, p12).
    pub const fn mdps_per_lsb(self) -> f32 {
        match self {
            Self::Dps250 => 8.75,
            Self::Dps500 => 17.5,
            Self::Dps1000 => 35.0,
            Self::Dps2000 => 70.0,
            Self::Dps4000 => 140.0,
        }
    }
}

/// High-g accelerometer full scale, CTRL1_XL_HG FS_XL_HG (p91, Table 150).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HighGFullScale {
    /// +-32 g.
    G32,
    /// +-64 g.
    G64,
    /// +-128 g.
    G128,
    /// +-256 g.
    G256,
    /// +-320 g.
    G320,
}

impl HighGFullScale {
    /// FS_XL_HG code.
    pub const fn code(self) -> u8 {
        match self {
            Self::G32 => reg::FS_XL_HG_32_G,
            Self::G64 => reg::FS_XL_HG_64_G,
            Self::G128 => reg::FS_XL_HG_128_G,
            Self::G256 => reg::FS_XL_HG_256_G,
            Self::G320 => reg::FS_XL_HG_320_G,
        }
    }

    /// Sensitivity in mg per LSB (Table 3, p12).
    pub const fn mg_per_lsb(self) -> f32 {
        match self {
            Self::G32 => 0.976,
            Self::G64 => 1.952,
            Self::G128 => 3.904,
            Self::G256 => 7.808,
            Self::G320 => 10.417,
        }
    }
}

/// Output data rate of the low-g accelerometer and of the gyroscope in high-performance mode
/// (Tables 54 and 57, p65-66). Both use the same codes.
///
/// The accelerometer's 1.875 Hz code is low-power only and is not offered here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Odr {
    /// Power-down.
    Off,
    /// 7.5 Hz.
    Hz7_5,
    /// 15 Hz.
    Hz15,
    /// 30 Hz.
    Hz30,
    /// 60 Hz.
    Hz60,
    /// 120 Hz.
    Hz120,
    /// 240 Hz.
    Hz240,
    /// 480 Hz.
    Hz480,
    /// 960 Hz.
    Hz960,
    /// 1.92 kHz.
    Hz1920,
    /// 3.84 kHz.
    Hz3840,
    /// 7.68 kHz.
    Hz7680,
}

impl Odr {
    /// ODR_XL / ODR_G code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Off => reg::ODR_POWER_DOWN,
            Self::Hz7_5 => reg::ODR_7_5_HZ,
            Self::Hz15 => reg::ODR_15_HZ,
            Self::Hz30 => reg::ODR_30_HZ,
            Self::Hz60 => reg::ODR_60_HZ,
            Self::Hz120 => reg::ODR_120_HZ,
            Self::Hz240 => reg::ODR_240_HZ,
            Self::Hz480 => reg::ODR_480_HZ,
            Self::Hz960 => reg::ODR_960_HZ,
            Self::Hz1920 => reg::ODR_1920_HZ,
            Self::Hz3840 => reg::ODR_3840_HZ,
            Self::Hz7680 => reg::ODR_7680_HZ,
        }
    }

    /// Nominal rate in hertz (0 for `Off`).
    pub const fn hz(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Hz7_5 => 7.5,
            Self::Hz15 => 15.0,
            Self::Hz30 => 30.0,
            Self::Hz60 => 60.0,
            Self::Hz120 => 120.0,
            Self::Hz240 => 240.0,
            Self::Hz480 => 480.0,
            Self::Hz960 => 960.0,
            Self::Hz1920 => 1920.0,
            Self::Hz3840 => 3840.0,
            Self::Hz7680 => 7680.0,
        }
    }

    /// Nominal rate in millihertz (0 for `Off`), outside the high-accuracy ODR mode.
    pub const fn nominal_mhz(self) -> u32 {
        match self {
            Self::Off => 0,
            Self::Hz7_5 => 7_500,
            Self::Hz15 => 15_000,
            Self::Hz30 => 30_000,
            Self::Hz60 => 60_000,
            Self::Hz120 => 120_000,
            Self::Hz240 => 240_000,
            Self::Hz480 => 480_000,
            Self::Hz960 => 960_000,
            Self::Hz1920 => 1_920_000,
            Self::Hz3840 => 3_840_000,
            Self::Hz7680 => 7_680_000,
        }
    }

    /// Nominal rate in millihertz in the high-accuracy ODR mode with the ODR set `set` (Table 22,
    /// p34). `None` for `Off`, which is power-down, and for 7.5 Hz, whose code is reserved there.
    /// The table rounds some rates (13 Hz for 12.5/0.96, 417 Hz and so on): the table value is
    /// returned.
    pub const fn haodr_mhz(self, set: HaodrSet) -> Option<u32> {
        let row: [u32; 4] = match self {
            Self::Off | Self::Hz7_5 => return None,
            Self::Hz15 => [15_000, 15_625, 12_500, 13_000],
            Self::Hz30 => [30_000, 31_250, 25_000, 26_000],
            Self::Hz60 => [60_000, 62_500, 50_000, 52_000],
            Self::Hz120 => [120_000, 125_000, 100_000, 104_000],
            Self::Hz240 => [240_000, 250_000, 200_000, 208_000],
            Self::Hz480 => [480_000, 500_000, 400_000, 417_000],
            Self::Hz960 => [960_000, 1_000_000, 800_000, 833_000],
            Self::Hz1920 => [1_920_000, 2_000_000, 1_600_000, 1_667_000],
            Self::Hz3840 => [3_840_000, 4_000_000, 3_200_000, 3_333_000],
            Self::Hz7680 => [7_680_000, 8_000_000, 6_400_000, 6_667_000],
        };
        let [a, b, c, d] = row;
        Some(match set {
            HaodrSet::Base960 => a,
            HaodrSet::Base1000 => b,
            HaodrSet::Base800 => c,
            HaodrSet::Base833 => d,
        })
    }
}

/// Output data rate of the high-g accelerometer (CTRL1_XL_HG, Table 149, p91).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HighGOdr {
    /// 480 Hz.
    Hz480,
    /// 960 Hz.
    Hz960,
    /// 1.92 kHz.
    Hz1920,
    /// 3.84 kHz.
    Hz3840,
    /// 7.68 kHz.
    Hz7680,
}

impl HighGOdr {
    /// ODR_XL_HG code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Hz480 => reg::ODR_XL_HG_480_HZ,
            Self::Hz960 => reg::ODR_XL_HG_960_HZ,
            Self::Hz1920 => reg::ODR_XL_HG_1920_HZ,
            Self::Hz3840 => reg::ODR_XL_HG_3840_HZ,
            Self::Hz7680 => reg::ODR_XL_HG_7680_HZ,
        }
    }

    /// Nominal rate in hertz.
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz480 => 480.0,
            Self::Hz960 => 960.0,
            Self::Hz1920 => 1920.0,
            Self::Hz3840 => 3840.0,
            Self::Hz7680 => 7680.0,
        }
    }

    /// Nominal rate in millihertz.
    pub const fn nominal_mhz(self) -> u32 {
        match self {
            Self::Hz480 => 480_000,
            Self::Hz960 => 960_000,
            Self::Hz1920 => 1_920_000,
            Self::Hz3840 => 3_840_000,
            Self::Hz7680 => 7_680_000,
        }
    }

    /// The rate in millihertz in the high-accuracy ODR mode with the ODR set `set`: the table
    /// rate (Table 22, p34) of the standard rate of the same name, measured to follow the set on
    /// the MARV V2 board, 2026-11-11 (`hil/logs/lsm6dsk320x_dual-20261111-184219.log`).
    pub(crate) const fn haodr_mhz(self, set: HaodrSet) -> u32 {
        let odr = match self {
            Self::Hz480 => Odr::Hz480,
            Self::Hz960 => Odr::Hz960,
            Self::Hz1920 => Odr::Hz1920,
            Self::Hz3840 => Odr::Hz3840,
            Self::Hz7680 => Odr::Hz7680,
        };
        match odr.haodr_mhz(set) {
            Some(mhz) => mhz,
            None => self.nominal_mhz(),
        }
    }
}

/// High-g channel settings. `Config::high_g` is `None` to leave the channel off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct HighGConfig {
    /// Output data rate.
    pub odr: HighGOdr,
    /// Full scale.
    pub full_scale: HighGFullScale,
    /// Applies the high-g user offset registers to the output (CTRL1_XL_HG HG_USR_OFF_ON_OUT,
    /// p91). The values are set with [`crate::calc::Offsets`].
    pub user_offset: bool,
}

impl HighGConfig {
    /// 960 Hz, +-32 g, no user offset.
    pub const fn new() -> Self {
        Self {
            odr: HighGOdr::Hz960,
            full_scale: HighGFullScale::G32,
            user_offset: false,
        }
    }

    setters! {
        /// Sets [`Self::odr`].
        odr, with_odr: HighGOdr;
        /// Sets [`Self::full_scale`].
        full_scale, with_full_scale: HighGFullScale;
        /// Sets [`Self::user_offset`].
        user_offset, with_user_offset: bool;
    }
}

impl Default for HighGConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Which data-ready signals drive one interrupt pin (they are ORed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct DataReady {
    /// Low-g accelerometer.
    pub accel: bool,
    /// Gyroscope.
    pub gyro: bool,
    /// High-g accelerometer.
    pub high_g: bool,
}

impl DataReady {
    /// No data-ready signal.
    pub const NONE: Self = Self::new();

    /// No data-ready signal.
    pub const fn new() -> Self {
        Self {
            accel: false,
            gyro: false,
            high_g: false,
        }
    }

    setters! {
        /// Sets [`Self::accel`].
        accel, with_accel: bool;
        /// Sets [`Self::gyro`].
        gyro, with_gyro: bool;
        /// Sets [`Self::high_g`].
        high_g, with_high_g: bool;
    }

    /// INT1_CTRL / INT2_CTRL value (the low-g and gyro bits share positions on both pins).
    pub const fn int_ctrl(self) -> u8 {
        (if self.accel { reg::INT_DRDY_XL } else { 0 })
            | (if self.gyro { reg::INT_DRDY_G } else { 0 })
    }
}

/// Everything that drives one interrupt pin. The signals are ORed on the pin (p63-64).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct IntRouting {
    /// Data-ready per channel.
    pub data_ready: DataReady,
    /// FIFO watermark flag (FIFO_WTM_IA).
    pub fifo_threshold: bool,
    /// FIFO overrun.
    pub fifo_overrun: bool,
    /// FIFO full at the next ODR (FIFO_FULL_IA).
    pub fifo_full: bool,
    /// High-g wake-up event (HG_FUNCTIONS_ENABLE INTx_HG_WU, p94).
    pub high_g_wake_up: bool,
    /// High-g shock event (INACTIVITY_THS INTx_HG_SHOCK_CHANGE, p96).
    pub high_g_shock: bool,
    /// Timestamp overflow alert, 5.6 ms ahead (MD2_CFG INT2_TIMESTAMP, p103). INT2 only: the
    /// device has no such bit for INT1, and [`Config::validate`] rejects it there.
    pub timestamp_overflow: bool,
}

impl IntRouting {
    /// Nothing routed.
    pub const fn new() -> Self {
        Self {
            data_ready: DataReady::new(),
            fifo_threshold: false,
            fifo_overrun: false,
            fifo_full: false,
            high_g_wake_up: false,
            high_g_shock: false,
            timestamp_overflow: false,
        }
    }

    setters! {
        /// Sets [`Self::data_ready`].
        data_ready, with_data_ready: DataReady;
        /// Sets [`Self::fifo_threshold`].
        fifo_threshold, with_fifo_threshold: bool;
        /// Sets [`Self::fifo_overrun`].
        fifo_overrun, with_fifo_overrun: bool;
        /// Sets [`Self::fifo_full`].
        fifo_full, with_fifo_full: bool;
        /// Sets [`Self::high_g_wake_up`].
        high_g_wake_up, with_high_g_wake_up: bool;
        /// Sets [`Self::high_g_shock`].
        high_g_shock, with_high_g_shock: bool;
        /// Sets [`Self::timestamp_overflow`].
        timestamp_overflow, with_timestamp_overflow: bool;
    }

    pub(crate) const fn fifo_ctrl_bits(&self) -> u8 {
        (if self.fifo_threshold {
            reg::INT_FIFO_TH
        } else {
            0
        }) | (if self.fifo_overrun {
            reg::INT_FIFO_OVR
        } else {
            0
        }) | (if self.fifo_full {
            reg::INT_FIFO_FULL
        } else {
            0
        })
    }

    pub(crate) const fn any_fifo(&self) -> bool {
        self.fifo_threshold || self.fifo_overrun || self.fifo_full
    }

    pub(crate) const fn any_high_g_event(&self) -> bool {
        self.high_g_wake_up || self.high_g_shock
    }
}

/// One ODR set of the high-accuracy ODR mode, HAODR_CFG HAODR_SEL (Table 22, p34, p103).
///
/// The names give the rate that ODR code 1001 (nominally 960 Hz) becomes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HaodrSet {
    /// HAODR_SEL = 00: 15, 30, ... 7680 Hz, the same as the standard rates.
    Base960,
    /// HAODR_SEL = 01: 15.625, 31.25, ... 8000 Hz.
    Base1000,
    /// HAODR_SEL = 10: 12.5, 25, ... 6400 Hz.
    Base800,
    /// HAODR_SEL = 11: 13, 26, ... 6667 Hz.
    Base833,
}

impl HaodrSet {
    /// HAODR_SEL code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Base960 => 0b00,
            Self::Base1000 => 0b01,
            Self::Base800 => 0b10,
            Self::Base833 => 0b11,
        }
    }
}

/// Operating mode of the low-g accelerometer, the gyroscope and the high-g channel (6.1, p33-34).
///
/// One mode applies to the low-g accelerometer and the gyroscope together: in the high-accuracy ODR
/// mode, "when this mode is used for one sensor, the other sensor also has to be configured in
/// HAODR mode" (6.1.4, p34).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum OperatingMode {
    /// OP_MODE = 000, valid for all ODRs from 7.5 Hz to 7.68 kHz.
    #[default]
    HighPerformance,
    /// OP_MODE = 001 with the given ODR set: ODRs from 15 Hz up, rate variation over temperature
    /// and supply +-1 % (gyroscope on) or +-3 % (gyroscope off) (Table 3, p13).
    HighAccuracyOdr(HaodrSet),
}

impl OperatingMode {
    /// OP_MODE_XL code (Table 53, p65).
    pub const fn xl_code(self) -> u8 {
        match self {
            Self::HighPerformance => reg::OP_MODE_XL_HIGH_PERFORMANCE,
            Self::HighAccuracyOdr(_) => reg::OP_MODE_XL_HIGH_ACCURACY_ODR,
        }
    }

    /// OP_MODE_G code (Table 56, p66).
    pub const fn g_code(self) -> u8 {
        match self {
            Self::HighPerformance => reg::OP_MODE_G_HIGH_PERFORMANCE,
            Self::HighAccuracyOdr(_) => reg::OP_MODE_G_HIGH_ACCURACY_ODR,
        }
    }

    /// HAODR_SEL code; 0 (the reset value) outside the high-accuracy ODR mode.
    pub const fn haodr_sel(self) -> u8 {
        match self {
            Self::HighPerformance => 0,
            Self::HighAccuracyOdr(set) => set.code(),
        }
    }

    /// True for the high-accuracy ODR mode.
    pub const fn is_haodr(self) -> bool {
        matches!(self, Self::HighAccuracyOdr(_))
    }
}

/// Gyroscope LPF1 bandwidth selection, CTRL6 LPF1_G_BW (Table 66, p70). The cutoff depends on the
/// gyroscope ODR; see [`Self::cutoff_dhz`]. Available in high-performance and high-accuracy ODR
/// modes (Figure 21 note 2, p38).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroLpf1Bandwidth {
    /// LPF1_G_BW = 000.
    Bw0,
    /// LPF1_G_BW = 001.
    Bw1,
    /// LPF1_G_BW = 010.
    Bw2,
    /// LPF1_G_BW = 011.
    Bw3,
    /// LPF1_G_BW = 100.
    Bw4,
    /// LPF1_G_BW = 101.
    Bw5,
    /// LPF1_G_BW = 110.
    Bw6,
    /// LPF1_G_BW = 111.
    Bw7,
}

impl GyroLpf1Bandwidth {
    /// LPF1_G_BW code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Bw0 => 0b000,
            Self::Bw1 => 0b001,
            Self::Bw2 => 0b010,
            Self::Bw3 => 0b011,
            Self::Bw4 => 0b100,
            Self::Bw5 => 0b101,
            Self::Bw6 => 0b110,
            Self::Bw7 => 0b111,
        }
    }

    /// Cutoff of the LPF1 + LPF2 chain in tenths of hertz (Table 66, p70). `None` for the ODRs the
    /// table does not list (below 60 Hz and power-down).
    pub const fn cutoff_dhz(self, odr: Odr) -> Option<u32> {
        let row: [u32; 8] = match self {
            Self::Bw0 => [246, 494, 960, 1750, 2410, 2730, 2800, 2810],
            Self::Bw1 => [246, 494, 960, 1570, 1950, 2100, 2130, 2130],
            Self::Bw2 => [246, 494, 960, 1310, 1490, 1550, 1560, 1560],
            Self::Bw3 => [246, 494, 960, 1880, 3100, 3870, 4030, 4070],
            Self::Bw4 => [246, 494, 784, 940, 1000, 1010, 1020, 1020],
            Self::Bw5 => [246, 426, 530, 567, 579, 582, 583, 580],
            Self::Bw6 => [180, 242, 273, 284, 287, 288, 288, 288],
            Self::Bw7 => [121, 136, 142, 143, 144, 144, 144, 144],
        };
        let [a, b, c, d, e, f, g, h] = row;
        match odr {
            Odr::Hz60 => Some(a),
            Odr::Hz120 => Some(b),
            Odr::Hz240 => Some(c),
            Odr::Hz480 => Some(d),
            Odr::Hz960 => Some(e),
            Odr::Hz1920 => Some(f),
            Odr::Hz3840 => Some(g),
            Odr::Hz7680 => Some(h),
            Odr::Off | Odr::Hz7_5 | Odr::Hz15 | Odr::Hz30 => None,
        }
    }
}

/// Low-g accelerometer LPF2 cutoff, CTRL8 HP_LPF2_XL_BW with LPF2_XL_EN = 1 (Table 71, p71).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelLpf2Bandwidth {
    /// ODR/4.
    OdrDiv4,
    /// ODR/10.
    OdrDiv10,
    /// ODR/20.
    OdrDiv20,
    /// ODR/45.
    OdrDiv45,
    /// ODR/100.
    OdrDiv100,
    /// ODR/200.
    OdrDiv200,
    /// ODR/400.
    OdrDiv400,
    /// ODR/800.
    OdrDiv800,
}

impl AccelLpf2Bandwidth {
    /// HP_LPF2_XL_BW code.
    pub const fn code(self) -> u8 {
        match self {
            Self::OdrDiv4 => reg::LPF2_XL_BW_ODR_DIV_4,
            Self::OdrDiv10 => reg::LPF2_XL_BW_ODR_DIV_10,
            Self::OdrDiv20 => reg::LPF2_XL_BW_ODR_DIV_20,
            Self::OdrDiv45 => reg::LPF2_XL_BW_ODR_DIV_45,
            Self::OdrDiv100 => reg::LPF2_XL_BW_ODR_DIV_100,
            Self::OdrDiv200 => reg::LPF2_XL_BW_ODR_DIV_200,
            Self::OdrDiv400 => reg::LPF2_XL_BW_ODR_DIV_400,
            Self::OdrDiv800 => reg::LPF2_XL_BW_ODR_DIV_800,
        }
    }

    /// The divisor of the ODR.
    pub const fn divisor(self) -> u32 {
        match self {
            Self::OdrDiv4 => 4,
            Self::OdrDiv10 => 10,
            Self::OdrDiv20 => 20,
            Self::OdrDiv45 => 45,
            Self::OdrDiv100 => 100,
            Self::OdrDiv200 => 200,
            Self::OdrDiv400 => 400,
            Self::OdrDiv800 => 800,
        }
    }
}

/// Data-ready signal style, CTRL4 DRDY_PULSED (p68).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataReadyMode {
    /// 65 us pulses (CTRL4, p68). This is what `init` has always configured. Measured on the MARV
    /// V2 board, 2026-11-11: pulses of at least 51 us were seen, 57 us on average; a lower bound
    /// of the observation, not a contradiction of the datasheet's 65 us.
    #[default]
    Pulsed,
    /// Held until the high byte of the output register is read (the device's reset state).
    Latched,
}

/// FIFO mode, FIFO_CTRL4 FIFO_MODE (p61, 6.5, p44-46).
///
/// The four modes that need a trigger (`ContinuousWtmToFull`, `ContinuousToFifo`,
/// `BypassToContinuous`, `BypassToFifo`) follow "single tap, double tap, wake-up, high-g wake-up,
/// free-fall, D6D" (p45-46). The only trigger this crate can configure is the high-g wake-up.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// 000: FIFO off. Writing it also empties the FIFO.
    #[default]
    Bypass,
    /// 001: stops collecting when the FIFO is full.
    Fifo,
    /// 010: continuous with the size limited to the watermark until the trigger, then until full.
    ContinuousWtmToFull,
    /// 011: continuous until the trigger, then FIFO mode.
    ContinuousToFifo,
    /// 100: bypass until the trigger, then continuous.
    BypassToContinuous,
    /// 110: continuous; when full, a new word overwrites the oldest.
    Continuous,
    /// 111: bypass until the trigger, then FIFO mode.
    BypassToFifo,
}

impl FifoMode {
    /// FIFO_MODE code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Bypass => reg::FIFO_MODE_BYPASS,
            Self::Fifo => reg::FIFO_MODE_FIFO,
            Self::ContinuousWtmToFull => reg::FIFO_MODE_CONTINUOUS_WTM_TO_FULL,
            Self::ContinuousToFifo => reg::FIFO_MODE_CONTINUOUS_TO_FIFO,
            Self::BypassToContinuous => reg::FIFO_MODE_BYPASS_TO_CONTINUOUS,
            Self::Continuous => reg::FIFO_MODE_CONTINUOUS,
            Self::BypassToFifo => reg::FIFO_MODE_BYPASS_TO_FIFO,
        }
    }

    /// True for the modes that wait for a trigger event.
    pub const fn needs_trigger(self) -> bool {
        matches!(
            self,
            Self::ContinuousWtmToFull
                | Self::ContinuousToFifo
                | Self::BypassToContinuous
                | Self::BypassToFifo
        )
    }
}

/// Batch data rate of the low-g accelerometer or the gyroscope in the FIFO (Table 40, p60).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BatchRate {
    /// Not batched.
    #[default]
    Off,
    /// 1.875 Hz.
    Hz1_875,
    /// 7.5 Hz.
    Hz7_5,
    /// 15 Hz.
    Hz15,
    /// 30 Hz.
    Hz30,
    /// 60 Hz.
    Hz60,
    /// 120 Hz.
    Hz120,
    /// 240 Hz.
    Hz240,
    /// 480 Hz.
    Hz480,
    /// 960 Hz.
    Hz960,
    /// 1.92 kHz.
    Hz1920,
    /// 3.84 kHz.
    Hz3840,
    /// 7.68 kHz.
    Hz7680,
}

impl BatchRate {
    /// BDR_XL / BDR_GY code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Off => reg::BDR_OFF,
            Self::Hz1_875 => reg::BDR_1_875_HZ,
            Self::Hz7_5 => reg::BDR_7_5_HZ,
            Self::Hz15 => reg::BDR_15_HZ,
            Self::Hz30 => reg::BDR_30_HZ,
            Self::Hz60 => reg::BDR_60_HZ,
            Self::Hz120 => reg::BDR_120_HZ,
            Self::Hz240 => reg::BDR_240_HZ,
            Self::Hz480 => reg::BDR_480_HZ,
            Self::Hz960 => reg::BDR_960_HZ,
            Self::Hz1920 => reg::BDR_1920_HZ,
            Self::Hz3840 => reg::BDR_3840_HZ,
            Self::Hz7680 => reg::BDR_7680_HZ,
        }
    }

    /// Nominal rate in millihertz (0 for `Off`).
    pub const fn nominal_mhz(self) -> u32 {
        match self {
            Self::Off => 0,
            Self::Hz1_875 => 1_875,
            Self::Hz7_5 => 7_500,
            Self::Hz15 => 15_000,
            Self::Hz30 => 30_000,
            Self::Hz60 => 60_000,
            Self::Hz120 => 120_000,
            Self::Hz240 => 240_000,
            Self::Hz480 => 480_000,
            Self::Hz960 => 960_000,
            Self::Hz1920 => 1_920_000,
            Self::Hz3840 => 3_840_000,
            Self::Hz7680 => 7_680_000,
        }
    }

    /// The rate in millihertz in the high-accuracy ODR mode with the ODR set `set`: the table
    /// rate (Table 22, p34) of the standard rate of the same name. Measured at 960 Hz only
    /// (`hil/logs/lsm6dsk320x_dual-20261111-184219.log`); the other rates follow the same table
    /// by analogy. 1.875 Hz and 7.5 Hz have no table rate and stay nominal.
    pub(crate) const fn haodr_mhz(self, set: HaodrSet) -> u32 {
        let odr = match self {
            Self::Off | Self::Hz1_875 | Self::Hz7_5 => return self.nominal_mhz(),
            Self::Hz15 => Odr::Hz15,
            Self::Hz30 => Odr::Hz30,
            Self::Hz60 => Odr::Hz60,
            Self::Hz120 => Odr::Hz120,
            Self::Hz240 => Odr::Hz240,
            Self::Hz480 => Odr::Hz480,
            Self::Hz960 => Odr::Hz960,
            Self::Hz1920 => Odr::Hz1920,
            Self::Hz3840 => Odr::Hz3840,
            Self::Hz7680 => Odr::Hz7680,
        };
        match odr.haodr_mhz(set) {
            Some(mhz) => mhz,
            None => self.nominal_mhz(),
        }
    }
}

/// Decimation of the timestamp written to the FIFO, FIFO_CTRL4 DEC_TS_BATCH (p61). The write rate
/// is the larger of the two batch rates divided by the decimation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimestampBatch {
    /// Timestamp not batched.
    #[default]
    Off,
    /// Every batch event of the faster channel.
    Dec1,
    /// One in 8.
    Dec8,
    /// One in 32.
    Dec32,
}

impl TimestampBatch {
    /// DEC_TS_BATCH code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Off => reg::DEC_TS_BATCH_OFF,
            Self::Dec1 => reg::DEC_TS_BATCH_1,
            Self::Dec8 => reg::DEC_TS_BATCH_8,
            Self::Dec32 => reg::DEC_TS_BATCH_32,
        }
    }
}

/// Batch data rate of the temperature in the FIFO, FIFO_CTRL4 ODR_T_BATCH (p61).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TempBatchRate {
    /// Not batched.
    #[default]
    Off,
    /// 1.875 Hz.
    Hz1_875,
    /// 15 Hz.
    Hz15,
    /// 60 Hz.
    Hz60,
}

impl TempBatchRate {
    /// ODR_T_BATCH code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Off => reg::ODR_T_BATCH_OFF,
            Self::Hz1_875 => reg::ODR_T_BATCH_1_875_HZ,
            Self::Hz15 => reg::ODR_T_BATCH_15_HZ,
            Self::Hz60 => reg::ODR_T_BATCH_60_HZ,
        }
    }
}

/// FIFO depth in words: the unread words the device holds before it overruns.
///
/// Measured on the board, 2026-11-10: `fifo_status` reported 256 unread words with full, overrun
/// and latched overrun set, and one burst returned all 256. The datasheet gives the size only as
/// "1.5 KB" (6.5, p44); with 7 bytes to a word that is 214 or 219 words, below the measured depth,
/// so the figure is the measurement. The watermark field is 8 bits (FIFO_CTRL1 WTM\[7:0\], p59), so
/// its largest value, 255, is always inside the depth and [`Config::validate`] accepts every
/// watermark the field can hold.
pub const FIFO_DEPTH_WORDS: u16 = 256;

/// FIFO settings. With the mode `Bypass` the FIFO is off and the rest is written but unused.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoConfig {
    /// FIFO mode.
    pub mode: FifoMode,
    /// Watermark in words (FIFO_CTRL1 WTM; one word is a tag and 6 data bytes, p59).
    pub watermark: u8,
    /// Gyroscope batch data rate.
    pub gyro: BatchRate,
    /// Low-g accelerometer batch data rate.
    pub accel: BatchRate,
    /// Batch the high-g accelerometer (COUNTER_BDR_REG1 XL_HG_BATCH_EN, p62).
    pub high_g: bool,
    /// Timestamp batching.
    pub timestamp: TimestampBatch,
    /// Temperature batch data rate.
    pub temperature: TempBatchRate,
    /// Limits the FIFO depth to the watermark (FIFO_CTRL2 STOP_ON_WTM, Table 38, p59; 6.5.2, p45).
    pub stop_on_wtm: bool,
    /// Batches a configuration-change word in the FIFO whenever the accelerometer or gyroscope
    /// configuration changes (FIFO_CTRL2 ODR_CHG_EN, Table 38, p59; 6.5.8, p46).
    ///
    /// Observed on the board, 2026-11-10: with this on, a configuration-change word is the FIRST
    /// word of the stream after every FIFO start or restart (including [`crate::Lsm6dsk320x::fifo_flush`]),
    /// even when nothing was changed, and a second one was seen once a few words later. Its data
    /// bytes 5 and 6 looked like the two rate codes (0x99 at 960 Hz gyroscope and accelerometer,
    /// 0x88 at 480 Hz); bytes 1 to 4 are not understood. The layout remains undefined by the
    /// datasheet; the word is carried raw as [`crate::fifo::FifoConfigChange`] and not decoded.
    pub odr_change: bool,
}

impl FifoConfig {
    /// FIFO off, nothing batched, watermark 0 (the reset state).
    pub const fn new() -> Self {
        Self {
            mode: FifoMode::Bypass,
            watermark: 0,
            gyro: BatchRate::Off,
            accel: BatchRate::Off,
            high_g: false,
            timestamp: TimestampBatch::Off,
            temperature: TempBatchRate::Off,
            stop_on_wtm: false,
            odr_change: false,
        }
    }

    setters! {
        /// Sets [`Self::mode`].
        mode, with_mode: FifoMode;
        /// Sets [`Self::watermark`].
        watermark, with_watermark: u8;
        /// Sets [`Self::gyro`].
        gyro, with_gyro: BatchRate;
        /// Sets [`Self::accel`].
        accel, with_accel: BatchRate;
        /// Sets [`Self::high_g`].
        high_g, with_high_g: bool;
        /// Sets [`Self::timestamp`].
        timestamp, with_timestamp: TimestampBatch;
        /// Sets [`Self::temperature`].
        temperature, with_temperature: TempBatchRate;
        /// Sets [`Self::stop_on_wtm`].
        stop_on_wtm, with_stop_on_wtm: bool;
        /// Sets [`Self::odr_change`].
        odr_change, with_odr_change: bool;
    }
}

/// Interrupt pin activation level, IF_CFG H_LACTIVE (p58).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IntPolarity {
    /// Active high.
    #[default]
    ActiveHigh,
    /// Active low.
    ActiveLow,
}

/// Interrupt pin output type, IF_CFG PP_OD (p58).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IntOutput {
    /// Push-pull.
    #[default]
    PushPull,
    /// Open-drain.
    OpenDrain,
}

/// Drive strength of the interrupt pads, PIN_CTRL IO_PAD_STRENGTH (p57). The code 10 is reserved.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PadDrive {
    /// 00: lowest, recommended for VDDIO >= 3.0 V.
    Lowest,
    /// 01: intermediate, recommended for 2.0 V <= VDDIO < 3 V.
    Intermediate,
    /// 11: highest, recommended for VDDIO < 2 V. The reset state.
    #[default]
    Highest,
}

impl PadDrive {
    /// IO_PAD_STRENGTH code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Lowest => reg::IO_PAD_STRENGTH_LOWEST,
            Self::Intermediate => reg::IO_PAD_STRENGTH_INTERMEDIATE,
            Self::Highest => reg::IO_PAD_STRENGTH_HIGHEST,
        }
    }
}

/// Electrical settings of the interrupt pins.
///
/// Measured on the MARV V2 board, 2026-11-11: both polarities, open-drain and push-pull output,
/// latched data-ready and the three drive settings behaved as documented here.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct PinConfig {
    /// Activation level of INT1 and INT2.
    pub polarity: IntPolarity,
    /// Output type of INT1 and INT2.
    pub output: IntOutput,
    /// Pad drive strength.
    pub drive: PadDrive,
}

impl PinConfig {
    /// Active high, push-pull, highest drive (the reset state).
    pub const fn new() -> Self {
        Self {
            polarity: IntPolarity::ActiveHigh,
            output: IntOutput::PushPull,
            drive: PadDrive::Highest,
        }
    }

    setters! {
        /// Sets [`Self::polarity`].
        polarity, with_polarity: IntPolarity;
        /// Sets [`Self::output`].
        output, with_output: IntOutput;
        /// Sets [`Self::drive`].
        drive, with_drive: PadDrive;
    }
}

/// Wake-up interrupt type of the high-g generator, HG_FUNCTIONS_ENABLE HG_WU_CHANGE_INT_SEL (p94).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HighGWakeUpMode {
    /// An interrupt each time one of the three axes is above the threshold.
    #[default]
    EachTime,
    /// An interrupt only when high-g passes from all axes under the threshold to at least one axis
    /// over it, or the reverse.
    OnChange,
}

/// High-g wake-up and shock generator. `Config::high_g_events` is `None` to leave it off.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct HighGEvents {
    /// HG_WAKE_UP_THS threshold in LSB: 1 g per LSB up to +-256 g full scale, 1.25 g per LSB at
    /// +-320 g (p94). See [`crate::calc::hg_wake_up_threshold_mg`].
    pub threshold: u8,
    /// Wake-up interrupt type.
    pub wake_up_mode: HighGWakeUpMode,
    /// HG_SHOCK_DUR, 0 to 14: the shock state ends after `(value + 1) * 512 / ODR_XL_HG` seconds
    /// below the threshold (p94).
    pub shock_duration: u8,
    /// Feeds the wake-up and shock with offset-corrected data (CTRL2_XL_HG HG_USR_OFF_ON_WU, p91).
    pub user_offset: bool,
}

impl HighGEvents {
    /// Threshold 0, every crossing, shock duration 0, no user offset (the reset state).
    pub const fn new() -> Self {
        Self {
            threshold: 0,
            wake_up_mode: HighGWakeUpMode::EachTime,
            shock_duration: 0,
            user_offset: false,
        }
    }

    setters! {
        /// Sets [`Self::threshold`].
        threshold, with_threshold: u8;
        /// Sets [`Self::wake_up_mode`].
        wake_up_mode, with_wake_up_mode: HighGWakeUpMode;
        /// Sets [`Self::shock_duration`].
        shock_duration, with_shock_duration: u8;
        /// Sets [`Self::user_offset`].
        user_offset, with_user_offset: bool;
    }
}

/// Weight of one LSB of the low-g user offset registers, CTRL9 USR_OFF_W (p72, p113).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OffsetWeight {
    /// 2^-10 g per LSB.
    #[default]
    Fine,
    /// 2^-6 g per LSB.
    Coarse,
}

/// Driver configuration.
///
/// The defaults reproduce what `init` configured before this type grew: +-8 g and +-2000 dps at
/// 960 Hz in high-performance mode, high-g off, low-g data-ready on INT1 only, pulsed data-ready,
/// push-pull active-high pins; every other field has the device's reset value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Low-g accelerometer full scale.
    pub accel_fs: AccelFullScale,
    /// Low-g accelerometer output data rate.
    pub accel_odr: Odr,
    /// Gyroscope full scale.
    pub gyro_fs: GyroFullScale,
    /// Gyroscope output data rate.
    pub gyro_odr: Odr,
    /// High-g channel, or `None` for off.
    pub high_g: Option<HighGConfig>,
    /// Signals routed to INT1.
    pub int1: IntRouting,
    /// Signals routed to INT2.
    pub int2: IntRouting,
    /// Operating mode of the low-g accelerometer and the gyroscope.
    pub mode: OperatingMode,
    /// Gyroscope LPF1 bandwidth, or `None` for LPF1 off.
    pub gyro_lpf1: Option<GyroLpf1Bandwidth>,
    /// Low-g accelerometer LPF2 cutoff, or `None` for the first filtering stage only (ODR/2).
    ///
    /// A narrow cutoff settles slowly after the sensor is started, and the data-ready mask does
    /// not cover all of it: see [`Self::drdy_mask`] and [`Self::accel_fast_settling`] (measured
    /// on the MARV V2 board, 2026-11-11: ODR/100 at 240 Hz needed 170 ms to be within 10 % of
    /// 1 g without fast settling). Noise fell with the bandwidth as expected.
    pub accel_lpf2: Option<AccelLpf2Bandwidth>,
    /// Masks the data-ready signals until the filter settling ends (CTRL4 DRDY_MASK, p68); each of
    /// the low-g accelerometer, the high-g accelerometer and the gyroscope is masked independently.
    /// On by default.
    ///
    /// The mask covers only the signalled data-ready of the sensor concerned, and only for the
    /// settling the device knows about. Measured on the MARV V2 board, 2026-11-11, with both
    /// sensors started by `configure`, the mask on and INT1 routed to the accelerometer's
    /// data-ready: the first 4 gyroscope samples read after `configure` returned were off by up
    /// to 6.6 dps from the later mean and the fifth by up to 2.3 dps (three runs). With INT1 routed
    /// to the gyroscope's data-ready the first edge came 64.8 ms after `configure` started the
    /// sensor and every sample was clean, so the gyroscope's data was valid about 65 ms after its
    /// start. `configure` and `init` therefore wait 70 ms after starting the gyroscope (the
    /// datasheet's 40 ms, Table 4 p15, is typical and not guaranteed), and the first gyroscope
    /// samples are valid once they return, whichever data-ready is routed. With the mask off the transient showed in one run of three. The mask hid about the first 2
    /// accelerometer samples (mask off: first sample 0.228 g; mask on: all in band).
    ///
    /// The accelerometer's own settling is not covered by that wait: see
    /// [`Self::accel_fast_settling`].
    pub drdy_mask: bool,
    /// LPF2 fast-settling mode of the low-g accelerometer, active while exiting power-down
    /// (CTRL9 XL_FASTSETTL_MODE, p72).
    ///
    /// Measured on the MARV V2 board, 2026-11-11, with the narrow second-stage filter
    /// ([`AccelLpf2Bandwidth::OdrDiv100`] at 240 Hz) and the mask on: without it the first
    /// data-ready edge came at 68.6 ms with the output at 0.62 g and the output was within 10 % of
    /// 1 g only at 170 ms; with it on the first sample was in band at 18 ms. With a narrow LPF2,
    /// turn this on or discard samples until the output has settled.
    pub accel_fast_settling: bool,
    /// Data-ready pulsed or latched.
    pub data_ready_mode: DataReadyMode,
    /// Timestamp counter on (TIMESTAMP0..3, FUNCTIONS_ENABLE TIMESTAMP_EN, p93).
    pub timestamp: bool,
    /// FIFO.
    pub fifo: FifoConfig,
    /// Interrupt pin electrical settings.
    pub pins: PinConfig,
    /// High-g wake-up and shock generator, or `None` for off.
    pub high_g_events: Option<HighGEvents>,
    /// Low-g user offset: the weight, or `None` for the offset block bypassed (CTRL9, p72).
    pub accel_offset: Option<OffsetWeight>,
}

impl Config {
    /// +-8 g and +-2000 dps at 960 Hz, high-g off, low-g data-ready on INT1 only. The gyroscope
    /// is not routed by default: its pulse is not aligned with the accelerometer's, so ORing both
    /// can give two edges per sample period.
    pub const fn new() -> Self {
        Self {
            accel_fs: AccelFullScale::G8,
            accel_odr: Odr::Hz960,
            gyro_fs: GyroFullScale::Dps2000,
            gyro_odr: Odr::Hz960,
            high_g: None,
            int1: IntRouting::new().with_data_ready(DataReady::new().with_accel(true)),
            int2: IntRouting::new(),
            mode: OperatingMode::HighPerformance,
            gyro_lpf1: None,
            accel_lpf2: None,
            drdy_mask: true,
            accel_fast_settling: false,
            data_ready_mode: DataReadyMode::Pulsed,
            timestamp: false,
            fifo: FifoConfig::new(),
            pins: PinConfig::new(),
            high_g_events: None,
            accel_offset: None,
        }
    }

    setters! {
        /// Sets [`Self::accel_fs`].
        accel_fs, with_accel_fs: AccelFullScale;
        /// Sets [`Self::accel_odr`].
        accel_odr, with_accel_odr: Odr;
        /// Sets [`Self::gyro_fs`].
        gyro_fs, with_gyro_fs: GyroFullScale;
        /// Sets [`Self::gyro_odr`].
        gyro_odr, with_gyro_odr: Odr;
        /// Sets [`Self::high_g`].
        high_g, with_high_g: Option<HighGConfig>;
        /// Sets [`Self::int1`].
        int1, with_int1: IntRouting;
        /// Sets [`Self::int2`].
        int2, with_int2: IntRouting;
        /// Sets [`Self::mode`].
        mode, with_mode: OperatingMode;
        /// Sets [`Self::gyro_lpf1`].
        gyro_lpf1, with_gyro_lpf1: Option<GyroLpf1Bandwidth>;
        /// Sets [`Self::accel_lpf2`].
        accel_lpf2, with_accel_lpf2: Option<AccelLpf2Bandwidth>;
        /// Sets [`Self::drdy_mask`].
        drdy_mask, with_drdy_mask: bool;
        /// Sets [`Self::accel_fast_settling`].
        accel_fast_settling, with_accel_fast_settling: bool;
        /// Sets [`Self::data_ready_mode`].
        data_ready_mode, with_data_ready_mode: DataReadyMode;
        /// Sets [`Self::timestamp`].
        timestamp, with_timestamp: bool;
        /// Sets [`Self::fifo`].
        fifo, with_fifo: FifoConfig;
        /// Sets [`Self::pins`].
        pins, with_pins: PinConfig;
        /// Sets [`Self::high_g_events`].
        high_g_events, with_high_g_events: Option<HighGEvents>;
        /// Sets [`Self::accel_offset`].
        accel_offset, with_accel_offset: Option<OffsetWeight>;
    }

    /// Checks the combinations the datasheet rules out. The first violation in the order of
    /// [`ConfigError`] is returned. Nothing is sent to the device.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.mode.is_haodr() {
            if self.accel_odr == Odr::Hz7_5 {
                return Err(ConfigError::HaodrOdrReserved {
                    channel: Channel::LowG,
                });
            }
            if self.gyro_odr == Odr::Hz7_5 {
                return Err(ConfigError::HaodrOdrReserved {
                    channel: Channel::Gyro,
                });
            }
        }

        if self.fifo.gyro.code() > self.gyro_odr.code() {
            return Err(ConfigError::BatchRateAboveOdr {
                channel: Channel::Gyro,
            });
        }
        if self.fifo.accel.code() > self.accel_odr.code() {
            return Err(ConfigError::BatchRateAboveOdr {
                channel: Channel::LowG,
            });
        }
        if self.fifo.high_g && self.high_g.is_none() {
            return Err(ConfigError::HighGBatchNeedsHighG);
        }

        let watermark_used = self.int1.fifo_threshold
            || self.int2.fifo_threshold
            || matches!(self.fifo.mode, FifoMode::ContinuousWtmToFull)
            || (self.fifo.stop_on_wtm && !matches!(self.fifo.mode, FifoMode::Bypass));
        if watermark_used && self.fifo.watermark == 0 {
            return Err(ConfigError::FifoWatermarkZero);
        }
        if matches!(self.fifo.mode, FifoMode::Bypass)
            && (self.int1.any_fifo() || self.int2.any_fifo())
        {
            return Err(ConfigError::FifoInterruptNeedsFifo);
        }
        if self.fifo.mode.needs_trigger() && self.high_g_events.is_none() {
            return Err(ConfigError::FifoTriggerNeedsHighGEvents);
        }

        if !self.timestamp && !matches!(self.fifo.timestamp, TimestampBatch::Off) {
            return Err(ConfigError::TimestampBatchNeedsCounter);
        }
        if self.int1.timestamp_overflow {
            return Err(ConfigError::TimestampOverflowOnInt1);
        }
        if self.int2.timestamp_overflow && !self.timestamp {
            return Err(ConfigError::TimestampOverflowNeedsCounter);
        }

        if self.high_g_events.is_some() && self.high_g.is_none() {
            return Err(ConfigError::HighGEventsNeedHighG);
        }
        if (self.int1.any_high_g_event() || self.int2.any_high_g_event())
            && self.high_g_events.is_none()
        {
            return Err(ConfigError::HighGRoutingNeedsEvents);
        }
        if let Some(e) = self.high_g_events
            && e.shock_duration > reg::HG_SHOCK_DUR_MAX
        {
            return Err(ConfigError::HighGShockDurationOutOfRange {
                value: e.shock_duration,
            });
        }
        Ok(())
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// A channel named by a [`ConfigError`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    /// The gyroscope.
    Gyro,
    /// The low-g accelerometer.
    LowG,
    /// The high-g accelerometer.
    HighG,
}

/// Why a [`Config`] (or [`crate::calc::Offsets`]) is rejected. Each variant cites its datasheet
/// page. [`Config::validate`] reports them in the order listed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The high-accuracy ODR mode has no 7.5 Hz rate: that code is reserved there (Table 22, p34).
    HaodrOdrReserved {
        /// The channel with the 7.5 Hz ODR.
        channel: Channel,
    },
    /// A channel is batched at a rate above its ODR, or batched while its ODR is power-down. The
    /// datasheet lists the batch rates (Table 40, p60) but states no such rule; it is enforced
    /// because the FIFO is fed by the channel's samples (6.5, p44).
    BatchRateAboveOdr {
        /// The channel.
        channel: Channel,
    },
    /// The high-g accelerometer is batched in the FIFO but the high-g channel is off
    /// (COUNTER_BDR_REG1, p62).
    HighGBatchNeedsHighG,
    /// The watermark is 0 while something depends on it: the FIFO threshold interrupt, the
    /// continuousWTM-to-full mode or STOP_ON_WTM, which limits the FIFO depth to the watermark
    /// (FIFO_CTRL1, p59; 6.5.2 and 6.5.5, p45).
    FifoWatermarkZero,
    /// A FIFO interrupt is routed while the FIFO mode is bypass, where "the FIFO is not
    /// operational and it remains empty" (6.5.1, p45).
    FifoInterruptNeedsFifo,
    /// A triggered FIFO mode is selected but the high-g generator, the only trigger this crate can
    /// configure, is off (6.5.4 to 6.5.7, p45-46).
    FifoTriggerNeedsHighGEvents,
    /// The FIFO batches timestamps but the timestamp counter is off (FIFO_CTRL4, p61;
    /// FUNCTIONS_ENABLE, p93).
    TimestampBatchNeedsCounter,
    /// INT1 has no timestamp overflow source; only INT2 does (MD2_CFG, p103).
    TimestampOverflowOnInt1,
    /// The timestamp overflow is routed to INT2 but the timestamp counter is off (p93, p103).
    TimestampOverflowNeedsCounter,
    /// The high-g wake-up and shock generator is on but the high-g channel is off (p94).
    HighGEventsNeedHighG,
    /// A high-g wake-up or shock event is routed to a pin but the generator is off (p94, p96).
    HighGRoutingNeedsEvents,
    /// HG_SHOCK_DUR allows 0 to 14 (p94).
    HighGShockDurationOutOfRange {
        /// The value.
        value: u8,
    },
    /// A user offset is -128: the registers accept -127 to 127 (p108, p113).
    OffsetOutOfRange {
        /// The channel group, `LowG` or `HighG`.
        channel: Channel,
    },
    /// A step list did not fit its fixed capacity: the register write plan
    /// ([`crate::plan::MAX_STEPS`]) or a self-test sequence ([`crate::selftest::MAX_STEPS`]). Not
    /// reachable from any [`Config`] or [`SelfTestConfig`](crate::SelfTestConfig) today; reported so
    /// that a step is never dropped silently.
    PlanOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_scale_codes() {
        use AccelFullScale::*;
        assert_eq!([G2.code(), G4.code(), G8.code(), G16.code()], [0, 1, 2, 3]);
        use GyroFullScale::*;
        assert_eq!(
            [
                Dps250.code(),
                Dps500.code(),
                Dps1000.code(),
                Dps2000.code(),
                Dps4000.code()
            ],
            [1, 2, 3, 4, 5]
        );
        use HighGFullScale::*;
        assert_eq!(
            [
                G32.code(),
                G64.code(),
                G128.code(),
                G256.code(),
                G320.code()
            ],
            [0, 1, 2, 3, 4]
        );
    }

    #[test]
    fn odr_codes() {
        let all = [
            Odr::Off,
            Odr::Hz7_5,
            Odr::Hz15,
            Odr::Hz30,
            Odr::Hz60,
            Odr::Hz120,
            Odr::Hz240,
            Odr::Hz480,
            Odr::Hz960,
            Odr::Hz1920,
            Odr::Hz3840,
            Odr::Hz7680,
        ];
        assert_eq!(all.map(Odr::code), [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
        assert_eq!(Odr::Hz7680.hz(), 7680.0);
        let hg = [
            HighGOdr::Hz480,
            HighGOdr::Hz960,
            HighGOdr::Hz1920,
            HighGOdr::Hz3840,
            HighGOdr::Hz7680,
        ];
        assert_eq!(hg.map(HighGOdr::code), [3, 4, 5, 6, 7]);
    }

    #[test]
    fn register_encoders() {
        let cfg = Config::new()
            .with_accel_fs(AccelFullScale::G4)
            .with_accel_odr(Odr::Hz960)
            .with_gyro_fs(GyroFullScale::Dps250)
            .with_gyro_odr(Odr::Hz120)
            .with_high_g(Some(
                HighGConfig::new()
                    .with_odr(HighGOdr::Hz960)
                    .with_full_scale(HighGFullScale::G32),
            ))
            .with_int1(
                IntRouting::new()
                    .with_data_ready(DataReady::new().with_accel(true).with_gyro(true)),
            )
            .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)));
        assert_eq!(cfg.ctrl1(), 0x09);
        assert_eq!(cfg.ctrl2(), 0x06);
        assert_eq!(cfg.ctrl6(), 0x09);
        assert_eq!(cfg.ctrl8(), 0x01);
        assert_eq!(cfg.ctrl7(), 0x40);
        assert_eq!(cfg.ctrl1_xl_hg(), 0x80 | (4 << 3));
        assert_eq!(cfg.int1_ctrl(), 0x03);
        assert_eq!(cfg.int2_ctrl(), 0x00);
        assert_eq!(cfg.ctrl4(), 0x0A);
        assert_eq!(cfg.with_high_g(None).ctrl1_xl_hg(), 0);
    }
}

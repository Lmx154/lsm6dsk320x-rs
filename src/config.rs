//! Typed configuration and `const fn` encoders to register values.

use crate::reg;

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
            Self::G2 => 0b00,
            Self::G4 => 0b01,
            Self::G8 => 0b10,
            Self::G16 => 0b11,
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
            Self::Dps250 => 0b001,
            Self::Dps500 => 0b010,
            Self::Dps1000 => 0b011,
            Self::Dps2000 => 0b100,
            Self::Dps4000 => 0b101,
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
            Self::G32 => 0b000,
            Self::G64 => 0b001,
            Self::G128 => 0b010,
            Self::G256 => 0b011,
            Self::G320 => 0b100,
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
            Self::Off => 0b0000,
            Self::Hz7_5 => 0b0010,
            Self::Hz15 => 0b0011,
            Self::Hz30 => 0b0100,
            Self::Hz60 => 0b0101,
            Self::Hz120 => 0b0110,
            Self::Hz240 => 0b0111,
            Self::Hz480 => 0b1000,
            Self::Hz960 => 0b1001,
            Self::Hz1920 => 0b1010,
            Self::Hz3840 => 0b1011,
            Self::Hz7680 => 0b1100,
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
            Self::Hz480 => 0b011,
            Self::Hz960 => 0b100,
            Self::Hz1920 => 0b101,
            Self::Hz3840 => 0b110,
            Self::Hz7680 => 0b111,
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
}

/// High-g channel settings. `Config::high_g` is `None` to leave the channel off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HighGConfig {
    /// Output data rate.
    pub odr: HighGOdr,
    /// Full scale.
    pub full_scale: HighGFullScale,
}

/// Which data-ready signals drive one interrupt pin (they are ORed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    pub const NONE: Self = Self {
        accel: false,
        gyro: false,
        high_g: false,
    };

    /// INT1_CTRL / INT2_CTRL value (the low-g and gyro bits share positions on both pins).
    pub const fn int_ctrl(self) -> u8 {
        (if self.accel { reg::INT_DRDY_XL } else { 0 })
            | (if self.gyro { reg::INT_DRDY_G } else { 0 })
    }
}

/// Driver configuration. All three channels run in high-performance mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// Data-ready signals routed to INT1.
    pub int1: DataReady,
    /// Data-ready signals routed to INT2.
    pub int2: DataReady,
}

impl Default for Config {
    /// +-8 g and +-2000 dps at 960 Hz, high-g off, low-g data-ready on INT1 only. The gyroscope
    /// is not routed by default: its pulse is not aligned with the accelerometer's, so ORing both
    /// can give two edges per sample period.
    fn default() -> Self {
        Self {
            accel_fs: AccelFullScale::G8,
            accel_odr: Odr::Hz960,
            gyro_fs: GyroFullScale::Dps2000,
            gyro_odr: Odr::Hz960,
            high_g: None,
            int1: DataReady {
                accel: true,
                gyro: false,
                high_g: false,
            },
            int2: DataReady::NONE,
        }
    }
}

impl Config {
    /// IF_CFG: 4-wire SPI, push-pull, active-high interrupts, I2C/I3C disabled.
    pub const fn if_cfg(&self) -> u8 {
        reg::IF_CFG_I2C_I3C_DISABLE
    }

    /// INT1_CTRL.
    pub const fn int1_ctrl(&self) -> u8 {
        self.int1.int_ctrl()
    }

    /// INT2_CTRL.
    pub const fn int2_ctrl(&self) -> u8 {
        self.int2.int_ctrl()
    }

    /// CTRL1: high-performance mode and the accelerometer ODR.
    pub const fn ctrl1(&self) -> u8 {
        self.accel_odr.code()
    }

    /// CTRL2: high-performance mode and the gyroscope ODR.
    pub const fn ctrl2(&self) -> u8 {
        self.gyro_odr.code()
    }

    /// CTRL4: pulsed data-ready.
    pub const fn ctrl4(&self) -> u8 {
        reg::CTRL4_DRDY_PULSED
    }

    /// CTRL6: the gyroscope full scale with the mandatory bit 3.
    pub const fn ctrl6(&self) -> u8 {
        reg::CTRL6_MUST_BE_ONE | self.gyro_fs.code()
    }

    /// CTRL7: high-g data-ready routing.
    pub const fn ctrl7(&self) -> u8 {
        (if self.int1.high_g {
            reg::CTRL7_INT1_DRDY_XL_HG
        } else {
            0
        }) | (if self.int2.high_g {
            reg::CTRL7_INT2_DRDY_XL_HG
        } else {
            0
        })
    }

    /// CTRL8: the accelerometer full scale.
    pub const fn ctrl8(&self) -> u8 {
        self.accel_fs.code()
    }

    /// CTRL1_XL_HG: output enabled with ODR and full scale, or 0 (power-down) when off.
    pub const fn ctrl1_xl_hg(&self) -> u8 {
        match self.high_g {
            Some(h) => reg::CTRL1_XL_HG_REGOUT_EN | (h.odr.code() << 3) | h.full_scale.code(),
            None => 0,
        }
    }
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
        let cfg = Config {
            accel_fs: AccelFullScale::G4,
            accel_odr: Odr::Hz960,
            gyro_fs: GyroFullScale::Dps250,
            gyro_odr: Odr::Hz120,
            high_g: Some(HighGConfig {
                odr: HighGOdr::Hz960,
                full_scale: HighGFullScale::G32,
            }),
            int1: DataReady {
                accel: true,
                gyro: true,
                high_g: false,
            },
            int2: DataReady {
                accel: false,
                gyro: false,
                high_g: true,
            },
        };
        assert_eq!(cfg.ctrl1(), 0x09);
        assert_eq!(cfg.ctrl2(), 0x06);
        assert_eq!(cfg.ctrl6(), 0x09);
        assert_eq!(cfg.ctrl8(), 0x01);
        assert_eq!(cfg.ctrl7(), 0x40);
        assert_eq!(cfg.ctrl1_xl_hg(), 0x80 | (4 << 3));
        assert_eq!(cfg.int1_ctrl(), 0x03);
        assert_eq!(cfg.int2_ctrl(), 0x00);
        assert_eq!(cfg.ctrl4(), 0x02);
        assert_eq!(
            Config {
                high_g: None,
                ..cfg
            }
            .ctrl1_xl_hg(),
            0
        );
    }
}

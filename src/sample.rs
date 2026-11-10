//! Raw sample types, pure decoders and conversions to SI units.

use crate::config::{AccelFullScale, GyroFullScale, HighGFullScale};
use crate::reg;

/// Standard gravity, m/s² per g.
pub const G: f32 = 9.80665;
const RAD_PER_DEG: f32 = core::f32::consts::PI / 180.0;

/// Three raw 16-bit axis values, two's complement counts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawAxes {
    /// X axis.
    pub x: i16,
    /// Y axis.
    pub y: i16,
    /// Z axis.
    pub z: i16,
}

impl RawAxes {
    /// Decodes six little-endian bytes (X low, X high, Y low, ...).
    pub const fn from_bytes(b: &[u8; 6]) -> Self {
        Self {
            x: i16::from_le_bytes([b[0], b[1]]),
            y: i16::from_le_bytes([b[2], b[3]]),
            z: i16::from_le_bytes([b[4], b[5]]),
        }
    }

    const fn scaled(self, per_lsb: f32) -> [f32; 3] {
        [
            self.x as f32 * per_lsb,
            self.y as f32 * per_lsb,
            self.z as f32 * per_lsb,
        ]
    }

    /// Low-g accelerometer counts to m/s².
    pub const fn accel_ms2(self, fs: AccelFullScale) -> [f32; 3] {
        self.scaled(fs.mg_per_lsb() * 1e-3 * G)
    }

    /// Gyroscope counts to rad/s.
    pub const fn gyro_rads(self, fs: GyroFullScale) -> [f32; 3] {
        self.scaled(fs.mdps_per_lsb() * 1e-3 * RAD_PER_DEG)
    }

    /// High-g accelerometer counts to m/s².
    pub const fn high_g_ms2(self, fs: HighGFullScale) -> [f32; 3] {
        self.scaled(fs.mg_per_lsb() * 1e-3 * G)
    }
}

/// Gyroscope and low-g accelerometer, as read by [`crate::Lsm6dsk320x::read`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSample {
    /// Gyroscope, OUTX_L_G (0x22) onward.
    pub gyro: RawAxes,
    /// Low-g accelerometer, OUTX_L_A (0x28) onward.
    pub accel: RawAxes,
}

impl RawSample {
    /// Decodes the 12 bytes starting at OUTX_L_G.
    pub const fn from_bytes(b: &[u8; 12]) -> Self {
        Self {
            gyro: RawAxes::from_bytes(&[b[0], b[1], b[2], b[3], b[4], b[5]]),
            accel: RawAxes::from_bytes(&[b[6], b[7], b[8], b[9], b[10], b[11]]),
        }
    }
}

/// The whole output block, as read by [`crate::Lsm6dsk320x::read_all`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawAll {
    /// Temperature, OUT_TEMP_L (0x20).
    pub temperature: i16,
    /// Gyroscope.
    pub gyro: RawAxes,
    /// Low-g accelerometer.
    pub accel: RawAxes,
    /// High-g accelerometer, UI_OUTX_L_A_OIS_HG (0x34) onward.
    pub high_g: RawAxes,
}

impl RawAll {
    /// Length of the block read, OUT_TEMP_L to UI_OUTZ_H_A_OIS_HG inclusive.
    pub const LEN: usize = (reg::UI_OUTZ_H_A_OIS_HG - reg::OUT_TEMP_L) as usize + 1;

    /// Decodes the block. The six bytes at 0x2E..=0x33 are skipped.
    pub const fn from_bytes(b: &[u8; Self::LEN]) -> Self {
        Self {
            temperature: i16::from_le_bytes([b[0], b[1]]),
            gyro: RawAxes::from_bytes(&[b[2], b[3], b[4], b[5], b[6], b[7]]),
            accel: RawAxes::from_bytes(&[b[8], b[9], b[10], b[11], b[12], b[13]]),
            high_g: RawAxes::from_bytes(&[b[20], b[21], b[22], b[23], b[24], b[25]]),
        }
    }

    /// Temperature in °C (p16).
    pub const fn temperature_c(&self) -> f32 {
        temperature_c(self.temperature)
    }
}

/// Raw temperature to °C: counts / 256 + 25 (p16).
pub const fn temperature_c(raw: i16) -> f32 {
    raw as f32 / 256.0 + 25.0
}

/// STATUS_REG (0x1E).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// New low-g accelerometer data.
    pub accel: bool,
    /// New gyroscope data.
    pub gyro: bool,
    /// New temperature data.
    pub temperature: bool,
    /// New high-g accelerometer data.
    pub high_g: bool,
}

impl Status {
    /// Decodes the register value.
    pub const fn from_bits(v: u8) -> Self {
        Self {
            accel: v & reg::STATUS_XLDA != 0,
            gyro: v & reg::STATUS_GDA != 0,
            temperature: v & reg::STATUS_TDA != 0,
            high_g: v & reg::STATUS_XLHGDA != 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_sample() {
        let b = [
            1, 0, 0xFF, 0xFF, 0x00, 0x80, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00,
        ];
        let s = RawSample::from_bytes(&b);
        assert_eq!(
            s.gyro,
            RawAxes {
                x: 1,
                y: -1,
                z: i16::MIN
            }
        );
        assert_eq!(
            s.accel,
            RawAxes {
                x: 0x10,
                y: 0x20,
                z: 0x30
            }
        );
    }

    #[test]
    fn decode_all() {
        let mut b = [0u8; RawAll::LEN];
        assert_eq!(RawAll::LEN, 26);
        b[0] = 0x00;
        b[1] = 0x01;
        b[2] = 7;
        b[13] = 0x7F;
        b[14] = 0xEE;
        b[20] = 9;
        b[25] = 0xFF;
        let a = RawAll::from_bytes(&b);
        assert_eq!(a.temperature, 256);
        assert_eq!(a.gyro.x, 7);
        assert_eq!(a.accel.z, 0x7F00);
        assert_eq!(
            a.high_g,
            RawAxes {
                x: 9,
                y: 0,
                z: -256
            }
        );
    }

    #[test]
    fn decode_status() {
        let s = Status::from_bits(0b1010);
        assert_eq!(
            s,
            Status {
                accel: false,
                gyro: true,
                temperature: false,
                high_g: true
            }
        );
    }

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-3 * b.abs().max(1.0)
    }

    #[test]
    fn conversions() {
        // 8192 counts at +-4 g (0.122 mg/LSB) is 0.999424 g.
        let a = RawAxes {
            x: 8192,
            y: 0,
            z: 0,
        }
        .accel_ms2(AccelFullScale::G4);
        assert!(close(a[0], 0.999424 * G));
        // 1000 counts at +-250 dps is 8.75 dps.
        let g = RawAxes {
            x: 0,
            y: 1000,
            z: 0,
        }
        .gyro_rads(GyroFullScale::Dps250);
        assert!(close(g[1], 8.75 * RAD_PER_DEG));
        // 1024 counts at +-32 g is 0.976 mg * 1024 = 0.999424 g.
        let h = RawAxes {
            x: 0,
            y: 0,
            z: 1024,
        }
        .high_g_ms2(HighGFullScale::G32);
        assert!(close(h[2], 0.999424 * G));
        assert!(close(temperature_c(-2560), 15.0));
    }
}

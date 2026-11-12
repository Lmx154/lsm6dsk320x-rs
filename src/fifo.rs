//! FIFO words: pure, in-place decoding of a byte buffer read from the FIFO, the conversions of the
//! values in it, and the expected word rate of a [`Config`].
//!
//! A FIFO word is 7 bytes: one tag byte (FIFO_DATA_OUT_TAG, 0x78) and six data bytes
//! (FIFO_DATA_OUT_X_L to FIFO_DATA_OUT_Z_H, 0x79 to 0x7E), 6.5.8, p46. [`fifo_frames`] walks a
//! caller buffer word by word. It never skips a byte and never panics, whatever the input: a word
//! it does not know comes out as [`FifoFrame::Unknown`] with its seven bytes, and a tail shorter
//! than one word comes out once as [`FifoFrame::Trailing`].
//!
//! # Tag byte
//!
//! TAG_SENSOR\[4:0\] in bits 7:3, TAG_CNT\[1:0\] (a 2-bit counter that identifies the sensor time
//! slot) in bits 2:1, and bit 0 not used (Tables 230-231, p114; sensor codes in Table 232, p114).
//! The datasheet defines no parity bit and no check on this byte, so none is made: a word is
//! unknown only when its sensor code is not one of the six this crate decodes. Bit 0 is kept in
//! [`FifoTag::unused`] so that a decoded word can be put back byte for byte.
//!
//! Confirmed on the board, 2026-11-10: the 2-bit tag counter goes up by one (mod 4) per time slot,
//! with 0 irregularities in about 5800 words; high-g words have their own tag (0x1D) and can be
//! batched alone.
//!
//! # What the datasheet does not say
//!
//! The datasheet gives the tag codes and says that the data bytes are "6 bytes of fixed data"
//! (6.5.8, p46) but does not give the layout of the temperature, timestamp and configuration-change
//! words, nor a value that marks an invalid sample. The temperature is read from the first two data
//! bytes and the timestamp from the first four; the remaining bytes are kept in `rest` and the
//! configuration-change payload is kept whole. There are no invalid-sample markers to expose.
//! Confirmed on the board, 2026-11-10: the temperature layout and the timestamp layout below. Not
//! settled: the layout of the configuration-change word.

use crate::calc;
use crate::config::{Config, FifoMode, OperatingMode, TempBatchRate, TimestampBatch};
use crate::reg;
use crate::sample::RawAxes;

/// The decoded tag byte (FIFO_DATA_OUT_TAG, Tables 230-231, p114).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoTag {
    /// TAG_SENSOR\[4:0\], bits 7:3 (Table 232, p114).
    pub sensor: u8,
    /// TAG_CNT\[1:0\], bits 2:1: the 2-bit time-slot counter.
    pub slot: u8,
    /// Bit 0, not used (Table 230, p114).
    pub unused: bool,
}

impl FifoTag {
    /// Splits a tag byte into its fields.
    pub const fn from_byte(b: u8) -> Self {
        Self {
            sensor: (b & reg::FIFO_TAG_SENSOR_MASK) >> reg::FIFO_TAG_SENSOR_SHIFT,
            slot: (b & reg::FIFO_TAG_CNT_MASK) >> reg::FIFO_TAG_CNT_SHIFT,
            unused: b & 1 != 0,
        }
    }

    /// The tag byte. Fields wider than their place are truncated.
    pub const fn to_byte(self) -> u8 {
        ((self.sensor << reg::FIFO_TAG_SENSOR_SHIFT) & reg::FIFO_TAG_SENSOR_MASK)
            | ((self.slot << reg::FIFO_TAG_CNT_SHIFT) & reg::FIFO_TAG_CNT_MASK)
            | self.unused as u8
    }
}

/// A word with three axes: gyroscope (tag 0x01), low-g accelerometer (0x02) or high-g
/// accelerometer (0x1D), each as X, Y, Z little-endian (6.5.8, p46; Table 232, p114).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoSample {
    /// The tag.
    pub tag: FifoTag,
    /// The raw axes; convert with [`RawAxes::gyro_rads`], [`RawAxes::accel_ms2`] or
    /// [`RawAxes::high_g_ms2`] and the full scale that was configured.
    pub axes: RawAxes,
}

/// A temperature word (tag 0x03, Table 232, p114).
///
/// The datasheet does not define the layout of the six data bytes. Confirmed on the board,
/// 2026-11-10: the first two data bytes are the temperature as a little-endian i16 with the scale of
/// OUT_TEMP (FIFO 26.49 C against register 26.51 C), and the other four data bytes are 0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoTemperature {
    /// The tag.
    pub tag: FifoTag,
    /// The first 16-bit field, two's complement; scale as OUT_TEMP ([`crate::sample::temperature_c`]).
    pub raw: i16,
    /// Data bytes 3 to 6, as received.
    pub rest: [u8; 4],
}

/// A timestamp word (tag 0x04, Table 232, p114).
///
/// The datasheet does not define the layout of the six data bytes. Confirmed on the board,
/// 2026-11-10: the first four data bytes are the counter, a little-endian u32; with the tick
/// computed from the frequency trim, 8 output periods measured 8487.94 us against 8487.81 us
/// expected (15 ppm).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoTimestamp {
    /// The tag.
    pub tag: FifoTag,
    /// The 32-bit counter in the first four data bytes, little-endian, in device ticks. Convert
    /// with [`timestamp_ns`].
    pub ticks: u32,
    /// Data bytes 5 and 6, as received.
    pub rest: [u8; 2],
}

/// A configuration-change word (tag 0x05, "CFG_Change", Table 232, p114): written when ODR_CHG_EN
/// is set and the accelerometer or gyroscope configuration changes (Table 38, p59; 6.5.8, p46).
///
/// The datasheet does not define the layout of its six data bytes, so they are kept whole and not
/// decoded. Observed on the board, 2026-11-10, with ODR_CHG_EN on: this word is the FIRST word of the
/// stream after every FIFO start or restart, even when nothing was changed, and a second one was
/// seen once a few words later. Its data bytes 5 and 6 (`payload[4]` and `payload[5]`) carried 0x99
/// at 960 Hz gyroscope and accelerometer and 0x88 at 480 Hz, which look like the two rate codes;
/// data bytes 1 to 4 were seen as 0, 80, 64, 0 or 64 and are not understood. The layout remains
/// undefined by the datasheet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoConfigChange {
    /// The tag.
    pub tag: FifoTag,
    /// Data bytes 1 to 6, as received.
    pub payload: [u8; 6],
}

/// A word whose sensor code this crate does not decode (including 0x00, "FIFO empty").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoUnknown {
    /// The seven bytes, tag first.
    pub raw: [u8; reg::FIFO_WORD_LEN],
}

impl FifoUnknown {
    /// The tag byte decoded.
    pub const fn tag(&self) -> FifoTag {
        FifoTag::from_byte(self.raw[0])
    }
}

/// One item of [`fifo_frames`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum FifoFrame<'a> {
    /// Gyroscope (tag 0x01).
    Gyro(FifoSample),
    /// Low-g accelerometer (tag 0x02).
    Accel(FifoSample),
    /// High-g accelerometer (tag 0x1D).
    HighG(FifoSample),
    /// Temperature (tag 0x03).
    Temperature(FifoTemperature),
    /// Timestamp (tag 0x04).
    Timestamp(FifoTimestamp),
    /// Configuration change (tag 0x05).
    ConfigChange(FifoConfigChange),
    /// A word of another kind, with its seven bytes.
    Unknown(FifoUnknown),
    /// The bytes after the last whole word, fewer than seven; reported once, at the end.
    Trailing(&'a [u8]),
}

impl FifoFrame<'_> {
    /// Decodes one word.
    pub const fn from_word(w: &[u8; reg::FIFO_WORD_LEN]) -> FifoFrame<'static> {
        let tag = FifoTag::from_byte(w[0]);
        let axes = RawAxes::from_bytes(&[w[1], w[2], w[3], w[4], w[5], w[6]]);
        match tag.sensor {
            reg::TAG_GYROSCOPE_NC => FifoFrame::Gyro(FifoSample { tag, axes }),
            reg::TAG_ACCELEROMETER_NC => FifoFrame::Accel(FifoSample { tag, axes }),
            reg::TAG_HIGH_G_ACCELEROMETER => FifoFrame::HighG(FifoSample { tag, axes }),
            reg::TAG_TEMPERATURE => FifoFrame::Temperature(FifoTemperature {
                tag,
                raw: i16::from_le_bytes([w[1], w[2]]),
                rest: [w[3], w[4], w[5], w[6]],
            }),
            reg::TAG_TIMESTAMP => FifoFrame::Timestamp(FifoTimestamp {
                tag,
                ticks: u32::from_le_bytes([w[1], w[2], w[3], w[4]]),
                rest: [w[5], w[6]],
            }),
            reg::TAG_CFG_CHANGE => FifoFrame::ConfigChange(FifoConfigChange {
                tag,
                payload: [w[1], w[2], w[3], w[4], w[5], w[6]],
            }),
            _ => FifoFrame::Unknown(FifoUnknown { raw: *w }),
        }
    }
}

/// The iterator returned by [`fifo_frames`].
#[derive(Clone, Debug)]
pub struct FifoFrames<'a> {
    rest: &'a [u8],
}

/// Decodes the FIFO bytes in `buf` in place: one item per 7-byte word, then at most one
/// [`FifoFrame::Trailing`] for the bytes left over. Every input byte ends up in exactly one item.
pub const fn fifo_frames(buf: &[u8]) -> FifoFrames<'_> {
    FifoFrames { rest: buf }
}

impl<'a> Iterator for FifoFrames<'a> {
    type Item = FifoFrame<'a>;

    fn next(&mut self) -> Option<FifoFrame<'a>> {
        if self.rest.is_empty() {
            return None;
        }
        match self.rest.split_first_chunk::<{ reg::FIFO_WORD_LEN }>() {
            Some((word, after)) => {
                self.rest = after;
                Some(FifoFrame::from_word(word))
            }
            None => {
                let tail = self.rest;
                self.rest = &[];
                Some(FifoFrame::Trailing(tail))
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let n = self.rest.len().div_ceil(reg::FIFO_WORD_LEN);
        (n, Some(n))
    }
}

impl ExactSizeIterator for FifoFrames<'_> {}

impl core::iter::FusedIterator for FifoFrames<'_> {}

/// Timestamp ticks to nanoseconds: `ticks * tick_ns`. `tick_ns` is the real tick from
/// [`calc::timestamp_tick_ns`] (21 701 ns at INTERNAL_FREQ_FINE = 0, p85, p92). Exact; at most
/// 2^32 * 2^32 fits in 64 bits.
pub const fn timestamp_ns(ticks: u32, tick_ns: u32) -> u64 {
    ticks as u64 * tick_ns as u64
}

/// Ticks elapsed from `earlier` to `later` in the 32-bit counter, correct across one wrap-around.
pub const fn timestamp_delta_ticks(earlier: u32, later: u32) -> u32 {
    later.wrapping_sub(earlier)
}

/// The expected FIFO word rate for `config`, in millihertz (words per second times 1000), rounded
/// up. `freq_fine` is the INTERNAL_FREQ_FINE value (9.54, p92) that scales all rates; use 0 for
/// the nominal rates. In the high-accuracy ODR mode the trim is not applied (the datasheet gives
/// the rate there as the table rate, +-1 % with the gyroscope on, 6.1.4 and Table 3, p13, p34) and
/// the batch rates and the high-g ODR are the table rates of the selected set (Table 22, p34):
/// measured on the MARV V2 board, 2026-11-11 (`lsm6dsk320x_dual`), with gyroscope and low-g
/// batched at 960 Hz and the high-g channel at each of its five settings, in all four sets, the
/// FIFO delivered the figure returned here within 1 %. Other batch rates in that mode follow the
/// same table by analogy and were not measured. The configuration is not validated.
///
/// Counted: the gyroscope and low-g batch rates (Table 40, p60), the high-g channel at its ODR when
/// it is batched (COUNTER_BDR_REG1 has no rate field, p62; the rate is taken as the high-g ODR),
/// the temperature batch rate (Table 42, p61) and the timestamp at `max(BDR_XL, BDR_GY)` divided by
/// the decimation (Table 42, p61). Not counted: configuration-change words (ODR_CHG_EN), which are
/// written on events. 0 in bypass mode, where the FIFO stays empty (6.5.1, p45).
pub const fn fifo_words_per_second_mhz(config: &Config, freq_fine: i8) -> u32 {
    if matches!(config.fifo.mode, FifoMode::Bypass) {
        return 0;
    }
    let (gyro, accel, high_g_mhz) = match config.mode {
        OperatingMode::HighAccuracyOdr(set) => (
            config.fifo.gyro.haodr_mhz(set),
            config.fifo.accel.haodr_mhz(set),
            match config.high_g {
                Some(h) => h.odr.haodr_mhz(set),
                None => 0,
            },
        ),
        _ => (
            config.fifo.gyro.nominal_mhz(),
            config.fifo.accel.nominal_mhz(),
            match config.high_g {
                Some(h) => h.odr.nominal_mhz(),
                None => 0,
            },
        ),
    };
    let high_g = if config.fifo.high_g { high_g_mhz } else { 0 };
    let temperature = match config.fifo.temperature {
        TempBatchRate::Off => 0,
        TempBatchRate::Hz1_875 => 1_875,
        TempBatchRate::Hz15 => 15_000,
        TempBatchRate::Hz60 => 60_000,
    };
    let fastest = if gyro > accel { gyro } else { accel };
    let timestamp = match config.fifo.timestamp {
        TimestampBatch::Off => 0,
        TimestampBatch::Dec1 => fastest,
        TimestampBatch::Dec8 => fastest.div_ceil(8),
        TimestampBatch::Dec32 => fastest.div_ceil(32),
    };
    // bound: each term is at most 7_680_000 and there are five, so the sum is below 4e7.
    let total = gyro + accel + high_g + temperature + timestamp;
    let freq_fine = if config.mode.is_haodr() { 0 } else { freq_fine };
    calc::scale_rate_mhz(total, freq_fine)
}

/// [`fifo_words_per_second_mhz`] in whole words per second, rounded up: the figure for sizing
/// buffers and watermarks.
pub const fn fifo_words_per_second(config: &Config, freq_fine: i8) -> u32 {
    fifo_words_per_second_mhz(config, freq_fine).div_ceil(1000)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{BatchRate, FifoConfig, HighGConfig, HighGOdr};
    use proptest::prelude::*;
    use std::vec::Vec;

    fn one(w: [u8; 7]) -> FifoFrame<'static> {
        let mut it = fifo_frames(&w);
        let f = it.next().unwrap();
        assert_eq!(it.next(), None);
        assert!(!matches!(f, FifoFrame::Trailing(_)));
        assert_eq!(f, FifoFrame::from_word(&w));
        FifoFrame::from_word(&w)
    }

    // Tag byte layout: sensor in bits 7:3, counter in bits 2:1, Tables 230-231, p114.
    // Sensor codes: Table 232, p114.
    #[test]
    fn tag_byte_fields() {
        // 0x1D high-g << 3 = 0xE8, counter 3 << 1 = 6, bit 0 set.
        let t = FifoTag::from_byte(0xE8 | 0x06 | 0x01);
        assert_eq!(
            t,
            FifoTag {
                sensor: 0x1D,
                slot: 3,
                unused: true
            }
        );
        assert_eq!(t.to_byte(), 0xEF);
        assert_eq!(
            FifoTag::from_byte(0x0A),
            FifoTag {
                sensor: 1,
                slot: 1,
                unused: false
            }
        );
        for b in 0..=255u8 {
            assert_eq!(FifoTag::from_byte(b).to_byte(), b);
        }
    }

    #[test]
    fn golden_gyro() {
        // Tag 0x01 << 3 = 0x08, counter 2 << 1 = 4: 0x0C. X = 1, Y = -1, Z = i16::MIN.
        let w = [0x0C, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80];
        assert_eq!(
            one(w),
            FifoFrame::Gyro(FifoSample {
                tag: FifoTag {
                    sensor: 1,
                    slot: 2,
                    unused: false
                },
                axes: RawAxes {
                    x: 1,
                    y: -1,
                    z: i16::MIN
                }
            })
        );
    }

    #[test]
    fn golden_accel() {
        // Tag 0x02 << 3 = 0x10, counter 0.
        let w = [0x10, 0x34, 0x12, 0x00, 0x80, 0xFF, 0x7F];
        assert_eq!(
            one(w),
            FifoFrame::Accel(FifoSample {
                tag: FifoTag {
                    sensor: 2,
                    slot: 0,
                    unused: false
                },
                axes: RawAxes {
                    x: 0x1234,
                    y: i16::MIN,
                    z: i16::MAX
                }
            })
        );
    }

    #[test]
    fn golden_high_g() {
        // Tag 0x1D << 3 = 0xE8, counter 1 << 1 = 2: 0xEA.
        let w = [0xEA, 0x00, 0x04, 0x00, 0xFC, 0x10, 0x00];
        assert_eq!(
            one(w),
            FifoFrame::HighG(FifoSample {
                tag: FifoTag {
                    sensor: 0x1D,
                    slot: 1,
                    unused: false
                },
                axes: RawAxes {
                    x: 1024,
                    y: -1024,
                    z: 16
                }
            })
        );
    }

    #[test]
    fn golden_temperature() {
        // Tag 0x03 << 3 = 0x18, counter 3: 0x1E.
        let w = [0x1E, 0x00, 0xF6, 0xAA, 0xBB, 0xCC, 0xDD];
        assert_eq!(
            one(w),
            FifoFrame::Temperature(FifoTemperature {
                tag: FifoTag {
                    sensor: 3,
                    slot: 3,
                    unused: false
                },
                raw: -2560,
                rest: [0xAA, 0xBB, 0xCC, 0xDD]
            })
        );
    }

    #[test]
    fn golden_timestamp() {
        // Tag 0x04 << 3 = 0x20.
        let w = [0x20, 0x78, 0x56, 0x34, 0x12, 0x01, 0x02];
        assert_eq!(
            one(w),
            FifoFrame::Timestamp(FifoTimestamp {
                tag: FifoTag {
                    sensor: 4,
                    slot: 0,
                    unused: false
                },
                ticks: 0x1234_5678,
                rest: [1, 2]
            })
        );
    }

    #[test]
    fn golden_config_change() {
        // Tag 0x05 << 3 = 0x28, counter 1: 0x2A.
        let w = [0x2A, 1, 2, 3, 4, 5, 6];
        assert_eq!(
            one(w),
            FifoFrame::ConfigChange(FifoConfigChange {
                tag: FifoTag {
                    sensor: 5,
                    slot: 1,
                    unused: false
                },
                payload: [1, 2, 3, 4, 5, 6]
            })
        );
    }

    #[test]
    fn golden_unknown_and_empty() {
        // 0x00 FIFO empty, 0x06 accelerometer NC_T_2, 0x1F FSM results: not decoded here.
        for tag in [0x00u8, 0x06 << 3, 0x1F << 3, 0xFF] {
            let w = [tag, 9, 8, 7, 6, 5, 4];
            let f = fifo_frames(&w).next().unwrap();
            assert_eq!(f, FifoFrame::Unknown(FifoUnknown { raw: w }));
            if let FifoFrame::Unknown(u) = f {
                assert_eq!(u.tag(), FifoTag::from_byte(tag));
            }
        }
    }

    // The datasheet defines no parity: bit 0 of the tag never changes the kind of a word.
    #[test]
    fn bit_zero_does_not_change_the_kind() {
        for sensor in [1u8, 2, 3, 4, 5, 0x1D] {
            let wa = [sensor << 3, 1, 2, 3, 4, 5, 6];
            let a = fifo_frames(&wa).next().unwrap();
            let wb = [sensor << 3 | 1, 1, 2, 3, 4, 5, 6];
            let b = fifo_frames(&wb).next().unwrap();
            assert_eq!(core::mem::discriminant(&a), core::mem::discriminant(&b));
            assert!(!matches!(b, FifoFrame::Unknown(_)));
        }
    }

    #[test]
    fn trailing_bytes_are_reported_once() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&[0x08, 1, 0, 0, 0, 0, 0]);
        buf.extend_from_slice(&[0x10, 2, 0, 0, 0, 0, 0]);
        buf.extend_from_slice(&[0x20, 0xFF, 0xEE]);
        let frames: Vec<_> = fifo_frames(&buf).collect();
        assert_eq!(frames.len(), 3);
        assert!(matches!(frames[0], FifoFrame::Gyro(_)));
        assert!(matches!(frames[1], FifoFrame::Accel(_)));
        assert_eq!(frames[2], FifoFrame::Trailing(&[0x20, 0xFF, 0xEE]));
        assert_eq!(fifo_frames(&buf).len(), 3);
        // Empty input, one byte, six bytes, seven bytes.
        assert_eq!(fifo_frames(&[]).count(), 0);
        assert_eq!(
            fifo_frames(&[7]).collect::<Vec<_>>(),
            std::vec![FifoFrame::Trailing(&[7])]
        );
        assert_eq!(fifo_frames(&[0; 6]).count(), 1);
        assert_eq!(fifo_frames(&[0; 7]).count(), 1);
        assert_eq!(fifo_frames(&[0; 8]).count(), 2);
        let mut it = fifo_frames(&[1, 2]);
        assert!(it.next().is_some());
        assert_eq!(it.next(), None);
        assert_eq!(it.next(), None);
    }

    #[test]
    fn timestamp_conversions() {
        // 46080 Hz counter: 21 701 ns per tick at FREQ_FINE = 0 (p85, p92).
        let tick = calc::timestamp_tick_ns(0);
        assert_eq!(tick, 21_701);
        assert_eq!(timestamp_ns(0, tick), 0);
        assert_eq!(timestamp_ns(46_080, tick), 46_080 * 21_701);
        assert_eq!(
            timestamp_ns(u32::MAX, u32::MAX),
            u64::from(u32::MAX) * u64::from(u32::MAX)
        );
        assert_eq!(timestamp_delta_ticks(10, 15), 5);
        assert_eq!(timestamp_delta_ticks(u32::MAX - 1, 3), 5);
    }

    fn base() -> Config {
        Config::new().with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Continuous)
                .with_gyro(BatchRate::Hz960)
                .with_accel(BatchRate::Hz960),
        )
    }

    // Measured on the MARV V2 board, 2026-11-11 (hil/logs/lsm6dsk320x_dual-20261111-184219.log,
    // section D3): gyroscope and low-g batched at 960 and the high-g channel batched, the FIFO
    // words per second for each ODR set and high-g setting. The reported figure is the sum of the
    // three table rates of the set (Table 22, p34) and agrees with the measurement within 1 %.
    #[test]
    fn words_per_second_follows_the_haodr_set() {
        use crate::{HaodrSet, OperatingMode};
        // (set, [measured words/s for the high-g settings 480, 960, 1920, 3840, 7680])
        let measured = [
            (HaodrSet::Base960, [2394.0, 2873.6, 3831.1, 5746.7, 9576.5]),
            (HaodrSet::Base1000, [2498.0, 2998.5, 3998.3, 5997.1, 9999.1]),
            (HaodrSet::Base800, [2000.5, 2401.8, 3203.9, 4803.5, 8008.2]),
            (HaodrSet::Base833, [2084.8, 2502.3, 3337.0, 5004.3, 8340.7]),
        ];
        let rates = [
            HighGOdr::Hz480,
            HighGOdr::Hz960,
            HighGOdr::Hz1920,
            HighGOdr::Hz3840,
            HighGOdr::Hz7680,
        ];
        for (set, words) in measured {
            for (hodr, want) in rates.into_iter().zip(words) {
                let c = base()
                    .with_mode(OperatingMode::HighAccuracyOdr(set))
                    .with_high_g(Some(HighGConfig::new().with_odr(hodr)))
                    .with_fifo(base().fifo.with_high_g(true));
                let got = fifo_words_per_second_mhz(&c, -3) as f32 / 1000.0;
                assert!(
                    (got / want - 1.0).abs() <= 0.01,
                    "{set:?} {hodr:?}: reported {got}, measured {want}"
                );
            }
        }
        // The sum for set 800 at high-g 1920: 800 + 800 + 1600.
        let c = base()
            .with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base800))
            .with_high_g(Some(HighGConfig::new().with_odr(HighGOdr::Hz1920)))
            .with_fifo(base().fifo.with_high_g(true));
        assert_eq!(fifo_words_per_second_mhz(&c, 0), 3_200_000);
    }

    // Hand-computed: words per second = sum of the batch rates (Tables 40 and 42, p60-61).
    #[test]
    fn words_per_second() {
        // Bypass: nothing, whatever the rates (6.5.1, p45).
        let bypass = base().with_fifo(base().fifo.with_mode(FifoMode::Bypass));
        assert_eq!(fifo_words_per_second_mhz(&bypass, 0), 0);
        // 960 + 960 words/s.
        assert_eq!(fifo_words_per_second_mhz(&base(), 0), 1_920_000);
        assert_eq!(fifo_words_per_second(&base(), 0), 1920);
        // Gyroscope only at 7.68 kHz.
        let g = Config::new().with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Fifo)
                .with_gyro(BatchRate::Hz7680),
        );
        assert_eq!(fifo_words_per_second(&g, 0), 7680);
        // Plus high-g at 3840, timestamp every 8th of 960, temperature 60: 5940 words/s.
        let hg = HighGConfig::new().with_odr(HighGOdr::Hz3840);
        let c = base().with_high_g(Some(hg)).with_fifo(
            base()
                .fifo
                .with_high_g(true)
                .with_timestamp(TimestampBatch::Dec8)
                .with_temperature(TempBatchRate::Hz60),
        );
        assert_eq!(fifo_words_per_second_mhz(&c, 0), 5_940_000);
        // High-g batching with the channel off adds nothing.
        let off = c.with_high_g(None);
        assert_eq!(fifo_words_per_second(&off, 0), 1920 + 120 + 60);
        // Timestamp decimations of max(BDR) = 960: 960, 120, 30.
        for (dec, n) in [
            (TimestampBatch::Dec1, 960),
            (TimestampBatch::Dec8, 120),
            (TimestampBatch::Dec32, 30),
        ] {
            let c = base().with_fifo(base().fifo.with_timestamp(dec));
            assert_eq!(fifo_words_per_second(&c, 0), 1920 + n);
        }
        // 1.875 Hz accelerometer only: 1875 mHz, rounded up to 2 words/s.
        let slow = Config::new().with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Continuous)
                .with_accel(BatchRate::Hz1_875),
        );
        assert_eq!(fifo_words_per_second_mhz(&slow, 0), 1_875);
        assert_eq!(fifo_words_per_second(&slow, 0), 2);
        // The trim scales the normal mode (7680 * 0.9961 = 7650.048) but not the high-accuracy
        // ODR mode (6.1.4 and Table 3, p13, p34).
        assert_eq!(fifo_words_per_second_mhz(&g, -3), 7_650_048);
        let haodr = g.with_mode(crate::OperatingMode::HighAccuracyOdr(
            crate::HaodrSet::Base960,
        ));
        assert_eq!(fifo_words_per_second_mhz(&haodr, -3), 7_680_000);
        // Timestamp at 1/32 of 1.875 Hz: 58.59 mHz, rounded up to 59.
        let ts = slow.with_fifo(slow.fifo.with_timestamp(TimestampBatch::Dec32));
        assert_eq!(fifo_words_per_second_mhz(&ts, 0), 1_875 + 59);
        // FREQ_FINE = -14 scales by 1 - 0.0182 (p92): 1_920_000 * 0.9818 = 1_885_056 mHz.
        assert_eq!(fifo_words_per_second_mhz(&base(), -14), 1_885_056);
        assert_eq!(fifo_words_per_second(&base(), -14), 1886);
        // The configuration-change words are not counted.
        let chg = base().with_fifo(base().fifo.with_odr_change(true));
        assert_eq!(fifo_words_per_second(&chg, 0), 1920);
        // Nothing batched.
        assert_eq!(fifo_words_per_second(&Config::new(), 0), 0);
    }

    /// Test-only encoder: the inverse of the decoder.
    fn encode(f: &FifoFrame<'_>, out: &mut Vec<u8>) {
        let axes = |a: &RawAxes| {
            let mut d = [0u8; 6];
            d[0..2].copy_from_slice(&a.x.to_le_bytes());
            d[2..4].copy_from_slice(&a.y.to_le_bytes());
            d[4..6].copy_from_slice(&a.z.to_le_bytes());
            d
        };
        let mut put = |tag: FifoTag, d: [u8; 6]| {
            out.push(tag.to_byte());
            out.extend_from_slice(&d);
        };
        match f {
            FifoFrame::Gyro(s) | FifoFrame::Accel(s) | FifoFrame::HighG(s) => {
                put(s.tag, axes(&s.axes))
            }
            FifoFrame::Temperature(t) => {
                let r = t.raw.to_le_bytes();
                put(
                    t.tag,
                    [r[0], r[1], t.rest[0], t.rest[1], t.rest[2], t.rest[3]],
                )
            }
            FifoFrame::Timestamp(t) => {
                let r = t.ticks.to_le_bytes();
                put(t.tag, [r[0], r[1], r[2], r[3], t.rest[0], t.rest[1]])
            }
            FifoFrame::ConfigChange(c) => put(c.tag, c.payload),
            FifoFrame::Unknown(u) => out.extend_from_slice(&u.raw),
            FifoFrame::Trailing(t) => out.extend_from_slice(t),
        }
    }

    fn arb_axes() -> impl Strategy<Value = RawAxes> {
        (any::<i16>(), any::<i16>(), any::<i16>()).prop_map(|(x, y, z)| RawAxes { x, y, z })
    }

    fn arb_tag(sensor: u8) -> impl Strategy<Value = FifoTag> {
        (0u8..4, any::<bool>()).prop_map(move |(slot, unused)| FifoTag {
            sensor,
            slot,
            unused,
        })
    }

    fn arb_frame() -> impl Strategy<Value = FifoFrame<'static>> {
        let sample =
            |sensor| (arb_tag(sensor), arb_axes()).prop_map(|(tag, axes)| FifoSample { tag, axes });
        prop_oneof![
            sample(reg::TAG_GYROSCOPE_NC).prop_map(FifoFrame::Gyro),
            sample(reg::TAG_ACCELEROMETER_NC).prop_map(FifoFrame::Accel),
            sample(reg::TAG_HIGH_G_ACCELEROMETER).prop_map(FifoFrame::HighG),
            (
                arb_tag(reg::TAG_TEMPERATURE),
                any::<i16>(),
                any::<[u8; 4]>()
            )
                .prop_map(|(tag, raw, rest)| FifoFrame::Temperature(FifoTemperature {
                    tag,
                    raw,
                    rest
                })),
            (arb_tag(reg::TAG_TIMESTAMP), any::<u32>(), any::<[u8; 2]>()).prop_map(
                |(tag, ticks, rest)| FifoFrame::Timestamp(FifoTimestamp { tag, ticks, rest })
            ),
            (arb_tag(reg::TAG_CFG_CHANGE), any::<[u8; 6]>()).prop_map(|(tag, payload)| {
                FifoFrame::ConfigChange(FifoConfigChange { tag, payload })
            }),
        ]
    }

    proptest! {
        #[test]
        fn prop_arbitrary_bytes_are_all_accounted_for(buf in proptest::collection::vec(any::<u8>(), 0..600)) {
            let mut words = 0usize;
            let mut trailing = 0usize;
            let mut last_was_trailing = false;
            let mut it = fifo_frames(&buf);
            prop_assert_eq!(it.len(), buf.len().div_ceil(7));
            let mut out = Vec::new();
            for f in it.by_ref() {
                prop_assert!(!last_was_trailing, "item after the trailing bytes");
                match f {
                    FifoFrame::Trailing(t) => {
                        prop_assert!(!t.is_empty() && t.len() < 7);
                        trailing += t.len();
                        last_was_trailing = true;
                    }
                    FifoFrame::Unknown(_) => words += 1,
                    _ => words += 1,
                }
                encode(&f, &mut out);
            }
            prop_assert_eq!(words * 7 + trailing, buf.len());
            prop_assert_eq!(trailing, buf.len() % 7);
            // Re-encoding every item gives the input back, byte for byte.
            prop_assert_eq!(&out, &buf);
            prop_assert_eq!(it.next(), None);
        }

        #[test]
        fn prop_known_tags_decode_to_known_kinds(w in any::<[u8; 7]>()) {
            let tag = FifoTag::from_byte(w[0]);
            let f = fifo_frames(&w).next().unwrap();
            let known = matches!(tag.sensor, 1 | 2 | 3 | 4 | 5 | 0x1D);
            prop_assert_eq!(!matches!(f, FifoFrame::Unknown(_)), known);
        }

        #[test]
        fn prop_encoder_decoder_round_trip(frames in proptest::collection::vec(arb_frame(), 0..40)) {
            let mut buf = Vec::new();
            for f in &frames {
                encode(f, &mut buf);
            }
            prop_assert_eq!(buf.len(), frames.len() * 7);
            let back: Vec<_> = fifo_frames(&buf).collect();
            prop_assert_eq!(back, frames);
        }

        #[test]
        fn prop_timestamp_ns_matches_wide_multiplication(t in any::<u32>(), tick in 1u32..30_000) {
            prop_assert_eq!(timestamp_ns(t, tick), u64::from(t) * u64::from(tick));
            prop_assert_eq!(timestamp_delta_ticks(t, t), 0);
        }

        #[test]
        fn prop_words_per_second_is_monotone_in_the_rates(
            g in 0u8..13, a in 0u8..13, ff in any::<i8>()
        ) {
            let rate = |c: u8| [
                BatchRate::Off, BatchRate::Hz1_875, BatchRate::Hz7_5, BatchRate::Hz15,
                BatchRate::Hz30, BatchRate::Hz60, BatchRate::Hz120, BatchRate::Hz240,
                BatchRate::Hz480, BatchRate::Hz960, BatchRate::Hz1920, BatchRate::Hz3840,
                BatchRate::Hz7680,
            ][usize::from(c)];
            let mk = |g, a| Config::new().with_fifo(
                FifoConfig::new().with_mode(FifoMode::Continuous).with_gyro(rate(g)).with_accel(rate(a)),
            );
            let w = fifo_words_per_second_mhz(&mk(g, a), ff);
            if g < 12 {
                prop_assert!(fifo_words_per_second_mhz(&mk(g + 1, a), ff) >= w);
            }
            prop_assert!(w <= 2 * 8_960_000);
            let whole = fifo_words_per_second(&mk(g, a), ff);
            prop_assert!(u64::from(whole) * 1000 >= u64::from(w));
            prop_assert!(u64::from(whole) * 1000 < u64::from(w) + 1000);
        }
    }
}

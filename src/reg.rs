//! Register addresses and bit fields (datasheet DS15060 Rev 1).
//!
//! Register names follow the datasheet. Page numbers are the printed "page N/210" numbers; the
//! address map is Table 26, p52-55. Bits the datasheet says "must be set to 0 for the correct
//! operation" are not named: every image built by this crate leaves them 0.
//!
//! A single-bit field is a mask (`1 << n`). A multi-bit field has a `_SHIFT` and a `_MASK`
//! (already shifted). Codes of a field are named after the field.
#![allow(missing_docs)]

pub const FUNC_CFG_ACCESS: u8 = 0x01;
pub const PIN_CTRL: u8 = 0x02;
pub const IF_CFG: u8 = 0x03;
pub const ODR_TRIG_CFG: u8 = 0x06;
pub const FIFO_CTRL1: u8 = 0x07;
pub const FIFO_CTRL2: u8 = 0x08;
pub const FIFO_CTRL3: u8 = 0x09;
pub const FIFO_CTRL4: u8 = 0x0A;
pub const COUNTER_BDR_REG1: u8 = 0x0B;
pub const COUNTER_BDR_REG2: u8 = 0x0C;
pub const INT1_CTRL: u8 = 0x0D;
pub const INT2_CTRL: u8 = 0x0E;
pub const WHO_AM_I: u8 = 0x0F;
pub const CTRL1: u8 = 0x10;
pub const CTRL2: u8 = 0x11;
pub const CTRL3: u8 = 0x12;
pub const CTRL4: u8 = 0x13;
pub const CTRL5: u8 = 0x14;
pub const CTRL6: u8 = 0x15;
pub const CTRL7: u8 = 0x16;
pub const CTRL8: u8 = 0x17;
pub const CTRL9: u8 = 0x18;
pub const CTRL10: u8 = 0x19;
pub const CTRL_STATUS: u8 = 0x1A;
pub const FIFO_STATUS1: u8 = 0x1B;
pub const FIFO_STATUS2: u8 = 0x1C;
pub const ALL_INT_SRC: u8 = 0x1D;
pub const STATUS_REG: u8 = 0x1E;
pub const OUT_TEMP_L: u8 = 0x20;
pub const OUT_TEMP_H: u8 = 0x21;
pub const OUTX_L_G: u8 = 0x22;
pub const OUTX_H_G: u8 = 0x23;
pub const OUTY_L_G: u8 = 0x24;
pub const OUTY_H_G: u8 = 0x25;
pub const OUTZ_L_G: u8 = 0x26;
pub const OUTZ_H_G: u8 = 0x27;
pub const OUTX_L_A: u8 = 0x28;
pub const OUTX_H_A: u8 = 0x29;
pub const OUTY_L_A: u8 = 0x2A;
pub const OUTY_H_A: u8 = 0x2B;
pub const OUTZ_L_A: u8 = 0x2C;
pub const OUTZ_H_A: u8 = 0x2D;
pub const UI_OUTX_L_G_OIS_EIS: u8 = 0x2E;
pub const UI_OUTX_H_G_OIS_EIS: u8 = 0x2F;
pub const UI_OUTY_L_G_OIS_EIS: u8 = 0x30;
pub const UI_OUTY_H_G_OIS_EIS: u8 = 0x31;
pub const UI_OUTZ_L_G_OIS_EIS: u8 = 0x32;
pub const UI_OUTZ_H_G_OIS_EIS: u8 = 0x33;
pub const UI_OUTX_L_A_OIS_HG: u8 = 0x34;
pub const UI_OUTX_H_A_OIS_HG: u8 = 0x35;
pub const UI_OUTY_L_A_OIS_HG: u8 = 0x36;
pub const UI_OUTY_H_A_OIS_HG: u8 = 0x37;
pub const UI_OUTZ_L_A_OIS_HG: u8 = 0x38;
pub const UI_OUTZ_H_A_OIS_HG: u8 = 0x39;
pub const TIMESTAMP0: u8 = 0x40;
pub const TIMESTAMP1: u8 = 0x41;
pub const TIMESTAMP2: u8 = 0x42;
pub const TIMESTAMP3: u8 = 0x43;
pub const UI_STATUS_REG_OIS: u8 = 0x44;
pub const WAKE_UP_SRC: u8 = 0x45;
pub const TAP_SRC: u8 = 0x46;
pub const D6D_SRC: u8 = 0x47;
pub const STATUS_CONTROLLER_MAINPAGE: u8 = 0x48;
pub const EMB_FUNC_STATUS_MAINPAGE: u8 = 0x49;
pub const FSM_STATUS_MAINPAGE: u8 = 0x4A;
pub const MLC_STATUS_MAINPAGE: u8 = 0x4B;
pub const HG_WAKE_UP_SRC: u8 = 0x4C;
pub const CTRL2_XL_HG: u8 = 0x4D;
pub const CTRL1_XL_HG: u8 = 0x4E;
pub const INTERNAL_FREQ_FINE: u8 = 0x4F;
pub const FUNCTIONS_ENABLE: u8 = 0x50;
pub const HG_FUNCTIONS_ENABLE: u8 = 0x52;
pub const HG_WAKE_UP_THS: u8 = 0x53;
pub const INACTIVITY_DUR: u8 = 0x54;
pub const INACTIVITY_THS: u8 = 0x55;
pub const TAP_CFG0: u8 = 0x56;
pub const TAP_CFG1: u8 = 0x57;
pub const TAP_CFG2: u8 = 0x58;
pub const TAP_THS_6D: u8 = 0x59;
pub const TAP_DUR: u8 = 0x5A;
pub const WAKE_UP_THS: u8 = 0x5B;
pub const WAKE_UP_DUR: u8 = 0x5C;
pub const FREE_FALL: u8 = 0x5D;
pub const MD1_CFG: u8 = 0x5E;
pub const MD2_CFG: u8 = 0x5F;
pub const HAODR_CFG: u8 = 0x62;
pub const EMB_FUNC_CFG: u8 = 0x63;
pub const UI_HANDSHAKE_CTRL: u8 = 0x64;
pub const UI_IF2_SHARED_0: u8 = 0x65;
pub const UI_IF2_SHARED_1: u8 = 0x66;
pub const UI_IF2_SHARED_2: u8 = 0x67;
pub const UI_IF2_SHARED_3: u8 = 0x68;
pub const UI_IF2_SHARED_4: u8 = 0x69;
pub const UI_IF2_SHARED_5: u8 = 0x6A;
pub const CTRL_EIS: u8 = 0x6B;
pub const XL_HG_X_OFS_USR: u8 = 0x6C;
pub const XL_HG_Y_OFS_USR: u8 = 0x6D;
pub const XL_HG_Z_OFS_USR: u8 = 0x6E;
pub const UI_INT_OIS: u8 = 0x6F;
pub const UI_CTRL1_OIS: u8 = 0x70;
pub const UI_CTRL2_OIS: u8 = 0x71;
pub const UI_CTRL3_OIS: u8 = 0x72;
pub const X_OFS_USR: u8 = 0x73;
pub const Y_OFS_USR: u8 = 0x74;
pub const Z_OFS_USR: u8 = 0x75;
/// Table 26, p55. FIFO word: one tag byte, then X, Y, Z as little-endian 16-bit values (6.5.8, p46).
pub const FIFO_DATA_OUT_TAG: u8 = 0x78;
pub const FIFO_DATA_OUT_X_L: u8 = 0x79;
pub const FIFO_DATA_OUT_X_H: u8 = 0x7A;
pub const FIFO_DATA_OUT_Y_L: u8 = 0x7B;
pub const FIFO_DATA_OUT_Y_H: u8 = 0x7C;
pub const FIFO_DATA_OUT_Z_L: u8 = 0x7D;
pub const FIFO_DATA_OUT_Z_H: u8 = 0x7E;

/// Fixed WHO_AM_I value (9.13, Table 51, p64).
pub const WHO_AM_I_VALUE: u8 = 0x75;

/// First bit of every SPI transaction: set for a read (p22).
pub const SPI_READ: u8 = 0x80;

/// Reset values of registers the configuration layer writes (Table 26, p52-54).
pub const PIN_CTRL_RESET: u8 = 0x23;
pub const CTRL3_RESET: u8 = 0x44;
pub const CTRL6_RESET: u8 = 0x08;

/// FUNC_CFG_ACCESS bits (9.1, Tables 27-28, p56).
pub const FUNC_CFG_EMB_FUNC_REG_ACCESS: u8 = 1 << 7;
pub const FUNC_CFG_SHUB_REG_ACCESS: u8 = 1 << 6;
pub const FUNC_CFG_FSM_WR_CTRL_EN: u8 = 1 << 3;
/// Global reset of the device.
pub const FUNC_CFG_SW_POR: u8 = 1 << 2;
pub const FUNC_CFG_IF2_RESET: u8 = 1 << 1;
pub const FUNC_CFG_OIS_CTRL_FROM_UI: u8 = 1 << 0;

/// PIN_CTRL bits (9.2, Tables 29-30, p57). Not reset by a software reset. Reset value 0x23.
pub const PIN_CTRL_OIS_PU_DIS: u8 = 1 << 7;
pub const PIN_CTRL_SDO_PU_EN: u8 = 1 << 6;
/// Reset value 1: global (power-on) reset after the I3C "reset whole chip" pattern.
pub const PIN_CTRL_IBHR_POR_EN: u8 = 1 << 5;
/// IO_PAD_STRENGTH\[1:0\], drive strength of the interrupt pads. Reset value 0b11.
pub const PIN_CTRL_IO_PAD_STRENGTH_SHIFT: u8 = 0;
pub const PIN_CTRL_IO_PAD_STRENGTH_MASK: u8 = 0b11;
/// Lowest strength (recommended for VDDIO >= 3.0 V).
pub const IO_PAD_STRENGTH_LOWEST: u8 = 0b00;
/// Intermediate strength (VDDIO >= 2.0 V and < 3 V).
pub const IO_PAD_STRENGTH_INTERMEDIATE: u8 = 0b01;
/// 0b10 is reserved. Highest strength (recommended for VDDIO < 2 V).
pub const IO_PAD_STRENGTH_HIGHEST: u8 = 0b11;

/// IF_CFG bits (9.3, Tables 31-32, p58). Not reset by a software reset.
pub const IF_CFG_I2C_I3C_DISABLE: u8 = 1 << 0;
pub const IF_CFG_SIM: u8 = 1 << 2;
pub const IF_CFG_PP_OD: u8 = 1 << 3;
pub const IF_CFG_H_LACTIVE: u8 = 1 << 4;
pub const IF_CFG_ASF_CTRL: u8 = 1 << 5;
pub const IF_CFG_SHUB_PU_EN: u8 = 1 << 6;
pub const IF_CFG_SDA_PU_EN: u8 = 1 << 7;

// ODR_TRIG_CFG (9.4, Tables 33-34, p58) holds ODR_TRIG_NODR[7:0]; ODR-triggered mode is not used.

/// FIFO_CTRL1 (9.5, Tables 35-36, p59): WTM\[7:0\], one LSB is one FIFO word (tag + 6 bytes).
pub const FIFO_CTRL1_WTM_MASK: u8 = 0xFF;

/// FIFO_CTRL2 bits (9.6, Tables 37-38, p59).
pub const FIFO_CTRL2_STOP_ON_WTM: u8 = 1 << 7;
pub const FIFO_CTRL2_FIFO_COMPR_RT_EN: u8 = 1 << 6;
pub const FIFO_CTRL2_ODR_CHG_EN: u8 = 1 << 4;
pub const FIFO_CTRL2_UNCOMPR_RATE_SHIFT: u8 = 1;
pub const FIFO_CTRL2_UNCOMPR_RATE_MASK: u8 = 0b11 << 1;

/// FIFO_CTRL3 fields (9.7, Tables 39-40, p60): BDR_XL\[3:0\] low nibble, BDR_GY\[3:0\] high nibble.
pub const FIFO_CTRL3_BDR_XL_SHIFT: u8 = 0;
pub const FIFO_CTRL3_BDR_XL_MASK: u8 = 0x0F;
pub const FIFO_CTRL3_BDR_GY_SHIFT: u8 = 4;
pub const FIFO_CTRL3_BDR_GY_MASK: u8 = 0xF0;
/// Batch data rate codes of BDR_XL and BDR_GY (Table 40, p60). 0b1101-0b1111 are reserved.
pub const BDR_OFF: u8 = 0b0000;
pub const BDR_1_875_HZ: u8 = 0b0001;
pub const BDR_7_5_HZ: u8 = 0b0010;
pub const BDR_15_HZ: u8 = 0b0011;
pub const BDR_30_HZ: u8 = 0b0100;
pub const BDR_60_HZ: u8 = 0b0101;
pub const BDR_120_HZ: u8 = 0b0110;
pub const BDR_240_HZ: u8 = 0b0111;
pub const BDR_480_HZ: u8 = 0b1000;
pub const BDR_960_HZ: u8 = 0b1001;
pub const BDR_1920_HZ: u8 = 0b1010;
pub const BDR_3840_HZ: u8 = 0b1011;
pub const BDR_7680_HZ: u8 = 0b1100;

/// FIFO_CTRL4 fields (9.8, Tables 41-42, p61).
pub const FIFO_CTRL4_DEC_TS_BATCH_SHIFT: u8 = 6;
pub const FIFO_CTRL4_DEC_TS_BATCH_MASK: u8 = 0b11 << 6;
/// DEC_TS_BATCH codes: timestamp not batched; decimation 1, 8, 32 of max(BDR_XL, BDR_GY).
pub const DEC_TS_BATCH_OFF: u8 = 0b00;
pub const DEC_TS_BATCH_1: u8 = 0b01;
pub const DEC_TS_BATCH_8: u8 = 0b10;
pub const DEC_TS_BATCH_32: u8 = 0b11;
pub const FIFO_CTRL4_ODR_T_BATCH_SHIFT: u8 = 4;
pub const FIFO_CTRL4_ODR_T_BATCH_MASK: u8 = 0b11 << 4;
/// ODR_T_BATCH codes: temperature batch rate.
pub const ODR_T_BATCH_OFF: u8 = 0b00;
pub const ODR_T_BATCH_1_875_HZ: u8 = 0b01;
pub const ODR_T_BATCH_15_HZ: u8 = 0b10;
pub const ODR_T_BATCH_60_HZ: u8 = 0b11;
pub const FIFO_CTRL4_G_EIS_FIFO_EN: u8 = 1 << 3;
pub const FIFO_CTRL4_FIFO_MODE_SHIFT: u8 = 0;
pub const FIFO_CTRL4_FIFO_MODE_MASK: u8 = 0b111;
/// FIFO_MODE codes (p61). 0b101 is reserved.
pub const FIFO_MODE_BYPASS: u8 = 0b000;
pub const FIFO_MODE_FIFO: u8 = 0b001;
pub const FIFO_MODE_CONTINUOUS_WTM_TO_FULL: u8 = 0b010;
pub const FIFO_MODE_CONTINUOUS_TO_FIFO: u8 = 0b011;
pub const FIFO_MODE_BYPASS_TO_CONTINUOUS: u8 = 0b100;
pub const FIFO_MODE_CONTINUOUS: u8 = 0b110;
pub const FIFO_MODE_BYPASS_TO_FIFO: u8 = 0b111;

/// COUNTER_BDR_REG1 fields (9.9, Tables 43-44, p62).
pub const COUNTER_BDR_REG1_TRIG_COUNTER_BDR_SHIFT: u8 = 5;
pub const COUNTER_BDR_REG1_TRIG_COUNTER_BDR_MASK: u8 = 0b11 << 5;
/// TRIG_COUNTER_BDR codes: low-g, gyroscope, gyroscope EIS, high-g batch event.
pub const TRIG_COUNTER_BDR_XL: u8 = 0b00;
pub const TRIG_COUNTER_BDR_G: u8 = 0b01;
pub const TRIG_COUNTER_BDR_G_EIS: u8 = 0b10;
pub const TRIG_COUNTER_BDR_XL_HG: u8 = 0b11;
/// Enables batching of the high-g accelerometer in the FIFO.
pub const COUNTER_BDR_REG1_XL_HG_BATCH_EN: u8 = 1 << 3;
/// CNT_BDR_TH\[9:8\].
pub const COUNTER_BDR_REG1_CNT_BDR_TH_MASK: u8 = 0b11;
/// COUNTER_BDR_REG2 (9.10, Tables 45-46, p62): CNT_BDR_TH\[7:0\].
pub const COUNTER_BDR_REG2_CNT_BDR_TH_MASK: u8 = 0xFF;

/// INT1_CTRL bits (9.11, Tables 47-48, p63).
pub const INT1_CTRL_DRDY_XL: u8 = 1 << 0;
pub const INT1_CTRL_DRDY_G: u8 = 1 << 1;
pub const INT1_CTRL_FIFO_TH: u8 = 1 << 3;
pub const INT1_CTRL_FIFO_OVR: u8 = 1 << 4;
pub const INT1_CTRL_FIFO_FULL: u8 = 1 << 5;
pub const INT1_CTRL_CNT_BDR: u8 = 1 << 6;

/// INT2_CTRL bits (9.12, Tables 49-50, p64).
pub const INT2_CTRL_DRDY_XL: u8 = 1 << 0;
pub const INT2_CTRL_DRDY_G: u8 = 1 << 1;
pub const INT2_CTRL_DRDY_G_EIS: u8 = 1 << 2;
pub const INT2_CTRL_FIFO_TH: u8 = 1 << 3;
pub const INT2_CTRL_FIFO_OVR: u8 = 1 << 4;
pub const INT2_CTRL_FIFO_FULL: u8 = 1 << 5;
pub const INT2_CTRL_CNT_BDR: u8 = 1 << 6;
pub const INT2_CTRL_EMB_FUNC_ENDOP: u8 = 1 << 7;

/// The low-g and gyroscope data-ready bits share positions on INT1_CTRL and INT2_CTRL.
pub const INT_DRDY_XL: u8 = 1 << 0;
pub const INT_DRDY_G: u8 = 1 << 1;
/// The FIFO bits share positions on INT1_CTRL and INT2_CTRL.
pub const INT_FIFO_TH: u8 = 1 << 3;
pub const INT_FIFO_OVR: u8 = 1 << 4;
pub const INT_FIFO_FULL: u8 = 1 << 5;

/// CTRL1 fields (9.14, Tables 52-54, p65): OP_MODE_XL\[2:0\] in bits 6:4, ODR_XL\[3:0\] in bits 3:0.
pub const CTRL1_OP_MODE_XL_SHIFT: u8 = 4;
pub const CTRL1_OP_MODE_XL_MASK: u8 = 0b111 << 4;
pub const CTRL1_ODR_XL_MASK: u8 = 0x0F;
/// OP_MODE_XL codes (Table 53, p65). 0b010 is reserved.
pub const OP_MODE_XL_HIGH_PERFORMANCE: u8 = 0b000;
pub const OP_MODE_XL_HIGH_ACCURACY_ODR: u8 = 0b001;
pub const OP_MODE_XL_ODR_TRIGGERED: u8 = 0b011;
pub const OP_MODE_XL_LOW_POWER_1: u8 = 0b100;
pub const OP_MODE_XL_LOW_POWER_2: u8 = 0b101;
pub const OP_MODE_XL_LOW_POWER_3: u8 = 0b110;
pub const OP_MODE_XL_NORMAL: u8 = 0b111;

/// CTRL2 fields (9.15, Tables 55-56, p66): OP_MODE_G\[2:0\] in bits 6:4, ODR_G\[3:0\] in bits 3:0.
pub const CTRL2_OP_MODE_G_SHIFT: u8 = 4;
pub const CTRL2_OP_MODE_G_MASK: u8 = 0b111 << 4;
pub const CTRL2_ODR_G_MASK: u8 = 0x0F;
/// OP_MODE_G codes (Table 56, p66). 0b010 and 0b110-0b111 are reserved.
pub const OP_MODE_G_HIGH_PERFORMANCE: u8 = 0b000;
pub const OP_MODE_G_HIGH_ACCURACY_ODR: u8 = 0b001;
pub const OP_MODE_G_ODR_TRIGGERED: u8 = 0b011;
pub const OP_MODE_G_SLEEP: u8 = 0b100;
pub const OP_MODE_G_LOW_POWER: u8 = 0b101;

/// ODR_XL and ODR_G codes (Tables 54 and 57, p65-66). The 1.875 Hz code (0b0001) is accelerometer
/// low-power only; for the gyroscope it is reserved. Codes above 0b1100 are reserved.
pub const ODR_POWER_DOWN: u8 = 0b0000;
pub const ODR_1_875_HZ: u8 = 0b0001;
pub const ODR_7_5_HZ: u8 = 0b0010;
pub const ODR_15_HZ: u8 = 0b0011;
pub const ODR_30_HZ: u8 = 0b0100;
pub const ODR_60_HZ: u8 = 0b0101;
pub const ODR_120_HZ: u8 = 0b0110;
pub const ODR_240_HZ: u8 = 0b0111;
pub const ODR_480_HZ: u8 = 0b1000;
pub const ODR_960_HZ: u8 = 0b1001;
pub const ODR_1920_HZ: u8 = 0b1010;
pub const ODR_3840_HZ: u8 = 0b1011;
pub const ODR_7680_HZ: u8 = 0b1100;

/// CTRL3 bits (9.16, Tables 58-59, p67). Reset value 0x44.
pub const CTRL3_SW_RESET: u8 = 1 << 0;
pub const CTRL3_IF_INC: u8 = 1 << 2;
pub const CTRL3_BDU: u8 = 1 << 6;
/// Reboots the memory content (reloads trimming); self-clearing.
pub const CTRL3_BOOT: u8 = 1 << 7;

/// CTRL4 bits (9.17, Tables 60-61, p68).
pub const CTRL4_INT2_IN_LH: u8 = 1 << 0;
/// 1 = data-ready pulsed (65 us pulses); 0 = latched until the high output byte is read.
pub const CTRL4_DRDY_PULSED: u8 = 1 << 1;
pub const CTRL4_INT2_DRDY_TEMP: u8 = 1 << 2;
/// Masks the data-ready signals of all three channels until their filter settling ends.
pub const CTRL4_DRDY_MASK: u8 = 1 << 3;
pub const CTRL4_INT2_ON_INT1: u8 = 1 << 4;

/// CTRL5 fields (9.18, Tables 62-63, p69). I3C only; left at the reset value 0.
pub const CTRL5_INT_EN_I3C: u8 = 1 << 0;
pub const CTRL5_BUS_ACT_SEL_SHIFT: u8 = 1;
pub const CTRL5_BUS_ACT_SEL_MASK: u8 = 0b11 << 1;
pub const CTRL5_IF2_TA0_PID: u8 = 1 << 7;

/// CTRL6 fields (9.19, Tables 64-65, p69). Reset value 0x08.
pub const CTRL6_FS_G_SHIFT: u8 = 0;
pub const CTRL6_FS_G_MASK: u8 = 0b111;
/// Bit 3 must be written as 1 (Table 64, footnote 2, p69).
pub const CTRL6_MUST_BE_ONE: u8 = 1 << 3;
/// LPF1_G_BW\[2:0\], gyroscope LPF1 bandwidth selection (Table 66, p70).
pub const CTRL6_LPF1_G_BW_SHIFT: u8 = 4;
pub const CTRL6_LPF1_G_BW_MASK: u8 = 0b111 << 4;
/// FS_G codes (Table 65, p69). The reset value 0b000 is reserved; 0b110-0b111 are reserved.
pub const FS_G_250_DPS: u8 = 0b001;
pub const FS_G_500_DPS: u8 = 0b010;
pub const FS_G_1000_DPS: u8 = 0b011;
pub const FS_G_2000_DPS: u8 = 0b100;
pub const FS_G_4000_DPS: u8 = 0b101;

/// CTRL7 bits (9.20, Tables 67-68, p70).
/// Enables the gyroscope digital LPF1; its bandwidth is LPF1_G_BW in CTRL6.
pub const CTRL7_LPF1_G_EN: u8 = 1 << 0;
pub const CTRL7_INT2_DRDY_XL_HG: u8 = 1 << 6;
pub const CTRL7_INT1_DRDY_XL_HG: u8 = 1 << 7;

/// CTRL8 fields (9.21, Tables 69-71, p71).
pub const CTRL8_FS_XL_SHIFT: u8 = 0;
pub const CTRL8_FS_XL_MASK: u8 = 0b11;
/// HP_LPF2_XL_BW\[2:0\], accelerometer LPF2 (or high-pass) cutoff selection.
pub const CTRL8_HP_LPF2_XL_BW_SHIFT: u8 = 5;
pub const CTRL8_HP_LPF2_XL_BW_MASK: u8 = 0b111 << 5;
/// FS_XL codes (Table 70, p71).
pub const FS_XL_2_G: u8 = 0b00;
pub const FS_XL_4_G: u8 = 0b01;
pub const FS_XL_8_G: u8 = 0b10;
pub const FS_XL_16_G: u8 = 0b11;
/// LPF2 cutoff codes with LPF2_XL_EN = 1 and HP_SLOPE_XL_EN = 0 (Table 71, p71).
pub const LPF2_XL_BW_ODR_DIV_4: u8 = 0b000;
pub const LPF2_XL_BW_ODR_DIV_10: u8 = 0b001;
pub const LPF2_XL_BW_ODR_DIV_20: u8 = 0b010;
pub const LPF2_XL_BW_ODR_DIV_45: u8 = 0b011;
pub const LPF2_XL_BW_ODR_DIV_100: u8 = 0b100;
pub const LPF2_XL_BW_ODR_DIV_200: u8 = 0b101;
pub const LPF2_XL_BW_ODR_DIV_400: u8 = 0b110;
pub const LPF2_XL_BW_ODR_DIV_800: u8 = 0b111;

/// CTRL9 bits (9.22, Tables 72-73, p72).
/// Applies the low-g user offset to the output registers (low-pass path).
pub const CTRL9_USR_OFF_ON_OUT: u8 = 1 << 0;
/// Weight of the low-g user offset LSB: 0 = 2^-10 g, 1 = 2^-6 g (Tables 225-229, p113).
pub const CTRL9_USR_OFF_W: u8 = 1 << 1;
/// 1 = output of the LPF2 second filtering stage.
pub const CTRL9_LPF2_XL_EN: u8 = 1 << 3;
pub const CTRL9_HP_SLOPE_XL_EN: u8 = 1 << 4;
/// LPF2 and HPF fast-settling mode, active only while exiting power-down.
pub const CTRL9_XL_FASTSETTL_MODE: u8 = 1 << 5;
pub const CTRL9_HP_REF_MODE_XL: u8 = 1 << 6;

/// CTRL10 fields (9.23, Tables 74-75, p74).
pub const CTRL10_ST_XL_SHIFT: u8 = 0;
pub const CTRL10_ST_XL_MASK: u8 = 0b11;
pub const CTRL10_ST_G_SHIFT: u8 = 2;
pub const CTRL10_ST_G_MASK: u8 = 0b11 << 2;
pub const CTRL10_EMB_FUNC_DEBUG: u8 = 1 << 6;
/// Self-test codes of ST_XL, ST_G and XL_HG_ST: normal mode, positive sign, negative sign. 0b11 is
/// reserved.
pub const SELF_TEST_OFF: u8 = 0b00;
pub const SELF_TEST_POSITIVE: u8 = 0b01;
pub const SELF_TEST_NEGATIVE: u8 = 0b10;

/// CTRL_STATUS (9.24, Tables 76-77, p74).
pub const CTRL_STATUS_FSM_WR_CTRL_STATUS: u8 = 1 << 2;

/// FIFO_STATUS1 (9.25, Tables 78-79, p75): DIFF_FIFO\[7:0\], unread words (tag + 6 bytes).
pub const FIFO_STATUS1_DIFF_FIFO_MASK: u8 = 0xFF;

/// FIFO_STATUS2 bits (9.26, Tables 80-81, p75).
/// DIFF_FIFO\[8\], the high bit of the unread-word count.
pub const FIFO_STATUS2_DIFF_FIFO_8: u8 = 1 << 0;
/// Latched overrun flag; cleared by reading this register.
pub const FIFO_STATUS2_FIFO_OVR_LATCHED: u8 = 1 << 3;
/// COUNTER_BDR threshold reached; cleared by reading COUNTER_BDR_REG1/2.
pub const FIFO_STATUS2_COUNTER_BDR_IA: u8 = 1 << 4;
pub const FIFO_STATUS2_FIFO_FULL_IA: u8 = 1 << 5;
pub const FIFO_STATUS2_FIFO_OVR_IA: u8 = 1 << 6;
pub const FIFO_STATUS2_FIFO_WTM_IA: u8 = 1 << 7;

/// ALL_INT_SRC bits (9.27, Tables 82-83, p76).
pub const ALL_INT_SRC_FF_IA: u8 = 1 << 0;
pub const ALL_INT_SRC_WU_IA: u8 = 1 << 1;
pub const ALL_INT_SRC_TAP_IA: u8 = 1 << 2;
/// Logical OR of the high-g wake-up and the high-g shock.
pub const ALL_INT_SRC_HG_IA: u8 = 1 << 3;
pub const ALL_INT_SRC_D6D_IA: u8 = 1 << 4;
pub const ALL_INT_SRC_SLEEP_CHANGE_IA: u8 = 1 << 5;
pub const ALL_INT_SRC_SHUB_IA: u8 = 1 << 6;
pub const ALL_INT_SRC_EMB_FUNC_IA: u8 = 1 << 7;

/// STATUS_REG bits (9.28, Tables 84-85, p77).
pub const STATUS_XLDA: u8 = 1 << 0;
pub const STATUS_GDA: u8 = 1 << 1;
pub const STATUS_TDA: u8 = 1 << 2;
pub const STATUS_XLHGDA: u8 = 1 << 3;
pub const STATUS_GDA_EIS: u8 = 1 << 4;
pub const STATUS_OIS_DRDY: u8 = 1 << 5;
/// Timestamp overflow alert, 5.6 ms ahead of the overflow.
pub const STATUS_TIMESTAMP_ENDCOUNT: u8 = 1 << 7;

/// TIMESTAMP0..3 (9.42, p85): 32-bit counter, little-endian, one LSB is 21.7 us typical.
pub const TIMESTAMP_LEN: usize = 4;

/// HG_WAKE_UP_SRC bits (9.51, Tables 143-144, p90).
pub const HG_WAKE_UP_SRC_HG_Z_WU: u8 = 1 << 0;
pub const HG_WAKE_UP_SRC_HG_Y_WU: u8 = 1 << 1;
pub const HG_WAKE_UP_SRC_HG_X_WU: u8 = 1 << 2;
pub const HG_WAKE_UP_SRC_HG_WU_IA: u8 = 1 << 3;
pub const HG_WAKE_UP_SRC_HG_WU_CHANGE_IA: u8 = 1 << 4;
pub const HG_WAKE_UP_SRC_HG_SHOCK_STATE: u8 = 1 << 5;
pub const HG_WAKE_UP_SRC_HG_SHOCK_CHANGE_IA: u8 = 1 << 6;

/// CTRL2_XL_HG bits (9.52, Tables 145-146, p91).
pub const CTRL2_XL_HG_ST_SHIFT: u8 = 0;
pub const CTRL2_XL_HG_ST_MASK: u8 = 0b11;
/// Drives the high-g data with user offset correction to the high-g wake-up and shock.
pub const CTRL2_XL_HG_HG_USR_OFF_ON_WU: u8 = 1 << 4;

/// CTRL1_XL_HG fields (9.53, Tables 147-150, p91).
pub const CTRL1_XL_HG_FS_SHIFT: u8 = 0;
pub const CTRL1_XL_HG_FS_MASK: u8 = 0b111;
pub const CTRL1_XL_HG_ODR_SHIFT: u8 = 3;
pub const CTRL1_XL_HG_ODR_MASK: u8 = 0b111 << 3;
/// Applies the high-g user offset to the output registers.
pub const CTRL1_XL_HG_HG_USR_OFF_ON_OUT: u8 = 1 << 6;
/// Enables reading the high-g channel from UI_OUTX_L_A_OIS_HG (0x34) to 0x39.
pub const CTRL1_XL_HG_REGOUT_EN: u8 = 1 << 7;
/// ODR_XL_HG codes (Table 149, p91). 0b001 and 0b010 are N/A.
pub const ODR_XL_HG_POWER_DOWN: u8 = 0b000;
pub const ODR_XL_HG_480_HZ: u8 = 0b011;
pub const ODR_XL_HG_960_HZ: u8 = 0b100;
pub const ODR_XL_HG_1920_HZ: u8 = 0b101;
pub const ODR_XL_HG_3840_HZ: u8 = 0b110;
pub const ODR_XL_HG_7680_HZ: u8 = 0b111;
/// FS_XL_HG codes (Table 150, p91).
pub const FS_XL_HG_32_G: u8 = 0b000;
pub const FS_XL_HG_64_G: u8 = 0b001;
pub const FS_XL_HG_128_G: u8 = 0b010;
pub const FS_XL_HG_256_G: u8 = 0b011;
pub const FS_XL_HG_320_G: u8 = 0b100;

/// INTERNAL_FREQ_FINE (9.54, Tables 151-153, p92): FREQ_FINE\[7:0\], two's complement. One LSB changes
/// the output rate and the timestamp rate by 0.13 % (1300 parts per million).
pub const INTERNAL_FREQ_FINE_STEP_PPM: u32 = 1300;

/// FUNCTIONS_ENABLE bits (9.55, Tables 154-155, p93).
pub const FUNCTIONS_ENABLE_INACT_EN_SHIFT: u8 = 0;
pub const FUNCTIONS_ENABLE_INACT_EN_MASK: u8 = 0b11;
pub const FUNCTIONS_ENABLE_DIS_RST_LIR_ALL_INT: u8 = 1 << 3;
/// Enables the timestamp counter (TIMESTAMP0..3).
pub const FUNCTIONS_ENABLE_TIMESTAMP_EN: u8 = 1 << 6;
pub const FUNCTIONS_ENABLE_INTERRUPTS_ENABLE: u8 = 1 << 7;

/// HG_FUNCTIONS_ENABLE fields (9.56, Tables 156-157, p94).
pub const HG_FUNCTIONS_ENABLE_HG_SHOCK_DUR_SHIFT: u8 = 0;
pub const HG_FUNCTIONS_ENABLE_HG_SHOCK_DUR_MASK: u8 = 0x0F;
/// Allowed HG_SHOCK_DUR values are 0 to 14. Duration = (HG_SHOCK_DUR + 1) * 512 / ODR_XL_HG.
pub const HG_SHOCK_DUR_MAX: u8 = 14;
pub const HG_FUNCTIONS_ENABLE_INT1_HG_WU: u8 = 1 << 4;
pub const HG_FUNCTIONS_ENABLE_INT2_HG_WU: u8 = 1 << 5;
/// 0 = interrupt each time an axis is over the threshold; 1 = only on the change between all axes
/// under and at least one axis over the threshold.
pub const HG_FUNCTIONS_ENABLE_HG_WU_CHANGE_INT_SEL: u8 = 1 << 6;
/// Enables the high-g interrupt generator (wake-up and shock).
pub const HG_FUNCTIONS_ENABLE_HG_INTERRUPTS_ENABLE: u8 = 1 << 7;

/// HG_WAKE_UP_THS (9.57, Tables 158-159, p94): HG_WK_THS\[7:0\]. 1 g per LSB up to +-256 g full
/// scale, 1.25 g per LSB at +-320 g.
pub const HG_WAKE_UP_THS_MASK: u8 = 0xFF;

/// INACTIVITY_THS bits (9.59, Tables 162-163, p96). Bits 5:0 are INACT_THS (activity/inactivity).
pub const INACTIVITY_THS_INACT_THS_MASK: u8 = 0b0011_1111;
/// Routes the high-g shock event to INT1.
pub const INACTIVITY_THS_INT1_HG_SHOCK_CHANGE: u8 = 1 << 6;
/// Routes the high-g shock event to INT2.
pub const INACTIVITY_THS_INT2_HG_SHOCK_CHANGE: u8 = 1 << 7;

/// MD1_CFG bits (9.68, Tables 183-184, p102). The high-g events are routed in
/// HG_FUNCTIONS_ENABLE and INACTIVITY_THS, not here.
pub const MD1_CFG_INT1_SHUB: u8 = 1 << 0;
pub const MD1_CFG_INT1_EMB_FUNC: u8 = 1 << 1;
pub const MD1_CFG_INT1_6D: u8 = 1 << 2;
pub const MD1_CFG_INT1_DOUBLE_TAP: u8 = 1 << 3;
pub const MD1_CFG_INT1_FF: u8 = 1 << 4;
pub const MD1_CFG_INT1_WU: u8 = 1 << 5;
pub const MD1_CFG_INT1_SINGLE_TAP: u8 = 1 << 6;
pub const MD1_CFG_INT1_SLEEP_CHANGE: u8 = 1 << 7;

/// MD2_CFG bits (9.69, Tables 185-186, p103).
/// Routes the timestamp overflow alert (5.6 ms ahead) to INT2. INT1 has no such bit.
pub const MD2_CFG_INT2_TIMESTAMP: u8 = 1 << 0;
pub const MD2_CFG_INT2_EMB_FUNC: u8 = 1 << 1;
pub const MD2_CFG_INT2_6D: u8 = 1 << 2;
pub const MD2_CFG_INT2_DOUBLE_TAP: u8 = 1 << 3;
pub const MD2_CFG_INT2_FF: u8 = 1 << 4;
pub const MD2_CFG_INT2_WU: u8 = 1 << 5;
pub const MD2_CFG_INT2_SINGLE_TAP: u8 = 1 << 6;
pub const MD2_CFG_INT2_SLEEP_CHANGE: u8 = 1 << 7;

/// HAODR_CFG (9.70, Tables 187-188, p103): HAODR_SEL\[1:0\] selects the ODR set of the
/// high-accuracy ODR mode (Table 22, p34).
pub const HAODR_CFG_HAODR_SEL_SHIFT: u8 = 0;
pub const HAODR_CFG_HAODR_SEL_MASK: u8 = 0b11;

/// XL_HG_{X,Y,Z}_OFS_USR (9.80-9.82, Tables 208-213, p108): two's complement, valid range
/// -127..=127, 0.25 g per LSB up to +-256 g full scale, 0.33 g per LSB at +-320 g.
/// X_OFS_USR, Y_OFS_USR, Z_OFS_USR (9.87-9.89, Tables 224-229, p113): two's complement, valid
/// range -127..=127, weight set by CTRL9 USR_OFF_W.
pub const OFS_USR_MIN: i8 = -127;
pub const OFS_USR_MAX: i8 = 127;

/// FIFO_DATA_OUT_TAG fields (9.90, Tables 230-231, p114): TAG_SENSOR\[4:0\] in bits 7:3, TAG_CNT\[1:0\]
/// in bits 2:1, bit 0 not used.
pub const FIFO_TAG_SENSOR_SHIFT: u8 = 3;
pub const FIFO_TAG_SENSOR_MASK: u8 = 0b1_1111 << 3;
pub const FIFO_TAG_CNT_SHIFT: u8 = 1;
pub const FIFO_TAG_CNT_MASK: u8 = 0b11 << 1;
/// TAG_SENSOR codes (Table 232, p114). 0x14 and 0x15 are not listed.
pub const TAG_FIFO_EMPTY: u8 = 0x00;
pub const TAG_GYROSCOPE_NC: u8 = 0x01;
pub const TAG_ACCELEROMETER_NC: u8 = 0x02;
pub const TAG_TEMPERATURE: u8 = 0x03;
pub const TAG_TIMESTAMP: u8 = 0x04;
pub const TAG_CFG_CHANGE: u8 = 0x05;
pub const TAG_ACCELEROMETER_NC_T_2: u8 = 0x06;
pub const TAG_ACCELEROMETER_NC_T_1: u8 = 0x07;
pub const TAG_ACCELEROMETER_2XC: u8 = 0x08;
pub const TAG_ACCELEROMETER_3XC: u8 = 0x09;
pub const TAG_GYROSCOPE_NC_T_2: u8 = 0x0A;
pub const TAG_GYROSCOPE_NC_T_1: u8 = 0x0B;
pub const TAG_GYROSCOPE_2XC: u8 = 0x0C;
pub const TAG_GYROSCOPE_3XC: u8 = 0x0D;
pub const TAG_SENSOR_HUB_TARGET_0: u8 = 0x0E;
pub const TAG_SENSOR_HUB_TARGET_1: u8 = 0x0F;
pub const TAG_SENSOR_HUB_TARGET_2: u8 = 0x10;
pub const TAG_SENSOR_HUB_TARGET_3: u8 = 0x11;
pub const TAG_STEP_COUNTER: u8 = 0x12;
pub const TAG_SFLP_GAME_ROTATION_VECTOR: u8 = 0x13;
pub const TAG_SFLP_GYROSCOPE_BIAS: u8 = 0x16;
pub const TAG_SFLP_GRAVITY_VECTOR: u8 = 0x17;
pub const TAG_HIGH_G_PEAK_VALUE: u8 = 0x18;
pub const TAG_SENSOR_HUB_NACK: u8 = 0x19;
pub const TAG_MLC_RESULT: u8 = 0x1A;
pub const TAG_MLC_FILTER: u8 = 0x1B;
pub const TAG_MLC_FEATURE: u8 = 0x1C;
pub const TAG_HIGH_G_ACCELEROMETER: u8 = 0x1D;
pub const TAG_ENHANCED_EIS_GYROSCOPE: u8 = 0x1E;
pub const TAG_FSM_RESULTS: u8 = 0x1F;

/// Bytes in one FIFO word: tag, then X, Y, Z (6.5.8, p46).
pub const FIFO_WORD_LEN: usize = 7;

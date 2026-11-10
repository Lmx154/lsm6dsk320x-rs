//! Register addresses (datasheet DS15060 Rev 1, Table 26, p52-54) and the bit fields the driver uses.
//!
//! Register names follow the datasheet. Addresses 0x76 and above are not listed.
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

/// Fixed WHO_AM_I value (p64).
pub const WHO_AM_I_VALUE: u8 = 0x75;

/// First bit of every SPI transaction: set for a read (p22).
pub const SPI_READ: u8 = 0x80;

/// IF_CFG bits (p58). Not reset by a software reset.
pub const IF_CFG_I2C_I3C_DISABLE: u8 = 1 << 0;
pub const IF_CFG_SIM: u8 = 1 << 2;
pub const IF_CFG_PP_OD: u8 = 1 << 3;
pub const IF_CFG_H_LACTIVE: u8 = 1 << 4;

/// INT1_CTRL / INT2_CTRL bits (p63-64).
pub const INT_DRDY_XL: u8 = 1 << 0;
pub const INT_DRDY_G: u8 = 1 << 1;

/// CTRL3 bits (p67). Reset value 0x44.
pub const CTRL3_SW_RESET: u8 = 1 << 0;
pub const CTRL3_IF_INC: u8 = 1 << 2;
pub const CTRL3_BDU: u8 = 1 << 6;
pub const CTRL3_BOOT: u8 = 1 << 7;

/// CTRL4 bits (p68).
pub const CTRL4_DRDY_PULSED: u8 = 1 << 1;

/// CTRL6 bit 3 must be written as 1 (p69).
pub const CTRL6_MUST_BE_ONE: u8 = 1 << 3;

/// CTRL7 bits (p70).
pub const CTRL7_INT2_DRDY_XL_HG: u8 = 1 << 6;
pub const CTRL7_INT1_DRDY_XL_HG: u8 = 1 << 7;

/// CTRL1_XL_HG bit (p91).
pub const CTRL1_XL_HG_REGOUT_EN: u8 = 1 << 7;

/// STATUS_REG bits (p77).
pub const STATUS_XLDA: u8 = 1 << 0;
pub const STATUS_GDA: u8 = 1 << 1;
pub const STATUS_TDA: u8 = 1 << 2;
pub const STATUS_XLHGDA: u8 = 1 << 3;

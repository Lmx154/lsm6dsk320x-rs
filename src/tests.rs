//! Host tests of the pure layer: register images against the datasheet tables, `Config`
//! construction, validation, the write plan, the helpers, and properties over all configurations.
//!
//! Expected values are written out as numbers worked from the datasheet tables (DS15060 Rev 1,
//! printed page numbers in the comments), not computed from the `reg` constants under test.

use crate::calc::*;
use crate::config::*;
use crate::plan::*;
use crate::reg;
use proptest::prelude::*;
use std::vec::Vec;

// ---------------------------------------------------------------------------------------------
// Register map (Table 26, p52-55)
// ---------------------------------------------------------------------------------------------

#[test]
fn register_addresses() {
    assert_eq!(reg::PIN_CTRL, 0x02);
    assert_eq!(reg::IF_CFG, 0x03);
    assert_eq!(reg::FIFO_CTRL1, 0x07);
    assert_eq!(reg::FIFO_CTRL4, 0x0A);
    assert_eq!(reg::COUNTER_BDR_REG1, 0x0B);
    assert_eq!(reg::COUNTER_BDR_REG2, 0x0C);
    assert_eq!(reg::INT1_CTRL, 0x0D);
    assert_eq!(reg::INT2_CTRL, 0x0E);
    assert_eq!(reg::CTRL1, 0x10);
    assert_eq!(reg::CTRL10, 0x19);
    assert_eq!(reg::FIFO_STATUS1, 0x1B);
    assert_eq!(reg::FIFO_STATUS2, 0x1C);
    assert_eq!(reg::STATUS_REG, 0x1E);
    assert_eq!(reg::TIMESTAMP0, 0x40);
    assert_eq!(reg::TIMESTAMP3, 0x43);
    assert_eq!(reg::HG_WAKE_UP_SRC, 0x4C);
    assert_eq!(reg::CTRL2_XL_HG, 0x4D);
    assert_eq!(reg::CTRL1_XL_HG, 0x4E);
    assert_eq!(reg::INTERNAL_FREQ_FINE, 0x4F);
    assert_eq!(reg::FUNCTIONS_ENABLE, 0x50);
    assert_eq!(reg::HG_FUNCTIONS_ENABLE, 0x52);
    assert_eq!(reg::HG_WAKE_UP_THS, 0x53);
    assert_eq!(reg::INACTIVITY_THS, 0x55);
    assert_eq!(reg::MD1_CFG, 0x5E);
    assert_eq!(reg::MD2_CFG, 0x5F);
    assert_eq!(reg::HAODR_CFG, 0x62);
    assert_eq!(reg::XL_HG_X_OFS_USR, 0x6C);
    assert_eq!(reg::XL_HG_Z_OFS_USR, 0x6E);
    assert_eq!(reg::X_OFS_USR, 0x73);
    assert_eq!(reg::Z_OFS_USR, 0x75);
    assert_eq!(reg::FIFO_DATA_OUT_TAG, 0x78);
    assert_eq!(reg::FIFO_DATA_OUT_X_L, 0x79);
    assert_eq!(reg::FIFO_DATA_OUT_Z_H, 0x7E);
    assert_eq!(reg::FIFO_WORD_LEN, 7);
}

#[test]
fn register_reset_values() {
    // Table 26, p52: PIN_CTRL 00100011, CTRL3 01000100, CTRL6 00001000.
    assert_eq!(reg::PIN_CTRL_RESET, 0x23);
    assert_eq!(reg::CTRL3_RESET, 0x44);
    assert_eq!(reg::CTRL6_RESET, 0x08);
}

#[test]
fn bit_fields_of_status_and_fifo_registers() {
    // STATUS_REG, Table 84, p77: TIMESTAMP_ENDCOUNT b7, OIS_DRDY b5, GDA_EIS b4, XLHGDA b3, TDA b2,
    // GDA b1, XLDA b0.
    assert_eq!(reg::STATUS_TIMESTAMP_ENDCOUNT, 0x80);
    assert_eq!(reg::STATUS_OIS_DRDY, 0x20);
    assert_eq!(reg::STATUS_GDA_EIS, 0x10);
    assert_eq!(reg::STATUS_XLHGDA, 0x08);
    assert_eq!(reg::STATUS_TDA, 0x04);
    assert_eq!(reg::STATUS_GDA, 0x02);
    assert_eq!(reg::STATUS_XLDA, 0x01);
    // FIFO_STATUS2, Table 80, p75.
    assert_eq!(reg::FIFO_STATUS2_FIFO_WTM_IA, 0x80);
    assert_eq!(reg::FIFO_STATUS2_FIFO_OVR_IA, 0x40);
    assert_eq!(reg::FIFO_STATUS2_FIFO_FULL_IA, 0x20);
    assert_eq!(reg::FIFO_STATUS2_COUNTER_BDR_IA, 0x10);
    assert_eq!(reg::FIFO_STATUS2_FIFO_OVR_LATCHED, 0x08);
    assert_eq!(reg::FIFO_STATUS2_DIFF_FIFO_8, 0x01);
    // ALL_INT_SRC, Table 82, p76.
    assert_eq!(reg::ALL_INT_SRC_HG_IA, 0x08);
    assert_eq!(reg::ALL_INT_SRC_EMB_FUNC_IA, 0x80);
    // HG_WAKE_UP_SRC, Table 143, p90.
    assert_eq!(reg::HG_WAKE_UP_SRC_HG_SHOCK_CHANGE_IA, 0x40);
    assert_eq!(reg::HG_WAKE_UP_SRC_HG_SHOCK_STATE, 0x20);
    assert_eq!(reg::HG_WAKE_UP_SRC_HG_WU_CHANGE_IA, 0x10);
    assert_eq!(reg::HG_WAKE_UP_SRC_HG_WU_IA, 0x08);
    assert_eq!(reg::HG_WAKE_UP_SRC_HG_Z_WU, 0x01);
    // CTRL3, Table 58, p67.
    assert_eq!(reg::CTRL3_BOOT, 0x80);
    assert_eq!(reg::CTRL3_BDU, 0x40);
    assert_eq!(reg::CTRL3_IF_INC, 0x04);
    assert_eq!(reg::CTRL3_SW_RESET, 0x01);
    // MD1_CFG and MD2_CFG, Tables 183 and 185, p102-103.
    assert_eq!(reg::MD1_CFG_INT1_SLEEP_CHANGE, 0x80);
    assert_eq!(reg::MD1_CFG_INT1_SHUB, 0x01);
    assert_eq!(reg::MD2_CFG_INT2_TIMESTAMP, 0x01);
    assert_eq!(reg::MD2_CFG_INT2_SLEEP_CHANGE, 0x80);
    // Self-test, Tables 74 and 145, p74 and p91.
    assert_eq!(reg::CTRL10_ST_G_SHIFT, 2);
    assert_eq!(reg::CTRL10_ST_XL_SHIFT, 0);
    assert_eq!(reg::CTRL2_XL_HG_ST_SHIFT, 0);
    assert_eq!(reg::CTRL2_XL_HG_HG_USR_OFF_ON_WU, 0x10);
}

#[test]
fn fifo_tag_codes() {
    // Table 232, p114, and the field layout of Table 230.
    assert_eq!(reg::TAG_FIFO_EMPTY, 0x00);
    assert_eq!(reg::TAG_GYROSCOPE_NC, 0x01);
    assert_eq!(reg::TAG_ACCELEROMETER_NC, 0x02);
    assert_eq!(reg::TAG_TEMPERATURE, 0x03);
    assert_eq!(reg::TAG_TIMESTAMP, 0x04);
    assert_eq!(reg::TAG_CFG_CHANGE, 0x05);
    assert_eq!(reg::TAG_ACCELEROMETER_NC_T_2, 0x06);
    assert_eq!(reg::TAG_GYROSCOPE_3XC, 0x0D);
    assert_eq!(reg::TAG_SENSOR_HUB_TARGET_0, 0x0E);
    assert_eq!(reg::TAG_STEP_COUNTER, 0x12);
    assert_eq!(reg::TAG_SFLP_GAME_ROTATION_VECTOR, 0x13);
    assert_eq!(reg::TAG_SFLP_GYROSCOPE_BIAS, 0x16);
    assert_eq!(reg::TAG_HIGH_G_PEAK_VALUE, 0x18);
    assert_eq!(reg::TAG_HIGH_G_ACCELEROMETER, 0x1D);
    assert_eq!(reg::TAG_ENHANCED_EIS_GYROSCOPE, 0x1E);
    assert_eq!(reg::TAG_FSM_RESULTS, 0x1F);
    assert_eq!(reg::FIFO_TAG_SENSOR_SHIFT, 3);
    assert_eq!(reg::FIFO_TAG_SENSOR_MASK, 0xF8);
    assert_eq!(reg::FIFO_TAG_CNT_MASK, 0x06);
}

// ---------------------------------------------------------------------------------------------
// Code tables
// ---------------------------------------------------------------------------------------------

#[test]
fn code_tables() {
    use BatchRate::*;
    // Table 40, p60.
    assert_eq!(
        [
            Off, Hz1_875, Hz7_5, Hz15, Hz30, Hz60, Hz120, Hz240, Hz480, Hz960, Hz1920, Hz3840,
            Hz7680
        ]
        .map(BatchRate::code),
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    );
    // FIFO_MODE, p61.
    assert_eq!(
        [
            FifoMode::Bypass,
            FifoMode::Fifo,
            FifoMode::ContinuousWtmToFull,
            FifoMode::ContinuousToFifo,
            FifoMode::BypassToContinuous,
            FifoMode::Continuous,
            FifoMode::BypassToFifo,
        ]
        .map(FifoMode::code),
        [0b000, 0b001, 0b010, 0b011, 0b100, 0b110, 0b111]
    );
    // DEC_TS_BATCH and ODR_T_BATCH, p61.
    assert_eq!(
        [
            TimestampBatch::Off,
            TimestampBatch::Dec1,
            TimestampBatch::Dec8,
            TimestampBatch::Dec32
        ]
        .map(TimestampBatch::code),
        [0, 1, 2, 3]
    );
    assert_eq!(
        [
            TempBatchRate::Off,
            TempBatchRate::Hz1_875,
            TempBatchRate::Hz15,
            TempBatchRate::Hz60
        ]
        .map(TempBatchRate::code),
        [0, 1, 2, 3]
    );
    // HP_LPF2_XL_BW, Table 71, p71: ODR/4, /10, /20, /45, /100, /200, /400, /800.
    let lpf2 = [
        AccelLpf2Bandwidth::OdrDiv4,
        AccelLpf2Bandwidth::OdrDiv10,
        AccelLpf2Bandwidth::OdrDiv20,
        AccelLpf2Bandwidth::OdrDiv45,
        AccelLpf2Bandwidth::OdrDiv100,
        AccelLpf2Bandwidth::OdrDiv200,
        AccelLpf2Bandwidth::OdrDiv400,
        AccelLpf2Bandwidth::OdrDiv800,
    ];
    assert_eq!(lpf2.map(AccelLpf2Bandwidth::code), [0, 1, 2, 3, 4, 5, 6, 7]);
    assert_eq!(
        lpf2.map(AccelLpf2Bandwidth::divisor),
        [4, 10, 20, 45, 100, 200, 400, 800]
    );
    // IO_PAD_STRENGTH, p57 (10 is reserved).
    assert_eq!(
        [PadDrive::Lowest, PadDrive::Intermediate, PadDrive::Highest].map(PadDrive::code),
        [0b00, 0b01, 0b11]
    );
    // HAODR_SEL, Table 22, p34.
    assert_eq!(
        [
            HaodrSet::Base960,
            HaodrSet::Base1000,
            HaodrSet::Base800,
            HaodrSet::Base833
        ]
        .map(HaodrSet::code),
        [0, 1, 2, 3]
    );
}

#[test]
fn haodr_odr_table() {
    // Table 22, p34, row ODR code 1001 and 0011.
    assert_eq!(Odr::Hz960.haodr_mhz(HaodrSet::Base960), Some(960_000));
    assert_eq!(Odr::Hz960.haodr_mhz(HaodrSet::Base1000), Some(1_000_000));
    assert_eq!(Odr::Hz960.haodr_mhz(HaodrSet::Base800), Some(800_000));
    assert_eq!(Odr::Hz960.haodr_mhz(HaodrSet::Base833), Some(833_000));
    assert_eq!(Odr::Hz15.haodr_mhz(HaodrSet::Base1000), Some(15_625));
    assert_eq!(Odr::Hz15.haodr_mhz(HaodrSet::Base800), Some(12_500));
    assert_eq!(Odr::Hz7680.haodr_mhz(HaodrSet::Base833), Some(6_667_000));
    // 7.5 Hz is reserved there, power-down has no rate.
    assert_eq!(Odr::Hz7_5.haodr_mhz(HaodrSet::Base960), None);
    assert_eq!(Odr::Off.haodr_mhz(HaodrSet::Base960), None);
}

#[test]
fn gyro_lpf1_table() {
    // Table 66, p70.
    assert_eq!(GyroLpf1Bandwidth::Bw0.cutoff_dhz(Odr::Hz7680), Some(2810));
    assert_eq!(GyroLpf1Bandwidth::Bw3.cutoff_dhz(Odr::Hz960), Some(3100));
    assert_eq!(GyroLpf1Bandwidth::Bw4.cutoff_dhz(Odr::Hz240), Some(784));
    assert_eq!(GyroLpf1Bandwidth::Bw5.cutoff_dhz(Odr::Hz7680), Some(580));
    assert_eq!(GyroLpf1Bandwidth::Bw6.cutoff_dhz(Odr::Hz60), Some(180));
    assert_eq!(GyroLpf1Bandwidth::Bw7.cutoff_dhz(Odr::Hz120), Some(136));
    assert_eq!(GyroLpf1Bandwidth::Bw7.cutoff_dhz(Odr::Hz30), None);
    assert_eq!(GyroLpf1Bandwidth::Bw7.cutoff_dhz(Odr::Off), None);
    let all = [
        GyroLpf1Bandwidth::Bw0,
        GyroLpf1Bandwidth::Bw1,
        GyroLpf1Bandwidth::Bw2,
        GyroLpf1Bandwidth::Bw3,
        GyroLpf1Bandwidth::Bw4,
        GyroLpf1Bandwidth::Bw5,
        GyroLpf1Bandwidth::Bw6,
        GyroLpf1Bandwidth::Bw7,
    ];
    assert_eq!(all.map(GyroLpf1Bandwidth::code), [0, 1, 2, 3, 4, 5, 6, 7]);
}

// ---------------------------------------------------------------------------------------------
// Register images
// ---------------------------------------------------------------------------------------------

#[test]
fn default_images_reproduce_init() {
    let c = Config::new();
    assert_eq!(c.if_cfg(), 0x01); // I2C_I3C_disable, p58
    assert_eq!(c.ctrl4(), 0x0A); // DRDY_MASK b3 (default on) and DRDY_PULSED b1, p68
    assert_eq!(c.int1_ctrl(), 0x01); // INT1_DRDY_XL b0, p63
    assert_eq!(c.int2_ctrl(), 0x00);
    assert_eq!(c.ctrl7(), 0x00);
    assert_eq!(c.ctrl8(), 0x02); // FS_XL = 10 (+-8 g), p71
    assert_eq!(c.ctrl6(), 0x0C); // bit 3 = 1, FS_G = 100 (+-2000 dps), p69
    assert_eq!(c.ctrl1_xl_hg(), 0x00);
    assert_eq!(c.ctrl1(), 0x09); // ODR_XL = 1001 (960 Hz), p65
    assert_eq!(c.ctrl2(), 0x09); // ODR_G = 1001, p66
}

#[test]
fn default_images_of_new_registers_are_reset_values() {
    let c = Config::new();
    assert_eq!(c.pin_ctrl(), 0x23); // p52 Table 26 and p57
    assert_eq!(c.ctrl9(), 0x00);
    assert_eq!(c.ctrl2_xl_hg(), 0x00);
    assert_eq!(c.functions_enable(), 0x00);
    assert_eq!(c.hg_functions_enable(), 0x00);
    assert_eq!(c.hg_wake_up_ths(), 0x00);
    assert_eq!(c.inactivity_ths(), 0x00);
    assert_eq!(c.md2_cfg(), 0x00);
    assert_eq!(c.haodr_cfg(), 0x00);
    assert_eq!(c.fifo_ctrl1(), 0x00);
    assert_eq!(c.fifo_ctrl2(), 0x00);
    assert_eq!(c.fifo_ctrl3(), 0x00);
    assert_eq!(c.fifo_ctrl4(), 0x00);
    assert_eq!(c.counter_bdr_reg1(), 0x00);
}

#[test]
fn image_filters() {
    // CTRL6 b6:4 LPF1_G_BW and CTRL7 b0 LPF1_G_EN, Tables 64 and 67, p69-70.
    let c = Config::new().with_gyro_lpf1(Some(GyroLpf1Bandwidth::Bw5));
    assert_eq!(c.ctrl6(), 0x08 | 0x04 | 0x50);
    assert_eq!(c.ctrl7(), 0x01);
    // CTRL8 b7:5 HP_LPF2_XL_BW and CTRL9 b3 LPF2_XL_EN, Tables 69 and 72, p71-72.
    let c = Config::new().with_accel_lpf2(Some(AccelLpf2Bandwidth::OdrDiv45));
    assert_eq!(c.ctrl8(), 0x02 | 0x60);
    assert_eq!(c.ctrl9(), 0x08);
    // The top setting.
    let c = Config::new().with_accel_lpf2(Some(AccelLpf2Bandwidth::OdrDiv800));
    assert_eq!(c.ctrl8(), 0x02 | 0xE0);
    // CTRL9 b5 XL_FASTSETTL_MODE, p72. CTRL4 b3 DRDY_MASK, p68.
    assert_eq!(Config::new().with_accel_fast_settling(true).ctrl9(), 0x20);
    assert_eq!(Config::new().with_drdy_mask(true).ctrl4(), 0x0A);
    assert!(Config::new().drdy_mask);
    assert_eq!(Config::new().with_drdy_mask(false).ctrl4(), 0x02);
    // CTRL4 b1 DRDY_PULSED = 0 is latched, p68.
    assert_eq!(
        Config::new()
            .with_data_ready_mode(DataReadyMode::Latched)
            .ctrl4(),
        0x08
    );
    assert_eq!(
        Config::new()
            .with_data_ready_mode(DataReadyMode::Latched)
            .with_drdy_mask(false)
            .ctrl4(),
        0x00
    );
}

#[test]
fn image_operating_mode() {
    // CTRL1/CTRL2 b6:4 = 001 high-accuracy ODR mode, Tables 53 and 56, p65-66.
    let c = Config::new().with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base800));
    assert_eq!(c.ctrl1(), 0x19);
    assert_eq!(c.ctrl2(), 0x19);
    assert_eq!(c.ctrl1_mode_only(), 0x10);
    assert_eq!(c.ctrl2_mode_only(), 0x10);
    // HAODR_SEL = 10, p103.
    assert_eq!(c.haodr_cfg(), 0x02);
    let c = Config::new().with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base833));
    assert_eq!(c.haodr_cfg(), 0x03);
    assert_eq!(Config::new().ctrl1_mode_only(), 0x00);
}

#[test]
fn image_data_ready_and_fifo_routing() {
    // INT1_CTRL, Table 47, p63: CNT_BDR b6, FIFO_FULL b5, FIFO_OVR b4, FIFO_TH b3, DRDY_G b1,
    // DRDY_XL b0. INT2_CTRL has the same positions, Table 49, p64.
    let r = IntRouting::new()
        .with_fifo_threshold(true)
        .with_fifo_overrun(true)
        .with_fifo_full(true)
        .with_data_ready(DataReady::new().with_gyro(true));
    let c = Config::new().with_int1(r).with_int2(r);
    assert_eq!(c.int1_ctrl(), 0x3A);
    assert_eq!(c.int2_ctrl(), 0x3A);
    let c = Config::new()
        .with_int1(IntRouting::new().with_fifo_overrun(true))
        .with_int2(IntRouting::new().with_fifo_full(true));
    assert_eq!(c.int1_ctrl(), 0x10);
    assert_eq!(c.int2_ctrl(), 0x20);
    // CTRL7 b7 INT1_DRDY_XL_HG, b6 INT2_DRDY_XL_HG, p70.
    let c = Config::new()
        .with_int1(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)))
        .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)));
    assert_eq!(c.ctrl7(), 0xC0);
    // MD2_CFG b0 INT2_TIMESTAMP, p103; FUNCTIONS_ENABLE b6 TIMESTAMP_EN, p93.
    let c = Config::new()
        .with_timestamp(true)
        .with_int2(IntRouting::new().with_timestamp_overflow(true));
    assert_eq!(c.md2_cfg(), 0x01);
    assert_eq!(c.functions_enable(), 0x40);
}

#[test]
fn image_fifo() {
    let c = Config::new().with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::Continuous)
            .with_watermark(100)
            .with_gyro(BatchRate::Hz960)
            .with_accel(BatchRate::Hz480)
            .with_high_g(true)
            .with_timestamp(TimestampBatch::Dec8)
            .with_temperature(TempBatchRate::Hz15),
    );
    assert_eq!(c.fifo_ctrl1(), 0x64); // WTM, p59
    assert_eq!(c.fifo_ctrl3(), 0x98); // BDR_GY = 1001, BDR_XL = 1000, p60
    assert_eq!(c.fifo_ctrl4(), 0xA6); // DEC_TS 10, ODR_T 10, mode 110, p61
    assert_eq!(c.counter_bdr_reg1(), 0x08); // XL_HG_BATCH_EN b3, p62
    let c = Config::new().with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::BypassToFifo)
            .with_timestamp(TimestampBatch::Dec32)
            .with_temperature(TempBatchRate::Hz60),
    );
    assert_eq!(c.fifo_ctrl4(), 0xF7);
}

#[test]
fn image_fifo_ctrl2() {
    // FIFO_CTRL2 b7 STOP_ON_WTM and b4 ODR_CHG_EN, Tables 37-38, p59.
    assert_eq!(
        Config::new()
            .with_fifo(FifoConfig::new().with_stop_on_wtm(true))
            .fifo_ctrl2(),
        0x80
    );
    assert_eq!(
        Config::new()
            .with_fifo(FifoConfig::new().with_odr_change(true))
            .fifo_ctrl2(),
        0x10
    );
    let both = FifoConfig::new()
        .with_stop_on_wtm(true)
        .with_odr_change(true);
    assert_eq!(Config::new().with_fifo(both).fifo_ctrl2(), 0x90);
    assert!(!FifoConfig::new().stop_on_wtm && !FifoConfig::new().odr_change);
}

#[test]
fn plan_writes_fifo_ctrl2_before_the_batch_rates() {
    let c = Config::new().with_fifo(
        fifo_on()
            .with_watermark(20)
            .with_stop_on_wtm(true)
            .with_odr_change(true),
    );
    let plan = c.plan(Apply::AfterReset).unwrap();
    let pos = |r: u8| {
        plan.steps()
            .iter()
            .position(|s| matches!(*s, Write { reg, .. } if reg == r))
    };
    assert_eq!(
        plan.steps()[pos(reg::FIFO_CTRL2).unwrap()],
        w(reg::FIFO_CTRL2, 0x90)
    );
    assert!(pos(reg::FIFO_CTRL2) < pos(reg::FIFO_CTRL4));
    // At its reset value it is not written after a reset.
    let plan = Config::new().plan(Apply::AfterReset).unwrap();
    assert_eq!(pos_of(&plan, reg::FIFO_CTRL2), None);
}

fn pos_of(plan: &Plan, r: u8) -> Option<usize> {
    plan.steps()
        .iter()
        .position(|s| matches!(*s, Write { reg, .. } if reg == r))
}

#[test]
fn validate_stop_on_wtm_needs_a_watermark() {
    // STOP_ON_WTM limits the FIFO depth to the watermark, 6.5.2, p45 and Table 38, p59.
    let c = Config::new().with_fifo(fifo_on().with_stop_on_wtm(true));
    assert_eq!(c.validate(), Err(ConfigError::FifoWatermarkZero));
    let c = Config::new().with_fifo(fifo_on().with_stop_on_wtm(true).with_watermark(1));
    assert_eq!(c.validate(), Ok(()));
    // In bypass the FIFO is off and the bit does nothing (6.5.1, p45).
    let c = Config::new().with_fifo(FifoConfig::new().with_stop_on_wtm(true));
    assert_eq!(c.validate(), Ok(()));
    // ODR_CHG_EN has no stated precondition.
    let c = Config::new().with_fifo(FifoConfig::new().with_odr_change(true));
    assert_eq!(c.validate(), Ok(()));
}

#[test]
fn image_pins() {
    // IF_CFG b4 H_LACTIVE, b3 PP_OD, b0 I2C_I3C_disable, Table 31, p58.
    let c = Config::new().with_pins(
        PinConfig::new()
            .with_polarity(IntPolarity::ActiveLow)
            .with_output(IntOutput::OpenDrain),
    );
    assert_eq!(c.if_cfg(), 0x19);
    // PIN_CTRL b5 IBHR_POR_EN (reset 1) and IO_PAD_STRENGTH b1:0, Table 29, p57.
    let pin = |d| {
        Config::new()
            .with_pins(PinConfig::new().with_drive(d))
            .pin_ctrl()
    };
    assert_eq!(pin(PadDrive::Lowest), 0x20);
    assert_eq!(pin(PadDrive::Intermediate), 0x21);
    assert_eq!(pin(PadDrive::Highest), 0x23);
}

#[test]
fn image_high_g() {
    // CTRL1_XL_HG: b7 XL_HG_REGOUT_EN, b6 HG_USR_OFF_ON_OUT, b5:3 ODR, b2:0 FS, Table 147, p91.
    let h = HighGConfig::new()
        .with_odr(HighGOdr::Hz1920)
        .with_full_scale(HighGFullScale::G320)
        .with_user_offset(true);
    let c = Config::new().with_high_g(Some(h));
    assert_eq!(c.ctrl1_xl_hg(), 0x80 | 0x40 | (0b101 << 3) | 0b100);
    let h = HighGConfig::new()
        .with_odr(HighGOdr::Hz480)
        .with_full_scale(HighGFullScale::G32);
    assert_eq!(
        Config::new().with_high_g(Some(h)).ctrl1_xl_hg(),
        0x80 | (0b011 << 3)
    );
}

#[test]
fn image_high_g_events() {
    // HG_FUNCTIONS_ENABLE: b7 HG_INTERRUPTS_ENABLE, b6 HG_WU_CHANGE_INT_SEL, b5 INT2_HG_WU,
    // b4 INT1_HG_WU, b3:0 HG_SHOCK_DUR, Table 156, p94. INACTIVITY_THS b7 INT2_HG_SHOCK_CHANGE,
    // b6 INT1_HG_SHOCK_CHANGE, p96. CTRL2_XL_HG b4 HG_USR_OFF_ON_WU, p91.
    let e = HighGEvents::new()
        .with_threshold(20)
        .with_wake_up_mode(HighGWakeUpMode::OnChange)
        .with_shock_duration(14)
        .with_user_offset(true);
    let c = Config::new()
        .with_high_g(Some(HighGConfig::new()))
        .with_high_g_events(Some(e))
        .with_int1(
            IntRouting::new()
                .with_high_g_wake_up(true)
                .with_high_g_shock(true),
        )
        .with_int2(IntRouting::new().with_high_g_wake_up(true));
    assert_eq!(c.hg_functions_enable(), 0xFE);
    assert_eq!(c.hg_wake_up_ths(), 20);
    assert_eq!(c.inactivity_ths(), 0x40);
    assert_eq!(c.ctrl2_xl_hg(), 0x10);
    let c = c.with_int2(IntRouting::new().with_high_g_shock(true));
    assert_eq!(c.inactivity_ths(), 0xC0);
    // Enabled with the defaults: only the enable bit.
    let c = Config::new().with_high_g_events(Some(HighGEvents::new()));
    assert_eq!(c.hg_functions_enable(), 0x80);
}

#[test]
fn image_user_offset() {
    // CTRL9 b0 USR_OFF_ON_OUT, b1 USR_OFF_W, Table 72, p72.
    let c = Config::new().with_accel_offset(Some(OffsetWeight::Fine));
    assert_eq!(c.ctrl9(), 0x01);
    let c = Config::new().with_accel_offset(Some(OffsetWeight::Coarse));
    assert_eq!(c.ctrl9(), 0x03);
    assert_eq!(Config::new().ctrl9(), 0x00);
}

#[test]
fn self_test_images() {
    use SelfTestSign::*;
    // CTRL10: b3:2 ST_G, b1:0 ST_XL; 01 positive, 10 negative, p74.
    assert_eq!(ctrl10_self_test(Positive, Off), 0x01);
    assert_eq!(ctrl10_self_test(Negative, Off), 0x02);
    assert_eq!(ctrl10_self_test(Off, Positive), 0x04);
    assert_eq!(ctrl10_self_test(Off, Negative), 0x08);
    assert_eq!(ctrl10_self_test(Negative, Positive), 0x06);
    assert_eq!(ctrl10_self_test(Off, Off), 0x00);
    // CTRL2_XL_HG: b1:0 XL_HG_ST, p91.
    assert_eq!(ctrl2_xl_hg_self_test(Positive), 0x01);
    assert_eq!(ctrl2_xl_hg_self_test(Negative), 0x02);
}

#[test]
fn self_test_limits() {
    // Table 3, p13.
    assert_eq!((SELF_TEST_LOW_G_MG.min, SELF_TEST_LOW_G_MG.max), (50, 1700));
    assert_eq!(
        (SELF_TEST_HIGH_G_MG.min, SELF_TEST_HIGH_G_MG.max),
        (150, 2500)
    );
    assert_eq!(
        (
            SELF_TEST_GYRO_250_DPS_MDPS.min,
            SELF_TEST_GYRO_250_DPS_MDPS.max
        ),
        (20_000, 80_000)
    );
    assert_eq!(
        (
            SELF_TEST_GYRO_4000_DPS_MDPS.min,
            SELF_TEST_GYRO_4000_DPS_MDPS.max
        ),
        (150_000, 700_000)
    );
    assert!(SELF_TEST_LOW_G_MG.contains(50));
    assert!(SELF_TEST_LOW_G_MG.contains(1700));
    assert!(!SELF_TEST_LOW_G_MG.contains(49));
    assert!(!SELF_TEST_LOW_G_MG.contains(1701));
}

// ---------------------------------------------------------------------------------------------
// Config construction
// ---------------------------------------------------------------------------------------------

const BUILT: Config = Config::new()
    .with_accel_fs(AccelFullScale::G16)
    .with_accel_odr(Odr::Hz1920)
    .with_gyro_fs(GyroFullScale::Dps4000)
    .with_gyro_odr(Odr::Hz7680)
    .with_high_g(Some(
        HighGConfig::new()
            .with_odr(HighGOdr::Hz3840)
            .with_full_scale(HighGFullScale::G256)
            .with_user_offset(true),
    ))
    .with_int1(IntRouting::new().with_fifo_threshold(true))
    .with_int2(IntRouting::new().with_timestamp_overflow(true))
    .with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base1000))
    .with_gyro_lpf1(Some(GyroLpf1Bandwidth::Bw2))
    .with_accel_lpf2(Some(AccelLpf2Bandwidth::OdrDiv10))
    .with_drdy_mask(true)
    .with_accel_fast_settling(true)
    .with_data_ready_mode(DataReadyMode::Latched)
    .with_timestamp(true)
    .with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::Continuous)
            .with_watermark(64),
    )
    .with_pins(PinConfig::new().with_drive(PadDrive::Lowest))
    .with_high_g_events(Some(HighGEvents::new().with_threshold(5)))
    .with_accel_offset(Some(OffsetWeight::Coarse));

#[test]
fn config_new_equals_default() {
    assert_eq!(Config::new(), Config::default());
    assert_eq!(HighGConfig::new(), HighGConfig::default());
    assert_eq!(DataReady::new(), DataReady::default());
    assert_eq!(DataReady::NONE, DataReady::default());
    assert_eq!(IntRouting::new(), IntRouting::default());
    assert_eq!(FifoConfig::new(), FifoConfig::default());
    assert_eq!(PinConfig::new(), PinConfig::default());
    assert_eq!(HighGEvents::new(), HighGEvents::default());
    assert_eq!(Offsets::new(), Offsets::default());
    // What `init` has always configured.
    let c = Config::new();
    assert_eq!(c.accel_fs, AccelFullScale::G8);
    assert_eq!(c.accel_odr, Odr::Hz960);
    assert_eq!(c.gyro_fs, GyroFullScale::Dps2000);
    assert_eq!(c.gyro_odr, Odr::Hz960);
    assert_eq!(c.high_g, None);
    assert!(c.int1.data_ready.accel && !c.int1.data_ready.gyro && !c.int1.data_ready.high_g);
    assert_eq!(c.int2, IntRouting::new());
    assert_eq!(c.mode, OperatingMode::HighPerformance);
    assert_eq!(c.data_ready_mode, DataReadyMode::Pulsed);
}

#[test]
fn config_const_with_setters() {
    assert_eq!(BUILT.accel_fs, AccelFullScale::G16);
    assert_eq!(BUILT.gyro_odr, Odr::Hz7680);
    assert_eq!(
        BUILT.mode,
        OperatingMode::HighAccuracyOdr(HaodrSet::Base1000)
    );
    assert_eq!(BUILT.fifo.watermark, 64);
    assert_eq!(BUILT.high_g_events.map(|e| e.threshold), Some(5));
    assert_eq!(BUILT.accel_offset, Some(OffsetWeight::Coarse));
    // Each setter changes exactly its field.
    assert_ne!(BUILT, Config::new());
    assert_eq!(
        BUILT.with_accel_fs(AccelFullScale::G8).accel_fs,
        AccelFullScale::G8
    );
    assert_eq!(
        BUILT
            .with_accel_fs(AccelFullScale::G8)
            .with_accel_fs(AccelFullScale::G16),
        BUILT
    );
}

// ---------------------------------------------------------------------------------------------
// Validation: every ConfigError variant
// ---------------------------------------------------------------------------------------------

fn fifo_on() -> FifoConfig {
    FifoConfig::new().with_mode(FifoMode::Continuous)
}

fn with_high_g() -> Config {
    Config::new().with_high_g(Some(HighGConfig::new()))
}

#[test]
fn validate_accepts_defaults_and_a_rich_config() {
    assert_eq!(Config::new().validate(), Ok(()));
    let rich = with_high_g()
        .with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base833))
        .with_timestamp(true)
        .with_fifo(
            fifo_on()
                .with_watermark(214)
                .with_gyro(BatchRate::Hz960)
                .with_accel(BatchRate::Hz960)
                .with_high_g(true)
                .with_timestamp(TimestampBatch::Dec32)
                .with_temperature(TempBatchRate::Hz60),
        )
        .with_int1(
            IntRouting::new()
                .with_fifo_threshold(true)
                .with_fifo_overrun(true),
        )
        .with_int2(
            IntRouting::new()
                .with_timestamp_overflow(true)
                .with_fifo_full(true),
        )
        .with_high_g_events(Some(HighGEvents::new().with_shock_duration(14)));
    assert_eq!(rich.validate(), Ok(()));
    assert_eq!(BUILT.validate(), Ok(()));
}

#[test]
fn validate_haodr_odr_reserved() {
    // Table 22, p34: 7.5 Hz is reserved in the high-accuracy ODR mode.
    let m = OperatingMode::HighAccuracyOdr(HaodrSet::Base960);
    assert_eq!(
        Config::new()
            .with_mode(m)
            .with_accel_odr(Odr::Hz7_5)
            .validate(),
        Err(ConfigError::HaodrOdrReserved {
            channel: Channel::LowG
        })
    );
    assert_eq!(
        Config::new()
            .with_mode(m)
            .with_gyro_odr(Odr::Hz7_5)
            .validate(),
        Err(ConfigError::HaodrOdrReserved {
            channel: Channel::Gyro
        })
    );
    // 7.5 Hz is fine in high-performance mode.
    assert_eq!(Config::new().with_gyro_odr(Odr::Hz7_5).validate(), Ok(()));
}

#[test]
fn validate_batch_rate_above_odr() {
    let c = Config::new().with_fifo(fifo_on().with_gyro(BatchRate::Hz1920));
    assert_eq!(
        c.validate(),
        Err(ConfigError::BatchRateAboveOdr {
            channel: Channel::Gyro
        })
    );
    let c = Config::new().with_fifo(fifo_on().with_accel(BatchRate::Hz1920));
    assert_eq!(
        c.validate(),
        Err(ConfigError::BatchRateAboveOdr {
            channel: Channel::LowG
        })
    );
    // Batching a channel in power-down.
    let c = Config::new()
        .with_gyro_odr(Odr::Off)
        .with_fifo(fifo_on().with_gyro(BatchRate::Hz1_875));
    assert_eq!(
        c.validate(),
        Err(ConfigError::BatchRateAboveOdr {
            channel: Channel::Gyro
        })
    );
    // Equal and lower rates are fine.
    let c = Config::new().with_fifo(
        fifo_on()
            .with_gyro(BatchRate::Hz960)
            .with_accel(BatchRate::Hz7_5),
    );
    assert_eq!(c.validate(), Ok(()));
}

#[test]
fn validate_high_g_batch_needs_high_g() {
    let c = Config::new().with_fifo(fifo_on().with_high_g(true));
    assert_eq!(c.validate(), Err(ConfigError::HighGBatchNeedsHighG));
    assert_eq!(
        with_high_g()
            .with_fifo(fifo_on().with_high_g(true))
            .validate(),
        Ok(())
    );
}

#[test]
fn validate_watermark() {
    // The FIFO holds 256 words (measured on the board, 2026-11-10); the field is 8 bits, so every
    // value it can hold, up to 255, is accepted.
    assert_eq!(FIFO_DEPTH_WORDS, 256);
    for wm in [214, 215, 219, 220, 254, 255] {
        let c = Config::new().with_fifo(fifo_on().with_watermark(wm));
        assert_eq!(c.validate(), Ok(()), "watermark {wm}");
        assert_eq!(c.fifo_ctrl1(), wm);
    }
    for wm in 0..=u8::MAX {
        let fifo = fifo_on().with_watermark(wm);
        let c = Config::new().with_fifo(fifo.with_stop_on_wtm(wm != 0));
        assert_eq!(c.validate(), Ok(()), "watermark {wm}");
        let c = Config::new().with_fifo(fifo.with_stop_on_wtm(false));
        assert_eq!(c.validate(), Ok(()), "watermark {wm}");
    }
    // Zero while the threshold interrupt or the WTM-limited mode uses it.
    let c = Config::new()
        .with_fifo(fifo_on())
        .with_int1(IntRouting::new().with_fifo_threshold(true));
    assert_eq!(c.validate(), Err(ConfigError::FifoWatermarkZero));
    let c = with_high_g()
        .with_high_g_events(Some(HighGEvents::new()))
        .with_fifo(FifoConfig::new().with_mode(FifoMode::ContinuousWtmToFull));
    assert_eq!(c.validate(), Err(ConfigError::FifoWatermarkZero));
    let c = c.with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::ContinuousWtmToFull)
            .with_watermark(1),
    );
    assert_eq!(c.validate(), Ok(()));
}

#[test]
fn validate_fifo_interrupt_needs_fifo() {
    for r in [
        IntRouting::new().with_fifo_threshold(true),
        IntRouting::new().with_fifo_overrun(true),
        IntRouting::new().with_fifo_full(true),
    ] {
        let c = Config::new()
            .with_fifo(FifoConfig::new().with_watermark(10))
            .with_int2(r);
        assert_eq!(c.validate(), Err(ConfigError::FifoInterruptNeedsFifo));
    }
}

#[test]
fn validate_fifo_trigger_needs_high_g_events() {
    for mode in [
        FifoMode::ContinuousWtmToFull,
        FifoMode::ContinuousToFifo,
        FifoMode::BypassToContinuous,
        FifoMode::BypassToFifo,
    ] {
        let c = Config::new().with_fifo(FifoConfig::new().with_mode(mode).with_watermark(5));
        assert_eq!(c.validate(), Err(ConfigError::FifoTriggerNeedsHighGEvents));
    }
    for mode in [FifoMode::Fifo, FifoMode::Continuous] {
        let c = Config::new().with_fifo(FifoConfig::new().with_mode(mode));
        assert_eq!(c.validate(), Ok(()));
    }
}

#[test]
fn validate_timestamp() {
    let c = Config::new().with_fifo(fifo_on().with_timestamp(TimestampBatch::Dec1));
    assert_eq!(c.validate(), Err(ConfigError::TimestampBatchNeedsCounter));
    assert_eq!(c.with_timestamp(true).validate(), Ok(()));
    let c = Config::new()
        .with_timestamp(true)
        .with_int1(IntRouting::new().with_timestamp_overflow(true));
    assert_eq!(c.validate(), Err(ConfigError::TimestampOverflowOnInt1));
    let c = Config::new().with_int2(IntRouting::new().with_timestamp_overflow(true));
    assert_eq!(
        c.validate(),
        Err(ConfigError::TimestampOverflowNeedsCounter)
    );
    assert_eq!(c.with_timestamp(true).validate(), Ok(()));
}

#[test]
fn validate_high_g_events() {
    let c = Config::new().with_high_g_events(Some(HighGEvents::new()));
    assert_eq!(c.validate(), Err(ConfigError::HighGEventsNeedHighG));
    assert_eq!(
        with_high_g()
            .with_high_g_events(Some(HighGEvents::new()))
            .validate(),
        Ok(())
    );
    for r in [
        IntRouting::new().with_high_g_wake_up(true),
        IntRouting::new().with_high_g_shock(true),
    ] {
        assert_eq!(
            with_high_g().with_int1(r).validate(),
            Err(ConfigError::HighGRoutingNeedsEvents)
        );
        assert_eq!(
            with_high_g().with_int2(r).validate(),
            Err(ConfigError::HighGRoutingNeedsEvents)
        );
    }
    // p94: HG_SHOCK_DUR is 0 to 14.
    let e = HighGEvents::new().with_shock_duration(15);
    assert_eq!(
        with_high_g().with_high_g_events(Some(e)).validate(),
        Err(ConfigError::HighGShockDurationOutOfRange { value: 15 })
    );
}

#[test]
fn validate_offsets() {
    // p108, p113: -127 to 127.
    assert_eq!(Offsets::new().validate(), Ok(()));
    assert_eq!(
        Offsets::new()
            .with_low_g([-127, 127, 0])
            .with_high_g([127, -127, 1])
            .validate(),
        Ok(())
    );
    assert_eq!(
        Offsets::new().with_low_g([0, -128, 0]).validate(),
        Err(ConfigError::OffsetOutOfRange {
            channel: Channel::LowG
        })
    );
    assert_eq!(
        Offsets::new().with_high_g([0, 0, -128]).validate(),
        Err(ConfigError::OffsetOutOfRange {
            channel: Channel::HighG
        })
    );
}

#[test]
fn offset_bytes() {
    let o = Offsets::new()
        .with_low_g([1, -1, -127])
        .with_high_g([127, 0, -2]);
    assert_eq!(o.low_g_bytes(), [0x01, 0xFF, 0x81]);
    assert_eq!(o.high_g_bytes(), [0x7F, 0x00, 0xFE]);
}

// ---------------------------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------------------------

use Step::{DelayUs, Modify, Write};

fn w(reg: u8, value: u8) -> Step {
    Write { reg, value }
}

/// The register writes `init` made after the identity check before the settings layer, in order,
/// then the gyroscope turn-on wait; with two lead decisions applied: PIN_CTRL is always written
/// (it survives a software reset, p57, so the pad state must not depend on an earlier program) and
/// DRDY_MASK is on by default (CTRL4 0x0A instead of 0x02, p68).
fn legacy_init_steps() -> Vec<Step> {
    std::vec![
        // CTRL3 (p67): BDU | IF_INC, written first in both plans.
        w(reg::CTRL3, 0x44),
        ctrl10_off(),
        w(reg::IF_CFG, 0x01),
        w(reg::PIN_CTRL, 0x23),
        w(reg::CTRL4, 0x0A),
        w(reg::INT1_CTRL, 0x01),
        w(reg::INT2_CTRL, 0x00),
        w(reg::CTRL7, 0x00),
        w(reg::CTRL8, 0x02),
        w(reg::CTRL6, 0x0C),
        w(reg::CTRL1_XL_HG, 0x00),
        w(reg::CTRL1, 0x09),
        w(reg::CTRL2, 0x09),
        DelayUs(40_000),
    ]
}

/// The step that switches ST_XL (1:0) and ST_G (3:2) of CTRL10 off and leaves the other bits (p74).
fn ctrl10_off() -> Step {
    Modify {
        reg: reg::CTRL10,
        mask: 0x0F,
        value: 0,
    }
}

#[test]
fn plan_default_is_the_legacy_init_plus_pin_ctrl() {
    let plan = Config::new().plan(Apply::AfterReset).unwrap();
    assert_eq!(plan.steps(), legacy_init_steps().as_slice());
    assert_eq!(plan.len(), 14);
    assert!(!plan.is_empty());
    assert_eq!(plan.iter().count(), 14);
    assert_eq!((&plan).into_iter().count(), 14);
}

#[test]
fn plan_legacy_with_high_g_and_gyro_off() {
    // The configuration of the blocking `init_then_read` test.
    let c = Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_gyro_odr(Odr::Hz240)
        .with_high_g(Some(
            HighGConfig::new()
                .with_odr(HighGOdr::Hz960)
                .with_full_scale(HighGFullScale::G64),
        ))
        .with_int2(IntRouting::new().with_data_ready(DataReady::new().with_high_g(true)));
    let plan = c.plan(Apply::AfterReset).unwrap();
    assert_eq!(
        plan.steps(),
        [
            w(reg::CTRL3, 0x44),
            ctrl10_off(),
            w(reg::IF_CFG, 0x01),
            w(reg::PIN_CTRL, 0x23),
            w(reg::CTRL4, 0x0A),
            w(reg::INT1_CTRL, 0x01),
            w(reg::INT2_CTRL, 0x00),
            w(reg::CTRL7, 0x40),
            w(reg::CTRL8, 0x01),
            w(reg::CTRL6, 0x0A),
            w(reg::CTRL1_XL_HG, 0x80 | (4 << 3) | 1),
            w(reg::CTRL1, 0x09),
            w(reg::CTRL2, 0x07),
            DelayUs(40_000),
        ]
    );
    let off = c.with_gyro_odr(Odr::Off).plan(Apply::AfterReset).unwrap();
    assert_eq!(off.steps().last(), Some(&w(reg::CTRL2, 0x00)));
    assert!(!off.steps().iter().any(|s| matches!(s, DelayUs(_))));
}

#[test]
fn plan_orders_new_registers_around_the_legacy_ones() {
    let c = with_high_g()
        .with_timestamp(true)
        .with_pins(PinConfig::new().with_drive(PadDrive::Lowest))
        .with_accel_lpf2(Some(AccelLpf2Bandwidth::OdrDiv20))
        .with_accel_offset(Some(OffsetWeight::Fine))
        .with_high_g_events(Some(HighGEvents::new().with_threshold(30)))
        .with_int1(
            IntRouting::new()
                .with_high_g_shock(true)
                .with_data_ready(DataReady::new().with_accel(true)),
        )
        .with_int2(IntRouting::new().with_timestamp_overflow(true))
        .with_fifo(
            fifo_on()
                .with_watermark(50)
                .with_gyro(BatchRate::Hz480)
                .with_timestamp(TimestampBatch::Dec8),
        );
    let plan = c.plan(Apply::AfterReset).unwrap();
    let regs: Vec<u8> = plan
        .steps()
        .iter()
        .filter_map(|s| match *s {
            Write { reg, .. } | Modify { reg, .. } => Some(reg),
            _ => None,
        })
        .collect();
    assert_eq!(
        regs,
        std::vec![
            reg::CTRL3,
            reg::CTRL10,
            reg::IF_CFG,
            reg::PIN_CTRL,
            reg::CTRL4,
            reg::INT1_CTRL,
            reg::INT2_CTRL,
            reg::MD2_CFG,
            reg::FUNCTIONS_ENABLE,
            reg::HG_WAKE_UP_THS,
            reg::INACTIVITY_THS,
            reg::HG_FUNCTIONS_ENABLE,
            reg::CTRL7,
            reg::CTRL8,
            reg::CTRL9,
            reg::CTRL6,
            reg::FIFO_CTRL1,
            reg::FIFO_CTRL3,
            reg::FIFO_CTRL4,
            reg::CTRL1_XL_HG,
            reg::CTRL1,
            reg::CTRL2,
        ]
    );
    assert!(plan.steps().contains(&Modify {
        reg: reg::MD2_CFG,
        mask: 0x01,
        value: 0x01
    }));
    assert!(plan.steps().contains(&Modify {
        reg: reg::FUNCTIONS_ENABLE,
        mask: 0x40,
        value: 0x40
    }));
    assert!(plan.steps().contains(&Modify {
        reg: reg::INACTIVITY_THS,
        mask: 0xC0,
        value: 0x40
    }));
    assert_eq!(plan.steps().last(), Some(&DelayUs(40_000)));
    // The ODRs come last, the FIFO mode before them.
    assert_eq!(Step::modified(0x3F, 0xC0, 0x40), 0x7F);
}

#[test]
fn plan_haodr_sets_the_mode_in_power_down_first() {
    let c = Config::new().with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base1000));
    let plan = c.plan(Apply::AfterReset).unwrap();
    // p34: the mode is enabled in power-down, then the ODRs are written.
    let tail: Vec<Step> = plan.steps().iter().rev().take(6).rev().copied().collect();
    assert_eq!(
        tail,
        std::vec![
            w(reg::CTRL1, 0x10),
            w(reg::CTRL2, 0x10),
            w(reg::CTRL1_XL_HG, 0x00),
            w(reg::CTRL1, 0x19),
            w(reg::CTRL2, 0x19),
            DelayUs(40_000),
        ]
    );
    assert!(plan.steps().contains(&w(reg::HAODR_CFG, 0x01)));
    let pos = |s: Step| plan.steps().iter().position(|x| *x == s).unwrap();
    assert!(pos(w(reg::HAODR_CFG, 0x01)) < pos(w(reg::CTRL1, 0x10)));
    assert!(pos(w(reg::CTRL6, 0x0C)) < pos(w(reg::CTRL1, 0x10)));
}

#[test]
fn plan_in_place_starts_with_bypass_and_power_down() {
    let plan = Config::new().plan(Apply::InPlace).unwrap();
    assert_eq!(
        plan.steps().get(..6),
        Some(
            [
                w(reg::CTRL3, 0x44),
                ctrl10_off(),
                w(reg::FIFO_CTRL4, 0x00),
                w(reg::CTRL1, 0x00),
                w(reg::CTRL2, 0x00),
                w(reg::CTRL1_XL_HG, 0x00),
            ]
            .as_slice()
        )
    );
    // Everything is written, including the registers at their reset values.
    for r in [
        reg::PIN_CTRL,
        reg::CTRL9,
        reg::HAODR_CFG,
        reg::FIFO_CTRL1,
        reg::FIFO_CTRL2,
        reg::FIFO_CTRL3,
        reg::FIFO_CTRL4,
        reg::COUNTER_BDR_REG1,
        reg::HG_FUNCTIONS_ENABLE,
        reg::HG_WAKE_UP_THS,
        reg::CTRL2_XL_HG,
    ] {
        assert!(
            plan.steps()
                .iter()
                .any(|s| matches!(*s, Write { reg, .. } if reg == r)),
            "register {r:#04x} not written"
        );
    }
    assert!(plan.steps().iter().any(|s| matches!(
        *s,
        Modify { reg: r, value: 0, .. } if r == reg::INACTIVITY_THS
    )));
    assert_eq!(plan.len(), 31);
    // The last non-delay step is the gyroscope ODR.
    assert_eq!(plan.steps().get(plan.len() - 2), Some(&w(reg::CTRL2, 0x09)));
}

#[test]
fn plan_rejects_invalid_config() {
    let c = Config::new()
        .with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base960))
        .with_accel_odr(Odr::Hz7_5);
    assert_eq!(
        c.plan(Apply::AfterReset),
        Err(ConfigError::HaodrOdrReserved {
            channel: Channel::LowG
        })
    );
    assert_eq!(c.plan(Apply::InPlace), c.plan(Apply::AfterReset));
}

#[test]
fn plan_longest_fits() {
    let c = BUILT
        .with_mode(OperatingMode::HighAccuracyOdr(HaodrSet::Base1000))
        .with_fifo(
            fifo_on()
                .with_watermark(10)
                .with_high_g(true)
                .with_timestamp(TimestampBatch::Dec1),
        )
        .with_int1(IntRouting::new().with_high_g_wake_up(true))
        .with_high_g_events(Some(HighGEvents::new()));
    let plan = c.plan(Apply::InPlace).unwrap();
    assert_eq!(plan.len(), 33);
    assert!(plan.len() < MAX_STEPS);
}

// A small model of the register file to execute a plan.

fn execute(regs: &mut [u8; 128], plan: &Plan) {
    for step in plan {
        match *step {
            Write { reg, value } => regs[reg as usize] = value,
            Modify { reg, mask, value } => {
                regs[reg as usize] = Step::modified(regs[reg as usize], mask, value);
            }
            DelayUs(_) => {}
        }
    }
}

fn reset_regs() -> [u8; 128] {
    let mut r = [0u8; 128];
    r[reg::PIN_CTRL as usize] = 0x23;
    r[reg::CTRL3 as usize] = 0x44;
    r[reg::CTRL6 as usize] = 0x08;
    r[reg::INACTIVITY_DUR as usize] = 0x04;
    r
}

/// (register, bits the configuration owns, their value) for every register the plan may write.
fn owned(c: &Config) -> Vec<(u8, u8, u8)> {
    std::vec![
        (reg::CTRL3, 0xFF, c.ctrl3()),
        (reg::CTRL10, 0x0F, 0),
        (reg::IF_CFG, 0xFF, c.if_cfg()),
        (reg::PIN_CTRL, 0xFF, c.pin_ctrl()),
        (reg::CTRL4, 0xFF, c.ctrl4()),
        (reg::INT1_CTRL, 0xFF, c.int1_ctrl()),
        (reg::INT2_CTRL, 0xFF, c.int2_ctrl()),
        (reg::MD2_CFG, Config::MD2_CFG_MASK, c.md2_cfg()),
        (
            reg::FUNCTIONS_ENABLE,
            Config::FUNCTIONS_ENABLE_MASK,
            c.functions_enable()
        ),
        (reg::HG_WAKE_UP_THS, 0xFF, c.hg_wake_up_ths()),
        (
            reg::INACTIVITY_THS,
            Config::INACTIVITY_THS_MASK,
            c.inactivity_ths()
        ),
        (reg::CTRL2_XL_HG, 0xFF, c.ctrl2_xl_hg()),
        (reg::HG_FUNCTIONS_ENABLE, 0xFF, c.hg_functions_enable()),
        (reg::CTRL7, 0xFF, c.ctrl7()),
        (reg::CTRL8, 0xFF, c.ctrl8()),
        (reg::CTRL9, 0xFF, c.ctrl9()),
        (reg::CTRL6, 0xFF, c.ctrl6()),
        (reg::HAODR_CFG, 0xFF, c.haodr_cfg()),
        (reg::FIFO_CTRL1, 0xFF, c.fifo_ctrl1()),
        (reg::COUNTER_BDR_REG1, 0xFF, c.counter_bdr_reg1()),
        (reg::FIFO_CTRL2, 0xFF, c.fifo_ctrl2()),
        (reg::FIFO_CTRL3, 0xFF, c.fifo_ctrl3()),
        (reg::FIFO_CTRL4, 0xFF, c.fifo_ctrl4()),
        (reg::CTRL1_XL_HG, 0xFF, c.ctrl1_xl_hg()),
        (reg::CTRL1, 0xFF, c.ctrl1()),
        (reg::CTRL2, 0xFF, c.ctrl2()),
    ]
}

#[test]
fn plan_execution_reaches_the_images() {
    let c = BUILT;
    for apply in [Apply::AfterReset, Apply::InPlace] {
        let plan = c.plan(apply).unwrap();
        let mut regs = reset_regs();
        execute(&mut regs, &plan);
        for (r, mask, value) in owned(&c) {
            assert_eq!(
                regs[r as usize] & mask,
                value,
                "register {r:#04x} {apply:?}"
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Frequency helpers
// ---------------------------------------------------------------------------------------------

#[test]
fn freq_fine_real_odr() {
    // 7680 Hz * (1 - 0.0013 * 14) = 7540.224 Hz, the rate measured on the board (-1.8 %).
    assert_eq!(real_odr_mhz(Odr::Hz7680, -14), 7_540_224);
    assert_eq!(real_odr_mhz(Odr::Hz7680, 0), 7_680_000);
    assert_eq!(real_odr_mhz(Odr::Hz7680, 127), 8_947_968);
    assert_eq!(real_odr_mhz(Odr::Hz7680, -128), 6_402_048);
    assert_eq!(real_odr_mhz(Odr::Hz7680, 1), 7_689_984);
    // ODRcoeff 8 at 960 Hz (Table 153, p92): 960 Hz * 0.9818 = 942.528 Hz.
    assert_eq!(real_odr_mhz(Odr::Hz960, -14), 942_528);
    // 7.5 Hz: 7500 mHz * 0.9818 = 7363.5, half rounds up.
    assert_eq!(real_odr_mhz(Odr::Hz7_5, -14), 7_364);
    assert_eq!(real_odr_mhz(Odr::Hz7_5, 0), 7_500);
    assert_eq!(real_odr_mhz(Odr::Off, 50), 0);
    for odr in [
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
    ] {
        // The coefficient table gives the nominal rate at FREQ_FINE = 0.
        assert_eq!(real_odr_mhz(odr, 0), odr.nominal_mhz());
    }
    assert_eq!(
        [
            Odr::Hz7_5,
            Odr::Hz15,
            Odr::Hz30,
            Odr::Hz60,
            Odr::Hz120,
            Odr::Hz240
        ]
        .map(odr_coeff),
        [
            Some(1024),
            Some(512),
            Some(256),
            Some(128),
            Some(64),
            Some(32)
        ]
    );
    assert_eq!(odr_coeff(Odr::Hz7680), Some(1));
    assert_eq!(odr_coeff(Odr::Off), None);
    assert_eq!(scale_rate_mhz(1_000_000, 10), 1_013_000);
}

#[test]
fn freq_fine_timestamp_tick() {
    // 1 / (46080 Hz * factor): 21.701 us at 0 (the datasheet's 21.7 us, p85).
    assert_eq!(timestamp_tick_ns(0), 21_701);
    assert_eq!(timestamp_tick_ns(-14), 22_104);
    assert_eq!(timestamp_tick_ns(127), 18_626);
    assert_eq!(timestamp_tick_ns(-128), 26_033);
    assert_eq!(timestamp_tick_ns(1), 21_673);
}

#[test]
fn offset_conversions() {
    // p113: 2^-10 g = 976.5625 ug, 2^-6 g = 15625 ug.
    assert_eq!(low_g_offset_ug(1, OffsetWeight::Fine), 977);
    assert_eq!(low_g_offset_ug(-1, OffsetWeight::Fine), -977);
    assert_eq!(low_g_offset_ug(16, OffsetWeight::Fine), 15_625);
    assert_eq!(low_g_offset_ug(127, OffsetWeight::Fine), 124_023);
    assert_eq!(low_g_offset_ug(1, OffsetWeight::Coarse), 15_625);
    assert_eq!(low_g_offset_ug(-127, OffsetWeight::Coarse), -1_984_375);
    assert_eq!(low_g_offset_ug(0, OffsetWeight::Coarse), 0);
    // p108: 0.25 g per LSB, 0.33 g at +-320 g.
    assert_eq!(high_g_offset_mg(4, HighGFullScale::G32), 1000);
    assert_eq!(high_g_offset_mg(-127, HighGFullScale::G256), -31_750);
    assert_eq!(high_g_offset_mg(10, HighGFullScale::G320), 3300);
    // p94: 1 g per LSB, 1.25 g at +-320 g.
    assert_eq!(hg_wake_up_threshold_mg(20, HighGFullScale::G128), 20_000);
    assert_eq!(hg_wake_up_threshold_mg(255, HighGFullScale::G256), 255_000);
    assert_eq!(hg_wake_up_threshold_mg(20, HighGFullScale::G320), 25_000);
}

// ---------------------------------------------------------------------------------------------
// Properties
// ---------------------------------------------------------------------------------------------

fn odr_s() -> impl Strategy<Value = Odr> {
    proptest::sample::select(std::vec![
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
    ])
}

fn batch_s() -> impl Strategy<Value = BatchRate> {
    proptest::sample::select(std::vec![
        BatchRate::Off,
        BatchRate::Hz1_875,
        BatchRate::Hz7_5,
        BatchRate::Hz15,
        BatchRate::Hz30,
        BatchRate::Hz60,
        BatchRate::Hz120,
        BatchRate::Hz240,
        BatchRate::Hz480,
        BatchRate::Hz960,
        BatchRate::Hz1920,
        BatchRate::Hz3840,
        BatchRate::Hz7680,
    ])
}

fn mode_s() -> impl Strategy<Value = OperatingMode> {
    proptest::sample::select(std::vec![
        OperatingMode::HighPerformance,
        OperatingMode::HighAccuracyOdr(HaodrSet::Base960),
        OperatingMode::HighAccuracyOdr(HaodrSet::Base1000),
        OperatingMode::HighAccuracyOdr(HaodrSet::Base800),
        OperatingMode::HighAccuracyOdr(HaodrSet::Base833),
    ])
}

fn fifo_mode_s() -> impl Strategy<Value = FifoMode> {
    proptest::sample::select(std::vec![
        FifoMode::Bypass,
        FifoMode::Fifo,
        FifoMode::ContinuousWtmToFull,
        FifoMode::ContinuousToFifo,
        FifoMode::BypassToContinuous,
        FifoMode::Continuous,
        FifoMode::BypassToFifo,
    ])
}

fn routing_s() -> impl Strategy<Value = IntRouting> {
    (any::<[bool; 3]>(), any::<[bool; 6]>()).prop_map(|(d, f)| {
        IntRouting::new()
            .with_data_ready(
                DataReady::new()
                    .with_accel(d[0])
                    .with_gyro(d[1])
                    .with_high_g(d[2]),
            )
            .with_fifo_threshold(f[0])
            .with_fifo_overrun(f[1])
            .with_fifo_full(f[2])
            .with_high_g_wake_up(f[3])
            .with_high_g_shock(f[4])
            .with_timestamp_overflow(f[5])
    })
}

fn fifo_s() -> impl Strategy<Value = FifoConfig> {
    (
        fifo_mode_s(),
        any::<u8>(),
        batch_s(),
        batch_s(),
        any::<bool>(),
        0u8..4,
        0u8..4,
        any::<bool>(),
        any::<bool>(),
    )
        .prop_map(|(mode, wm, g, a, hg, ts, t, stop, chg)| {
            FifoConfig::new()
                .with_mode(mode)
                .with_watermark(wm)
                .with_gyro(g)
                .with_accel(a)
                .with_high_g(hg)
                .with_stop_on_wtm(stop)
                .with_odr_change(chg)
                .with_timestamp(
                    [
                        TimestampBatch::Off,
                        TimestampBatch::Dec1,
                        TimestampBatch::Dec8,
                        TimestampBatch::Dec32,
                    ][ts as usize],
                )
                .with_temperature(
                    [
                        TempBatchRate::Off,
                        TempBatchRate::Hz1_875,
                        TempBatchRate::Hz15,
                        TempBatchRate::Hz60,
                    ][t as usize],
                )
        })
}

fn high_g_s() -> impl Strategy<Value = Option<HighGConfig>> {
    proptest::option::of((0u8..5, 0u8..5, any::<bool>()).prop_map(|(o, f, u)| {
        HighGConfig::new()
            .with_odr(
                [
                    HighGOdr::Hz480,
                    HighGOdr::Hz960,
                    HighGOdr::Hz1920,
                    HighGOdr::Hz3840,
                    HighGOdr::Hz7680,
                ][o as usize],
            )
            .with_full_scale(
                [
                    HighGFullScale::G32,
                    HighGFullScale::G64,
                    HighGFullScale::G128,
                    HighGFullScale::G256,
                    HighGFullScale::G320,
                ][f as usize],
            )
            .with_user_offset(u)
    }))
}

fn events_s() -> impl Strategy<Value = Option<HighGEvents>> {
    proptest::option::of(
        (any::<u8>(), any::<bool>(), 0u8..16, any::<bool>()).prop_map(|(t, change, dur, u)| {
            HighGEvents::new()
                .with_threshold(t)
                .with_wake_up_mode(if change {
                    HighGWakeUpMode::OnChange
                } else {
                    HighGWakeUpMode::EachTime
                })
                .with_shock_duration(dur)
                .with_user_offset(u)
        }),
    )
}

fn config_s() -> impl Strategy<Value = Config> {
    let analog = (
        0u8..4,
        odr_s(),
        0u8..5,
        odr_s(),
        high_g_s(),
        mode_s(),
        proptest::option::of(0u8..8),
        proptest::option::of(0u8..8),
    );
    let digital = (
        routing_s(),
        routing_s(),
        any::<[bool; 4]>(),
        fifo_s(),
        (0u8..2, 0u8..2, 0u8..3),
        events_s(),
        proptest::option::of(any::<bool>()),
    );
    (analog, digital).prop_map(|(a, d)| {
        let (afs, aodr, gfs, godr, hg, mode, lpf1, lpf2) = a;
        let (i1, i2, flags, fifo, pins, events, off) = d;
        let lpf1 = lpf1.map(|c| {
            [
                GyroLpf1Bandwidth::Bw0,
                GyroLpf1Bandwidth::Bw1,
                GyroLpf1Bandwidth::Bw2,
                GyroLpf1Bandwidth::Bw3,
                GyroLpf1Bandwidth::Bw4,
                GyroLpf1Bandwidth::Bw5,
                GyroLpf1Bandwidth::Bw6,
                GyroLpf1Bandwidth::Bw7,
            ][c as usize]
        });
        let lpf2 = lpf2.map(|c| {
            [
                AccelLpf2Bandwidth::OdrDiv4,
                AccelLpf2Bandwidth::OdrDiv10,
                AccelLpf2Bandwidth::OdrDiv20,
                AccelLpf2Bandwidth::OdrDiv45,
                AccelLpf2Bandwidth::OdrDiv100,
                AccelLpf2Bandwidth::OdrDiv200,
                AccelLpf2Bandwidth::OdrDiv400,
                AccelLpf2Bandwidth::OdrDiv800,
            ][c as usize]
        });
        Config::new()
            .with_accel_fs(
                [
                    AccelFullScale::G2,
                    AccelFullScale::G4,
                    AccelFullScale::G8,
                    AccelFullScale::G16,
                ][afs as usize],
            )
            .with_accel_odr(aodr)
            .with_gyro_fs(
                [
                    GyroFullScale::Dps250,
                    GyroFullScale::Dps500,
                    GyroFullScale::Dps1000,
                    GyroFullScale::Dps2000,
                    GyroFullScale::Dps4000,
                ][gfs as usize],
            )
            .with_gyro_odr(godr)
            .with_high_g(hg)
            .with_int1(i1)
            .with_int2(i2)
            .with_mode(mode)
            .with_gyro_lpf1(lpf1)
            .with_accel_lpf2(lpf2)
            .with_drdy_mask(flags[0])
            .with_accel_fast_settling(flags[1])
            .with_data_ready_mode(if flags[2] {
                DataReadyMode::Latched
            } else {
                DataReadyMode::Pulsed
            })
            .with_timestamp(flags[3])
            .with_fifo(fifo)
            .with_pins(
                PinConfig::new()
                    .with_polarity(if pins.0 == 0 {
                        IntPolarity::ActiveHigh
                    } else {
                        IntPolarity::ActiveLow
                    })
                    .with_output(if pins.1 == 0 {
                        IntOutput::PushPull
                    } else {
                        IntOutput::OpenDrain
                    })
                    .with_drive(
                        [PadDrive::Lowest, PadDrive::Intermediate, PadDrive::Highest]
                            [pins.2 as usize],
                    ),
            )
            .with_high_g_events(events)
            .with_accel_offset(off.map(|c| {
                if c {
                    OffsetWeight::Coarse
                } else {
                    OffsetWeight::Fine
                }
            }))
    })
}

/// Changes a random configuration into a valid one by applying the rules of `validate`.
fn repair(mut c: Config) -> Config {
    if c.mode.is_haodr() {
        if c.accel_odr == Odr::Hz7_5 {
            c.accel_odr = Odr::Hz15;
        }
        if c.gyro_odr == Odr::Hz7_5 {
            c.gyro_odr = Odr::Hz15;
        }
    }
    if c.fifo.gyro.code() > c.gyro_odr.code() {
        c.fifo.gyro = BatchRate::Off;
    }
    if c.fifo.accel.code() > c.accel_odr.code() {
        c.fifo.accel = BatchRate::Off;
    }
    if c.fifo.mode.needs_trigger() && c.high_g_events.is_none() {
        c.high_g_events = Some(HighGEvents::new());
    }
    if c.high_g_events.is_some() && c.high_g.is_none() {
        c.high_g = Some(HighGConfig::new());
    }
    if c.high_g_events.is_none() {
        for r in [&mut c.int1, &mut c.int2] {
            r.high_g_wake_up = false;
            r.high_g_shock = false;
        }
    }
    if c.high_g.is_none() {
        c.fifo.high_g = false;
    }
    if !matches!(c.fifo.timestamp, TimestampBatch::Off) || c.int2.timestamp_overflow {
        c.timestamp = true;
    }
    c.int1.timestamp_overflow = false;
    if matches!(c.fifo.mode, FifoMode::Bypass) {
        for r in [&mut c.int1, &mut c.int2] {
            r.fifo_threshold = false;
            r.fifo_overrun = false;
            r.fifo_full = false;
        }
    }
    let used = c.int1.fifo_threshold
        || c.int2.fifo_threshold
        || matches!(c.fifo.mode, FifoMode::ContinuousWtmToFull)
        || (c.fifo.stop_on_wtm && !matches!(c.fifo.mode, FifoMode::Bypass));
    if used && c.fifo.watermark == 0 {
        c.fifo.watermark = 1;
    }
    if let Some(e) = c.high_g_events.as_mut() {
        e.shock_duration = e.shock_duration.min(14);
    }
    c
}

pub(crate) fn valid_config_s() -> impl Strategy<Value = Config> {
    config_s().prop_map(repair)
}

fn only_bits(v: u8, allowed: u8) -> bool {
    v & !allowed == 0
}

fn valid_odr_code(c: u8) -> bool {
    c == 0 || (2..=12).contains(&c)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(4000))]

    #[test]
    fn prop_images_of_valid_configs_have_reserved_bits_zero(c in valid_config_s()) {
        // IF_CFG: only I2C_I3C_disable, PP_OD, H_LACTIVE (SIM stays 0: 4-wire), p58.
        prop_assert!(only_bits(c.if_cfg(), 0x19));
        prop_assert_eq!(c.if_cfg() & 1, 1);
        // PIN_CTRL: IBHR_POR_EN and the strength, never the reserved 10, p57.
        prop_assert!(only_bits(c.pin_ctrl(), 0x23));
        prop_assert_eq!(c.pin_ctrl() & 0x20, 0x20);
        prop_assert!(c.pin_ctrl() & 3 != 2);
        // CTRL1 and CTRL2: bit 7 zero, OP_MODE 000 or 001, valid ODR code, p65-66.
        for v in [c.ctrl1(), c.ctrl2()] {
            prop_assert_eq!(v & 0x80, 0);
            prop_assert!((v >> 4) & 7 <= 1);
            prop_assert!(valid_odr_code(v & 0x0F));
        }
        prop_assert_eq!(c.mode.is_haodr(), (c.ctrl1() >> 4) & 7 == 1);
        // CTRL4: DRDY_PULSED and DRDY_MASK only, p68.
        prop_assert!(only_bits(c.ctrl4(), 0x0A));
        // CTRL6: bit 7 zero, bit 3 one, FS_G 1 to 5, p69.
        prop_assert_eq!(c.ctrl6() & 0x80, 0);
        prop_assert_eq!(c.ctrl6() & 0x08, 0x08);
        prop_assert!((1..=5).contains(&(c.ctrl6() & 7)));
        // CTRL7: b7, b6, b0 only, p70.
        prop_assert!(only_bits(c.ctrl7(), 0xC1));
        // CTRL8: b7:5 and b1:0 only, p71.
        prop_assert!(only_bits(c.ctrl8(), 0xE3));
        // CTRL9: b5, b3, b1, b0 only, p72.
        prop_assert!(only_bits(c.ctrl9(), 0x2B));
        // CTRL1_XL_HG: off is 0; on has REGOUT_EN, ODR 3 to 7, FS 0 to 4, p91.
        let h = c.ctrl1_xl_hg();
        if c.high_g.is_some() {
            prop_assert_eq!(h & 0x80, 0x80);
            prop_assert!((3..=7).contains(&((h >> 3) & 7)));
            prop_assert!((h & 7) <= 4);
        } else {
            prop_assert_eq!(h, 0);
        }
        prop_assert!(only_bits(c.ctrl2_xl_hg(), 0x10));
        prop_assert!(only_bits(c.functions_enable(), 0x40));
        // HG_FUNCTIONS_ENABLE: shock duration at most 14, p94.
        prop_assert!(c.hg_functions_enable() & 0x0F <= 14);
        if c.high_g_events.is_none() {
            prop_assert_eq!(c.hg_functions_enable(), 0);
            prop_assert_eq!(c.hg_wake_up_ths(), 0);
        }
        prop_assert!(only_bits(c.inactivity_ths(), 0xC0));
        prop_assert!(only_bits(c.md2_cfg(), 0x01));
        prop_assert!(only_bits(c.haodr_cfg(), 0x03));
        prop_assert!(only_bits(c.int1_ctrl(), 0x3B));
        prop_assert!(only_bits(c.int2_ctrl(), 0x3B));
        // FIFO: BDR codes at most 12, bit 3 of FIFO_CTRL4 (G_EIS) zero, mode 101 never, p60-62.
        prop_assert!((c.fifo_ctrl3() >> 4) <= 12);
        prop_assert!((c.fifo_ctrl3() & 0x0F) <= 12);
        prop_assert_eq!(c.fifo_ctrl4() & 0x08, 0);
        prop_assert!(c.fifo_ctrl4() & 7 != 0b101);
        prop_assert!(only_bits(c.counter_bdr_reg1(), 0x08));
        prop_assert!(c.fifo_ctrl1() as u16 <= FIFO_DEPTH_WORDS);
    }

    #[test]
    fn prop_plan_of_valid_configs(c in valid_config_s()) {
        for apply in [Apply::AfterReset, Apply::InPlace] {
            let plan = c.plan(apply).unwrap();
            prop_assert!(plan.len() < MAX_STEPS);
            prop_assert_eq!(plan.steps().len(), plan.len());
            // The same plan twice.
            prop_assert_eq!(Ok(plan), c.plan(apply));
            let mut delays = 0;
            for step in &plan {
                match *step {
                    Write { reg, .. } => {
                        prop_assert!((0x02..=0x62).contains(&reg));
                        // Never a reserved or unowned register.
                        prop_assert!(owned(&c).iter().any(|(r, _, _)| *r == reg));
                    }
                    Modify { reg, mask, value } => {
                        prop_assert!(only_bits(value, mask));
                        prop_assert!(owned(&c).iter().any(|(r, m, _)| *r == reg && *m == mask));
                    }
                    DelayUs(us) => {
                        delays += 1;
                        prop_assert_eq!(us, 40_000);
                    }
                }
            }
            prop_assert_eq!(delays, usize::from(c.gyro_odr != Odr::Off));
            // The sensors are switched on last: ODRs after everything else.
            let steps = plan.steps();
            let n = steps.len() - delays;
            prop_assert_eq!(steps[n - 1], w(reg::CTRL2, c.ctrl2()));
            prop_assert_eq!(steps[n - 2], w(reg::CTRL1, c.ctrl1()));
            prop_assert_eq!(steps[n - 3], w(reg::CTRL1_XL_HG, c.ctrl1_xl_hg()));
        }
    }

    /// Skipping the writes of reset values after a reset leaves the same register file as writing
    /// everything, and every register the configuration owns ends up with its image, also when the
    /// device started in an arbitrary state.
    #[test]
    fn prop_plans_agree_with_the_images(
        c in valid_config_s(),
        junk in proptest::collection::vec(any::<u8>(), 128),
    ) {
        let after_reset = c.plan(Apply::AfterReset).unwrap();
        let in_place = c.plan(Apply::InPlace).unwrap();
        let mut a = reset_regs();
        execute(&mut a, &after_reset);
        let mut b = reset_regs();
        execute(&mut b, &in_place);
        prop_assert_eq!(a, b);
        let mut j = [0u8; 128];
        j.copy_from_slice(&junk);
        execute(&mut j, &in_place);
        for (r, mask, value) in owned(&c) {
            prop_assert_eq!(a[r as usize] & mask, value);
            prop_assert_eq!(j[r as usize] & mask, value);
            // Bits the configuration does not own are untouched by the in-place plan.
            prop_assert_eq!(j[r as usize] & !mask, junk[r as usize] & !mask);
        }
    }

    #[test]
    fn prop_repair_gives_valid_configs(c in config_s()) {
        prop_assert_eq!(repair(c).validate(), Ok(()));
    }

    #[test]
    fn prop_invalid_configs_are_not_planned(c in config_s()) {
        prop_assert_eq!(c.plan(Apply::AfterReset).is_ok(), c.validate().is_ok());
        prop_assert_eq!(c.plan(Apply::InPlace).is_ok(), c.validate().is_ok());
    }

    #[test]
    fn prop_default_with_one_field_changed_keeps_the_rest(odr in odr_s()) {
        let c = Config::new().with_gyro_odr(odr);
        prop_assert_eq!(c.ctrl1(), 0x09);
        prop_assert_eq!(c.ctrl2(), odr.code());
    }

    #[test]
    fn prop_real_odr_against_f64(ff in any::<i8>(), idx in 0usize..11) {
        let odrs = [
            Odr::Hz7_5, Odr::Hz15, Odr::Hz30, Odr::Hz60, Odr::Hz120, Odr::Hz240,
            Odr::Hz480, Odr::Hz960, Odr::Hz1920, Odr::Hz3840, Odr::Hz7680,
        ];
        let odr = odrs[idx];
        let reference = odr.nominal_mhz() as f64 * (1.0 + 0.0013 * ff as f64);
        let got = real_odr_mhz(odr, ff) as f64;
        prop_assert!((got - reference).abs() <= 0.5 + 1e-6, "{got} vs {reference}");
    }

    #[test]
    fn prop_timestamp_tick_against_f64(ff in any::<i8>()) {
        let reference = 1e9 / (46080.0 * (1.0 + 0.0013 * ff as f64));
        let got = timestamp_tick_ns(ff) as f64;
        prop_assert!((got - reference).abs() <= 0.5 + 1e-6, "{got} vs {reference}");
    }

    #[test]
    fn prop_offset_conversions_against_f64(raw in -127i8..=127, hg in 0usize..5, th in any::<u8>()) {
        let fine = low_g_offset_ug(raw, OffsetWeight::Fine) as f64;
        prop_assert!((fine - raw as f64 / 1024.0 * 1e6).abs() <= 0.5 + 1e-6);
        let coarse = low_g_offset_ug(raw, OffsetWeight::Coarse) as f64;
        prop_assert!((coarse - raw as f64 / 64.0 * 1e6).abs() <= 0.5 + 1e-6);
        let fs = [
            HighGFullScale::G32, HighGFullScale::G64, HighGFullScale::G128,
            HighGFullScale::G256, HighGFullScale::G320,
        ][hg];
        let step = if fs == HighGFullScale::G320 { 330.0 } else { 250.0 };
        prop_assert_eq!(high_g_offset_mg(raw, fs) as f64, raw as f64 * step);
        let tstep = if fs == HighGFullScale::G320 { 1250.0 } else { 1000.0 };
        prop_assert_eq!(hg_wake_up_threshold_mg(th, fs) as f64, th as f64 * tstep);
    }

    #[test]
    fn prop_offset_bytes_round_trip(l in any::<[i8; 3]>(), h in any::<[i8; 3]>()) {
        let o = Offsets::new().with_low_g(l).with_high_g(h);
        prop_assert_eq!(o.low_g_bytes().map(|b| b as i8), l);
        prop_assert_eq!(o.high_g_bytes().map(|b| b as i8), h);
        prop_assert_eq!(o.validate().is_ok(), !l.contains(&-128) && !h.contains(&-128));
    }
}

#[test]
fn owned_registers_match_the_plan_for_both_applies() {
    let c = BUILT;
    let list = c.owned_registers(&crate::calc::Offsets::new());
    assert_eq!(list.len(), crate::image::OWNED_COUNT);
    // The same registers and values as the table above, under the masks that leave out the
    // reserved bits.
    for (o, (r, mask, value)) in list.iter().zip(owned(&c)) {
        assert_eq!(o.reg, r);
        assert_eq!(o.mask & !mask, 0);
        assert_eq!(o.expected, value & o.mask);
    }
    for apply in [Apply::AfterReset, Apply::InPlace] {
        let plan = c.plan(apply).unwrap();
        let mut regs = reset_regs();
        execute(&mut regs, &plan);
        for o in &list {
            assert_eq!(regs[o.reg as usize] & o.mask, o.expected, "{:#04x}", o.reg);
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    #[test]
    fn prop_owned_registers_are_what_both_plans_leave(c in valid_config_s()) {
        let list = c.owned_registers(&crate::calc::Offsets::new());
        for apply in [Apply::AfterReset, Apply::InPlace] {
            let plan = c.plan(apply).unwrap();
            let mut regs = reset_regs();
            execute(&mut regs, &plan);
            for o in &list {
                prop_assert_eq!(regs[o.reg as usize] & o.mask, o.expected);
                prop_assert_eq!(o.expected & !o.mask, 0);
            }
            for step in &plan {
                if let Write { reg, .. } | Modify { reg, .. } = *step {
                    prop_assert!(list.iter().any(|o| o.reg == reg));
                }
            }
        }
    }
}

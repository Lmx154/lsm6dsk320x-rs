# LSM6DSK320X device facts (MARV V2 Rev 1)

Copied from the MARV V2 firmware repository's `docs/devices.md` and `docs/pinout.md`. Page numbers are PDF pages of the datasheet listed at the end. Items marked UNKNOWN or "unverified" have not been checked against the datasheet or on hardware. "Measured" means measured on this board.

## LSM6DSK320X (STMicroelectronics, IMU with high-g channel on SPI1)

Pins: see "Pins and bus" below.

- SPI: first byte bit7 = 1 for read (ST convention: RW bit first, then AD6:0), modes 0 and 3, address
  auto-increment on by default (CTRL3 bit2) (p22, p25, p67).
- WHO_AM_I 0x0F = 0x75 (p64).
- CTRL1 0x10: bits 6:4 OP_MODE_XL (000 high performance), bits 3:0 ODR_XL (0101 = 60 Hz, 0110 = 120 Hz,
  0111 = 240 Hz) (p65).
- CTRL2 0x11: same layout for the gyroscope (p66).
- CTRL6 0x15: bits 2:0 FS_G (001 = +-250, 010 = +-500, 011 = +-1000, 100 = +-2000, 101 = +-4000 dps). The
  reset value selects a reserved code, so FS_G must be written. Keep bit3 as the datasheet requires (p69).
- CTRL8 0x17: bits 1:0 FS_XL (00 = +-2 g, 01 = +-4 g, 10 = +-8 g, 11 = +-16 g) (p71).
- High-g channel: CTRL1_XL_HG 0x4E: bit7 XL_HG_REGOUT_EN, bits 5:3 ODR_XL_HG (011 = 480 Hz, 100 = 960 Hz),
  bits 2:0 FS_XL_HG (000 = +-32 g ... 100 = +-320 g) (p91). Field positions: verify on p91 before use.
  Table 21 (p33) allows the high-g channel with any low-g ODR in high-performance mode; lowest high-g ODR is 480 Hz.
- Interrupt routing: INT1_CTRL 0x0D bit0 = low-g accel data-ready, bit1 = gyro data-ready. INT2_CTRL 0x0E same
  bits. CTRL7 0x16 bit7 = high-g data-ready on INT1, bit6 = on INT2 (p63, p64, p70).
- CTRL4 0x13 bit1 DRDY_PULSED (verify bit position on p68): 1 = 65 us pulses, 0 = latched (default).
- IF_CFG 0x03: bit4 H_LACTIVE (0 = active high), bit3 PP_OD (0 = push-pull), bit2 SIM (0 = 4-wire), bit0
  I2C_I3C_disable; not reset by software reset (p58, Table 31).
- CTRL3 0x12: bit0 SW_RESET (self-clearing), bit7 BOOT (p67). Boot time is not in the datasheet; measured:
  SW_RESET self-clears in about 230 us.
- STATUS_REG 0x1E: bit0 XLDA, bit1 GDA, bit2 TDA, bit3 XLHGDA (p77).
- Outputs, little-endian 16-bit: temperature 0x20; gyro X 0x22, Y 0x24, Z 0x26; low-g accel X 0x28, Y 0x2A,
  Z 0x2C; high-g accel X 0x34, Y 0x36, Z 0x38 (p52-53, p78-84).
- Sensitivity (p12): low-g mg/LSB +-2 g 0.061, +-4 g 0.122, +-8 g 0.244, +-16 g 0.488. High-g mg/LSB +-32 g
  0.976, +-64 g 1.952, +-128 g 3.904, +-256 g 7.808, +-320 g 10.417. Gyro mdps/LSB +-250 8.75, +-500 17.5,
  +-1000 35, +-2000 70, +-4000 140.
- Temperature: degC = raw/256 + 25 (p16). Gyro turn-on 40 ms, typical, not guaranteed (Table 4, p15); measured on the MARV V2 board, 2026-11-11: clean gyro samples from about 65 ms after the start. The driver waits 70 ms.
- ODR accuracy in normal high-performance mode is not specified (the +-1 %/+-3 % figures on p13 are for
  high-accuracy-ODR mode). Measured: the output rate runs 1.8 % below nominal (the ODR setting is not recorded).
- High-g zero-g offset: +-250 mg typical (Table 3, p12).

## Pins and bus (from pinout.md)

All GPIO are 3.3 V logic and are not 5 V tolerant.

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 6 | IMU2_INT2 | GPIO input | LSM6DSK320X INT2 |
| 7 | IMU2_INT1 | GPIO input | LSM6DSK320X INT1 |
| 8 | IMU2_MISO | SPI1 RX (F1) | LSM6DSK320X |
| 9 | IMU2_CS | SPI1 CSn (F1) | LSM6DSK320X hardware CSn, 10k pull-up R51 |
| 10 | IMU2_SCK | SPI1 SCK (F1) | LSM6DSK320X |
| 11 | IMU2_MOSI | SPI1 TX (F1) | LSM6DSK320X |

| Peripheral | GPIOs | Device |
|---|---|---|
| SPI1 | SCK 10, MOSI 11, MISO 8, CSn 9 (hardware) | LSM6DSK320X only (U23), 4-wire |
| Sensor interrupts | LSM6DSK320X INT1 7, INT2 6 | separate nets and GPIOs |

| Item | Value |
|---|---|
| LSM6DSK320X CS (GPIO9) | 10k pull-up R51 to V3V3_ANA |
| Sensor SPI clock and data | no pull resistors on either bus |
| LSM6DSK320X pins SDx / SCx | SDx tied to V3V3_ANA, SCx to GND; 4-wire SPI |

Rails: V3V3_ANA (sensors, their pull-ups, X1, ADC_AVDD) is enabled from V3V3_SYS, so it comes up after V3V3_SYS. V3V3_SYS powers the microSD, its pull-ups, the PSRAM socket and the IO block power pins.

## Datasheet

| Device | URL |
|---|---|
| LSM6DSK320X | https://www.st.com/resource/en/datasheet/lsm6dsk320x.pdf (DS15060 Rev 1, July 2026) |

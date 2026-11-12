# lsm6dsk320x

Driver for the STMicroelectronics LSM6DSK320X, a six-axis IMU (gyroscope and low-g accelerometer) with a separate high-g accelerometer, over four-wire SPI. It is `no_std`, written against the `embedded-hal` 1.0 and `embedded-hal-async` 1.0 traits (optionally `defmt`), and owns no HAL, pin, timer or executor. The crate root is the async driver; `lsm6dsk320x::blocking` has the same methods for blocking buses. It was written for the MARV V2 Rev 1 flight controller (RP2354B). Version 0.1.0: the API may still change (`Cargo.toml` has `publish = false`).

## Quick start

The settings structs are `#[non_exhaustive]`: build them with `Config::new().with_...`, a struct literal does not compile outside the crate. The scaffolding hidden below (a dummy SPI device and delay) is only there so that this example is compiled by `cargo test --doc`.

```rust,no_run
# use core::convert::Infallible;
# use embedded_hal::spi::{ErrorType, Operation};
# use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
# struct Spi;
# struct Delay;
# impl ErrorType for Spi {
#     type Error = Infallible;
# }
# impl SpiDevice for Spi {
#     async fn transaction(&mut self, _ops: &mut [Operation<'_, u8>]) -> Result<(), Infallible> {
#         Ok(())
#     }
# }
# impl DelayNs for Delay {
#     async fn delay_ns(&mut self, _ns: u32) {}
# }
# async fn run(spi: Spi, mut delay: Delay) -> Result<(), lsm6dsk320x::Error<Infallible>> {
use lsm6dsk320x::{
    AccelFullScale, BatchRate, Config, DataReady, FifoConfig, FifoFrame, FifoMode, GyroFullScale,
    HighGConfig, HighGFullScale, IntRouting, Lsm6dsk320x, Odr, fifo_frames,
};

let config = Config::new()
    .with_accel_fs(AccelFullScale::G4)
    .with_accel_odr(Odr::Hz960)
    .with_gyro_fs(GyroFullScale::Dps500)
    .with_gyro_odr(Odr::Hz960)
    .with_high_g(Some(HighGConfig::new().with_full_scale(HighGFullScale::G32)))
    .with_int1(IntRouting::new().with_data_ready(DataReady::new().with_accel(true)))
    .with_int2(IntRouting::new().with_fifo_threshold(true))
    .with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::Continuous)
            .with_watermark(32)
            .with_gyro(BatchRate::Hz960)
            .with_accel(BatchRate::Hz960),
    );

let mut imu = Lsm6dsk320x::new(spi);
let info = imu.init(&mut delay, &config).await?; // reset, identity check, configuration
let _gyro_rate_mhz = info.gyro_odr_mhz;

// Wait for the INT1 edge here (the application owns the pin). Put any timeout around this wait.
let raw = imu.read().await?; // gyroscope and low-g accelerometer, one transaction
let accel = raw.accel.accel_ms2(config.accel_fs); // m/s^2
let gyro = raw.gyro.gyro_rads(config.gyro_fs); // rad/s
let all = imu.read_all().await?; // temperature, gyroscope, low-g and high-g
let high_g = all.high_g.high_g_ms2(HighGFullScale::G32);
let celsius = all.temperature_c();

// FIFO: after the INT2 watermark edge, read into your own buffer, then decode it in place.
let mut buf = [0u8; 7 * 32];
let read = imu.fifo_read(&mut buf).await?;
for frame in fifo_frames(&buf[..read.bytes]) {
    if let FifoFrame::Gyro(sample) = frame {
        let _rate = sample.axes.gyro_rads(config.gyro_fs);
    }
}
# let _ = (accel, gyro, high_g, celsius);
# Ok(())
# }
# fn main() {}
```

Blocking: `lsm6dsk320x::blocking::Lsm6dsk320x::new(spi)` takes an `embedded_hal::spi::SpiDevice` and its methods are the same without `.await`, with an `embedded_hal` `DelayNs`.

## What the application provides

- An SPI device: `embedded_hal_async::spi::SpiDevice` (async) or `embedded_hal::spi::SpiDevice` (blocking), with chip select handled by the device. The chip supports SPI modes 0 and 3 (datasheet 5.1, p22) and a clock up to 10 MHz (Tables 6 and 7, p17); the driver sets neither.
- A `DelayNs` for `init`, `configure` and `self_test`.
- The INT1 and INT2 pins and the waiting on them. The driver configures the chip's outputs (`Config::int1`, `Config::int2`, `Config::pins`) and never touches a pin. Data-ready is pulsed by default (`DataReadyMode::Pulsed`, 65 us per the datasheet, CTRL4 p68; on the MARV V2 board pulses of at least 51 us, 57 us on average, were seen); `DataReadyMode::Latched` holds it until the high byte of the output register is read.

## Rules for correct use

- Never cancel a bus transfer in flight: no timeout and no `select` around a driver call that is on the bus. Put the timeout around the wait for data-ready. On embassy-rp 0.10 a cancelled SPI transfer leaves stale bytes behind and the next read is wrong. The `lsm6dsk320x_cancel` program measures this.
- After any doubt call `recover()`. It re-frames the bus, checks WHO_AM_I and then runs `verify()`. `verify()` is the health check: it reads the registers the applied configuration owns (32 transactions) and compares them with it. It writes nothing and changes no state; a mismatch is `Error::ConfigMismatch`.
- `read`, `read_high_g`, `read_all`, `status`, `interrupt_status`, the FIFO methods, `set_offsets`, `verify` and `self_test` return `Error::NotConfigured` until `init` or `configure` has completed. A failed or dropped `init`, `configure` or `self_test` leaves the driver not configured.
- Start-up. `init` and `configure` wait 70 ms after starting the gyroscope (`plan::GYRO_TURN_ON_US`): the datasheet's 40 ms is typical and not guaranteed (Table 4, p15), and on the MARV V2 board gyroscope samples were clean from about 65 ms. No wait is added for the accelerometers' turn-on or filter settling, which the datasheet does not give. `Config::drdy_mask` (on by default) hides the data-ready signal of a sensor until its filters settle, but only for the settling the device knows about: with the mask on, the first 4 gyroscope samples read after `configure` returned (with the earlier 40 ms wait) were off by up to 6.6 dps. With a narrow low-g `accel_lpf2` the output settles slowly: ODR/100 at 240 Hz took 170 ms to come within 10 % of 1 g; `Config::accel_fast_settling` brought the first sample into band at 18 ms. Turn it on with a narrow LPF2, or discard samples.
- A data-ready flag stays set until the data is read, also across `configure` and also when the sensor is then off (150 of 150 polls on the board). Read the data once after `configure` and discard it before relying on `status()` or `interrupt_status()` for a sensor you switched off or reconfigured.

## What the driver does

- Settings (`Config`, checked by `Config::validate` before any bus access; `ConfigError` names the rule broken): low-g `Odr` and `AccelFullScale` (+-2 to +-16 g), gyroscope `Odr` and `GyroFullScale` (+-250 to +-4000 dps), 7.5 Hz to 7.68 kHz; high-g `HighGConfig` (`HighGOdr` 480 Hz to 7.68 kHz, `HighGFullScale` +-32 to +-320 g). `OperatingMode::HighPerformance` or `HighAccuracyOdr(HaodrSet)` with the four ODR sets (960, 1000, 800, 833 base). Gyroscope LPF1 (`GyroLpf1Bandwidth`), low-g LPF2 (`AccelLpf2Bandwidth`, ODR/4 to ODR/800), `drdy_mask`, `accel_fast_settling`, `DataReadyMode`, `PinConfig` (polarity, push-pull or open-drain, pad drive).
- Setup: `init` (software reset, then the configuration), `configure` (in place, on a chip that is not reset), `config`, `release`. The register write plan is data: `Config::plan`, `plan::Plan`.
- Reads: `read`, `read_high_g`, `read_all` (one transaction each), `status`; conversions `RawAxes::accel_ms2`, `gyro_rads`, `high_g_ms2`, `RawAll::temperature_c`.
- FIFO (`FifoConfig`): modes, watermark, batch rates for gyroscope, low-g (`BatchRate`), high-g, temperature (`TempBatchRate`) and timestamp (`TimestampBatch`), `stop_on_wtm`, `odr_change`. Methods `fifo_read`, `fifo_read_exact`, `fifo_status`, `fifo_flush`. Decoding is the pure `fifo_frames` (7-byte words with tags: `FifoFrame::Gyro`, `Accel`, `HighG`, `Temperature`, `Timestamp`, `ConfigChange`, `Unknown`, `Trailing`; it never skips a byte). `fifo::timestamp_ns`, `fifo::timestamp_delta_ticks`, `fifo::fifo_words_per_second`. `FIFO_DEPTH_WORDS` is 256, the depth measured on the board (the datasheet says 1.5 KB).
- Interrupt routing per pin (`IntRouting`): data-ready of each channel, FIFO threshold, overrun and full, high-g wake-up and shock (`HighGEvents`), timestamp overflow (INT2 only). `interrupt_status` reports the sources in 1 or 3 transactions.
- `stats` (`Stats`): FIFO overruns seen, FIFO words read, bus errors; never cleared by the driver.
- `info` (`Info`): WHO_AM_I, the clock trim `freq_fine`, and the output rates of the three sensors in millihertz plus the expected FIFO words per second. In normal mode these are the real rates with the trim; in the high-accuracy ODR mode they are the datasheet's table rate without trim (Table 22, p34).
- User offsets: `set_offsets` with `calc::Offsets` (low-g weight `OffsetWeight`, high-g), `offsets`. `init` clears them; `configure` writes the stored values back. They change the output only when enabled in the configuration: `Config::accel_offset` (`Some(OffsetWeight)`) for the low-g accelerometer and `HighGConfig::user_offset` for the high-g one.
- Self-test of the low-g accelerometer, the gyroscope and the high-g accelerometer: `self_test` with `SelfTestConfig`, returning a `SelfTestReport` per channel and axis against the datasheet windows (`calc::SELF_TEST_*`). The call takes about one second with the defaults, the applied configuration and offsets are re-applied when it completes, and while it runs the outputs and the interrupt pins are not meaningful. ST publishes no sequence for this part; the module documentation says which parts follow the datasheet and which follow a sibling part's example.
- `verify` and `recover` (see the rules above).
- Raw register access: `read_register`, `read_registers`, `write_register`, and the register map in `reg`. These neither check nor change the driver state.
- The blocking front-end `blocking::Lsm6dsk320x`, one implementation shared with the async driver (the SPI and delay futures never pend).

## Not covered

Left out on purpose: the gyroscope EIS channel, the ODR-triggered mode, SFLP sensor fusion, the low-power and normal operating modes, the embedded functions (FSM, machine-learning core, tap, free-fall, pedometer and the others), the sensor hub, OIS, I2C and I3C (`init` disables them in IF_CFG, so the part is SPI only), FIFO compression and the accelerometer high-pass filter. No `Config` field or method exists for any of them. `reg` still lists some of their register bits and FIFO tags.

There is no method to reload the factory trimming (BOOT).

The only FIFO trigger the crate can configure is the high-g wake-up (`FifoMode` documentation), and `Config::validate` rejects a triggered mode without it.

Present in the driver but not exercised by any program in `hil/src/bin` (checked with a text search of that folder; the host tests cover them against a simulated bus only):

- The high-g wake-up and shock generator (`Config::high_g_events`, `HighGEvents`, `HighGWakeUpMode`) and routing them to a pin (`IntRouting::high_g_wake_up`, `high_g_shock`). `interrupt_status` is called only with the generator off.
- FIFO modes other than `Continuous` (the triggered modes, `Fifo`, `ContinuousWtmToFull`), `FifoConfig::stop_on_wtm`, and routing the FIFO full or overrun flag (`IntRouting::fifo_full`, `fifo_overrun`) or the timestamp overflow (`IntRouting::timestamp_overflow`) to a pin.
- SPI mode 3 and any clock other than 8 MHz: every program in `hil/src/bin` sets `SPI_HZ` to 8 MHz and none selects a mode. The mode 3 and higher-clock figures below were not produced by a program in that folder.
- Full scales that no program names (grep of `hil/src/bin`): gyroscope +-1000, +-2000 (the `Config::new` default) and +-4000 dps, low-g +-8 g (the default) and +-16 g. The programs name only G2, G4, Dps250 and Dps500 for those; `lsm6dsk320x_dual` runs all five high-g full scales and all five high-g rates.

## Measured on the MARV V2 Rev 1 board

RP2354B at 150 MHz, embassy-rp 0.10, 2026-11-11. These are observations on one unit, not guarantees.

- One `read()` (13 bytes on the wire), median: 25.1 us at 7.5 MHz and 22.1 us at 9.375 MHz in SPI mode 0; 23.0 us and 21.0 us in SPI mode 3. Ten repeats of the sweep: 0 wrong reads of 10 000 at every 8.33 and 9.375 MHz point and at every mode 3 point.
- On this MCU (RP2350, PL022 SPI block) a burst costs 9.5 bit times per byte in mode 0 and 8.0 in mode 3 (1267 against 1067 ns per byte at 7.5 MHz): mode 3 saves about 16 % of bus time.
- Transfers are carried by DMA: a 1792-byte burst took 0.4 % longer while another task kept the CPU busy.
- `fifo_frames` decode: 0.83 us per FIFO word.
- Driver CPU time on top of the bare HAL for the same transaction: 0.84 us for `read()`, 0.87 us for `read_all()`, 1.2 us for a 28-word FIFO read (medians, identical over ten runs of one build). In other builds these moved by several microseconds with code placement in flash.
- Reading on every data-ready edge at 7.68 kHz for 2 s, from an interrupt executor and from the second core, while a thread-mode task spun: no edge missed, no read failed.
- High-accuracy ODR sets (960 / 1000 / 800 / 833): 957.66, 999.998, 800.664 and 833.998 Hz. The normal mode at the 960 Hz setting ran at 942.66 Hz on this unit (the frequency trim explains it).
- `lsm6dsk320x_dual` (low-g, gyroscope and high-g on together, 2026-11-11): the high-g channel at full scales 32 / 64 / 128 / 256 / 320 g gives the same vector within 0.15 %. Against the low-g vector at rest the high-g one differs by about 0.05 / -0.15 / 0.30 g on X / Y / Z, the same at every full scale (the datasheet gives only a typical high-g zero-g offset of +-250 mg, Table 3, p12): the application has to calibrate it out with the high-g offset registers (`set_offsets`). High-g rates 480 / 960 / 1920 / 3840 / 7680 Hz were measured against the low-g data-ready rate. In the high-accuracy ODR mode the high-g rate and the FIFO words per second follow the selected ODR set like the low-g rate: in all four sets and at all five high-g settings, `Info::high_g_odr_mhz` and `fifo_words_per_second` are within 0.5 % and 1 % of the measurement (low-g and gyroscope batched at 960 Hz; other batch rates in that mode are by analogy, not measured).
- Also on this board (dates in the rustdoc and `docs/device.md`): `SW_RESET` clears in about 230 us; the gyroscope is clean from about 65 ms after start; the FIFO held 256 words (2026-11-10).

## Tests

Host tests (no hardware, a simulated bus): `cargo test --locked`. Observed: 169 unit tests pass, plus 1 README doctest (compiled, not run).

On the board, through a debug probe on the SWD header (the script builds, flashes, runs and takes a lock so runs never overlap): `tools/run.sh <name>`, for example `tools/run.sh lsm6dsk320x_smoke`. The exit status is 0 when every check passed; logs are kept under `hil/logs/`. The programs use SPI1 (SCK GPIO10, MOSI GPIO11, MISO GPIO8, hardware CSn GPIO9 with pull-up R51), INT1 on GPIO7 and INT2 on GPIO6; the smoke program runs at 8 MHz.

| Program | What it proves |
|---|---|
| `alive` | tool chain, probe and board; prints the system clock |
| `lsm6dsk320x_smoke` | identity, data-ready rate with missed-sample count, time per read, plausible values at rest, high-g reads and routing, the blocking front-end |
| `lsm6dsk320x_fifo` | burst over many FIFO words, the temperature, timestamp and configuration-change word layouts, FIFO depth, overrun and flush, the watermark interrupt, high-g words, `verify()` |
| `lsm6dsk320x_cal` | the user-offset registers: which offsets `USR_OFF_W` scales, weights, signs, `set_offsets`, `configure`, `init`, `recover` |
| `lsm6dsk320x_selftest` | `self_test` with the defaults (five runs), negative stimulus, each channel alone, stimulus settling, margins to the windows, shortened settings, nothing left behind |
| `lsm6dsk320x_cancel` | cancelled `read()` calls followed by `recover()`: wrong reads after a cancel, with and without `recover()` |
| `lsm6dsk320x_modes` | gyroscope LPF1, low-g LPF2, `drdy_mask`, fast settling, the four high-accuracy ODR sets, latched data-ready, pin polarity, output type and drive, status flags |
| `lsm6dsk320x_dual` | the low-g, gyroscope and high-g channels on together: every high-g full scale at rest, every high-g rate, the high-accuracy ODR sets with the high-g channel (rates and FIFO), `read_all` timing |

## Features, layout, licence

The only feature is `defmt` (off by default): `defmt::Format` for the public data and error types. Without it the crate is plain `no_std` code. Edition 2024.

```text
Cargo.toml       the driver crate (its own workspace root)
src/             the driver: lib.rs (driver), config.rs, plan.rs, fifo.rs, data.rs, selftest.rs,
                 calc.rs, sample.rs, reg.rs, blocking.rs
hil/             the on-board programs: one package, its own workspace; the only place using embassy-rp
tools/run.sh     builds, flashes and runs one on-board program
docs/device.md   register, timing and measured facts, pins, datasheet reference
LICENSE          MIT
```

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md` (DS15060 Rev 1).

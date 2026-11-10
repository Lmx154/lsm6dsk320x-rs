# lsm6dsk320x

Driver for the STMicroelectronics LSM6DSK320X, a six-axis IMU (gyroscope and low-g accelerometer) with a high-g accelerometer, over four-wire SPI, for the MARV V2 Rev 1 flight controller (RP2354B). It is built on the `embedded-hal` 1.0 traits only: no executor, no HAL, no pin, no timer. The crate root is the async driver. The `blocking` module is the same driver for blocking buses. It wraps the bus in an adapter that never pends and runs the same code, so nothing is written twice. All device knowledge (register map, configuration codes, decoding, units) is pure code tested on the host.

## Usage

Async, for example in an embassy task:

```rust
use lsm6dsk320x::{Config, Lsm6dsk320x};

let mut imu = Lsm6dsk320x::new(spi_device);
let config = Config::default();
imu.init(&mut delay, &config).await?;
loop {
    int1.wait_for_rising_edge().await;
    let raw = imu.read().await?;
    let accel = raw.accel.accel_ms2(config.accel_fs);
    let gyro = raw.gyro.gyro_rads(config.gyro_fs);
}
```

Blocking, on any embedded-hal bus:

```rust
use lsm6dsk320x::{blocking, Config};

let mut imu = blocking::Lsm6dsk320x::new(spi_device);
let config = Config::default();
imu.init(&mut delay, &config)?;
let raw = imu.read()?;
```

Units: conversions give acceleration in m/s², angular rate in rad/s and temperature in °C, as `f32`, from the raw counts and the full scale that was configured.

Not covered: FIFO, embedded functions (FSM, MLC, step counter, tap, free-fall), self-test, I2C/I3C, sensor hub, low-power modes.

## What the application provides

- An SPI device: `embedded_hal_async::spi::SpiDevice` for the async driver, the blocking `embedded-hal` SPI device trait for `blocking`.
- A `DelayNs` for `init`.
- The interrupt pins and the waiting on them. The driver owns no interrupt pin, timer or task. `init` configures the chip's INT1 and INT2 outputs; the application waits on the pins. Every sample signalled by data-ready after `init` returns is valid.

Dependencies: `embedded-hal` 1.0, `embedded-hal-async` 1.0 and, optionally, `defmt`.

Same shape as the other sensor drivers of this board: `new(bus)`, `init(&mut delay, &config)`, `read()` (one bus transaction, integers only), unit conversion as separate pure functions, public register access, `release()`.

## Cargo features

All features are off by default. Without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data and error types |

## Measured on the MARV V2 Rev 1 board

- 7.68 kHz setting (7540 Hz actual) with no missed sample.
- 23 us per read at 8 MHz SPI.
- The output rate runs 1.8 % below nominal.
- SW_RESET self-clears in about 230 us.

## Tests

On the host:

    cargo test

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh alive
    tools/run.sh lsm6dsk320x_smoke

Used here: Rust 1.99.1 with the `thumbv8m.main-none-eabihf` target, probe-rs 0.31.0, and a debug probe on the SWD header J10.

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `lsm6dsk320x_smoke` | identity, data-ready rate with missed-sample count, time per read, plausible values at rest, the blocking front-end |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`.

Pins the test uses: SPI1 SCK on GPIO10, MOSI on GPIO11, MISO on GPIO8 and hardware CSn on GPIO9 (10k pull-up R51), INT1 on GPIO7 and INT2 on GPIO6. The test drives the driver through embassy-rp's DMA SPI and through its blocking SPI, at 8 MHz.

## Status

Version 0.1.0, as tested on the Rev 1 board.

- Host: `cargo test` passes. Clippy, rustdoc and rustfmt are clean for the driver crate, and every public item is documented.
- Board: `alive` and `lsm6dsk320x_smoke` exited 0 when last run.

Not done for this device:

- FIFO and batch reads, self-test, motion and embedded functions.
- Calibration of the installed sensor: the high-g channel reads about 0.3 g high at rest on this board.

## Layout

```
Cargo.toml       the driver crate (its own workspace root)
src/             the driver
hil/             the on-board test: one package, its own workspace; the only place that uses embassy-rp
  src/lib.rs     reports the verdict to the host
  src/bin/       alive.rs and lsm6dsk320x_smoke.rs
  build.rs       links memory.x
tools/run.sh     builds, flashes and runs one on-board test
docs/device.md   register, timing and measured facts, pins, datasheet
LICENSE          MIT
```

## License

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md`.

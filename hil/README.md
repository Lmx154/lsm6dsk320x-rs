# hil

Part of [lsm6dsk320x](../README.md)

On-board test programs for the MARV V2 Rev 1 board. This folder is its own workspace and the only user of embassy-rp. `tools/run.sh <name> [timeout_s]` builds one program, flashes it through a debug probe on the SWD header under a lock that serialises probe use, and runs it. Exit status 0 means every check passed; logs go to `hil/logs/`. The programs use SPI1 (SCK GPIO10, MOSI GPIO11, MISO GPIO8, CSn GPIO9), INT1 on GPIO7 and INT2 on GPIO6. `lsm6dsk320x_soak`, `_dual` and `_drdy_lead` run SPI mode 3 at 9.375 MHz; the others run mode 0 at 7.5 MHz.

| Program | What it checks |
|---|---|
| `alive` | tool chain, probe and board; prints the system clock |
| `lsm6dsk320x_smoke` | identity, data-ready rate with missed-sample count, time per read, plausible values at rest, high-g reads and routing, the blocking front-end |
| `lsm6dsk320x_fifo` | burst over many FIFO words, temperature, timestamp and configuration-change word layouts, FIFO depth, overrun and flush, watermark interrupt, high-g words, `verify()` |
| `lsm6dsk320x_cal` | user-offset registers: which offsets `USR_OFF_W` scales, weights, signs, `set_offsets`, `configure`, `init`, `recover` |
| `lsm6dsk320x_selftest` | `self_test` with the defaults, negative stimulus, each channel alone, settling, margins to the windows, shortened settings, nothing left behind |
| `lsm6dsk320x_cancel` | cancelled `read()` calls followed by `recover()`, with and without `recover()` |
| `lsm6dsk320x_modes` | gyro LPF1, low-g LPF2, `drdy_mask`, fast settling, the four high-accuracy ODR sets, latched data-ready, pin polarity, output type and drive, status flags |
| `lsm6dsk320x_dual` | low-g, gyroscope and high-g together: every high-g full scale at rest, every high-g rate, high-accuracy ODR sets with the high-g channel, `read_all` timing |
| `lsm6dsk320x_shock` | high-g wake-up and shock events with a person striking the board: run `tools/shock.sh [seconds]`, which shows when to keep hands off, strike and rest; the status LED cues the phases; at rest it fails with "no strike detected" |
| `lsm6dsk320x_drdy_lead` | a measurement, not a pass/fail test: when each output register changes relative to the data-ready edge (see [driver](../docs/driver.md)) |
| `lsm6dsk320x_soak` | long data-integrity run with all three channels on: single reads at 7.68 kHz, double reads at 3.84 kHz, the FIFO with high-g words; length set at build time, e.g. `SOAK_SECONDS=1800 tools/run.sh lsm6dsk320x_soak 2100`; never part of start-up |

The soak pass criteria are zero missed or lost edges, zero bus errors, zero mismatches between paired reads, and in the FIFO phase a constant number of words of each channel between consecutive timestamp words with no malformed words or overruns; `verify()` runs every minute.

## Related

[driver](../docs/driver.md) · [device](../docs/device.md)

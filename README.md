# Prox Sensor Test

Test-rig firmware for the **ST Nucleo-F401RE** (STM32F401RE, Cortex-M4F @ 84 MHz, 512 KB flash, 96 KB SRAM), built with PlatformIO.

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU board | Nucleo-F401RE | USB Mini-B to the on-board ST-LINK/V2-1 (flash, debug, serial) |
| Stepper drive | L298N module, both bridges, 4-wire bipolar stepper | PWM @ 20 kHz on IN1 = D8 (PA9, TIM1_CH2), IN2 = D7 (PA8, TIM1_CH1), IN3 = D15 (PB8, TIM4_CH3), IN4 = D14 (PB9, TIM4_CH4); on/off ENA = D9 (PC7), ENB = D6 (PB10) |
| Analog input | STM32 ADC1 | A0 (PA0), 12-bit, 0 to 3.3 V |
| Precision ADC | TI ADS1220 | SPI1: SCLK = D13 (PA5), DOUT→MISO = D12 (PA6), DIN←MOSI = D11 (PA7), CS = D10 (PB6), DRDY = D2 (PA10) |

Pin assignments live in `include/main.h`.

### Wiring notes

- **Stepper:** find the coil pairs with a meter (a few ohms between the two wires of one coil, open
  between coils). Coil A goes to OUT1/OUT2 and coil B to OUT3/OUT4. If the motor buzzes instead of
  turning, swap the two wires of one coil.
- **L298N:** remove the ENA and ENB jumpers so D9/D6 can switch the bridges on and off. The
  current-limiting PWM is on the IN pins. Keep the
  5V-EN jumper on for supplies up to 12 V. Tie the L298N GND to a Nucleo GND. Don't feed the Nucleo
  from the module's 5 V regulator while USB is plugged in.
- **A0** isn't 5 V tolerant in analog mode. Put a divider in front of anything above 3.3 V and set
  `ANALOG_SNS_GAIN` in `include/util/AdcToVolt.h` to match.
- **ADS1220:** AVDD on the Nucleo 5V (it's the reference), DVDD on 3V3 so its SPI logic levels
  match the Nucleo. DRDY has the MCU's internal pull-up enabled.
- **LD2:** the green user LED is on PA5, the SPI clock pin, so it flickers whenever the ADS1220 is
  being read.

**Console:** `Serial` (USART2) over the ST-LINK virtual COM port at **921600 baud**.

---

## Firmware overview

```
src/
├── main.cpp       Entry point: setup(), loop(), sample ISR
├── app/           Application logic
└── drivers/       Peripheral / device drivers
    ├── ADS1220.cpp
    ├── L298NStepper.cpp
    └── PeriodicTimer.cpp

include/
├── main.h         Shared pin mapping and cross-module declarations
├── app/           Headers for src/app/ (header-only code lives here too)
│   └── SampleLog.h    Lock-free SPSC ring buffer, ISR → loop()
├── drivers/       Headers for src/drivers/
└── util/          Shared helpers (Console, Guard, LineBuffer, AdcToVolt)
```

`include/app`, `include/drivers` and `include/util` are on the include path in `platformio.ini`, so
headers resolve both qualified (`"drivers/foo.h"`) and unqualified (`"foo.h"`).

### Data flow

```
TIM5 @ 2 kHz ───► sampleIsr()
                   ├─ ADS1220::readIfReady()      (only if DRDY is low)
                   ├─ stepperProfile()            (moveTo / setVelocity)
                   ├─ L298NStepper::update()      (ramp, at most one step per tick)
                   └─ every 4th tick (500 Hz):
                        analogRead(A0), SampleLog::push()
                                     │
loop() @ 100 Hz ◄────────────────────┘ pop → LineBuffer → CSV over Serial
```

The ISR stores raw counts and `loop()` converts them to volts, which keeps the ISR short. Logging
and the stepper are held off for `LOG_START_DELAY` (2 s) after boot so the monitor has time to
attach. Then the firmware prints the CSV header once:

```
t_us,pos,adc_raw,ads_code,ads_new      # LOG_RAW = true (default)
t_us,pos,adc_v,ads_v,ads_new           # LOG_RAW = false
```

| Column | Meaning |
|--------|---------|
| `t_us` | `micros()` at the start of the tick (wraps after ~71 min) |
| `pos` | Stepper position, in steps of `STEP_MODE` |
| `adc_raw` | STM32 ADC raw reading of A0, 12-bit (0–4095) |
| `adc_v` | Sensor voltage from A0 (after `ANALOG_SNS_GAIN`, the 3.231:1 divider) |
| `ads_code` | ADS1220 raw result, signed 24-bit (AIN0 vs AVSS: 0 to 8388607 over 0 to AVDD) |
| `ads_v` | Sensor voltage from the ADS1220 (after the 3.231:1 divider) |
| `ads_new` | 1 if a fresh conversion arrived since the previous row, 0 if the previous one was held over |

If the serial link falls behind and the ring fills, the firmware prints `# dropped N` before the
next rows, so a gap in `t_us` is always explained.

### Tuning knobs (`src/main.cpp`)

| Constant | Default | Notes |
|----------|---------|-------|
| `ISR_PERIOD` | 500 µs | Tick rate; also the step-timing grid and the top step rate (2000 steps/s) |
| `LOG_DECIMATION` | 4 | Log every Nth tick (500 Hz). Keep the ADS1220 data rate above the log rate |
| `MOTOR_SUPPLY_V`, `COIL_RESISTANCE_OHM`, `BRIDGE_DROP_V` | 12 V, 2 Ω, 1.8 V | Inputs to the default duty and `STEPPER_MAX_DUTY`; use worst-case (lowest R, lowest drop) values |
| `STEPPER_RUN_DUTY` | 0.4 | Run duty, set from bench testing. Must not exceed `STEPPER_MAX_DUTY` |
| `L298N_MAX_CURRENT_A` | 2.5 A | Ceiling behind `STEPPER_MAX_DUTY` (the L298's repetitive peak rating; 2 A continuous) |
| `HOLD_CURRENT_FRACTION` | 0.5 | Fraction of the run current used while stationary |
| `STEP_MODE` | `FULL` | `FULL` (200/rev, more torque) or `HALF` (400/rev, smoother) |
| `STEPPER_MAX_SPEED`, `STEPPER_ACCEL` | 50 steps/s, 800 steps/s² | Raise until the motor misses steps, then back off |
| `SWEEP_REVS`, `SWEEP_DWELL` | 3, 500 ms | Square wave between −`SWEEP_REVS` and +`SWEEP_REVS` revolutions about the start position, pausing at each end; 0 holds position. Replace `stepperProfile()` for other motion |
| `STEPPER_PWM_HZ` | 20 kHz | Keep at or below ~25 kHz for the L298N |
| `ADS_CONFIG` | AIN0 vs AVSS, gain 1, PGA bypassed, 1000 SPS, AVDD (5 V) ref | See `ADS1220::Config` |
| `LOG_RAW` | `true` | Raw counts from both ADCs, or sensor volts |
| `LOG_POSITIVE_ONLY` | `true` | Log only while moving in the positive direction (how the device will be used). `false` logs every tick, which you need for stationary-noise figures and forward/reverse comparisons |

### Stepper current limit

The L298N can't sense current, so the coil current is set open-loop by the PWM duty. The PWM goes on
one IN pin per coil while the other is held low, with ENA/ENB held high. During the off-time both
IN pins are low, so the coil current keeps flowing through the bottom of the bridge (slow decay)
and follows the duty:

```
I_coil <= duty × (MOTOR_SUPPLY_V − BRIDGE_DROP_V) / COIL_RESISTANCE_OHM
```

PWM on ENA/ENB instead doesn't work at low duty. The bridge switches fully off, the coil discharges
into the supply at about −12 V, and the current collapses every period. The first version of this
firmware did that and measured only 8–24 mA.

`include/app/RigConfig.h` computes the default `STEPPER_RUN_DUTY` from that bound using worst-case
values, so the real current comes out lower, probably about half. `STEPPER_MAX_DUTY` is the duty
where the bound reaches `L298N_MAX_CURRENT_A` (2.5 A, about 0.49 duty); the build fails above it, and the bring-up keys
stop there.

**Tune it on the bench** with the bring-up firmware:

1. Put a DC ammeter in series with one coil.
2. Press `h` so the stationary motor holds at the *run* duty. The ammeter only reads correctly
   while the motor is still: while stepping, the coil current reverses and averages toward zero.
3. Press `[` / `]` to step the duty until the meter reads the current you want.
4. Put that duty in `STEPPER_RUN_DUTY`, e.g. `STEPPER_RUN_DUTY{0.33f}`.

At 500 Hz each row is ~45 bytes (~23 kB/s), well within 921600 baud (~92 kB/s). If you raise the
sample rate, check that budget.

### Drivers

| Driver | Purpose |
|--------|---------|
| `L298NStepper` | Bipolar stepper on both L298N bridges. Full or half step, slow-decay PWM current limit on the IN pins with a lower hold current, and a trapezoidal profile (`moveTo()` position mode, `setVelocity()` velocity mode). Everything except `begin()` is ISR-safe |
| `ADS1220` | Resets the part, writes and **reads back** the config (`begin()` fails if the chip is missing or miswired), then runs it in continuous mode. `readIfReady()` polls DRDY and clocks out 24 bits |
| `PeriodicTimer` | Fixed-rate interrupt on a spare timer (TIM5) |

---

## Development environment

- **Build system:** [PlatformIO](https://platformio.org/) with `platform = ststm32`, `framework = arduino` (STM32duino)
- **Toolchain:** `arm-none-eabi-gcc` (managed by PlatformIO)
- **Flash / debug:** on-board ST-LINK over SWD (no external programmer needed)

### Prerequisites

- [VS Code](https://code.visualstudio.com/) + [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
  - or - PlatformIO Core CLI: `pip install platformio`
- A USB Mini-B cable to the Nucleo's ST-LINK port
- On Windows, the ST-LINK USB driver (STSW-LINK009) if the board doesn't enumerate

### Build & flash

```bash
# Build
pio run

# Flash (board connected over USB)
pio run --target upload

# Serial monitor (921600 baud). Also saves the session to logs/
pio device monitor
```

The first build downloads the STM32 platform and the STM32duino core, which takes a few minutes.
Later builds are incremental.

### Bring-up test

`src/bringup.cpp` is a separate firmware for checking the hardware before a real run. It shares the
stepper current limit and ADS1220 settings with `main.cpp` through `include/app/RigConfig.h`.

```bash
pio run -e nucleo_f401re_bringup --target upload
pio device monitor        # then press RESET to see the boot report
```

On boot it runs the ADS1220 checks:

| Check | What a pass proves |
|-------|--------------------|
| 1. Register write/readback | SPI wiring (SCLK, DIN, DOUT, CS) and power |
| 2. Conversion rate | DRDY wiring, and the chip is converting at the configured rate |
| 3. Shorted input | Real conversion data comes back (≈ 0 V) |
| 4. Supply monitor | Data is scaled correctly (reports AVDD, about 5 V) |
| 5. Configured input | Informational reading of whatever is on the inputs |

Then it takes single-key commands: `f`/`b` one revolution forward/back, `+`/`-` one step, `z` zero,
`e` energise, `x` release, `[`/`]` run duty down/up, `h` hold at run current (for measuring it),
`a` rerun the ADS1220 test, `?` help. The coils start off; any motion
key energises them.

### Load-cell build

`src/loadcell.cpp` is a separate firmware for a 4-wire strain-gauge load cell (a bridge, about
1 kΩ across S+/S− unloaded). It uses only the ADS1220: no stepper and no STM32 ADC. The L298N
enables are held low in case the driver is still connected.

| Load cell | Connect to |
|-----------|-----------|
| E+ (excitation +) | Nucleo 5V, the same pin as the ADS1220 AVDD (DVDD stays on 3V3) |
| E− (excitation −) | GND |
| S+ (signal +) | ADS1220 AIN1 |
| S− (signal −) | ADS1220 AIN2 |

- **Settings:** AIN1−AIN2, gain 128, 20 SPS with 50/60 Hz rejection, AVDD reference. The range is
  ±39 mV at 5 V excitation, or ±7.8 mV/V.
- **Ratiometric:** the bridge is excited from AVDD and AVDD is also the reference, so supply drift
  cancels. For the best accuracy, also run REFP0/REFN0 to E+/E− at the load cell and set
  `.vref = EXT_REFP0_REFN0`.
- **Offset:** at boot the firmware measures the ADC's own offset with its inputs shorted
  internally, and subtracts it from every reading. The bridge's zero-load offset remains, so tare
  in post-processing.

```bash
pio run -e nucleo_f401re_loadcell --target upload
pio device monitor
python scripts/plot_log.py      # mV/V over time, plus noise in uV rms / p-p and effective bits
```

The log has one row per conversion: `t_us,ads_code,uv,mv_per_v`.

### Capturing and plotting

`pio device monitor` writes each session to `logs/device-monitor-*.log` through the `log2file`
filter. To plot one:

```bash
python scripts/plot_log.py                  # newest log
python scripts/plot_log.py logs/<file>.log  # a specific one
```

Press the black RESET button after the monitor opens to capture from the CSV header onward. If
you don't, the script still works: it infers the columns from the rows and drops the first one,
which the capture may have cut into. If the board resets during a capture, each run is plotted as
its own figure.

### Target position and ADC comparison

`plot_log.py` opens a second window (via `scripts/dip_analysis.py`) whenever the log has stepper
motion. The inductive sensor reads about 10.7 V with no metal in range and drops toward 0 V near
metal, so each time the target passes the sensor shows up as a dip in voltage against position.

- **Fit:** for every sweep, each dip below half depth gets a parabola fitted to its bottom 30%. The
  parabola's lowest point is that sweep's estimate of the target position, to a fraction of a step.
  Dips are grouped by position, so several metal features are handled separately.
- **Repeatability:** the scatter (σ and spread) of the per-sweep estimates for each ADC and
  direction, plus each ADC's noise while the motor is stationary.
- **Accuracy:** the log has no ground truth, so the STM32 ADC is scored against the ADS1220. Pass
  `--true-pos <steps>` to score both against the real position.
- **Direction split:** sensor lag and backlash shift forward and reverse sweeps in opposite
  directions. The report gives each direction separately and their average, which cancels both.

```bash
python scripts/plot_log.py                       # newest log, both windows + report
python scripts/plot_log.py --true-pos 137        # known target position, in steps
python scripts/plot_log.py --mm-per-rev 8        # also report positions in mm
python scripts/plot_log.py --ads-vref 3.28       # measured AVDD, for accurate volts
python scripts/plot_log.py --no-dips             # time plots only
```

Positions are in steps from where the motor was at power-on.

### Debug build

The debug environment prints every failed `Guard` check (file, line and expression) to the console:

```bash
pio run -e nucleo_f401re_debug --target upload
```

### Code style

`.clang-format` is Google style at a 100-column limit. `src/` and `include/` must match it.

Install the pinned formatter once:

```bash
pip install -r requirements-dev.txt
```

Then check or fix:

```bash
python scripts/check_format.py          # report violations, exit 1 if any
python scripts/check_format.py --fix    # rewrite the offending files
```

CI (`.github/workflows/format.yml`) runs the same command on every push to `main` and every pull
request, so a clean local run means a green check. The clang-format version is pinned in
`requirements-dev.txt`.

---

## Project files

| File | Purpose |
|------|---------|
| `platformio.ini` | PlatformIO build config |
| `.clang-format` | Google style, 100-column limit |
| `.gitattributes` | Forces LF line endings (Windows CRLF churn) |
| `.github/workflows/format.yml` | CI: fails the build on formatting violations |
| `scripts/check_format.py` | Runs clang-format over `src/` + `include/` (CI and local) |
| `src/bringup.cpp` | Bring-up test firmware (`nucleo_f401re_bringup` env) |
| `src/loadcell.cpp` | Load-cell firmware, ADS1220 only (`nucleo_f401re_loadcell` env) |
| `include/app/RigConfig.h` | Stepper duty and current limit, motion and ADS1220 settings shared by both firmwares |
| `scripts/plot_log.py` | Plots a captured CSV log |
| `scripts/dip_analysis.py` | Target position from sensor dips, and the two ADCs' repeatability and accuracy |
| `requirements-dev.txt` | Pinned developer tooling (clang-format) and plotting deps |

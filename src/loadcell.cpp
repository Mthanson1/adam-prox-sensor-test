/*
 * Load-cell firmware: the ADS1220 reading a 4-wire strain-gauge bridge, nothing
 * else. No stepper, no STM32 ADC. Built by the nucleo_f401re_loadcell
 * environment in place of main.cpp:
 *
 *   pio run -e nucleo_f401re_loadcell -t upload
 *   pio device monitor
 *
 * Wiring (4-wire bridge, ~1 kohm across S+/S- unloaded):
 *
 *   Load cell        ADS1220 / Nucleo
 *   ---------        ----------------
 *   E+ (exc +)  ->   Nucleo 5V (same pin as the ADS1220 AVDD; DVDD stays on 3V3)
 *   E- (exc -)  ->   GND (ADS1220 AVSS)
 *   S+ (sig +)  ->   AIN1
 *   S- (sig -)  ->   AIN2
 *
 * The bridge is excited from AVDD and AVDD is also the reference, so the
 * measurement is ratiometric: excitation drift cancels and the result is in
 * mV/V, the unit load cells are rated in. For the best accuracy, also tie
 * REFP0 to E+ and REFN0 to E- right at the load cell and set .vref to
 * EXT_REFP0_REFN0, so wiring drops in the excitation leads cancel too.
 *
 * Output, one row per conversion (20 per second):
 *
 *   t_us,ads_code,uv,mv_per_v
 *
 * ads_code has the ADC's own offset (measured at boot with the inputs shorted
 * internally) subtracted; the bridge's zero-load offset is still in it, so tare
 * in post-processing.
 */

#include <SPI.h>

#include <chrono>

#include "drivers/ADS1220.h"
#include "main.h"
#include "util/Guard.h"
#include "util/LineBuffer.h"

/*
 * Bridge excitation [V] = AVDD, the reference. Only scales the uv column;
 * mv_per_v is ratiometric and does not depend on it.
 */
constexpr float EXCITATION_V{5.0f};  // set to the measured 5V rail for exact uV

/*
 * Register 2 bits [5:4] = 01: simultaneous 50 and 60 Hz rejection. Only
 * effective at 20 SPS in normal mode.
 */
constexpr uint8_t REG2_REJECT_50_60{0x10};

/*
 * Gain 128 gives +/-EXCITATION_V/128 = +/-39 mV full scale (+/-7.8 mV/V),
 * room for a typical 1-3 mV/V cell plus its zero offset. The PGA must be on for
 * gain above 4; the bridge output sits at mid-supply, inside its common-mode
 * range.
 */
constexpr ADS1220::Config LOADCELL_CONFIG{
    .mux = ADS1220::Mux::AIN1_AIN2,
    .gain = ADS1220::Gain::X128,
    .pga_bypass = false,
    .rate = ADS1220::DataRate::SPS_20,
    .mode = ADS1220::Mode::NORMAL,
    .vref = ADS1220::VRef::AVDD,
    .vref_volts = EXCITATION_V,
    .reg2_low = REG2_REJECT_50_60,
};

constexpr float GAIN{128.0f};

static_assert(LOADCELL_CONFIG.gain == ADS1220::Gain::X128, "GAIN must match LOADCELL_CONFIG.gain");

/*
 * Conversions averaged for the boot-time offset measurement (0.8 s at 20 SPS).
 */
constexpr int OFFSET_CAL_SAMPLES{16};

/*
 * Longest wait for one conversion. 20 SPS takes 50 ms.
 */
constexpr uint32_t READ_TIMEOUT_MS{250};

/*
 * Minimum spacing of over-range warnings.
 */
constexpr uint32_t OVER_RANGE_WARN_MS{2000};

constexpr int LOG_UV_DECIMALS{3};
constexpr int LOG_MVV_DECIMALS{6};

static ADS1220 g_ads{SPI, ADS_CS_PIN, ADS_DRDY_PIN};

/*
 * ADC offset [codes] measured at boot, subtracted from every reading.
 */
static int32_t g_offset_code{0};

static bool readBlocking(int32_t& code) {
  const uint32_t start{millis()};
  while (!g_ads.readIfReady(code)) {
    if (millis() - start > READ_TIMEOUT_MS) {
      return false;
    }
  }
  return true;
}

/**
 * Measure the ADC's offset with AINp and AINn shorted internally at the same
 * gain, then switch back to the bridge inputs.
 */
static bool calibrateOffset() {
  ADS1220::Config shorted{LOADCELL_CONFIG};
  shorted.mux = ADS1220::Mux::SHORTED;
  Guard(g_ads.configure(shorted), false);

  int32_t code;
  Guard(readBlocking(code), false);  // first result straddles the reconfigure
  int64_t sum{0};
  for (int i{0}; i < OFFSET_CAL_SAMPLES; ++i) {
    Guard(readBlocking(code), false);
    sum += code;
  }
  g_offset_code = static_cast<int32_t>(sum / OFFSET_CAL_SAMPLES);

  Guard(g_ads.configure(LOADCELL_CONFIG), false);
  return readBlocking(code);  // discard the first bridge result too
}

void setup() {
  CONSOLE.begin(CONSOLE_BAUD);
  delay(INIT_DELAY.count());

  /*
   * No motor in this build. Hold the L298N enables low in case the driver is
   * still wired, so floating pins cannot switch a bridge on.
   */
  for (const uint32_t pin : {STEPPER_ENA_PIN, STEPPER_ENB_PIN}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  GuardFatal(g_ads.begin(LOADCELL_CONFIG));
  GuardFatal(calibrateOffset());

  static LineBuffer line;
  line.text("# load cell: AIN1-AIN2, gain 128, 20 SPS, AVDD ref, 50/60 Hz rejection").send();
  line.text("# ADC offset ").sint(g_offset_code).text(" codes, subtracted").send();
  CONSOLE.println("t_us,ads_code,uv,mv_per_v");
}

void loop() {
  int32_t raw;
  if (!g_ads.readIfReady(raw)) {
    return;
  }
  const uint32_t now_us{micros()};

  static LineBuffer line;

  /*
   * A saturated reading means the inputs are outside +/-39 mV: S+/S- open or
   * miswired, or a cell with more output than the gain allows.
   */
  static uint32_t last_warn_ms{0};
  const bool over_range{raw >= ADS1220::FULL_SCALE - 1 || raw <= -ADS1220::FULL_SCALE};
  if (over_range && millis() - last_warn_ms > OVER_RANGE_WARN_MS) {
    line.text("# over range: check S+/S- wiring, or lower the gain").send();
    last_warn_ms = millis();
  }

  const int32_t code{raw - g_offset_code};
  const float uv{g_ads.toVolts(code) * 1e6f};
  const float mv_per_v{static_cast<float>(code) / static_cast<float>(ADS1220::FULL_SCALE) / GAIN *
                       1000.0f};

  line.uint(now_us).comma();
  line.sint(code).comma();
  line.fixed(uv, LOG_UV_DECIMALS).comma();
  line.fixed(mv_per_v, LOG_MVV_DECIMALS).send();
}

void faultHandler(const char* file, int line) {
  while (true) {
    CONSOLE.print("loadcell: fault at ");
    CONSOLE.print(file);
    CONSOLE.print(":");
    CONSOLE.println(line);
    delay(1000);
  }
}

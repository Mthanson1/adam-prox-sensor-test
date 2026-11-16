/*
 * Bring-up test firmware: checks the ADS1220 over SPI and jogs the stepper from
 * the serial console. Built by the nucleo_f401re_bringup environment in place
 * of main.cpp:
 *
 *   pio run -e nucleo_f401re_bringup -t upload
 *   pio device monitor
 *
 * On boot it runs the ADS1220 checks once, then waits for single-key commands
 * (press ? for the list). The stepper starts with its coils off; any motion
 * command energises it.
 */

#include <SPI.h>

#include <chrono>
#include <cmath>

#include "app/RigConfig.h"
#include "drivers/ADS1220.h"
#include "drivers/L298NStepper.h"
#include "drivers/PeriodicTimer.h"
#include "main.h"
#include "util/Guard.h"

/*
 * Stepper update tick. Same as main.cpp, so motion here matches a real run.
 */
constexpr std::chrono::microseconds TICK_PERIOD{500};
constexpr float TICK_PERIOD_S{std::chrono::duration<float>(TICK_PERIOD).count()};

static_assert(STEPPER_MAX_SPEED <= 1.0f / TICK_PERIOD_S,
              "STEPPER_MAX_SPEED exceeds one step per tick");

/*
 * Longest wait for one ADS1220 result. The slowest rate (20 SPS) takes 50 ms.
 */
constexpr uint32_t ADS_READ_TIMEOUT_MS{200};

/*
 * Samples averaged per ADS1220 check.
 */
constexpr int ADS_AVERAGE_SAMPLES{64};

/*
 * Window over which the conversion rate is counted.
 */
constexpr uint32_t ADS_RATE_WINDOW_MS{1000};

/*
 * Pass limits. The shorted-input offset is a few uV typical; 1 mV only catches
 * a broken signal path. The ADS1220 runs from 2.3 to 5.5 V.
 */
constexpr float SHORTED_MAX_ABS_V{1.0e-3f};
constexpr float AVDD_MIN_V{2.3f};
constexpr float AVDD_MAX_V{5.5f};
constexpr float RATE_TOLERANCE{0.10f};

static L298NStepper g_stepper{STEPPER_PINS, STEP_MODE, STEPPER_PWM_HZ};
static ADS1220 g_ads{SPI, ADS_CS_PIN, ADS_DRDY_PIN};
static PeriodicTimer g_tick{TIM5};

/*
 * Set once begin() has succeeded, so the checks can be rerun without resetting.
 */
static bool g_ads_ok{false};

static void tickIsr() { g_stepper.update(TICK_PERIOD_S); }

/**
 * Run a stepper command against the ISR's motion state. The driver requires
 * motion calls to come from the update() context, so block the tick briefly.
 */
template <typename F>
static void withTickBlocked(F&& fn) {
  noInterrupts();
  fn();
  interrupts();
}

static const char* passFail(bool ok) { return ok ? "PASS" : "FAIL"; }

/**
 * Nominal conversion rate for a config [SPS], from the register 1 DR/MODE table.
 */
static float nominalRate(const ADS1220::Config& config) {
  constexpr float NORMAL_SPS[] = {20, 45, 90, 175, 330, 600, 1000};
  const float sps{NORMAL_SPS[static_cast<uint8_t>(config.rate)]};
  switch (config.mode) {
    case ADS1220::Mode::TURBO:
      return 2.0f * sps;
    case ADS1220::Mode::DUTY_CYCLE:
      return 0.25f * sps;
    default:
      return sps;
  }
}

/**
 * Block until the next conversion result.
 *
 * @return false on timeout (DRDY never asserted).
 */
static bool readBlocking(int32_t& code) {
  const uint32_t start{millis()};
  while (!g_ads.readIfReady(code)) {
    if (millis() - start > ADS_READ_TIMEOUT_MS) {
      return false;
    }
  }
  return true;
}

/**
 * Mean and standard deviation of ADS_AVERAGE_SAMPLES results [V]. The first
 * result after a (re)configure is discarded.
 */
static bool readStats(float& mean_v, float& std_v) {
  int32_t code;
  if (!readBlocking(code)) {
    return false;
  }
  double sum{0.0};
  double sum_sq{0.0};
  for (int i{0}; i < ADS_AVERAGE_SAMPLES; ++i) {
    if (!readBlocking(code)) {
      return false;
    }
    const double v{g_ads.toVolts(code)};
    sum += v;
    sum_sq += v * v;
  }
  const double mean{sum / ADS_AVERAGE_SAMPLES};
  const double var{sum_sq / ADS_AVERAGE_SAMPLES - mean * mean};
  mean_v = static_cast<float>(mean);
  std_v = static_cast<float>(std::sqrt(var > 0.0 ? var : 0.0));
  return true;
}

/**
 * Point the ADS1220 at one of its internal monitor inputs. Gain 1 with the PGA
 * bypassed is valid for every monitor, and the internal reference needs nothing
 * external.
 */
static bool configureMonitor(ADS1220::Mux mux) {
  ADS1220::Config config{ADS_CONFIG};
  config.mux = mux;
  config.gain = ADS1220::Gain::X1;
  config.pga_bypass = true;
  config.vref = ADS1220::VRef::INTERNAL_2V048;
  config.vref_volts = 2.048f;
  return g_ads.configure(config);
}

static void printRegisters() {
  uint8_t regs[4];
  g_ads.readRegisters(regs);
  CONSOLE.printf("      registers: %02X %02X %02X %02X\r\n", regs[0], regs[1], regs[2], regs[3]);
}

/**
 * The ADS1220 checks, in order of what each one proves:
 *   1. Register write/readback  - SPI wiring (SCLK, MOSI, MISO, CS) and power.
 *   2. Conversion rate          - DRDY wiring, and that the part is converting.
 *   3. Shorted input            - real conversion data comes back (~0 V).
 *   4. Supply monitor           - data is scaled right (AVDD/4 -> AVDD).
 *   5. Configured input         - reading of whatever is wired to the inputs.
 * Later checks are skipped once one fails, since they depend on it.
 */
static void runAdsTests() {
  CONSOLE.println("\r\n=== ADS1220 test ===");

  /*
   * 1. Registers.
   */
  const bool regs_ok{g_ads_ok ? g_ads.configure(ADS_CONFIG) : g_ads.begin(ADS_CONFIG)};
  g_ads_ok = g_ads_ok || regs_ok;
  CONSOLE.printf("[%s] 1. register write/readback\r\n", passFail(regs_ok));
  printRegisters();
  if (!regs_ok) {
    CONSOLE.println(
        "      All FF: MISO floating or chip unpowered. All 00: MISO shorted low or CS");
    CONSOLE.println(
        "      not reaching the chip. Check D10=CS D11=DIN D12=DOUT D13=SCLK, 3.3V, GND.");
    return;
  }

  /*
   * 2. Conversion rate, on the configured settings.
   */
  int32_t code;
  uint32_t count{0};
  const uint32_t start{millis()};
  while (millis() - start < ADS_RATE_WINDOW_MS) {
    if (g_ads.readIfReady(code)) {
      ++count;
    }
  }
  const float rate{count * 1000.0f / ADS_RATE_WINDOW_MS};
  const float expected{nominalRate(ADS_CONFIG)};
  const bool rate_ok{std::fabs(rate - expected) <= RATE_TOLERANCE * expected};
  CONSOLE.printf("[%s] 2. conversion rate: %.0f SPS (expect %.0f)\r\n", passFail(rate_ok), rate,
                 expected);
  if (count == 0) {
    CONSOLE.println("      DRDY never went low. Check DRDY -> D2.");
    return;
  }

  /*
   * 3. Inputs shorted internally to mid-supply.
   */
  float mean_v;
  float std_v;
  bool ok{configureMonitor(ADS1220::Mux::SHORTED) && readStats(mean_v, std_v)};
  ok = ok && std::fabs(mean_v) < SHORTED_MAX_ABS_V;
  CONSOLE.printf("[%s] 3. shorted input: %.2f uV, noise %.2f uV rms (expect ~0)\r\n", passFail(ok),
                 mean_v * 1e6f, std_v * 1e6f);

  /*
   * 4. (AVDD - AVSS) / 4.
   */
  ok = configureMonitor(ADS1220::Mux::SUPPLY_DIV4) && readStats(mean_v, std_v);
  const float avdd{4.0f * mean_v};
  ok = ok && avdd >= AVDD_MIN_V && avdd <= AVDD_MAX_V;
  CONSOLE.printf("[%s] 4. supply monitor: AVDD = %.4f V (expect your ADS1220 supply, e.g. 3.3)\r\n",
                 passFail(ok), avdd);

  /*
   * 5. The configured inputs. Informational: no right answer without knowing
   * what is connected.
   */
  ok = g_ads.configure(ADS_CONFIG) && readStats(mean_v, std_v);
  CONSOLE.printf("[%s] 5. configured input: %.7f V, noise %.2f uV rms\r\n", ok ? "INFO" : "FAIL",
                 mean_v, std_v * 1e6f);
}

/*
 * Duty change per [ / ] keypress.
 */
constexpr float DUTY_STEP{0.01f};

/*
 * Live duty, adjustable with [ and ]. Starts at the configured value.
 */
static float g_run_duty{STEPPER_RUN_DUTY};

/*
 * When set, a stationary motor holds at the run duty instead of the hold duty,
 * so an ammeter in series with a coil reads the run current.
 */
static bool g_hold_at_run{false};

/**
 * Worst-case coil current for a duty, from the bound in L298NStepper.h [A]. The
 * real current is lower.
 */
static float boundCurrent(float duty) {
  return duty * (MOTOR_SUPPLY_V - BRIDGE_DROP_V) / COIL_RESISTANCE_OHM;
}

static void printDuty() {
  const float hold{g_hold_at_run ? g_run_duty : g_run_duty * HOLD_CURRENT_FRACTION};
  CONSOLE.printf("run duty %.3f (<= %.2f A), stationary duty %.3f (<= %.2f A)%s\r\n", g_run_duty,
                 boundCurrent(g_run_duty), hold, boundCurrent(hold),
                 g_hold_at_run ? "  [holding at run current]" : "");
}

/**
 * Push the live duty settings to the driver.
 */
static void applyDuty() {
  const float hold{g_hold_at_run ? g_run_duty : g_run_duty * HOLD_CURRENT_FRACTION};
  withTickBlocked([&] { g_stepper.setDuty(g_run_duty, hold); });
  printDuty();
}

static void adjustDuty(float delta) {
  float duty{g_run_duty + delta};
  duty = duty < 0.0f ? 0.0f : duty;
  if (duty > STEPPER_MAX_DUTY) {
    duty = STEPPER_MAX_DUTY;
    CONSOLE.printf("at STEPPER_MAX_DUTY (%.1f A ceiling)\r\n", L298N_MAX_CURRENT_A);
  }
  g_run_duty = duty;
  applyDuty();
}

static void printHelp() {
  CONSOLE.println("\r\n=== Commands ===");
  CONSOLE.printf("  f / b   one revolution forward / back (%ld steps)\r\n",
                 static_cast<long>(STEPS_PER_REV));
  CONSOLE.println("  + / -   one step forward / back");
  CONSOLE.println("  z       zero the position here");
  CONSOLE.println("  e       energise (hold position)");
  CONSOLE.println("  x       release (coils off)");
  CONSOLE.printf("  [ / ]   run duty down / up by %.2f (hold follows at %.0f%%)\r\n", DUTY_STEP,
                 HOLD_CURRENT_FRACTION * 100.0f);
  CONSOLE.println("  h       toggle holding at run current (to measure it)");
  CONSOLE.println("  a       rerun the ADS1220 test");
  CONSOLE.println("  ?       this list");
  printDuty();
}

/**
 * Energise and move by a relative number of steps.
 */
static void jog(int32_t steps) {
  int32_t target{0};
  withTickBlocked([&] {
    g_stepper.enable();
    target = g_stepper.position() + steps;
    g_stepper.moveTo(target);
  });
  CONSOLE.printf("moving to %ld\r\n", static_cast<long>(target));
}

static void handleKey(int key) {
  switch (key) {
    case 'f':
      jog(STEPS_PER_REV);
      break;
    case 'b':
      jog(-STEPS_PER_REV);
      break;
    case '+':
    case '=':
      jog(1);
      break;
    case '-':
      jog(-1);
      break;
    case 'z':
      withTickBlocked([] { g_stepper.setPosition(0); });
      CONSOLE.println("position zeroed");
      break;
    case 'e':
      withTickBlocked([] { g_stepper.enable(); });
      CONSOLE.println("energised");
      break;
    case 'x':
      withTickBlocked([] { g_stepper.release(); });
      CONSOLE.println("released");
      break;
    case '[':
      adjustDuty(-DUTY_STEP);
      break;
    case ']':
      adjustDuty(DUTY_STEP);
      break;
    case 'h':
      g_hold_at_run = !g_hold_at_run;
      withTickBlocked([] { g_stepper.enable(); });
      applyDuty();
      break;
    case 'a':
      runAdsTests();
      break;
    case '?':
      printHelp();
      break;
    default:
      break;
  }
}

void setup() {
  CONSOLE.begin(CONSOLE_BAUD);
  delay(INIT_DELAY.count());

  GuardFatal(g_stepper.begin());
  g_stepper.setDuty(STEPPER_RUN_DUTY, STEPPER_HOLD_DUTY);
  g_stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  g_stepper.setAcceleration(STEPPER_ACCEL);
  GuardFatal(g_tick.begin(TICK_PERIOD, tickIsr));

  CONSOLE.println("\r\nprox-sensor-test bring-up");
  runAdsTests();
  printHelp();
}

void loop() {
  if (CONSOLE.available() > 0) {
    handleKey(CONSOLE.read());
  }

  /*
   * Report each move once it finishes.
   */
  static bool was_moving{false};
  bool moving{false};
  int32_t pos{0};
  withTickBlocked([&] {
    moving = g_stepper.moving();
    pos = g_stepper.position();
  });
  if (was_moving && !moving) {
    CONSOLE.printf("at %ld\r\n", static_cast<long>(pos));
  }
  was_moving = moving;
}

void faultHandler(const char* file, int line) {
  g_tick.stop();
  g_stepper.release();
  while (true) {
    CONSOLE.printf("bring-up: fault at %s:%d\r\n", file, line);
    delay(1000);
  }
}

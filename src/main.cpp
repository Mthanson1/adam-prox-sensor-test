#include "main.h"

#include <SPI.h>

#include <chrono>

#include "app/RigConfig.h"
#include "app/SampleLog.h"
#include "drivers/ADS1220.h"
#include "drivers/L298NStepper.h"
#include "drivers/PeriodicTimer.h"
#include "util/AdcToVolt.h"
#include "util/Guard.h"
#include "util/LineBuffer.h"

/*
 * ISR period: every 500 us (2 kHz) the stepper profile advances and the ADS1220
 * is polled. Steps can only land on ticks, so this sets the step-timing grid.
 */
constexpr std::chrono::microseconds ISR_PERIOD{500};

/*
 * ISR_PERIOD in seconds, for the stepper profile.
 */
constexpr float ISR_PERIOD_S{std::chrono::duration<float>(ISR_PERIOD).count()};

/*
 * Buffer drain cadence in loop(): 10 ms (100 Hz).
 */
constexpr std::chrono::milliseconds TELEMETRY_PERIOD{10};

/*
 * Dead time after boot before the stepper is energised and logging starts.
 */
constexpr std::chrono::milliseconds LOG_START_DELAY{2000};

/*
 * Log every Nth ISR tick. 4 = 500 Hz, ~23 kB/s of CSV. The on-chip ADC is only
 * sampled on logged ticks.
 */
constexpr uint32_t LOG_DECIMATION{4};

/*
 * Ring capacity in samples (power of two). 512 logged samples is ~1 s of slack
 * at 500 Hz if the host stalls.
 */
constexpr uint16_t LOG_CAPACITY{512};

/*
 * Most sample rows printed per loop() pass, so the drop report still gets out if
 * the serial link falls behind.
 */
constexpr uint32_t MAX_SAMPLES_PER_PASS{64};

/*
 * Decimal places per logged on-chip ADC voltage. One count is 0.8 mV at 12 bits.
 */
constexpr int LOG_ADC_DECIMALS{4};

/*
 * true: log raw counts from both ADCs (t_us,pos,adc_raw,ads_code,ads_new).
 * false: log sensor volts instead (t_us,pos,adc_v,ads_v,ads_new), with the
 * divider gain applied to both, so they read the sensor output directly.
 */
constexpr bool LOG_RAW{true};

/*
 * true: log only while the stepper is moving in the positive direction
 * (velocity > 0, ramps included), the way the finished device will be used.
 * Dwells and negative moves are skipped, so t_us has gaps between sweeps and
 * the stationary-noise statistics in plot_log.py have no data.
 * false: log every tick.
 */
constexpr bool LOG_POSITIVE_ONLY{true};

/*
 * Decimal places per logged ADS1220 sensor voltage. One count is ~0.6 uV at AIN0
 * with a 5 V reference, ~1.9 uV at the sensor after the 3.231:1 divider.
 */
constexpr int LOG_ADS_DECIMALS{6};

static_assert(STEPPER_MAX_SPEED <= 1.0f / ISR_PERIOD_S,
              "STEPPER_MAX_SPEED exceeds one step per ISR tick");

/*
 * Square-wave sweep about the starting position: move to -SWEEP_REVS, pause
 * SWEEP_DWELL, move to +SWEEP_REVS, pause, and repeat for as long as the run
 * lasts. The position at power-on is 0. Set SWEEP_REVS to 0 to hold at 0.
 */
constexpr int32_t SWEEP_REVS{3};

constexpr int32_t SWEEP_STEPS{SWEEP_REVS * STEPS_PER_REV};

constexpr std::chrono::milliseconds SWEEP_DWELL{500};

constexpr uint32_t SWEEP_DWELL_TICKS{
    static_cast<uint32_t>(std::chrono::microseconds(SWEEP_DWELL) / ISR_PERIOD)};

static_assert(SWEEP_STEPS >= 0, "SWEEP_REVS must be non-negative");

/*
 * Written by the ISR, drained by loop().
 */
static SampleLog<LOG_CAPACITY> g_log;

/*
 * Gates the stepper and logging together, so the capture covers the whole run.
 * Written by loop(), read by the ISR.
 */
static volatile bool g_run{false};

/*
 * Peripherals. Once g_tick starts, the ISR owns the stepper and the ADS1220's
 * SPI bus.
 */
static L298NStepper g_stepper{STEPPER_PINS, STEP_MODE, STEPPER_PWM_HZ};
static ADS1220 g_ads{SPI, ADS_CS_PIN, ADS_DRDY_PIN};
static PeriodicTimer g_tick{TIM5};

/**
 * Stepper motion profile, called every ISR tick once the run has started. Runs
 * in the ISR, so it may command the stepper directly.
 */
static void stepperProfile() {
  static uint32_t dwell_ticks{0};

  /*
   * Which end the next move heads for. Starts negative, so the first move is
   * from 0 down to -SWEEP_STEPS.
   */
  static bool next_positive{false};

  if (SWEEP_STEPS == 0 || g_stepper.moving()) {
    dwell_ticks = 0;
    return;
  }
  if (++dwell_ticks < SWEEP_DWELL_TICKS) {
    return;
  }
  dwell_ticks = 0;
  g_stepper.moveTo(next_positive ? SWEEP_STEPS : -SWEEP_STEPS);
  next_positive = !next_positive;
}

static void sampleIsr();

void setup() {
  analogReadResolution(ANALOG_READ_RES);
  CONSOLE.begin(CONSOLE_BAUD);

  delay(INIT_DELAY.count());
  GuardFatal(g_stepper.begin());
  g_stepper.setDuty(STEPPER_RUN_DUTY, STEPPER_HOLD_DUTY);
  g_stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  g_stepper.setAcceleration(STEPPER_ACCEL);
  GuardFatal(g_ads.begin(ADS_CONFIG));
  GuardFatal(g_tick.begin(ISR_PERIOD, sampleIsr));
}

void loop() {
  static std::chrono::milliseconds next_tick{0};

  const std::chrono::milliseconds now{std::chrono::milliseconds(millis())};
  if (static_cast<int32_t>(now.count() - next_tick.count()) < 0) {
    return;
  }
  next_tick = now + TELEMETRY_PERIOD;

  /*
   * Keep the coils off until the monitor has had time to attach, then emit the
   * CSV header once and release the ISR.
   */
  if (!g_run) {
    if (now < LOG_START_DELAY) {
      return;
    }
    CONSOLE.println(LOG_RAW ? "t_us,pos,adc_raw,ads_code,ads_new" : "t_us,pos,adc_v,ads_v,ads_new");
    g_run = true;
    return;
  }

  static LineBuffer line;

  /*
   * Drops leave a gap in the timestamp column, so flag them explicitly rather
   * than letting the log look continuous. Reported before the samples so a
   * backed-up sample stream cannot starve it.
   */
  static uint32_t reported_drops{0};
  const uint32_t drops{g_log.dropped()};
  if (drops != reported_drops) {
    line.text("# dropped ").uint(drops - reported_drops).send();
    reported_drops = drops;
  }

  Sample sample;
  for (uint32_t n{0}; n < MAX_SAMPLES_PER_PASS && g_log.pop(sample); ++n) {
    line.uint(sample.timestamp_us).comma();
    line.sint(sample.stepper_pos).comma();
    if (LOG_RAW) {
      line.uint(sample.adc_raw).comma();
      line.sint(sample.ads_code).comma();
    } else {
      /*
       * Both ADCs sit on the divider midpoint, so the same gain recovers the
       * sensor voltage from each.
       */
      line.fixed(adcToVolt(sample.adc_raw), LOG_ADC_DECIMALS).comma();
      line.fixed(g_ads.toVolts(sample.ads_code) * ANALOG_SNS_GAIN, LOG_ADS_DECIMALS).comma();
    }
    line.uint(sample.ads_new ? 1 : 0).send();
  }
}

/*
 * Timer ISR, every ISR_PERIOD. Polls the ADS1220, advances the stepper and logs
 * on every LOG_DECIMATION-th tick.
 */
static void sampleIsr() {
  static uint32_t tick{0};
  static bool energised{false};

  /*
   * Held across ticks so a logged row without a fresh ADS1220 result carries the
   * last one. ads_new latches until the next logged row.
   */
  static int32_t ads_code{0};
  static bool ads_new{false};

  const uint32_t now_us{micros()};
  ads_new |= g_ads.readIfReady(ads_code);

  /*
   * Coils stay off (as begin() left them) until the run starts.
   */
  if (!g_run) {
    tick = 0;
    return;
  }
  if (!energised) {
    g_stepper.enable();
    energised = true;
  }

  stepperProfile();
  g_stepper.update(ISR_PERIOD_S);

  const bool log_this_tick{!LOG_POSITIVE_ONLY || g_stepper.velocity() > 0.0f};
  if ((tick % LOG_DECIMATION) == 0 && log_this_tick) {
    const uint16_t adc_raw{static_cast<uint16_t>(analogRead(ANALOG_SNS_PIN))};
    g_log.push(Sample{now_us, g_stepper.position(), adc_raw, ads_code, ads_new});
    ads_new = false;
  }

  if (++tick >= LOG_DECIMATION) {
    tick = 0;
  }
}

/*
 * Repeat period of the fault message, so a monitor attached late still sees it.
 */
constexpr std::chrono::milliseconds FAULT_REPORT_PERIOD{1000};

void faultHandler(const char* file, int line) {
  /*
   * Stop the tick first so the ISR cannot energise the coils again, then turn
   * them off. Interrupts stay on: the UART TX path and delay() both need them.
   */
  g_tick.stop();
  g_stepper.release();

  while (true) {
    CONSOLE.print("prox-sensor-test: fault at ");
    CONSOLE.print(file);
    CONSOLE.print(":");
    CONSOLE.println(line);
    delay(FAULT_REPORT_PERIOD.count());
  }
}

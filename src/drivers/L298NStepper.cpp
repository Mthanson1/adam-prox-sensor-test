#include "drivers/L298NStepper.h"

#include <algorithm>
#include <cmath>

#include "util/Guard.h"

namespace {

/*
 * Half-step sequence as {coil A, coil B} current direction: +1, -1 or 0 (off).
 * Odd entries have both coils on; FULL mode steps through those only.
 */
constexpr int8_t HALF_STEP_TABLE[8][2] = {
    {+1, 0}, {+1, +1}, {0, +1}, {-1, +1}, {-1, 0}, {-1, -1}, {0, -1}, {+1, -1},
};

constexpr uint8_t PHASE_MASK{7};

}  // namespace

bool L298NStepper::begin() {
  Guard(started_ == 0, false);

  /*
   * Enables first and low, so neither bridge drives a coil while the IN pins
   * come up.
   */
  bridge_a_.en_pin = pins_.en_a;
  bridge_b_.en_pin = pins_.en_b;
  for (const uint32_t pin : {pins_.en_a, pins_.en_b}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  Guard(beginPwm(bridge_a_.in_pos, pins_.in1), false);
  Guard(beginPwm(bridge_a_.in_neg, pins_.in2), false);
  Guard(beginPwm(bridge_b_.in_pos, pins_.in3), false);
  Guard(beginPwm(bridge_b_.in_neg, pins_.in4), false);

  enabled_ = false;
  return true;
}

bool L298NStepper::beginPwm(PwmOut& out, uint32_t pin) {
  const PinName name{digitalPinToPinName(pin)};
  auto* const instance{static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_PWM))};
  GuardMsg(instance != nullptr, false, "pin %lu has no PWM timer", pin);
  out.channel = STM_PIN_CHANNEL(pinmap_function(name, PinMap_PWM));

  /*
   * STM32duino allows one HardwareTimer per timer peripheral (a second one calls
   * Error_Handler), so share it with any earlier output on the same timer.
   */
  for (uint8_t i{0}; i < started_; ++i) {
    const PwmOut& other{*outputs_[i]};
    if (other.timer->getHandle()->Instance == instance) {
      GuardMsg(other.channel != out.channel, false, "two IN pins on one timer channel");
      out.timer = other.timer;
      break;
    }
  }
  if (out.timer == nullptr) {
    out.timer = new HardwareTimer(instance);
    out.timer->setOverflow(pwm_hz_, HERTZ_FORMAT);
  }

  out.timer->setMode(out.channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  out.period_ticks = out.timer->getOverflow(TICK_FORMAT);
  out.timer->setCaptureCompare(out.channel, 0, TICK_COMPARE_FORMAT);
  out.timer->resume();
  ++started_;
  return true;
}

/*
 * Everything below runs in ISR context, so it bails out with a plain return
 * rather than Guard, which prints in DEBUG builds.
 */

void L298NStepper::setDuty(float run_duty, float hold_duty) {
  run_duty_ = std::isnan(run_duty) ? 0.0f : std::clamp(run_duty, 0.0f, 1.0f);
  hold_duty_ = std::isnan(hold_duty) ? 0.0f : std::clamp(hold_duty, 0.0f, 1.0f);
  if (enabled_) {
    applyPhase();
  }
}

void L298NStepper::enable() {
  if (started_ != 4) {
    return;
  }
  enabled_ = true;
  applyPhase();
}

void L298NStepper::release() {
  if (started_ != 4) {
    return;
  }
  enabled_ = false;
  running_ = false;
  motion_ = Motion::VELOCITY;
  target_velocity_ = 0.0f;
  velocity_ = 0.0f;
  step_accum_ = 0.0f;
  setCoil(bridge_a_, 0, 0.0f);
  setCoil(bridge_b_, 0, 0.0f);
}

void L298NStepper::moveTo(int32_t target_steps) {
  motion_ = Motion::POSITION;
  target_ = target_steps;
}

void L298NStepper::setVelocity(float steps_per_s) {
  motion_ = Motion::VELOCITY;
  target_velocity_ = std::isnan(steps_per_s) ? 0.0f : steps_per_s;
}

void L298NStepper::setPosition(int32_t steps) {
  position_ = steps;
  target_ = steps;
  if (motion_ == Motion::POSITION) {
    velocity_ = 0.0f;
    step_accum_ = 0.0f;
  }
}

bool L298NStepper::moving() const {
  return velocity_ != 0.0f || (motion_ == Motion::POSITION && position_ != target_);
}

void L298NStepper::update(float dt_s) {
  if (!enabled_ || !(dt_s > 0.0f)) {
    return;
  }

  /*
   * At most one step per call, so speeds above 1/dt cannot be produced.
   */
  const float v_max{std::min(max_speed_, 1.0f / dt_s)};
  const float dv{accel_ * dt_s};

  float v_goal{0.0f};
  if (motion_ == Motion::POSITION) {
    const int32_t remaining{target_ - position_};
    if (remaining == 0) {
      /*
       * Arrived. The profile has already slowed to about sqrt(2 * accel) steps/s
       * here, which a stepper stops from without losing position.
       */
      velocity_ = 0.0f;
      step_accum_ = 0.0f;
    } else {
      const float dir{remaining > 0 ? 1.0f : -1.0f};
      const float stopping_steps{velocity_ * velocity_ / (2.0f * std::max(accel_, 1.0f))};
      const bool heading_away{velocity_ * dir < 0.0f};
      const bool must_brake{heading_away ||
                            static_cast<float>(std::abs(remaining)) <= stopping_steps};
      v_goal = must_brake ? 0.0f : dir * v_max;
    }
  } else {
    v_goal = std::clamp(target_velocity_, -v_max, v_max);
  }

  velocity_ += std::clamp(v_goal - velocity_, -dv, dv);

  step_accum_ += velocity_ * dt_s;
  if (step_accum_ >= 1.0f) {
    step_accum_ -= 1.0f;
    step(+1);
  } else if (step_accum_ <= -1.0f) {
    step_accum_ += 1.0f;
    step(-1);
  }

  /*
   * Switch between run and hold current only on a change, so a stationary motor
   * does not get its PWM rewritten every tick.
   */
  const bool running{moving()};
  if (running != running_) {
    running_ = running;
    applyPhase();
  }
}

void L298NStepper::step(int dir) {
  position_ += dir;
  phase_ = static_cast<uint8_t>((phase_ + dir * step_inc_) & PHASE_MASK);
  applyPhase();
}

void L298NStepper::applyPhase() {
  if (!enabled_) {
    return;
  }
  const float duty{running_ ? run_duty_ : hold_duty_};
  setCoil(bridge_a_, HALF_STEP_TABLE[phase_][0], duty);
  setCoil(bridge_b_, HALF_STEP_TABLE[phase_][1], duty);
}

void L298NStepper::setCoil(const Bridge& bridge, int8_t sign, float duty) {
  if (sign == 0) {
    /*
     * Coil off: disable the bridge so the coil floats, rather than holding both
     * IN pins low, which would short it through the low side and add drag.
     */
    digitalWrite(bridge.en_pin, LOW);
    setCompare(bridge.in_pos, 0.0f);
    setCompare(bridge.in_neg, 0.0f);
    return;
  }

  /*
   * Idle side to 0 before the driven side gets its duty, so a reversal passes
   * through "both low" (slow-decay brake) rather than "both PWM".
   */
  const PwmOut& driven{sign > 0 ? bridge.in_pos : bridge.in_neg};
  const PwmOut& idle{sign > 0 ? bridge.in_neg : bridge.in_pos};
  setCompare(idle, 0.0f);
  setCompare(driven, duty);
  digitalWrite(bridge.en_pin, HIGH);
}

void L298NStepper::setCompare(const PwmOut& out, float duty) {
  const uint32_t ticks{
      static_cast<uint32_t>(std::lround(duty * static_cast<float>(out.period_ticks)))};
  out.timer->setCaptureCompare(out.channel, ticks, TICK_COMPARE_FORMAT);
}

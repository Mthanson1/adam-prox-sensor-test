/*
 * Driver for a 4-wire (bipolar) stepper motor on an L298N dual H-bridge.
 *
 * Bridge A (ENA, IN1, IN2 -> OUT1/OUT2) drives coil A and bridge B (ENB, IN3,
 * IN4 -> OUT3/OUT4) drives coil B.
 *
 * Drive scheme (sign-magnitude, slow decay)
 * -----------------------------------------
 * ENA/ENB are plain on/off outputs: high while that coil should carry current,
 * low to switch the bridge off (half-step "coil off" states, release()). For
 * the energised direction, one IN pin carries the PWM and the other is held low:
 *
 *   coil +   IN_pos = PWM, IN_neg = L
 *   coil -   IN_pos = L,   IN_neg = PWM
 *
 * During the PWM off-time both IN pins are low, so both low-side switches are
 * on and the coil current recirculates through the bottom of the bridge (one
 * low-side transistor plus one flyback diode, ~1.5-2 V). It decays slowly, so
 * the current stays continuous at the PWM frequency and tracks the duty.
 *
 * This is why the IN pins, not the enables, need timer channels. PWM on the
 * enables instead gives fast decay: the coil discharges into the supply through
 * all four diodes at ~-Vs, the current collapses each period, and almost none
 * flows at low duty.
 *
 * Current limiting
 * ----------------
 * The L298N has no current sensing, so the coil current is set open-loop by the
 * duty. With slow decay,
 *
 *   I_coil = [duty * (V_supply - V_drop) - (1 - duty) * V_recirc] / R_coil
 *         <= duty * (V_supply - V_drop) / R_coil
 *
 * The bound (V_recirc taken as 0) is what dutyForCurrent() uses, so a duty from
 * it never gives more than the target; expect the real current to be lower,
 * perhaps half, and measure it. A lower hold duty applies whenever the motor is
 * stationary, so holding position does not cook the motor or the L298N.
 *
 * Motion
 * ------
 * update() is called at a fixed rate from a timer ISR. It ramps the speed with
 * a constant acceleration (trapezoidal profile) and takes at most one step per
 * call, so the top step rate is 1 / update period. Steps land on update ticks,
 * so call update() several times faster than the step rate you need.
 *
 * Every method except begin() is ISR-safe. moveTo()/setVelocity()/etc. must be
 * called from the same context as update() (or with that ISR stopped), because
 * the motion state is not otherwise protected.
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

class L298NStepper {
 public:
  /**
   * Step sequence.
   *
   *  FULL: both coils always on (two-phase-on full step). Full torque, the
   *        motor's native step angle (1.8 deg on a 200-step motor).
   *  HALF: alternates one and two coils on. Half the step angle and smoother,
   *        with ~30% less torque on the one-coil steps.
   *
   * position() counts steps of the selected mode.
   */
  enum class StepMode : uint8_t { FULL, HALF };

  struct Pins {
    uint32_t en_a;  // ENA, plain GPIO
    uint32_t in1;   // IN1..IN4 must each be on a timer channel (PinMap_PWM)
    uint32_t in2;
    uint32_t en_b;  // ENB, plain GPIO
    uint32_t in3;
    uint32_t in4;
  };

  /**
   * @param pins   Pin mapping. IN pins may share timers (on different channels);
   *               every timer used is set to pwm_hz.
   * @param mode   Step sequence; fixed for the life of the driver.
   * @param pwm_hz IN-pin PWM frequency. The L298N's bipolar outputs switch
   *               slowly, so keep this at or below ~25 kHz.
   */
  L298NStepper(const Pins& pins, StepMode mode, uint32_t pwm_hz)
      : pins_{pins},
        step_inc_{static_cast<uint8_t>(mode == StepMode::FULL ? 2 : 1)},
        pwm_hz_{pwm_hz} {}

  /**
   * Duty for a target coil current, from the slow-decay bound above.
   * Clamped to [0, 1]. Use the lowest coil resistance and bridge drop you expect
   * so the real current lands at or below current_a.
   *
   * @param current_a   Target coil current [A].
   * @param supply_v    Motor supply voltage at the L298N VS terminal [V].
   * @param drop_v      Total L298N bridge drop (upper + lower switch) [V].
   * @param coil_ohm    Coil resistance [ohm].
   */
  static constexpr float dutyForCurrent(float current_a, float supply_v, float drop_v,
                                        float coil_ohm) {
    const float duty{current_a * coil_ohm / (supply_v - drop_v)};
    return duty < 0.0f ? 0.0f : (duty > 1.0f ? 1.0f : duty);
  }

  /**
   * Configure the pins and start the PWM timers with both coils off.
   *
   * @return false if an IN pin has no timer channel, two IN pins share one, or
   *         begin() was already called.
   */
  bool begin();

  /**
   * Set the PWM duty used while moving and while stationary. Both clamped to
   * [0, 1]. Takes effect immediately.
   */
  void setDuty(float run_duty, float hold_duty);

  /**
   * Energise the coils at the present step so the motor holds position.
   */
  void enable();

  /**
   * Turn both coils off. The motor spins freely and motion stops; position() is
   * kept but may no longer match the shaft if it is moved by hand.
   */
  void release();

  /**
   * Speed limit for both motion modes [steps/s]. Also capped at one step per
   * update() call.
   */
  void setMaxSpeed(float steps_per_s) { max_speed_ = steps_per_s > 0.0f ? steps_per_s : 0.0f; }

  /**
   * Acceleration and deceleration [steps/s^2]. Too high and the motor stalls
   * (loses steps) instead of following.
   */
  void setAcceleration(float steps_per_s2) { accel_ = steps_per_s2 > 0.0f ? steps_per_s2 : 0.0f; }

  /**
   * Position mode: move to an absolute step count, decelerating to stop on it.
   */
  void moveTo(int32_t target_steps);

  /**
   * Velocity mode: ramp to a signed speed and hold it [steps/s].
   */
  void setVelocity(float steps_per_s);

  /**
   * Decelerate to a stop (velocity mode, 0 steps/s).
   */
  void stop() { setVelocity(0.0f); }

  /**
   * Redefine the present position, e.g. to zero it at a home position. Cancels
   * any position move in progress.
   */
  void setPosition(int32_t steps);

  /**
   * Advance the motion profile by dt_s and take a step if one is due. Call at a
   * fixed rate from a timer ISR.
   */
  void update(float dt_s);

  /**
   * Current position [steps of the selected StepMode].
   */
  int32_t position() const { return position_; }

  /**
   * Current profile speed [steps/s], signed.
   */
  float velocity() const { return velocity_; }

  /**
   * True while the profile is moving or a position move has not finished.
   */
  bool moving() const;

 private:
  enum class Motion : uint8_t { POSITION, VELOCITY };

  /*
   * One PWM output on a timer channel.
   */
  struct PwmOut {
    HardwareTimer* timer{nullptr};
    uint32_t channel{0};
    uint32_t period_ticks{0};
  };

  /*
   * One H-bridge: its enable pin and the PWM outputs on its two IN pins.
   */
  struct Bridge {
    uint32_t en_pin{0};
    PwmOut in_pos;
    PwmOut in_neg;
  };

  /**
   * Start PWM on pin, reusing the HardwareTimer of any already-started output on
   * the same timer peripheral.
   */
  bool beginPwm(PwmOut& out, uint32_t pin);

  /**
   * Move one step in direction dir (+1 / -1) and drive the new coil state.
   */
  void step(int dir);

  /**
   * Drive both coils for the present phase at the present duty.
   */
  void applyPhase();

  static void setCoil(const Bridge& bridge, int8_t sign, float duty);

  static void setCompare(const PwmOut& out, float duty);

  const Pins pins_;

  /*
   * Phase-table increment per step: 2 walks the two-coil entries only (full
   * step), 1 walks every entry (half step).
   */
  const uint8_t step_inc_;

  const uint32_t pwm_hz_;

  Bridge bridge_a_;
  Bridge bridge_b_;

  /*
   * Every PWM output, in begin() order, so later ones can find a timer to share.
   */
  PwmOut* outputs_[4]{&bridge_a_.in_pos, &bridge_a_.in_neg, &bridge_b_.in_pos, &bridge_b_.in_neg};
  uint8_t started_{0};

  /*
   * Index into HALF_STEP_TABLE, 0..7. Starts on a two-coil entry so FULL mode
   * only ever visits two-coil entries.
   */
  uint8_t phase_{1};

  bool enabled_{false};
  bool running_{false};
  float run_duty_{0.0f};
  float hold_duty_{0.0f};

  Motion motion_{Motion::VELOCITY};
  int32_t position_{0};
  int32_t target_{0};
  float target_velocity_{0.0f};
  float velocity_{0.0f};
  float max_speed_{0.0f};
  float accel_{0.0f};

  /*
   * Fractional steps accumulated from velocity * dt, kept in (-1, 1).
   */
  float step_accum_{0.0f};
};

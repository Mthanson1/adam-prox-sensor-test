/*
 * Rig hardware configuration shared by the main firmware (main.cpp) and the
 * bring-up test (bringup.cpp), so both drive the stepper at the same current and
 * read the ADS1220 the same way.
 */

#pragma once

#include <stdint.h>

#include "drivers/ADS1220.h"
#include "drivers/L298NStepper.h"
#include "main.h"

/*
 * ----- Stepper current limit -----
 * The coil current is set open-loop by the IN-pin PWM duty (see L298NStepper.h).
 * The values below are chosen at the end of their range that gives the MOST
 * current, so the ceiling they produce is conservative: the real current at a
 * given duty is lower, perhaps half. Tune the duty on the bench with the
 * bring-up firmware's [ ] keys and an ammeter.
 */

/*
 * Motor supply at the L298N VS terminal [V].
 */
constexpr float MOTOR_SUPPLY_V{12.0f};

/*
 * Coil resistance [ohm]. Measured 2 to 5 ohm; the low end is the worst case.
 */
constexpr float COIL_RESISTANCE_OHM{2.0f};

/*
 * Total L298N bridge drop at ~1 A [V]. The datasheet gives roughly 1.8 to 3.2 V
 * (upper + lower switch); the low end is the worst case.
 */
constexpr float BRIDGE_DROP_V{1.8f};

/*
 * Holding current as a fraction of the run current, applied whenever the motor
 * is stationary. Holding torque scales with it; heat scales with its square.
 */
constexpr float HOLD_CURRENT_FRACTION{0.5f};

static_assert(MOTOR_SUPPLY_V > BRIDGE_DROP_V, "MOTOR_SUPPLY_V must exceed BRIDGE_DROP_V");
static_assert(HOLD_CURRENT_FRACTION >= 0.0f && HOLD_CURRENT_FRACTION <= 1.0f,
              "HOLD_CURRENT_FRACTION must be in [0, 1]");

/*
 * Coil current ceiling [A]. 2.5 A is the L298's repetitive peak rating; its
 * continuous DC rating is 2 A per bridge, so the margin here is thinner than at
 * 2 A. Watch the heatsink temperature at high duty.
 */
constexpr float L298N_MAX_CURRENT_A{2.5f};

/*
 * Hard ceiling on the duty: the duty at which the worst-case bound reaches
 * L298N_MAX_CURRENT_A (~0.49 at 12 V, 2 ohm). The real current at this duty is
 * lower, so any duty up to it keeps the driver within the ceiling (not
 * necessarily the motor within its rating).
 */
constexpr float STEPPER_MAX_DUTY{L298NStepper::dutyForCurrent(L298N_MAX_CURRENT_A, MOTOR_SUPPLY_V,
                                                              BRIDGE_DROP_V, COIL_RESISTANCE_OHM)};

/*
 * IN-pin PWM duty while moving. Set from bench testing; the worst-case bound puts
 * 0.4 at <= 2.04 A into 2 ohm from 12 V, and the real current is lower.
 */
constexpr float STEPPER_RUN_DUTY{0.4f};
constexpr float STEPPER_HOLD_DUTY{STEPPER_RUN_DUTY * HOLD_CURRENT_FRACTION};

static_assert(STEPPER_RUN_DUTY > 0.0f && STEPPER_RUN_DUTY <= STEPPER_MAX_DUTY,
              "STEPPER_RUN_DUTY must be in (0, STEPPER_MAX_DUTY] to keep the L298N under "
              "L298N_MAX_CURRENT_A");

/*
 * IN-pin PWM frequency. Above the audible range, and slow enough for the
 * L298N's bipolar output stage to switch cleanly.
 */
constexpr uint32_t STEPPER_PWM_HZ{20000};

/*
 * ----- Stepper motion -----
 */

/*
 * FULL for torque, HALF for finer, smoother steps. Positions below are in steps
 * of this mode (200/rev full, 400/rev half on a 1.8 deg motor).
 */
constexpr L298NStepper::StepMode STEP_MODE{L298NStepper::StepMode::FULL};

/*
 * Motor full steps per revolution (200 for a 1.8 deg motor).
 */
constexpr int32_t MOTOR_FULL_STEPS_PER_REV{200};

/*
 * Steps per revolution in the configured STEP_MODE.
 */
constexpr int32_t STEPS_PER_REV{STEP_MODE == L298NStepper::StepMode::FULL
                                    ? MOTOR_FULL_STEPS_PER_REV
                                    : 2 * MOTOR_FULL_STEPS_PER_REV};

/*
 * Speed limit [steps/s]. Voltage-limited drive loses torque quickly with speed,
 * so start low and raise it until the motor misses steps, then back off.
 */
constexpr float STEPPER_MAX_SPEED{50.0f};

/*
 * Acceleration [steps/s^2].
 */
constexpr float STEPPER_ACCEL{800.0f};

/*
 * ----- ADS1220 -----
 * The data rate is above main.cpp's log rate, so every logged row normally
 * carries a fresh conversion (ads_new = 1).
 */
/*
 * The sensor (0 to 10.7 V) feeds a divider that scales it to 0 to 3.31 V (see
 * ANALOG_SNS_GAIN in AdcToVolt.h); the divider output goes to AIN0 and to A0 on
 * the Nucleo. Single-ended, so AIN0 is measured against AVSS,
 * which requires the PGA bypassed (legal at gain 1, 2 or 4). AVDD is the
 * reference, giving a 0 to AVDD range; vref_volts only scales the volts columns,
 * so set it to the measured AVDD.
 */
constexpr ADS1220::Config ADS_CONFIG{
    .mux = ADS1220::Mux::AIN0_AVSS,
    .gain = ADS1220::Gain::X1,
    .pga_bypass = true,
    .rate = ADS1220::DataRate::SPS_1000,
    .mode = ADS1220::Mode::NORMAL,
    .vref = ADS1220::VRef::AVDD,
    .vref_volts = 5.0f,  // AVDD on the Nucleo 5V rail; set to its measured value
};

/**
 * Pin mapping for the stepper, from main.h.
 */
constexpr L298NStepper::Pins STEPPER_PINS{
    .en_a = STEPPER_ENA_PIN,
    .in1 = STEPPER_IN1_PIN,
    .in2 = STEPPER_IN2_PIN,
    .en_b = STEPPER_ENB_PIN,
    .in3 = STEPPER_IN3_PIN,
    .in4 = STEPPER_IN4_PIN,
};

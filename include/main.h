/*
 * Shared application header: board pin mapping and cross-module declarations.
 *
 * Pin names are the Nucleo's Arduino-header labels; the STM32 port pin and the
 * peripheral it is used for are noted on each.
 */

#pragma once

#include <Arduino.h>

/*
 * Arduino.h defines abs() as a one-argument macro, which breaks the
 * std::chrono::abs(duration) declaration inside <chrono>.
 */
#undef abs

#include <chrono>

#include "util/Console.h"

/*
 * ----- L298N driving a bipolar stepper -----
 * Coil A on OUT1/OUT2 (bridge A), coil B on OUT3/OUT4 (bridge B). The IN pins
 * carry the current-limiting PWM, so each is on a timer channel: IN1/IN2 on
 * TIM1, IN3/IN4 on TIM4. ENA/ENB are plain on/off outputs; remove the module's
 * ENA/ENB jumpers so these pins drive them. TIM5 is the sample tick.
 */

/**
 * ENA, coil A on/off. D9 = PC7.
 */
static constexpr uint32_t STEPPER_ENA_PIN{D9};

/**
 * IN1, coil A + PWM. D8 = PA9 = TIM1_CH2.
 */
static constexpr uint32_t STEPPER_IN1_PIN{D8};

/**
 * IN2, coil A - PWM. D7 = PA8 = TIM1_CH1.
 */
static constexpr uint32_t STEPPER_IN2_PIN{D7};

/**
 * ENB, coil B on/off. D6 = PB10.
 */
static constexpr uint32_t STEPPER_ENB_PIN{D6};

/**
 * IN3, coil B + PWM. D15 = PB8 = TIM4_CH3.
 */
static constexpr uint32_t STEPPER_IN3_PIN{D15};

/**
 * IN4, coil B - PWM. D14 = PB9 = TIM4_CH4.
 */
static constexpr uint32_t STEPPER_IN4_PIN{D14};

/*
 * ----- On-chip ADC -----
 */

/**
 * Analog voltage input. A0 = PA0 = ADC1_IN0. 0 to 3.3 V only: the pin is not
 * 5 V tolerant in analog mode.
 */
static constexpr uint32_t ANALOG_SNS_PIN{A0};

/**
 * AnalogRead Resolution (bits). 12 is the STM32F401's native ADC width.
 */
static constexpr uint8_t ANALOG_READ_RES{12};

/*
 * ----- ADS1220 on SPI1 -----
 * SCLK = D13 = PA5, MISO (DOUT) = D12 = PA6, MOSI (DIN) = D11 = PA7. These are
 * the default SPI pins, so they are not configured here. PA5 is also the green
 * user LED (LD2), which flickers with SPI traffic.
 */

/**
 * ADS1220 chip select, active low. D10 = PB6.
 */
static constexpr uint32_t ADS_CS_PIN{D10};

/**
 * ADS1220 DRDY, active low. D2 = PA10.
 */
static constexpr uint32_t ADS_DRDY_PIN{D2};

/**
 * Init/Setup Delay After CONSOLE.begin().
 */
static constexpr std::chrono::milliseconds INIT_DELAY{300};

/*
 * Helper Util function for converting raw on-chip ADC counts to volts.
 */

#pragma once

#include <stdint.h>

#include "main.h"

/**
 * ADC reference [V]. On the Nucleo VREF+ is tied to VDDA, which comes from the
 * 3.3 V regulator; measure it and update this for better absolute accuracy.
 */
static constexpr float ADC_VREF_V{3.3f};

/**
 * Largest code at ANALOG_READ_RES bits.
 */
static constexpr float ADC_MAX_CODE{static_cast<float>((1UL << ANALOG_READ_RES) - 1)};

/**
 * Volts at the sensed node per volt at ANALOG_SNS_PIN. Set to (R1 + R2) / R2 if
 * the signal goes through a divider to stay under 3.3 V; 1 for a direct connection.
 */
static constexpr float DIVIDER_R_TOP_OHM{2231.0f};
static constexpr float DIVIDER_R_BOTTOM_OHM{1000.0f};
static constexpr float ANALOG_SNS_GAIN{(DIVIDER_R_TOP_OHM + DIVIDER_R_BOTTOM_OHM) /
                                       DIVIDER_R_BOTTOM_OHM};  // 3.231: 10.7 V -> 3.31 V

/**
 * Convert an ANALOG_SNS_PIN reading to volts at the sensed node.
 *
 * @param raw_adc The raw ADC reading.
 * @return The sensed voltage in volts.
 */
inline float adcToVolt(uint16_t raw_adc) {
  return static_cast<float>(raw_adc) * (ADC_VREF_V / ADC_MAX_CODE) * ANALOG_SNS_GAIN;
}

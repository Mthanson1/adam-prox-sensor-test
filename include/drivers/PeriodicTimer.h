/*
 * Periodic interrupt on a general-purpose timer: the fixed-rate tick that drives
 * sampling and the motor command.
 *
 * The callback runs in interrupt context at TIM_IRQ_PRIO (14 by default in
 * STM32duino), below SysTick and the UART, so millis()/micros() keep counting
 * and the console keeps draining while it runs. Keep it short and do not call
 * CONSOLE or delay() from it.
 */

#pragma once

#include <Arduino.h>

#undef abs

#include <chrono>

class PeriodicTimer {
 public:
  /**
   * @param instance Timer peripheral to use, e.g. TIM5. Pick one with no PWM
   *                 pins in use: the period reconfigures the whole timer.
   */
  explicit PeriodicTimer(TIM_TypeDef* instance) : instance_{instance} {}

  /**
   * Start calling isr every period.
   *
   * @param period Tick period; must be positive.
   * @param isr    Callback, run in interrupt context.
   * @return false on a bad argument or if begin() was already called.
   */
  bool begin(std::chrono::microseconds period, void (*isr)());

  /**
   * Stop the tick. Safe to call before begin().
   */
  void stop();

 private:
  TIM_TypeDef* const instance_;

  /*
   * Allocated once in begin(): HardwareTimer touches the RCC in its constructor,
   * so it cannot be a static-storage object constructed before HAL init.
   */
  HardwareTimer* timer_{nullptr};
};

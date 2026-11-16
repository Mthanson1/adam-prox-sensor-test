#include "drivers/PeriodicTimer.h"

#include "util/Guard.h"

bool PeriodicTimer::begin(std::chrono::microseconds period, void (*isr)()) {
  Guard(timer_ == nullptr, false);
  Guard(period.count() > 0, false);
  Guard(isr != nullptr, false);

  timer_ = new HardwareTimer(instance_);
  timer_->setOverflow(static_cast<uint32_t>(period.count()), MICROSEC_FORMAT);
  timer_->attachInterrupt(isr);
  timer_->resume();
  return true;
}

void PeriodicTimer::stop() {
  if (timer_ != nullptr) {
    timer_->pause();
  }
}

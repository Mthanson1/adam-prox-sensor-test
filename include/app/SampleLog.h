/*
 * Lock-free SPSC ring buffer for sample telemetry.
 *
 * The sample ISR is the only producer and loop() is the only consumer, so no
 * critical section is needed.
 *
 * push() is ISR-safe: no allocation, no blocking, no console output.
 */

#pragma once

#include <stdint.h>

#include <atomic>

/**
 * One sample, as logged by the ISR. Raw values only: conversion to engineering
 * units happens in loop() so the ISR stays short.
 */
struct Sample {
  /**
   * micros() at the top of the sample ISR.
   */
  uint32_t timestamp_us;

  /**
   * Stepper position [steps of the configured StepMode].
   */
  int32_t stepper_pos;

  /**
   * Raw on-chip ADC reading of ANALOG_SNS_PIN [counts].
   */
  uint16_t adc_raw;

  /**
   * Latest ADS1220 conversion result, sign-extended 24-bit [counts].
   */
  int32_t ads_code;

  /**
   * True if at least one new ADS1220 conversion arrived since the previous
   * logged sample, false if ads_code is the previous value held over.
   */
  bool ads_new;
};

/**
 * Single-producer / single-consumer ring buffer of Sample.
 *
 * @tparam N Capacity in samples; must be a power of two. One slot is kept empty
 *           to disambiguate full from empty, so N-1 samples are storable.
 */
template <uint16_t N>
class SampleLog {
  static_assert(N >= 2, "SampleLog capacity must be at least 2");
  static_assert((N & (N - 1)) == 0, "SampleLog capacity must be a power of two");

 public:
  /**
   * Append one sample. Producer side only.
   *
   * @param sample The sample to store.
   * @return true if stored, false if the buffer was full.
   */
  bool push(const Sample& sample) {
    const uint16_t head{head_};
    const uint16_t next{static_cast<uint16_t>((head + 1) & (N - 1))};
    if (next == tail_) {
      ++dropped_;
      return false;
    }

    buffer_[head] = sample;
    /*
     * Publish the payload before the index that makes it visible.
     */
    std::atomic_signal_fence(std::memory_order_release);
    head_ = next;
    return true;
  }

  /**
   * Remove the oldest sample. Consumer side only.
   *
   * @param out Destination for the sample, untouched if the buffer is empty.
   * @return true if a sample was written, false if the buffer is empty.
   */
  bool pop(Sample& out) {
    const uint16_t tail{tail_};
    if (tail == head_) {
      return false;
    }

    out = buffer_[tail];
    /*
     * Consume the payload before freeing the slot for the producer.
     */
    std::atomic_signal_fence(std::memory_order_acquire);
    tail_ = static_cast<uint16_t>((tail + 1) & (N - 1));
    return true;
  }

  /**
   * Total samples dropped since boot. Monotonic, so the consumer can diff it
   * against its own last-seen value without racing the producer's increment.
   */
  uint32_t dropped() const { return dropped_; }

 private:
  /**
   * The buffer itself. One slot is kept empty to disambiguate full from empty.
   */
  Sample buffer_[N];

  /**
   * Next slot the producer writes. Written by the producer only.
   */
  volatile uint16_t head_{0};

  /**
   * Next slot the consumer reads. Written by the consumer only.
   */
  volatile uint16_t tail_{0};

  /**
   * Samples discarded because the buffer was full. Written by the producer only.
   */
  volatile uint32_t dropped_{0};
};

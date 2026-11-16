/*
 * Line builder for console output.
 *
 * Print::print(float) writes a digit at a time and goes through the UART TX ring
 * byte by byte, so each row is formatted here and handed to the ring with a single
 * write().
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <cmath>

#include "util/Console.h"
#include "util/Guard.h"

class LineBuffer {
 public:
  /**
   * Append a C string. Truncated if the line is full.
   */
  LineBuffer& text(const char* s) {
    while (*s != '\0' && len_ < CAPACITY) {
      buf_[len_++] = *s++;
    }
    return *this;
  }

  /**
   * Append an unsigned integer in decimal.
   */
  LineBuffer& uint(uint32_t v) {
    char digits[10];
    int n{0};
    do {
      digits[n++] = static_cast<char>('0' + v % 10);
      v /= 10;
    } while (v != 0);
    while (n > 0 && len_ < CAPACITY) {
      buf_[len_++] = digits[--n];
    }
    return *this;
  }

  /**
   * Append a signed integer in decimal.
   */
  LineBuffer& sint(int32_t v) {
    if (v < 0) {
      text("-");
      return uint(static_cast<uint32_t>(-static_cast<int64_t>(v)));
    }
    return uint(static_cast<uint32_t>(v));
  }

  /**
   * Append a float in fixed-point notation, rounded to the given decimal places.
   * Appends "?" for decimals outside [0, MAX_DECIMALS] and "ovf" if the scaled
   * value does not fit in a uint32_t.
   */
  LineBuffer& fixed(float v, int decimals) {
    if (std::isnan(v)) {
      return text("nan");
    }
    Guard(decimals >= 0 && decimals <= MAX_DECIMALS, text("?"));
    uint32_t scale{1};
    for (int i{0}; i < decimals; ++i) {
      scale *= 10;
    }
    Guard(std::isfinite(v) && std::fabs(v) * static_cast<float>(scale) < MAX_SCALED, text("ovf"));
    const float scaled{std::round(std::fabs(v) * static_cast<float>(scale))};
    const uint32_t whole{static_cast<uint32_t>(scaled)};
    if (v < 0.0f && whole != 0) {
      text("-");
    }
    uint(whole / scale);
    if (decimals > 0) {
      text(".");
      uint32_t frac{whole % scale};
      for (uint32_t place{scale / 10}; place > 0; place /= 10) {
        const char digit[2]{static_cast<char>('0' + frac / place), '\0'};
        text(digit);
        frac %= place;
      }
    }
    return *this;
  }

  LineBuffer& comma() { return text(","); }

  /**
   * Terminate the line, send it in one write() and clear the buffer.
   */
  void send() {
    text("\r\n");
    CONSOLE.write(reinterpret_cast<const uint8_t*>(buf_), len_);
    len_ = 0;
  }

 private:
  static constexpr size_t CAPACITY{128};

  /*
   * 10^9 is the largest power of ten that fits in a uint32_t.
   */
  static constexpr int MAX_DECIMALS{9};

  /*
   * Largest |v| * scale accepted by fixed(). Below UINT32_MAX with margin, since
   * float rounding near 2^32 could still overflow the cast.
   */
  static constexpr float MAX_SCALED{4.0e9f};

  char buf_[CAPACITY];
  size_t len_{0};
};

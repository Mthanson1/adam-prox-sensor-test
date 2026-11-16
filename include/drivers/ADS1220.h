/*
 * Driver for the TI ADS1220 24-bit delta-sigma ADC over SPI.
 *
 * The part is run in continuous-conversion mode: begin() writes the config and
 * issues START/SYNC once, then the ADC converts back to back. Each time a result
 * is ready DRDY goes low and stays low until the result is clocked out, so a
 * periodic ISR can poll it with readIfReady().
 *
 * SPI mode 1 (CPOL = 0, CPHA = 1), MSB first. The bus is assumed to belong to
 * this driver alone after begin(): readIfReady() runs from the sample ISR, so
 * nothing in loop() may use the same SPIClass.
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

class ADS1220 {
 public:
  /**
   * Input multiplexer, register 0 bits [7:4]. Named AINp_AINn.
   */
  enum class Mux : uint8_t {
    AIN0_AIN1 = 0x0,
    AIN0_AIN2 = 0x1,
    AIN0_AIN3 = 0x2,
    AIN1_AIN2 = 0x3,
    AIN1_AIN3 = 0x4,
    AIN2_AIN3 = 0x5,
    AIN1_AIN0 = 0x6,
    AIN3_AIN2 = 0x7,
    AIN0_AVSS = 0x8,
    AIN1_AVSS = 0x9,
    AIN2_AVSS = 0xA,
    AIN3_AVSS = 0xB,
    REF_DIV4 = 0xC,     // (VREFP - VREFN) / 4, monitor
    SUPPLY_DIV4 = 0xD,  // (AVDD - AVSS) / 4, monitor
    SHORTED = 0xE,      // AINp = AINn = mid-supply, offset calibration
  };

  /**
   * PGA gain, register 0 bits [3:1].
   */
  enum class Gain : uint8_t {
    X1 = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
    X32 = 5,
    X64 = 6,
    X128 = 7,
  };

  /**
   * Operating mode, register 1 bits [4:3]. Turbo doubles every data rate.
   */
  enum class Mode : uint8_t {
    NORMAL = 0,
    DUTY_CYCLE = 1,
    TURBO = 2,
  };

  /**
   * Data rate code, register 1 bits [7:5]. Names are the normal-mode rate in SPS;
   * TURBO doubles them, DUTY_CYCLE divides them by four.
   */
  enum class DataRate : uint8_t {
    SPS_20 = 0,
    SPS_45 = 1,
    SPS_90 = 2,
    SPS_175 = 3,
    SPS_330 = 4,
    SPS_600 = 5,
    SPS_1000 = 6,
  };

  /**
   * Voltage reference, register 2 bits [7:6].
   */
  enum class VRef : uint8_t {
    INTERNAL_2V048 = 0,
    EXT_REFP0_REFN0 = 1,
    EXT_AIN0_AIN3 = 2,
    AVDD = 3,
  };

  /**
   * Full device configuration. Defaults are the power-on register values except
   * for continuous conversion, which this driver always uses.
   */
  struct Config {
    Mux mux{Mux::AIN0_AIN1};
    Gain gain{Gain::X1};

    /**
     * Bypass the PGA. Only legal for gain 1, 2 or 4; extends the input range to
     * the supply rails at those gains.
     */
    bool pga_bypass{false};

    DataRate rate{DataRate::SPS_20};
    Mode mode{Mode::NORMAL};
    VRef vref{VRef::INTERNAL_2V048};

    /**
     * Reference voltage [V] used by toVolts(). Must match vref: 2.048 for the
     * internal reference, otherwise the measured external/supply value.
     */
    float vref_volts{2.048f};

    /**
     * Register 2 bits [5:0] (50/60 Hz FIR, low-side switch, IDAC current) and
     * register 3 bits [7:1] (IDAC routing, DRDY mode). Left raw: this rig does not
     * use the excitation sources. DRDYM (reg 3 bit 1) must stay 0 so results are
     * signalled on the dedicated DRDY pin.
     */
    uint8_t reg2_low{0x00};
    uint8_t reg3{0x00};
  };

  /**
   * @param spi      Bus the device is on; begin() calls spi.begin().
   * @param cs_pin   Chip select, active low.
   * @param drdy_pin Data-ready output from the ADS1220, active low.
   * @param sclk_hz  SPI clock. The datasheet minimum SCLK period is 150 ns.
   */
  ADS1220(SPIClass& spi, uint32_t cs_pin, uint32_t drdy_pin, uint32_t sclk_hz = 2000000)
      : spi_{spi}, cs_pin_{cs_pin}, drdy_pin_{drdy_pin}, settings_{sclk_hz, MSBFIRST, SPI_MODE1} {}

  /**
   * Set up the pins and bus, reset the device, then configure() it.
   *
   * @return false if configure() fails.
   */
  bool begin(const Config& config);

  /**
   * Write a configuration, read it back to confirm the part is present and
   * wired correctly, then (re)start continuous conversion. Call from the same
   * context that reads the device.
   *
   * @return false if the readback does not match or the config is invalid.
   */
  bool configure(const Config& config);

  /**
   * True while a conversion result is waiting to be read.
   */
  bool dataReady() const { return digitalRead(drdy_pin_) == LOW; }

  /**
   * Read the latest conversion if DRDY is asserted. ISR-safe.
   *
   * @param code Destination for the sign-extended 24-bit result, untouched if no
   *             result is ready.
   * @return true if a new result was written.
   */
  bool readIfReady(int32_t& code);

  /**
   * Convert a result code to the differential input voltage [V] using the
   * configured gain and reference.
   */
  float toVolts(int32_t code) const { return static_cast<float>(code) * volts_per_code_; }

  /**
   * Read the four configuration registers into regs[0..3].
   */
  void readRegisters(uint8_t regs[4]);

  /**
   * Full-scale code magnitude: results span [-2^23, 2^23 - 1].
   */
  static constexpr int32_t FULL_SCALE{1L << 23};

 private:
  /*
   * SPI commands (datasheet section 8.5.3).
   */
  static constexpr uint8_t CMD_RESET{0x06};
  static constexpr uint8_t CMD_START{0x08};
  static constexpr uint8_t CMD_POWERDOWN{0x02};
  static constexpr uint8_t CMD_RDATA{0x10};
  static constexpr uint8_t CMD_RREG{0x20};
  static constexpr uint8_t CMD_WREG{0x40};

  /*
   * Clocked out on DIN while reading. Not a valid command, so it is ignored.
   */
  static constexpr uint8_t DUMMY{0xFF};

  /*
   * Register 1 bit 2: continuous conversion.
   */
  static constexpr uint8_t REG1_CM{0x04};

  /*
   * Register 3 bit 0 is reserved and must be written 0.
   */
  static constexpr uint8_t REG3_RESERVED_MASK{0x01};

  /*
   * Reset settles in 50 us + 32 modulator clocks; wait comfortably longer.
   */
  static constexpr uint32_t RESET_DELAY_US{1000};

  /**
   * Pack a Config into the four register bytes.
   */
  static void encode(const Config& config, uint8_t regs[4]);

  void command(uint8_t cmd);
  void writeRegisters(const uint8_t regs[4]);

  SPIClass& spi_;
  const uint32_t cs_pin_;
  const uint32_t drdy_pin_;
  const SPISettings settings_;

  float volts_per_code_{0.0f};
};

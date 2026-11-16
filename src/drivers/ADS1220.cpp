#include "drivers/ADS1220.h"

#include "util/Guard.h"

bool ADS1220::begin(const Config& config) {
  pinMode(cs_pin_, OUTPUT);
  digitalWrite(cs_pin_, HIGH);
  pinMode(drdy_pin_, INPUT_PULLUP);
  spi_.begin();

  command(CMD_RESET);
  delayMicroseconds(RESET_DELAY_US);

  return configure(config);
}

bool ADS1220::configure(const Config& config) {
  GuardMsg(!config.pga_bypass || config.gain <= Gain::X4, false,
           "PGA bypass is only valid at gain 1, 2 or 4");
  GuardMsg((config.reg3 & 0x02) == 0, false, "DRDYM must be 0: DRDY pin is required");
  Guard(config.vref_volts > 0.0f, false);

  uint8_t regs[4];
  encode(config, regs);
  writeRegisters(regs);

  /*
   * A missing or miswired part reads back all 0x00 or all 0xFF, neither of which
   * matches a continuous-mode config, so this doubles as a presence check.
   */
  uint8_t readback[4];
  readRegisters(readback);
  for (int i{0}; i < 4; ++i) {
    GuardMsg(readback[i] == regs[i], false, "reg%d wrote 0x%02X read 0x%02X", i, regs[i],
             readback[i]);
  }

  const uint8_t gain{static_cast<uint8_t>(1U << static_cast<uint8_t>(config.gain))};
  volts_per_code_ = config.vref_volts / (static_cast<float>(gain) * FULL_SCALE);

  /*
   * START/SYNC also restarts a conversion already running, so the next result
   * is entirely from the new configuration.
   */
  command(CMD_START);
  return true;
}

bool ADS1220::readIfReady(int32_t& code) {
  if (!dataReady()) {
    return false;
  }

  /*
   * In continuous mode the result can be clocked out directly; no RDATA needed.
   */
  spi_.beginTransaction(settings_);
  digitalWrite(cs_pin_, LOW);
  const uint32_t b2{spi_.transfer(DUMMY)};
  const uint32_t b1{spi_.transfer(DUMMY)};
  const uint32_t b0{spi_.transfer(DUMMY)};
  digitalWrite(cs_pin_, HIGH);
  spi_.endTransaction();

  /*
   * Shift the 24-bit two's complement value to the top of the word, then
   * arithmetic-shift back down to sign-extend.
   */
  const uint32_t raw{(b2 << 16) | (b1 << 8) | b0};
  code = static_cast<int32_t>(raw << 8) >> 8;
  return true;
}

void ADS1220::readRegisters(uint8_t regs[4]) {
  spi_.beginTransaction(settings_);
  digitalWrite(cs_pin_, LOW);
  spi_.transfer(CMD_RREG | 0x03);  // start at reg 0, read 4
  for (int i{0}; i < 4; ++i) {
    regs[i] = spi_.transfer(DUMMY);
  }
  digitalWrite(cs_pin_, HIGH);
  spi_.endTransaction();
}

void ADS1220::encode(const Config& config, uint8_t regs[4]) {
  regs[0] =
      static_cast<uint8_t>((static_cast<uint8_t>(config.mux) << 4) |
                           (static_cast<uint8_t>(config.gain) << 1) | (config.pga_bypass ? 1 : 0));
  regs[1] = static_cast<uint8_t>((static_cast<uint8_t>(config.rate) << 5) |
                                 (static_cast<uint8_t>(config.mode) << 3) | REG1_CM);
  regs[2] =
      static_cast<uint8_t>((static_cast<uint8_t>(config.vref) << 6) | (config.reg2_low & 0x3F));
  regs[3] = static_cast<uint8_t>(config.reg3 & ~REG3_RESERVED_MASK);
}

void ADS1220::command(uint8_t cmd) {
  spi_.beginTransaction(settings_);
  digitalWrite(cs_pin_, LOW);
  spi_.transfer(cmd);
  digitalWrite(cs_pin_, HIGH);
  spi_.endTransaction();
}

void ADS1220::writeRegisters(const uint8_t regs[4]) {
  spi_.beginTransaction(settings_);
  digitalWrite(cs_pin_, LOW);
  spi_.transfer(CMD_WREG | 0x03);  // start at reg 0, write 4
  for (int i{0}; i < 4; ++i) {
    spi_.transfer(regs[i]);
  }
  digitalWrite(cs_pin_, HIGH);
  spi_.endTransaction();
}

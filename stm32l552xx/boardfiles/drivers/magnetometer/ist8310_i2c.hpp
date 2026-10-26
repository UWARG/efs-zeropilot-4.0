/**
 * @file ist8310_i2c.hpp
 * @brief I2C driver for the iSentek IST8310 three-axis magnetometer.
 */

#ifndef IST8310_DRIVER_IST8310_I2C_HPP_
#define IST8310_DRIVER_IST8310_I2C_HPP_

#include "stm32l5xx_hal.h"
#include "zp_error.h"

#include <cstdint>

class Ist8310
{
public:
  static constexpr uint8_t kDefaultAddress = 0x0E;
  static constexpr uint8_t kWhoAmIValue = 0x10;
  static constexpr float kMicroteslaPerLsb = 0.3F;

  struct Sample
  {
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    float x_microtesla;
    float y_microtesla;
    float z_microtesla;
    float heading_degrees;
  };

  explicit Ist8310(I2C_HandleTypeDef *i2c,
                   uint8_t address = kDefaultAddress);

  ZP_Error init();
  ZP_Error read(Sample *sample);

  ZP_Error getAddress(uint8_t *out_address) const;
  ZP_Error isReady(bool *out_ready) const;

private:
  enum Register_e : uint8_t
  {
    kWhoAmI = 0x00,
    kStatus1 = 0x02,
    kDataXLow = 0x03,
    kControl1 = 0x0A,
    kControl2 = 0x0B,
    kAverageControl = 0x41,
    kPulseDurationControl = 0x42,
  };

  static constexpr uint8_t kStatusDataReady = 0x01;
  static constexpr uint8_t kSingleMeasurement = 0x01;
  static constexpr uint8_t kSoftReset = 0x01;
  static constexpr uint8_t kAverage16 = 0x24;
  static constexpr uint8_t kNormalPulseDuration = 0xC0;

  ZP_Error probe();
  ZP_Error configure();
  ZP_Error waitForDataReady();
  ZP_Error writeRegister(uint8_t reg, uint8_t value);
  ZP_Error readRegister(uint8_t reg, uint8_t *value) const;
  ZP_Error readRegisters(uint8_t reg, uint8_t *data, uint16_t length) const;

  static ZP_Error combineBytes(uint8_t most_significant,
                               uint8_t least_significant,
                               int16_t *out_value);
  static ZP_Error calculateHeading(float x_microtesla, float y_microtesla,
                                   float &out_heading);

  I2C_HandleTypeDef *i2c_handle;
  uint8_t address;
  bool ready;
};
#endif  // IST8310_DRIVER_IST8310_I2C_HPP_

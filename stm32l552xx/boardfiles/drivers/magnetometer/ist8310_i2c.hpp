/**
 * @file ist8310_i2c.hpp
 * @brief I2C driver for the iSentek IST8310 three-axis magnetometer.
 */

#pragma once

#include "magnetometer_iface.hpp"
#include "stm32l5xx_hal.h"
#include "zp_error.h"

#include <cstdint>

class Ist8310 : public IMagnetometer
{
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x0E;
  static constexpr uint8_t WHO_AM_I_VALUE = 0x10;
  static constexpr float MICROTESLA_PER_LSB = 0.3F;

  explicit Ist8310(I2C_HandleTypeDef *i2c,
                   uint8_t address = DEFAULT_ADDRESS);

  ZP_Error init() override;
  ZP_Error readData(MagData_t &data) override;

  // Heading in degrees, 0 to 360, from the X and Y field in microtesla
  static ZP_Error calculateHeading(float x_microtesla, float y_microtesla,
                                   float &out_heading);

  // Called from the I2C interrupt callbacks
  void masterTxCpltCallback();
  void masterRxCpltCallback();
  void errorCallback();
  I2C_HandleTypeDef *getI2C();

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

  static constexpr uint8_t STATUS_DATA_READY = 0x01;
  static constexpr uint8_t SINGLE_MEASUREMENT = 0x01;
  static constexpr uint8_t SOFT_RESET = 0x01;
  static constexpr uint8_t AVERAGE16 = 0x24;
  static constexpr uint8_t NORMAL_PULSE_DIRECTION = 0xC0;

  static constexpr uint8_t DATA_BYTES = 6;

  enum ReadState_e
  {
    kIdle,
    kStartTx,
    kWaitConversion,
    kPointerTx,
    kDataRx,
  };

  ZP_Error probe();
  ZP_Error configure();
  ZP_Error startMeasurement();
  ZP_Error readMeasurement();
  void decode();
  ZP_Error writeRegister(uint8_t reg, uint8_t value);
  ZP_Error readRegister(uint8_t reg, uint8_t *value) const;
  ZP_Error readRegisters(uint8_t reg, uint8_t *data, uint16_t length) const;

  static ZP_Error combineBytes(uint8_t most_significant,
                               uint8_t least_significant,
                               int16_t *out_value);

  I2C_HandleTypeDef *i2c_handle;
  uint8_t address;
  bool ready;

  volatile ReadState_e state;
  volatile bool sample_ready;
  volatile uint32_t pending_error;
  uint32_t conversion_done_ms;
  uint32_t transfer_start_ms;
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[DATA_BYTES];
  MagData_t mag_data;
};
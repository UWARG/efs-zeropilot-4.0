/**
 * @file ist8310_i2c.cpp
 * @brief I2C driver for the iSentek IST8310 three-axis magnetometer.
 */
#include "ist8310_i2c.hpp"
#include "utils.h"

#include <cmath>

namespace
{
constexpr uint32_t kI2cTimeoutMilliseconds = 100U;
constexpr uint32_t kResetDelayMilliseconds = 50U;
constexpr uint32_t kDataReadyTimeoutMilliseconds = 20U;
constexpr float kRadiansToDegrees = 57.29577951308232F;
}  // namespace

Ist8310::Ist8310(I2C_HandleTypeDef *i2c, uint8_t address)
    : i2c_handle(i2c), address(address), ready(false)
{
}

ZP_Error Ist8310::init()
{
  ready = false;

  if (i2c_handle == nullptr) {
    return ZP_ERROR_NULLPTR;
  }

  ZP_Error error = probe();
  if (error != ZP_ERROR_OK) {
    return error;
  }

  error = writeRegister(kControl2, SOFT_RESET);
  if (error != ZP_ERROR_OK) {
    return error;
  }
  HAL_Delay(kResetDelayMilliseconds);

  uint8_t who_am_i = 0U;
  error = readRegister(kWhoAmI, &who_am_i);
  if (error != ZP_ERROR_OK) {
    return error;
  }
  if (who_am_i != WHO_AM_I_VALUE) {
    return ZP_ERROR_INVALID_DATA;
  }

  error = configure();
  if (error != ZP_ERROR_OK) {
    return error;
  }

  ready = true;
  return ZP_ERROR_OK;
}

ZP_Error Ist8310::read(Sample *sample)
{
  if (sample == nullptr) {
    return ZP_ERROR_NULLPTR;
  }
  if (!ready) {
    return ZP_ERROR_NOT_READY;
  }

  ZP_Error error = writeRegister(kControl1, SINGLE_MEASUREMENT);
  if (error != ZP_ERROR_OK) {
    ready = false;
    return error;
  }

  error = waitForDataReady();
  if (error != ZP_ERROR_OK) {
    ready = false;
    return error;
  }

  uint8_t data[6] = {};
  error = readRegisters(kDataXLow, data, sizeof(data));
  if (error != ZP_ERROR_OK) {
    ready = false;
    return error;
  }

  error = combineBytes(data[1], data[0], &sample->x_raw);
  if (error != ZP_ERROR_OK) {
    return error;
  }
  error = combineBytes(data[3], data[2], &sample->y_raw);
  if (error != ZP_ERROR_OK) {
    return error;
  }
  error = combineBytes(data[5], data[4], &sample->z_raw);
  if (error != ZP_ERROR_OK) {
    return error;
  }

  sample->x_microtesla = static_cast<float>(sample->x_raw) * MICROTESLA_PER_LSB;
  sample->y_microtesla = static_cast<float>(sample->y_raw) * MICROTESLA_PER_LSB;
  sample->z_microtesla = static_cast<float>(sample->z_raw) * MICROTESLA_PER_LSB;
  return calculateHeading(sample->x_microtesla, sample->y_microtesla,
                          sample->heading_degrees);
}

ZP_Error Ist8310::readData(MagData_t &data)
{
  Sample sample = {};
  const ZP_Error error = read(&sample);

  data.isNew = (error == ZP_ERROR_OK);
  if (data.isNew) {
    data.x = sample.x_microtesla;
    data.y = sample.y_microtesla;
    data.z = sample.z_microtesla;
    data.rawX = sample.x_raw;
    data.rawY = sample.y_raw;
    data.rawZ = sample.z_raw;
    data.timestamp = HAL_GetTick();
  }
  return error;
}

ZP_Error Ist8310::getAddress(uint8_t *out_address) const
{
  if (out_address == nullptr) {
    return ZP_ERROR_NULLPTR;
  }
  *out_address = address;
  return ZP_ERROR_OK;
}

ZP_Error Ist8310::isReady(bool *out_ready) const
{
  if (out_ready == nullptr) {
    return ZP_ERROR_NULLPTR;
  }
  *out_ready = ready;
  return ZP_ERROR_OK;
}

ZP_Error Ist8310::probe()
{
  constexpr uint8_t kCandidateAddresses[] = {0x0E, 0x0C, 0x0D, 0x0F};

  const uint8_t preferred_address = address;
  uint8_t who_am_i = 0U;
  if (readRegister(kWhoAmI, &who_am_i) == ZP_ERROR_OK &&
      who_am_i == WHO_AM_I_VALUE) {
    return ZP_ERROR_OK;
  }

  for (uint8_t candidate : kCandidateAddresses) {
    if (candidate == preferred_address) {
      continue;
    }

    address = candidate;

    who_am_i = 0U;
    if (readRegister(kWhoAmI, &who_am_i) == ZP_ERROR_OK &&
        who_am_i == WHO_AM_I_VALUE) {
      return ZP_ERROR_OK;
    }
  }

  address = DEFAULT_ADDRESS;
  return ZP_ERROR_EXT_API | ZP_ERROR_NACK;
}

ZP_Error Ist8310::configure()
{
  ZP_Error error = writeRegister(kAverageControl, AVERAGE16);
  if (error != ZP_ERROR_OK) {
    return error;
  }
  return writeRegister(kPulseDurationControl, NORMAL_PULSE_DIRECTION);
}

ZP_Error Ist8310::waitForDataReady()
{
  const uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < kDataReadyTimeoutMilliseconds)
  {
    uint8_t status = 0U;
    ZP_Error error = readRegister(kStatus1, &status);
    if (error != ZP_ERROR_OK) {
      return error;
    }

    if ((status & STATUS_DATA_READY) != 0U) {
      return ZP_ERROR_OK;
    }

    HAL_Delay(1U);
  }

  return ZP_ERROR_TIMEOUT;
}

ZP_Error Ist8310::writeRegister(uint8_t reg, uint8_t value)
{
  return halStatusToError(HAL_I2C_Mem_Write(
      i2c_handle, static_cast<uint16_t>(address) << 1U, reg,
      I2C_MEMADD_SIZE_8BIT, &value, 1U, kI2cTimeoutMilliseconds));
}

ZP_Error Ist8310::readRegister(uint8_t reg, uint8_t *value) const
{
  return readRegisters(reg, value, 1U);
}

ZP_Error Ist8310::readRegisters(uint8_t reg, uint8_t *data,
                                 uint16_t length) const
{
  if (data == nullptr) {
    return ZP_ERROR_NULLPTR;
  }
  if (length == 0U) {
    return ZP_ERROR_INVALID_ARG;
  }
  return halStatusToError(HAL_I2C_Mem_Read(
      i2c_handle, static_cast<uint16_t>(address) << 1U, reg,
      I2C_MEMADD_SIZE_8BIT, data, length, kI2cTimeoutMilliseconds));
}

ZP_Error Ist8310::combineBytes(uint8_t most_significant,
                               uint8_t least_significant, int16_t *out_value)
{
  if (out_value == nullptr) {
    return ZP_ERROR_NULLPTR;
  }
  *out_value = static_cast<int16_t>(
      (static_cast<uint16_t>(most_significant) << 8U) | least_significant);
  return ZP_ERROR_OK;
}

ZP_Error Ist8310::calculateHeading(float x_microtesla, float y_microtesla,
                                   float &out_heading)
{
  // Preserve the board-frame convention used by the original firmware.
  float heading = std::atan2(-y_microtesla, x_microtesla) * kRadiansToDegrees;
  if (heading < 0.0F) {
    heading += 360.0F;
  }

  out_heading = heading;
  return ZP_ERROR_OK;
}

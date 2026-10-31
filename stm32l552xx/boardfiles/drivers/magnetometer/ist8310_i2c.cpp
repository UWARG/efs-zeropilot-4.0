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
// Datasheet minimum is 6 ms with 16x averaging, with margin on top of that
constexpr uint32_t kConversionTimeMilliseconds = 10U;
constexpr uint32_t kTransferTimeoutMilliseconds = 100U;
constexpr float kRadiansToDegrees = 57.29577951308232F;
}  // namespace

Ist8310::Ist8310(I2C_HandleTypeDef *i2c, uint8_t address)
    : i2c_handle(i2c),
      address(address),
      ready(false),
      state(kIdle),
      sample_ready(false),
      pending_error(0U),
      conversion_done_ms(0U),
      transfer_start_ms(0U),
      tx_buffer{0},
      rx_buffer{0},
      mag_data{}
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

  state = kIdle;
  sample_ready = false;
  pending_error = 0U;
  mag_data = {};
  ready = true;
  return ZP_ERROR_OK;
}

ZP_Error Ist8310::readData(MagData_t &data)
{
  if (!ready) {
    mag_data.isNew = false;
    data = mag_data;
    return ZP_ERROR_NOT_READY;
  }

  ZP_Error error(pending_error);
  pending_error = 0U;

  if (sample_ready) {
    sample_ready = false;
    decode();
  } else {
    mag_data.isNew = false;
  }

  const uint32_t now = HAL_GetTick();

  switch (state)
  {
    case kIdle:
      error |= startMeasurement();
      break;

    case kWaitConversion:
      if (static_cast<int32_t>(now - conversion_done_ms) >= 0) {
        error |= readMeasurement();
      }
      break;

    default:
      // A transfer is in flight. Give up on it if the bus went quiet.
      if ((now - transfer_start_ms) > kTransferTimeoutMilliseconds &&
          i2c_handle->State == HAL_I2C_STATE_READY) {
        state = kIdle;
        error |= ZP_ERROR_TIMEOUT;
      }
      break;
  }

  data = mag_data;
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

ZP_Error Ist8310::startMeasurement()
{
  tx_buffer[0] = kControl1;
  tx_buffer[1] = SINGLE_MEASUREMENT;
  transfer_start_ms = HAL_GetTick();
  state = kStartTx;

  const ZP_Error status = halStatusToError(HAL_I2C_Master_Transmit_IT(
      i2c_handle, static_cast<uint16_t>(address) << 1U, tx_buffer, 2U));
  if (status != ZP_ERROR_OK) {
    state = kIdle;
  }
  return status;
}

ZP_Error Ist8310::readMeasurement()
{
  // Point the register pointer at the first data byte, then read the data
  tx_buffer[0] = kDataXLow;
  transfer_start_ms = HAL_GetTick();
  state = kPointerTx;

  const ZP_Error status = halStatusToError(HAL_I2C_Master_Transmit_IT(
      i2c_handle, static_cast<uint16_t>(address) << 1U, tx_buffer, 1U));
  if (status != ZP_ERROR_OK) {
    state = kIdle;
  }
  return status;
}

void Ist8310::masterTxCpltCallback()
{
  switch (state)
  {
    case kStartTx:
      conversion_done_ms = HAL_GetTick() + kConversionTimeMilliseconds;
      state = kWaitConversion;
      break;

    case kPointerTx:
      state = kDataRx;
      if (HAL_I2C_Master_Receive_IT(i2c_handle,
                                    static_cast<uint16_t>(address) << 1U,
                                    rx_buffer, DATA_BYTES) != HAL_OK) {
        state = kIdle;
        pending_error |= (ZP_ERROR_EXT_API | ZP_ERROR_FAIL).raw();
      }
      break;

    default:
      break;
  }
}

void Ist8310::masterRxCpltCallback()
{
  if (state == kDataRx) {
    sample_ready = true;
  }
  state = kIdle;
}

void Ist8310::errorCallback()
{
  // The bus can be shared, so only react when one of our transfers is in flight
  if (state != kStartTx && state != kPointerTx && state != kDataRx) {
    return;
  }

  if ((HAL_I2C_GetError(i2c_handle) & HAL_I2C_ERROR_AF) != 0U) {
    pending_error |= (ZP_ERROR_EXT_API | ZP_ERROR_NACK).raw();
  } else {
    pending_error |= (ZP_ERROR_EXT_API | ZP_ERROR_FAIL).raw();
  }
  state = kIdle;
}

void Ist8310::decode()
{
  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;

  ZP_Error error = combineBytes(rx_buffer[1], rx_buffer[0], &raw_x);
  error |= combineBytes(rx_buffer[3], rx_buffer[2], &raw_y);
  error |= combineBytes(rx_buffer[5], rx_buffer[4], &raw_z);
  if (error != ZP_ERROR_OK) {
    pending_error |= error.raw();
    return;
  }

  mag_data.rawX = raw_x;
  mag_data.rawY = raw_y;
  mag_data.rawZ = raw_z;
  mag_data.x = static_cast<float>(raw_x) * MICROTESLA_PER_LSB;
  mag_data.y = static_cast<float>(raw_y) * MICROTESLA_PER_LSB;
  mag_data.z = static_cast<float>(raw_z) * MICROTESLA_PER_LSB;
  mag_data.timestamp = HAL_GetTick();
  mag_data.isNew = true;
}

I2C_HandleTypeDef *Ist8310::getI2C()
{
  return i2c_handle;
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

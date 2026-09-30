/**
 * @file ist8310_i2c.cpp
 * @brief I2C driver for the iSentek IST8310 three-axis magnetometer.
 */
#include "ist8310_i2c.hpp"

#include <cmath>

namespace
{
constexpr uint32_t kI2cTimeoutMilliseconds = 100U;
constexpr uint32_t kResetDelayMilliseconds = 50U;
constexpr uint32_t kDataReadyTimeoutMilliseconds = 20U;
constexpr float kRadiansToDegrees = 57.29577951308232F;
}  // namespace

Ist8310::Ist8310(I2C_HandleTypeDef *i2c, uint8_t address)
    : i2c_(i2c), address_(address), ready_(false)
{
}

bool Ist8310::init()
{
  ready_ = false;

  if ((i2c_ == nullptr) || !probe())
  {
    return false;
  }

  if (!writeRegister(kControl2, kSoftReset))
  {
    return false;
  }

  HAL_Delay(kResetDelayMilliseconds);

  uint8_t who_am_i = 0U;
  if (!readRegister(kWhoAmI, who_am_i) || (who_am_i != kWhoAmIValue))
  {
    return false;
  }

  if (!configure())
  {
    return false;
  }

  ready_ = true;
  return true;
}

bool Ist8310::read(Sample &sample)
{
  if (!ready_ || !writeRegister(kControl1, kSingleMeasurement))
  {
    ready_ = false;
    return false;
  }

  if (!waitForDataReady())
  {
    ready_ = false;
    return false;
  }

  uint8_t data[6] = {};
  if (!readRegisters(kDataXLow, data, sizeof(data)))
  {
    ready_ = false;
    return false;
  }

  sample.x_raw = combineBytes(data[1], data[0]);
  sample.y_raw = combineBytes(data[3], data[2]);
  sample.z_raw = combineBytes(data[5], data[4]);

  sample.x_microtesla = static_cast<float>(sample.x_raw) * kMicroteslaPerLsb;
  sample.y_microtesla = static_cast<float>(sample.y_raw) * kMicroteslaPerLsb;
  sample.z_microtesla = static_cast<float>(sample.z_raw) * kMicroteslaPerLsb;
  sample.heading_degrees = calculateHeading(sample.x_microtesla,
                                            sample.y_microtesla);

  return true;
}

uint8_t Ist8310::address() const
{
  return address_;
}

bool Ist8310::isReady() const
{
  return ready_;
}

bool Ist8310::probe()
{
  constexpr uint8_t kCandidateAddresses[] = {0x0E, 0x0C, 0x0D, 0x0F};

  const uint8_t preferred_address = address_;
  uint8_t who_am_i = 0U;
  if (readRegister(kWhoAmI, who_am_i) && (who_am_i == kWhoAmIValue))
  {
    return true;
  }

  for (uint8_t candidate : kCandidateAddresses)
  {
    if (candidate == preferred_address)
    {
      continue;
    }

    address_ = candidate;

    who_am_i = 0U;
    if (readRegister(kWhoAmI, who_am_i) && (who_am_i == kWhoAmIValue))
    {
      return true;
    }
  }

  address_ = kDefaultAddress;
  return false;
}

bool Ist8310::configure()
{
  return writeRegister(kAverageControl, kAverage16) &&
         writeRegister(kPulseDurationControl, kNormalPulseDuration);
}

bool Ist8310::waitForDataReady()
{
  const uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < kDataReadyTimeoutMilliseconds)
  {
    uint8_t status = 0U;
    if (!readRegister(kStatus1, status))
    {
      return false;
    }

    if ((status & kStatusDataReady) != 0U)
    {
      return true;
    }

    HAL_Delay(1U);
  }

  return false;
}

bool Ist8310::writeRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(i2c_, static_cast<uint16_t>(address_) << 1U, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           kI2cTimeoutMilliseconds) == HAL_OK;
}

bool Ist8310::readRegister(uint8_t reg, uint8_t &value) const
{
  return HAL_I2C_Mem_Read(i2c_, static_cast<uint16_t>(address_) << 1U, reg,
                          I2C_MEMADD_SIZE_8BIT, &value, 1U,
                          kI2cTimeoutMilliseconds) == HAL_OK;
}

bool Ist8310::readRegisters(uint8_t reg, uint8_t *data,
                            uint16_t length) const
{
  return HAL_I2C_Mem_Read(i2c_, static_cast<uint16_t>(address_) << 1U, reg,
                          I2C_MEMADD_SIZE_8BIT, data, length,
                          kI2cTimeoutMilliseconds) == HAL_OK;
}

int16_t Ist8310::combineBytes(uint8_t most_significant,
                              uint8_t least_significant)
{
  return static_cast<int16_t>((static_cast<uint16_t>(most_significant) << 8U) |
                              least_significant);
}

float Ist8310::calculateHeading(float x_microtesla, float y_microtesla)
{
  // Preserve the board-frame convention used by the original firmware.
  float heading = std::atan2(-y_microtesla, x_microtesla) * kRadiansToDegrees;
  if (heading < 0.0F)
  {
    heading += 360.0F;
  }

  return heading;
}

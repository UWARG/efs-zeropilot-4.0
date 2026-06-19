#include "drivers.hpp"
#include "managers.hpp"
#include "zp_params.hpp"

ZP_ERROR_e initModel()
{
  if (ZP_PARAM::init() != ZP_ERROR_OK) Error_Handler();
  if (initDrivers() != ZP_ERROR_OK) Error_Handler();
  if (initManagers() != ZP_ERROR_OK) Error_Handler();
  return ZP_ERROR_OK;
}
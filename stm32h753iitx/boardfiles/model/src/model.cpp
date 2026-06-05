#include "drivers.hpp"
#include "managers.hpp"
#include "zp_params.hpp"

void initModel()
{
  initDrivers();
  ZP_PARAM::init();
  initManagers();
}

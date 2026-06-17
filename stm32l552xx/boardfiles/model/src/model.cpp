#include "drivers.hpp"
#include "managers.hpp"
#include "zp_params.hpp"

void initModel()
{
  ZP_PARAM::init();
  initDrivers();
  initManagers();
}

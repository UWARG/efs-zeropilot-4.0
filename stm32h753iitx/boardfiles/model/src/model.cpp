#include "drivers.hpp"
#include "managers.hpp"
#include "model.hpp"
#include "zp_params.hpp"
#include "zp_bit.hpp"

void initModel()
{
  initSystemUtils();
  (void)ZP_BIT::init(systemUtilsHandle);

  (void)ZP_BIT::report(ZP_BIT_ID::PARAM_TABLE_INIT, ZP_PARAM::init());

  initDrivers();

  initManagers();
}

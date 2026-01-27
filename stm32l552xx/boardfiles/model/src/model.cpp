#include "drivers.hpp"
#include "managers.hpp"
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

  ZP_ERROR_e initModel()
  {
    ZP_ERROR_e error = ZP_ERROR_OK;

    error = initDrivers();
    if (error != ZP_ERROR_OK) {
      return error;
    }

    error = initManagers();
    if (error != ZP_ERROR_OK) {
      return error;
    }

    return ZP_ERROR_OK;
  }

#ifdef __cplusplus
}
#endif

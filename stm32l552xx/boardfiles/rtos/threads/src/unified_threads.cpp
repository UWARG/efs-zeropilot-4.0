#include "unified_threads.hpp"

#ifdef __cplusplus
extern "C" {
#endif

  ZP_ERROR_e initThreads()
  {
    ZP_ERROR_e error = ZP_ERROR_OK;

    error = startUpInitThreads();
    if (error != ZP_ERROR_OK) {
      return error;
    }

    error = amInitThreads();
    if (error != ZP_ERROR_OK) {
      return error;
    }

    error = smInitThreads();
    if (error != ZP_ERROR_OK) {
      return error;
    }

    error = tmInitThreads();
    if (error != ZP_ERROR_OK) {
      return error;
    }

    return ZP_ERROR_OK;
  }

#ifdef __cplusplus
}
#endif

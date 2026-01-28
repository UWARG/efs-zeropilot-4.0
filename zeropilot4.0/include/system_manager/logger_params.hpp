#pragma once

#define SD_CARD_LOGGING
//#define SWO_LOGGING

#if defined(SD_CARD_LOGGING) && defined(SWO_LOGGING)
  #error Only one can be defined
#elif !defined(SD_CARD_LOGGING) && !defined(SWO_LOGGING)
  #error Define a Logging interface
#endif

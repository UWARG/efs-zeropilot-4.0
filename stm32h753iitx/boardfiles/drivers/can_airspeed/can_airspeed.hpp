#pragma once

#include <cstdint>
#include "canard.h"
#include "can_listener_iface.hpp"
#include "rangefinder_iface.hpp"
#include "systemutils_iface.hpp"   // getCurrentTimestampMs()
#include "zp_error.h"   

static constexpr uint16_t AIRSPEED_TIMEOUT = 500;
static constexpr uint16_t AIRSPEED_MSG_SIZE = 4; // 2 bytes for indicated_airspeed, 2 bytes for indicated_airspeed_variance
// CAN FRAME for fun:
// airspeed byte 0, airspeed byte 1, airspeed var byte 0, airspeed var byte 1, tail byte

// 1. Create airspeed data struct

typedef struct {
    float speed; // The detected speed
    float variance; // The detected variance

    bool isValid; 
    bool isNew; 
} AirspeedData_t;

// TODO:
/**
 * Just look at like the existing can opflow and get things down
 * But basically I'm just decoding messages from one end to antoher
 * 
 * And improviding a module / "driver" that does that, which also runs on same
 * STM32, so the integration is like
 * Holybro sensor w/ own mcu inside + sensor, all data read through -> send through its can transceiver to our ZP board w/ our own can transceiver, sends signal to MCU slowly, then the can controller will consume it, then it will call this driver and then it will decode it and stuff
 * Tomorrow just start copying over stuff to get a feel for what the driver should look like, and then try to get it in a good spot for saturday to just hammer through it
 */

  /**
  * Questions:
  * 
  * 1. How does can node integrate with this, and how do i integrate this basic driver into cancontroller?
  * 2. How will CanController work and how will it subscribe to the outputs from the CAN thing? Since i imagine its like, it receives the data from the can transceiver by reading stuff and then once it gets the message and accepts it, it then calls the correct driver right?
  * 3. So now its like integration + building the thing, which I don't really know about either
  */
class CanAirspeed : public ICanListener {
    public: 
        CanAirspeed(ISystemUtils *systemUtils);

        // ICanListener
        ZP_Error init();
        ZP_Error onTransfer(CanardRxTransfer *transfer) override; 
        ZP_Error readAirspeedData(AirspeedData_t &data);
    private:
        ISystemUtils *systemUtils;
        ZP_Error subscribeStatus = ZP_ERROR_OK; 

        AirspeedData_t airspeedData = {};
        uint32_t lastAirSpeedMs = 0;

        ZP_Error decodeAirspeed(CanardRxTransfer *transfer);
};
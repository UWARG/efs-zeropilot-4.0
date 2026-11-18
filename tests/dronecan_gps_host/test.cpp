#include "dronecan_gps.hpp"
#include "uavcan.equipment.gnss.Fix2.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
bool lockBusy=false, allocationFails=false, decodeFails=false;
uint32_t nowMs=0;
uavcan_equipment_gnss_Fix2 decoded{};
int main() {
    CANController controller;
    DroneCanGPS gps(&controller);
    GpsData_t out{};
    assert(gps.init()==ZP_ERROR_OK && controller.listener==&gps);
    assert(gps.readData(out)==ZP_ERROR_NOT_READY);
    assert(gps.onTransfer(nullptr)==ZP_ERROR_NULLPTR);
    CanardRxTransfer t{1063,0,42};
    decodeFails=true; assert(gps.onTransfer(&t)==ZP_ERROR_PARSE); decodeFails=false;
    decoded.status=3; decoded.latitude_deg_1e8=4350000000LL; decoded.longitude_deg_1e8=-8050000000LL;
    decoded.height_msl_mm=123456; decoded.ned_velocity[0]=3; decoded.ned_velocity[1]=4; decoded.sats_used=12;
    assert(gps.onTransfer(&t)==ZP_ERROR_OK);
    assert(gps.readData(out)==ZP_ERROR_OK && out.isNew);
    assert(std::abs(out.latitude-43.5f)<0.00001f && std::abs(out.longitude+80.5f)<0.00001f);
    assert(std::abs(out.altitude-123.456f)<0.001f && out.groundSpeed==500 && out.numSatellites==12);
    assert(gps.readData(out)==ZP_ERROR_OK && !out.isNew);
    t.source_node_id=43; assert(gps.onTransfer(&t)==ZP_ERROR_INVALID_DATA);
    nowMs=2001; assert(gps.readData(out)==ZP_ERROR_TIMEOUT && !out.isNew);
    assert(gps.onTransfer(&t)==ZP_ERROR_OK);
    decoded.status=2; assert(gps.onTransfer(&t)==ZP_ERROR_OK); assert(gps.readData(out)==ZP_ERROR_OK && out.altitude==INVALID_ALTITUDE);
    decoded.status=0; assert(gps.onTransfer(&t)==ZP_ERROR_INVALID_DATA); assert(gps.readData(out)==ZP_ERROR_NOT_READY);
    decoded.status=3; decoded.latitude_deg_1e8=9000000001LL; assert(gps.onTransfer(&t)==ZP_ERROR_INVALID_DATA);
    decoded.latitude_deg_1e8=0; decoded.ned_velocity[0]=std::numeric_limits<float>::quiet_NaN(); assert(gps.onTransfer(&t)==ZP_ERROR_INVALID_DATA);
    lockBusy=true; assert(gps.readData(out)==ZP_ERROR_BUSY); lockBusy=false;
    DroneCanGPS nullGps(nullptr); assert(nullGps.init()==ZP_ERROR_NULLPTR);
    allocationFails=true; DroneCanGPS noMutex(&controller); assert(noMutex.init()==ZP_ERROR_RESOURCE_UNAVAILABLE); allocationFails=false;
    controller.fail=true; DroneCanGPS noSubscription(&controller); assert(noSubscription.init()==ZP_ERROR_MEMORY_OVERFLOW);
    std::cout << "GPS host behavior checks passed\n";
}

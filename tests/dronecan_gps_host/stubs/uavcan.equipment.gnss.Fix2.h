#pragma once
#include "can_controller.hpp"
#define UAVCAN_EQUIPMENT_GNSS_FIX2_ID 1063
#define UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE 0xCA41E7000F37435FULL
#define UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX 2
#define UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX 3
#define UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC 2
struct uavcan_equipment_gnss_Fix2 {
    int64_t latitude_deg_1e8=0, longitude_deg_1e8=0;
    int32_t height_msl_mm=0;
    float ned_velocity[3]{};
    uint8_t status=0, sats_used=0, gnss_time_standard=0;
    struct { uint64_t usec=0; } gnss_timestamp;
};
extern uavcan_equipment_gnss_Fix2 decoded;
extern bool decodeFails;
inline bool uavcan_equipment_gnss_Fix2_decode(const CanardRxTransfer*,uavcan_equipment_gnss_Fix2* out) { *out=decoded; return decodeFails; }

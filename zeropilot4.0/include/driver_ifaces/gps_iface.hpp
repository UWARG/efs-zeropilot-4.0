#pragma once

#include <cstdint>

#define INVALID_TRACK_ANGLE -1
#define INVALID_DATA -2

typedef enum {
    NMEA,
    UBX
} GpsProtocol_t;

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} GpsTime_t;

/*
    Positive lat -> N else S
    Positive lon -> E else W
    Invalid track angle is indicated by INVALID_TRACK_ANGLE = -1
*/
typedef struct {
    GpsTime_t time;
    float latitude; // deg
    float longitude; // deg
    float groundSpeed; // cm/s
    uint8_t numSatellites;
    float altitude; // m, -1 if not valid
    float trackAngle;
    bool isNew;
    float vx; // m/s
    float vy; // m/s
    float vz; // m/s
} GpsData_t;


class IGPS {
protected:
    IGPS() = default;

public:
    virtual ~IGPS() = default;

    virtual GpsData_t readData() = 0;
};

#pragma once

typedef struct {
    float altitude;         // m
    float rateOfClimb;      // m/s
    long double latitude;   // Decimal degrees
    float latitudeSpeed;    // m/s
    long double longitude;  // Decimal degrees
    float longitudeSpeed;   // m/s
} InputSF;

typedef struct {
    float roll;             // degrees
    float pitch;            // degrees
    float throttle;         // %
} OutputFBWA;

typedef struct {
    float roll;             // degrees
    float speed;            // % (scales the max speed in AM)
    int altitude_rate;      // m/s
} OutputFBWB;
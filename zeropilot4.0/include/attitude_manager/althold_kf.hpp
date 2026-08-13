#include "mathutils_iface.hpp"

enum AltholdStatesIdx_e {
    altholdStateAltIdx,
    altholdStateVelIdx,
    altholdStateBiasAccelIdx,
    altholdStateBiasBaroIdx,
    altholdStateCount
};

typedef struct {
    float processNoiseAccel;
    float processNoiseBiasAccel;
    float processNoiseBiasBaro;
    float processNoiseTerrainAlt; // Lower to around 5e-5 if flying over flat terrain, increase to around 5e-3 if flying somewhere with steep terrain changes
    float measNoiseBarometer;
    float measNoiseRangefinder;
    float measNoiseGPSAlt;
    float measNoiseGPSVel;
} AltholdConfig_t;

class AltholdKF {
public:
    AltholdKF(IMathUtils* mathUtils);
    void init(AltholdConfig_t config, float initStates[4]);
    void predict(float u, float dt);
    void updateBarometer(float altitude);
    void updateGPSAlt(float altitude);
    void updateGPSVel(float verticalVelocity);
    void setBaroBiasEnabled(bool enabled);
    float getEstimatedAltitude();
    float getEstimatedVerticalVel();

private:
    IMathUtils* math;
    
    AltholdConfig_t config;
    /*
    z: altitude from takeoff 
    vz: vertical velocity
    b_accel: accelerometer bias
    b_baro: barometer bias
    */
    float states[altholdStateCount] = {};

    // NOLINTBEGIN(readability-identifier-naming)
    float P[altholdStateCount * altholdStateCount] = {}; // Covariance matrix

    // H
    const float H_BARO[altholdStateCount] = {1.0f, 0, 0, 1.0f};
    const float H_GPS_ALT[altholdStateCount] = {1.0f, 0, 0, 0};
    const float H_GPS_VEL[altholdStateCount] = {0, 1.0f, 0, 0};

    float F[altholdStateCount * altholdStateCount] = {}; // State transition matrix
    float B[altholdStateCount] = {}; // Control input matrix
    float Q[altholdStateCount * altholdStateCount] = {}; // Process noise covariance
    // NOLINTEND(readability-identifier-naming)
    
    float dtPrev;

    bool baroBiasEnabled = true;

    uint8_t rangefinderRejectCount = 0;
    uint8_t barometerRejectCount = 0;
    uint8_t gpsAltRejectCount = 0;
    uint8_t gpsVelRejectCount = 0;
    
    void rebuildFBQ(float dt);
    void update(float measurement, const float *H, float R);

    uint16_t rangeUpdateCount = 0;
    uint16_t baroUpdateCount = 0;
    uint16_t gpsUpdateCount = 0;


};

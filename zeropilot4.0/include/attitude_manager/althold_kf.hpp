#include "mathutils_iface.hpp"

enum AltholdStatesIdx_e {
    ALTHOLD_STATE_ALT_IDX,
    ALTHOLD_STATE_VEL_IDX,
    ALTHOLD_STATE_BIAS_ACCEL_IDX,
    ALTHOLD_STATE_BIAS_BARO_IDX,
    ALTHOLD_STATE_COUNT
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
    float states[ALTHOLD_STATE_COUNT] = {};

    // NOLINTBEGIN(readability-identifier-naming)
    float P[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT] = {}; // Covariance matrix

    // H
    const float H_BARO[ALTHOLD_STATE_COUNT] = {1.0f, 0, 0, 1.0f};
    const float H_GPS_ALT[ALTHOLD_STATE_COUNT] = {1.0f, 0, 0, 0};
    const float H_GPS_VEL[ALTHOLD_STATE_COUNT] = {0, 1.0f, 0, 0};

    float F[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT] = {}; // State transition matrix
    float B[ALTHOLD_STATE_COUNT] = {}; // Control input matrix
    float Q[ALTHOLD_STATE_COUNT * ALTHOLD_STATE_COUNT] = {}; // Process noise covariance
    // NOLINTEND(readability-identifier-naming)
    
    float dtPrev;

    bool baroBiasEnabled = true;
    
    static constexpr float NIS_GATE = 16.0f; // 4 sigma
    static constexpr uint8_t MAX_CONSECUTIVE_REJECTS = 20;

    uint8_t barometerRejectCount = 0;
    uint8_t gpsAltRejectCount = 0;
    uint8_t gpsVelRejectCount = 0;

    void rebuildFBQ(float dt);
    void update(float measurement, const float *H, float R, uint8_t &rejectCount);
};

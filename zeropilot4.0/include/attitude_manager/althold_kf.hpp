#include "mathutils_iface.hpp"

typedef struct {
    float altitude;
    float verticalVelocity;
    float biasAccel;
    float biasBaro;
    float terrainAlt;
} AltholdStates;

typedef struct {
    float processNoiseAccel;
    float processNoiseBiasAccel;
    float processNoiseBiasBaro;
    float processNoiseTerrainAlt; // Lower to around 5e-5 if flying over flat terrain, increase to around 5e-3 if flying somewhere with steep terrain changes
    float measNoiseBarometer;
    float measNoiseRangefinder;
    float measNoiseGPSAlt;
    float measNoiseGPSVel;
} AltholdConfig;

class AltholdKF {
public:
    AltholdKF(IMathUtils* mathUtils);
    void init(AltholdConfig config, float initStates[4]);
    void predict(float u, float dt);
    void updateBarometer(float altitude);
    void updateGPSAlt(float altitude);
    void updateGPSVel(float verticalVelocity);
    void setBaroBiasEnabled(bool enabled);
    float getEstimatedAltitude();

private:
    IMathUtils* math;
    
    AltholdConfig config;
    static constexpr uint16_t STATE_SIZE = 4;
    /*
    z: altitutude from takeoff 
    vz: vertical velocity
    b_accel: accelerometer bias
    b_baro: barometer bias
    */
    float states[STATE_SIZE] = {};

    float P[STATE_SIZE * STATE_SIZE] = {}; // Covariance matrix

    // H
    const float measMatBaro[STATE_SIZE] = {1.0f, 0, 0, 1.0f};
    const float measMatGPSAlt[STATE_SIZE] = {1.0f, 0, 0, 0};
    const float measMatGPSVel[STATE_SIZE] = {0, 1.0f, 0, 0};

    float F[STATE_SIZE * STATE_SIZE] = {}; // State transition matrix
    float B[STATE_SIZE] = {}; // Control input matrix
    float Q[STATE_SIZE * STATE_SIZE] = {}; // Process noise covariance

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
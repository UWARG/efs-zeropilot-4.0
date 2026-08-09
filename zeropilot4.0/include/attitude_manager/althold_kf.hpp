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
    void init(AltholdConfig config, float initStates[5]);
    void predict(float u, float dt);
    void updateRangefinder(float altitude);
    void updateBarometer(float altitude);
    void updateGPSAlt(float altitude);
    void updateGPSVel(float verticalVelocity);
    void setBaroBiasEnabled(bool enabled);
    void setRangefinderBiasEnabled(bool enabled);
    void setGPSBiasEnabled(bool enabled);
    float getEstimatedAltitude();
    float getAltTerrain();

private:
    IMathUtils* math;
    
    AltholdConfig config;
    static constexpr uint16_t STATE_SIZE = 5;
    /*
        z: altitutude from takeoff 
        vz: vertical velocity
        b_accel: accelerometer bias
        b_baro: barometer bias
        h_terrain: terrain height relative to takeoff point 
                   (for rangefinder: rangefinder measurement = z - h_terrain)
    */
    float states[STATE_SIZE] = {};

    float P[STATE_SIZE * STATE_SIZE] = {}; // Covariance matrix

    // H
    const float measMatRangefinder[STATE_SIZE] = {1.0f, 0, 0, 0, -1.0f};
    const float measMatBaro[STATE_SIZE] = {1.0f, 0, 0, 1.0f, 0};
    const float measMatGPSAlt[STATE_SIZE] = {1.0f, 0, 0, 0, 0};
    const float measMatGPSVel[STATE_SIZE] = {0, 1.0f, 0, 0, 0};

    float F[STATE_SIZE * STATE_SIZE] = {}; // State transition matrix
    float B[STATE_SIZE] = {}; // Control input matrix
    float Q[STATE_SIZE * STATE_SIZE] = {}; // Process noise covariance

    float dtPrev;
    // float dtMin;
    // float dtMax;

    bool baroBiasEnabled = true;

    uint8_t activeSensorCount = 0; // *if < 2, then system is unobservable, send critical message*

    uint8_t rangefinderRejectCount = 0;
    uint8_t barometerRejectCount = 0;
    uint8_t gpsAltRejectCount = 0;
    uint8_t gpsVelRejectCount = 0;
    
    void rebuildFBQ(float dt);
    void update(float measurement,const float *H, float R, uint8_t &rejectCount, void (AltholdKF::*gatingWrapper)(uint8_t &rejectCount));
    
    void rangefinderGatingWrapper(uint8_t &rangefinderRejectCount);


    uint16_t rangeUpdateCount = 0;
    uint16_t baroUpdateCount = 0;
    uint16_t gpsUpdateCount = 0;


};
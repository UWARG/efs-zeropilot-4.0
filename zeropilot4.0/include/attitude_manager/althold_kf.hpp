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
    float processNoiseTerrainAlt;
    float measNoiseBarometer;
    float measNoiseRangefinder;
    float measNoiseGPSAlt;
    float measNoiseGPSVel;
} AltholdConfig;

class AltholdKF {
public:
    AltholdKF(IMathUtils* mathUtils);
    void init(AltholdConfig config);
    void predict(float u, float dt);
    void updateRangefinder(float altitude);
    void updateBarometer(float altitude);
    void updateGPSAlt(float altitude);
    void updateGPSVel(float verticalVelocity);
    float getEstimatedAltitude();

private:
    IMathUtils* math;

    static constexpr uint16_t STATE_SIZE = 5;
    float states[STATE_SIZE];
    AltholdConfig config;

    float P[STATE_SIZE * STATE_SIZE]; // Covariance matrix

    // H
    const float measMatRangefinder[STATE_SIZE] = {1.0f, 0, 0, 0, -1.0f};
    const float measMatBaro[STATE_SIZE] = {1.0f, 0, 0, 1.0f, 0};
    const float measMatGPSAlt[STATE_SIZE] = {1.0f, 0, 0, 0, 0};
    const float measMatGPSVel[STATE_SIZE] = {0, 1.0f, 0, 0, 0};

    float F[STATE_SIZE * STATE_SIZE]; // State transition matrix
    float B[STATE_SIZE]; // Control input matrix
    float Q[STATE_SIZE * STATE_SIZE]; // Process noise covariance

    uint32_t dtPrev;
    uint32_t dtMin;
    uint32_t dtMax;

    uint8_t rangefinderRejectCount = 0;
    uint8_t barometerRejectCount = 0;
    uint8_t gpsAltRejectCount = 0;
    uint8_t gpsVelRejectCount = 0;
    
    void rebuildFBQ(float dt);
    void update(float measurement,const float *H, float R, uint8_t &rejectCount, void (AltholdKF::*gatingWrapper)(uint8_t &rejectCount));
    
    void rangefinderGatingWrapper(uint8_t &rangefinderRejectCount);
    


};
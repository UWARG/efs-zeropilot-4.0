#include "system_manager.hpp"

#define SOC_IDLE_MODE true
#define SOC_CHARGE_DISCHARGE_MODE false

typedef struct {
    float voltage;
    float soc;
} VoltageToSoc_t;

typedef struct {
    uint8_t socPercentage;
    int32_t timeRemaining;
} StateOfCharge_t;

static constexpr float V_MAX = 4.20f;
static constexpr float V_MIN = 3.50f;
static constexpr VoltageToSoc_t SOC_LUT[] = {
    {4.20f, 100.0f},
    {4.15f, 95.0f},
    {4.11f, 90.0f},
    {4.08f, 85.0f},
    {4.02f, 80.0f},
    {3.98f, 75.0f},
    {3.95f, 70.0f},
    {3.91f, 65.0f},
    {3.87f, 60.0f},
    {3.85f, 55.0f},
    {3.84f, 50.0f},
    {3.82f, 45.0f},
    {3.80f, 40.0f},
    {3.79f, 35.0f},
    {3.77f, 30.0f},
    {3.75f, 25.0f},
    {3.73f, 20.0f},
    {3.71f, 15.0f},
    {3.69f, 10.0f},
    {3.61f, 5.0f},
    {3.50f, 0.0f}
};
constexpr size_t SOC_LUT_SIZE = sizeof(SOC_LUT) / sizeof(SOC_LUT[0]);

class SocEstimator {
    public:
        SocEstimator(BatteryData_t batteryData);
        uint8_t getSocPercentage();
        int32_t getTimeRemaining();
        void calcStateOfCharge(BatteryData_t batteryData, bool mode);

    private:
        StateOfCharge_t socData;
};


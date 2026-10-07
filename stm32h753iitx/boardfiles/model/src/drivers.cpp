#include "drivers.hpp"
#include "zp_bit.hpp"
#include "museq.hpp"
#include "stm32h7xx_hal.h"
#include "zp_params.hpp"

#define MOT_TYPE_PWM 0
#define MOT_TYPE_DSHOT 5

// External hardware handles
extern IWDG_HandleTypeDef hiwdg1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern FDCAN_HandleTypeDef hfdcan1;

// ----------------------------------------------------------------------------
// Global handles
// ----------------------------------------------------------------------------
SystemUtils *systemUtilsHandle = nullptr;
MathUtils *mathUtilsHandle = nullptr;
FFT *fftHandle = nullptr;
IndependentWatchdog *iwdgHandle = nullptr;
Logger *loggerHandle = nullptr;

IMotorControl *motorHandles[8] = {0};

CANController *canControllerHandle = nullptr;
SafetySwitch *safetySwitchHandle = nullptr;
GPS *gps1Handle = nullptr;
GPS *gps2Handle = nullptr;
CRSFReceiver *rcHandle = nullptr;
RFD *telemLinkHandle = nullptr;
FusedIMU *imuHandle = nullptr;
PowerModule *pmHandle = nullptr;
Rangefinder *rangefinderHandle = nullptr;
Barometer *barometerHandle = nullptr;

MessageQueue<RCMotorControlMessage_t> *amRCQueueHandle = nullptr;
MessageQueue<char[100]> *smLoggerQueueHandle = nullptr;
MessageQueue<TMMessage_t> *tmQueueHandle = nullptr;
MessageQueue<mavlink_message_t> *messageBufferHandle = nullptr;

// ----------------------------------------------------------------------------
// Motor instances & group
// ----------------------------------------------------------------------------
MotorInstance_t motorInstances[8];
MotorGroupInstance_t mainMotorGroup;

typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
} motorChannel_t;

const motorChannel_t MOTOR_MAP[8] = {
    {&htim1, TIM_CHANNEL_1},
    {&htim1, TIM_CHANNEL_2},
    {&htim1, TIM_CHANNEL_3},
    {&htim1, TIM_CHANNEL_4},
    {&htim2, TIM_CHANNEL_1},
    {&htim2, TIM_CHANNEL_2},
    {&htim2, TIM_CHANNEL_3},
    {&htim2, TIM_CHANNEL_4},
};

const ZP_PARAM_ID SERVO_FUNC[8] = {
    ZP_PARAM_ID::SERVO1_FUNCTION,
    ZP_PARAM_ID::SERVO2_FUNCTION,
    ZP_PARAM_ID::SERVO3_FUNCTION,
    ZP_PARAM_ID::SERVO4_FUNCTION,
    ZP_PARAM_ID::SERVO5_FUNCTION,
    ZP_PARAM_ID::SERVO6_FUNCTION,
    ZP_PARAM_ID::SERVO7_FUNCTION,
    ZP_PARAM_ID::SERVO8_FUNCTION,
};

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------
// Split out so initModel can start BIT off this clock before any driver reports into it
void initSystemUtils() {
    systemUtilsHandle = new SystemUtils();
}

void initDrivers() {
    // Core utilities
    mathUtilsHandle = new MathUtils();
    fftHandle = new FFT();
    iwdgHandle = new IndependentWatchdog(&hiwdg1);
    loggerHandle = new Logger(); // Initialized later in RTOS task

    // Motors (servo index matches SERVOx param)
    float val = 0.0f;
    ZP_Error paramStatus = ZP_PARAM::get(ZP_PARAM_ID::MOT_PWM_TYPE, val);
    uint32_t servoType = static_cast<uint32_t>(val);

    for (int i = 0; i < 8; i++) {
        // Determine if it is brushless DC motor
        float funcVal = 0.0f;
        paramStatus |= ZP_PARAM::get(SERVO_FUNC[i], funcVal);

        MotorFunction_e func = static_cast<MotorFunction_e>(static_cast<int>(funcVal));
        bool isBLDC = false;
        #ifdef PLANE
        isBLDC = (func == MotorFunction_e::THROTTLE);
        #endif
        #ifdef QUADCOPTER
        isBLDC = (func == MotorFunction_e::MOTOR_1) || (func == MotorFunction_e::MOTOR_2) ||
                 (func == MotorFunction_e::MOTOR_3) || (func == MotorFunction_e::MOTOR_4);
        #endif
        if (isBLDC) {
            switch (servoType) {
            case MOT_TYPE_DSHOT: // DShot
                motorHandles[i] = new DshotMotorControl(MOTOR_MAP[i].timer, MOTOR_MAP[i].channel, false);
                break;
            case MOT_TYPE_PWM: // PWM
            default:
                motorHandles[i] = new MotorControl(MOTOR_MAP[i].timer, MOTOR_MAP[i].channel, 5, 10, i + 1);
                break;
            }
        }
        else {
            motorHandles[i] = new MotorControl(MOTOR_MAP[i].timer, MOTOR_MAP[i].channel, 5, 10, i + 1);
        }
    }

    // Peripherals
    safetySwitchHandle = new SafetySwitch(GPIOH, GPIO_PIN_12, GPIOH, GPIO_PIN_11);    
    gps1Handle = new GPS(&huart6);
    gps2Handle = new GPS(&huart3);
    rcHandle = new CRSFReceiver(&huart4);
    telemLinkHandle = new RFD(&huart1);
    IMU *imu0 = new IMU(&hspi1, GPIOC, GPIO_PIN_4, 0, IMU_ODR_1KHZ);
    IMU *imu1 = new IMU(&hspi1, GPIOC, GPIO_PIN_5, 1, IMU_ODR_1KHZ);
    imuHandle = new FusedIMU(&hspi1, imu0, imu1);
    pmHandle = new PowerModule(&hi2c1);
    float rngfndEnable = 0.0f;
    paramStatus |= ZP_PARAM::get(ZP_PARAM_ID::RNGFND_ENABLE, rngfndEnable);
    if (static_cast<int>(rngfndEnable) == 1) {
        rangefinderHandle = new Rangefinder(&hi2c3);
    }
    barometerHandle = new Barometer(&hi2c2);

    // Queues
    amRCQueueHandle = new MessageQueue<RCMotorControlMessage_t>(&amQueueId);
    smLoggerQueueHandle = new MessageQueue<char[100]>(&smLoggerQueueId);
    tmQueueHandle = new MessageQueue<TMMessage_t>(&tmQueueId);
    messageBufferHandle = new MessageQueue<mavlink_message_t>(&messageBufferId);

    // Initialize hardware components
    ZP_Error motorStatus = ZP_ERROR_OK;
    for (int i = 0; i < 8; i++) {
        motorStatus |= motorHandles[i]->init();
    }

    MotorControl::enableServo(GPIOF, GPIO_PIN_1);
    motorStatus |= MotorControl::enableServoSwitch(GPIOE, GPIO_PIN_3, &hspi4);
    (void)ZP_BIT::report(ZP_BIT_ID::MOTOR_INIT, motorStatus);

    canControllerHandle = new CANController(&hfdcan1, systemUtilsHandle);

    (void)ZP_BIT::report(ZP_BIT_ID::RC_INIT, rcHandle->init());
    
    (void)ZP_BIT::report(ZP_BIT_ID::GPS1_INIT, gps1Handle->init());
    (void)ZP_BIT::report(ZP_BIT_ID::GPS2_INIT, gps2Handle->init());
    (void)ZP_BIT::report(ZP_BIT_ID::IMU_INIT, imuHandle->init());
    (void)ZP_BIT::report(ZP_BIT_ID::TELEM_INIT, telemLinkHandle->init());
    (void)ZP_BIT::report(ZP_BIT_ID::PM_INIT, pmHandle->init());
    if (rangefinderHandle != nullptr) {
        (void)ZP_BIT::report(ZP_BIT_ID::RANGEFINDER_INIT,
                             rangefinderHandle->init());
    }
    (void)ZP_BIT::report(ZP_BIT_ID::BARO_INIT, barometerHandle->init());

    // Motor instances — fields loaded from ZP_PARAM by AttitudeManager::loadServoParams()
    for (int i = 0; i < 8; i++) {
        motorInstances[i] = {motorHandles[i]};
    }

    mainMotorGroup = {motorInstances, 8};

    (void)ZP_BIT::report(ZP_BIT_ID::PARAM_TABLE_INIT, paramStatus);
}

#include "drivers.hpp"
#include "museq.hpp"
#include "stm32l5xx_hal.h"
#include "zp_params.hpp"

#define MOT_TYPE_PWM   0
#define MOT_TYPE_DSHOT 5

// External hardware handles
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;

// ----------------------------------------------------------------------------
// Global handles
// ----------------------------------------------------------------------------
SystemUtils *systemUtilsHandle = nullptr;
MathUtils *mathUtilsHandle = nullptr;
FFT *fftHandle = nullptr;
IndependentWatchdog *iwdgHandle = nullptr;
Logger *loggerHandle = nullptr;

IMotorControl *motorHandles[8] = {0};

GPS *gpsHandle = nullptr;
CRSFReceiver *rcHandle = nullptr;
RFD *telemLinkHandle = nullptr;
IMU *imuHandle = nullptr;
PowerModule *pmHandle = nullptr;

MessageQueue<RCMotorControlMessage_t> *amRCQueueHandle = nullptr;
MessageQueue<char[100]> *smLoggerQueueHandle = nullptr;
MessageQueue<TMMessage_t> *tmQueueHandle = nullptr;
MessageQueue<mavlink_message_t> *messageBufferHandle = nullptr;

// ----------------------------------------------------------------------------
// Motor instances & group
// ----------------------------------------------------------------------------
MotorInstance_t motorInstances[8];
MotorGroupInstance_t mainMotorGroup;

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
} motorChannel_t;

const motorChannel_t MOTOR_MAP[8] = {
    {&htim3, TIM_CHANNEL_1}, {&htim3, TIM_CHANNEL_2}, {&htim3, TIM_CHANNEL_3}, {&htim3, TIM_CHANNEL_4},
    {&htim4, TIM_CHANNEL_1}, {&htim1, TIM_CHANNEL_1}, {&htim1, TIM_CHANNEL_2}, {&htim1, TIM_CHANNEL_3},
};

const ZP_PARAM_ID SERVO_FUNC[8] = {
    ZP_PARAM_ID::SERVO1_FUNCTION, ZP_PARAM_ID::SERVO2_FUNCTION,
    ZP_PARAM_ID::SERVO3_FUNCTION, ZP_PARAM_ID::SERVO4_FUNCTION,
    ZP_PARAM_ID::SERVO5_FUNCTION, ZP_PARAM_ID::SERVO6_FUNCTION,
    ZP_PARAM_ID::SERVO7_FUNCTION, ZP_PARAM_ID::SERVO8_FUNCTION,
};

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------
void initDrivers()
{
    // Core utilities
    fftHandle = new FFT();
    systemUtilsHandle = new SystemUtils();
    mathUtilsHandle = new MathUtils();
    iwdgHandle = new IndependentWatchdog(&hiwdg);
    loggerHandle = new Logger(); // Initialized later in RTOS task

    // Motors (servo index matches SERVOx param)
    uint32_t servoType = int(ZP_PARAM::get(ZP_PARAM_ID::MOT_PWM_TYPE));
    for (int i = 0; i < 8; i++) {
        bool isBLDC = false;
        #ifdef PLANE
        isBLDC = int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::THROTTLE);
        #endif
        #ifdef QUADCOPTER
        isBLDC = int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_1)
                        || int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_2)
                        || int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_3)
                        || int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_4);
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
        } else {
            motorHandles[i] = new MotorControl(MOTOR_MAP[i].timer, MOTOR_MAP[i].channel, 5, 10, i + 1);
        }
    }

    // Peripherals
    gpsHandle = new GPS(&huart2);
    rcHandle = new CRSFReceiver(&huart4);
    telemLinkHandle = new RFD(&huart3);
    imuHandle = new IMU(&hspi2, GPIOD, GPIO_PIN_0, 0, IMU_ODR_4KHZ);
    pmHandle = new PowerModule(&hi2c1);

    // Queues
    amRCQueueHandle = new MessageQueue<RCMotorControlMessage_t>(&amQueueId);
    smLoggerQueueHandle = new MessageQueue<char[100]>(&smLoggerQueueId);
    tmQueueHandle = new MessageQueue<TMMessage_t>(&tmQueueId);
    messageBufferHandle = new MessageQueue<mavlink_message_t>(&messageBufferId);

    // Initialize hardware components
    for (int i = 0; i < 8; i++) {
        motorHandles[i]->init();
    }

    rcHandle->init();
    gpsHandle->init();
    imuHandle->init();
    pmHandle->init();
    telemLinkHandle->init();

    // Motor instances — fields loaded from ZP_PARAM by AttitudeManager::loadServoParams()
    for (int i = 0; i < 8; i++) {
        motorInstances[i] = {motorHandles[i]};
    }

    mainMotorGroup = {motorInstances, 8};
}

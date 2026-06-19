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
// Initialization (no heap allocations)
// ----------------------------------------------------------------------------
ZP_ERROR_e initDrivers()
{
    // 1. Core utilities
    systemUtilsHandle = new SystemUtils();
    iwdgHandle = new IndependentWatchdog(&hiwdg);
    loggerHandle = new Logger();

    // 2. Motors (Fail-fast on parameter read)
    float val = 0.0f;
    ZP_ERROR_e status = ZP_PARAM::get(ZP_PARAM_ID::MOT_PWM_TYPE, val);
    if (status != ZP_ERROR_OK) Error_Handler();
    
    uint32_t servoType = static_cast<uint32_t>(val);
    
    for (int i = 0; i < 8; i++) {
        float funcVal = 0.0f;
        status = ZP_PARAM::get(SERVO_FUNC[i], funcVal);
        if (status != ZP_ERROR_OK) Error_Handler();

        MotorFunction_e func = static_cast<MotorFunction_e>(static_cast<int>(funcVal));
        bool isMotor = (func == MotorFunction_e::THROTTLE);

        if (isMotor && servoType == MOT_TYPE_DSHOT) {
            motorHandles[i] = new DshotMotorControl(MOTOR_MAP[i].timer, MOTOR_MAP[i].channel, false);
        } else {
            motorHandles[i] = new MotorControl(MOTOR_MAP[i].timer, MOTOR_MAP[i].channel, 5, 10, i + 1);
        }
    }

    // 3. Peripheral allocation
    gpsHandle = new GPS(&huart2);
    rcHandle = new CRSFReceiver(&huart4);
    telemLinkHandle = new RFD(&huart3);
    imuHandle = new IMU(&hspi2, GPIOD, GPIO_PIN_0);
    pmHandle = new PowerModule(&hi2c1);

    // 4. Queue allocation
    amRCQueueHandle = new MessageQueue<RCMotorControlMessage_t>(&amQueueId);
    smLoggerQueueHandle = new MessageQueue<char[100]>(&smLoggerQueueId);
    tmQueueHandle = new MessageQueue<TMMessage_t>(&tmQueueId);
    messageBufferHandle = new MessageQueue<mavlink_message_t>(&messageBufferId);

    // 5. Hardware Initialization (Fail-Fast)
    for (int i = 0; i < 8; i++) {
        status = motorHandles[i]->init();
        if (status != ZP_ERROR_OK) Error_Handler();
    }

    if ((status = rcHandle->init()) != ZP_ERROR_OK) Error_Handler();
    if ((status = gpsHandle->init()) != ZP_ERROR_OK) Error_Handler();
    if ((status = imuHandle->init()) != ZP_ERROR_OK) Error_Handler();
    if ((status = pmHandle->init()) != ZP_ERROR_OK) Error_Handler();
    if ((status = telemLinkHandle->init()) != ZP_ERROR_OK) Error_Handler();

    // 6. Finalize structure
    for (int i = 0; i < 8; i++) {
        motorInstances[i] = {motorHandles[i]};
    }
    mainMotorGroup = {motorInstances, 8};

    return ZP_ERROR_OK;
}
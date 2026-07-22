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
SDFileSystem *sdFileSystemHandle = nullptr;
FatFsBackend *fatFsBackendHandle = nullptr;

IMotorControl *motorHandles[8] = {0};

GPS *gpsHandle = nullptr;
CRSFReceiver *rcHandle = nullptr;
RFD *telemLinkHandle = nullptr;
IMU *imuHandle = nullptr;
PowerModule *pmHandle = nullptr;

MessageQueue<RCMotorControlMessage_t> *amRCQueueHandle = nullptr;
MessageQueue<TMMessage_t> *tmQueueHandle = nullptr;
MessageQueue<mavlink_message_t> *messageBufferHandle = nullptr;

MessageQueue<ExMemReqMsg> *sdRequestQueueHandle = nullptr;
MessageQueue<ExMemReqBuf> *sdBufferQueueHandle = nullptr;
IMessageQueue<PollResult> *sdResponseQueuesHandle[static_cast<size_t>(ManagerId_e::NUM_MANAGERS)] = {nullptr};

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
void initDrivers()
{
    // Core utilities
    systemUtilsHandle = new SystemUtils();
    iwdgHandle = new IndependentWatchdog(&hiwdg);

    // Motors (servo index matches SERVOx param)
    uint32_t servoType = int(ZP_PARAM::get(ZP_PARAM_ID::MOT_PWM_TYPE));
    for (int i = 0; i < 8; i++) {
        bool isMotor = int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::THROTTLE);
        if (isMotor) {
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
    imuHandle = new IMU(&hspi2, GPIOD, GPIO_PIN_0);
    pmHandle = new PowerModule(&hi2c1);

    // Queues
    amRCQueueHandle = new MessageQueue<RCMotorControlMessage_t>(&amQueueId);
    tmQueueHandle = new MessageQueue<TMMessage_t>(&tmQueueId);
    messageBufferHandle = new MessageQueue<mavlink_message_t>(&messageBufferId);
    sdRequestQueueHandle = new MessageQueue<ExMemReqMsg>(&sdRequestQueueId);
    sdBufferQueueHandle = new MessageQueue<ExMemReqBuf>(&sdBufferQueueId);
    for (int i = 0; i < static_cast<int>(ManagerId_e::NUM_MANAGERS); ++i) {
        sdResponseQueuesHandle[i] = new MessageQueue<PollResult>(&sdResponseQueueId[i]);
    }

    // File system
    sdFileSystemHandle = new SDFileSystem(sdRequestQueueHandle, sdBufferQueueHandle, sdResponseQueuesHandle);
    fatFsBackendHandle = new FatFsBackend();

    // Initialize hardware components
    for (int i = 0; i < 8; i++) {
        motorHandles[i]->init();
    }

    rcHandle->init();
    gpsHandle->init();
    imuHandle->init();
    pmHandle->init();
    telemLinkHandle->init();
    sdFileSystemHandle->init();
    
    // Motor instances — fields loaded from ZP_PARAM by AttitudeManager::loadServoParams()
    for (int i = 0; i < 8; i++) {
        motorInstances[i] = {motorHandles[i]};
    }

    mainMotorGroup = {motorInstances, 8};
}

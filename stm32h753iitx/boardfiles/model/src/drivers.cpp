#include "drivers.hpp"
#include "museq.hpp"
#include "stm32h7xx_hal.h"

// External hardware handles
extern IWDG_HandleTypeDef hiwdg1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern I2C_HandleTypeDef hi2c1;

// ----------------------------------------------------------------------------
// Static storage for each driver (aligned for correct type)
// ----------------------------------------------------------------------------
alignas(SystemUtils) static uint8_t systemUtilsStorage[sizeof(SystemUtils)];
alignas(IndependentWatchdog) static uint8_t iwdgStorage[sizeof(IndependentWatchdog)];
alignas(Logger) static uint8_t loggerStorage[sizeof(Logger)];

alignas(MotorControl) static uint8_t leftAileronMotorStorage[sizeof(MotorControl)];
alignas(MotorControl) static uint8_t rightAileronMotorStorage[sizeof(MotorControl)];
alignas(MotorControl) static uint8_t elevatorMotorStorage[sizeof(MotorControl)];
alignas(MotorControl) static uint8_t rudderMotorStorage[sizeof(MotorControl)];
alignas(MotorControl) static uint8_t throttleMotorStorage[sizeof(MotorControl)];
alignas(MotorControl) static uint8_t leftFlapMotorStorage[sizeof(MotorControl)];
alignas(MotorControl) static uint8_t rightFlapMotorStorage[sizeof(MotorControl)];
alignas(MotorControl) static uint8_t steeringMotorStorage[sizeof(MotorControl)];

alignas(GPS) static uint8_t gpsStorage[sizeof(GPS)];
alignas(CRSFReceiver) static uint8_t rcStorage[sizeof(CRSFReceiver)];
alignas(RFD) static uint8_t rfdStorage[sizeof(RFD)];
alignas(IMU) static uint8_t imuStorage[sizeof(IMU)];
alignas(PowerModule) static uint8_t pmStorage[sizeof(PowerModule)];

alignas(MessageQueue<RCMotorControlMessage_t>) static uint8_t amRCQueueStorage[sizeof(MessageQueue<RCMotorControlMessage_t>)];
alignas(MessageQueue<char[100]>) static uint8_t smLoggerQueueStorage[sizeof(MessageQueue<char[100]>)];
alignas(MessageQueue<TMMessage_t>) static uint8_t tmQueueStorage[sizeof(MessageQueue<TMMessage_t>)];
alignas(MessageQueue<mavlink_message_t>) static uint8_t messageBufferStorage[sizeof(MessageQueue<mavlink_message_t>)];

// ----------------------------------------------------------------------------
// Global handles
// ----------------------------------------------------------------------------
SystemUtils *systemUtilsHandle = nullptr;
IndependentWatchdog *iwdgHandle = nullptr;
Logger *loggerHandle = nullptr;

MotorControl *leftAileronMotorHandle = nullptr;
MotorControl *rightAileronMotorHandle = nullptr;
MotorControl *elevatorMotorHandle = nullptr;
MotorControl *rudderMotorHandle = nullptr;
MotorControl *throttleMotorHandle = nullptr;
MotorControl *leftFlapMotorHandle = nullptr;
MotorControl *rightFlapMotorHandle = nullptr;
MotorControl *steeringMotorHandle = nullptr;

GPS *gpsHandle = nullptr;
CRSFReceiver *rcHandle = nullptr;
RFD *rfdHandle = nullptr;
IMU *imuHandle = nullptr;
PowerModule *pmHandle = nullptr;

MessageQueue<RCMotorControlMessage_t> *amRCQueueHandle = nullptr;
MessageQueue<char[100]> *smLoggerQueueHandle = nullptr;
MessageQueue<TMMessage_t> *tmQueueHandle = nullptr;
MessageQueue<mavlink_message_t> *messageBufferHandle = nullptr;

// ----------------------------------------------------------------------------
// Motor instances & groups
// ----------------------------------------------------------------------------
MotorInstance_t leftAileronMotorInstance;
MotorInstance_t rightAileronMotorInstance;
MotorInstance_t elevatorMotorInstance;
MotorInstance_t rudderMotorInstance;
MotorInstance_t throttleMotorInstance;
MotorInstance_t leftFlapMotorInstance;
MotorInstance_t rightFlapMotorInstance;
MotorInstance_t steeringMotorInstance;

MotorInstance_t aileronMotorInstances[2];
MotorInstance_t flapMotorInstances[2];

MotorGroupInstance_t rollMotors;
MotorGroupInstance_t pitchMotors;
MotorGroupInstance_t yawMotors;
MotorGroupInstance_t throttleMotors;
MotorGroupInstance_t flapMotors;
MotorGroupInstance_t steeringMotors;

// ----------------------------------------------------------------------------
// Initialization (no heap allocations)
// ----------------------------------------------------------------------------
void initDrivers()
{
    // Core utilities
    systemUtilsHandle = new (&systemUtilsStorage) SystemUtils();
    iwdgHandle = new (&iwdgStorage) IndependentWatchdog(&hiwdg1);
    loggerHandle = new (&loggerStorage) Logger(); // Initialized later in RTOS task

    // Motors
    leftAileronMotorHandle = new (&leftAileronMotorStorage) MotorControl(&htim1, TIM_CHANNEL_1, 5, 10);
    rightAileronMotorHandle = new (&rightAileronMotorStorage) MotorControl(&htim1, TIM_CHANNEL_2, 5, 10);
    elevatorMotorHandle = new (&elevatorMotorStorage) MotorControl(&htim1, TIM_CHANNEL_3, 5, 10);
    rudderMotorHandle = new (&rudderMotorStorage) MotorControl(&htim1, TIM_CHANNEL_4, 5, 10);
    throttleMotorHandle = new (&throttleMotorStorage) MotorControl(&htim2, TIM_CHANNEL_1, 5, 10);
    leftFlapMotorHandle = new (&leftFlapMotorStorage) MotorControl(&htim2, TIM_CHANNEL_2, 5, 10);
    rightFlapMotorHandle = new (&rightFlapMotorStorage) MotorControl(&htim2, TIM_CHANNEL_3, 5, 10);
    steeringMotorHandle = new (&steeringMotorStorage) MotorControl(&htim2, TIM_CHANNEL_4, 5, 10);

    // Peripherals
    gpsHandle = new (&gpsStorage) GPS(&huart2);
    rcHandle = new (&rcStorage) CRSFReceiver(&huart4);
    rfdHandle = new (&rfdStorage) RFD(&huart1);
    imuHandle = new (&imuStorage) IMU(&hspi1, GPIOC, GPIO_PIN_5);
    pmHandle = new (&pmStorage) PowerModule(&hi2c1);

    // Queues
    amRCQueueHandle = new (&amRCQueueStorage) MessageQueue<RCMotorControlMessage_t>(&amQueueId);
    smLoggerQueueHandle = new (&smLoggerQueueStorage) MessageQueue<char[100]>(&smLoggerQueueId);
    tmQueueHandle = new (&tmQueueStorage) MessageQueue<TMMessage_t>(&tmQueueId);
    messageBufferHandle = new (&messageBufferStorage) MessageQueue<mavlink_message_t>(&messageBufferId);

    // Initialize hardware components
    leftAileronMotorHandle->init();
    rightAileronMotorHandle->init();
    elevatorMotorHandle->init();
    rudderMotorHandle->init();
    throttleMotorHandle->init();
    leftFlapMotorHandle->init();
    rightFlapMotorHandle->init();
    steeringMotorHandle->init();
    MotorControl::enableServo(GPIOF, GPIO_PIN_1);
    MotorControl::enableServoSwitch(GPIOE, GPIO_PIN_3, &hspi4);

    rcHandle->init();
    gpsHandle->init();
    imuHandle->init();
    rfdHandle->startReceive();

    // Motor instance bindings
    leftAileronMotorInstance = {leftAileronMotorHandle, true};
    rightAileronMotorInstance = {rightAileronMotorHandle, true};
    elevatorMotorInstance = {elevatorMotorHandle, false};
    rudderMotorInstance = {rudderMotorHandle, false};
    throttleMotorInstance = {throttleMotorHandle, false};
    leftFlapMotorInstance = {leftFlapMotorHandle, false};
    rightFlapMotorInstance = {rightFlapMotorHandle, true};
    steeringMotorInstance = {steeringMotorHandle, true};

    aileronMotorInstances[0] = leftAileronMotorInstance;
    aileronMotorInstances[1] = rightAileronMotorInstance;

    flapMotorInstances[0] = leftFlapMotorInstance;
    flapMotorInstances[1] = rightFlapMotorInstance;

    rollMotors = {aileronMotorInstances, 2};
    pitchMotors = {&elevatorMotorInstance, 1};
    yawMotors = {&rudderMotorInstance, 1};
    throttleMotors = {&throttleMotorInstance, 1};
    flapMotors = {flapMotorInstances, 2};
    steeringMotors = {&steeringMotorInstance, 1};
}

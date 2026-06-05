#include "drivers.hpp"
#include "museq.hpp"
#include "stm32l5xx_hal.h"

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

#ifdef FIXED_WING
MotorControl *motor1Handle = nullptr;
MotorControl *motor2Handle = nullptr;
MotorControl *motor3Handle = nullptr;
MotorControl *motor4Handle = nullptr;
MotorControl *motor5Handle = nullptr;
MotorControl *motor6Handle = nullptr;
MotorControl *motor7Handle = nullptr;
MotorControl *motor8Handle = nullptr;
#endif

#ifdef QUADCOPTER
DshotMotorControl *motor1Handle = nullptr;
DshotMotorControl *motor2Handle = nullptr;
DshotMotorControl *motor3Handle = nullptr;
DshotMotorControl *motor4Handle = nullptr;
MotorControl *motor5Handle = nullptr;
MotorControl *motor6Handle = nullptr;
MotorControl *motor7Handle = nullptr;
MotorControl *motor8Handle = nullptr;
#endif

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

// ----------------------------------------------------------------------------
// Initialization (no heap allocations)
// ----------------------------------------------------------------------------
void initDrivers()
{
    // Core utilities
    systemUtilsHandle = new (&systemUtilsStorage) SystemUtils();
    iwdgHandle = new (&iwdgStorage) IndependentWatchdog(&hiwdg);
    loggerHandle = new (&loggerStorage) Logger(); // Initialized later in RTOS task


    // Motors (servo index matches SERVOx param)
    #ifdef FIXED_WING
    motor1Handle = new (&motor1Storage) MotorControl(&htim3, TIM_CHANNEL_1, 5, 10, 1);
    motor2Handle = new (&motor2Storage) MotorControl(&htim3, TIM_CHANNEL_2, 5, 10, 2);
    motor3Handle = new (&motor3Storage) MotorControl(&htim3, TIM_CHANNEL_3, 5, 10, 3);
    motor4Handle = new (&motor4Storage) MotorControl(&htim3, TIM_CHANNEL_4, 5, 10, 4);
    motor5Handle = new (&motor5Storage) MotorControl(&htim4, TIM_CHANNEL_1, 5, 10, 5);
    motor6Handle = new (&motor6Storage) MotorControl(&htim1, TIM_CHANNEL_1, 5, 10, 6);
    motor7Handle = new (&motor7Storage) MotorControl(&htim1, TIM_CHANNEL_2, 5, 10, 7);
    motor8Handle = new (&motor8Storage) MotorControl(&htim1, TIM_CHANNEL_3, 5, 10, 8);
	#endif

	#ifdef QUADCOPTER
    motor1Handle = new (&motor1Storage) DshotMotorControl(&htim3, TIM_CHANNEL_1, false;
    motor2Handle = new (&motor2Storage) DshotMotorControl(&htim3, TIM_CHANNEL_2, false);
    motor3Handle = new (&motor3Storage) DshotMotorControl(&htim3, TIM_CHANNEL_3, false);
    motor4Handle = new (&motor4Storage) DshotMotorControl(&htim3, TIM_CHANNEL_4, false);
    motor5Handle = new (&motor5Storage) MotorControl(&htim4, TIM_CHANNEL_1, 5, 10, 5);
    motor6Handle = new (&motor6Storage) MotorControl(&htim1, TIM_CHANNEL_1, 5, 10, 6);
    motor7Handle = new (&motor7Storage) MotorControl(&htim1, TIM_CHANNEL_2, 5, 10, 7);
    motor8Handle = new (&motor8Storage) MotorControl(&htim1, TIM_CHANNEL_3, 5, 10, 8);
    #endif
    
    // Peripherals
    gpsHandle = new (&gpsStorage) GPS(&huart2);
    rcHandle = new (&crsfStorage) CRSFReceiver(&huart4);
    telemLinkHandle = new (&telemLinkStorage) RFD(&huart3);
    imuHandle = new (&imuStorage) IMU(&hspi2, GPIOD, GPIO_PIN_0);
    pmHandle = new (&pmStorage) PowerModule(&hi2c1);

    // Queues
    amRCQueueHandle = new (&amRCQueueStorage) MessageQueue<RCMotorControlMessage_t>(&amQueueId);
    smLoggerQueueHandle = new (&smLoggerQueueStorage) MessageQueue<char[100]>(&smLoggerQueueId);
    tmQueueHandle = new (&tmQueueStorage) MessageQueue<TMMessage_t>(&tmQueueId);
    messageBufferHandle = new (&messageBufferStorage) MessageQueue<mavlink_message_t>(&messageBufferId);

    // Initialize hardware components
    motor1Handle->init();
    motor2Handle->init();
    motor3Handle->init();
    motor4Handle->init();
    motor5Handle->init();
    motor6Handle->init();
    motor7Handle->init();
    motor8Handle->init();

    rcHandle->init();
    gpsHandle->init();
    imuHandle->init();
    pmHandle->init();
    telemLinkHandle->init();

    // Motor instances — fields loaded from ZP_PARAM by AttitudeManager::loadServoParams()
    motorInstances[0] = {motor1Handle};
    motorInstances[1] = {motor2Handle};
    motorInstances[2] = {motor3Handle};
    motorInstances[3] = {motor4Handle};
    motorInstances[4] = {motor5Handle};
    motorInstances[5] = {motor6Handle};
    motorInstances[6] = {motor7Handle};
    motorInstances[7] = {motor8Handle};

    mainMotorGroup = {motorInstances, 8};
}

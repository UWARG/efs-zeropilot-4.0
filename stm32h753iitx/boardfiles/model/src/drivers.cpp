#include "drivers.hpp"
#include "museq.hpp"
#include "stm32h7xx_hal.h"
#include "zp_params.hpp"

// ---- SELFTEST BEGIN (temporary, delete before committing) ----
#include "../../drivers/eeprom/eeprom.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstring>
// ---- SELFTEST END ----

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

// ---- SELFTEST BEGIN (temporary, delete before committing) ----
// Sets up SPI6 by hand, so the .ioc file does not need to change.
// Pins from the ZeroPilot 4.0c schematic:
//   PG13 = SCK, PG12 = MISO, PG14 = MOSI  (SPI6 alternate function 5)
//   PG15 = chip select, PG11 = write protect (plain outputs, both held high)
static SPI_HandleTypeDef hspi6Test;
static volatile int eepromSpiInitResult = -1;   // 0 means HAL_OK

// Prints text over UART8. Read it on the computer with:
//   screen /dev/ttyACM1 115200      (quit with Ctrl-A then K)
extern UART_HandleTypeDef huart8;

static void eepromTestPrint(const char *format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart8, reinterpret_cast<uint8_t *>(buf), strlen(buf), 100);
}

// Crude pause that does not depend on any timer or the RTOS.
static void eepromTestPause() {
    for (volatile uint32_t i = 0; i < 40000000; i++) {}
}

static void eepromTestInitSpi6() {
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_SPI6_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    // CS and WP first: drive them high BEFORE switching them to outputs,
    // so the chip never sees a stray low on chip select.
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15 | GPIO_PIN_11, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_15 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &gpio);

    // SCK, MISO, MOSI handed over to the SPI6 hardware.
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_SPI6;
    HAL_GPIO_Init(GPIOG, &gpio);

    hspi6Test.Instance = SPI6;
    hspi6Test.Init.Mode = SPI_MODE_MASTER;
    hspi6Test.Init.Direction = SPI_DIRECTION_2LINES;
    hspi6Test.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi6Test.Init.CLKPolarity = SPI_POLARITY_LOW;      // SPI mode 0
    hspi6Test.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi6Test.Init.NSS = SPI_NSS_SOFT;                   // we toggle CS ourselves
    hspi6Test.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;   // slow on purpose
    hspi6Test.Init.FirstBit = SPI_FIRSTBIT_MSB;
    // Every other field is left at 0, which means "off" or "default" in this HAL.

    eepromSpiInitResult = static_cast<int>(HAL_SPI_Init(&hspi6Test));
}

// The whole test, printing to the terminal. Never returns, so the rest of the
// firmware does not start while testing (props off!).
static void eepromRunTestAndPrint() {
    // Step 0: prove the print path works before anything else can go wrong.
    for (int i = 0; i < 3; i++) {
        HAL_IWDG_Refresh(&hiwdg1);
        eepromTestPrint("hello from the board (%d)\r\n", i);
        eepromTestPause();
    }

    eepromTestPrint("=== EEPROM test starting ===\r\n");

    HAL_IWDG_Refresh(&hiwdg1);
    eepromTestInitSpi6();
    eepromTestPrint("SPI6 init (0 is OK): %d\r\n", eepromSpiInitResult);

    EEPROM *eepromTest = new EEPROM(&hspi6Test, GPIOG, GPIO_PIN_15);

    HAL_IWDG_Refresh(&hiwdg1);
    eepromTestPrint("Running internals test...\r\n");
    int internalsResult = eepromTest->selfTestInternals();
    eepromTestPrint("Internals (0 is pass): %d\r\n", internalsResult);

    HAL_IWDG_Refresh(&hiwdg1);
    eepromTestPrint("Running public function tests...\r\n");
    int testResult = eepromTest->selfTest();
    eepromTestPrint("Tests (0 is pass): %d\r\n", testResult);

    HAL_IWDG_Refresh(&hiwdg1);
    int bootCount = eepromTest->bootCounter();
    eepromTestPrint("Boot counter: %d\r\n", bootCount);

    // Repeat the summary forever, so a terminal opened late still catches it.
    while (true) {
        HAL_IWDG_Refresh(&hiwdg1);
        eepromTestPrint("SUMMARY spi=%d internals=%d tests=%d boots=%d\r\n",
                        eepromSpiInitResult, internalsResult, testResult, bootCount);
        eepromTestPause();
    }
}
// ---- SELFTEST END ----

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------
void initDrivers() {
    // ---- SELFTEST BEGIN (temporary, delete before committing) ----
    // Runs FIRST, before any other driver, so nothing earlier can hang it.
    eepromRunTestAndPrint();
    // ---- SELFTEST END ----

    // Core utilities
    systemUtilsHandle = new SystemUtils();
    mathUtilsHandle = new MathUtils();
    fftHandle = new FFT();
    iwdgHandle = new IndependentWatchdog(&hiwdg1);
    loggerHandle = new Logger(); // Initialized later in RTOS task

    // Motors (servo index matches SERVOx param)
    uint32_t servoType = int(ZP_PARAM::get(ZP_PARAM_ID::MOT_PWM_TYPE));
    for (int i = 0; i < 8; i++) {
        // Determine if it is brushless DC motor
        bool isBLDC = false;
        #ifdef PLANE
        isBLDC = int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::THROTTLE);
        #endif
        #ifdef QUADCOPTER
        isBLDC = int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_1) || int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_2) || int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_3) || int(ZP_PARAM::get(SERVO_FUNC[i])) == int(MotorFunction_e::MOTOR_4);
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
    if (ZP_PARAM::get(ZP_PARAM_ID::RNGFND_ENABLE) == 1) {
        rangefinderHandle = new Rangefinder(&hi2c3);
    }
    barometerHandle = new Barometer(&hi2c2);

    // Queues
    amRCQueueHandle = new MessageQueue<RCMotorControlMessage_t>(&amQueueId);
    smLoggerQueueHandle = new MessageQueue<char[100]>(&smLoggerQueueId);
    tmQueueHandle = new MessageQueue<TMMessage_t>(&tmQueueId);
    messageBufferHandle = new MessageQueue<mavlink_message_t>(&messageBufferId);

    // Initialize hardware components
    for (int i = 0; i < 8; i++) {
        motorHandles[i]->init();
    }
    MotorControl::enableServo(GPIOF, GPIO_PIN_1);
    MotorControl::enableServoSwitch(GPIOE, GPIO_PIN_3, &hspi4);

    canControllerHandle = new CANController(&hfdcan1, systemUtilsHandle);

    rcHandle->init();
    gps1Handle->init();
    gps2Handle->init();
    imuHandle->init();
    telemLinkHandle->init();
    pmHandle->init();
    if (rangefinderHandle != nullptr) {
        rangefinderHandle->init();
    }
    barometerHandle->init();

    // Motor instances — fields loaded from ZP_PARAM by AttitudeManager::loadServoParams()
    for (int i = 0; i < 8; i++) {
        motorInstances[i] = {motorHandles[i]};
    }

    mainMotorGroup = {motorInstances, 8};
}

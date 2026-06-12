#include "dshot.hpp"
#include <cstring>

static constexpr uint16_t DSHOT_1_CCR =	150;    // CCR for dshot300 logic 1 
static constexpr uint16_t DSHOT_0_CCR =	75;    // CCR for dshot300 logic 0 

static constexpr uint16_t MAX_THROTTLE = 2047;  // 11 bit max val
static constexpr uint16_t OFFSET = 100;     // Armed standby throttle
static constexpr uint16_t MIN_THROTTLE = 48 + OFFSET;    // 0-47 reserved for special commands

static constexpr uint16_t THROTTLE_MASK = 0x7FF;
static constexpr uint8_t THROTTLE_SHIFT = 5;
static constexpr uint8_t TEL_MASK = 1;
static constexpr uint8_t TEL_SHIFT = 4;
static constexpr uint8_t CRC_MASK = 0x0F;

DshotMotorControl::DshotMotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, bool telReq):
    timer(timer), 
    timerChannel(timerChannel), 
    telReq(telReq){}

void DshotMotorControl::set(uint32_t percent) {
    percent =  (percent > 100) ? 100 : percent;

    // Throttle 0 = disarm, 48-2047 = active throttle range
    uint32_t throttleVal = 0;
    if (armFlag) {
        throttleVal = (percent == 0) ? MIN_THROTTLE : (percent / 100.0) * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE;
    }

    // 11 bits throttle + 1 bit telemetry request + 4 bits CRC
    uint8_t crc = DshotMotorControl::calculateCrc(throttleVal, telReq);
    uint16_t frame = ( (throttleVal & THROTTLE_MASK) << THROTTLE_SHIFT ) | ( (telReq & TEL_MASK) << TEL_SHIFT ) | (crc & CRC_MASK);

    // Encode each bit to CRC val into temp buffer
	for (int i = 0; i < DSHOT_FRAME_LEN; i++) {
		uint8_t bit = (frame >> (DSHOT_FRAME_LEN - 1 - i)) & 1;
        updateBuffer[i] = (bit == 1) ? DSHOT_1_CCR : DSHOT_0_CCR;
	}

    // Set last index CRC to 0 to idle low after the frame until next PID call
    updateBuffer[DSHOT_BUF_LEN - 1] = 0;

    memcpy(dmaBuffer, updateBuffer, sizeof(updateBuffer));
    if (HAL_TIM_PWM_Start_DMA(timer, timerChannel, (uint32_t*)dmaBuffer, DSHOT_BUF_LEN) != HAL_OK) {
        // Error_Handler();    Error handling to be done
    }
}

void DshotMotorControl::init() {
    timer->Init.Prescaler = 0;
    timer->Init.Period = 199;
    if (HAL_TIM_Base_Init(timer) != HAL_OK) {
        Error_Handler();
    }
    setArm(false);
    this->set(0);
}

uint8_t DshotMotorControl::calculateCrc(uint16_t throttleVal, uint8_t telReq) {
    uint16_t preCrc = (throttleVal << 1) | telReq;
    return (preCrc ^ (preCrc >> 4) ^ (preCrc >> 8)) & CRC_MASK;
}

#include "dshot.hpp"
#include "string.h"

static constexpr uint16_t DSHOT_1_CCR =	600;    // CCR for dshot300 logic 1 
static constexpr uint16_t DSHOT_0_CCR =	300;    // CCR for dshot300 logic 0 

static constexpr uint16_t MAX_THROTTLE = 2047;  // 11 bit max val
static constexpr uint16_t OFFSET = 100;     // armed standby throttle
static constexpr uint16_t MIN_THROTTLE = 48 + OFFSET;    // 0-47 reserved for special commands


DshotMotorControl::DshotMotorControl(TIM_HandleTypeDef *timer, uint32_t timerChannel, bool telReq):
    timer(timer), 
    timerChannel(timerChannel), 
    telReq(telReq){
        // blank
}

// should include tel_req or not
void DshotMotorControl::set(uint32_t percent) {
    percent =  (percent > 100) ? 100 : percent;

    // throttle 0 = disarm, 48-2047 = active throttle range
    uint32_t throttle_val = 0;
    if(armFlag) {
        throttle_val = (percent == 0) ? MIN_THROTTLE : (percent / 100.0) * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE;
    }

    // 11 bits throttle + 1 bit telemetry request + 4 bits CRC
    uint16_t pre_crc = (throttle_val << 1) | telReq;
	uint8_t crc = (pre_crc ^ (pre_crc >> 4) ^ (pre_crc >> 8)) & 0x0F;
	uint16_t frame = ( (throttle_val & 0x7FF) << 5 ) | ( (telReq & 1) << 4 ) | (crc & 0xF);

    // encode each bit to CRC val into temp buffer
	for(int i = 0; i < DSHOT_FRAME_LEN; i++) {
		uint8_t bit = (frame >> (DSHOT_FRAME_LEN - 1 - i)) & 1;
        update_buffer[i] = (bit == 1) ? DSHOT_1_CCR : DSHOT_0_CCR;
	}

    // set last index CRC to 0 to idle low after the frame until next PID call
    update_buffer[DSHOT_BUF_LEN - 1] = 0;

    memcpy(dma_buffer, update_buffer, sizeof(update_buffer));
    if( HAL_TIM_PWM_Start_DMA(timer, timerChannel, (uint32_t*)dma_buffer, DSHOT_BUF_LEN) != HAL_OK){
        // Error_Handler();    error handling to be done
    }
}

void DshotMotorControl::init() {
    this->set(0);
}

void DshotMotorControl::setArm(bool arm) { 
    armFlag = arm;
}

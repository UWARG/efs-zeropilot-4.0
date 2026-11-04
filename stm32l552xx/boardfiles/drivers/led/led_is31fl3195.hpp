/*
 * IS31FL3195 Driver
 * used on the M10 GPS Status LED
 *
 * Made with love (and sleep deprivation)
 * with research and emotional support from Fei Wang
 *
 *  Created on: Oct 27, 2026
 *     Authors: Colin Cai & Fei Wang, graduating 2031
 *
 * Datasheet: https://lumissil.com/assets/pdf/core/IS31FL3195_DS.pdf
 *
 * Note: currently this only has support for "current level" mode i.e. setting raw colors directly from the MCU.
 * TODO: implement pattern mode?
 */

#ifndef IS31FL3195_I2C_H_
#define IS31FL3195_I2C_H_

#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_i2c.h"

// from datasheet
enum IS31FL3195_Timing {
	IS31FL3195_TIMING_30 = 0,
	IS31FL3195_TIMING_130 = 1,
	IS31FL3195_TIMING_260 = 2,
	IS31FL3195_TIMING_380 = 3,
	IS31FL3195_TIMING_510 = 4,
	IS31FL3195_TIMING_770 = 5,
	IS31FL3195_TIMING_1040 = 6,
	IS31FL3195_TIMING_1600 = 7,
	IS31FL3195_TIMING_2100 = 8,
	IS31FL3195_TIMING_2600 = 9,
	IS31FL3195_TIMING_3100 = 10,
	IS31FL3195_TIMING_4200 = 11,
	IS31FL3195_TIMING_5200 = 12,
	IS31FL3195_TIMING_6200 = 13,
	IS31FL3195_TIMING_7300 = 14,
	IS31FL3195_TIMING_8300 = 15,
};

//static constexpr IS31FL3195_Timing timingForMillis(uint16_t millis) {
//	const size_t NUM_TIMINGS = 16;
//	const uint16_t millis_for_timings[NUM_TIMINGS] = {    30,  130,  260,  380,  510,  770, 1040, 1600, 2100, 2600, 3100, 4200, 5200, 6200, 7300, 8300 };
//
//	for(int i = 0; i < NUM_TIMINGS - 1; i++) {
//		if(millis_for_timings[i] < millis)
//			return (millis - millis_for_timings[i]) < (millis_for_timings[i + 1] - millis) ? i : i + 1;
//	}
//	return NUM_TIMINGS - 1;
//}

typedef uint8_t IS31FL3195_CurrentLevel;

enum IS31FL3195_OperatingMode {
	IS31FL3195_MODE_CURRENT_LEVEL = 0,
// not implemented :(
// IS31FL3195_MODE_PATTERN = 1,
};

typedef struct __attribute__((packed)) {
	uint8_t _unused :4;

	IS31FL3195_OperatingMode mode_1 :1;
	IS31FL3195_OperatingMode mode_2 :1;
	IS31FL3195_OperatingMode mode_3 :1;
	IS31FL3195_OperatingMode mode_4 :1;
} IS31FL3195_ConfReg;

typedef struct __attribute__((packed)) {
	// TODO: flesh out according to datasheet (custom enums, etc)

	// SSD: 0 = shutdown, 1 = normal operation
	uint8_t softwareShutdownEnable: 1;
	uint8_t sleepModeEnable: 2;
	uint8_t chargePumpPowerUpMode: 1;

	uint8_t enable_1: 1;
	uint8_t enable_2: 1;
	uint8_t enable_3: 1;
	uint8_t enable_4: 1;
} IS31FL3195_ShutdownCtrl;

typedef uint8_t IS31FL3195_Register;

typedef struct {
	IS31FL3195_Register REG_CURRENT_LEVEL;
} IS31FL3195_Channel;

extern IS31FL3195_Channel IS31FL3195_ALL_CHANNELS[4];

#define IS31FL3195_CH1 IS31FL3195_ALL_CHANNELS[0]
#define IS31FL3195_CH2 IS31FL3195_ALL_CHANNELS[1]
#define IS31FL3195_CH3 IS31FL3195_ALL_CHANNELS[2]
#define IS31FL3195_CH4 IS31FL3195_ALL_CHANNELS[3]

// aliases to RGBW
#define IS31FL3195_CH_RED   IS31FL3195_CH1
#define IS31FL3195_CH_GREEN IS31FL3195_CH2
#define IS31FL3195_CH_BLUE  IS31FL3195_CH3
#define IS31FL3195_CH_WHITE IS31FL3195_CH4

class IS31FL3195 {
public:
	// for the M10 GPS, this is 0b10101000u.
	// (i found it via brute forcing all 256 addresses)
	IS31FL3195(I2C_HandleTypeDef *hi2c, uint8_t address);

	HAL_StatusTypeDef begin();

	HAL_StatusTypeDef reset();

	HAL_StatusTypeDef setConfigure(IS31FL3195_ConfReg conf);
	HAL_StatusTypeDef setShutdownControl(IS31FL3195_ShutdownCtrl conf);

	HAL_StatusTypeDef writeCurrentLevel(IS31FL3195_Channel &channel,
			IS31FL3195_CurrentLevel level);

	HAL_StatusTypeDef flush();

	HAL_StatusTypeDef setSolidColor(IS31FL3195_CurrentLevel rgbw[4]);

	~IS31FL3195();

private:
	I2C_HandleTypeDef *hi2c;
	uint8_t address;

	HAL_StatusTypeDef registerWrite(IS31FL3195_Register reg, uint8_t val);
};

#endif /* IS31FL3195_I2C_H_ */

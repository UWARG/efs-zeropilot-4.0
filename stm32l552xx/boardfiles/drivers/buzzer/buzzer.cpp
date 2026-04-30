#include "buzzer.hpp"

static M10Buzzer *activeBuzzer = nullptr;

namespace {
constexpr uint16_t DISARMED_CHIRP_TICKS = 120;
constexpr uint16_t ARMED_CHIRP_TICKS = 60;
constexpr uint16_t ARMED_CHIRP_GAP_TICKS = 40;
constexpr uint16_t ARMED_CONFIRM_TICKS = (ARMED_CHIRP_TICKS * 2) + ARMED_CHIRP_GAP_TICKS;
constexpr uint8_t DISARMED_TONE_DIVIDER = 2;
constexpr uint8_t ARMED_TONE_DIVIDER = 1;
}

M10Buzzer::M10Buzzer(
	GPIO_TypeDef *buzzerPort,
	uint16_t buzzerPin
):
	buzzerPort(buzzerPort),
	buzzerPin(buzzerPin) {
}

void M10Buzzer::init() {
	activeBuzzer = this;
	buzzerEnabled = false;
	startMode(Mode::Silent);
}

void M10Buzzer::buzzerOn() {
	if (!buzzerEnabled) {
		buzzerEnabled = true;
		startMode(Mode::DisarmedConfirm);
	}
}

void M10Buzzer::buzzerOff() {
	if (buzzerEnabled) {
		buzzerEnabled = false;
		startMode(Mode::ArmedConfirm);
	}
}

void M10Buzzer::serviceTick() {
	switch (mode) {
	case Mode::DisarmedConfirm:
		if (patternTick < DISARMED_CHIRP_TICKS) {
			driveTone(DISARMED_TONE_DIVIDER);
		} else {
			startMode(Mode::Silent);
			return;
		}

		patternTick += 1;
		break;
	case Mode::ArmedConfirm:
		if (patternTick < ARMED_CHIRP_TICKS) {
			driveTone(ARMED_TONE_DIVIDER);
		} else if (patternTick < (ARMED_CHIRP_TICKS + ARMED_CHIRP_GAP_TICKS)) {
			driveLow();
		} else if (patternTick < ARMED_CONFIRM_TICKS) {
			driveTone(ARMED_TONE_DIVIDER);
		} else {
			startMode(Mode::Silent);
			return;
		}

		patternTick += 1;
		break;
	case Mode::Silent:
		driveLow();
		break;
	}
}

void M10Buzzer::startMode(Mode nextMode) {
	mode = nextMode;
	patternTick = 0;
	toneDividerCounter = 0;
	driveLow();
}

void M10Buzzer::driveTone(uint8_t toggleEveryTicks) {
	toneDividerCounter += 1;
	if (toneDividerCounter >= toggleEveryTicks) {
		toneDividerCounter = 0;
		pinHigh = !pinHigh;
		HAL_GPIO_WritePin(buzzerPort, buzzerPin, pinHigh ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

void M10Buzzer::driveLow() {
	toneDividerCounter = 0;
	pinHigh = false;
	HAL_GPIO_WritePin(buzzerPort, buzzerPin, GPIO_PIN_RESET);
}

void serviceM10BuzzerTick() {
	if (activeBuzzer != nullptr) {
		activeBuzzer->serviceTick();
	}
}

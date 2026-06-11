#pragma once

#include "stm32l5xx_hal.h"
#include "buzzer_iface.hpp"

class M10Buzzer : public IBuzzer {
public:
	M10Buzzer(GPIO_TypeDef *buzzerPort, uint16_t buzzerPin);

	void buzzerOn() override;
	void buzzerOff() override;

	void init();
	void serviceTick();

		private:
			enum class Mode {
				DisarmedConfirm,
				ArmedConfirm,
				Silent,
			};

			GPIO_TypeDef * const buzzerPort;
			const uint16_t buzzerPin;
			Mode mode = Mode::Silent;
			bool buzzerEnabled = false;
			uint16_t patternTick = 0;
			uint8_t toneDividerCounter = 0;
			bool pinHigh = false;

		void startMode(Mode nextMode);
		void driveTone(uint8_t toggleEveryTicks);
		void driveLow();
	};

void serviceM10BuzzerTick();

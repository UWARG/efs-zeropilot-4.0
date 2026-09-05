#include "nvm_flash_message.hpp"

class BatteryLog : public AbstractMessage {
public:
	uint16_t voltage;
	uint16_t current;
	uint16_t power;

	static const uint16_t PACKED_SIZE = sizeof(uint16_t) * 3;

	BatteryLog(uint32_t id, uint16_t voltage, uint16_t current, uint16_t power);

	int unpack(const uint8_t* data, uint16_t len) override;
	int pack(uint8_t* data, uint16_t& len) override;

	uint16_t packed_size() const override { return PACKED_SIZE; };
};

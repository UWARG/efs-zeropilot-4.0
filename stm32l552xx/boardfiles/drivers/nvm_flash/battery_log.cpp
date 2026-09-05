#include "battery_log.hpp"

BatteryLog::BatteryLog(uint32_t id, uint16_t voltage, uint16_t current, uint16_t power)
	: AbstractMessage(id), voltage(voltage), current(current), power(power) {}

template<typename T>
static T readBE(const uint8_t*& p) {
	T val = 0;
	for (std::size_t i = 0; i < sizeof(T); i++)
		val = (val << 8) | *p++;
	return val;
}
template<typename T>
static void writeBE(uint8_t*& p, T val) {
	for (int i = (sizeof(T) - 1) * 8; i >= 0; i -= 8)
		*p++ = (val >> i) & 0xFF;
}

int BatteryLog::unpack(const uint8_t* data, uint16_t len) {
	if (len < PACKED_SIZE) return -1;

	const uint8_t* p = data;
	voltage = readBE<uint16_t>(p);
	current = readBE<uint16_t>(p);
	power = readBE<uint16_t>(p);

	return 0;
}

int BatteryLog::pack(uint8_t* data, uint16_t& len) {
	memset(data, 0xFF, PACKED_SIZE);
	uint8_t* p = data;

	writeBE(p, voltage);
	writeBE(p, current);
	writeBE(p, power);

	len = static_cast<uint16_t>(p - data);

	return 0;
}

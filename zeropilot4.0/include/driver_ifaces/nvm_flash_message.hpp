// NVM_Flash_Message.hpp
#pragma once

enum class MessageType {

};

class AbstractMessage {
public:
	uint32_t id;

	AbstractMessage(uint32_t id);
	virtual ~AbstractMessage() = default;

	virtual int unpack(const uint8_t* data, uint16_t len) = 0;
	virtual int pack(uint8_t* data, uint16_t& len) = 0;

	virtual uint16_t packed_size() const = 0;
};

#pragma once

#include <cstddef>
#include <array>

#define MAX_KEY_LENGTH 17
#define MAX_VALUE_LENGTH 10 // 1 for comma separator
#define MAX_LINE_LENGTH (MAX_KEY_LENGTH + MAX_VALUE_LENGTH + 2) // +2 for newline and separator (,)

// Managers that own config keys
enum class Owner_e : size_t {
    ATTITUDE_MANAGER = 0,
    TELEMETRY_MANAGER = 1,
    COUNT,
};

struct Param_t {
    char key[MAX_KEY_LENGTH] = {};
    float value;
    Owner_e owner;
    bool reboot_on_change = false;

    constexpr Param_t(const char *k, float v, Owner_e o, bool r = false) : value(v), owner(o), reboot_on_change(r) {
        for (size_t i = 0; i < MAX_KEY_LENGTH - 1; ++i) {
            key[i] = k[i];
            if (k[i] == '\0') break;
        }
    }
};

constexpr std::array<Param_t, 4> CONFIG_VALUES = {{
    {"baud_rate", 0.0, Owner_e::ATTITUDE_MANAGER, false}, // Example
    {"p", 100.0, Owner_e::ATTITUDE_MANAGER, false},
	{"i", 25.0, Owner_e::ATTITUDE_MANAGER, false},
	{"d", 10.0, Owner_e::ATTITUDE_MANAGER, false}
}};

static constexpr size_t NUM_KEYS = CONFIG_VALUES.size();

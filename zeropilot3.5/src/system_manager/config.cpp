#include "config.hpp"
#include "config_utils/config_defines.hpp"
#include <cstring>
#include <cstdio>
#include <string>

Config::Config(ITextIO* textIO) : textIO(textIO) {
  //blank
}

void Config::getXCharNum(float num, char final[MAX_VALUE_LENGTH + 1]) {
    char temp[128];
    if (num > 1e-6) {
      snprintf(temp, 128, "%f", num);

      //Remove trailing zeros after decimal
      char* dot = strchr(temp, '.');
      if (dot) {
          char* end = temp + strlen(temp) - 1;
          while(end > dot && *end == '0') --end;
          if (*end == '.') --end;
          *(end + 1) = '\0';
      }
    } else {
      snprintf(temp, 128, "%g", num);
    }

    if (strlen(temp) <= MAX_VALUE_LENGTH) {
        int len = snprintf(final, MAX_VALUE_LENGTH + 1, temp);
        // Add spaces
        for (int i = len; i < MAX_VALUE_LENGTH; i++) {
            final[i] = ' ';
        }
        final[MAX_VALUE_LENGTH] = '\0';
    } else {
        snprintf(final, MAX_VALUE_LENGTH + 1, "%.4e", num);
    }
}

int Config::init() {
  snprintf(configFile, 100, "config.txt");
  if (textIO->open(configFile, FA_OPEN_EXISTING) != 0) {
    return 1; // Error opening file
  }

  // Table was already initialized with default values so we loop and find all params in file
  for (size_t i = 0; i < NUM_KEYS; i++) {
    // findParam sets the value in config_table[i].value with the value read from the file if it exists
    // If the parameter does not exist in the file, it will not change the value in config_table[i].value
    findParam(config_table[i].key, config_table[i].value);
  }

  // Close the file after reading all parameters
  if (textIO->close() != 0) {
    return 2; // Error closing file
  }
  return 0;
}

int Config::findParam(const char param[MAX_KEY_LENGTH], float &val, int &tableIdx) {
  // move read/write pointer to param value
  char *key = nullptr, *value = nullptr;
  char msg[MAX_LINE_LENGTH];
  int length;
  
  while (textIO->read(msg, sizeof(msg)) != nullptr) {
    key = strtok(msg, ",");
    value = strtok(NULL, ",");

    if (!strcmp(param, key)) break;
  }

  if (key == nullptr || value == nullptr) return 2;
  if (strcmp(param, key)) return 2;

  length = (textIO->eof() != 0) ? strlen(value) : strlen(value) + 2; //accounting for \n

  textIO->seek(textIO->tell() - length);

  val = std::atof(value);

  // find idx of param in config table
  if (tableIdx != -1) {
    for (size_t i = 0; i < NUM_KEYS; i++) {
      if (!strcmp(param, config_table[i].key)) {
        tableIdx = static_cast<int>(i);
        break;
      }
    }
  }

  return 0;
}

int Config::findParam(const char param[100], float &val)  {
  int tableIdx = -1;
  return findParam(param, val, tableIdx);
}

int Config::findParam(const char param[100], int &tableIdx)  {
  float val;
  return findParam(param, val, tableIdx);
}

int Config::readParam(size_t key, float &val) {
  if (textIO->open(configFile, FA_READ | FA_WRITE) != 0) {
      return 1;
  }
  
  int res = findParam(config_table[static_cast<size_t>(key)].key, val);

  textIO->close();

  if (res != 0) {
    return 2;
  }

  return 0;
}

int Config::writeParam(size_t key, float newValue) {
  char strValue[MAX_VALUE_LENGTH + 1];
  getXCharNum(newValue, strValue);

  if (textIO->open(configFile, FA_READ | FA_WRITE) != 0) {
    return 1;
  }

  float val;
  int tblIdx;
  int writeRes;
  if (findParam(config_table[static_cast<size_t>(key)].key, val, tblIdx) == 0) {
    writeRes = textIO->write(strValue);
  } else {
    char keyStr[MAX_KEY_LENGTH + 1]; // 1 extra space for the comma
    snprintf(keyStr, MAX_KEY_LENGTH + 1, "%s,", config_table[static_cast<size_t>(key)].key);

    textIO->seek(textIO->fsize());
    if (textIO->tell() != 0) {
      textIO->write("\n");
    }

    textIO->write(keyStr);
    writeRes = textIO->write(strValue);
  }

  textIO->close();

  // on success, write to config table
  if (writeRes > 0) {
    config_table[tblIdx].value = newValue;
  }
  
  return 0;
}

int Config::writeParamByName(const char param[MAX_KEY_LENGTH], float newValue) {
  for (size_t i = 0; i < NUM_KEYS; i++) {
    if (!strcmp(param, config_table[i].key)) {
      return writeParam(i, newValue);
    }
  }
  return 2; // Parameter not found
}

size_t Config::getParamConfigKey(const char param[MAX_KEY_LENGTH]) {
  for (size_t i = 0; i < NUM_KEYS; i++) {
    if (!strcmp(param, config_table[i].key)) {
      return i;
    }
  }
  return NUM_KEYS; // Invalid key
}

Owner_e Config::getParamOwner(size_t key) {
  size_t index = static_cast<size_t>(key);
  if (index >= NUM_KEYS) {
      return Owner_e::COUNT; // Invalid owner
  }
  return config_table[index].owner;
}

Param_t Config::getParam(size_t key) {
    size_t index = static_cast<size_t>(key);
    if (index >= NUM_KEYS) {
        return Param_t{"ERROR", 0, Owner_e::COUNT, false}; // Return default Param_t if key is invalid for now
    }
    return config_table[index];
}

#if defined(SWO_LOGGING)
extern "C" {
  int __io_putchar(int ch) {
    return ITM_SendChar(ch);
  }
}
#endif
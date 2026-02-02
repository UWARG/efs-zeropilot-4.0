#pragma once

#include "config_keys.hpp"
#include "textio_iface.hpp"

class Config {
    private:
        char configFile[100];
        ITextIO *textIO;

        std::array<Param_t, NUM_KEYS> config_table = CONFIG_VALUES;

        /**
         * @brief Moves read/write pointer to parameter
         * @param param parameter to find
         * @param val buffer to store value at parameter
         * @param tableIdx buffer to store config table index of parameter
         * @retval Operation success
         */
        int findParam(const char param[100], float &val, int &tableIdx);
        int findParam(const char param[100], float &val);
        int findParam(const char param[100], int &tableIdx);

        void getXCharNum(float num, char final[MAX_VALUE_LENGTH + 1]);

    public:
        Config(ITextIO *textIO);

        /**
         * @brief Adds textIO object and reads config file
         * @retval Operation success
         */
        int init();

        /**
         * @brief reads parameter from config file
         * @param param parameter to read
         * @param val buffer to store value
         * @retval Operation success
         */
        int readParam(size_t key, float &val);

        /**
         * @brief writes parameter from config file
         * @param param parameter to write too
         * @param newValue updated value
         * @retval Operation success
         */
        int writeParam(size_t key, float newValue);

        /**
         * @brief writes parameter from config file by name
         * @param param parameter to write too
         * @param newValue updated value
         * @retval Operation success
         */
        int writeParamByName(const char param[MAX_KEY_LENGTH], float newValue);

        /**
         * @brief gets the index of a parameter by name
         * @param param parameter to get index for
         * @retval index of the parameter
         */
        size_t getParamConfigKey(const char param[MAX_KEY_LENGTH]);

        /**
         * @brief gets the owner of a parameter
         * @param key key of parameter to get owner for
         * @retval Owner of the parameter
         */
        Owner_e getParamOwner(size_t key);

        /**
         * @brief gets parameter struct by key
         * @param key parameter key to get
         * @retval Parameter struct
         */
        Param_t getParam(size_t key);
};
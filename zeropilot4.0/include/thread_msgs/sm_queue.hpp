#pragma once
#include <cstdint>
#include <string.h> 

typedef union SMMessageData {
    
} SMMessageData_t; // Messasage data

typedef struct SMMessage{
    enum{ // Type of data being sent to SM
        
    } dataType;
    SMMessageData_t tmMessageData; // Message data
    uint32_t timeBootMs = 0; // When the message was generated
} SMMessage_t;
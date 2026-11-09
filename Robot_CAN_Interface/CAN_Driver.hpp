/****************************************************************
File Name  : CAN_Driver.hpp
Description:
****************************************************************/

#ifndef CAN_DRIVER_HPP
#define CAN_DRIVER_HPP


/****************************************************************
*   INCLUDES
****************************************************************/

#include <Arduino.h>
#include <cstring>
#include <stdint.h>

/****************************************************************
*   DEFINES, CONSTANTS, ENUMS, STRUCTS
****************************************************************/

/*******************************
*   Constants
*******************************/

// CAN Parameters

// Defines what the number of bytes of data the CAN message can have
const uint8_t MAX_BYTES_OF_DATA_IN_MSG = 8;


/*******************************
*   Structures
*******************************/

typedef struct
{
    uint32_t msgId;
    uint8_t dlc;
    uint8_t data[MAX_BYTES_OF_DATA_IN_MSG];

} CAN_Msg_t;


/****************************************************************
*   GLOBAL VARIABLES
****************************************************************/

/****************************************************************
*   GLOBAL FUNCTION PROTOTYPES
****************************************************************/

bool CAN_Driver_Init(void);
bool CAN_Driver_SendMsg(uint32_t msgId, uint8_t dlc, const uint8_t *outgoingMsgData);
bool CAN_Driver_ReceiveMsg(CAN_Msg_t *incomingMsg);

#endif // CAN_DRIVER_HPP
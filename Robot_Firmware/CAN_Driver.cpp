/****************************************************************
*   File Name  : CAN_Driver.cpp
*   Description:
****************************************************************/

/****************************************************************
*   INCLUDES
****************************************************************/

#include "CAN_Driver.hpp"

// Library Includes
#include <SPI.h>
#include <mcp2515.h>

/****************************************************************
*   DEFINES, CONSTANTS
****************************************************************/

// Pin Definitions
const uint8_t CAN_INT_PIN = 2;
const uint8_t CAN_CS_PIN = 4;

// CAN Parameters
const uint16_t MAX_CAN_ID = 0x7FF; // Based on CAN Standard 11 bit identifier limit

// Uses built in enums for valid baud rates and clock frequencies
const CAN_SPEED BAUD_RATE = CAN_500KBPS;
const CAN_CLOCK MCP_CLOCK_FREQ = MCP_8MHZ;

/****************************************************************
*   GLOBAL VARIABLES
****************************************************************/

/****************************************************************
*   LOCAL VARIABLES (declare as static)
****************************************************************/

static MCP2515 mcp(CAN_CS_PIN);

/****************************************************************
*   LOCAL FUNCTION PROTOTYPES (declare as static)
****************************************************************/

static bool configureMCP(void);

/****************************************************************
*   GLOBAL FUNCTIONS
****************************************************************/

/**************************************************
*   Function: CAN_Driver_Init
*   Description:
**************************************************/

bool CAN_Driver_Init(void)
{
    // Initialize MCP
    bool mcpConfigSuccessful = configureMCP();
    if(mcpConfigSuccessful == false)
    {
        return false;
    }

    // Additional initialization stuff
    // Here

    // No initialization failures
    return true;
}


/**************************************************
*   Function: CAN_Driver_SendMsg
*   Description:
**************************************************/

bool CAN_Driver_SendMsg(uint32_t msgId, uint8_t dlc, const uint8_t *outgoingMsgData)
{
    // Checking to see if outgoing message parameters are valid
    if(msgId > MAX_CAN_ID || dlc > MAX_BYTES_OF_DATA_IN_MSG || outgoingMsgData == nullptr)
    {
        return false;
    }

    // Creating library specific frame to interface correctly with library functions
    struct can_frame libOutgoingMsg = {};

    // Converting generic frame to library frame
    libOutgoingMsg.can_id = msgId;
    libOutgoingMsg.can_dlc = dlc;
    memcpy(libOutgoingMsg.data, outgoingMsgData, libOutgoingMsg.can_dlc);

    // Sending message
    MCP2515::ERROR sendMsgStatus = mcp.sendMessage(&libOutgoingMsg);
    if(sendMsgStatus != MCP2515::ERROR_OK)
    {
        Serial.printf("CAN TX Error: ID 0x%03lX, Code %d\r\n", (unsigned long) msgId, (int)sendMsgStatus);
        
        return false;
    }

    return true;
}


/**************************************************
*   Function: CAN_Driver_ReceiveMsg
*   Description:
**************************************************/

bool CAN_Driver_ReceiveMsg(CAN_Msg_t *incomingMsg)
{
    // Checking to see if incoming message struct is valid
    if(incomingMsg == nullptr)
    {
        return false;
    }

    // Creating library specific frame to interface correctly with library functions
    struct can_frame libIncomingMsg = {};

    // Receive message
    MCP2515::ERROR receiveMsgStatus = mcp.readMessage(&libIncomingMsg);
    if(receiveMsgStatus == MCP2515::ERROR_NOMSG)
    {
        return false;
    }
    else if(receiveMsgStatus != MCP2515::ERROR_OK)
    {
        Serial.printf("Error receiveing message: Code %d\r\n", (int)receiveMsgStatus);
        return false;
    }
    
    // Checking to see whether received data is valid
    if(libIncomingMsg.can_id > MAX_CAN_ID || libIncomingMsg.can_dlc > MAX_BYTES_OF_DATA_IN_MSG)
    {
        return false;
    }

    // Converting library frame to generic frame
    incomingMsg->msgId = libIncomingMsg.can_id;
    incomingMsg->dlc = libIncomingMsg.can_dlc;
    memcpy(incomingMsg->data, libIncomingMsg.data, incomingMsg->dlc);

    return true;
}


/****************************************************************
*   LOCAL FUNCTIONS
****************************************************************/

/**************************************************
*   Function: configureMCP
*   Description: Initializes MCP chip using built in
*   function, captures return codes, and prints
*   debug information based on the return codes.
**************************************************/

bool configureMCP(void)
{
    // Return code variable for holding codes from the different initialization functions
    MCP2515::ERROR returnCode;
    
    // Resetting MCP
    returnCode = mcp.reset();
    if(returnCode != MCP2515::ERROR_OK)
    {
        Serial.printf("MCP Reset Error: Return Code %d\r\n", (int)returnCode);
        return false;
    }

    // Setting bitrate on MCP
    returnCode = mcp.setBitrate(BAUD_RATE, MCP_CLOCK_FREQ);
    if(returnCode != MCP2515::ERROR_OK)
    {
        Serial.printf("MCP Set Bitrate Error: Code %d\r\n", (int)returnCode);
        return false;
    }

    // Enabling normal mode on MPC
    returnCode = mcp.setNormalMode();
    if(returnCode != MCP2515::ERROR_OK)
    {
        Serial.printf("MCP Set Mode Error: Code %d\r\n", (int)returnCode);
        return false;
    }

    // If no errors occured then MCP initialized successfully
    Serial.println("MCP Successfully Initialized");
    return true;
}
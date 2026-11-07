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
const uint8_t CAN_CS_PIN = 10;

// CAN Parameters

// Uses built in enums for valid baud rates and clock frequencies
const CAN_SPEED BAUD_RATE = CAN_500KBPS;
const CAN_CLOCK MCP_CLOCK_FREQ = MCP_16MHZ;

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

bool configureMCP(void);

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

bool CAN_Driver_SendMsg(CAN_Msg_t *outgoingMsg)
{
    // Checking to see if outgoing message is valid
    if(outgoingMsg == nullptr || outgoingMsg->dlc > 8)
    {
        return false;
    }

    // Creating library specific frame to interface correctly with library functions
    struct can_frame libOutgoingMsg;

    // Converting generic frame to library frame
    libOutgoingMsg.can_id = outgoingMsg->msgId;
    libOutgoingMsg.can_dlc = outgoingMsg->dlc;
    memcpy(libOutgoingMsg.data, outgoingMsg->data, libOutgoingMsg.can_dlc);

    // Sending message
    MCP2515::ERROR sendMsgStatus = mcp.sendMessage(&libOutgoingMsg);
    if(sendMsgStatus != MCP2515::ERROR_OK)
    {
        Serial.printf("Error sending message: Code %dr\n", (int)sendMsgStatus);
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
    // Checking to see if incoming message is valid
    if(incomingMsg == nullptr)
    {
        return false;
    }

    // Creating library specific frame to interface correctly with library functions
    struct can_frame libIncomingMsg = {};

    // Receive message
    MCP2515::ERROR receiveMsgStatus = mcp.readMessage(&libIncomingMsg);
    if(receiveMsgStatus != MCP2515::ERROR_OK)
    {
        Serial.printf("Error receiveing message: Code %dr\n", (int)receiveMsgStatus);
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
        Serial.printf("MCP Reset Error: Return Code %dr\n", (int)returnCode);
        return false;
    }

    // Setting bitrate on MCP
    returnCode = mcp.setBitrate(BAUD_RATE, MCP_CLOCK_FREQ);
    if(returnCode != MCP2515::ERROR_OK)
    {
        Serial.printf("MCP Set Bitrate Error: Code %dr\n", (int)returnCode);
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
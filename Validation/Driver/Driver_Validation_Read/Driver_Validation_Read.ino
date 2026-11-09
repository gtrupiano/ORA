// Read
#include "CAN_Driver.hpp"

CAN_Msg_t incomingMsg;

void setup()
{
    Serial.begin(115200);

    if(CAN_Driver_Init() == false)
    {
        delay(2000);
        ESP.restart(); 
    }
}

void loop()
{
    bool msgPresent = CAN_Driver_ReceiveMsg(&incomingMsg);

    if(msgPresent)
    {
        Serial.print(incomingMsg.msgId, HEX);
        Serial.print(" "); 
        Serial.print(incomingMsg.dlc, HEX);
        Serial.print(" "); 

        for(uint8_t i = 0; i < incomingMsg.dlc; i++)
        {
            Serial.print(incomingMsg.data[i], HEX);
            Serial.print(" "); 
        }
        
        Serial.println();
    }
}
// Write
#include "CAN_Driver.hpp"

CAN_Msg_t test1;
CAN_Msg_t test2;

void setup()
{
    Serial.begin(115200);

    if(CAN_Driver_Init() == false)
    {
        delay(2000);
        ESP.restart(); 
    }

    test1.msgId = 0x036;
    test1.dlc = 8;
    test1.data[0] = 0x0E;
    test1.data[1] = 0x00;
    test1.data[2] = 0x00;
    test1.data[3] = 0x08;
    test1.data[4] = 0x01;
    test1.data[5] = 0x00;
    test1.data[6] = 0x00;
    test1.data[7] = 0xA0;

    test2.msgId = 0x064;
    test2.dlc = 8;
    test2.data[0] = 0x8E;
    test2.data[1] = 0x87;
    test2.data[2] = 0x32;
    test2.data[3] = 0xFA;
    test2.data[4] = 0x26;
    test2.data[5] = 0x8E;
    test2.data[6] = 0xBE;
    test2.data[7] = 0x86;
}

void loop()
{
    bool msg1SendSuccessful = CAN_Driver_SendMsg(test1.msgId, test1.dlc, &test1.data[0]);
    if(msg1SendSuccessful == false)
    {
        Serial.println("Failed to send Test 1");
    }
    delay(10);

    bool msg2SendSuccessful = CAN_Driver_SendMsg(test2.msgId, test2.dlc, &test2.data[0]);
    if(msg2SendSuccessful == false)
    {
        Serial.println("Failed to send Test 2");
    }
    delay(10);
}
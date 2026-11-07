#include "ORA_CAN.hpp"

void setup()
{
    Serial.begin(115200);

    ORA_CAN_Init();
}

void loop()
{
    ORA_CAN_Loop();
}
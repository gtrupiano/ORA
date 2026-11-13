#include "ORA_CAN.hpp"
#include "ORA_IMU.hpp"
#include "tasks.hpp"

// Task initialization for period and current time (initialized to 0 for start)
Task_t tasks[MAX_NUM_OF_TASKS] = {
//  {period, lastTime} 
    {CAN_TASK_PERIOD_MS, 0},
    {IMU_TASK_PERIOD_MS, 0}
};


void setup()
{
    Serial.begin(115200);

    // Initialize CAN
    if(ORA_CAN_Init() == false)
    {
        while(1)
        {
            Serial.println("CAN Initialization Failed");
            delay(5000);
        }
    }

    // Initialize IMU
    // if(ORA_IMU_Init() == false)
    // {
    //     while(1)
    //     {
    //         Serial.println("IMU Initialization Failed");
    //         delay(5000);
    //     }
    // }

    Serial.println("All Initialization Passed");
    
    delay(50);
}

void loop()
{
    // Capture current tie
    unsigned long currentTime = millis();

    // If elapsed time is greater than CAN task time, then execute main CAN loop
    if((currentTime - tasks[CAN_TASK].lastTime) >= tasks[CAN_TASK].period)
    {
        // Main CAN loop
        ORA_CAN_Loop();

        // Updating time since last CAN loop
        tasks[CAN_TASK].lastTime = currentTime;
    }

    // If elapsed time is greater than IMU task time, then execute main IMU loop
    // if((currentTime - tasks[IMU_TASK].lastTime) >= tasks[IMU_TASK].period)
    // {
    //     // Main IMU loop
    //     ORA_IMU_Loop();

    //     // Updating time since last IMU loop
    //     tasks[IMU_TASK].lastTime = currentTime;
    // }
}
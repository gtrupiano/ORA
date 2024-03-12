#include <PS4Controller.h>
#include <mcp_can.h>
#include <SPI.h>


#define CAN0_INT 2    // Set INT to pin 2 (This is the Interupt pin)
MCP_CAN CAN0(10);   // Set CS to pin 10
// add definitions for LED's, IMU, etc.

void setup()
{
  pinMode(CAN0_INT, INPUT);
  Serial.begin(115200);
  PS4.begin();
  Serial.println("Ready");

  // Add initialization for LED's, IMU, etc.

  // Continuously tries to establish connection to MCP chip until it does
  while(1)
  {
    // Tries to initalizes MCP CAN chip with clock of 8MHz and data transfer speed is 500KB/second
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    {
      // CAN BUS is initalized, then break out of while loop
      Serial.println("MCP2515 Initialized Successfully!");
      break;
    }
    else 
    {
      // Stays in infinite loop but waits 20ms to give time for bus to be initialized
      Serial.println("Error Initializing MCP2515...");
      delay(20);
    }
  }
  
}

void loop()
{
  // In this, take the inputs from each button / stick on the controller and assign it a variable (EStop, State, Movement, etc)

}

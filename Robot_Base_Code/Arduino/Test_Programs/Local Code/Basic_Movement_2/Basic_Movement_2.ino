#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2    // Set INT to pin 2
MCP_CAN CAN0(10);   // Set CS to pin 10
byte node_id = 1;   

void setup() 
{
  Serial.begin(9600);

  // Initialize MCP2515 module
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) 
  {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else 
  {
    Serial.println("Error Initializing MCP2515...");
    while(1);
  }
   // Put axis into closed loop control state
  CAN0.sendMsgBuf((node_id << 5 | 0x07), 0, 4, (byte*)"\x08");

  Serial.println("Closed Loop Control State Complete");
}

void loop()
{
  // Set velocity to 1.0 turns/s
  float velocity = 3.0f;
  float torque_feedforward = 0.1;
  byte velData[] = { (byte)velocity, (byte)torque_feedforward };
  CAN0.sendMsgBuf((node_id << 5 | 0x0D), 0, 4, velData);
  Serial.println("Move?");
  delay(1000);
}

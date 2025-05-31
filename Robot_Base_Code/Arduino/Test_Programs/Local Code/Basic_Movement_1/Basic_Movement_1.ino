#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2    // Set INT to pin 2
MCP_CAN CAN0(10);   // Set CS to pin 10

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
}

void loop()
{
  byte node_id = 1;

  // Put axis into closed loop control state
  CAN0.sendMsgBuf((node_id << 5 | 0x07), 0, 4, (byte*)"\x08");

  Serial.println("Step 1 complete");


  // Set velocity to 1.0 turns/s
  float velocity = 1.0;
  float torque_feedforward = 0.1;
  byte velData[] = [velocity, torque_feedforward];
  CAN0.sendMsgBuf((node_id << 5 | 0x0D), 0, 2, velData);

  delay(1000);

  // Set velocity to -1.0 turns/s
  velocity = -1.0;
  velData = [velocity, torque_feedforward];
  CAN0.sendMsgBuf((node_id << 5 | 0x0D), 0, 2, velData);

  delay(1000);
}
/*
  // Receive and print encoder feedback
  unsigned long rxId;
  byte len = 0;
  byte buf[8];

  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    CAN0.readMsgBuf(&len, buf);
    rxId = CAN0.getCanId();

    if (rxId == (node_id << 5 | 0x09)) {
      float pos, vel;
      memcpy(&pos, buf, sizeof(float));
      memcpy(&vel, buf + sizeof(float), sizeof(float));
      Serial.print("pos: ");
      Serial.print(pos, 3);
      Serial.print(" [turns], vel: ");
      Serial.print(vel, 3);
      Serial.println(" [turns/s]");
    }
    
  }
} 
*/
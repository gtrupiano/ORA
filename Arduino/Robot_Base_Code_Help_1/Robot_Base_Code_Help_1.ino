#include <PS4Controller.h>
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2    // Set INT to pin 2 (This is the Interupt pin)
MCP_CAN CAN0(10);   // Set CS to pin 10 (This is the Chip select)

// ODrive & Can Definitions
#define CAN_BAUDRATE 500000
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1

double verticalMov = 0;
double horizontalMov = 0;
bool EStop = false; // Change to false initially

void setup()
{
  pinMode(CAN0_INT, INPUT);
  Serial.begin(115200);
  PS4.begin();
  Serial.println("Ready");

  // Continuously tries to establish connection to MCP chip until it does
  while(1)
  {
    // Tries to initialize MCP CAN chip with clock of 8MHz and data transfer speed is 500KB/second
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    {
      Serial.println("MCP2515 Initialized Successfully!");
      break;
    }
    else 
    {
      Serial.println("Error Initializing MCP2515...");
      delay(20);
    }
  }
}

void loop() {
  if(PS4.isConnected()) 
  {
    verticalMov = PS4.LStickY();
    horizontalMov = PS4.LStickX();
    EStop = PS4.R1();
  }
  else
  {
    verticalMov = 0;
    horizontalMov = 0;
    EStop = false; // Ensure EStop is false when PS4 is not connected
  }

  if(EStop)
  {
    // Send EStop command to ODrive
    ODriveEStop();
  }
  else
  {
    // Set values of joysticks to velocity using CAN commands
    ODriveMovement(verticalMov, horizontalMov);
  }
}

// Function to send commands to ODrive via CAN bus
void ODriveMovement(double verticalVelocity)
{
  // Construct CAN message and send it
  float torque_feedforward = 0.1;
  byte velData[] = {verticalVelocity, torque_feedforward};
  CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x0D), 0, 2, velData);
}



void ODriveEStop()
{
  CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x02), 0, 0, nullptr);
  CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x02), 0, 0, nullptr);
}


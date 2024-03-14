#include <PS4Controller.h>
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2    // Set INT to pin 2 (This is the Interupt pin)
MCP_CAN CAN0(10);   // Set CS to pin 10 (This is the Chip select)

#define CAN_BAUDRATE 500000
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1

// Add pin definitions for LED,IMU,etc.

double verticalMov = 0;
double horizontalMov = 0;
bool EStop = false;

void setup()
{
  pinMode(CAN0_INT, INPUT);
  // Assign pin modes for LED,IMU,etc.
  Serial.begin(115200);
  // MAC Address for my controller
  PS4.begin("bc:03:58:28:67:42");
  Serial.println("Ready");

  // Continuously tries to establish connection to MCP chip until it does
  while(1) // Fix?
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

void loop() 
{
  if(PS4.isConnected()) // could combine with estop if not sure if it's easier to read this way or not. Might be easier for them to be seperate
  {
    verticalMov = PS4.LStickY(); // Possibly add some error detection (debounce)?
    horizontalMov = PS4.LStickX();
    EStop = PS4.R1();

    Serial.print("X: ");
    Serial.print(verticalMov);
    Serial.print("  Y: ");
    Serial.println(horizontalMov);
  }
  else
  {
    verticalMov = 0;
    horizontalMov = 0;
    EStop = false;
  }

  if(EStop)
  {
    // Send EStop command to ODrive
    ODriveEStop();
    Serial.println("EStop Activated");
  }
  else
  {
    // Send CAN command to put axis into closed loop control state
    ODriveControlState()
    Serial.println("Put into closed loop control state");

    // Set values of joysticks to velocity using CAN commands
    ODriveMovement(verticalMov, horizontalMov);
    /*
    Serial.println("Moving at: ");
    Serial.print("X: ");
    Serial.print(horizontalMov);
    Serial.print("  Y: ");
    Serial.print(verticalMov);
    */
  }
}

// Function to send commands to ODrive via CAN bus
void ODriveMovement(double verticalVelocity, double horizontalVelocity)
{
    // Scales inputs
    verticalVelocity = constrain(verticalVelocity, -1.0, 1.0);
    horizontalVelocity = constrain(horizontalVelocity, -1.0, 1.0);

    // Standard for differential drive control
    double leftMotorVel = verticalVelocity - horizontalVelocity;
    double rightMotorVel = verticalVelocity + horizontalVelocity;
    
    int maxRPM = 4600; // Change later if we need
    int maxVelocity = 5 // IGVC rules, should be m/s units but not sure
    int leftMotorRPM = leftMotorVel * maxVelocity * maxRPM; // possibly change to double but not sure if data length can handle that precision
    int rightMotorRPM = rightMotorVel * maxVelocity * maxRPM;

    // Send velocity commands to left and right motors
    CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x0D), 0, 4, (byte*)&leftMotorRPM); // check 4
    CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x0D), 0, 4, (byte*)&rightMotorRPM);
}


void ODriveEStop() 
{
  CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x02), 0, 0, nullptr);
  CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x02), 0, 0, nullptr);
}

void ODriveControlState()
{
  // Send CAN command to put axis into closed loop control state
    CAN0.sendMsgBuf((ODRV0_NODE_ID << 5 | 0x07), 0, 4, (byte*)"\x08");
    CAN0.sendMsgBuf((ODRV1_NODE_ID << 5 | 0x07), 0, 4, (byte*)"\x08");
}
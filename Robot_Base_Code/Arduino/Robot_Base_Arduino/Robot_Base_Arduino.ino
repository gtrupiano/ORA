#include "MCP2515.h"
#include "ODriveMCPCAN.hpp" // Includes ODriveCAN.h and MCP2515.h
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

#define CAN_BAUDRATE 500000
#define CAN0_INT 2 // This is the Interrupt pin
#define ODRV0_NODE_ID 0 // Left Motor
#define ODRV1_NODE_ID 1 // Right Motor

MCP2515Class& can_intf = CAN;

MCP_CAN CAN0(10);

#define MCP2515_CS 10
#define MCP2515_INT 2
#define MCP2515_CLK_HZ 8000000
#define EStop 8 // Hardware EStop

// Robot State Declarations
#define EStopButtonIndicator 1
#define AutonButtonIndicator 5

// Battery Voltage Detection Declaration
#define Battery_Voltage A0

// Physical perameters of robot
#define Wheel_Radius 0.184f // meters
#define Wheel_Seperation 0.7112f // meters
#define Max_Velocity 2.2352f //  Speed limit m/s according to IGVC rules

// Global Variable Declaration

// Controller Variables
int8_t angularMov = 0;
int8_t linearMov = 0;
bool EStopButton = false;
bool EStopState = false;
bool prevEStopButton = false;
bool AutonButton = false;
bool AutonState = false;
bool prevAutonButton = false;

// CAN IDs of the messages we are interested in
const unsigned long rightWheelSpeedID = 2;
const unsigned long leftWheelSpeedID = 3;
const unsigned long rightEncoderID = 4;
const unsigned long leftEncoderID = 5;
const unsigned long AutonStateID = 6;

// Buffers to store incoming CAN data
byte msgWheelSpeed[8];

// Variables to store wheel speeds
// They are delared as floats in the ODriveCAN library
float rightWheelSpeed;
float leftWheelSpeed;

// Variables to store encoder wheel speed 
// They are delared as floats in the ODriveCAN library
float odrv0EncoderVel; 
float odrv1EncoderVel;

// Function Prototypes
static inline void receiveCallback(int);
bool setupCan();
void onHeartbeat(Heartbeat_msg_t&, void*);
void onFeedback(Get_Encoder_Estimates_msg_t&, void*);
void onCanMessage(const CanMsg&);

// Data type created for ODrive (Consists of last heartbeat, if heartbeat was received, encoder feedback, and if there is feedback)
struct ODriveUserData 
{
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Initialize ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Initalize odrv0 as ODriveCAN object with respective node ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Initalize odrv1 as ODriveCAN object with respective node ID
ODriveCAN* odrives[] = {&odrv0, &odrv1}; // Make sure all ODriveCAN instances are accounted for here

// Initalizes variables to hold ODrive configuation data
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;


void setup() 
{
  Serial.begin(115200);

  Wire.begin(8);

  pinMode(EStop, INPUT);

  pinMode(CAN0_INT, INPUT);

  pinMode(EStopButtonIndicator, OUTPUT);
  pinMode(AutonButtonIndicator, OUTPUT);

  pinMode(Battery_Voltage, INPUT);


  // Wait for up to 3 seconds for the serial port to be opened on the PC side
  for (int i = 0; i < 30 && !Serial; ++i) 
  {
    delay(100);
  }
  delay(200);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);


  if (!setupCan()) 
  {
    Serial.println("CAN failed to initialize: reset required");
    while (true);
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat) 
  {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("ODrive Detected");

  // request bus voltage and current (1sec timeout)
  Serial.println("Attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) 
  {
    Serial.println("vbus request failed!");
    while (true);
  }
  else if (!odrv1.request(vbus, 1))
  {
    Serial.println("vbus request failed!");
    while (true);
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  ODriveControlState();
  Serial.println("ODrive Setup Completed: ");
}

void loop() 
{
  // This is required on some platforms to handle incoming feedback CAN messages
  pumpEvents(can_intf);

  // Fetches values from controller that are sent over I2C
  fetchControllerData();

  // EStop Logic

  // Falling Edge Detection (Goes from High to Low)
  // EStop (If statement) only activates when button is pressed
  if (prevEStopButton == HIGH && EStopButton == LOW)
  {
    // Toggles EStop State
    // If state was off before then the action of pressing the button means that it is enabled
    if(!EStopState)
    {
      EStopState = true;
      Serial.println("EStop True");
    }

    else
    {
      EStopState = false;
      Serial.println("EStop False");
    }
  }
  
  prevEStopButton = EStopButton;
  

  if(EStopState)
  {
    ODriveEStop();
    digitalWrite(EStopButtonIndicator, HIGH);
  }
  else
  {
    ODriveControlState();
    ODriveMovement(angularMov, linearMov);
    digitalWrite(EStopButtonIndicator, LOW);
  }

  // Auton Logic

  // EStop has priority
  if(!EStopState)
  {
    // Falling Edge Detection (Goes from High to Low)
    // Auton (If statement) only activates when button is pressed
    if (AutonButton == LOW && prevAutonButton == HIGH)
    {
      // If state was off before then the action of pressing the button means that it is enabled
      if(!AutonState) 
      {
        AutonState = true;
      }
      
      else 
      {
        AutonState = false;
      }
    }
    prevAutonButton = AutonButton;
  }

  // Checks AutonState data (which is determined by the toggle of the controller button)
  if(AutonState)
  {
    autonMovement();
    digitalWrite(AutonButtonIndicator, HIGH);
    //Serial.println("Auton On");
  }
  else 
  {
    digitalWrite(AutonButtonIndicator, LOW);
    //Serial.println("Auton Off");
  }

  autonEncoderData();

  // Checks hardware EStop for light indication changes
  if(digitalRead(EStop) == LOW)
  {
    digitalWrite(EStopButtonIndicator, HIGH);
  } 

  else
  {
    digitalWrite(EStopButtonIndicator, LOW);
  }

  // Prints ODrive Velocities and Position via Encoders
  //ViewODriveEncoderData();  

  delay(50);
}

void fetchControllerData()
{
  // Requests data from slave device 8 (ESP32)
  Wire.requestFrom(8, sizeof(int8_t) + sizeof(int8_t) + sizeof(bool) + sizeof(bool));

  // Checks to see if data is recieved over I2C. If so sets values from controller to predefined variables
  if(Wire.available() >= sizeof(int8_t) + sizeof(int8_t) + sizeof(bool) + sizeof(bool))
  {
    // Read rotational movement from left joystick (Controller only sends 8 bit values)
    linearMov = Wire.read();

    // Read linear movement from left joystick
    angularMov = Wire.read();

    // Read EStopButton from right bumper (Controller only sends 8 bit values)
    EStopButton = Wire.read();

    // Read Robot State from left bumper
    AutonButton = Wire.read();
    
    // Reads controller values from ESP-32
    // ViewControllerData()
  } 
  else // make this engage estop
  {
    angularMov = 0;
    linearMov = 0;
    EStopState = true; // check if this logic makes sure if estop actually engages and doesn't mess up 
    AutonState = false;
    Serial.println("Insufficient bytes received");
  }
}

void ODriveMovement(int8_t angularVelIn, int8_t linearVelIn)
{
  // Normalize inputs to directional velocity vectors
  float angularVelocity = (float)angularVelIn / 127.0;
  float linearVelocity = (float)linearVelIn / 127.0;
  
  // Radians Per Second
  float leftMotorTPS = (1 / Wheel_Radius) * (linearVelocity - ((Wheel_Seperation * angularVelocity) / 2));
  float rightMotorTPS = (1 / Wheel_Radius) * (linearVelocity + ((Wheel_Seperation * angularVelocity) / 2));

  // Converting to Turns per second
  // Convert wheel speeds from rad/s to turns per second with scaling factor 21
  leftMotorTPS = (leftMotorTPS / (2 * M_PI)) * 21;
  rightMotorTPS = (rightMotorTPS / (2 * M_PI)) * -21;
  
  //Serial.println("Angular Velocity: " + (String) angularMov + "\tLinear Velocity: " + (String) linearMov + "\tLeft Motor: " + (String) leftMotorTPS + "\tRight Motor: " + (String) rightMotorTPS);
  
  // Send velocity CAN commands to left and right motors
  odrv0.setVelocity(leftMotorTPS);
  odrv1.setVelocity(rightMotorTPS);
}

// Sends command over CAN to ODrive to initiate EStop
void ODriveEStop()
{
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE || odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) 
  {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);

    odrv1.clearErrors();
    delay(1);
    odrv1.setState(ODriveAxisState::AXIS_STATE_IDLE);

    for (int i = 0; i < 15; ++i) 
    {
      delay(10);
      pumpEvents(can_intf); // Pump events for 150ms (Look at example for full explanation)
    }
  }
  Serial.println("EStop Activated");
}

// Send CAN command to put axis into closed loop control state
void ODriveControlState() 
{
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL || odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) 
  {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    odrv1.clearErrors();
    delay(1);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; ++i) 
    {
      delay(10);
      pumpEvents(can_intf);  // Pump events for 150ms (Look at example for full explanation)
    }
  }
  //Serial.println("Closed loop control state active");
}

// Prints position and velocity from each ODrive (For debugging)
void ViewODriveEncoderData()
{
  if (odrv0_user_data.received_feedback) 
  {
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    Serial.print("ODrive 0 Velocity:");
    Serial.println(feedback.Vel_Estimate);
    Serial.println("------------------------");
  }

  if (odrv1_user_data.received_feedback) 
  {
    Get_Encoder_Estimates_msg_t feedback = odrv1_user_data.last_feedback;
    odrv1_user_data.received_feedback = false;
    Serial.print("ODrive 1 Velocity:");
    Serial.println(feedback.Vel_Estimate);
    Serial.println("------------------------");
  }
}

// Prints received values of controller from ESP-32 (For debugging)
void ViewControllerData()
{
  Serial.print("Received from ESP32 - Angular Movement: ");
  Serial.println(angularMov);

  Serial.print("Received from ESP32 - Linear Movement: ");
  Serial.println(linearMov);

  Serial.print("Received from ESP32 - EStop Button: ");
  Serial.println(EStopButton);

  Serial.print("Received from ESP32 - Autonomous Button: ");
  Serial.println(AutonButton);

  Serial.println("------------------------");
}

void stateCommunication()
{
  byte AutonStateBA = (byte)AutonState;

  byte AutonStateMsgStatus = CAN0.sendMsgBuf(AutonStateID, 1, AutonStateBA);
}


// Receives data from path planner and send motor speeds to ODrives
void autonMovement() // Assuming data is given in rad/sec
{
  unsigned long receivedID;
  byte msgLen;

  // Check if a CAN messages are available (Now looks for all messages in the bus rather than just per function call)
  while(CAN_MSGAVAIL == CAN0.checkReceive()) 
  {
    // Read the CAN messages in the RX buffer
    CAN0.readMsgBuf(&receivedID, &msgLen, msgWheelSpeed);
    
    if(msgLen == 8)
    {
      // Check if the received message ID matches the right wheel speed ID
      if(receivedID == rightWheelSpeedID)
      {
        memcpy(&rightWheelSpeed, msgWheelSpeed, sizeof(float));
      }

      // Check if the received message ID matches the left wheel speed ID
      else if(receivedID == leftWheelSpeedID)
      {
        memcpy(&leftWheelSpeed, msgWheelSpeed, sizeof(float));
      }

    }
    else 
    {
      Serial.println("Received message length does not match expected float size.");
    }
  }

  leftWheelSpeed = (leftWheelSpeed / ((float)(2f * M_PI))) * 21f;
  rightWheelSpeed = (rightWheelSpeed / ((float)(2f * M_PI))) * -21f;
  
  odrv0.setVelocity(leftWheelSpeed);
  odrv1.setVelocity(rightWheelSpeed);
}

// Obtains Encoder data from each ODrive and sends it over CAN to it's respective ID
void autonEncoderData()
{
  // Obtaining ODrive Encoder data
  if (odrv0_user_data.received_feedback) 
  {
    Get_Encoder_Estimates_msg_t feedbackOdrv0 = odrv0_user_data.last_feedback;
    odrv0EncoderVel = feedbackOdrv0.Vel_Estimate;
    odrv0_user_data.received_feedback = false;
  }

  if (odrv1_user_data.received_feedback) 
  {
    Get_Encoder_Estimates_msg_t feedbackOdrv1 = odrv1_user_data.last_feedback;
    odrv1EncoderVel = feedbackOdrv1.Vel_Estimate;
    odrv1_user_data.received_feedback = false;
  }

  // Sending that data over CAN using left and right wheel IDs

  // leftWheelSpeedID,  ODrv 0 is the left wheel
  // rightWheelSpeedID, ODrv 1 is the right wheel
  
  byte* odrv0EncoderVelBA = (byte*)&odrv0EncoderVel;
  byte* odrv1EncoderVelBA = (byte*)&odrv1EncoderVel;
  

  byte leftMsgStatus = CAN0.sendMsgBuf(leftEncoderID, 4, odrv0EncoderVelBA); // length might be incorrect.
  byte rightMsgStatus = CAN0.sendMsgBuf(rightEncoderID, 4, odrv1EncoderVelBA);

  // Check the feedback message to see if the message was sent successfully (was the message acknowledged)
  if (leftMsgStatus != CAN_OK) 
  {
    Serial.println("Error Sending Left Encoder Data...");
  }

  if (rightMsgStatus != CAN_OK) 
  {
    Serial.println("Error Sending Right Encoder Data...");
  }
  else
  {
    //Serial.println("Message Sent Successfully!");
  }
}

// Below are all of the ODriveCAN Specific function declarations

static inline void receiveCallback(int packet_size)
{
  if (packet_size > 8) 
  {
    return;
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

// Configure and initialize the CAN bus interface
bool setupCan() 
{
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) // Should this be MCP_Normal?
  {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else
  {
    Serial.println("Error Initializing MCP2515...");
  }
  
  CAN0.setMode(MCP_NORMAL); // Should this be MCP_Any?


  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE))
  {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) 
{
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) 
{
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) 
{
  for (auto odrive: odrives) 
  {
    onReceive(msg, *odrive);
  }

}
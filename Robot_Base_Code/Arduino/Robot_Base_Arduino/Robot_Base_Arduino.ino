#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#include <SPI.h>
#include <Wire.h>

#define CAN_BAUDRATE 500000
#define CAN0_INT 2    // Set INT to pin 2 (This is the Interrupt pin)
#define ODRV0_NODE_ID 0 // Left Motor
#define ODRV1_NODE_ID 1 // Right Motor

MCP2515Class& can_intf = CAN;

#define MCP2515_CS 10
#define MCP2515_INT 2
#define MCP2515_CLK_HZ 8000000
#define EStop 0 // Hardware EStop
#define IMU_SDA 2 // On AtMega32U4 Connected to Pin 19 (PD1)
#define IMU_SCL 3 // On AtMega32U4 Connected to Pin 18 (PD0) // CHECK

/*
// LED Declarations
#define LEDPWR_R 4 // On AtMega32U4 Connected to Pin 25 (PD4)
#define LEDPWR_G 12 // On AtMega32U4 Connected to Pin 26 (PD6)
#define LEDPWR_B 6 // On AtMega32U4 Connected to Pin 27 (PD7)
*/

// FIX NAME TO ButtonSignal or something like that
// Robot State Declarations
#define EStopButtonIndicator 4 // On AtMega32U4 Connected to Pin 25 (PD4)
#define AutonButtonIndicator 5 // On AtMega32U4 Connected to Pin 26 (PD6)

// Battery Voltage Detection Declaration
#define Battery_Voltage A0 // On AtMega32U4 Connected to Pin 27 (PF0)

int verticalMov = 0;
int horizontalMov = 0;
bool EStopButton = false;
bool EStopState = false;
bool AutonButton = false;
bool AutonState = false;

static inline void receiveCallback(int packet_size) 
{
  if (packet_size > 8) 
  {
    return; // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

bool setupCan() 
{
  // configure and initialize the CAN bus interface
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE))
  {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0, &odrv1}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData 
{
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

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

void setup() 
{
  Serial.begin(115200);

  Wire.begin(8);

  pinMode(EStop, INPUT);

  pinMode(CAN0_INT, INPUT);

  pinMode(IMU_SCL, INPUT); // Work on later
  pinMode(IMU_SDA, INPUT); // Work on later

  pinMode(EStopButtonIndicator, OUTPUT);
  pinMode(AutonButtonIndicator, OUTPUT);

  /*
  pinMode(LEDPWR_R, OUTPUT);
  pinMode(LEDPWR_G, OUTPUT);
  pinMode(LEDPWR_B, OUTPUT);
  */
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

  Serial.println("Enabling closed loop control...");
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
      pumpEvents(can_intf); // Pump events for 150ms (Look at example for full explaination)
    }
  }

  Serial.println("ODrive running!");
}

void loop() 
{
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  Wire.requestFrom(8, sizeof(int) + sizeof(int) + sizeof(bool) + sizeof(bool)); // request data from slave device #8 (ESP32)
  
  // Checks to see if data is recieved over I2C. If so sets values from controller to predefined variables
  if(Wire.available() >= sizeof(int) + sizeof(int) + sizeof(bool) + sizeof(bool)) // could combine with If statement below but this is easier to read for now
  {
    // Read vertical movement from left joystick
    verticalMov = Wire.read();
    verticalMov |= Wire.read() << 8; // Combine with the next byte

    // Read horizontal movement from left joystick
    horizontalMov = Wire.read();
    horizontalMov |= Wire.read() << 8; // Combine with the next byte

    // Read EStopButton from right bumper
    EStopButton = Wire.read();

    // Read Robot State from left bumper
    AutonButton = Wire.read();
    
    // Reads controller values from ESP-32
    //ControllerData();

    delay(10);
  } 
  else
  {
    verticalMov = 0;
    horizontalMov = 0;
    EStopState = false;
    AutonState = false;
    Serial.println("Insufficient bytes received");
  }

  // Make toggle instead of push to activate
  
  // Button was pressed
  if (EStopButton)
  {
    // Send EStop command to ODrive
    ODriveEStop();
    EStopState = true;
    digitalWrite(EStopButtonIndicator, HIGH);
  } 
  else 
  {
    ODriveControlState();
    ODriveMovement(verticalMov, horizontalMov);
    EStopState = false;
    digitalWrite(EStopButtonIndicator, LOW);
  }

  if (AutonButton)
  {
    AutonState = true;
    ODriveEStop();
    digitalWrite(AutonButtonIndicator, HIGH);
  } 
  else 
  {
    AutonState = false;
    digitalWrite(AutonButtonIndicator, LOW);
  }

  // Prints ODrive Velocities and Position via Encoders
  //ODriveEncoderData();
}

void ODriveMovement(double verticalVelocity, double horizontalVelocity) // Confirm if working
{
  // Scales inputs to directional velocity vectors
  verticalVelocity = constrain(verticalVelocity, -1.0, 1.0);
  horizontalVelocity = constrain(horizontalVelocity, -1.0, 1.0);


  // Standard for differential drive control
    // When one wheel moves faster the other should turn if there is horizontal movement
  double leftMotorVel = verticalVelocity - horizontalVelocity;
  double rightMotorVel = verticalVelocity + horizontalVelocity;

  // Gearbox ratio
  const double gearboxRatio = 21.0;

  // Input RPM
  const double inputRPM = 4400.0;

  // Converts the velocity requested into Turns / second
  const double maxVelocity = 5.0; //  Speed limit according to IGVC rules
  double leftMotorTPS = leftMotorVel * maxVelocity; // Turns/second
  double rightMotorTPS = rightMotorVel * maxVelocity; // Turns/second

  // Adjust for gearbox ratio and input RPM
  leftMotorTPS = (leftMotorTPS / gearboxRatio) * (inputRPM / 60.0);
  rightMotorTPS = (rightMotorTPS / gearboxRatio) * (inputRPM / 60.0);

  // Send velocity CAN commands to left and right motors
  odrv0.setVelocity(leftMotorTPS);
  odrv1.setVelocity(rightMotorTPS);
}

// Send CAN command to ODrive to initate EStop
void ODriveEStop()
{
  Serial.println("Enabling EStop...");
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
      pumpEvents(can_intf); // Pump events for 150ms (Look at example for full explaination)
    }
  }
  Serial.println("EStop Activated");
}

// Send CAN command to put axis into closed loop control state
void ODriveControlState() // Could combine with estop function but meh
{
  Serial.println("Enabling closed loop control...");
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
      pumpEvents(can_intf);  // Pump events for 150ms (Look at example for full explaination)
    }
  }
  Serial.println("Put into closed loop control state");
}

// Prints position and velocity from each ODrive (For debugging)
void ODriveEncoderData()
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
void ControllerData()
{
  Serial.print("Received from ESP32 - Vertical Movement: ");
  Serial.println(verticalMov);

  Serial.print("Received from ESP32 - Horizontal Movement: ");
  Serial.println(horizontalMov);

  Serial.print("Received from ESP32 - EStop Button: ");
  Serial.println(EStopButton);

  Serial.print("Received from ESP32 - Autonomous Button: ");
  Serial.println(AutonButton);

  Serial.println("------------------------");
}
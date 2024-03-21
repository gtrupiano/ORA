#include <Wire.h>
#include <PS4Controller.h>

void setup() 
{
  Wire.begin();
  // MAC Address for my controller
  PS4.begin("bc:03:58:28:67:42");
  Serial.begin(115200);
}

void loop() 
{
  // Fethces R1 button state as well as horizontal and vertical joystick values from controller 
  int verticalMov = PS4.LStickY();
  int horizontalMov = PS4.LStickX();
  int EStopButton = PS4.R1();

  // Pick an approch that makes the most sense to me

              // Byte array Approch
  byte controllerValues[6];
  controllerValues[0] = highByte(verticalMov);
  controllerValues[1] = lowByte(verticalMov);
  controllerValues[2] = highByte(horizontalMov);
  controllerValues[3] = lowByte(horizontalMov);
  controllerValues[4] = lowByte(EStopButton);

  Wire.beginTransmission(8); // Address of the slave device
  Wire.write(controllerValues, sizeof(controllerValues)); // Send the byte array
  byte error = Wire.endTransmission(); // Finish transmission
  
  // check to see if message was recieved
  if (error == 0) 
  {
    Serial.println("Transmission successful");
  } 
  else 
  {
    Serial.println("Error in transmission");
  }
  delay(10);
  /*
              // String Parse Approch
  // Converts integer values to an combined string so it can be sent over I2C
  String controllerValues = String(verticalMov) + "," + String(horizontalMov)  + "," + String(EStopButton);

  // Establishes a transmission line between ESP32 and Control board with the address being 8
  Wire.beginTransmission(8);

  // Writes
  Wire.write(controllerValues.c_str());
  Wire.endTransmission();
  delay(10);
  */
}

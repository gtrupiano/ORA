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
  // Fetches R1 button state as well as horizontal and vertical joystick values from controller 
  int verticalMov = PS4.LStickY();
  int horizontalMov = PS4.LStickX();
  int EStopButton = PS4.R1();
  Serial.println(verticalMov);

  // Byte array Approch
  byte controllerValues[6];
  controllerValues[0] = highByte(verticalMov);
  controllerValues[1] = lowByte(verticalMov);
  controllerValues[2] = highByte(horizontalMov);
  controllerValues[3] = lowByte(horizontalMov);
  controllerValues[4] = EStopButton;
  controllerValues[5] = 0;


  Wire.beginTransmission(8); // Address of the slave device
  Wire.write(controllerValues, sizeof(controllerValues)); // Send the byte array
  byte error = Wire.endTransmission(); // Finish transmission
  
  // Checks to see if message was recieved
  if (error == 0) 
  {
    Serial.println("Transmission successful");
  } 
  else 
  {
    Serial.println("Error in transmission");
  }
  delay(80);
}

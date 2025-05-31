#include <Wire.h>

int yPos, xPos;
bool rBump;

void setup() 
{
  Wire.begin();        // join I2C bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() 
{
  Wire.requestFrom(8, sizeof(int)); // request size of integer from slave device #8

  while (Wire.available()) 
  {
    Wire.readBytes((char*)&yPos, sizeof(yPos)); // read bytes into integer variable
    Wire.readBytes((char*)&xPos, sizeof(xPos)); // read bytes into integer variable
    Wire.readBytes((char*)&rBump, sizeof(rBump)); // read bytes into integer variable
    
    // Print joystick positions for debugging
    Serial.print("Y Position: ");
    Serial.println(yPos);
    Serial.print("X Position: ");
    Serial.println(xPos);
    Serial.print("R1 Button State: ");
    Serial.println(rBump);
  }

  delay(100);
}




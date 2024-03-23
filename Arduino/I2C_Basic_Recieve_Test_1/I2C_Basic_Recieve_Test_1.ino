#include <Wire.h>

double verticalMov = 0;
double horizontalMov = 0;
bool EStopButton = false;
bool EStopState = false;

void setup() 
{
  Wire.begin(9);  // Initialize I2C communication with address 8
  Serial.begin(115200);
}

void loop() 
{
  // Request 6 bytes from the master device (sender)

  verticalMov = (Wire.read() << 8) | Wire.read();       // Combine two bytes into an integer (vertical movement)
  horizontalMov = (Wire.read() << 8) | Wire.read();    // Combine two bytes into an integer (horizontal movement)
  EStopButton = Wire.read();                           // Read the third byte (EStop button state)

  // Print received values
  Serial.print("Received from ESP32 - Vertical Movement: ");
  Serial.println(verticalMov);

  Serial.print("Received from ESP32 - Horizontal Movement: ");
  Serial.println(horizontalMov);

  Serial.print("Received from ESP32 - EStop Button: ");
  Serial.println(EStopButton);
} 



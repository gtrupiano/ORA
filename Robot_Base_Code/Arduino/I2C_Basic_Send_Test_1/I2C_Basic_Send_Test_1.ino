#include <Wire.h>
#include <PS4Controller.h>

int yPos, xPos; // Variables to store joystick positions
bool rBump;

void setup() 
{
  PS4.begin("bc:03:58:28:67:42"); // Initialize PS4 controller with MAC address
  Serial.begin(115200);
  Wire.begin(8); // Join I2C bus with address #8
  Wire.onRequest(requestEvent); // Register request event
}

void loop() 
{
  // Read joystick positions
  yPos = PS4.LStickY();
  xPos = PS4.LStickX();
  rBump = PS4.R1();

  // Print joystick positions for debugging
  Serial.print("Y Position: ");
  Serial.println(yPos);
  Serial.print("X Position: ");
  Serial.println(xPos);
  Serial.print("R1 Button State: ");
  Serial.println(rBump);
  Serial.println();

  delay(100); // Delay for stability
}

// Function that executes whenever data is requested by master
// This function is registered as an event, see setup()
void requestEvent() 
{
  // Convert the integer joystick positions and boolean button state to byte arrays
  byte yArray[sizeof(yPos)];
  byte xArray[sizeof(xPos)];
  byte rBumpArray[sizeof(rBump)];
  memcpy(yArray, &yPos, sizeof(yPos));
  memcpy(xArray, &xPos, sizeof(xPos));
  memcpy(rBumpArray, &rBump, sizeof(rBump));

  // Send the byte arrays over I2C
  Wire.write(yArray, sizeof(yArray));
  Wire.write(xArray, sizeof(xArray));
  Wire.write(rBumpArray, sizeof(rBumpArray));
}
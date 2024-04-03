#include <Wire.h>
#include <PS4Controller.h>

int dataToSend = PS4.LStickY(); // Initialize the integer variable to send

void setup()
{
  PS4.begin("bc:03:58:28:67:42");
  Serial.begin(115200);
  Wire.begin(8);                // join I2C bus with address #8
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
  dataToSend = PS4.LStickY();
  // Increment the value to send;
  Serial.println(dataToSend);
  delay(100); // Delay for stability
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() 
{
  // Convert the integer to a byte array
  byte byteArray[sizeof(dataToSend)];
  memcpy(byteArray, &dataToSend, sizeof(dataToSend));
  
  // Send the byte array
  Wire.write(byteArray, sizeof(byteArray));
}



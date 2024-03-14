#include <PS4Controller.h>

void setup() 
{
  Serial.begin(230400);
  PS4.begin("bc:03:58:28:67:42");
  Serial.println("Ready.");
}

void loop()
{
  // Below has all accessible outputs from the controller
  if (PS4.isConnected()) 
  {
    Serial.print("Left Stick x at %d\n");
    Serial.println(PS4.LStickY());
    Serial.print("Right Stick x at %d\n");
    Serial.println(PS4.RStickY());
    Serial.println();
    delay(64);
  }
}

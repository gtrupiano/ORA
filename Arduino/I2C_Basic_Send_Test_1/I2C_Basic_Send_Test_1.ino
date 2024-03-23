#include <Wire.h>
#include <PS4Controller.h>

#define ARDUINO_I2C_ADDRESS 9

void setup() {
  Wire.begin();
  PS4.begin("bc:03:58:28:67:42");
  Serial.begin(115200);
}

void loop() {
  int verticalMov = PS4.LStickY();
  int horizontalMov = PS4.LStickX();
  
  byte controllerValues[2];
  controllerValues[0] = map(verticalMov, -127, 127, 0, 255); // Map to 8-bit range
  controllerValues[1] = map(horizontalMov, -127, 127, 0, 255); // Map to 8-bit range

  Wire.beginTransmission(ARDUINO_I2C_ADDRESS);
  Wire.write(controllerValues, sizeof(controllerValues));
  byte error = Wire.endTransmission(9);

  if (error == 0) {
    Serial.println("Transmission successful");
  } else {
    Serial.println("Error in transmission");
  }
  
  delay(80);
}


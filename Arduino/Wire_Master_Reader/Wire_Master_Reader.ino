#include <Wire.h>

void setup() {
  Wire.begin();        // join I2C bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {
  Wire.requestFrom(8, sizeof(int)); // request size of integer from slave device #8

  while (Wire.available()) {
    int receivedInt = 0;
    Wire.readBytes((char*)&receivedInt, sizeof(receivedInt)); // read bytes into integer variable

    Serial.println(receivedInt); // print the received integer
  }

  delay(100);
}




#include <Wire.h>

int yPos, xPos;
bool rBump, lBump;

void setup() {
  Wire.begin(8);        // join I2C bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.print(sizeof(int));
}

void loop() {
  // Request data from slave device #8
  Wire.requestFrom(8, sizeof(int) * 2 + sizeof(bool)*2);

  // Make sure there's enough data available
  if (Wire.available() >= sizeof(int) * 2 + sizeof(bool)*2) {
    // Read yPos
    yPos = Wire.read();
    yPos |= Wire.read() << 8; // Combine with the next byte

    // Read xPos
    xPos = Wire.read();
    xPos |= Wire.read() << 8; // Combine with the next byte

    // Read rBump
    rBump = Wire.read();

    // Read lBump
    lBump = Wire.read();

    // Print joystick positions for debugging
    Serial.print("Y Position: ");
    Serial.println(yPos);
    Serial.print("X Position: ");
    Serial.println(xPos);
    Serial.print("R1 Button State: ");
    Serial.println(rBump);
    Serial.print("L1 Button State: ");
    Serial.println(lBump);
    Serial.println();
  }

  delay(100);
}

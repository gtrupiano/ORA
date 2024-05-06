#include <ps5Controller.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const char macAddress[] = "d0:bc:c1:fc:a1:84";

const int deadband = 5;

const int motorFL = 2;
const int motorFR = 14;
const int motorBL = 19;
const int motorBR = 32;

const int dirFL = 4;
const int dirFR = 12;
const int dirBL = 21;
const int dirBR = 33;

const int buzzer = 18;
int buzzerVal = 0;

int speedY = 0, speedX = 0, rotation = 0;

void setup() 
{
  Serial.begin(921600);
  ps5.begin(macAddress);

  pinMode(dirFL, OUTPUT);
  pinMode(dirFR, OUTPUT);
  pinMode(dirBL, OUTPUT);
  pinMode(dirBR, OUTPUT);

  pinMode(motorFL, OUTPUT);
  pinMode(motorFR, OUTPUT);
  pinMode(motorBL, OUTPUT);
  pinMode(motorBR, OUTPUT);

  pinMode(buzzer, OUTPUT);

  Serial.println("Ready.");
}

void loop() 
{
  while (ps5.isConnected() == true) 
  {
    int joystickX = ps5.LStickX();
    int joystickY = ps5.LStickY();
    int rotatestick = ps5.RStickX();
    int buzzerVal = ps5.R2();

    Serial.println(joystickX);
    Serial.println(joystickY);
    Serial.println(rotatestick);
    Serial.println("---");


    speedX = (abs(joystickX) > deadband) ? map(joystickX, -128, 127, -255, 255) : 0;
    speedY = (abs(joystickY) > deadband) ? map(joystickY, -128, 127, -255, 255) : 0;
    rotation = (abs(rotatestick) > deadband) ? map(rotatestick, -128, 127, -255, 255) : 0;
    
    tone(buzzer, buzzerVal ? 1020 : 0);

    mecanumDrive(speedX, speedY, rotation);
    delay(10);
  }
}

void setMotorSpeedAndDirection(int motorPin, int dirPin, int speed, int direction) 
{
  speed = constrain(speed, 0, 255);
  digitalWrite(dirPin, direction);
  analogWrite(motorPin, speed);
}

void mecanumDrive(int x, int y, int rotation) 
{
  int FL_speed = y + x - rotation;
  int FR_speed = y - x - rotation;
  int BL_speed = y - x + rotation;
  int BR_speed = y + x - rotation;

  int maxSpeed = max(max(abs(FL_speed), abs(FR_speed)), max(abs(BL_speed), abs(BR_speed)));

  if (maxSpeed > 255) 
  {
    FL_speed = map(FL_speed, -maxSpeed, maxSpeed, -255, 255);
    FR_speed = map(FR_speed, -maxSpeed, maxSpeed, -255, 255);
    BL_speed = map(BL_speed, -maxSpeed, maxSpeed, -255, 255);
    BR_speed = map(BR_speed, -maxSpeed, maxSpeed, -255, 255);
  }

  setMotorSpeedAndDirection(motorFL, dirFL, abs(FL_speed), FL_speed >= 0 ? HIGH : LOW);
  setMotorSpeedAndDirection(motorFR, dirFR, abs(FR_speed), FR_speed >= 0 ? HIGH : LOW);
  setMotorSpeedAndDirection(motorBL, dirBL, abs(BL_speed), BL_speed >= 0 ? HIGH : LOW);
  setMotorSpeedAndDirection(motorBR, dirBR, abs(BR_speed), BR_speed >= 0 ? HIGH : LOW);
}
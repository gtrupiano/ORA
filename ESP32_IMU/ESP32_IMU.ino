#include "MPU9250.h"
#include <Wire.h>

#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

typedef struct
{
  float x;
  float y;
  float z;
} Vector_t;

typedef struct
{
  float x;
  float y;
  float z;
  float w;
} Quaternion_t;

typedef struct
{
  Vector_t acceleration;
  Vector_t gyro;
  Quaternion_t quat;
  float temperature;
  unsigned long sampleTime;
} IMU_t;

IMU_t imu = {};
MPU9250 mpu;

// Function Defines
void updateImuObject();
void printImuData();


void setup() 
{
  Serial.begin(115200);
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

  if(!mpu.setup(0x68)) 
  {
    while (1) 
    {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  Serial.println("MPU9250 ready!");
}

void loop() 
{
  // Fetching data from IMU and updating IMU struct
  updateImuObject();

  // Printing acceleration data
  printImuData();

  delay(100);
}

void updateImuObject()
{
  if (mpu.update())
  {

    // Copying data to IMU structure
    imu.acceleration.x = mpu.getAccX();
    imu.acceleration.y = mpu.getAccY();
    imu.acceleration.z = mpu.getAccZ();

    imu.gyro.x = mpu.getGyroX();
    imu.gyro.y = mpu.getGyroY();
    imu.gyro.z = mpu.getGyroZ();

    imu.quat.w = mpu.getQuaternionW();
    imu.quat.x = mpu.getQuaternionX();
    imu.quat.y = mpu.getQuaternionY();
    imu.quat.z = mpu.getQuaternionZ();

    imu.temperature = mpu.getTemperature();

    imu.sampleTime = millis();
  }
}

void printImuData()
{
  // Printing acceleration data
  Serial.print("[");
  Serial.print(imu.sampleTime);
  Serial.print("] X: ");
  Serial.print(imu.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(imu.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(imu.acceleration.z);
  Serial.println(" m/s^2");
  Serial.println();

  // Printing gyroscopic data
  Serial.print("[");
  Serial.print(imu.sampleTime);
  Serial.print("] X: ");
  Serial.print(imu.gyro.x);
  Serial.print(", Y: ");
  Serial.print(imu.gyro.y);
  Serial.print(", Z: ");
  Serial.print(imu.gyro.z);
  Serial.println(" rad/s");
  Serial.println();

  // Printing quaternion data
  Serial.print("[");
  Serial.print(imu.sampleTime);
  Serial.print("] quat (w,x,y,z): ");
  Serial.print(imu.quat.w, 6); Serial.print(", ");
  Serial.print(imu.quat.x, 6); Serial.print(", ");
  Serial.print(imu.quat.y, 6); Serial.print(", ");
  Serial.println(imu.quat.z, 6);
  Serial.println();

  // Printing temperature data
  Serial.print("[");
  Serial.print(imu.sampleTime);
  Serial.print("] Temperature: ");
  Serial.print(imu.temperature);
  Serial.println(" c");
  Serial.println();

  Serial.println("--------------------------");
  Serial.println();
}
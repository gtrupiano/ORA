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
bool updateImuObject();
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

  mpu.selectFilter(QuatFilterSel::MADGWICK);

  // Calibration sequence
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();

  mpu.verbose(false);
}

void loop() 
{
  // Fetching data from IMU and updating IMU struct
  bool imuDataPresent = updateImuObject();

  // Printing acceleration data only if imu data is present
  if(imuDataPresent)
  {
    printImuData();
  }

  // Quaternions are computed by the fusion filter and only update when mpu.update() runs frequently. 
  // Keep loop fast; avoid long delays or quat may freeze at (1,0,0,0). 
  delay(1);
}

bool updateImuObject()
{
  bool imuDataPresent = false;
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

    imuDataPresent = true;
  }

  return imuDataPresent;
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
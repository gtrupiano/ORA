#include "MPU9250.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

#include <stdio.h>
#include <Wire.h>

#define ERROR_LED_PIN 13

#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

// This is the I2C address of the IMU chip itself.
// May change with chip variant so run the example program connection_check.ino
#define IMU_I2C_ADDRESS 0x68

// Checks return value of RCLC function. If the function failed, then either send to error state
// or (in soft checks case) ignore.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("Soft Failed");}}

// Typedefs
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
  unsigned long sampleTimeMS;
} IMU_t;


// ROS expects: accel = m/s^2, gyro = rad/s
static const float G_TO_MPS2 = 9.80665f;

// Timer Interrupt timeout
const unsigned int TIMER_TIMEOUT_MS = 20;


// Global Variables

// ROS Client Library (RCL) Variables
// C API's for implementing ROS2 objects
rcl_publisher_t imuPublisher;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ROS Client Library for C (RCLC)
// Helper functions for easier use of RCL
rclc_executor_t executor;
rclc_support_t support;

// ROS message object for IMU
sensor_msgs__msg__Imu ImuMsg;


// IMU struct and object initialization
IMU_t imu = {};
MPU9250 mpu;

// Function Definitions
void initIMU();
void initMicroRos();
void errorLoop();
void timerCallback(rcl_timer_t * timer, int64_t last_call_time);
void updateImuObject();
void printImuData();


void setup() 
{
  Serial.begin(115200);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, HIGH);
  initIMU();
  initMicroRos();
}


void loop() 
{
  // Runs callback (timer publishes data)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  delay(1);
}


void initIMU()
{
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


void initMicroRos()
{
  // Configures how ESP talks to micro ROS agent (serial, wifi, etc)
  set_microros_transports();
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &imuPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),"imu_publisher"));

  // Create timer
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
    timerCallback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  static char IMU_FRAME[] = "imu_frame";
  ImuMsg.header.frame_id.data = IMU_FRAME;
  ImuMsg.header.frame_id.size = strlen(IMU_FRAME);
  ImuMsg.header.frame_id.capacity = ImuMsg.header.frame_id.size + 1;
}


void errorLoop()
{
  while(1)
  {
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
    delay(100);
  }
}


void timerCallback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) 
  {
    updateImuObject();
    fillImuMsgFromImuStruct();
    RCSOFTCHECK(rcl_publish(&imuPublisher, &ImuMsg, NULL));
  }
}


void updateImuObject()
{
  if (mpu.update())
  {
    // Copying data to IMU structure

    // Coverting to SI units for ROS
    imu.acceleration.x = mpu.getAccX() * G_TO_MPS2;
    imu.acceleration.y = mpu.getAccY() * G_TO_MPS2;
    imu.acceleration.z = mpu.getAccZ() * G_TO_MPS2;

    imu.gyro.x = mpu.getGyroX() * DEG_TO_RAD;
    imu.gyro.y = mpu.getGyroY() * DEG_TO_RAD;
    imu.gyro.z = mpu.getGyroZ() * DEG_TO_RAD;

    imu.quat.w = mpu.getQuaternionW();
    imu.quat.x = mpu.getQuaternionX();
    imu.quat.y = mpu.getQuaternionY();
    imu.quat.z = mpu.getQuaternionZ();

    imu.temperature = mpu.getTemperature();

    imu.sampleTimeMS = millis();
  }
}


void fillImuMsgFromImuStruct()
{
  // Header
  //
  ImuMsg.header.stamp.sec = (int32_t)(imu.sampleTimeMS / 1000);
  ImuMsg.header.stamp.nanosec = (uint32_t)((imu.sampleTimeMS  % 1000) * 1000000);

  // Linear acceleration (m/s^2)
  ImuMsg.linear_acceleration.x = imu.acceleration.x;
  ImuMsg.linear_acceleration.y = imu.acceleration.y;
  ImuMsg.linear_acceleration.z = imu.acceleration.z;

  // Angular velocity (rad/s)
  ImuMsg.angular_velocity.x = imu.gyro.x;
  ImuMsg.angular_velocity.y = imu.gyro.y;
  ImuMsg.angular_velocity.z = imu.gyro.z;

  // Orientation (only if valid)
  ImuMsg.orientation.w = imu.quat.w;
  ImuMsg.orientation.x = imu.quat.x;
  ImuMsg.orientation.y = imu.quat.y;
  ImuMsg.orientation.z = imu.quat.z;
}


void printImuData()
{
  // Printing acceleration data
  Serial.print("[");
  Serial.print(imu.sampleTimeMS);
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
  Serial.print(imu.sampleTimeMS);
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
  Serial.print(imu.sampleTimeMS);
  Serial.print("] quat (w,x,y,z): ");
  Serial.print(imu.quat.w, 6); Serial.print(", ");
  Serial.print(imu.quat.x, 6); Serial.print(", ");
  Serial.print(imu.quat.y, 6); Serial.print(", ");
  Serial.println(imu.quat.z, 6);
  Serial.println();

  // Printing temperature data
  Serial.print("[");
  Serial.print(imu.sampleTimeMS);
  Serial.print("] Temperature: ");
  Serial.print(imu.temperature);
  Serial.println(" c");
  Serial.println();

  Serial.println("--------------------------");
  Serial.println();
}
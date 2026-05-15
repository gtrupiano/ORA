/*
 ******************************************************************************
      File Name  : ESP32_IMU.hpp
      Author     : George Trupiano
      Date       : 
      Description:
 ******************************************************************************
 */
  
#ifndef ESP32_IMU_H
#define ESP32_IMU_H
  
/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

/*
 ******************************************************************************
 * DEFINES, CONSTANTS, ENUMS, STRUCTS
 ******************************************************************************
 */

#define LED_PIN 2
#define IMU_RST_PIN 13
#define IMU_INT_PIN 14
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

#define IMU_I2C_ADDRESS 0x4B // Alternative address is 0x4A

#define IMU_CONNECT_RETRY_NUM 8

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
    Vector_t accel;
    Vector_t gyro;
    Quaternion_t quat;
    float temp;
    unsigned long sampleTimeMS;
} IMU_t;


typedef struct 
{
    // Time is in MS
    unsigned long period;
    unsigned long lastTime;
} Task_t;

typedef struct
{
    Task_t printImu;
    Task_t led;
} TaskTime_t;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************
 */

bool configureIMU();
void imuOutputDataConfig();
void updateImuObject();
void printImuData(bool accel, bool gyro, bool quat);
  
#endif // ESP32_IMU_H
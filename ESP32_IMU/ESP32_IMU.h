/*
 ******************************************************************************
      File Name  : ESP32_IMU.h
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

#include "MPU9250.h"
#include <Wire.h>

/*
 ******************************************************************************
 * DEFINES, CONSTANTS, ENUMS, STRUCTS
 ******************************************************************************
 */

#define LED_PIN 13
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

#define MPU_ADDRESS 0x68

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
    unsigned long sampleTime;
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

bool updateImuObject();
void printImuData(bool accel, bool gyro, bool quat, bool temp);
  
#endif // ESP32_IMU_H
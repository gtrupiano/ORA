/****************************************************************
File Name  : ORA_IMU.hpp
Description:
****************************************************************/

#ifndef ORA_IMU_H
#define ORA_IMU_H
  
/****************************************************************
*   INCLUDES
****************************************************************/

#include <Arduino.h>
#include <cstring>
#include <stdint.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

/****************************************************************
*   DEFINES, CONSTANTS, ENUMS, STRUCTS
****************************************************************/

/*******************************
*   Structures
*******************************/

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

/****************************************************************
*   GLOBAL VARIABLES
****************************************************************/

/****************************************************************
*   GLOBAL FUNCTION PROTOTYPES
****************************************************************/

bool ORA_IMU_Init(void);
void ORA_IMU_Loop(void);
Vector_t ORA_IMU_GetAccelerationData(void);
Vector_t ORA_IMU_GetGyroscopicData(void);
Quaternion_t ORA_IMU_GetQuaternionData(void);
  
#endif // ORA_IMU_H
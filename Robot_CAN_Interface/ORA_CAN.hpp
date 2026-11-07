/****************************************************************
File Name  : ORA_CAN.hpp
Description:
****************************************************************/

#ifndef ORA_CAN_HPP
#define ORA_CAN_HPP

/****************************************************************
*   INCLUDES
****************************************************************/

#include "CAN_Driver.hpp"

/****************************************************************
*   DEFINES, CONSTANTS, ENUMS, STRUCTS
****************************************************************/

/*******************************
*   Structures
*******************************/

// CAN message structures
typedef struct
{
    float targetWheelSpeedLeft;
    float targetWheelSpeedRight;
} TargetWheelSpeedsMsg_t;

typedef struct
{
    float actualWheelSpeedLeft;
    float actualWheelSpeedRight;
} ActualWheelSpeedsMsg_t;

typedef struct
{
    int16_t accelerationX;
    int16_t accelerationY;
    int16_t accelerationZ;
    int8_t padding[2]; 
} IMUAccelerationDataMsg_t;

typedef struct
{
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int8_t padding[2]; 
} IMUGyroscopicDataMsg_t;

typedef struct
{
    int16_t quaternion_X;
    int16_t quaternion_Y;
    int16_t quaternion_Z;
    int8_t padding[2]; 
} IMUQuaternionDataMsg_t;

typedef struct
{
    int32_t encoderTicksLeft;
    int32_t encoderTicksRight;
} EncoderDataMsg_t;

typedef struct
{
    uint8_t autonomousState;
    uint8_t estopState;
    uint8_t padding[6];
} RobotStateMicroMsg_t;

typedef struct
{
    uint8_t autonomousState;
    uint8_t estopState;
    uint8_t padding[6];
} RobotStateROSMsg_t;

typedef union
{
    TargetWheelSpeedsMsg_t wheelSpeeds;
    ActualWheelSpeedsMsg_t actualWheelSpeeds;
    IMUAccelerationDataMsg_t imuAcceleration;
    IMUGyroscopicDataMsg_t imuGyroscopic;
    IMUQuaternionDataMsg_t imuQuaternion;
    EncoderDataMsg_t encoder;
    RobotStateMicroMsg_t robotStateMicro;
    RobotStateROSMsg_t robotStateROS;
} CanMsgs;


/****************************************************************
*   GLOBAL VARIABLES
****************************************************************/

/****************************************************************
*   GLOBAL FUNCTION PROTOTYPES
****************************************************************/

void ORA_CAN_Init(void);
void ORA_CAN_Loop(void);

#endif // ORA_CAN_HPP
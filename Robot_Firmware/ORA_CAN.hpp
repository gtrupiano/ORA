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
*   Defines
*******************************/

#define MAX_CAN_RX_PER_LOOP 8

// TX CAN IDs
#define ACTUAL_WHEEL_SPEED_CAN_ID 0X002
#define IMU_ACCELERATION_DATA_XY_CAN_ID 0x010
#define IMU_ACCELERATION_DATA_Z_CAN_ID 0x011
#define IMU_GYROSCOPIC_DATA_XY_CAN_ID 0x012
#define IMU_GYROSCOPIC_DATA_Z_CAN_ID 0x013
#define IMU_QUATERNION_DATA_WX_CAN_ID 0x014
#define IMU_QUATERNION_DATA_YZ_CAN_ID 0x015
#define ENCODER_DATA_CAN_ID 0x020
#define ROBOT_STATE_ROS 0x031

// RX CAN IDs
#define TARGET_WHEEL_SPEED_CAN_ID 0x001
#define ROBOT_STATE_MICRO 0x030

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
    float accelerationX;
    float accelerationY;
} IMUAccelerationDataXYMsg_t;

typedef struct
{
    float accelerationZ;
    uint8_t padding[4]; 
} IMUAccelerationDataZMsg_t;

typedef struct
{
    float gyroX;
    float gyroY;
} IMUGyroscopicDataXYMsg_t;

typedef struct
{
    float gyroZ;
    uint8_t padding[4]; 
} IMUGyroscopicDataZMsg_t;

typedef struct
{
    float quaternionW;
    float quaternionX;
} IMUQuaternionDataWXMsg_t;

typedef struct
{
    float quaternionY;
    float quaternionZ;
} IMUQuaternionDataYZMsg_t;

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
    TargetWheelSpeedsMsg_t targetWheelSpeeds;
    ActualWheelSpeedsMsg_t actualWheelSpeeds;
    IMUAccelerationDataXYMsg_t imuAccelerationXY;
    IMUAccelerationDataZMsg_t imuAccelerationZ;
    IMUGyroscopicDataXYMsg_t imuGyroscopicXY;
    IMUGyroscopicDataZMsg_t imuGyroscopicZ;
    IMUQuaternionDataWXMsg_t imuQuaternionWX;
    IMUQuaternionDataYZMsg_t imuQuaternionYZ;
    EncoderDataMsg_t encoderTicks;
    RobotStateMicroMsg_t robotStateMicro;
    RobotStateROSMsg_t robotStateROS;
    uint8_t rawData[MAX_BYTES_OF_DATA_IN_MSG];
} CAN_Data_t;


/****************************************************************
*   GLOBAL VARIABLES
****************************************************************/

/****************************************************************
*   GLOBAL FUNCTION PROTOTYPES
****************************************************************/

bool ORA_CAN_Init(void);
void ORA_CAN_Loop(void);

#endif // ORA_CAN_HPP
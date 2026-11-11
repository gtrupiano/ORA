/****************************************************************
*   File Name  : ORA_CAN.cpp
*   Description:
****************************************************************/

/****************************************************************
*   INCLUDES
****************************************************************/

#include "ORA_CAN.hpp"
#include "ORA_IMU.hpp"

/****************************************************************
*   DEFINES, CONSTANTS
****************************************************************/

/****************************************************************
*   GLOBAL VARIABLES
****************************************************************/

/****************************************************************
*   LOCAL VARIABLES (declare as static)
****************************************************************/

/****************************************************************
*   LOCAL FUNCTION PROTOTYPES (declare as static)
****************************************************************/

// CAN Message Handler Functions

// TX
void canTxMsgs(void);
void txActualSpeed(float leftWheelSpeed, float rightWheelSpeed);
void txImuAccelerationMsgs(Vector_t currentAccel);
void txImuAccelerationDataXY(Vector_t accel);
void txImuAccelerationDataZ(Vector_t accel);
void txImuGyroscopicMsgs(Vector_t currentGyro);
void txImuGyroscopicDataXY(Vector_t gyro);
void txImuGyroscopicDataZ(Vector_t gyro);
void txImuQuaternionMsgs(Quaternion_t currentQuat);
void txImuQuaternionDataWX(Quaternion_t quat);
void txImuQuaternionDataYZ(Quaternion_t quat);
void txEncoderData(int32_t leftWheelEncoderTicks, int32_t rightWheelEncoderTicks);
void txRobotState(uint8_t autonState, uint8_t estopState);

// RX
void canRxMsgs(void);
void rxTargetSpeed(CAN_Msg_t *msg);
void rxRobotState(CAN_Msg_t *msg);

/****************************************************************
*   GLOBAL FUNCTIONS
****************************************************************/

/**************************************************
*   Function: ORA_CAN_Init
*   Description:
**************************************************/

bool ORA_CAN_Init(void)
{
    // Initialize CAN driver (MCP2515 chip)
    bool initStatus = CAN_Driver_Init();

    // Return true or false depending on status of driver initialization
    if(initStatus == false)
    {
        Serial.println("ORA_CAN initialization not valid. Reset");
        return false;
    }
    else
    {
        return true;
    }
}


/**************************************************
*   Function: ORA_CAN_Loop
*   Description:
**************************************************/

void ORA_CAN_Loop(void)
{
    // Process both TX and RX messages
    canTxMsgs();
    canRxMsgs();
}


/****************************************************************
*   LOCAL FUNCTIONS
****************************************************************/

/**************************************************
*   Function: canTxMsgs
*   Description: 
**************************************************/

void canTxMsgs(void)
{
    // Transmit all TX CAN messages with the proper API data from other ORA_ files
    // NOTE: Some interfaces aren't implemented (commented ones) but I wrote what they might look like just to give examples
    //txActualSpeed(ORA_Control_Get_Left_Wheel_Speed(), ORA_Control_Get_Right_Wheel_Speed());
    txImuAccelerationMsgs(ORA_IMU_GetAccelerationData());
    txImuGyroscopicMsgs(ORA_IMU_GetGyroscopicData());
    txImuQuaternionMsgs(ORA_IMU_GetQuaternionData());
    //txEncoderData(ORA_Control_Get_Left_Wheel_Encoder_Ticks(), ORA_Control_Get_Right_Wheel_Encoder_Ticks());
    //txRobotState(ORA_Control_Get_Auton_State(), ORA_Control_Get_EStop_State()); // Interface not written yet
}


/**************************************************
*   Function: txImuAccelerationMsgs
*   Description: 
**************************************************/

void txImuAccelerationMsgs(Vector_t currentAccel)
{
    // Handles sending out the 2 part message for IMU acceleration data
    txImuAccelerationDataXY(currentAccel);
    txImuAccelerationDataZ(currentAccel);
}


/**************************************************
*   Function: txImuGyroscopicMsgs
*   Description: 
**************************************************/

void txImuGyroscopicMsgs(Vector_t currentGyro)
{
    // Handles sending out the 2 part message for IMU gyroscopic data
    txImuGyroscopicDataXY(currentGyro);
    txImuGyroscopicDataZ(currentGyro);
}


/**************************************************
*   Function: txImuQuaternionMsgs
*   Description: 
**************************************************/

void txImuQuaternionMsgs(Quaternion_t currentQuat)
{
    // Handles sending out the 2 part message for IMU quaternion data
    txImuQuaternionDataWX(currentQuat);
    txImuQuaternionDataYZ(currentQuat);
}


/**************************************************
*   Function: canRxMsgs
*   Description: 
**************************************************/

void canRxMsgs(void)
{
    // Creating structure to hold incoming CAN message
    CAN_Msg_t rxMsg = {};

    // Used to exit loop if enough messages were received in a single loop
    uint8_t rxCount = 0;

    // Fetching CAN data and validity from driver while data is present or until N messages were received
    while(CAN_Driver_ReceiveMsg(&rxMsg) && rxCount < MAX_CAN_RX_PER_LOOP)
    {
        // Incrementing based on how many messages were received
        rxCount++;

        // Depending on the message ID in the received CAN message.
        switch(rxMsg.msgId)
        {
            case TARGET_WHEEL_SPEED_CAN_ID:
                rxTargetSpeed(&rxMsg);
                break;
            case ROBOT_STATE_MICRO:
                rxRobotState(&rxMsg);
                break;
            default:
                break;
        }
    }
}


/****************************************************************
*   TX FUNCTIONS
****************************************************************/


/**************************************************
*   Function: txActualSpeed
*   Description: 
**************************************************/

void txActualSpeed(float leftWheelSpeed, float rightWheelSpeed)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;

    // Populating structure with data from APIs
    txMsg.actualWheelSpeeds.actualWheelSpeedLeft = leftWheelSpeed;
    txMsg.actualWheelSpeeds.actualWheelSpeedRight = rightWheelSpeed;

    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(ACTUAL_WHEEL_SPEED_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txImuAccelerationDataXY
*   Description: 
**************************************************/

void txImuAccelerationDataXY(Vector_t accel)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.imuAccelerationXY.accelerationX = accel.x;
    txMsg.imuAccelerationXY.accelerationY = accel.y;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(IMU_ACCELERATION_DATA_XY_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txImuAccelerationDataZ
*   Description: 
**************************************************/

void txImuAccelerationDataZ(Vector_t accel)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.imuAccelerationZ.accelerationZ = accel.z;
    txMsg.imuAccelerationZ.padding[0] = 0;
    txMsg.imuAccelerationZ.padding[1] = 0;
    txMsg.imuAccelerationZ.padding[2] = 0;
    txMsg.imuAccelerationZ.padding[3] = 0;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(IMU_ACCELERATION_DATA_Z_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txImuGyroscopicDataXY
*   Description: 
**************************************************/

void txImuGyroscopicDataXY(Vector_t gyro)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.imuGyroscopicXY.gyroX = gyro.x;
    txMsg.imuGyroscopicXY.gyroY = gyro.y;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(IMU_GYROSCOPIC_DATA_XY_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txImuGyroscopicDataZ
*   Description: 
**************************************************/

void txImuGyroscopicDataZ(Vector_t gyro)
{    
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.imuGyroscopicZ.gyroZ = gyro.z;
    txMsg.imuGyroscopicZ.padding[0] = 0;
    txMsg.imuGyroscopicZ.padding[1] = 0;
    txMsg.imuGyroscopicZ.padding[2] = 0;
    txMsg.imuGyroscopicZ.padding[3] = 0;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(IMU_GYROSCOPIC_DATA_Z_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txImuQuaternionDataWX
*   Description: 
**************************************************/

void txImuQuaternionDataWX(Quaternion_t quat)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.imuQuaternionWX.quaternionW = quat.w;
    txMsg.imuQuaternionWX.quaternionX = quat.x;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(IMU_QUATERNION_DATA_WX_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txImuQuaternionDataYZ
*   Description: 
**************************************************/

void txImuQuaternionDataYZ(Quaternion_t quat)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.imuQuaternionYZ.quaternionY = quat.y;
    txMsg.imuQuaternionYZ.quaternionZ = quat.z;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(IMU_QUATERNION_DATA_YZ_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txEncoderData
*   Description: 
**************************************************/

void txEncoderData(int32_t leftWheelEncoderTicks, int32_t rightWheelEncoderTicks)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.encoderTicks.encoderTicksLeft = leftWheelEncoderTicks;
    txMsg.encoderTicks.encoderTicksRight = rightWheelEncoderTicks;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(ENCODER_DATA_CAN_ID, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/**************************************************
*   Function: txRobotState
*   Description: 
**************************************************/

void txRobotState(uint8_t autonState, uint8_t estopState)
{
    // Creating structure to hold data outgoing CAN message
    CAN_Data_t txMsg;
    
    // Populating structure with data from APIs
    txMsg.robotStateROS.autonomousState = autonState;
    txMsg.robotStateROS.estopState = estopState;
    txMsg.robotStateROS.padding[0] = 0;
    txMsg.robotStateROS.padding[1] = 0;
    txMsg.robotStateROS.padding[2] = 0;
    txMsg.robotStateROS.padding[3] = 0;
    txMsg.robotStateROS.padding[4] = 0;
    txMsg.robotStateROS.padding[5] = 0;
    
    // Transmit the populated message with its corresponding message ID
    CAN_Driver_SendMsg(ROBOT_STATE_ROS, MAX_BYTES_OF_DATA_IN_MSG, &txMsg.rawData[0]);
}


/****************************************************************
*   RX FUNCTIONS
****************************************************************/

/**************************************************
*   Function: rxTargetSpeed
*   Description: 
**************************************************/

void rxTargetSpeed(CAN_Msg_t *msg)
{
    // Creating structure to hold CAN message data representing all CAN messages
    CAN_Data_t rxMsg = {};

    // Copying just the data from the received CAN message into the data structure
    memcpy(rxMsg.rawData, msg->data, MAX_BYTES_OF_DATA_IN_MSG);

    // Using the data captured from the received message, interfacing with the proper API to drive wheels
    // NOTE: Interfaces aren't implemented (commented ones) but I wrote what they might look like just to give examples
    // ORA_Control_Set_Left_Wheel_Speed(rxMsg.targetWheelSpeeds.targetWheelSpeedLeft);
    // ORA_Control_Set_Right_Wheel_Speed(rxMsg.targetWheelSpeeds.targetWheelSpeedRight);}
}


/**************************************************
*   Function: rxRobotState
*   Description: 
**************************************************/

void rxRobotState(CAN_Msg_t *msg)
{
    // Creating structure to hold CAN message data representing all CAN messages
    CAN_Data_t rxMsg = {};

    // Copying just the data from the received CAN message into the data structure
    memcpy(rxMsg.rawData, msg->data, MAX_BYTES_OF_DATA_IN_MSG);

    // Using the data captured from the received message, interfacing with the proper API to set robot state
    // Controller function calls to set robot state (both Auton and EStop)
    // NOTE: Interfaces aren't implemented (commented ones) but I wrote what they might look like just to give examples
    // ORA_Control_Set_Auton_State(rxMsg.robotStateMicro.autonomousState);
    // ORA_Control_Set_EStop_State(rxMsg.robotStateMicro.estopState);
}
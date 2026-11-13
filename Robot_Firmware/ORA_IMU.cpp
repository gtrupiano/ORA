/****************************************************************
*   File Name  : ORA_IMU.cpp
*   Description:
****************************************************************/

/****************************************************************
*   INCLUDES
****************************************************************/

#include "ORA_IMU.hpp"
#include "flags.hpp"

/****************************************************************
*   DEFINES, CONSTANTS
****************************************************************/

/*******************************
*   Defines
*******************************/

#define LED_PIN 2
#define IMU_RST_PIN 1
#define IMU_INT_PIN 5
#define IMU_SDA_PIN 8
#define IMU_SCL_PIN 9

#define IMU_I2C_ADDRESS 0x4B // Alternative address is 0x4A

#define IMU_CONNECT_RETRY_NUM 8

/****************************************************************
*   GLOBAL VARIABLES
****************************************************************/

/****************************************************************
*   LOCAL VARIABLES (declare as static)
****************************************************************/

static IMU_t imu = {};
static BNO08x bno;

/****************************************************************
*   LOCAL FUNCTION PROTOTYPES (declare as static)
****************************************************************/

static bool configureIMU();
static void imuOutputDataConfig();
static void updateImuObject();
static void printImuData(bool accel, bool gyro, bool quat);
  
/****************************************************************
*   GLOBAL FUNCTIONS
****************************************************************/

/**************************************************
*   Function: ORA_IMU_Init
*   Description:
**************************************************/

bool ORA_IMU_Init(void)
{
    // Setting up IMU
    bool imuConfigSuccessful = configureIMU();

    delay(100);

    if(imuConfigSuccessful == false)
    {
        Serial.println("ORA_IMU_Init ran into an error");
        return false;
    }
    else
    {
        Serial.println("ORA_IMU_Init Completed!");
        return true;
    }
}


/**************************************************
*   Function: ORA_IMU_Loop
*   Description:
**************************************************/

void ORA_IMU_Loop(void)
{
    /*
        Process incoming IMU data to obtain the
        latest acceleration, gyroscope, and quaternion readings.
    */
    if (bno.wasReset()) 
    {
        Serial.print("Sensor was reset");
        imuOutputDataConfig();
    }

    // Fetching data from IMU and updating IMU struct
    updateImuObject();

    #if DEBUG
        printImuData(IMU_PRINT_ACCEL, IMU_PRINT_GYRO, IMU_PRINT_QUAT);
    #endif
}


/**************************************************
 * Function Name: ORA_IMU_GetAccelerationData
 * Description: 
**************************************************/

Vector_t ORA_IMU_GetAccelerationData(void)
{
    return imu.accel;
}


/**************************************************
 * Function Name: ORA_IMU_GetGyroscopicData
 * Description: 
**************************************************/

Vector_t ORA_IMU_GetGyroscopicData(void)
{
    return imu.gyro;
}


/**************************************************
 * Function Name: ORA_IMU_GetQuaternionData
 * Description: 
**************************************************/

Quaternion_t ORA_IMU_GetQuaternionData(void)
{
    return imu.quat;
}


/****************************************************************
*   LOCAL FUNCTIONS
****************************************************************/

/**************************************************
 * Function Name: configureIMU
 * Description: 
**************************************************/

bool configureIMU()
{
    // Configuring communication over I2C
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);   // 400 kHz I2C fast mode
    Wire.setTimeOut(50);     // prevent long I2C blocking/hanging
    
    delay(250);

    uint8_t imuConnectAttempts = 0;
    bool imuConnected = false;

    // Attempt to connect to IMU multiple times
    while(imuConnectAttempts < IMU_CONNECT_RETRY_NUM)
    {
        // Setting up IMU with proper interrupt and reset pins
        if(bno.begin(IMU_I2C_ADDRESS, Wire, IMU_INT_PIN, IMU_RST_PIN) == false) 
        {
            Serial.println("IMU connection failed. Please check connection with examples.");
        }
        else
        {
            Serial.println("IMU connection successful");
            imuConnected = true;
            break;
        }

        imuConnectAttempts++;
    }

    // Configuring what data to report
    if(imuConnected)
    {
        imuOutputDataConfig();
    }

    return imuConnected;
}


/**************************************************
 * Function Name: imuOutputDataConfig
 * Description: 
**************************************************/

void imuOutputDataConfig()
{
    // Accelerometer data
    if(bno.enableAccelerometer() == true)
    {
        Serial.println("Accelerometer enabled");
    }
    else
    {
        Serial.println("Can't enable  accelerometer");
    }

    // Gyroscopic data
    if(bno.enableGyro() == true)
    {
        Serial.println("Gyro enabled");
    }
    else
    {
        Serial.println("Can't enable  gyro");
    }

    // Rotational vector data (used for quaternions)
    if(bno.enableRotationVector() == true)
    {
        Serial.println(("Rotation vector enabled"));
    }
    else
    {
        Serial.println("Can't enable  rotation vector");
    }
}

/**************************************************
 * Function Name: updateImuObject
 * Description: 
**************************************************/

void updateImuObject()
{
    // Checking whether an event occured
    if(bno.getSensorEvent() == true)
    {
        uint8_t sensorEvent = bno.getSensorEventID();

        switch(sensorEvent)
        {
            case SENSOR_REPORTID_ACCELEROMETER:
                imu.accel.x = bno.getAccelX();
                imu.accel.y = bno.getAccelY();
                imu.accel.z = bno.getAccelZ();
            break;

            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                imu.gyro.x = bno.getGyroX();
                imu.gyro.y = bno.getGyroY();
                imu.gyro.z = bno.getGyroZ();
            break;

            case SENSOR_REPORTID_ROTATION_VECTOR:
                imu.quat.w = bno.getQuatReal();
                imu.quat.x = bno.getQuatI();
                imu.quat.y = bno.getQuatJ();
                imu.quat.z = bno.getQuatK();
                // there is a getQuatRadianAccuracy() but not sure if needed?
            break;

            default:
            break;
        }

        imu.sampleTimeMS = millis();
    }
}


/**************************************************
 * Function Name: printImuData
 * Description: 
**************************************************/

void printImuData(bool accel, bool gyro, bool quat)
    {
    // Printing acceleration data
    if(accel)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] Acceleration (x,y,z): ");
        Serial.print("X: ");
        Serial.print(imu.accel.x);
        Serial.print(", Y: ");
        Serial.print(imu.accel.y);
        Serial.print(", Z: ");
        Serial.print(imu.accel.z);
        Serial.println(" m/s^2");
        Serial.println();
    }

    // Printing gyroscopic data
    if(gyro)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] Gyroscope (x,y,z): ");
        Serial.print("X: ");
        Serial.print(imu.gyro.x);
        Serial.print(", Y: ");
        Serial.print(imu.gyro.y);
        Serial.print(", Z: ");
        Serial.print(imu.gyro.z);
        Serial.println(" rad/s");
        Serial.println();
    }

    // Printing quaternion data
    if(quat)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] Quaternion (w,x,y,z): ");
        Serial.print(imu.quat.w, 6); Serial.print(", ");
        Serial.print(imu.quat.x, 6); Serial.print(", ");
        Serial.print(imu.quat.y, 6); Serial.print(", ");
        Serial.println(imu.quat.z, 6);
        Serial.println();
    }
    
    // Ending line if any value are requested
    if(accel || gyro || quat)
    {
        Serial.println("--------------------------");
        Serial.println();
    }
}
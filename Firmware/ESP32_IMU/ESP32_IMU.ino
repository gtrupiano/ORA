/*
 **********************************************************************
 * INCLUDES
 **********************************************************************
*/

#include "ESP32_IMU.hpp"

/*
 **********************************************************************
 * DEFINES and CONSTANTS
 **********************************************************************
*/

/*
 **********************************************************************
 * GLOBAL VARIABLES
 **********************************************************************
*/

TaskTime_t tasks = 
{
    // {period, lastTime}
    .printImu = {60, 0},
    .led = {500, 0}
};

IMU_t imu = {};
BNO08x bno;

/*
 **********************************************************************
 * LOCAL TYPES
 **********************************************************************
*/

/*
 **********************************************************************
 * LOCAL VARIABLES (declare as static)
 **********************************************************************
*/

/*
 **********************************************************************
 * LOCAL FUNCTION PROTOTYPES (declare as static)
 **********************************************************************
*/
  
/*
 **********************************************************************
 * GLOBAL FUNCTIONS
 **********************************************************************
*/


void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    // Setting up IMU
    bool imuConfigSuccessful = configureIMU();

    if(imuConfigSuccessful == false)
    {
        while(1)
        {
            Serial.println("An issue occured during configuration. Please look at serial logs to determine error and then try again.");
            delay(5000);
        }
    }
    else
    {
        Serial.println("IMU ready!");
    }
    
    delay(100);

    // Configuring all tasks last updated time
    unsigned long currTime = millis();
    tasks.printImu.lastTime = currTime;
    tasks.led.lastTime = currTime;

}


void loop() 
{
    /*
        Since IMU update needs to be called frequently so fusion filter 
        can be computed, other slower tasks need to be kept track of
        in terms of their timing. So checking the time of them is needed.
    */
    unsigned long currentTime = millis();

    if (bno.wasReset()) 
    {
        Serial.print("Sensor was reset ");
        imuOutputDataConfig();
    }

    // Fetching data from IMU and updating IMU struct
    updateImuObject();

    // Only print data when time proper amount of time has elapsed
    if((currentTime - tasks.printImu.lastTime) >= tasks.printImu.period)
    {
        bool printAccel = true; 
        bool printGyro = true;
        bool printQuat = true;

        printImuData(printAccel, printGyro, printQuat);
        
        // Updating time here since it is possible that when it's time to print data, that the IMU wasn't ready to update.
        tasks.printImu.lastTime = currentTime;
    }

    if((currentTime - tasks.led.lastTime) >= tasks.led.period)
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        tasks.led.lastTime = currentTime;
    }

    // Quaternions are computed by the fusion filter and only update when mpu.update() runs frequently. 
    // Keep loop fast; avoid long delays or quat may freeze at (1,0,0,0). 
    delay(1);
}


/*
 **********************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************
*/

/**************************************************
 * Function Name: configureIMU
 * Description: 
**************************************************/

bool configureIMU()
{
    // Configuring communication over I2C
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    // TODO: Add in a multi attempt to connect to the bno

    // Setting up IMU with proper interrupt and reset pins
    if(bno.begin(IMU_I2C_ADDRESS, Wire, IMU_INT_PIN, IMU_RST_PIN) == false) 
    {
        Serial.println("IMU connection failed. Please check connection with examples.");
        return false;
    }
    else
    {
        Serial.println("IMU connection successful");
    }

    // Configuring what data to report
    imuOutputDataConfig();

    return true;
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
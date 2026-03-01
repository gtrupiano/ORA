/*
 **********************************************************************
 * INCLUDES
 **********************************************************************
*/

#include "ESP32_IMU.h"

/*
 **********************************************************************
 * DEFINES and CONSTANTS
 **********************************************************************
*/

// Bias values for calibration (X,Y,Z)
static const Vector_t ACCEL_BIAS = {-1558.80f, 64.50f, -890.80f};
static const Vector_t GYRO_BIAS = {-764.85f, -37.55f, -66.93f};
static const Vector_t MAG_BIAS = {214.03f, 184.00f, 114.59f};
static const Vector_t MAG_SCALE = {0.86f, 0.89f, 01.39f};

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
MPU9250 mpu;

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

    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    if(!mpu.setup(MPU_ADDRESS)) 
    {
        while(1) 
        {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    mpu.selectFilter(QuatFilterSel::MADGWICK);

    // Apply saved calibration values
    mpu.setAccBias(ACCEL_BIAS.x,  ACCEL_BIAS.y,  ACCEL_BIAS.z);
    mpu.setGyroBias(GYRO_BIAS.x, GYRO_BIAS.y, GYRO_BIAS.z);
    mpu.setMagBias(MAG_BIAS.x, MAG_BIAS.y, MAG_BIAS.z);
    mpu.setMagScale(MAG_SCALE.x, MAG_SCALE.y, MAG_SCALE.z);

    Serial.println("MPU9250 ready!");

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

    // Fetching data from IMU and updating IMU struct
    bool imuDataPresent = updateImuObject();

    // Only print data when time proper amount of time has elapsed
    if((currentTime - tasks.printImu.lastTime) >= tasks.printImu.period)
    {
        // Printing acceleration data only if imu data is present
        if(imuDataPresent)
        {
            bool printAccel = true; 
            bool printGyro = true;
            bool printQuat = true;
            bool printTemp = false;

            printImuData(printAccel, printGyro, printQuat, printTemp);
            
            // Updating time here since it is possible that when it's time to print data, that the IMU wasn't ready to update.
            tasks.printImu.lastTime = currentTime;
        }
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


/**************************************************
 * Function Name: updateImuObject
 * Description: 
**************************************************/

bool updateImuObject()
{
    bool imuDataPresent = false;
    if (mpu.update())
    {
        // Copying data to IMU structure
        imu.accel.x = mpu.getAccX();
        imu.accel.y = mpu.getAccY();
        imu.accel.z = mpu.getAccZ();

        imu.gyro.x = mpu.getGyroX();
        imu.gyro.y = mpu.getGyroY();
        imu.gyro.z = mpu.getGyroZ();

        imu.quat.w = mpu.getQuaternionW();
        imu.quat.x = mpu.getQuaternionX();
        imu.quat.y = mpu.getQuaternionY();
        imu.quat.z = mpu.getQuaternionZ();

        imu.temp = mpu.getTemperature();

        imu.sampleTime = millis();

        imuDataPresent = true;
    }

    return imuDataPresent;
}


/**************************************************
 * Function Name: printImuData
 * Description: 
**************************************************/

void printImuData(bool accel, bool gyro, bool quat, bool temp)
    {
    // Printing acceleration data
    if(accel)
    {
        Serial.print("[");
        Serial.print(imu.sampleTime);
        Serial.print("] X: ");
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
        Serial.print(imu.sampleTime);
        Serial.print("] X: ");
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
        Serial.print(imu.sampleTime);
        Serial.print("] quat (w,x,y,z): ");
        Serial.print(imu.quat.w, 6); Serial.print(", ");
        Serial.print(imu.quat.x, 6); Serial.print(", ");
        Serial.print(imu.quat.y, 6); Serial.print(", ");
        Serial.println(imu.quat.z, 6);
        Serial.println();
    }

    // Printing temperature data
    if(temp)
    {
        Serial.print("[");
        Serial.print(imu.sampleTime);
        Serial.print("] Temperature: ");
        Serial.print(imu.temp);
        Serial.println(" c");
        Serial.println();
    }
    
    // Ending line if any value are requested
    if(accel || gyro || quat || temp)
    {
        Serial.println("--------------------------");
        Serial.println();
    }
}
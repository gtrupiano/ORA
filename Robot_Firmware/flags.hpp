/****************************************************************
File Name  : flags.hpp
Description:
****************************************************************/

#ifndef FLAGS_HPP
#define FLAGS_HPP


/****************************************************************
*   INCLUDES
****************************************************************/

#include <Arduino.h>
#include <stdint.h>

/****************************************************************
*   DEFINES, CONSTANTS, ENUMS, STRUCTS
****************************************************************/

/*******************************
*   Defines
*******************************/

// Set to 1 if printing data is wanted or features only needed for debugging
#define DEBUG 0

#define IMU_PRINT_ACCEL 1
#define IMU_PRINT_GYRO 1
#define IMU_PRINT_QUAT 1


#endif // FLAGS_HPP
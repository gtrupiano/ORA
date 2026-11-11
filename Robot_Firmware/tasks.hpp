/****************************************************************
File Name  : tasks.hpp
Description:
****************************************************************/

#ifndef TASKS_HPP
#define TASKS_HPP


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

#define CAN_TASK_PERIOD_MS 20
#define IMU_TASK_PERIOD_MS 10

/*******************************
*   Enums
*******************************/

typedef enum
{
    CAN_TASK = 0,
    IMU_TASK,
    MAX_NUM_OF_TASKS
} Task_List_e;

/*******************************
*   Structures
*******************************/

typedef struct 
{
    // Time is in MS
    unsigned long period;
    unsigned long lastTime;
} Task_t;

#endif // TASKS_HPP
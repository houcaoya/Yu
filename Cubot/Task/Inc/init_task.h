#ifndef _INIT_TASK_H_
#define _INIT_TASK_H_

#include "freertos.h"
#include "queue.h" 
#include "rm_motor.h"
#include "pid.h"


void Init_Task(void *argument);


extern Motor_t motor3508;
extern SinglePID_t speedPID;


#endif 

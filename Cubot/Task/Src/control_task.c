#include "control_task.h"
#include "driver_can.h"
#include "init_task.h"
#include "projdefs.h"
#include "rm_motor.h"

UBaseType_t uxHighWaterMark_control_task;

/**
 * @brief 控制任务函数，周期性执行电机控制逻辑
 * @param argument 任务参数指针（未使用）
 * @note 该函数为FreeRTOS任务函数，周期性执行PID控制算法并输出电机控制指令
 */
void Control_Task(void *argument) { 
    (void) argument;
    TickType_t xLastWakeTime  = xTaskGetTickCount();
    while(1) {
        // 执行PID控制算法，计算电机输出值
        motor3508.treatedData.motor_output = One_Pid_Ctrl(500, motor3508.rawData.speed_rpm,&speedPID);
        
        // 填充电机控制数据
        MotorFillData(&motor3508, motor3508.treatedData.motor_output);
        
        // 通过CAN总线输出电机控制指令
        MotorCanOutput(can1, 0x200);
        
        // 任务延时，确保1ms周期执行
        vTaskDelayUntil(&xLastWakeTime , pdMS_TO_TICKS(1));

        
        uxHighWaterMark_control_task = uxTaskGetStackHighWaterMark(NULL);
    }
}










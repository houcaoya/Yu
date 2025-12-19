#include "can_task.h"
#include "driver_can.h"
#include "rm_motor.h"
#include "init_task.h"

UBaseType_t uxHighWaterMark_can;
/**
 * @brief 解算CAN通信数据任务函数
 * @param argument 任务参数，指向CAN实例结构体的指针
 * @note 该函数为RTOS任务函数，用于处理CAN总线数据接收和处理
 */
void CanTask_Process(void *argument)
{
    CAN_Instance_t *can = argument;
    CAN_RxBuffer_t task_local_buffer;
    /* 任务主循环 */
    while(1)
    {
        /* 等待并从CAN接收队列中获取数据，获取成功后进行相应处理 */
        if (xQueueReceive(can->xQueueCan, &task_local_buffer, portMAX_DELAY) == pdTRUE)
        {   
            /* 根据CAN实例类型调用对应的电机接收回调函数 */
            if(can == &can1)
                MotorRxCallback(&can1, &task_local_buffer);
            else if(can == &can2)
                MotorRxCallback(&can2, &task_local_buffer);
        }
        
        /* 获取任务堆栈使用情况，用于监控任务堆栈使用峰值 */
        uxHighWaterMark_can = uxTaskGetStackHighWaterMark(NULL);
    }
}


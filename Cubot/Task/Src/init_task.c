#include "init_task.h"
#include "cmsis_os2.h"
#include "control_task.h"
#include "driver_usart.h"
#include "driver_can.h"
#include "referee_task.h"
#include "rm_motor.h"
#include "shoot_task.h"
#include "uart_task.h"
#include "can_task.h"
#include "control_task.h"

UBaseType_t uxHighWaterMark_init;

static void BasePID_Init_All(void)
{
    /* 初始化PID控制参数 */
}
/**
 * @brief 初始化任务函数
 * @param argument 任务参数指针，本函数中未使用
 * 
 * 该函数负责系统的初始化工作，包括UART、CAN硬件初始化，队列创建，
 * 电机初始化，以及创建其他任务。初始化完成后会删除自身任务。
 */
void Init_Task(void *argument)
{
    (void)argument;

    vTaskSuspendAll();

    UARTx_Init(&uart1);
    UARTx_Init(&uart3);
    UARTx_Init(&uart4);
    UARTx_Init(&uart5);
    /* 初始化CAN硬件并打开CAN设备 */
    CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can1);
	CAN_Open(&can2);

    BasePID_Init_All();
    /* 创建UART任务用于收发数据 */
    xTaskCreate(Referee_Task, "Referee_Task", 512, NULL, osPriorityNormal-1, NULL);
    xTaskCreate(Brain_Task, "Brian_Task", 512, NULL, osPriorityNormal-1, NULL);
    xTaskCreate(Print_Task,"Print_Task",256,NULL,osPriorityNormal-2,NULL);
    /* 创建CAN任务用于接收数据 */
    xTaskCreate(CanTask_Process, "CanTask_Process", 256, &can1, osPriorityNormal+1, NULL);
    xTaskCreate(CanTask_Process, "CanTask_Process", 256, &can2, osPriorityNormal+1, NULL);
    /* 创建运动学解算任务用于处理数据 */
    xTaskCreate(Shoot_Task,"Shoot_Task",256,NULL,osPriorityNormal,NULL);
    xTaskCreate(Chassis_Task,"Chassis_Task",256,NULL,osPriorityNormal,NULL);
    xTaskCreate(Holder_Task,"Holder_Task",512,NULL,osPriorityNormal,NULL);

    /* 初始化电机驱动任务 */
    Motor_DriverInit();
    xTaskResumeAll();

    #ifdef DEBUG
    uxHighWaterMark_init = uxTaskGetStackHighWaterMark(NULL);
    #endif
    /* 删除初始化任务自身，释放资源 */
    vTaskDelete(NULL);
}

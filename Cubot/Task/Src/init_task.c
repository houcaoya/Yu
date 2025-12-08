#include "init_task.h"
#include "cmsis_os2.h"
#include "control_task.h"
#include "driver_usart.h"
#include "driver_can.h"
#include "rm_motor.h"
#include "uart_task.h"
#include "can_task.h"
#include "control_task.h"

UBaseType_t uxHighWaterMark_init;
Motor_t motor3508;
SinglePID_t speedPID;


static void BasePID_Init_All(void)
{
    BasePID_Init(&speedPID,10, 0, 0, 16000, 0, 0, 
                0,  0, 
                1600);

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
    /* 避免编译器警告，标记参数为未使用 */
    (void)argument;

    /* 进入临界区，保护初始化过程 */
    taskENTER_CRITICAL();

    /* 初始化UART5硬件并注册回调函数 */
    UARTx_Init(&huart5, uart5_callback);

    /* 初始化CAN硬件并打开CAN设备 */
    CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can1);
	CAN_Open(&can2);

    /* 初始化3508电机 */
    MotorInit(&motor3508, 0, Motor3508, 1, CAN1, 0x201);

    BasePID_Init_All();
    /* 创建UART任务用于处理串口通信 */
    xTaskCreate(Uart_Task, "Uart_Task", 512, NULL, osPriorityNormal, NULL);
    /* 创建CAN任务用于处理CAN通信 */
    xTaskCreate(Can_Task, "Can1_Task", 256, &can1, osPriorityNormal, NULL);
    xTaskCreate(Can_Task, "Can2_Task", 256, &can2, osPriorityNormal, NULL);

    xTaskCreate(Control_Task, "Control_Task", 256, NULL, osPriorityNormal, NULL);

    /* 退出临界区 */
    taskEXIT_CRITICAL();

    /* 获取init任务的栈空间使用情况 */
    uxHighWaterMark_init = uxTaskGetStackHighWaterMark(NULL);

    /* 删除初始化任务自身，释放资源 */
    vTaskDelete(NULL);
}


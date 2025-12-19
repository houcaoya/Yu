/**
 **********************************************************************************
 * @file       	driver_usart.c
 * @brief   	驱动层，串口管理器配置文件，用户回调重定义
 * @details  	主要包括构建串口管理器，提供串口初始化和用户回调重定义
 * @author      RyanJiao  any question please send mail to 1095981200@qq.com
 * @date        2024-07-24
 * @version     V1.2
 * @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date        	<th>Version  	<th>Author    	<th>Description
 * <tr><td>2021-08-12  	<td>1.0      	<td>RyanJiao  	<td>创建初始版本
 * <tr><td>2024-04-12  	<td>1.2      	<td>EmberLuo  	<td>为接收回调增加DMA双缓冲
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                          How to use this driver
 ==============================================================================

    添加driver_can.h

    1. 调UARTx_Init() 将 句柄 和 用户定义的接收回调函数 拷贝至UART结构体  （回调函数中对接收到的数据进行 ID识别 和 合并解算）

    2. 用户编写 UART_RxBuffer，填入 目标缓存区地址 和 数据长度。

    3. 调用UART_Open() 传入 UART_Object 和 用户编写 UART_RxBuffer。

    4. 将 UART_Idle_Handler 添加到 stm32H7xx_it.c 的 USARTx_IRQHandler() 中，调用用户编写的同一个 UART_RxBuffer_t 。

    5. 应用层编写 UART_TxBuffer_t （发送缓存区结构体），填入待发送字节数组首地址和字节长度

    6. 调UART_Send()传入 UART设备结构体 和 UART_TxBuffer结构体，将数据发送出去

 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
    对DMA中NDTR寄存器描述：
        This register can be written only
        when the stream is disabled. When the stream is enabled, this register is read-only,
        indicating the remaining data items to be transmitted. This register decrements after each
        DMA transfer.
        指明了DMA中待传输的剩余数据个数 每次DMA传输完成后自动减一
        参考手册中对idle空闲中断触发条件的描述：
 **********************************************************************************
 */
#include "driver_usart.h"
#include "usart.h"


UART_Object uart1, uart2, uart3, uart4, uart5, uart6;

/**
 * @brief 初始化UART接口
 * @param handle UART句柄指针，用于配置和控制UART硬件
 * @param rxIdleCallback UART接收空闲回调函数指针，当UART接收空闲时被调用
 * @return 无返回值
 * 
 * 该函数用于初始化UART接口，包括设置句柄、配置接收空闲中断以及启动DMA接收。
 * 如果提供了接收空闲回调函数，则会启用UART接收空闲中断并启动DMA接收功能。
 */
void UARTx_Init(UART_Object* uart, UART_RxIdleCallback rxIdleCallback)
{
	uart->is_first_idle = 0;

	uart->stream_buffer = xStreamBufferCreate(648, 1);
	/* 如果提供了接收空闲回调函数，则配置相关中断和DMA接收功能 */
	if (rxIdleCallback!=NULL) 
	{
		uart->RxIdleCallback = rxIdleCallback;
		/* 清除UART接收空闲标志位 */
		__HAL_UART_CLEAR_IDLEFLAG(uart->Handle);
		/* 使能UART接收空闲中断 */
		__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE); 
		/* 启动UART DMA接收，接收缓冲区大小为200字节 */
		HAL_UART_Receive_DMA(uart->Handle, uart->uart_RxBuffer[uart->activeBuffer].Data,200);		
    }
}

/**
 * @brief UART空闲中断处理函数
 * @param uart UART对象指针，包含UART句柄、回调函数、缓冲区等信息
 * @note 该函数处理UART空闲中断，当检测到总线空闲时停止DMA传输，
 *       清除相关标志位，并将接收到的数据发送到流缓冲区中
 */
void UART_Idle_Handler(UART_Object *uart)
{
	// 检查UART空闲标志位
	if ((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET))
    {
    	// 停止当前正在进行的DMA接收操作
		HAL_UART_DMAStop(uart->Handle);          
		
		// 清除UART空闲标志位，为下一次接收做准备
		__HAL_UART_CLEAR_IDLEFLAG(uart->Handle); 
		
		// 清除可能存在的溢出错误标志位
     	 __HAL_UART_CLEAR_OREFLAG(uart->Handle);
     	 
		// 判断是否为首次接收到空闲信号，若是则标记为非首次，否则处理数据
		if (uart->is_first_idle == 0)  
       		uart->is_first_idle = 1;
		else 
		{
			
			uint16_t received_len = sizeof(uart->uart_RxBuffer[uart->activeBuffer].Data) - __HAL_DMA_GET_COUNTER(uart->Handle->hdmarx);
			// 将接收到的数据从DMA缓冲区发送到流缓冲区中
			if(uart->stream_buffer != NULL)
			{
				BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
				xStreamBufferSendFromISR(uart->stream_buffer, &uart->uart_RxBuffer[uart->activeBuffer].Data, received_len, &pxHigherPriorityTaskWoken);
				
				// 根据任务优先级情况决定是否进行任务切换
				portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
			}
		}
		HAL_UART_DMAResume(uart->Handle);
		HAL_UART_Receive_DMA(uart->Handle, uart->uart_RxBuffer[uart->activeBuffer].Data,200);      
    }
}

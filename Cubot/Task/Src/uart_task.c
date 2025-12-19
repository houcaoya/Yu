#include "uart_task.h"
#include "driver_usart.h"
#include "init_task.h"

uint8_t uart5_txBuffer[20];
uint8_t uart5_rxBuffer[20];

UBaseType_t uxHighWaterMark_uart;
/**
 * @brief UART5接收回调函数
 * 
 * 当UART5接收到数据时，将接收到的数据缓冲区指针发送到队列中
 * 供其他任务处理
 * 
 * @param recBuffer 指向接收数据缓冲区的指针
 * @return uint8_t 返回状态码，固定返回0
 */
uint8_t uart5_callback(uint8_t *recBuffer)
{
    (void)recBuffer;
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	if(uart5.stream_buffer != NULL)
	{
		xStreamBufferSendFromISR(uart5.stream_buffer, &uart5.uart_RxBuffer[uart5.activeBuffer].Data, sizeof(uart5.uart_RxBuffer[uart5.activeBuffer].Data), &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
	return 0;
}

/**
 * @brief UART任务函数，用于处理UART通信的发送和接收
 * @param argument 任务参数指针（未使用）
 * @return 无返回值
 * 
 * 该函数首先初始化UART5的DMA发送，然后进入循环等待接收队列中的数据。
 * 当接收到数据时，重新触发UART5的DMA发送操作。
 */
void Uart_Task(void *argument)
{
    /* 避免编译器警告，标记参数未被使用 */
    (void)argument;
    size_t receivedBytes;
    
    /* 初始化UART5的DMA发送功能 */
    HAL_UART_Transmit_DMA(&huart5, uart5_txBuffer, sizeof(uart5_txBuffer));
    
    /* 任务主循环 */
    while(1)
    {
		receivedBytes = xStreamBufferReceive(uart5.stream_buffer, uart5.uart_RxBuffer[uart5.activeBuffer].Data, sizeof(uart5.uart_RxBuffer[uart5.activeBuffer].Data), portMAX_DELAY);
        if(receivedBytes > 0)
			HAL_UART_Transmit_DMA(&huart5, uart5_txBuffer, sizeof(uart5_txBuffer));
        uxHighWaterMark_uart = uxTaskGetStackHighWaterMark(NULL);
    }
}

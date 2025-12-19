#ifndef _DRIVER_USART_H_
#define _DRIVER_USART_H_

#include "stm32h7xx_hal.h"
#include "main.h"
#include "usart.h"
#ifdef FREERTOS_ENABLED
#include "freertos.h"
#include "queue.h"
#include "stream_buffer.h"
#endif


/**
 * @brief	UART接收缓冲区
 */
typedef struct
{
    uint8_t Data[200];
    uint16_t Size;
} UART_RxBuffer_t;

/**
 * @brief	UART发送缓冲区
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} UART_TxBuffer_t;

/**
 * @brief   串口结构体,包含句柄,接收回调和缓存区
 */
typedef struct
{
    UART_HandleTypeDef *Handle;
    UART_RxBuffer_t uart_RxBuffer[2];
	uint8_t recv_buff_size;
	uint8_t is_first_idle;
    StreamBufferHandle_t stream_buffer;
	uint8_t activeBuffer;
} UART_Object;




/**
 * @brief  串口设备中断函数，执行中断DMA操作，调用串口用户回调函数，需要将其添加到stm32h7xx_it.c中
 * @param[in]  uart		    串口结构体, 标明串口句柄和回调函数
 * @param[in]  rx_buffer		发送缓存区, 用户定义。注意！ 此处的rxBuffer应该与 UART_Open中调用rxBuffer一致
 * @retval
 */
void UART_Idle_Handler(UART_Object *uart);
void UARTx_Init(UART_Object* uart);


extern UART_Object uart1;
extern UART_Object uart2;
extern UART_Object uart3;
extern UART_Object uart4;
extern UART_Object uart5;
extern UART_Object uart6;

#endif

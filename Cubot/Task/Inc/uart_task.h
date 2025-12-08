#ifndef _UART_TASK_H_
#define _UART_TASK_H_

#include "driver_usart.h"


uint8_t uart5_callback(uint8_t *recBuffer);

void Uart_Task(void *argument);


extern QueueHandle_t uart5_xQueueUart;



#endif 

#ifndef _DRIVER_CAN_H_
#define _DRIVER_CAN_H_

#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include "linux_list.h"
#include "freertos.h"
#include "queue.h"

/**
 * @brief	CAN设备号枚举
 */
typedef enum {
    CAN1 = 0x01U,
    CAN2 = 0x02U
} CanNumber;

/**
 * @brief	CAN接收缓冲区
 */
typedef struct
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
} CAN_RxBuffer_t;

/**
 * @brief	CAN发送缓冲区
 */
typedef struct
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8];
} CAN_TxBuffer_t;

/**
 * @brief	CAN设备实例结构体
 */
typedef struct _CAN_Instance_t {
    FDCAN_HandleTypeDef *canHandler;                    // CAN句柄
    CAN_RxBuffer_t rxBuffer;                         // 接收缓存区结构体
    CAN_TxBuffer_t txBuffer;                            // 发送缓存区结构体
    uint8_t (*RxCallBackCAN)(struct _CAN_Instance_t *); // 接收回调函数
    QueueHandle_t xQueueCan;                            // CAN接收队列
    uint8_t activeBuffer;                               // 当前活动缓冲区索引 (0 或 1)
} CAN_Instance_t;

/**
 * @brief   CAN用户回调函数
 */
typedef uint8_t (*CAN_RxCpltCallback)(CAN_Instance_t *);

/**
 * @brief  CAN初始化，将句柄和接收回调拷贝至CAN结构体
 * @param[in]  h_can		CAN句柄
 */
void CANx_Init(FDCAN_HandleTypeDef *h_can, CAN_RxCpltCallback rxCallback);

/**
 * @brief 打开CAN设备，配置过滤器为空，使能fifo0接收到新信息中断，注册用户回调
 * @param[in]	can	CAN设备
 */
uint8_t CAN_Open(CAN_Instance_t *can);

/**
 * @brief 通过CAN设备发送数据
 * @param[in] txBuffer CAN的发送缓冲区
 *
 */
uint8_t CAN_Send(CAN_Instance_t *can, CAN_TxBuffer_t *txBuffer);

/**
 * @brief 通过CAN设备发送数据
 *	@param[in] txBuffer CAN的发送缓冲区
 *
 */
uint8_t CAN1_rxCallBack(CAN_Instance_t *canObject);
uint8_t CAN2_rxCallBack(CAN_Instance_t *canObject);

extern CAN_Instance_t can1;
extern CAN_Instance_t can2;

#endif

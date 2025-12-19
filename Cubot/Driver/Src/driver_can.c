/**
 **********************************************************************************
 * @file        driver_can.c
 * @brief       驱动层，CAN外设配置文件
 * @details     主要包括CAN过滤器初始化，指定中断回调函数，不同CAN_ID下的数据收发函数
 * @date        2024-07-24
 * @version     V1.3
 * @copyright   Copyright (c) 2021-2121  中国矿业大学CUBOT战队
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2021-08-12   <td>1.0         <td>RyanJiao    <td>创建初始版本
 * <tr><td>2021-10-09   <td>1.0         <td>RyanJiao    <td>规范变量名，确定了CAN_TxBuffer结构
 * <tr><td>2024-06-04   <td>1.3         <td>EmberLuo    <td>创建CAN_Instance_t结构体，整合了收发缓存区结构体
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                            How to use this driver
 ==============================================================================

    添加driver_can.h

    1. 创建 (*CAN_RxCpltCallback)(CAN_Instance_t *) 类型的用户回调

    2. 调用 CANx_Init() 将 句柄 和 用户定义的接收回调函数 拷贝至CAN结构体（回调函数中对接收到的数据进行ID识别和合并解算）

    3. 调用 CAN_Open() 传入实例化的结构体，开启can设备

    4. 应用层编写 CAN_TxBuffer_t （发送缓存区结构体），填入待发送的字节数据和目标ID

    5. 调用 CAN_Send() 传入 can设备结构体 和 TxBuffer结构体，将数据发送出去

 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
    CAN设备讲解

        1.过滤器:

        2.FIFO队列: （参考资料：https://blog.csdn.net/flydream0/article/details/8155942 ）

            当bxCAN接收到报文，经过过滤器过滤后，会将报文存储到FIFO中。每个过滤器组都会关联一个FIFO，--> FIFO0和FIFO1
            这个FIFO为3级邮箱深度（每个FIFO由三个邮箱组成），且完全由硬件来管理，节约CPU资源，简化了  FIFO0 --> 3 x mailbox
            软件并保证了数据的一致性。应用程序只能通过读取FIFO输出邮箱，来读取FIFO中最先收到的报文。

            FIFO共有五个状态：空状态，挂号1状态，挂号2状态，挂号3状态，溢出状态

            在初始化状态时，FIFO是处于空状态的，当接收到一个报文时，这个报文存储到FIFO内部
            的邮箱中，此时，FIFO的状态变成挂号1状态，如果应用程序取走这个消息，则FIFO恢复空状态。
            现在假设FIFO处于挂号1状态，即已接收到一个报文，且应用程序不没来得及取走接收到的报文，
            此时若再次接收到一个报文，那么FIFO将变成挂号2状态，以此类推，由于FIFO共有3个邮箱，只能缓存3个报文，
            因此，当接收到3个报文（假设期间应用程序从未取走任何报文）时，此时FIFO已满，若再来一个报文时，
            已无法再存储，此时FIFO将变成溢出状态。

            STM32中与CAN接收相关的中断有三个：
            接收中断：每当bxCAN接收到一个报文时产生一个中断。
            FIFO满中断：当FIFO满时，即存储了3个报文时产生的中断。
            FIFO溢出中断：当FIFO溢出时产生此中断。
 **********************************************************************************
 */
#include "driver_can.h"
#include "freertos.h"
#include "queue.h"
/**
 * @brief CAN设备实例化结构体
 * 
 * 定义了CAN1的发送缓冲区配置和设备相关信息
 * - txBuffer.txHeader: 发送消息头配置，设置为标准ID、数据帧、8字节数据长度等
 */
CAN_Instance_t can1 = {
    .txBuffer.txHeader =
        {
            .IdType              = FDCAN_STANDARD_ID,        /**< 使用标准ID格式 */
            .TxFrameType         = FDCAN_DATA_FRAME,         /**< 发送数据帧 */
            .DataLength          = FDCAN_DLC_BYTES_8,        /**< 数据长度为8字节 */
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,         /**< 错误状态指示为主动模式 */
            .BitRateSwitch       = FDCAN_BRS_OFF,            /**< 不使用可变速率切换 */
            .FDFormat            = FDCAN_CLASSIC_CAN,        /**< 使用经典CAN格式 */
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,       /**< 不记录发送事件 */
            .MessageMarker       = 0x00,                     /**< 消息标记为0 */
        }
};
/**
 * @brief CAN设备实例化结构体
 * 
 * 定义了CAN2的发送缓冲区配置和设备相关信息
 * 结构与can1相同，用于区分两个不同的CAN控制器
 */
CAN_Instance_t can2 = {
    .txBuffer.txHeader =
        {
            .IdType              = FDCAN_STANDARD_ID,        /**< 使用标准ID格式 */
            .TxFrameType         = FDCAN_DATA_FRAME,         /**< 发送数据帧 */
            .DataLength          = FDCAN_DLC_BYTES_8,        /**< 数据长度为8字节 */
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,         /**< 错误状态指示为主动模式 */
            .BitRateSwitch       = FDCAN_BRS_OFF,            /**< 不使用可变速率切换 */
            .FDFormat            = FDCAN_CLASSIC_CAN,        /**< 使用经典CAN格式 */
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,       /**< 不记录发送事件 */
            .MessageMarker       = 0x00,                     /**< 消息标记为0 */
        }
};
/**
 * @brief 初始化CAN控制器
 * @param h_can CAN控制器句柄指针
 * @param rxCallback CAN接收完成回调函数指针
 * @retval 无
 * 
 * 该函数根据传入的CAN控制器句柄，初始化对应的CAN控制器结构体，
 * 并设置接收完成回调函数。支持FDCAN1和FDCAN2两个控制器的初始化。
 * 同时创建FreeRTOS消息队列用于异步处理接收到的CAN数据。
 */
void CANx_Init(FDCAN_HandleTypeDef *h_can, CAN_RxCpltCallback rxCallback)
{
    CAN_Instance_t *pCan = NULL;

    if (h_can->Instance == FDCAN1) pCan = &can1;
    else if (h_can->Instance == FDCAN2) pCan = &can2;

    if (pCan != NULL) {
        pCan->canHandler     = h_can;
        pCan->RxCallBackCAN  = rxCallback;
        pCan->xQueueCan      = xQueueCreate(32, sizeof(CAN_RxBuffer_t));
        pCan->tx_congest_cnt = 0;
    }
}


/**
 * @brief 初始化并启动CAN通信接口
 * 
 * 该函数用于配置CAN控制器的接收过滤器、全局过滤策略、启动CAN模块，
 * 并使能接收FIFO中新消息的中断通知。
 * 
 * @param can 指向CAN实例结构体的指针，包含CAN句柄等信息
 * @return uint8_t 返回操作结果：0表示成功，1表示失败
 */
uint8_t CAN_Open(CAN_Instance_t *can)
{
    FDCAN_FilterTypeDef filter; 

    /* 配置接收所有标准帧到 FIFO0 */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;
    if (HAL_FDCAN_ConfigFilter(can->canHandler, &filter) != HAL_OK) return 1;

    /* 配置接收所有标准帧到 FIFO1 (冗余或作为备用) */
    filter.FilterIndex  = 1;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    if (HAL_FDCAN_ConfigFilter(can->canHandler, &filter) != HAL_OK) return 1;

    HAL_FDCAN_ConfigGlobalFilter(can->canHandler, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_Start(can->canHandler);

    /* 使能两个FIFO的新消息中断 */
    HAL_FDCAN_ActivateNotification(can->canHandler, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(can->canHandler, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    return 0;
}

/**
 * @brief 通过CAN接口发送数据
 * @param can: CAN实例指针，包含CAN控制器句柄和发送缓冲区信息
 * @param bufferTx: 发送数据缓冲区指针，包含要发送的CAN消息头和数据
 * @retval uint8_t: 发送结果，1表示发送成功，0表示发送失败
 * 
 * 该函数首先检查发送FIFO是否有空闲位置，如果有则尝试发送数据。
 * 如果发送FIFO长时间满载，则会采取措施清除发送请求以避免死锁。
 */
uint8_t CAN_Send(CAN_Instance_t *can, CAN_TxBuffer_t *bufferTx)
{
    /* 复制发送消息头信息到CAN实例的发送缓冲区 */
    can->txBuffer.txHeader.Identifier = bufferTx->txHeader.Identifier;
    can->txBuffer.txHeader.IdType     = bufferTx->txHeader.IdType;
    can->txBuffer.txHeader.DataLength = bufferTx->txHeader.DataLength;

    // 检查 FIFO 是否有空位
    if (HAL_FDCAN_GetTxFifoFreeLevel(can->canHandler) > 0)
    {
        can->tx_congest_cnt = 0; 
        if (HAL_FDCAN_AddMessageToTxFifoQ(can->canHandler, &can->txBuffer.txHeader, bufferTx->data) == HAL_OK)
        {
            return 1; // 发送成功
        }
        else
        {
            return 0; // 发送失败
        }
    }
    else
    {
        // 发送FIFO满，增加拥塞计数器
        can->tx_congest_cnt++;
        if (can->tx_congest_cnt >= 10)
        {
            HAL_FDCAN_AbortTxRequest(can->canHandler, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
            can->tx_congest_cnt = 0;
        }
        return 0; 
    }
}
/**
 * @brief 内部函数：通用 RX 处理逻辑，合并 FIFO0 和 FIFO1
 * @param h_can: CAN控制器句柄
 * @param rxFifo: 接收FIFO编号（FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1）
 * 
 * 此函数统一处理来自两个接收FIFO的中断，从中读取数据并通过回调函数传递给上层应用
 */
static void CAN_CommonRxHandler(FDCAN_HandleTypeDef *h_can, uint32_t rxFifo)
{
    CAN_Instance_t *pCan = NULL;

    if (h_can->Instance == FDCAN1) pCan = &can1;
    else if (h_can->Instance == FDCAN2) pCan = &can2;

    if (pCan == NULL) return;

    if (HAL_FDCAN_GetRxMessage(h_can, rxFifo, &pCan->rxBuffer.rxHeader, pCan->rxBuffer.data) == HAL_OK)
    {
        if (pCan->RxCallBackCAN != NULL)
        {
            pCan->RxCallBackCAN(pCan);
        }
    }
}

// 统一调用通用处理函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h_can, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    CAN_CommonRxHandler(h_can, FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *h_can, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    CAN_CommonRxHandler(h_can, FDCAN_RX_FIFO1);
}

/**
 * @brief  CAN接收中断回调 (通用)
 * @note   将整个 rxBuffer (含ID和数据) 发送至队列
 */
static uint8_t CAN_General_RxCallback(CAN_Instance_t *canObject)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 在中断服务例程中向队列发送接收到的CAN数据
    xQueueSendFromISR(canObject->xQueueCan, &canObject->rxBuffer, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

uint8_t CAN1_rxCallBack(CAN_Instance_t *canObject) { return CAN_General_RxCallback(canObject); }
uint8_t CAN2_rxCallBack(CAN_Instance_t *canObject) { return CAN_General_RxCallback(canObject); }

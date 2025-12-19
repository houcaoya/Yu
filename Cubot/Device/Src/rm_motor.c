/**
 **********************************************************************************
 * @file       	rm_motor.c
 * @brief       设备层，电机控制代码。设备调用驱动层产生的数据结构进行再设备层完成构建，不暴露給用户
 * @details     通过调用CAN驱动层接收处理和发送电机的相关数据
 * @date        2024-07-10
 * @version     V1.1
 * @copyright   Copyright (c) 2021-2121  中国矿业大学CUBOT战队
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2021-10-10   <td>1.0         <td>RyanJiao    <td>完成收发函数编写
 * <tr><td>2024-04-12   <td>1.1         <td>EmberLuo    <td>删除文件中电机返回值的滤波，加入电机多圈角度换算
 * <tr><td>2024-06-04   <td>1.1         <td>EmberLuo    <td>适配driver_can文件，函数参数均改为CAN_Instance_t类型
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                        How to use this module
 ==============================================================================

    添加driver_can.h

    1. 创建Motor结构体，作为电机实例。

    2. 调用MotorInit()，初始化电机静态数据。

    3. 在启动CAN时注册的CANx_rxCallBack回调中判断ID并添加MotorRxCallback()，接收电机动态数据。

    4. 经过PID计算后产生待发送数据OutputCurrent。

    5. 发送数据应当调用MotorFillData()填写对应控制ID下的待发送数据

    6. 所有数据填写完毕后调用MotorCanOutput()发送对应CAN设备下特定控制ID的CAN数据

 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
 */
#include "rm_motor.h"
#include "user_lib.h"

static CAN_TxBuffer_t txBuffer[2][3];

static Motor_t *Motor_QuickMap[2][0x0C] = {NULL};

void Motor_DriverInit(void)
{
    // 定义三个标准控制ID: 0=0x1FF, 1=0x200, 2=0x2FF (顺序需与填充逻辑对应)
    const uint16_t std_ids[3] = {0x1FF, 0x200, 0x2FF};
    
    for (int i = 0; i < 2; i++) // 遍历 CAN1, CAN2
    {
        for (int j = 0; j < 3; j++) // 遍历 3 个控制帧
        {
            txBuffer[i][j].txHeader.Identifier = std_ids[j];
            txBuffer[i][j].txHeader.DataLength = FDCAN_DLC_BYTES_8;
            txBuffer[i][j].txHeader.IdType     = FDCAN_STANDARD_ID;
            txBuffer[i][j].txHeader.TxFrameType= FDCAN_DATA_FRAME;
            txBuffer[i][j].txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            txBuffer[i][j].txHeader.BitRateSwitch = FDCAN_BRS_OFF;
            txBuffer[i][j].txHeader.FDFormat = FDCAN_CLASSIC_CAN;
            txBuffer[i][j].txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            txBuffer[i][j].txHeader.MessageMarker = 0;
        }
    }
}



/**
 * @brief 将电机编码器数据转换为角度值
 * @param motor 指向电机结构体的指针，包含编码器原始数据、参数和处理后的数据
 * @note 该函数根据编码器的减速比和零点偏移量，将原始编码器数据转换为以零点为中心的±180°角度值
 */
static void MotorEcdtoAngle(Motor_t *motor)
{
    if ((motor->param).reduction_ratio == 1) 
    {
        /* 处理编码器数据越界情况，确保数据在合理范围内 */
        if ((&motor->param)->ecd_offset < ((&motor->param)->ecd_range / 2)) 
        {
            if ((motor->rawData.raw_ecd) > (&motor->param)->ecd_offset + (&motor->param)->ecd_range / 2)
                motor->rawData.raw_ecd = (motor->rawData.raw_ecd) - (&motor->param)->ecd_range;
        } 
        else 
        {
            if ((motor->rawData.raw_ecd) < (&motor->param)->ecd_offset - (&motor->param)->ecd_range / 2)
                motor->rawData.raw_ecd = (motor->rawData.raw_ecd) + (&motor->param)->ecd_range;
        }
        /*将编码器数据特定的零点位置转换为+-180°的角度*/
        (&motor->treatedData)->angle = K_ECD_TO_ANGLE * ((motor->rawData.raw_ecd) - (&motor->param)->ecd_offset);
    }
}

/**
 * @brief  电机数据更新回调
 */
static uint8_t CAN_update_data(RawData_t *raw, TreatedData_t *treated, uint8_t *data)
{
    treated->last_ecd    = raw->raw_ecd;
    raw->raw_ecd         = (int16_t)(data[0] << 8 | data[1]);
    raw->speed_rpm       = (int16_t)(data[2] << 8 | data[3]);
    raw->torque_current  = (int16_t)(data[4] << 8 | data[5]);
    raw->temperature     = data[6];
    treated->fps++;
    return 0;
}

/**
 * @brief 电机初始化，设置静态参数，包括编码器零位，电机类型，减速比和id
 *
 * @param motor 		所要初始化的电机
 * @param ecdOffset 	编码器零位
 * @param type 			电机类型
 * @param gearRatio 	减速比，目前只支持转子与输出轴之比为2:1和3:1的情况
 * @param canx 			使用的是CAN1还是CAN2
 * @param id 			CAN_ID
 */
void MotorInit(Motor_t *motor, uint16_t ecdOffset, motor_type type, uint16_t gearRatio, CanNumber canx, uint16_t id)
{
    motor->param.ecd_offset      = ecdOffset;
    motor->param.motor_type      = type;
    motor->param.can_id          = id;
    motor->param.reduction_ratio = gearRatio;
    motor->param.can_number      = canx;
    motor->MotorUpdate           = CAN_update_data;

    if (id >= 0x200 && id <= 0x20B) 
    {
        uint8_t can_idx = (canx == CAN2) ? 1 : 0;
        uint8_t id_idx  = id - 0x200;
        Motor_QuickMap[can_idx][id_idx] = motor;
    }

    switch (type) 
    {
        case Motor3508: 
            motor->param.current_limit = CURRENT_LIMIT_FOR_3508;
            motor->param.ecd_range     = ECD_RANGE_FOR_3508;
            break;
        case Motor6020: 
            motor->param.current_limit = VOLTAGE_LIMIT_FOR_6020;
            motor->param.ecd_range     = ECD_RANGE_FOR_6020;
            break;
        case Motor2006: 
            motor->param.current_limit = CURRENT_LIMIT_FOR_2006;
            motor->param.ecd_range     = ECD_RANGE_FOR_2006;
            break;
        default: break;
    }
}

/**
 * @brief 电机CAN接收回调函数
 * @param canObject CAN实例对象指针，包含接收到的CAN数据
 * @note 该函数用于处理电机相关的CAN接收数据，解析电机反馈信息并更新电机状态
 */
void MotorRxCallback(CAN_Instance_t *canObject, CAN_RxBuffer_t *bufferRx)
{
    uint32_t id = bufferRx->rxHeader.Identifier;
    
    // 1. 安全检查：ID是否在支持范围内
    if (id < 0x200 || id > 0x20B) return;

    // 2. 计算索引
    uint8_t can_idx = (canObject->canHandler == &hfdcan2) ? 1 : 0;
    uint8_t id_idx  = id - 0x200;

    // 3. 直接获取对象 (无需 MotorFind 遍历)
    Motor_t *motor = Motor_QuickMap[can_idx][id_idx];

    // 4. 处理数据
    if (motor != NULL) 
    {
        motor->online_cnt = 0; 
        motor->MotorUpdate(&motor->rawData, &motor->treatedData, bufferRx->data);
        MotorEcdtoAngle(motor);
    }
}


/**
 * @brief  将treatedData.motor_output限幅后填入发送缓存区等待发送。
 */
void MotorFillData(Motor_t *motor, int32_t output)
{
    // 1. 设置并限幅
    motor->treatedData.motor_output = output;
    (&motor->treatedData)->motor_output = LIMIT((&motor->treatedData)->motor_output, -motor->param.current_limit, motor->param.current_limit);

    // 2. 准备参数
    uint8_t can_idx = (motor->param.can_number == CAN2) ? 1 : 0;
    uint16_t id     = motor->param.can_id;
    int16_t val     = (int16_t)motor->treatedData.motor_output;
    
    int buf_idx = -1; // 0:1FF, 1:200, 2:2FF
    int offset  = 0;

    // 3. 计算缓冲区索引和字节偏移
    if (id >= 0x201 && id <= 0x204) {
        buf_idx = 1; // ID: 0x200
        offset  = (id - 0x201) * 2;
    } 
    else if (id >= 0x205 && id <= 0x208) {
        buf_idx = 0; // ID: 0x1FF
        offset  = (id - 0x205) * 2;
    } 
    else if (id >= 0x209 && id <= 0x20B) {
        buf_idx = 2; // ID: 0x2FF
        offset  = (id - 0x209) * 2;
    }

    // 4. 填充数据
    if (buf_idx >= 0) {
        txBuffer[can_idx][buf_idx].data[offset]     = (val >> 8) & 0xFF;
        txBuffer[can_idx][buf_idx].data[offset + 1] = val & 0xFF;
    }
}

/**
 * @brief  将特定ID的CAN_TxBuffer_t发送出去。
 */
uint16_t MotorCanOutput(CAN_Instance_t can, int16_t IDforTxBuffer)
{
    uint8_t can_idx = (can.canHandler == &hfdcan2) ? 1 : 0;
    int buf_idx = -1;

    // 匹配 Tx Buffer 索引 (与 MotorFillData 中的逻辑对应)
    switch (IDforTxBuffer) {
        case 0x1FF: buf_idx = 0; break;
        case 0x200: buf_idx = 1; break;
        case 0x2FF: buf_idx = 2; break;
        default: return 1; // 错误 ID
    }

    // 直接发送，无需多余判断
    // 注意：确保 driver_can.c 的 CAN_Send 能够处理我们构造的 txBuffer
    return CAN_Send(&can, &txBuffer[can_idx][buf_idx]);
}

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
/**
 * @brief 对应大疆电机不同控制ID的CAN数据发送缓存区
 */
CAN_TxBuffer_t txBuffer0x200forCAN1 =   
    {
    .txHeader.Identifier = 0x200,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x1FFforCAN1 = 
    {
    .txHeader.Identifier = 0x1FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x2FFforCAN1 =   
    {
    .txHeader.Identifier = 0x2FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x200forCAN2 = 
    {
    .txHeader.Identifier = 0x200,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x1FFforCAN2 =   
    {
    .txHeader.Identifier = 0x1FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x2FFforCAN2 = 
    {
    .txHeader.Identifier = 0x2FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
	
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
 * @brief  电机输出限幅。
 */
static void MotorOutputLimit(Motor_t *motor)
{
    if ((&motor->treatedData)->motor_output > motor->param.current_limit)
        (&motor->treatedData)->motor_output = motor->param.current_limit;
    else if ((&motor->treatedData)->motor_output < (-motor->param.current_limit))
        (&motor->treatedData)->motor_output = (-motor->param.current_limit);
}

/**
 * @brief  针对C610和C620电调的控制ID，将待发送数据填入CAN发送缓存区的函数指针。
 */
static uint8_t CAN_fill_3508_2006_data(CAN_Instance_t can, TreatedData_t motorData, uint16_t id)
{
    if (can.canHandler == &hfdcan1) 
    {
        if (id >= 0x201 && id <= 0x204) 
        {
            txBuffer0x200forCAN1.data[(id - 0x201) * 2]     = motorData.motor_output >> 8;
            txBuffer0x200forCAN1.data[(id - 0x201) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x205 && id <= 0x208) 
        {
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        }
    } 
    else if (can.canHandler == &hfdcan2) 
    {
        if (id >= 0x201 && id <= 0x204) 
        {
            txBuffer0x200forCAN2.data[(id - 0x201) * 2]     = motorData.motor_output >> 8;
            txBuffer0x200forCAN2.data[(id - 0x201) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x205 && id <= 0x208) 
        {
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x209 && id <= 0x20B) 
        {
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2]     = motorData.motor_output >> 8;
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2 + 1] = motorData.motor_output & 0xff;
        }
    }
    return 0;
}

/**
 * @brief  针对GM6020电调的控制ID，将待发送数据填入CAN发送缓存区的函数指针。
 */
static uint8_t CAN_fill_6020_data(CAN_Instance_t can, TreatedData_t motorData, uint16_t id)
{
    if (can.canHandler == &hfdcan1) 
    {
        if (id >= 0x205 && id <= 0x208) {
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        } else if (id >= 0x209 && id <= 0x20B) {
            txBuffer0x2FFforCAN1.data[(id - 0x209) * 2]     = motorData.motor_output >> 8;
            txBuffer0x2FFforCAN1.data[(id - 0x209) * 2 + 1] = motorData.motor_output & 0xff;
        }
    } 
    else if (can.canHandler == &hfdcan2) 
    {
        if (id >= 0x205 && id <= 0x208) 
        {
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x209 && id <= 0x20B) 
        {
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2]     = motorData.motor_output >> 8;
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2 + 1] = motorData.motor_output & 0xff;
        }
    }
    return 0;
}

/**
 * @brief  电机数据更新回调函数，只在motor.c文件内调用。（大疆电机反馈报文格式相同）
 */
static uint8_t CAN_update_data(RawData_t *raw, TreatedData_t *treated, uint8_t *data)
{
	treated->last_ecd 	 = raw->raw_ecd;
    raw->raw_ecd         = data[0] << 8 | data[1];
    raw->speed_rpm       = data[2] << 8 | data[3];
    raw->torque_current  = data[4] << 8 | data[5];
    raw->temperature     = data[6];
	treated->fps         ++;
    return 0;
}

/**
 * @brief 注册电机设备到CAN设备链表上
 */
void CAN_RegisterMotor(CAN_Instance_t *canx, Motor_t *motor)
{
    list_add(&motor->list, (&canx->devicesList));
}

/**
 * @brief 将电机结构体从CAN设备表上删除
 */
void CAN_DeleteMotor(Motor_t *motor)
{
    list_del(&(motor->list)); //< 判断无误后从链表中删除该设备
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
    (&motor->param)->ecd_offset      = ecdOffset;
    (&motor->param)->motor_type      = type;
    (&motor->param)->can_id          = id;
    (&motor->param)->reduction_ratio = gearRatio;
    (&motor->param)->can_number      = canx;

    if (canx == CAN1)
        CAN_RegisterMotor(&can1, motor);
    else if (canx == CAN2)
        CAN_RegisterMotor(&can2, motor);

    switch (type) 
    {
        case Motor3508: 
        {
            (&motor->param)->current_limit = CURRENT_LIMIT_FOR_3508;
            (&motor->param)->ecd_range     = ECD_RANGE_FOR_3508;
            motor->MotorUpdate             = CAN_update_data;
            motor->FillMotorData           = CAN_fill_3508_2006_data;
            break;
        }
        case Motor6020: 
        {
            (&motor->param)->current_limit = VOLTAGE_LIMIT_FOR_6020;
            (&motor->param)->ecd_range     = ECD_RANGE_FOR_6020;
            motor->MotorUpdate             = CAN_update_data;
            motor->FillMotorData           = CAN_fill_6020_data;
            break;
        }
        case Motor2006: 
        {
            (&motor->param)->current_limit = CURRENT_LIMIT_FOR_2006;
            (&motor->param)->ecd_range     = ECD_RANGE_FOR_2006;
            motor->MotorUpdate             = CAN_update_data;
            motor->FillMotorData           = CAN_fill_3508_2006_data;
            break;
        }
        default:;
    }
}

/**
 * @brief  根据canID在设备链表中寻找对应的电机
 */
static Motor_t *MotorFind(uint16_t canid, CAN_Instance_t canx)
{
    Motor_t *motor = NULL;
    list_t *node   = NULL;

    for (node = canx.devicesList.next; node != (canx.devicesList.prev->next); node = node->next) //< 对循环链表遍历一圈
    {
        motor = list_entry(node, Motor_t, list); //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
        if (motor->param.can_id == canid) {
            motor->online_cnt = 0; //< 电机在线，计数清零
            return motor;
        }
    }
    return NULL;
}

/**
 * @brief 电机CAN接收回调函数
 * @param canObject CAN实例对象指针，包含接收到的CAN数据
 * @note 该函数用于处理电机相关的CAN接收数据，解析电机反馈信息并更新电机状态
 */
void MotorRxCallback(CAN_Instance_t *canObject, CAN_RxBuffer_t *bufferRx)
{
    /* 从CAN接收数据中提取电机ID，并查找对应的电机对象 */
    uint32_t id;
    Motor_t *temp_motor = NULL;

    id         = bufferRx->rxHeader.Identifier;
    temp_motor = MotorFind(id, *canObject);
    if (temp_motor != NULL) 
    {
        /* 更新电机原始数据和处理后数据，并转换电机编码器值为角度值 */
        temp_motor->MotorUpdate(&temp_motor->rawData, &temp_motor->treatedData, bufferRx->data);
        MotorEcdtoAngle(temp_motor);
    }
}

/**
 * @brief 获得电机结构体中的ID
 */
uint16_t MotorReturnID(Motor_t motor)
{
    return motor.param.can_id;
}

/**
 * @brief  将treatedData.motor_output限幅后填入发送缓存区等待发送。
 */
void MotorFillData(Motor_t *motor, int32_t output)
{
    motor->treatedData.motor_output = output;
    MotorOutputLimit(motor);
    if (motor->param.can_number == CAN1)
        motor->FillMotorData(can1, motor->treatedData, motor->param.can_id);
    else if (motor->param.can_number == CAN2)
        motor->FillMotorData(can2, motor->treatedData, motor->param.can_id);
}

/**
 * @brief  将特定ID的CAN_TxBuffer_t发送出去。
 */
uint16_t MotorCanOutput(CAN_Instance_t can, int16_t IDforTxBuffer)
{
    switch (IDforTxBuffer) 
    {
        case 0x200: 
        {
            if (can.canHandler == &hfdcan1)
                CAN_Send(&can, &txBuffer0x200forCAN1);
            else if (can.canHandler == &hfdcan2)
                CAN_Send(&can, &txBuffer0x200forCAN2);
            break;
        }
        case 0x1ff: 
        {
            if (can.canHandler == &hfdcan1)
                CAN_Send(&can, &txBuffer0x1FFforCAN1);
            else if (can.canHandler == &hfdcan2)
                CAN_Send(&can, &txBuffer0x1FFforCAN2);
            break;
        }
        case 0x2ff: 
        {
            if (can.canHandler == &hfdcan1)
                CAN_Send(&can, &txBuffer0x2FFforCAN1);
            else if (can.canHandler == &hfdcan2)
                CAN_Send(&can, &txBuffer0x2FFforCAN2);
            break;
        }
        default:;
    }
	
    return 0;
}

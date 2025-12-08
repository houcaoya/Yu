#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32h7xx_hal.h"
#include "driver_can.h"

#define K_ECD_TO_ANGLE         0.043945f //< 角度转换编码器刻度的系数：360/8192
#define ECD_RANGE_FOR_3508     8191      //< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_3508 16000     //< 控制电流范围为正负16384
#define ECD_RANGE_FOR_6020     8191      //< 编码器刻度值为0-8191
#define VOLTAGE_LIMIT_FOR_6020 29000     //< 控制电压范围为正负30000
#define ECD_RANGE_FOR_2006     8191      //< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_2006 9900      //< 控制电流范围为正负10000
/** 
 * @brief  定义电机种类，用于发送函数的选择
 * @note   GM6020的反馈报文ID为 0x205-0x20B 之间
 * @note   M3508的反馈报文ID为 0x201-0x208 之间
 */
typedef enum {
    Motor3508 = 0x00U,
    Motor6020 = 0x01U,
    Motor2006 = 0x02U
} motor_type;

/**
 * @brief 电机返回的初始数据，由CAN中断回调更新
 */
typedef struct
{
    int16_t speed_rpm;      //< 每分钟所转圈数
    int16_t torque_current; //< 实际转矩电流
    uint8_t temperature;    //< 温度
    int16_t raw_ecd;        //< 原始编码器数据
} RawData_t;

/**
 * @brief  处理后的电机动态数据，电机运行中产生的数据，由CAN中断回调更新
 */
typedef struct
{
    int32_t ecd;              //< 当前编码器返回值处理值
    int32_t last_ecd;         //< 上一时刻编码器返回值处理值
	int32_t treated_ecd;
    float angle;              //< 解算后的编码器角度
	float last_angle;		  //< 上一时解算后的编码器角
	int32_t angle_demarcate;
    int16_t angle_speed;      //< 解算后的编码器角速度
    int32_t round_cnt;        //< 累计转动圈数
	int16_t axis_round_cnt;   //< 累计输出轴转动圈数
    int32_t total_ecd;        //< 编码器累计增量值
    float total_angle;        //< 累计旋转角度
	float axis_total_angle;   //< 累计输出轴旋转角度
    int32_t motor_output;     //< 输出给电机的值，通常为控制电流或电压
    int16_t filter_speed_rpm; //< 滑动平均滤波器后的转速
	uint16_t fps;
} TreatedData_t;

/**
 * @brief   电机参数，在初始化函数中确定
 */
typedef struct
{
    uint8_t can_number;       //< 电机所使用的CAN端口号
    uint16_t can_id;          //< 电机ID
    uint8_t motor_type;       //< 电机类型
    uint16_t ecd_offset;      //< 电机初始零点
    uint16_t ecd_range;       //< 编码器分度值
    uint16_t reduction_ratio; //< 减速比（电机转子转多少圈输出轴输出一圈）
    int16_t current_limit;    //< 电调能承受的最大电流
} MotorParam_t;

/**
 * @brief     将电机的待发送数据填入CAN发送缓存区
 * @param[in] motorData     电机动态数据结构体，只需要发送，不修改数据，不需要传入指针、
 * @param[in] id            电机标识符ID
 */
typedef uint8_t (*CAN_FillMotorData)(CAN_Instance_t can, TreatedData_t motorData, uint16_t id); //< 电机发送回调

/**
 * @brief  筛选CAN数据，并更新电机动态数据的回调函数
 */
typedef uint8_t (*Motor_DataUpdate)(RawData_t *raw, TreatedData_t *treated, uint8_t *data); //< 电机发送回调

/**
 * @brief  电机的数据和参数，以及两个不同电机之间略有区别的成员函数
 */
typedef struct
{
    int16_t online_cnt; //< 电机离线检测计数
    list_t list;                     //< 链表指针，创建指向自己的初始链表，并通过注册函数拓展成循环链表
    RawData_t rawData;               //< 电机初始动态数据，工作中更新
    TreatedData_t treatedData;       //< 电机处理后的数据，工作中更新
    MotorParam_t param;              //< 电机参数，在初始化时设置
    Motor_DataUpdate MotorUpdate;    //< 更新电机运行数据的函数指针
    CAN_FillMotorData FillMotorData; //< 对不同发送ID的CAN发送缓存区填入待发送数据的函数指针
} Motor_t;


void MotorInit(Motor_t *motor, uint16_t ecdOffset, motor_type type, uint16_t gearRatio, CanNumber canx, uint16_t id);
void MotorRxCallback(CAN_Instance_t *canObject, CAN_RxBuffer_t *bufferRx);
uint16_t MotorReturnID(Motor_t motor);
void MotorFillData(Motor_t *motor, int32_t output);
uint16_t MotorCanOutput(CAN_Instance_t can, int16_t IDforTxBuffer);


#endif

#ifndef _SHOOTTASK_H_
#define _SHOOTTASK_H_

#include "stm32h7xx_hal.h"
#include "rm_motor.h"
#include "pid.h"

#define SHOOT_ENABLE 1

/**
 * @brief 摩擦轮电机对象
 *
 */
typedef struct
{
    Motor_t m3508;                // 电机的参数和数据
    int16_t target_speed_config;  // 最终目标转速
    int16_t target_speed_current; // 当前目标转速
    SinglePID_t fricSpeedPID;     // 摩擦轮速度控制PID参数
} FricInstance_t;
/**
 * @brief 打弹数据
 *
 */
typedef struct
{
    // 标志位
    struct
    {
		uint8_t fric_ready;				  // 摩擦轮准备完毕
		uint8_t shoot_ready;			  // 微动开关标志位
        uint8_t fire;					  // 发弹指令
        uint8_t jam;                      // 链路卡弹
		uint8_t load_start;               // 拨弹盘启动
		uint8_t fric_close;               // 摩擦轮停止
		uint8_t hanging_shot;			  // 吊射模式
		uint8_t auto_shoot;				  // 自动击打模式
		uint8_t recorde;
		uint8_t forbid;
    } shootFlag;    
    // 计数
    struct 
	{
        uint16_t shoot_count; // 打弹计数，打一次加一，初始化或切换射击模式后清零
        uint16_t shoot_count_last;
        struct
        {
            float slowopen_time;  // 摩擦轮缓启动时间
            float slowclose_time; // 摩擦轮缓关闭时间
        } fric;
        struct
        {
            uint16_t jammed_time;  		 // 拨弹盘卡弹判断计时
            uint16_t jammed_judge_time;  		 // 拨弹盘卡弹判断计时
            int16_t  load_turnback_time; // 拨弹盘反转时间
            uint16_t load_time;          //
			uint16_t auto_time;			 // 自动发射间隔时间
			uint16_t interval_time; 	 // 发弹延迟
        } loader;
    } shootCount;
    struct
    {
        FricInstance_t  top;
        FricInstance_t  left;
        FricInstance_t  right;
		uint16_t        speed_top;
		uint16_t        speed_left;
		uint16_t        speed_right;
    } booster;
    struct
    {
		float       angle;
		float       last_angle;
		double      total_angle;
		double      axis_angle;
		double      axis_total_angle;
		double      target_angle;
		float       unit_target_angle;
		float       recorded_angle;
		Motor_t     m3508;                 // 电机的参数和数据
        int16_t     backward_speed;       // 大拨弹盘反转目标速度
		int16_t     forward_speed;
		DualPID_t   loadPID;
        SinglePID_t LoadForwardPID; 
        SinglePID_t LoadBackwardPID; 
        SinglePID_t LoadStopPID; 
    } loader;
} Shoot_t;

extern Shoot_t heroShoot;

void Shoot_Task(void *argument);
void Chassis_Task(void *argument);
void Holder_Task(void *argument);
void Print_Task(void *argument);
void ShootInit(Shoot_t *shoot);
#endif


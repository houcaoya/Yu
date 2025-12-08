#include "pid.h"
#include "user_lib.h"

/**
 * @brief 单环PID初始化
 *
 * @param single_pid 被赋值的结构体地址
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param pMaxlimit 比例部分控制量输出限幅
 * @param iMaxlimit 积分部分控制量输出限幅
 * @param dMaxlimit 微分部分控制量输出限幅
 * @param iPartDetach 积分部分死区
 * @param OutputLimit 总输出限幅
 */
void BasePID_Init(SinglePID_t *single_pid,
                         float kp, float ki, float kd,
                         float pMaxlimit, float iMaxlimit, float dMaxlimit,
                         float iPartDetach_lower, float iPartDetach_upper,
						 float OutputLimit)
{
    single_pid->P            		  = kp;
    single_pid->I            		  = ki;
    single_pid->D             		  = kd;
    single_pid->p_part_maxlimit 	  = pMaxlimit;
    single_pid->i_part_maxlimit 	  = iMaxlimit;
    single_pid->i_part_detach_upper   = iPartDetach_upper;
	single_pid->i_part_detach_lower   = iPartDetach_lower;
    single_pid->d_part_maxlimit 	  = dMaxlimit;
    single_pid->max_limit      		  = OutputLimit;
}
/**
 * @brief 双环PID初始化
 *
 * @param dual_pid 被赋值的结构体地址
 * @param shell_single_pid 赋外环PID值的结构体地址
 * @param core_single_pid 赋内环PID值的结构体地址
 */
void DualPID_Init(DualPID_t *dual_pid, SinglePID_t *shell_single_pid, SinglePID_t *core_single_pid)
{
    // 外环赋值
    dual_pid->shell.shell_P               = shell_single_pid->P;
    dual_pid->shell.shell_I               = shell_single_pid->I;
    dual_pid->shell.shell_D               = shell_single_pid->D;
    dual_pid->shell.shell_p_part_maxlimit = shell_single_pid->p_part_maxlimit;
    dual_pid->shell.shell_i_part_maxlimit = shell_single_pid->i_part_maxlimit;
    dual_pid->shell.shell_i_part_detach_upper   = shell_single_pid->i_part_detach_upper;
	dual_pid->shell.shell_i_part_detach_lower   = shell_single_pid->i_part_detach_lower;
    dual_pid->shell.shell_d_part_maxlimit = shell_single_pid->d_part_maxlimit;
    dual_pid->shell.shell_max_limit       = shell_single_pid->max_limit;
    // 内环赋值
    dual_pid->core.core_P               = core_single_pid->P;
    dual_pid->core.core_I               = core_single_pid->I;
    dual_pid->core.core_D               = core_single_pid->D;
    dual_pid->core.core_p_part_maxlimit = core_single_pid->p_part_maxlimit;
    dual_pid->core.core_i_part_maxlimit = core_single_pid->i_part_maxlimit;
    dual_pid->core.core_i_part_detach_upper   = core_single_pid->i_part_detach_upper;
	dual_pid->core.core_i_part_detach_lower   = core_single_pid->i_part_detach_lower;
    dual_pid->core.core_d_part_maxlimit = core_single_pid->d_part_maxlimit;
    dual_pid->core.core_max_limit       = core_single_pid->max_limit;
}
/**
 * @brief 单环PID计算
 *
 * @param target 目标值
 * @param feedback 反馈值
 * @param PID 单环PID结构体地址
 * @return float
 */
float One_Pid_Ctrl(float target, float feedback, SinglePID_t *PID)
{
    PID->delta = target - feedback;
    /************ P操作 ************/
    PID->p_part = PID->delta * PID->P;
    PID->p_part = LIMIT((PID->p_part), -(PID->p_part_maxlimit), (PID->p_part_maxlimit));
    /************ I操作 ************/
    PID->i_delta_sum += PID->delta;
    PID->i_part = PID->i_delta_sum * PID->I;
    if (ABS(PID->delta) > PID->i_part_detach_upper){
        PID->i_delta_sum = 0;
    }
	if(ABS(PID->delta) < PID->i_part_detach_lower){
		PID->i_delta_sum = 0;
    }
    PID->i_part = LIMIT((PID->i_part), -(PID->i_part_maxlimit), (PID->i_part_maxlimit));
    /************ D操作 ************/
    PID->d_part     = (PID->delta - PID->delta_last) * PID->D;
    PID->d_part     = LIMIT((PID->d_part), -(PID->d_part_maxlimit), (PID->d_part_maxlimit));
    PID->delta_last = PID->delta;
    /************  输出 ************/
    PID->out = (PID->p_part + PID->i_part + PID->d_part);
    PID->out = LIMIT((PID->out), -(PID->max_limit), (PID->max_limit));
    return PID->out;
}
/**
 * @brief 双环PID计算
 *
 * @param shell_target 目标值
 * @param shell_feedback 外环反馈值
 * @param core_feedback 内环反馈值
 * @param PID 双环PID结构体地址
 * @return float
 */
float Double_Pid_Ctrl(float shell_target, float shell_feedback, float core_feedback, DualPID_t *PID)
{
    PID->shell.shell_delta = shell_target - shell_feedback;
    /************ 外环P操作 ************/
    PID->shell.shell_p_part = PID->shell.shell_delta * PID->shell.shell_P;
    PID->shell.shell_p_part = LIMIT((PID->shell.shell_p_part), -(PID->shell.shell_p_part_maxlimit), (PID->shell.shell_p_part_maxlimit));
    /************ 外环I操作 ************/
    PID->shell.shell_i_part += PID->shell.shell_delta * PID->shell.shell_I;
    if (ABS(PID->shell.shell_delta) > PID->shell.shell_i_part_detach_upper){
        PID->shell.shell_i_part = 0;
    }
	if (ABS(PID->shell.shell_delta) < PID->shell.shell_i_part_detach_lower){
        PID->shell.shell_i_part = 0;
    }
    PID->shell.shell_i_part = LIMIT((PID->shell.shell_i_part), -(PID->shell.shell_i_part_maxlimit), (PID->shell.shell_i_part_maxlimit));
    /************ 外环D操作 ************/
    PID->shell.shell_d_part     = (PID->shell.shell_delta - PID->shell.shell_delta_last) * PID->shell.shell_D;
    PID->shell.shell_d_part     = LIMIT((PID->shell.shell_d_part), -(PID->shell.shell_d_part_maxlimit), (PID->shell.shell_d_part_maxlimit));
    PID->shell.shell_delta_last = PID->shell.shell_delta;
    /************ 外环输出 ************/
    PID->shell.shell_out = (PID->shell.shell_p_part + PID->shell.shell_i_part + PID->shell.shell_d_part);
    PID->shell.shell_out = LIMIT((PID->shell.shell_out), -(PID->shell.shell_max_limit), (PID->shell.shell_max_limit));
    PID->core.core_delta = PID->shell.shell_out - core_feedback;
    /************ 内环P操作 ************/
    PID->core.core_p_part = PID->core.core_delta * PID->core.core_P;
    PID->core.core_p_part = LIMIT((PID->core.core_p_part), -(PID->core.core_p_part_maxlimit), (PID->core.core_p_part_maxlimit));
    /************ 内环I操作 ************/
    PID->core.core_i_part += PID->core.core_delta * PID->core.core_I;
    if (ABS(PID->core.core_delta) > PID->core.core_i_part_detach_upper){
        PID->core.core_i_part = 0;
    }
	if (ABS(PID->core.core_delta) < PID->core.core_i_part_detach_lower){
        PID->core.core_i_part = 0;
    }
    PID->core.core_i_part = LIMIT((PID->core.core_i_part), -(PID->core.core_i_part_maxlimit), (PID->core.core_i_part_maxlimit));
    /************ 内环D操作 ************/
    PID->core.core_d_part     = (PID->core.core_delta - PID->core.core_delta_last) * PID->core.core_D;
    PID->core.core_d_part     = LIMIT((PID->core.core_d_part), -(PID->core.core_d_part_maxlimit), (PID->core.core_d_part_maxlimit));
    PID->core.core_delta_last = PID->core.core_delta;
    /************ 内环输出 ************/
    PID->core.core_out = (PID->core.core_p_part + PID->core.core_i_part + PID->core.core_d_part);
    PID->core.core_out = LIMIT((PID->core.core_out), -(PID->core.core_max_limit), (PID->core.core_max_limit));
    return PID->core.core_out;
}

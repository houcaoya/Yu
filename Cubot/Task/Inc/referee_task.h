#ifndef _REFEREETASK_H_
#define _REFEREETASK_H_

#include "stm32h7xx_hal.h"
#include "driver_usart.h"

#define OPEN_REFEREE 1

#define data_addr 7
#define max_single_pack_len 50
#define packs 15
#define frame_header_len 5
#define pack_len 7 + referee2024.frame_info.head.data_len + 2
#define BSP_USART3_DMA_RX_BUF_LEN 256

#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

typedef struct
{
	int16_t online_cnt;
	struct
	{
		struct
		{
			uint8_t sof;
			uint16_t data_len;
			uint8_t seq;
			uint8_t crc8;
		} head;
		uint16_t cmd_id;
		uint8_t frame_tail[2];
	} frame_info;

	/*1. 比赛状态数据： 0x0001。 发送频率： 1Hz*/
	struct
	{
		uint8_t game_type : 4;
		uint8_t game_progress : 4;
		uint16_t stage_remain_time;
	} game_status;

	/*2. 比赛结果数据： 0x0002。 发送频率：比赛结束后发送*/
	struct
	{
		uint8_t winner;
	} game_result;
	/*3. 机器人血量数据： 0x0003。发送频率： 1Hz*/
	struct
	{
		uint16_t red_1_robot_HP;
		uint16_t red_2_robot_HP;
		uint16_t red_3_robot_HP;
		uint16_t red_4_robot_HP;
		uint16_t red_7_robot_HP;
		uint16_t red_outpost_HP;
		uint16_t red_base_HP;
		uint16_t blue_1_robot_HP;
		uint16_t blue_2_robot_HP;
		uint16_t blue_3_robot_HP;
		uint16_t blue_4_robot_HP;
		uint16_t blue_7_robot_HP;
		uint16_t blue_outpost_HP;
		uint16_t blue_base_HP;
	}game_robot_HP;
	/*4. 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人*/
	struct
	{
		uint8_t dart_belong;		   // 发射飞镖队伍 1：红方飞镖 2：蓝方飞镖
		uint16_t stage_remaining_time; // 发射时的剩余比赛时间 单位 s
	} dart_state;

	/*5. 人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人。*/
	struct
	{
		uint8_t F1_zone_status : 1;
		uint8_t F1_zone_buff_debuff_status : 3;
		uint8_t F2_zone_status : 1;
		uint8_t F2_zone_buff_debuff_status : 3;
		uint8_t F3_zone_status : 1;
		uint8_t F3_zone_buff_debuff_status : 3;
		uint8_t F4_zone_status : 1;
		uint8_t F4_zone_buff_debuff_status : 3;
		uint8_t F5_zone_status : 1;
		uint8_t F5_zone_buff_debuff_status : 3;
		uint8_t F6_zone_status : 1;
		uint8_t F6_zone_buff_debuff_status : 3;
	} ext_ICRA_buff_debuff_zone_status;

	/*6.场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。*/
	struct
	{
		uint32_t event_data;
	} event_data;

	/*7. 补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。
	 */
	struct
	{
		uint8_t supply_projectile_id;
		uint8_t supply_robot_id;
		uint8_t supply_projectile_step;
		uint8_t supply_projectile_num;
	} supply_projectile_action;

	/*8. 裁判警告信息： cmd_id (0x0104)。发送频率：警告发生后发送*/
	struct
	{
		uint8_t level;
		uint8_t offending_robot_id;
		uint16_t dart_info;
	} referee_warning;

	/*9. 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。*/
	struct
	{
		uint8_t dart_remaining_time;
	} dart_remaining_time;

	/*10. 比赛机器人状态： 0x0201。 发送频率： 10Hz，发送范围：单一机器人。*/
	struct
	{
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t current_HP;
		uint16_t maximum_HP;
		uint16_t shooter_barrel_cooling_value;
		uint16_t shooter_barrel_heat_limit;
		uint16_t chassis_power_limit;
		uint8_t power_management_gimbal_output;
		uint8_t power_management_chassis_output; 
		uint8_t power_management_shooter_output; 
	} game_robot_status;

	/*11. 实时功率热量数据： 0x0202。 发送频率： 50Hz，发送范围：单一机器人。
	 */
	struct
	{
		uint16_t buffer_energy;
		uint16_t shooter_17mm_1_barrel_heat;
		uint16_t shooter_17mm_2_barrel_heat;
		uint16_t shooter_42mm_barrel_heat;
	} power_heat_data;

	/*12. 机器人位置： 0x0203。 发送频率： 10Hz，发送范围：单一机器人。
	 */
	struct
	{
		float x;
		float y;
		float angle;
	} game_robot_pos;

	/*13. 机器人增益： 0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。
	 */
	struct
	{
		uint8_t recovery_buff;
		uint8_t cooling_buff;
		uint8_t defence_buff;
		uint8_t vulnerability_buff;
		uint16_t attack_buff;
		uint8_t remaining_energy;
	} buff;

	/*14. 空中机器人能量状态： 0x0205。 发送频率： 10Hz
	 */
	struct
	{
		uint16_t energy_point;
		uint8_t attack_time;
	} ext_aerial_robot_energy_t;

	/*15. 伤害状态： 0x0206。 发送频率：伤害发生后发送
	 */
	struct
	{
		uint8_t armor_id : 4;
		uint8_t HP_deduction_reason : 4;
	} robot_hurt;

	/*16. 实时射击信息： 0x0207。 发送频率：射击后发送*/
	struct
	{
		uint8_t bullet_type;
		uint8_t shooter_number;
		uint8_t launching_frequency;
		float initial_speed;
	} shoot_data;

	// 17. 子弹剩余发射数： 0x0208。发送频率： 1Hz 周期发送，
	struct
	{
		uint16_t projectile_allowance_17mm;
		uint16_t projectile_allowance_42mm;
		uint16_t remaining_gold_coin;
	} bullet_remaining;

	// 18.机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。

	struct
	{
		uint32_t rfid_status;
	} rfid_status;

	// 19. 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
	struct
	{
		uint8_t dart_launch_opening_status;
		uint16_t target_change_time;
		uint16_t latest_launch_cmd_time;
	} dart_client_cmd;

	// 20. 机器人位置数据：0x020B。发送频率：10Hz，发送范围：单一机器人。
	 struct
	{	
		float hero_x;
		float hero_y;
		float engineer_x;
		float engineer_y;
		float standard_3_x;
		float standard_3_y;
		float standard_4_x;
		float standard_4_y;
	}ground_robot_position;

	
	//21.机器人易伤情况:0x020C
	struct
	{
		uint8_t mark_progress;
	}radar_mark_data;
	
	
	// 22. 哨兵机机器人客户端指令数据：0x020D。发送频率：10Hz，发送范围：单一机器人。
	struct
	{
		uint32_t sentry_info;
		uint16_t sentry_info_2;
	} sentry_info;
	//23.双倍易伤数据：0x020E
	struct
	{
	uint8_t radar_info;
	} radar_info;
	// 机器人间交互数据
	
	// 1. 交互数据接收信息：0x0301。
	struct
	{
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
	} robot_interactive_data;

	struct // 0x0301 客户端内容id 0xd180
	{
		uint8_t data[30];
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
	} ext_student_interactive_header_data;
	uint16_t sentryHP;
	char UI_sentryHP_string[30];
} Referee_t;
#if (OPEN_REFEREE == 1)
void _Data_Diapcak(uint8_t *pdata);
#endif
void Referee_Task(void *argument);
extern Referee_t referee2024;
extern UART_RxBuffer_t uart3_buffer;
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
#endif

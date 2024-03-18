#ifndef _QD_REFEREE_SYSTEM_HPP_
#define _QD_REFEREE_SYSTEM_HPP_
#include <iostream>
#include <stdbool.h>
#include <stdint.h>

typedef char Status; 
typedef uint8_t QElemType; 

/*------------------串口协议各个包详细说明------------------*/

/**
  * @brief  比赛状态数据
  * cmd:0x0001
  */
typedef struct
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
}game_status_t;

/**
  * @brief  机器人血量数据
  * cmd:0x0003
  */
typedef struct
{
	uint16_t red_7_robot_HP;    // 红 7 哨兵机器人血量 
	uint16_t red_outpost_HP;    // 红方前哨站血量
	uint16_t red_base_HP; 	    // 红方基地血量 
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP; 
	uint16_t blue_base_HP;

  uint16_t own_robot_HP;      // 己方哨兵机器人血量 
	uint16_t own_outpost_HP;    // 己方前哨站血量
	uint16_t own_base_HP; 	    // 己方基地血量 
	uint16_t enemy_outpost_HP;  // 敌方
	uint16_t enemy_base_HP;
} game_robot_HP_t;

/**
  * @brief  场地事件数据
  * cmd:0x0101
  */
typedef struct
{
    uint32_t event_data;
}event_data_t;

/**
  * @brief 机器人性能体系数据
  * cmd:0x0201
  */
typedef struct 
{   
	// uint8_t robot_id;
    // uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    // uint16_t shooter_barrel_cooling_value;
    // uint16_t shooter_barrel_heat_limit;
    // uint16_t chassis_power_limit;
} ext_supply_projectile_action_t;

/**
  * @brief  机器人位置
  * cmd:0x0203
  */
typedef struct 
{ 
	float x;    //位置 x 坐标，单位 m
	float y;    //位置 y 坐标，单位 m 
  float angle;
} robot_pos_t;

/**
  * @brief  机器人增益
  * cmd:0x0204
  */
typedef struct
{
    // uint8_t recovery_buff;      // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    // uint8_t cooling_buff;       // 机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
    uint8_t defence_buff;       // 机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff; // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff;       // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
}buff_t;

/**
  * @brief  伤害状态
  * cmd:0x0206
  */
typedef struct
{
	//bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。
	uint8_t armor_id : 4;
	//bit 4-7：血量变化类型
	//0x0 装甲伤害扣血；
	//0x1 模块掉线扣血；
	//0x2 超射速扣血；
	//0x3 超枪口热量扣血；
	//0x4 超底盘功率扣血；
	//0x5 装甲撞击扣血
	// uint8_t hurt_type : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/**
  * @brief 子弹剩余发射数
  * cmd:0x0208
  */
typedef struct
{
	uint16_t projectile_allowance_17mm; // 17mm 弹丸允许发弹量
	// uint16_t remaining_gold_coin;       // 剩余金币数量
}projectile_allowance_t; 

/**
  * @brief 机器人 RFID 状态
  * cmd:0x0209
  */
typedef struct 
{ 
	//bit 0：基地增益点 RFID 状态；
	//bit 1：高地增益点 RFID 状态；
	//bit 2：能量机关激活点 RFID 状态；
	//bit 3：飞坡增益点 RFID 状态；
	//bit 4：前哨岗增益点 RFID 状态；
	//bit 6：补血点增益点 RFID 状态；
	//bit 7：工程机器人复活卡 RFID 状态；
	//bit 8-31：保留
	uint32_t rfid_status; 
} rfid_status_t; 

/*
 * @brief 在下面实例化对象
*/
game_status_t game_status;
game_robot_HP_t game_robot_HP;
event_data_t event_data;
ext_supply_projectile_action_t ext_supply_projectile_action;
robot_pos_t robot_pos;
buff_t buff;
hurt_data_t hurt_data;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;

/*
 * @brief 串口协议对应命令ID
*/
typedef enum
{ 
  CMD_game_status                       = 0x0001,
	CMD_game_robot_HP_t                   = 0x0003,
	CMD_event_data_t                      = 0x0101,
	CMD_ext_supply_projectile_action_t    = 0x0201,
	CMD_robot_pos_t                       = 0x0203,
	CMD_buff_t                            = 0x0204,
	CMD_hurt_data_t                       = 0x0206,
	CMD_projectile_allowance_t            = 0x0208,
	CMD_rfid_status_t                     = 0x0209,
} CmdID;
























































































#endif
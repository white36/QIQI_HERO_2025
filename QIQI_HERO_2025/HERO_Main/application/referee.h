#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 11,
    BLUE_ENGINEER = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL = 16,
    BLUE_SENTRY = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;


typedef enum
{
    ARMOR_HURT = 0x00, // 装甲板伤害
} hurt_type_t; // 伤害类型

typedef struct __attribute__((packed)) // 0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t; // 比赛状态数据，固定以 1Hz 频率发送

typedef struct __attribute__((packed)) // 0002
{
    uint8_t winner;
} ext_game_result_t; // 比赛结果数据，比赛结束触发发送

typedef struct __attribute__((packed)) // 0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t reserved_1;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved_2;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t; // 机器人血量数据，固定以 3Hz 频率发送

typedef struct __attribute__((packed)) // 0x0101
{
    uint32_t event_data;
} ext_event_data_t; // 场地事件数据，固定以 1Hz 频率发送

typedef struct __attribute__((packed)) // 0x0102 (2025无)
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef struct __attribute__((packed)) // 0x0103 (2025无)
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef struct __attribute__((packed)) // 0x0104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t; // 裁判警告数据，己方判罚判负时触发发送，其余时间以 1Hz 频率发送

typedef struct __attribute__((packed)) // 0x105 (2025增)
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_info_t;

typedef struct __attribute__((packed)) // 0x0201
{
    uint8_t robot_id; // 本机器人 ID
    uint8_t robot_level; // 机器人等级
    uint16_t current_HP; // 机器人当前血量
    uint16_t maximum_HP; // 机器人血量上限
    uint16_t shooter_barrel_cooling_value; // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit; // 机器人射击热量上限
    uint16_t chassis_power_limit; // 机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1; // gimbal口输出
    uint8_t power_management_chassis_output : 1; // chassis 口输出
    uint8_t power_management_shooter_output : 1; // shooter 口输出
} ext_robot_state_t; // 机器人性能体系数据，固定以 10Hz 频率发送

typedef struct __attribute__((packed)) // 0x0202
{
    uint16_t reserved_1; // 保留位
    uint16_t reserved_2; // 保留位
    float reserved_3; // 保留位
    uint16_t buffer_energy; // 缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat; // 第 1 个 17mm 发射机构的射击热量
    uint16_t shooter_17mm_2_barrel_heat; //第 2 个 17mm 发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat; // 42mm 发射机构的射击热量
} ext_power_heat_data_t; // 实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送

typedef struct __attribute__((packed)) // 0x0203
{
    float x; // 本机器人位置 x 坐标，单位：m
    float y; // 本机器人位置 y 坐标，单位：m
    float angle; // 本机器人测速模块的朝向，单位：度。正北为0度
	
} ext_robot_pos_t; // 机器人位置数据，固定以 1Hz 频率发送

typedef struct __attribute__((packed)) // 0x0204
{
    uint8_t recovery_buff; // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint8_t cooling_buff; // 机器人射击热量冷却倍率（直接值，值为 5 表示 5 倍冷却）
    uint8_t defence_buff; // 机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff; // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff; // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
    uint8_t remaining_energy; // 机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例 !!不能返回精确值
} ext_buff_t; // 机器人增益和底盘能量数据，固定以 3Hz 频率发送

    typedef struct __attribute__((packed)) // 0x0205 (2025无)
{
    uint8_t airforce_status;
    uint8_t time_remain;
} aerial_robot_energy_t;

typedef struct __attribute__((packed)) // 0x0206
{
    uint8_t armor_id : 4; // bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号
    uint8_t HP_deduction_reason : 4; // bit 4-7：血量变化类型
	
} ext_hurt_data_t; // 伤害状态数据，伤害发生后发送(本地判定,实际以服务器为准)

typedef struct __attribute__((packed)) // 0x0207
{
    uint8_t bullet_type; // 弹丸类型
    uint8_t shooter_number; // 发射机构 ID
    uint8_t launching_frequency; // 弹丸射速（单位：Hz）
    float initial_speed; // 弹丸初速度（单位：m/s）
} ext_shoot_data_t; // 实时射击数据，弹丸发射后发送

typedef struct __attribute__((packed)) // 0x0208
{
    uint16_t projectile_allowance_17mm; // 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm; // 42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin; // 剩余金币数量
} ext_projectile_allowance_t; // 允许发弹量，固定以 10Hz 频率发送

typedef struct __attribute__((packed)) // 0x0209
{
    uint32_t rfid_status; // bit 位值对应是否已检测到该增益点 RFID 卡. bit 24-31：保留
} ext_rfid_status_t; // 机器人 RFID 模块状态，固定以 3Hz 频率发送


typedef struct __attribute__((packed)) // 0x0301
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t; // 机器人交互数据，发送方触发发送，频率上限为 30Hz

typedef struct __attribute__((packed))
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;

typedef struct __attribute__((packed))
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef struct __attribute__((packed))
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef struct __attribute__((packed))
{
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
} ext_robot_command_t;

extern ext_game_state_t game_state;       // 0001
extern ext_game_result_t game_result;     // 0002
extern ext_game_robot_HP_t game_robot_HP; // 0003

extern ext_event_data_t event_data;                               // 0x0101
extern ext_supply_projectile_action_t supply_projectile_action;   // 0x0102 25赛季无
extern ext_supply_projectile_booking_t supply_projectile_booking; // 0x0103 25赛季无
extern ext_referee_warning_t referee_warning;                     // 0x0104
extern ext_dart_info_t dart_info;                                 // 0x0105 (2025增)

extern ext_robot_state_t robot_state;                   // 0x0201
extern ext_power_heat_data_t power_heat_data;           // 0x0202
extern ext_robot_pos_t robot_pos;                       // 0x0203
extern ext_buff_t buff;                                 // 0x0204
extern aerial_robot_energy_t robot_energy;              // 0x0205 25赛季无
extern ext_hurt_data_t hurt_data;                       // 0x0206
extern ext_shoot_data_t shoot_data;                     // 0x0207
extern ext_projectile_allowance_t projectile_allowance; // 0x0208
extern ext_rfid_status_t rfid_status;                   // 0x0209

extern ext_student_interactive_data_t student_interactive_data_t; // 0x0301
extern ext_robot_command_t robot_command_t;                       // 0x0303

extern ext_rfid_status_t rfid_status_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

// 获取机器人id
extern uint8_t get_robot_id(void);

extern void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat);

// 获取机器人状态指针
ext_robot_state_t *get_robot_status_point(void);
// 获取实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送
ext_power_heat_data_t *get_power_heat_data_point(void);
// 获取伤害类型指针
ext_hurt_data_t *get_hurt_data_point(void);
// 获取发射机构弹速
ext_shoot_data_t *get_shoot_data_point(void);
// 获取机器人命令数据指针
ext_robot_command_t *get_robot_command_point(void);
// 获取场地状态指针
ext_event_data_t *get_field_event_point(void);
// 获取比赛机器血量指针
ext_game_robot_HP_t *get_game_robot_HP_point(void);
// 获取机器人位置指针
ext_robot_pos_t *get_robot_pos_point(void);

#endif

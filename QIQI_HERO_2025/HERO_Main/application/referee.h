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
    ARMOR_HURT = 0x00, // װ�װ��˺�
} hurt_type_t; // �˺�����

typedef struct __attribute__((packed)) // 0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_state_t; // ����״̬���ݣ��̶��� 1Hz Ƶ�ʷ���

typedef struct __attribute__((packed)) // 0002
{
    uint8_t winner;
} ext_game_result_t; // ����������ݣ�����������������

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
} ext_game_robot_HP_t; // ������Ѫ�����ݣ��̶��� 3Hz Ƶ�ʷ���

typedef struct __attribute__((packed)) // 0x0101
{
    uint32_t event_data;
} ext_event_data_t; // �����¼����ݣ��̶��� 1Hz Ƶ�ʷ���

typedef struct __attribute__((packed)) // 0x0102 (2025��)
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef struct __attribute__((packed)) // 0x0103 (2025��)
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
} ext_referee_warning_t; // ���о������ݣ������з��и�ʱ�������ͣ�����ʱ���� 1Hz Ƶ�ʷ���

typedef struct __attribute__((packed)) // 0x105 (2025��)
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_info_t;

typedef struct __attribute__((packed)) // 0x0201
{
    uint8_t robot_id; // �������� ID
    uint8_t robot_level; // �����˵ȼ�
    uint16_t current_HP; // �����˵�ǰѪ��
    uint16_t maximum_HP; // ������Ѫ������
    uint16_t shooter_barrel_cooling_value; // �������������ÿ����ȴֵ
    uint16_t shooter_barrel_heat_limit; // �����������������
    uint16_t chassis_power_limit; // �����˵��̹�������
    uint8_t power_management_gimbal_output : 1; // gimbal�����
    uint8_t power_management_chassis_output : 1; // chassis �����
    uint8_t power_management_shooter_output : 1; // shooter �����
} ext_robot_state_t; // ������������ϵ���ݣ��̶��� 10Hz Ƶ�ʷ���

typedef struct __attribute__((packed)) // 0x0202
{
    uint16_t reserved_1; // ����λ
    uint16_t reserved_2; // ����λ
    float reserved_3; // ����λ
    uint16_t buffer_energy; // ������������λ��J��
    uint16_t shooter_17mm_1_barrel_heat; // �� 1 �� 17mm ����������������
    uint16_t shooter_17mm_2_barrel_heat; //�� 2 �� 17mm ����������������
    uint16_t shooter_42mm_barrel_heat; // 42mm ����������������
} ext_power_heat_data_t; // ʵʱ���̻�������������������ݣ��̶���10HzƵ�ʷ���

typedef struct __attribute__((packed)) // 0x0203
{
    float x; // ��������λ�� x ���꣬��λ��m
    float y; // ��������λ�� y ���꣬��λ��m
    float angle; // �������˲���ģ��ĳ��򣬵�λ���ȡ�����Ϊ0��
	
} ext_robot_pos_t; // ������λ�����ݣ��̶��� 1Hz Ƶ�ʷ���

typedef struct __attribute__((packed)) // 0x0204
{
    uint8_t recovery_buff; // �����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��ʾÿ��ָ�Ѫ�����޵� 10%��
    uint8_t cooling_buff; // ���������������ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ʾ 5 ����ȴ��
    uint8_t defence_buff; // �����˷������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩
    uint8_t vulnerability_buff; // �����˸��������棨�ٷֱȣ�ֵΪ 30 ��ʾ-30%�������棩
    uint16_t attack_buff; // �����˹������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩
    uint8_t remaining_energy; // ������ʣ������ֵ�������� 16 ���Ʊ�ʶ������ʣ������ֵ���� !!���ܷ��ؾ�ȷֵ
} ext_buff_t; // ����������͵����������ݣ��̶��� 3Hz Ƶ�ʷ���

    typedef struct __attribute__((packed)) // 0x0205 (2025��)
{
    uint8_t airforce_status;
    uint8_t time_remain;
} aerial_robot_energy_t;

typedef struct __attribute__((packed)) // 0x0206
{
    uint8_t armor_id : 4; // bit 0-3������Ѫԭ��Ϊװ��ģ�鱻���蹥������ײ�������߻����ģ������ʱ���� 4 bit ��ɵ���ֵΪװ��ģ������ģ��� ID ���
    uint8_t HP_deduction_reason : 4; // bit 4-7��Ѫ���仯����
	
} ext_hurt_data_t; // �˺�״̬���ݣ��˺���������(�����ж�,ʵ���Է�����Ϊ׼)

typedef struct __attribute__((packed)) // 0x0207
{
    uint8_t bullet_type; // ��������
    uint8_t shooter_number; // ������� ID
    uint8_t launching_frequency; // �������٣���λ��Hz��
    float initial_speed; // ������ٶȣ���λ��m/s��
} ext_shoot_data_t; // ʵʱ������ݣ����跢�����

typedef struct __attribute__((packed)) // 0x0208
{
    uint16_t projectile_allowance_17mm; // 17mm ������������
    uint16_t projectile_allowance_42mm; // 42mm ������������
    uint16_t remaining_gold_coin; // ʣ��������
} ext_projectile_allowance_t; // �����������̶��� 10Hz Ƶ�ʷ���

typedef struct __attribute__((packed)) // 0x0209
{
    uint32_t rfid_status; // bit λֵ��Ӧ�Ƿ��Ѽ�⵽������� RFID ��. bit 24-31������
} ext_rfid_status_t; // ������ RFID ģ��״̬���̶��� 3Hz Ƶ�ʷ���


typedef struct __attribute__((packed)) // 0x0301
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t; // �����˽������ݣ����ͷ��������ͣ�Ƶ������Ϊ 30Hz

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
extern ext_supply_projectile_action_t supply_projectile_action;   // 0x0102 25������
extern ext_supply_projectile_booking_t supply_projectile_booking; // 0x0103 25������
extern ext_referee_warning_t referee_warning;                     // 0x0104
extern ext_dart_info_t dart_info;                                 // 0x0105 (2025��)

extern ext_robot_state_t robot_state;                   // 0x0201
extern ext_power_heat_data_t power_heat_data;           // 0x0202
extern ext_robot_pos_t robot_pos;                       // 0x0203
extern ext_buff_t buff;                                 // 0x0204
extern aerial_robot_energy_t robot_energy;              // 0x0205 25������
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

// ��ȡ������id
extern uint8_t get_robot_id(void);

extern void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat);

// ��ȡ������״ָ̬��
ext_robot_state_t *get_robot_status_point(void);
// ��ȡʵʱ���̻�������������������ݣ��̶���10HzƵ�ʷ���
ext_power_heat_data_t *get_power_heat_data_point(void);
// ��ȡ�˺�����ָ��
ext_hurt_data_t *get_hurt_data_point(void);
// ��ȡ�����������
ext_shoot_data_t *get_shoot_data_point(void);
// ��ȡ��������������ָ��
ext_robot_command_t *get_robot_command_point(void);
// ��ȡ����״ָ̬��
ext_event_data_t *get_field_event_point(void);
// ��ȡ��������Ѫ��ָ��
ext_game_robot_HP_t *get_game_robot_HP_point(void);
// ��ȡ������λ��ָ��
ext_robot_pos_t *get_robot_pos_point(void);

#endif

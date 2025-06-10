#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;       // 0001
ext_game_result_t game_result;     // 0002
ext_game_robot_HP_t game_robot_HP; // 0003

ext_event_data_t event_data;                                // 0x0101
ext_supply_projectile_action_t supply_projectile_action;   // 0x0102 25������
ext_supply_projectile_booking_t supply_projectile_booking; // 0x0103 25������
ext_referee_warning_t referee_warning;                     // 0x0104
ext_dart_info_t dart_info;                                   // 0x0105 (2025��)

ext_robot_state_t robot_state;                   // 0x0201
ext_power_heat_data_t power_heat_data;           // 0x0202
ext_robot_pos_t robot_pos;                       // 0x0203
ext_buff_t buff;                                 // 0x0204
aerial_robot_energy_t robot_energy;              // 0x0205 25������
ext_hurt_data_t hurt_data;                       // 0x0206
ext_shoot_data_t shoot_data;                     // 0x0207
ext_projectile_allowance_t projectile_allowance; // 0x0208
ext_rfid_status_t rfid_status;                   // 0x0209

ext_student_interactive_data_t student_interactive_data_t; // 0x0301
ext_robot_command_t robot_command_t;                       // 0x0303

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP, 0, sizeof(ext_game_robot_HP_t));

    memset(&event_data, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning, 0, sizeof(ext_referee_warning_t));
    memset(&dart_info, 0, sizeof(ext_dart_info_t));

    memset(&robot_state, 0, sizeof(ext_robot_state_t));
    memset(&power_heat_data, 0, sizeof(ext_power_heat_data_t));
    memset(&robot_pos, 0, sizeof(ext_robot_pos_t));
    memset(&buff, 0, sizeof(ext_buff_t));
    memset(&robot_energy, 0, sizeof(aerial_robot_energy_t));
    memset(&hurt_data, 0, sizeof(ext_hurt_data_t));
    memset(&shoot_data, 0, sizeof(ext_shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(ext_projectile_allowance_t));

    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
    memset(&robot_command_t, 0, sizeof(ext_robot_command_t));

}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(ext_game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;

        case EVENTS_DATA_CMD_ID:
        {
            memcpy(&event_data, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID: // 2025��
        {
            memcpy(&supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID: // 2025��
        {
            memcpy(&supply_projectile_booking, frame + index, sizeof(ext_supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
        break;
        case DART_INFO_CMD_ID:
        {
            memcpy(&dart_info, frame + index, sizeof(ext_dart_info_t));
        }
        break;


        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(ext_robot_state_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&robot_pos, frame + index, sizeof(ext_robot_pos_t));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy, frame + index, sizeof(aerial_robot_energy_t));
        }
        break;
        case HURT_DATA_CMD_ID:
        {
            memcpy(&hurt_data, frame + index, sizeof(ext_hurt_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(ext_shoot_data_t));
        }
        break;
        case PROJECTILE_ALLOWANCE_CMD_ID:
        {
            memcpy(&projectile_allowance, frame + index, sizeof(ext_projectile_allowance_t));
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            memcpy(&rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }


        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(ext_student_interactive_data_t));
        }
        break;
        case ROBOT_COMMAND_ID:
        {
            memcpy(&robot_command_t, frame + index, sizeof(ext_robot_command_t));
        }
        break;
        default:
        {
            break;
        }
    }
}
// 2025������ͨ������ϵͳ��ȡ���̹���
// void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
// {
//         *power = power_heat_data.chassis_power;
//         *buffer = power_heat_data.chassis_power_buffer;
// }

//��ȡ������id
uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

// ��ȡ 42mm ����������������
void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat)
{
    *heat = power_heat_data.shooter_42mm_barrel_heat;
}

//��ȡ������״̬
ext_robot_state_t* get_robot_status_point(void)
{
    return &robot_state;
}
// ��ȡʵʱ���̻�������������������ݣ��̶���10HzƵ�ʷ���
ext_power_heat_data_t* get_power_heat_data_point(void)
{
    return &power_heat_data;
}

    // ��ȡ�������˺�����ָ��
    ext_hurt_data_t *
    get_hurt_data_point(void)
{
    return &hurt_data;
}

//��ȡ�����˷������������Ϣ
ext_shoot_data_t* get_shoot_data_point(void)
{
    return &shoot_data;
}

//��ȡ��������������ָ��
ext_robot_command_t* get_robot_command_point(void)
{
    return &robot_command_t;
}

//��ȡ����״ָ̬��
ext_event_data_t* get_field_event_point(void)
{
    return &event_data;
}

//��ȡ��������Ѫ��ָ��
ext_game_robot_HP_t* get_game_robot_HP_point(void)
{
    return &game_robot_HP;
}

//��ȡ������λ��ָ��
ext_robot_pos_t* get_robot_pos_point(void)
{
    return &robot_pos;
}
ext_rfid_status_t* get_rfid_status(void)
{
	return &rfid_status;
}

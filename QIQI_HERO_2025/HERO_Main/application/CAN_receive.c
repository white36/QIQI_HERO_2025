/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "crc8_crc16.h"
#include "can_comm.h"

extern chassis_move_t chassis_move;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// motor data read
#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

motor_measure_t motor_chassis[12];
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef shoot_tx_message;
static uint8_t shoot_can_send_data[8];
static CAN_TxHeaderTypeDef ui_tx_message;
static uint8_t ui_can_send_data[8];
static CAN_TxHeaderTypeDef tui_tx_message;
static uint8_t tui_can_send_data[8];

static CAN_TxHeaderTypeDef cap_tx_message;
static uint8_t cap_can_send_data[8];

fp32 angle;
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
float voltent, current;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        case CAN_TRIGGER_MOTOR_ID:
        {
            get_motor_measure(&motor_chassis[11], rx_data);
            detect_hook(TRIGGER_MOTOR_TOE);
            break;
        }
        case CAN_PIT_MOTOR_ID:
        {
            get_motor_measure(&motor_chassis[5], rx_data);
            detect_hook(PITCH_GIMBAL_MOTOR_TOE);
            break;
        }
        case CAN_YAW_MOTOR_ID:
        {
            get_motor_measure(&motor_chassis[4], rx_data);
            detect_hook(YAW_GIMBAL_MOTOR_TOE);
            break;
        }
        case CAN_COMM_UP_ID:
        {
            UNPACK_CAN_STRUCT(rx_data, chassis_move.comm_rx_a);
            break;
        }
        default:
        {
            break;
        }
        }
    }
    else if (hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        case CAN_3508_LEFT_ID:
        {
            get_motor_measure(&motor_chassis[7], rx_data);
            detect_hook(FRIC_LEFT_MOTOR_TOE);
            break;
        }
        case CAN_3508_RIGHT_ID:
        {
            get_motor_measure(&motor_chassis[8], rx_data);
            detect_hook(FRIC_RIGHT_MOTOR_TOE);
            break;
        }
        case CAN_3508_BLEFT_ID:
        {
            get_motor_measure(&motor_chassis[9], rx_data);
            detect_hook(FRIC_BLEFT_MOTOR_TOE);
            break;
        }
        case CAN_3508_BRIGHT_ID:
        {
            get_motor_measure(&motor_chassis[10], rx_data);
            detect_hook(FRIC_BRIGHT_MOTOR_TOE);
            break;
        }
        default:
        {
            break;
        }
        }
    }
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      rev: (0x207) 保留，电机控制电流
 * @param[in]      rev: (0x208) 保留，电机控制电流
 * @retval         none
 */
void CAN_cmd_gimbal_shoot2(int16_t rev1, int16_t rev2, int16_t rev3, int16_t rev4)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_SHOOT2_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (rev1 >> 8);
    gimbal_can_send_data[1] = rev1;
    gimbal_can_send_data[2] = (rev2 >> 8);
    gimbal_can_send_data[3] = rev2;
    gimbal_can_send_data[4] = (rev3 >> 8);
    gimbal_can_send_data[5] = rev3;
    gimbal_can_send_data[6] = (rev4 >> 8);
    gimbal_can_send_data[7] = rev4;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID//
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流
 * @param[in]      s: (0x201) 2006电机控制电流, 范围 [-16384,16384]
 * @param[in]      left: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      right: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      trigger: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_shoot(int16_t left, int16_t right, int16_t bleft, int16_t bright)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = left >> 8;
    shoot_can_send_data[1] = left;
    shoot_can_send_data[2] = right >> 8;
    shoot_can_send_data[3] = right;
    shoot_can_send_data[4] = bleft >> 8;
    shoot_can_send_data[5] = bleft;
    shoot_can_send_data[6] = bright >> 8;
    shoot_can_send_data[7] = bright;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void CAN_cmd_cap(int16_t temPower) // 超级电容
{
    uint32_t send_mail_box;
    cap_tx_message.StdId = 0x210;
    cap_tx_message.IDE = CAN_ID_STD;
    cap_tx_message.RTR = CAN_RTR_DATA;
    cap_tx_message.DLC = 0x08;
    cap_can_send_data[0] = temPower >> 8;
    cap_can_send_data[1] = temPower;
    HAL_CAN_AddTxMessage(&CAP_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
}

void CAN_cmd_ui(int16_t key1, int16_t key2, int16_t key3, int16_t key4)
{
    uint32_t send_mail_box;
    ui_tx_message.StdId = 0x208;
    ui_tx_message.IDE = CAN_ID_STD;
    ui_tx_message.RTR = CAN_RTR_DATA;
    ui_tx_message.DLC = 0x08;
    ui_can_send_data[0] = key1 >> 8;
    ui_can_send_data[1] = key1;
    ui_can_send_data[2] = key2 >> 8;
    ui_can_send_data[3] = key2;
    ui_can_send_data[4] = key3 >> 8;
    ui_can_send_data[5] = key3;
    ui_can_send_data[6] = key4 >> 8;
    ui_can_send_data[7] = key4;

    HAL_CAN_AddTxMessage(&UI_CAN, &ui_tx_message, ui_can_send_data, &send_mail_box);
}

void CAN_cmd_ui2(int16_t Key1, int16_t Key2, int16_t Key3, int16_t Key4)
{
    uint32_t send_mail_box;
    tui_tx_message.StdId = 0x209;
    tui_tx_message.IDE = CAN_ID_STD;
    tui_tx_message.RTR = CAN_RTR_DATA;
    tui_tx_message.DLC = 0x08;
    tui_can_send_data[0] = Key1 >> 8;
    tui_can_send_data[1] = Key1;
    tui_can_send_data[2] = Key2 >> 8;
    tui_can_send_data[3] = Key2;
    tui_can_send_data[4] = Key3 >> 8;
    tui_can_send_data[5] = Key3;
    tui_can_send_data[6] = Key4 >> 8;
    tui_can_send_data[7] = Key4;

    HAL_CAN_AddTxMessage(&TUI_CAN, &tui_tx_message, tui_can_send_data, &send_mail_box);
}
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[11];
}

/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_can_3508_left_measure_point(void)
{
    return &motor_chassis[7];
}

/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_can_3508_right_measure_point(void)
{
    return &motor_chassis[8];
}
/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_can_3508_bleft_measure_point(void)
{
    return &motor_chassis[9];
}

/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_can_3508_bright_measure_point(void)
{
    return &motor_chassis[10];
}
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范? ?[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

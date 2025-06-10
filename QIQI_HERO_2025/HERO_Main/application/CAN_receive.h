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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define GIMBAL_CAN hcan1
#define SHOOT_CAN hcan2
// old ID
#define CHASSIS_CAN hcan1
#define POWER_CAN hcan2
#define CAP_CAN hcan2
#define UI_CAN hcan1
#define TUI_CAN hcan1

#define CAN_FEEDBACK_FREAM_ID_A 0x11
#define CAN_FEEDBACK_FREAM_ID_B 0x12
#define CAN_CTRL_FREAM_ID 0x21 // CAN帧ID号
/* CAN send and receive ID */
typedef enum
{
    // Can2
    CAN_SHOOT_ALL_ID = 0x200,
    CAN_3508_LEFT_ID = 0x201,
    CAN_3508_RIGHT_ID = 0x202,
    CAN_3508_BLEFT_ID = 0x203,
    CAN_3508_BRIGHT_ID = 0x204,
    // Can1

    CAN_GIMBAL_SHOOT2_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,

    CAN_CAP_ID = 0x211,

} can_msg_id_e;
// 自研超电
typedef enum
{
    BATTERY = 1,
    CAPACITY,
    OUT_OFF,
} power_source_enum;

// RM 电机数据
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    fp32 voltage;
    fp32 cuttent;
    fp32 power;
} pm_measure_t;

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      rev1: (0x207) 保留，电机控制电流
 * @param[in]      rev2: (0x208) 保留，电机控制电流
 * @retval         none
 */
extern void CAN_cmd_gimbal_shoot2(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);

extern void CAN_cmd_pitch(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
extern void CAN_cmd_chassis_reset_ID(void);

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      s0: (0x201) 2006电机控制电流, 范围 [-16384,16384]
 * @param[in]      s1: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      s2: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      trigger: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_shoot(int16_t left, int16_t right, int16_t bleft, int16_t bright);

extern void CAN_cmd_cap(int16_t temPower);
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
 * @brief          返回拨弹电机 数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_can_3508_left_measure_point(void);

/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_can_3508_right_measure_point(void);

/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_can_3508_bleft_measure_point(void);

/**
 * @brief          返回摩擦轮 3508电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_can_3508_bright_measure_point(void);
#endif

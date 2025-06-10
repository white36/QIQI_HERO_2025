/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "filter.h"
#include "controller.h"
#include "PID_control.h"
// ����ʼ����һ��ʱ��
#define SHOOT_TASK_INIT_TIME 300
// ��̨ģʽʹ�õĿ���ͨ��
#define SHOOT_CONTROL_TIME GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE 100.0f
// ���������Ƽ�� 2ms
#define SHOOT_CONTROL_TIME_MS 2
// ���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
// �������ʱ��
#define BUTTON_TIME 30
// �����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME 15
// ���ʱ��
#define SHOOT_TIME 100
// ��곤���ж�
#define PRESS_LONG_TIME 400
// ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000
// Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME 80
// �����������ֵ��Χ
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
// ���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000040372843107647f //-3.14--3.14
#define MOTOR_ECD_TO_ANG 0.002313193556470837f   //-180--180
#define FULL_COUNT 19

#define KEY_OFF_JUGUE_TIME 500

#define TRIGGER_DONE_TIME 500

#define READY 1
#define OFF 0

// ����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED 1.0f
#define BLOCK_TIME 1000
#define REVERSE_TIME 1000
#define REVERSE_SPEED_LIMIT 13.0f

#define SHOOT_DISABLE_TIME 200

#define PI_FOUR 0.78539816339744830961566084581988f
#define PI_TEN 0.314f

#define PWM_FIRST_UP 2380 // ����
#define PWM_FIRST_DOWN 630
#define PWM_MID_UP 1450 // �йؽ�
#define PWM_MID_DOWN 650
#define PWM_LAST_UP 1430 // �����ؽ�
#define PWM_LAST_DOWN 1020

// �����ֵ��PID
#define TRIGGER_ANGLE_PID_KP 1800.0f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 5.0f

#define TRIGGER_READY_PID_MAX_OUT 16000.0f
#define TRIGGER_READY_PID_MAX_IOUT 1000.0f

#define Trigger_MOTOR_Speed_PID_KP 3.4f
#define Trigger_MOTOR_Speed_PID_KI 2.0f
#define Trigger_MOTOR_Speed_PID_KD 0.0f
#define Trigger_MOTOR_Speed_PID_KF 0.0f
#define Trigger_MOTOR_Speed_PID_MAX_IOUT 10.0f
#define Trigger_MOTOR_Speed_PID_MAX_OUT 10.0f
#define Trigger_MOTOR_Speed_PID_DEAD_ZONE 0.0f
#define Trigger_MOTOR_Speed_I_Variable_Speed_A 0.0f
#define Trigger_MOTOR_Speed_I_Variable_Speed_B 0.0f
#define Trigger_MOTOR_Speed_I_Separate_Threshold 0.0f

// ǰ2Ħ����
#define FRIC_LEFT_MOTOR_SPEED_PID_KP 30.0f
#define FRIC_LEFT_MOTOR_SPEED_PID_KI 0.0f
#define FRIC_LEFT_MOTOR_SPEED_PID_KD 0.0f

#define FRIC_LEFT_MOTOR_SPEED_PID_MAX_OUT 16385.0f
#define FRIC_LEFT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define FRIC_RIGHT_MOTOR_SPEED_PID_KP 30.0f
#define FRIC_RIGHT_MOTOR_SPEED_PID_KI 0.0f
#define FRIC_RIGHT_MOTOR_SPEED_PID_KD 0.0f

#define FRIC_RIGHT_MOTOR_SPEED_PID_MAX_OUT 16385.0f
#define FRIC_RIGHT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f
// ��2Ħ����
#define FRIC_BLEFT_MOTOR_SPEED_PID_KP 30.0f
#define FRIC_BLEFT_MOTOR_SPEED_PID_KI 0.0f
#define FRIC_BLEFT_MOTOR_SPEED_PID_KD 0.0f

#define FRIC_BLEFT_MOTOR_SPEED_PID_MAX_OUT 16385.0f
#define FRIC_BLEFT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define FRIC_BRIGHT_MOTOR_SPEED_PID_KP 30.0f
#define FRIC_BRIGHT_MOTOR_SPEED_PID_KI 0.0f
#define FRIC_BRIGHT_MOTOR_SPEED_PID_KD 0.0f

#define FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_OUT 16385.0f
#define FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define SHOOT_HEAT_REMAIN_VALUE 80

#define SHOOT_THIRD_MODE 0

typedef struct
{
    ext_shoot_data_t *shoot_data;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *trigger_motor_measure;
    const motor_measure_t *fric_left_motor_measure;
    const motor_measure_t *fric_right_motor_measure;
    const motor_measure_t *fric_bleft_motor_measure;
    const motor_measure_t *fric_bright_motor_measure;
    pid_type_def trigger_pid;
    PID_control trigger_pid_B;
    pid_type_def fric_left_pid;
    pid_type_def fric_right_pid;
    pid_type_def fric_bleft_pid;
    pid_type_def fric_bright_pid;
    fp32 trigger_speed;
    fp32 trigger_speed_set;
    fp32 fric_left_speed;
    fp32 fric_left_speed_set;
    fp32 fric_right_speed;
    fp32 fric_right_speed_set;
    fp32 fric_bleft_speed;
    fp32 fric_bleft_speed_set;
    fp32 fric_bright_speed;
    fp32 fric_bright_speed_set;
    fp32 trigger_angle;
    fp32 trigger_angle_set;
    int16_t trigger_given_current;
    int8_t trigger_ecd_count;

    // ������ر���
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;
    TickType_t CTRL_V_press_time;
    TickType_t CTRL_B_press_time;
    TickType_t R_press_time;

    uint16_t block_time;   // �жϿ���ʱ��
    uint16_t reverse_time; // �˵�ʱ��
    uint16_t heat_limit;
    uint16_t heat;            // ����
    uint16_t shoot_time;      // ����ʱ�Ĳ���ʱ��
    uint8_t heat_delay_start; //
    TickType_t heat_delay_time;

    uint16_t bullet_number;
    uint16_t last_bullet_number;
    bool_t shoot_flag;             // ����
    bool_t last_shoot_flag;        // �ϴη����־λ
    bool_t shoot_continu_flag;     // �˵�
    bool_t stuck_flag;             // ����
    bool_t number_stuck_flag;      // ���⿨��
    bool_t last_number_stuck_flag; // ���⿨��
    bool_t stuck_trigger_flag;

    bool_t last_butten_trig_pin; // �ϴη���΢��״̬

    // ��������
    int16_t turn_trigger_time;
    int16_t turn_trigger_speed;
    int16_t first_speed;
    int16_t add_speed;
    fp32 speed_k;
    int8_t key_pin;
    // �������
    int16_t pwm_first_up;
    int16_t pwm_first_down;
    int16_t pwm_mid_up;
    int16_t pwm_mid_down;
    int16_t pwm_last_up;
    int16_t pwm_last_down;
    // ����ʣ����������
    uint16_t remain_heat;
    uint16_t remain_count;
    uint16_t last_remain_count;
    uint16_t remain_fire_count;
    // ����ϵͳ��������
    const ext_robot_state_t *robot_state;

    TickType_t SHOOT_xTickCount;
    TickType_t start_shoot_time;
    TickType_t remain_shoot_time;
} shoot_control_t;

extern shoot_control_t shoot_control;

extern void shoot_task(void const *pvParameters);

#endif

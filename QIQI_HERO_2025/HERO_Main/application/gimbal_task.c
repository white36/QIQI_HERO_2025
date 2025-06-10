/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot_task.h"
#include "pid.h"
#include "bsp_usart.h"
#include "referee.h"
#include <stdio.h>
#include "vision.h"
#include "vision_task.h"
#include "math.h"

// motor enconde value format, range[0-8191]
// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif
int16_t angle_sin, angle_cos;
float angle_radto;
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init);
/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
//  * @brief          吊射时云台锁定控制, 以及pitch限位调整。 用于吊射、过洞
//  * @param[out]     locking_control:"gimbal_control"变量指针.
//  * @retval         none
//  */
// static void gimbal_locking_control(gimbal_control_t *locking_control);
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop);
/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
 * @brief          在GIMBAL_REMOTE_FIRE模式，限制yaw角度设定
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_pitch_speed_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_TURN_BACK，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_yaw_turn_round_absolute_angle_limit(gimbal_control_t *gimbal_control);

/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_yaw_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

// 云台控制相关数据
gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern int16_t trigger_can_set_current;
extern shoot_control_t shoot_control;
extern ExtY_stm32 stm32_Y_yaw;
extern ExtY_stm32 stm32_Y_pitch;
// 视觉任务结构体
extern vision_control_t vision_control;
// 视觉数据
vision_rxfifo_t *vision_rx;
// UI
extern int16_t R;
extern int16_t turn_flags;
extern int16_t anglesr;
extern int16_t angle_sin, angle_cos;
extern float angle_radto;
int16_t pitch_dian = 2000;
extern int8_t AUTO_ATTACK; // 自瞄UI标志位
int hipnuc_flag = 0; // 陀螺仪解码成功flag
int last_hipnuc_flag = 0;
TickType_t xTickCount;                                   // 获取freertos系统时钟滴答数
int16_t YAW_current, PITCH_current, TRIGGER_current = 0; // 为了减少can发送包,用于发送电流
/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // 云台初始化
    gimbal_init(&gimbal_control);

    while (1)
    {
        gimbal_control.GIMBAL_xTickCount = xTaskGetTickCount();
        gimbal_set_mode(&gimbal_control);                    // 设置云台控制模式
        gimbal_mode_change_control_transit(&gimbal_control); // 控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);             // 云台数据反馈
        gimbal_set_control(&gimbal_control);                 // 设置云台控制量
        gimbal_control_loop(&gimbal_control);                // 云台控制PID计算

        static uint16_t Reset_time = 0;
        if (gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B && !(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL))
        {
            Reset_time++;
            if (Reset_time > 1500)
                NVIC_SystemReset();
        }

        if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
        {
            if (toe_is_error(DBUS_TOE) || gimbal_behaviour == GIMBAL_ZERO_FORCE)
            {
                YAW_current = 0;
                PITCH_current = 0;
            }
            else
            {
                YAW_current = -gimbal_control.gimbal_yaw_motor.given_current;
                PITCH_current = gimbal_control.gimbal_pitch_motor.given_current;
            }
        }

        // 吊射锁定丝杆
        if (gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
        {
            // 如果没按Z
            if (!(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z && !(gimbal_control.gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)))
            {
                PITCH_current = 0;
            }
        }
        CAN_cmd_gimbal_shoot2(YAW_current, PITCH_current, TRIGGER_current, 0);

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度++++++++++++++++++++++++++++++++++指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
    memset(init, 0, sizeof(gimbal_control_t));

    // 吊射pitch轴pid参数
    const static fp32 pitch_remote_fire_pid[3] = {PITCH_REMOTE_FIRE_PID_KP, PITCH_REMOTE_FIRE_PID_KI, PITCH_REMOTE_FIRE_PID_KD};

    // 电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    // 陀螺仪数据指针获取
    init->gimbal_INS_point = get_INS_point();
    // 遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    // 获取上位机视觉数据指针
    init->gimbal_vision_point = get_vision_gimbal_point();
    // 获取裁判系统数据
    init->robot_state = get_robot_status_point();
    // 初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    // 初始化电机pid
    stm32_pid_pitch_init();
    stm32_pid_yaw_init();
    stm32_pid_auto_pitch_init();
    stm32_pid_auto_yaw_init();
    // 初始化云台吊射控制(速度环)
    PID_init(&init->pitch_remote_fire_pid, PID_POSITION, pitch_remote_fire_pid, PITCH_REMOTE_FIRE_PID_MAX_OUT, PITCH_REMOTE_FIRE_PID_MAX_IOUT);
    // 自瞄PID初始化
    pid_init(&init->yaw_pos_pid, 5, 3, 0, 40, 0.01, 0, PI / 4, 0.0001, 0, 0, 2, 0xaf);
    pid_init(&init->yaw_speed_pid, 30000, 10000, 0, 25000, 0, 15, 3, 0.1, 0.001, 0.001, 2, 0xbf);

    init->gimbal_yaw_motor.offset_ecd = 7324;
    // 获取云台测量数据
    gimbal_feedback_update(init);
    // 重置设定值
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle = 0;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
    init->gimbal_pitch_motor.relative_chassis_angle = 0;

    // yaw轴相对角限位
    init->gimbal_yaw_motor.max_relative_angle = 3.14f;
    init->gimbal_yaw_motor.min_relative_angle = -3.14f;

    init->gimbal_pitch_motor.min_relative_angle = -0.65f;
    init->gimbal_pitch_motor.max_relative_angle = 0.02f;
    angle_radto = 0;
    angle_sin = 0;
    angle_cos = 0;

    // 记录鼠标按键
    init->last_press_l = init->press_l;
    init->press_l = init->gimbal_rc_ctrl->mouse.press_l;
}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }

    angle_sin = (int16_t)(100 * sin(0.48f - gimbal_control.gimbal_pitch_motor.relative_angle));
    angle_cos = (int16_t)(fabs(100 * cos(0.48f - gimbal_control.gimbal_pitch_motor.relative_angle)));
    // 设置云台行为状态机、电机状态机
    gimbal_behaviour_mode_set(set_mode);
}

/**
 * @brief          云台测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    // 更新PITCH相对底盘角度
    feedback_update->gimbal_pitch_motor.relative_chassis_angle = (-feedback_update->gimbal_INS_point->Pitch) - 0;
    // 云台数据更新
    // Pitch绝对角度
    feedback_update->gimbal_pitch_motor.absolute_angle = -feedback_update->gimbal_INS_point->Pitch;

    // PITCH电机角速度
    feedback_update->gimbal_pitch_motor.pitch_speed = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm * RPM_TO_RADPS;
    // Yaw绝对角度
    feedback_update->gimbal_yaw_motor.absolute_angle = feedback_update->gimbal_INS_point->Yaw;
    // Yaw相对角度
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                  feedback_update->gimbal_yaw_motor.offset_ecd);
    // Yaw角速度、转速
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[Z]) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[X]);
    feedback_update->gimbal_yaw_motor.motor_speed = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
}

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }
    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }

    // yaw电机状态机切换
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        stm32_U_yaw.P_P = 2200;
        stm32_U_yaw.P_I = 0;
        stm32_U_yaw.P_D = 55;
        stm32_U_yaw.P_N = 35;
        stm32_U_yaw.S_P = 90;
        stm32_U_yaw.S_I = 0;
        stm32_U_yaw.S_D = 5;
        stm32_U_yaw.S_N = 25;
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
        stm32_pid_clear_yaw();
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_REMOTE_FIRE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
        stm32_pid_clear_yaw();
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_AUTO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_TURN_ROUND && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_TURN_ROUND)
    {
        gimbal_mode_change->gimbal_back_init_angle = gimbal_mode_change->gimbal_yaw_motor.absolute_angle; // 记录初始角度
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
    }

    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    // pitch电机状态机切换
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.given_current = 0;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.given_current = 0;
        stm32_pid_clear_pitch();
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_REMOTE_FIRE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        gimbal_mode_change->gimbal_pitch_motor.pitch_speed_set = gimbal_mode_change->gimbal_pitch_motor.pitch_speed;
        gimbal_mode_change->gimbal_pitch_motor.given_current = 0;
        stm32_pid_clear_pitch();
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->gimbal_pitch_motor.given_current = 0;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_AUTO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.given_current = 0;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;

    //
    static int16_t last_key_c = 0;
    if (last_key_c && !(gimbal_mode_change->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C && !(gimbal_mode_change->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)))
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
    last_key_c = gimbal_mode_change->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C && !(gimbal_mode_change->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL);
}

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    // 遥控器数值赋予
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    // yaw电机模式控制
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        // 吊射模式下，yaw陀螺仪角度控制
        gimbal_yaw_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        // AUTO模式下，陀螺仪角度控制
        gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_yaw_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRONOLIMIT)
    {
        gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_TURN_ROUND)
    {
        gimbal_yaw_turn_round_absolute_angle_limit(set_control);
    }

    // pitch电机模式控制
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);

        if (set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)
        {
            set_control->gimbal_pitch_motor.absolute_angle_set = -0.56f;
        }
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        // 吊射模式下，pitch速度环控制
        gimbal_pitch_speed_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        // AUTO模式下，陀螺仪角度控制
        gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_pitch_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set;
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_yaw_turn_round_absolute_angle_limit(gimbal_control_t *gimbal_control)
{
    static fp32 init_angle_set;
    init_angle_set = gimbal_control->gimbal_back_init_angle;
    gimbal_control->gimbal_yaw_motor.absolute_angle_set = rad_format(init_angle_set + 3.00f);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO时，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:pitch电机
 * @retval         将原相对角度改为相对底盘陀螺仪角度
 */
static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    // now angle error
    // 当前控制误差角度
    static fp32 bias_angle;
    static fp32 angle_set; // 中间量

    // now angle error
    // 当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    // relative angle + angle error + add_angle > max_relative angle
    // 云台相对角度 + 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_chassis_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        // 如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            // calculate max add_angle
            // 计算出一个最大的添加角度
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_chassis_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_chassis_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_chassis_angle - bias_angle;
        }
    }

    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_REMOTE_FIRE，pitch使用速度环控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_pitch_speed_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    // add即设定速度
    gimbal_motor->pitch_speed_set = add;
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    // 是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}
static void gimbal_yaw_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    // now angle error
    // 当前控制误差角度
    static fp32 bias_angle;
    static fp32 angle_set; // 中间量

    // now angle error
    // 当前控制误差角度
    bias_angle = rad_format(gimbal_motor->relative_angle_set - gimbal_motor->relative_angle);

    // relative angle + angle error + add_angle > max_relative angle
    // 云台相对角度 + 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_chassis_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        // 如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            // calculate max add_angle
            // 计算出一个最大的添加角度
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_chassis_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_chassis_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_chassis_angle - bias_angle;
        }
    }

    angle_set = gimbal_motor->relative_angle_set;
    gimbal_motor->relative_angle_set = rad_format(angle_set + add);
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        if (&(control_loop->gimbal_yaw_motor) == NULL)
        {
            return;
        }
        control_loop->gimbal_yaw_motor.given_current = 0;
    }

    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        if (&(control_loop->gimbal_yaw_motor) == NULL)
        {
            return;
        }

        stm32_step_yaw(control_loop->gimbal_yaw_motor.absolute_angle_set, control_loop->gimbal_yaw_motor.absolute_angle, control_loop->gimbal_yaw_motor.motor_speed);
        control_loop->gimbal_yaw_motor.given_current = stm32_Y_yaw.Out1;
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE) // 吊射模式, 和绝对角控制一样
    {
        if (&(control_loop->gimbal_yaw_motor) == NULL)
        {
            return;
        }
        if (control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C && !(control_loop->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL))
        {
            stm32_U_yaw.P_P = 2200;
            stm32_U_yaw.P_I = 0;
            stm32_U_yaw.P_D = 55;
            stm32_U_yaw.P_N = 35;
            stm32_U_yaw.S_P = 90;
            stm32_U_yaw.S_I = 0;
            stm32_U_yaw.S_D = 5;
            stm32_U_yaw.S_N = 25;
        }
        else
        {
            stm32_U_yaw.P_P = 2500;
            stm32_U_yaw.P_I = 0;
            stm32_U_yaw.P_D = 55;
            stm32_U_yaw.P_N = 40;
            stm32_U_yaw.S_P = 90;
            stm32_U_yaw.S_I = 0;
            stm32_U_yaw.S_D = 5;
            stm32_U_yaw.S_N = 25;
        }

        stm32_step_yaw(control_loop->gimbal_yaw_motor.relative_angle_set, control_loop->gimbal_yaw_motor.relative_angle, control_loop->gimbal_yaw_motor.motor_speed);
        control_loop->gimbal_yaw_motor.given_current = stm32_Y_yaw.Out1;
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        if (&(control_loop->gimbal_yaw_motor) == NULL)
        {
            return;
        }

        fp32 out = yaw_pid_calculate(&control_loop->yaw_pos_pid, control_loop->gimbal_yaw_motor.absolute_angle, control_loop->gimbal_yaw_motor.absolute_angle_set);
        control_loop->gimbal_yaw_motor.given_current = pid_calculate(&control_loop->yaw_speed_pid, control_loop->gimbal_yaw_motor.motor_gyro, out);
    }

    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_TURN_ROUND)
    {
        if (&(control_loop->gimbal_yaw_motor) == NULL)
        {
            return;
        }

        stm32_step_yaw(control_loop->gimbal_yaw_motor.absolute_angle_set, control_loop->gimbal_yaw_motor.absolute_angle, control_loop->gimbal_yaw_motor.motor_speed);
        control_loop->gimbal_yaw_motor.given_current = stm32_Y_yaw.Out1;
    }

    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        if (&(control_loop->gimbal_yaw_motor) == NULL)
        {
            return;
        }
        stm32_step_yaw(control_loop->gimbal_yaw_motor.relative_angle_set, control_loop->gimbal_yaw_motor.relative_angle, control_loop->gimbal_yaw_motor.motor_speed);
        control_loop->gimbal_yaw_motor.given_current = stm32_Y_yaw.Out1;
    }

    /* PITCH */
    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        if (&(control_loop->gimbal_pitch_motor) == NULL)
        {
            return;
        }
        control_loop->gimbal_pitch_motor.given_current = 0;
    }

    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        if (&(control_loop->gimbal_pitch_motor) == NULL)
        {
            return;
        }
        // Matlab控制器 速度改为角速度适配meng调的pid参数
        stm32_step_pitch(control_loop->gimbal_pitch_motor.absolute_angle_set, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.motor_speed * (float)PI / 30.0f);
        control_loop->gimbal_pitch_motor.given_current = stm32_Y_pitch.Out1;
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        if (&(control_loop->gimbal_pitch_motor) == NULL)
        {
            return;
        }
        // 吊射模式, 使用速度环控制
        control_loop->gimbal_pitch_motor.given_current = PID_calc(&control_loop->pitch_remote_fire_pid, control_loop->gimbal_pitch_motor.pitch_speed, control_loop->gimbal_pitch_motor.pitch_speed_set);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        if (&(control_loop->gimbal_pitch_motor) == NULL)
        {
            return;
        }
        // Matlab控制器 速度改为角速度适配meng调的pid参数
        stm32_step_auto_pitch(control_loop->gimbal_pitch_motor.absolute_angle_set, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.motor_speed * (float)PI / 30.0f);
        control_loop->gimbal_pitch_motor.given_current = stm32_Y_auto_pitch.Out1;
    }

    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        if (&(control_loop->gimbal_pitch_motor) == NULL)
        {
            return;
        }
        stm32_step_pitch(control_loop->gimbal_pitch_motor.relative_angle_set, control_loop->gimbal_pitch_motor.relative_angle, control_loop->gimbal_pitch_motor.motor_speed);
        control_loop->gimbal_pitch_motor.given_current = stm32_Y_pitch.Out1;
    }
}

// 获取场地正方向
fp32 get_yaw_positive_direction(void)
{
    return gimbal_control.yaw_positive_direction;
}

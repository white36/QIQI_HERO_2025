/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "bsp_usart.h"
#include "Mathh.h"
#include "referee.h"
#include "shoot_task.h"
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
 * @param[out]     chassis_move_transit:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
 * @brief
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

// 解包双板数据
static void chassis_get_comm_data(chassis_move_t *unpack_comm_data);
// 打包双板数据
static void comm_data_pack(chassis_move_t *send_data_pack);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
// 底盘运动数据
chassis_move_t chassis_move;
extern int8_t QA, BPIN, FOLLOW;
int16_t cnts = 0;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern vision_rxfifo_t *vision_rx;
extern int8_t turn_flags; // 就近对位 回头flag
extern int MODE;
/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    // 底盘初始化
    chassis_init(&chassis_move);

    while (1)
    {
        chassis_get_comm_data(&chassis_move);
        // 设置底盘控制模式
        chassis_set_mode(&chassis_move);
        // 控制模式切换 控制数据过渡
        chassis_mode_change_control_transit(&chassis_move);
        // 底盘控制量设置
        chassis_set_contorl(&chassis_move);
        // 打包双板数据结构体
        comm_data_pack(&chassis_move);

        // 双板发送
        CAN_comm_down_A(chassis_move.comm_A_output);
        CAN_comm_down_B(chassis_move.comm_B_output);
        CAN_comm_down_C(chassis_move.comm_C_output);

        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    memset(chassis_move_init, 0, sizeof(chassis_move_t));
    // 跟随角度环pid参数
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    // 底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    // 获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    // 获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    // 获取裁判系统数据结构体指针
    chassis_move_init->robot_state = get_robot_status_point();
    chassis_move_init->power_heat_data = get_power_heat_data_point();
    chassis_move_init->shoot_data = get_shoot_data_point();

    // 初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

    // 用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    // 最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    // 初始化 置零变量
    chassis_move_init->twist_init_flag = 0;
    chassis_move_init->change_twist_flag = 0;
}

/**
 * @brief          解包双板接收数据
 * @param[out]     unpack_comm_data:"chassis_move"变量指针.
 * @retval         none
 */
void chassis_get_comm_data(chassis_move_t *unpack_comm_data)
{
    /* a包 */
    // 当前热量
    unpack_comm_data->power_heat_data->shooter_42mm_barrel_heat = unpack_comm_data->comm_rx_a.rx_current_heat;
    // 等级 并算出热量上限
    unpack_comm_data->robot_state->robot_level = unpack_comm_data->comm_rx_a.rx_robo_level;
    if (unpack_comm_data->robot_state->robot_level == 1)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 200;
    else if (unpack_comm_data->robot_state->robot_level == 2)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 230;
    else if (unpack_comm_data->robot_state->robot_level == 3)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 260;
    else if (unpack_comm_data->robot_state->robot_level == 4)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 290;
    else if (unpack_comm_data->robot_state->robot_level == 5)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 320;
    else if (unpack_comm_data->robot_state->robot_level == 6)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 350;
    else if (unpack_comm_data->robot_state->robot_level == 7)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 380;
    else if (unpack_comm_data->robot_state->robot_level == 8)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 410;
    else if (unpack_comm_data->robot_state->robot_level == 9)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 440;
    else if (unpack_comm_data->robot_state->robot_level == 10)
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 500;
    else
        unpack_comm_data->robot_state->shooter_barrel_heat_limit = 500;
    // 射击初速度 (仅有两位小数)
    unpack_comm_data->shoot_data->initial_speed = ((float)unpack_comm_data->comm_rx_a.rx_initial_speed_x100) / 100.0f;
    // 接收的flag
    if (unpack_comm_data->comm_rx_a.rx_flag & COMM_FLAG_GIMBAL_OUTPUT)
    {
        unpack_comm_data->robot_state->power_management_gimbal_output = 1;
    }
    else
    {
        unpack_comm_data->robot_state->power_management_gimbal_output = 0;
    }
    if (unpack_comm_data->comm_rx_a.rx_flag & COMM_FLAG_SHOOTER_OUTPUT)
    {
        unpack_comm_data->robot_state->power_management_shooter_output = 1;
    }
    else
    {
        unpack_comm_data->robot_state->power_management_shooter_output = 0;
    }
    if (unpack_comm_data->comm_rx_a.rx_flag & COMM_FLAG_ROBOT_ID)
    {
        unpack_comm_data->robot_state->robot_id = 101; // 蓝方
    }
    else
    {
        unpack_comm_data->robot_state->robot_id = 1; // 红方
    }
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    // in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
 * @param[out]     chassis_move_transit:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    // 切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    // 切入跟随底盘角度模式（暂无）
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    // 切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    // 切入一键掉头模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_TURN_ROUND) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_TURN_ROUND)
    {
        chassis_move_transit->chassis_back_init_angle = chassis_move_transit->chassis_yaw_motor->relative_angle;
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

    // 键盘控制
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    // 一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    // 停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set;

    // get three control set-point, 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    // 跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        // 设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        // 计算旋转PID角速度
        if (gimbal_behaviour == GIMBAL_AUTO_ATTACK) // 自瞄模式下无力
        {
            chassis_move_control->wz_set = 0;
        }
        else // 普通计算
        {
            if (fabs(chassis_move_control->chassis_relative_angle_set - chassis_move_control->chassis_yaw_motor->relative_angle) >= 0.02f)
                chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
            else
                chassis_move_control->wz_set = 0;
        }

        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        // 设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        // 计算旋转的角速度
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        // 设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        // 底盘旋转角速度置零
        chassis_move_control->wz_set = 0;

        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        // 在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_BPIN)
    {

        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        sin_yaw = (arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle));
        cos_yaw = (arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle));

        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

        chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
        if (MODE == 1) // 遥控器陀螺
        {
            if (chassis_move.chassis_RC->rc.ch[4] > 50)
            {
                chassis_move_control->wz_set = angle_set;
            }
            if (chassis_move.chassis_RC->rc.ch[4] < -50)
            {
                chassis_move_control->wz_set = -angle_set;
            }
        }
        else // 键鼠陀螺
        {
            chassis_move_control->wz_set = angle_set;
            chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
            chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
        }
    }
    // 底盘摇摆模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_TWIST)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        // 设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        // 计算旋转PID角速度
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);

        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    // 一键掉头模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_TURN_ROUND)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        if (fabs(chassis_move_control->chassis_back_init_angle) < 1.57f) // 初始朝前
        {
            // 掉头朝后
            sin_yaw = arm_sin_f32(3.14f);
            cos_yaw = arm_cos_f32(3.14f);
        }
        else if (fabs(chassis_move_control->chassis_back_init_angle) > 1.57f) // 初始朝后
        {
            sin_yaw = arm_sin_f32(0.0f);
            cos_yaw = arm_cos_f32(0.0f);
        }
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        // 旋转角速度直接置零
        chassis_move_control->wz_set = 0;
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
}

/**
 * @brief          发送双板数据帧
 * @param[out]     send_data_pack:"chassis_move"变量指针.
 * @retval         none
 */
int8_t R, QA, BPIN, AUTO_ATTACK, STUCK, VISION, FOLLOW = 0;
void comm_data_pack(chassis_move_t *send_data_pack)
{
    /* A包 */
    // 打包数据至发送的结构体
    send_data_pack->comm_tx_A.tx_vx_set = send_data_pack->vx_set;
    send_data_pack->comm_tx_A.tx_vy_set = send_data_pack->vy_set;
    PACK_STRUCT_TO_CAN_BUFFER(send_data_pack->comm_tx_A, send_data_pack->comm_A_output);

    /* B包 */
    static uint8_t send_flag_b;
    static uint8_t chassis_mode = 4;
    static uint8_t send_Flag = 0;

    if (turn_flags)
        send_flag_b |= (1 << 7);
    else
        send_flag_b &= ~(1 << 7);
    if (send_data_pack->chassis_RC->key.v & KEY_PRESSED_OFFSET_C)
        send_flag_b |= (1 << 6);
    else
        send_flag_b &= ~(1 << 6);
    if (gimbal_behaviour == GIMBAL_REMOTE_FIRE)
        send_flag_b |= (1 << 5);
    else
        send_flag_b &= ~(1 << 5);

    // 保留位

    // 发送底盘模式(留出前四位发送flag,后四位可发16个底盘模式)(第34位发flag好像会影响mode)
    if (send_data_pack->chassis_mode == CHASSIS_VECTOR_RAW)
        chassis_mode = 0;
    else if (send_data_pack->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
        chassis_mode = 1;
    else if (send_data_pack->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
        chassis_mode = 2;
    else if (send_data_pack->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
        chassis_mode = 3;
    else if (send_data_pack->chassis_mode == CHASSIS_VECTOR_BPIN)
        chassis_mode = 4;
    else if (send_data_pack->chassis_mode == CHASSIS_VECTOR_TWIST)
        chassis_mode = 5;
    else if (send_data_pack->chassis_mode == CHASSIS_VECTOR_TURN_ROUND)
        chassis_mode = 6;
    else
        chassis_mode = 0; // 无力模式
    // 注意赋完模式再进行位操作
    if (send_data_pack->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z)
        chassis_mode |= (1 << 7);
    else
        send_Flag &= ~(1 << 7);
    if (send_data_pack->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q)
        chassis_mode |= (1 << 6);
    else
        send_Flag &= ~(1 << 6);

    /* Flag */
    if (send_data_pack->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) // 0
        send_Flag |= (1 << 0);
    else
        send_Flag &= ~(1 << 0);

    if (R) // 1
        send_Flag |= (1 << 1);
    else
        send_Flag &= ~(1 << 1);

    if (QA) // 2
        send_Flag |= (1 << 2);
    else
        send_Flag &= ~(1 << 2);

    if (BPIN) // 3
        send_Flag |= (1 << 3);
    else
        send_Flag &= ~(1 << 3);

    if (AUTO_ATTACK) // 4
        send_Flag |= (1 << 4);
    else
        send_Flag &= ~(1 << 4);

    if (STUCK) // 5
        send_Flag |= (1 << 5);
    else
        send_Flag &= ~(1 << 5);

    if (VISION) // 6
        send_Flag |= (1 << 6);
    else
        send_Flag &= ~(1 << 6);
    if (toe_is_error(DBUS_TOE)) // 7
        send_Flag |= (1 << 7);
    else
        send_Flag &= ~(1 << 7);

    // 打包数据至发送的结构体
    send_data_pack->comm_tx_B.tx_vz_set = send_data_pack->wz_set;
    send_data_pack->comm_tx_B.tx_send_flag_b = send_flag_b;
    // reserve
    send_data_pack->comm_tx_B.tx_chassis_mode = chassis_mode;
    send_data_pack->comm_tx_B.tx_send_Flag = send_Flag;
    PACK_STRUCT_TO_CAN_BUFFER(send_data_pack->comm_tx_B, send_data_pack->comm_B_output);

    /* C包 */
    static int16_t first_speed;
    static int16_t back_speed;

    first_speed = (int)(shoot_control.fric_left_speed + fabs(shoot_control.fric_right_speed)) / 2;
    back_speed = (int)(shoot_control.fric_bleft_speed + fabs(shoot_control.fric_bright_speed)) / 2;

    send_data_pack->comm_tx_C.tx_PITCH = send_data_pack->chassis_pitch_motor->absolute_angle;
    send_data_pack->comm_tx_C.tx_first_speed = first_speed;
    send_data_pack->comm_tx_C.tx_add_speed = back_speed;
    PACK_STRUCT_TO_CAN_BUFFER(send_data_pack->comm_tx_C, send_data_pack->comm_C_output);
}

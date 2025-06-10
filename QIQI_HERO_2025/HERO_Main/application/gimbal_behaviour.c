/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculate by
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
    add a gimbal behaviour mode
    1. in gimbal_behaviour.h , add a new behaviour name in gimbal_behaviour_e
    erum
    {
        ...
        ...
        GIMBAL_XXX_XXX, // new add
    }gimbal_behaviour_e,
    2. implement new function. gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" param is gimbal movement contorl input.
        first param: 'yaw' usually means  yaw axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.
        second param: 'pitch' usually means pitch axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.

        in this new function, you can assign set-point to "yaw" and "pitch",as your wish
    3.  in "gimbal_behavour_set" function, add new logical judgement to assign GIMBAL_XXX_XXX to  "gimbal_behaviour" variable,
        and in the last of the "gimbal_behaviour_mode_set" function, add "else if(gimbal_behaviour == GIMBAL_XXX_XXX)"
        choose a gimbal control mode.
        four mode:
        GIMBAL_MOTOR_RAW : will use 'yaw' and 'pitch' as motor current set,  derectly sent to can bus.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' are angle increment,  control enconde relative angle.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' are angle increment,  control gyro absolute angle.
    4. in the last of "gimbal_behaviour_control_set" function, add
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }


    如果要添加一个新的行为模式
    1.首先，在gimbal_behaviour.h文件中， 添加一个新行为名字在 gimbal_behaviour_e
    erum
    {
        ...
        ...
        GIMBAL_XXX_XXX, // 新添加的
    }gimbal_behaviour_e,

    2. 实现一个新的函数 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" 参数是云台运动控制输入量
        第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
    3.  在"gimbal_behavour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
        在gimbal_behaviour_mode_set函数最后，添加"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,然后选择一种云台控制模式
        3种:
        GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
    4.  在"gimbal_behaviour_control_set" 函数的最后，添加
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "vision.h"
#include "user_lib.h"
#include "referee.h"
#include "INS_task.h"
// when gimbal is in calibrating, set buzzer frequency and strenght
// 当云台在校准, 设置蜂鸣器频率和强度
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()

extern ext_robot_state_t robot_state;

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
 * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
 * @param          input:the raw channel value
 * @param          output: the processed channel value
 * @param          deadline
 */
/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
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
 * @brief          judge if gimbal reaches the limit by gyro
 * @param          gyro: rotation speed unit rad/s
 * @param          timing time, input "GIMBAL_CALI_STEP_TIME"
 * @param          record angle, unit rad
 * @param          feedback angle, unit rad
 * @param          record ecd, unit raw
 * @param          feedback ecd, unit raw
 * @param          cali step, +1 by one step
 */
/**
 * @brief          通过判断角速度来判断云台是否到达极限位置
 * @param          对应轴的角速度，单位rad/s
 * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
 * @param          记录的角度 rad
 * @param          反馈的角度 rad
 * @param          记录的编码值 raw
 * @param          反馈的编码值 raw
 * @param          校准的步骤 完成一次 加一
 */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
 * @brief          gimbal behave mode set.
 * @param[in]      gimbal_mode_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
 *                 and gimbal control mode is raw. The raw mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all zero.
 * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
 * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
 *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
 *                 and rotate yaw axis.
 * @param[out]     yaw: yaw motor relative angle increment, unit rad.
 * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
 *                 and gimbal control mode is gyro mode.
 * @param[out]     yaw: yaw axia absolute angle increment, unit rad
 * @param[out]     pitch: pitch axia absolute angle increment,unit rad
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_remote_fire_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
 *                 and gimbal control mode is encode mode.
 * @param[out]     yaw: yaw axia relative angle increment, unit rad
 * @param[out]     pitch: pitch axia relative angle increment,unit rad
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
 *                 and gimbal control mode is encode mode.
 * @param[out]     yaw: yaw axia relative angle increment,  unit rad
 * @param[out]     pitch: pitch axia relative angle increment, unit rad
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
 * @brief          陀螺模式
 * @param[in]      yaw: 绝对角度控制
 * @param[in]      pitch: 相对角度控制
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_absolute_spin_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_turn_round_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
// 云台行为状态机

static void gimbal_auto_attack_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
// 自动袭击

gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

extern vision_rxfifo_t *vision_rx;
extern int8_t AUTO_ATTACK;

/**
 * @brief          the function is called by gimbal_set_mode function in gimbal_task.c
 *                 the function set gimbal_behaviour variable, and set motor mode.
 * @param[in]      gimbal_mode_set: gimbal data
 * @retval         none
 */
/**
 * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    // set gimbal_behaviour variable
    // 云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);

    // accoring to gimbal_behaviour, set motor control mode
    // 根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_REMOTE_FIRE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_REMOTE_FIRE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_REMOTE_FIRE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_SPIN)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;
    }
    else if (gimbal_behaviour == GIMBAL_TURN_ROUND)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_TURN_ROUND;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
}

/**
 * @brief          the function is called by gimbal_set_contorl function in gimbal_task.c
 *                 accoring to the gimbal_behaviour variable, call the corresponding function
 * @param[out]     add_yaw:yaw axis increment angle, unit rad
 * @param[out]     add_pitch:pitch axis increment angle,unit rad
 * @param[in]      gimbal_mode_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
 * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
 * @param[in]      gimbal_mode_set:云台数据指针
 * @retval         none
 */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_REMOTE_FIRE)
    {
        // add为速度量
        gimbal_remote_fire_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_SPIN)
    {
        gimbal_absolute_spin_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        gimbal_auto_attack_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_TURN_ROUND)
    {
        gimbal_turn_round_control(add_yaw, add_pitch, gimbal_control_set);
    }
}

/**
 * @brief          in some gimbal mode, need chassis keep no move
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
/**
 * @brief          云台在某些行为下，需要底盘不动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          in some gimbal mode, need shoot keep no move
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          gimbal behave mode set.
 * @param[in]      gimbal_mode_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
extern int8_t REMOTE_FIRE_MODE;
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    static int16_t last_key_G = 0;
    static int16_t last_key_F = 0;
    static int16_t move_flag = 0;
    static bool_t turn_auto_flag = 0;
    static bool_t last_turn_auto_flag = 0;
    static bool_t gimbal_rc_mode = 0;

    // 键鼠移动开关
    if (!last_key_G && gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G)
    {
        move_flag = !move_flag;
    }

    static int auto_mode = 0; // 是否开启自瞄(后加入吊射)
    if (REMOTE_FIRE_MODE)
    {
        auto_mode = 2;
    }
    else if (gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r)
    {
        if (judge_vision_appear_target())
        {
            auto_mode = 1;
        }
        else
        {
            auto_mode = 0;
        }
    }
    else
    {
        auto_mode = 0;
    }

    if (gimbal_mode_set->gimbal_rc_ctrl->rc.ch[4] > -120)
    {
        turn_auto_flag = 0;
    }
    else if (gimbal_mode_set->gimbal_rc_ctrl->rc.ch[4] < -600)
    {
        turn_auto_flag = 1;
    }
    // 遥控器控制时切换自瞄模式
    if (last_turn_auto_flag == 0 && turn_auto_flag == 1)
    {
        gimbal_rc_mode = !gimbal_rc_mode;
    }

    if (gimbal_behaviour == GIMBAL_TURN_ROUND) // 一键掉头时不进入其他模式
    {
        if (gimbal_mode_set->GIMBAL_xTickCount - gimbal_mode_set->tmp_start_time > 700)
        {
            gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
            gimbal_mode_set->tmp_remain_time = gimbal_mode_set->GIMBAL_xTickCount - gimbal_mode_set->tmp_start_time;
        }
    }
    else // 非一键掉头
    {
        // 开关控制 云台状态
        if (!switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[1])) // 左开启
        {
            if (gimbal_rc_mode == 0)
            {
                gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
            }
            else if (gimbal_rc_mode == 1)
            {
                gimbal_behaviour = GIMBAL_AUTO_ATTACK;
            }
        }
        else if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[1]) && move_flag) // 左关闭 键鼠操作
        {
            if (auto_mode == 0)
            {
                gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
                AUTO_ATTACK = 0;
            }
            else if (auto_mode == 1)
            {
                gimbal_behaviour = GIMBAL_AUTO_ATTACK;
                AUTO_ATTACK = 1;
            }
            else if (auto_mode == 2)
            {
                gimbal_behaviour = GIMBAL_REMOTE_FIRE;
            }

            // 触发一键掉头
            if (!last_key_F && gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)
            {
                gimbal_behaviour = GIMBAL_TURN_ROUND;
                gimbal_mode_set->tmp_start_time = gimbal_mode_set->GIMBAL_xTickCount;
            }
        }
        else
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
    }
    last_key_F = gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F;
    last_key_G = gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G;
    last_turn_auto_flag = turn_auto_flag;
    gimbal_mode_set->last_press_r = gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r;
}

/**
 * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
 *                 and gimbal control mode is raw. The raw mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all zero.
 * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
 * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
 * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
 *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
 *                 and rotate yaw axis.
 * @param[out]     yaw: yaw motor relative angle increment, unit rad.
 * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 * @author         RM
 * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      云台数据指针
 * @retval         返回空
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    // 初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
 * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
 *                 and gimbal control mode is gyro mode.
 * @param[out]     yaw: yaw axia absolute angle increment, unit rad
 * @param[out]     pitch: pitch axia absolute angle increment,unit rad
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;
    //    static int16_t limit = 0;

    //    if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)
    //    {
    //        limit = !limit;
    //    }
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    //    if (limit == 1)
    //    {
    //        *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN * 0.2f - vision_rx->ang_z;
    //        *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN * 0.2f;
    //    }
    //    else
    //    {
    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN * 0.8f;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN * 0.8f;
    //    }
}

/**
 * @brief          云台吊射速度环控制
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴速度控制,为速度设定值
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_remote_fire_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    // pitch仅写了键鼠控制
    if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C && !(gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        *yaw = -gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN * 0.2f;
    }
    else
    {
        *yaw = 0;
    }

    if (gimbal_control_set->gimbal_rc_ctrl->mouse.y > 0.4) // 上移鼠标,抬头
    {
        *pitch = -48.0f;
    }
    else if (gimbal_control_set->gimbal_rc_ctrl->mouse.y < -0.4) // 下移鼠标,低头
    {
        *pitch = 27.0f;
    }
    else
    {
        *pitch = 0.0f;
    }
}

/**
 * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
 *                 and gimbal control mode is encode mode.
 * @param[out]     yaw: yaw axia relative angle increment, unit rad
 * @param[out]     pitch: pitch axia relative angle increment,unit rad
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = (pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN) * 0.1f;
}
/**
 * @brief          陀螺模式
 * @param[in]      yaw: 绝对角度控制
 * @param[in]      pitch: 相对角度控制
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_absolute_spin_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN * 0.8f;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN * 0.8f;
}

/**
 * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
 *                 and gimbal control mode is encode mode.
 * @param[out]     yaw: yaw axia relative angle increment,  unit rad
 * @param[out]     pitch: pitch axia relative angle increment, unit rad
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_turn_round_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = 0.0f;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN * 0.8f;
}

static void gimbal_auto_attack_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    // yaw pitch 轴设定值与当前值的差值
    fp32 pitch_error = 0;
    fp32 yaw_error = 0;

    // pitch轴yaw轴设定角度
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;

    pitch_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_pitch;
    yaw_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_yaw;

    // 计算过去设定角度与当前角度之间的差值
    yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    pitch_error = gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;
    //  获取上位机视觉数据

    // 赋值增量
    if (yaw_set_angle && pitch_set_angle && vision_control.vision_target_appear_state == TARGET_APPEAR)
    {
        *yaw = yaw_set_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle - yaw_error;
        *pitch = (pitch_set_angle - gimbal_control_set->gimbal_pitch_motor.absolute_angle - pitch_error);
    }
    else
    {
        *yaw = 0.0f;
        *pitch = 0.0f;
    }
}

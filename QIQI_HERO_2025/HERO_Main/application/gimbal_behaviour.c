/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculate by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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


    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���gimbal_behaviour.h�ļ��У� ���һ������Ϊ������ gimbal_behaviour_e
    erum
    {
        ...
        ...
        GIMBAL_XXX_XXX, // ����ӵ�
    }gimbal_behaviour_e,

    2. ʵ��һ���µĺ��� gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" ��������̨�˶�����������
        ��һ������: 'yaw' ͨ������yaw���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        �ڶ�������: 'pitch' ͨ������pitch���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        ������µĺ���, ���ܸ� "yaw"��"pitch"��ֵ��Ҫ�Ĳ���
    3.  ��"gimbal_behavour_set"��������У�����µ��߼��жϣ���gimbal_behaviour��ֵ��GIMBAL_XXX_XXX
        ��gimbal_behaviour_mode_set����������"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,Ȼ��ѡ��һ����̨����ģʽ
        3��:
        GIMBAL_MOTOR_RAW : ʹ��'yaw' and 'pitch' ��Ϊ��������趨ֵ,ֱ�ӷ��͵�CAN������.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' �ǽǶ�����,  ���Ʊ�����ԽǶ�.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' �ǽǶ�����,  ���������Ǿ��ԽǶ�.
    4.  ��"gimbal_behaviour_control_set" ������������
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
// ����̨��У׼, ���÷�����Ƶ�ʺ�ǿ��
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
 * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
 * @param          �����ң����ֵ
 * @param          ��������������ң����ֵ
 * @param          ����ֵ
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
 * @brief          ͨ���жϽ��ٶ����ж���̨�Ƿ񵽴Ｋ��λ��
 * @param          ��Ӧ��Ľ��ٶȣ���λrad/s
 * @param          ��ʱʱ�䣬����GIMBAL_CALI_STEP_TIME��ʱ������
 * @param          ��¼�ĽǶ� rad
 * @param          �����ĽǶ� rad
 * @param          ��¼�ı���ֵ raw
 * @param          �����ı���ֵ raw
 * @param          У׼�Ĳ��� ���һ�� ��һ
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
 * @brief          ��̨��Ϊ״̬������.
 * @param[in]      gimbal_mode_set: ��̨����ָ��
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
 * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
 *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
 * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      gimbal_control_set: ��̨����ָ��
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
 * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
 * @param[out]     yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      ��̨����ָ��
 * @retval         ���ؿ�
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
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
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
 * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set: ��̨����ָ��
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
 * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
 * @brief          ����ģʽ
 * @param[in]      yaw: ���ԽǶȿ���
 * @param[in]      pitch: ��ԽǶȿ���
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_absolute_spin_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_turn_round_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
// ��̨��Ϊ״̬��

static void gimbal_auto_attack_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
// �Զ�Ϯ��

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
 * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
 * @param[out]     gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    // set gimbal_behaviour variable
    // ��̨��Ϊ״̬������
    gimbal_behavour_set(gimbal_mode_set);

    // accoring to gimbal_behaviour, set motor control mode
    // ������̨��Ϊ״̬�����õ��״̬��
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
 * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
 * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
 * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
 * @param[in]      gimbal_mode_set:��̨����ָ��
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
        // addΪ�ٶ���
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
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
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
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
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
 * @brief          ��̨��Ϊ״̬������.
 * @param[in]      gimbal_mode_set: ��̨����ָ��
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

    // �����ƶ�����
    if (!last_key_G && gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G)
    {
        move_flag = !move_flag;
    }

    static int auto_mode = 0; // �Ƿ�������(��������)
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
    // ң��������ʱ�л�����ģʽ
    if (last_turn_auto_flag == 0 && turn_auto_flag == 1)
    {
        gimbal_rc_mode = !gimbal_rc_mode;
    }

    if (gimbal_behaviour == GIMBAL_TURN_ROUND) // һ����ͷʱ����������ģʽ
    {
        if (gimbal_mode_set->GIMBAL_xTickCount - gimbal_mode_set->tmp_start_time > 700)
        {
            gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
            gimbal_mode_set->tmp_remain_time = gimbal_mode_set->GIMBAL_xTickCount - gimbal_mode_set->tmp_start_time;
        }
    }
    else // ��һ����ͷ
    {
        // ���ؿ��� ��̨״̬
        if (!switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[1])) // ����
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
        else if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[1]) && move_flag) // ��ر� �������
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

            // ����һ����ͷ
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
 * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
 *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
 * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      gimbal_control_set: ��̨����ָ��
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
 * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
 * @author         RM
 * @param[out]     yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      ��̨����ָ��
 * @retval         ���ؿ�
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    // ��ʼ��״̬����������
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
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
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
 * @brief          ��̨�����ٶȻ�����
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch���ٶȿ���,Ϊ�ٶ��趨ֵ
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_remote_fire_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    // pitch��д�˼������
    if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C && !(gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        *yaw = -gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN * 0.2f;
    }
    else
    {
        *yaw = 0;
    }

    if (gimbal_control_set->gimbal_rc_ctrl->mouse.y > 0.4) // �������,̧ͷ
    {
        *pitch = -48.0f;
    }
    else if (gimbal_control_set->gimbal_rc_ctrl->mouse.y < -0.4) // �������,��ͷ
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
 * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set: ��̨����ָ��
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
 * @brief          ����ģʽ
 * @param[in]      yaw: ���ԽǶȿ���
 * @param[in]      pitch: ��ԽǶȿ���
 * @param[in]      gimbal_control_set: ��̨����ָ��
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
 * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
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
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
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
    // yaw pitch ���趨ֵ�뵱ǰֵ�Ĳ�ֵ
    fp32 pitch_error = 0;
    fp32 yaw_error = 0;

    // pitch��yaw���趨�Ƕ�
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;

    pitch_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_pitch;
    yaw_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_yaw;

    // �����ȥ�趨�Ƕ��뵱ǰ�Ƕ�֮��Ĳ�ֵ
    yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    pitch_error = gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;
    //  ��ȡ��λ���Ӿ�����

    // ��ֵ����
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

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
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
// �������ֵ���� 0��8191
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
 * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
 * @param[out]     init:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init);
/**
 * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
 * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
/**
 * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
 * @param[out]     mode_change:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
 * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
 * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
//  * @brief          ����ʱ��̨��������, �Լ�pitch��λ������ ���ڵ��䡢����
//  * @param[out]     locking_control:"gimbal_control"����ָ��.
//  * @retval         none
//  */
// static void gimbal_locking_control(gimbal_control_t *locking_control);
/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop);
/**
 * @brief          ��GIMBAL_MOTOR_GYROģʽ�����ƽǶ��趨,��ֹ�������
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
 * @brief          ��GIMBAL_REMOTE_FIREģʽ������yaw�Ƕ��趨
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_pitch_speed_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_TURN_BACK��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_yaw_turn_round_absolute_angle_limit(gimbal_control_t *gimbal_control);

/**
 * @brief          ��GIMBAL_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_yaw_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

// ��̨�����������
gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern int16_t trigger_can_set_current;
extern shoot_control_t shoot_control;
extern ExtY_stm32 stm32_Y_yaw;
extern ExtY_stm32 stm32_Y_pitch;
// �Ӿ�����ṹ��
extern vision_control_t vision_control;
// �Ӿ�����
vision_rxfifo_t *vision_rx;
// UI
extern int16_t R;
extern int16_t turn_flags;
extern int16_t anglesr;
extern int16_t angle_sin, angle_cos;
extern float angle_radto;
int16_t pitch_dian = 2000;
extern int8_t AUTO_ATTACK; // ����UI��־λ
int hipnuc_flag = 0; // �����ǽ���ɹ�flag
int last_hipnuc_flag = 0;
TickType_t xTickCount;                                   // ��ȡfreertosϵͳʱ�ӵδ���
int16_t YAW_current, PITCH_current, TRIGGER_current = 0; // Ϊ�˼���can���Ͱ�,���ڷ��͵���
/**
 * @brief          ����yaw �������ָ��
 * @param[in]      none
 * @retval         yaw���ָ��
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          ����pitch �������ָ��
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
 * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void gimbal_task(void const *pvParameters)
{
    // �ȴ������������������������
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // ��̨��ʼ��
    gimbal_init(&gimbal_control);

    while (1)
    {
        gimbal_control.GIMBAL_xTickCount = xTaskGetTickCount();
        gimbal_set_mode(&gimbal_control);                    // ������̨����ģʽ
        gimbal_mode_change_control_transit(&gimbal_control); // ����ģʽ�л� �������ݹ���
        gimbal_feedback_update(&gimbal_control);             // ��̨���ݷ���
        gimbal_set_control(&gimbal_control);                 // ������̨������
        gimbal_control_loop(&gimbal_control);                // ��̨����PID����

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

        // ��������˿��
        if (gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
        {
            // ���û��Z
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
 * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�++++++++++++++++++++++++++++++++++ָ���ʼ��
 * @param[out]     init:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
    memset(init, 0, sizeof(gimbal_control_t));

    // ����pitch��pid����
    const static fp32 pitch_remote_fire_pid[3] = {PITCH_REMOTE_FIRE_PID_KP, PITCH_REMOTE_FIRE_PID_KI, PITCH_REMOTE_FIRE_PID_KD};

    // �������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    // ����������ָ���ȡ
    init->gimbal_INS_point = get_INS_point();
    // ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();
    // ��ȡ��λ���Ӿ�����ָ��
    init->gimbal_vision_point = get_vision_gimbal_point();
    // ��ȡ����ϵͳ����
    init->robot_state = get_robot_status_point();
    // ��ʼ�����ģʽ
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    // ��ʼ�����pid
    stm32_pid_pitch_init();
    stm32_pid_yaw_init();
    stm32_pid_auto_pitch_init();
    stm32_pid_auto_yaw_init();
    // ��ʼ����̨�������(�ٶȻ�)
    PID_init(&init->pitch_remote_fire_pid, PID_POSITION, pitch_remote_fire_pid, PITCH_REMOTE_FIRE_PID_MAX_OUT, PITCH_REMOTE_FIRE_PID_MAX_IOUT);
    // ����PID��ʼ��
    pid_init(&init->yaw_pos_pid, 5, 3, 0, 40, 0.01, 0, PI / 4, 0.0001, 0, 0, 2, 0xaf);
    pid_init(&init->yaw_speed_pid, 30000, 10000, 0, 25000, 0, 15, 3, 0.1, 0.001, 0.001, 2, 0xbf);

    init->gimbal_yaw_motor.offset_ecd = 7324;
    // ��ȡ��̨��������
    gimbal_feedback_update(init);
    // �����趨ֵ
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle = 0;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
    init->gimbal_pitch_motor.relative_chassis_angle = 0;

    // yaw����Խ���λ
    init->gimbal_yaw_motor.max_relative_angle = 3.14f;
    init->gimbal_yaw_motor.min_relative_angle = -3.14f;

    init->gimbal_pitch_motor.min_relative_angle = -0.65f;
    init->gimbal_pitch_motor.max_relative_angle = 0.02f;
    angle_radto = 0;
    angle_sin = 0;
    angle_cos = 0;

    // ��¼��갴��
    init->last_press_l = init->press_l;
    init->press_l = init->gimbal_rc_ctrl->mouse.press_l;
}

/**
 * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
 * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
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
    // ������̨��Ϊ״̬�������״̬��
    gimbal_behaviour_mode_set(set_mode);
}

/**
 * @brief          ��̨�������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    // ����PITCH��Ե��̽Ƕ�
    feedback_update->gimbal_pitch_motor.relative_chassis_angle = (-feedback_update->gimbal_INS_point->Pitch) - 0;
    // ��̨���ݸ���
    // Pitch���ԽǶ�
    feedback_update->gimbal_pitch_motor.absolute_angle = -feedback_update->gimbal_INS_point->Pitch;

    // PITCH������ٶ�
    feedback_update->gimbal_pitch_motor.pitch_speed = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm * RPM_TO_RADPS;
    // Yaw���ԽǶ�
    feedback_update->gimbal_yaw_motor.absolute_angle = feedback_update->gimbal_INS_point->Yaw;
    // Yaw��ԽǶ�
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                  feedback_update->gimbal_yaw_motor.offset_ecd);
    // Yaw���ٶȡ�ת��
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[Z]) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[X]);
    feedback_update->gimbal_yaw_motor.motor_speed = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
}

/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
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
 * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
 * @param[out]     gimbal_mode_change:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }

    // yaw���״̬���л�
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
        gimbal_mode_change->gimbal_back_init_angle = gimbal_mode_change->gimbal_yaw_motor.absolute_angle; // ��¼��ʼ�Ƕ�
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
        gimbal_mode_change->gimbal_yaw_motor.given_current = 0;
    }

    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    // pitch���״̬���л�
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
 * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
 * @param[out]     set_control:"gimbal_control"����ָ��.
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
    // ң������ֵ����
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    // yaw���ģʽ����
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyroģʽ�£������ǽǶȿ���
        gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        // ����ģʽ�£�yaw�����ǽǶȿ���
        gimbal_yaw_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        // AUTOģʽ�£������ǽǶȿ���
        gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // encondeģʽ�£��������Ƕȿ���
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

    // pitch���ģʽ����
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyroģʽ�£������ǽǶȿ���
        gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);

        if (set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)
        {
            set_control->gimbal_pitch_motor.absolute_angle_set = -0.56f;
        }
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        // ����ģʽ�£�pitch�ٶȻ�����
        gimbal_pitch_speed_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        // AUTOģʽ�£������ǽǶȿ���
        gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // encondeģʽ�£��������Ƕȿ���
        gimbal_pitch_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set;
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_yaw_turn_round_absolute_angle_limit(gimbal_control_t *gimbal_control)
{
    static fp32 init_angle_set;
    init_angle_set = gimbal_control->gimbal_back_init_angle;
    gimbal_control->gimbal_yaw_motor.absolute_angle_set = rad_format(init_angle_set + 3.00f);
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYROʱ��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:pitch���
 * @retval         ��ԭ��ԽǶȸ�Ϊ��Ե��������ǽǶ�
 */
static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    // now angle error
    // ��ǰ�������Ƕ�
    static fp32 bias_angle;
    static fp32 angle_set; // �м���

    // now angle error
    // ��ǰ�������Ƕ�
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    // relative angle + angle error + add_angle > max_relative angle
    // ��̨��ԽǶ� + ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_motor->relative_chassis_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        // �����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)
        {
            // calculate max add_angle
            // �����һ��������ӽǶ�
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
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_REMOTE_FIRE��pitchʹ���ٶȻ�����
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_pitch_speed_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    // add���趨�ٶ�
    gimbal_motor->pitch_speed_set = add;
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    // �Ƿ񳬹���� ��Сֵ
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
    // ��ǰ�������Ƕ�
    static fp32 bias_angle;
    static fp32 angle_set; // �м���

    // now angle error
    // ��ǰ�������Ƕ�
    bias_angle = rad_format(gimbal_motor->relative_angle_set - gimbal_motor->relative_angle);

    // relative angle + angle error + add_angle > max_relative angle
    // ��̨��ԽǶ� + ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_motor->relative_chassis_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        // �����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)
        {
            // calculate max add_angle
            // �����һ��������ӽǶ�
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
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
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
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE) // ����ģʽ, �;��Խǿ���һ��
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
        // Matlab������ �ٶȸ�Ϊ���ٶ�����meng����pid����
        stm32_step_pitch(control_loop->gimbal_pitch_motor.absolute_angle_set, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.motor_speed * (float)PI / 30.0f);
        control_loop->gimbal_pitch_motor.given_current = stm32_Y_pitch.Out1;
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE)
    {
        if (&(control_loop->gimbal_pitch_motor) == NULL)
        {
            return;
        }
        // ����ģʽ, ʹ���ٶȻ�����
        control_loop->gimbal_pitch_motor.given_current = PID_calc(&control_loop->pitch_remote_fire_pid, control_loop->gimbal_pitch_motor.pitch_speed, control_loop->gimbal_pitch_motor.pitch_speed_set);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_AUTO)
    {
        if (&(control_loop->gimbal_pitch_motor) == NULL)
        {
            return;
        }
        // Matlab������ �ٶȸ�Ϊ���ٶ�����meng����pid����
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

// ��ȡ����������
fp32 get_yaw_positive_direction(void)
{
    return gimbal_control.yaw_positive_direction;
}

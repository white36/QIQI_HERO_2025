/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      �������.
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

#include "shoot_task.h"
#include "chassis_task.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_servo_pwm.h"
#include "bsp_pwm.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "tim.h"
#include "stm32.h"
#include "vision_task.h"

static void shoot_init(shoot_control_t *shoot_init);
/**
 * @brief          ͼ��̧������ �������
 * @param[in]      void
 * @retval         void
 */
static void servo_pwm_control(shoot_control_t *pwm_control);
/**
 * @brief          ������ݸ���
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(shoot_control_t *feedback_update);
/**
 * @brief          �����������ʣ��ɷ�������
 * @param[in]      void
 * @retval         void
 */
static void remain_fire_count_calc(shoot_control_t *remain_count_control);
/**
 * @brief          �ֶ�����Ħ����ת��
 * @param[out]     wheel_speed_control:"shoot_control_t"����ָ��.
 * @retval         none
 */
static void adjust_wheel_speed(shoot_control_t *wheel_speed_control);
/**
 * @brief          Ħ����ģʽ�л�
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(shoot_control_t *set_mode);
/**
 * @brief          ����ٶȼ���
 * @param[in]      void
 * @retval         void
 */
static void shoot_control_loop(shoot_control_t *control_loop);
/**
 * @brief          �������̻ز�
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_block_control(shoot_control_t *block_control);

// ��������Ħ����
#define shoot_fric(speed, add)                                    \
    do                                                            \
    {                                                             \
        if (speed == 0)                                           \
        {                                                         \
            shoot_control.fric_left_speed_set = 0;                \
            shoot_control.fric_right_speed_set = 0;               \
            shoot_control.fric_bleft_speed_set = 0;               \
            shoot_control.fric_bright_speed_set = 0;              \
        }                                                         \
        else                                                      \
        {                                                         \
            shoot_control.fric_left_speed_set = speed;            \
            shoot_control.fric_right_speed_set = -speed;          \
            shoot_control.fric_bleft_speed_set = (speed + add);   \
            shoot_control.fric_bright_speed_set = -(speed + add); \
        }                                                         \
    } while (0)
// ���κ궨��
#define FIRST_SPEED_1 5200
#define SPEED_K 0.75f
#define ADD_SPEED_1 SPEED_K *FIRST_SPEED_1 - FIRST_SPEED_1

#define TURN_TRIGGER_TIME 30  // ����ʱ��
#define TURN_TRIGGER_SPEED 40 // �����ٶ�

#define TRIGGER_COMMON_SPEED 7.4f // ���湩���ٶ�
// �����궨��
#define trigger_motor(speed) shoot_control.trigger_speed_set = speed // �����������
// ΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

extern int8_t STUCK;
int16_t limitnb = 0, a = 0, hh = 0;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
shoot_control_t shoot_control; // �������
int16_t s_can_set_current = 0, left_can_set_current = 0, right_can_set_current = 0, bleft_can_set_current = 0, bright_can_set_current = 0, trigger_can_set_current = 0;
fp32 last_speed;
uint8_t shoot_allow_flag = 0, success_flag = 0;
fp32 q, w, e, r;
extern int16_t TRIGGER_current;
int8_t REMOTE_FIRE_MODE = 0;

/**
 * @brief          �������
 * @param[in]      void
 * @retval         ����can����ֵ
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    shoot_init(&shoot_control);
    while (1)
    {
        shoot_control.SHOOT_xTickCount = xTaskGetTickCount();
        servo_pwm_control(&shoot_control); // ͼ���������
        shoot_set_mode(&shoot_control);
        shoot_feedback_update(&shoot_control);
        shoot_control_loop(&shoot_control); // ���÷���������

        if (toe_is_error(DBUS_TOE))
        {
            CAN_cmd_shoot(0, 0, 0, 0);
            TRIGGER_current = 0;
        }
        else
        {
            CAN_cmd_shoot(left_can_set_current, right_can_set_current, bleft_can_set_current, bright_can_set_current);
            TRIGGER_current = trigger_can_set_current;
        }
        vTaskDelay(SHOOT_CONTROL_TIME_MS);
    }
}

/**
 * @brief          ��ʼ��"shoot_control"����.
 * @param[out]     init:"shoot_control"����ָ��.
 * @retval         none
 */
static void shoot_init(shoot_control_t *shoot_init)
{
    memset(shoot_init, 0, sizeof(shoot_control_t));
    // ���PID������ʼ��
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 fric_left_pid[3] = {FRIC_LEFT_MOTOR_SPEED_PID_KP, FRIC_LEFT_MOTOR_SPEED_PID_KI, FRIC_LEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_right_pid[3] = {FRIC_RIGHT_MOTOR_SPEED_PID_KP, FRIC_RIGHT_MOTOR_SPEED_PID_KI, FRIC_RIGHT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bleft_pid[3] = {FRIC_BLEFT_MOTOR_SPEED_PID_KP, FRIC_BLEFT_MOTOR_SPEED_PID_KI, FRIC_BLEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bright_pid[3] = {FRIC_BRIGHT_MOTOR_SPEED_PID_KP, FRIC_BRIGHT_MOTOR_SPEED_PID_KI, FRIC_BRIGHT_MOTOR_SPEED_PID_KD};

    shoot_init->robot_state = get_robot_status_point();
    shoot_init->shoot_data = get_shoot_data_point();
    // ң����ָ��
    shoot_init->shoot_rc = get_remote_control_point();
    // ���ָ��
    shoot_init->trigger_motor_measure = get_trigger_motor_measure_point();
    shoot_init->fric_left_motor_measure = get_can_3508_left_measure_point();
    shoot_init->fric_right_motor_measure = get_can_3508_right_measure_point();
    shoot_init->fric_bleft_motor_measure = get_can_3508_bleft_measure_point();
    shoot_init->fric_bright_motor_measure = get_can_3508_bright_measure_point();
    // ��ʼ��PID
    PID_init(&shoot_init->trigger_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_left_pid, PID_POSITION, fric_left_pid, FRIC_LEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_LEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_right_pid, PID_POSITION, fric_right_pid, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_bleft_pid, PID_POSITION, fric_bleft_pid, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_bright_pid, PID_POSITION, fric_bright_pid, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_IOUT);

    // ��������
    shoot_init->shoot_time = 150;

    shoot_init->pwm_first_up = PWM_FIRST_UP;
    shoot_init->pwm_first_down = PWM_FIRST_DOWN;
    shoot_init->pwm_mid_up = PWM_MID_UP;
    shoot_init->pwm_mid_down = PWM_MID_DOWN;
    shoot_init->pwm_last_up = PWM_LAST_UP;
    shoot_init->pwm_last_down = PWM_LAST_DOWN;

    shoot_init->turn_trigger_time = TURN_TRIGGER_TIME;
    shoot_init->turn_trigger_speed = TURN_TRIGGER_SPEED;
    shoot_init->first_speed = FIRST_SPEED_1;
    shoot_init->speed_k = SPEED_K;
    shoot_init->add_speed = ADD_SPEED_1;
}

/**
 * @brief          ����ͼ��̧�������Ķ��
 * @param[out]     pwm_control:"shoot_control_t"����ָ��.
 * @retval         none
 */
static void servo_pwm_control(shoot_control_t *pwm_control)
{
    static int8_t servo_flag = -100;
    static TickType_t servo_last_tick = 0;
    static uint16_t KEY_CTRL_C = 0;
    static uint16_t KEY_CTRL_Z = 0;
    static uint16_t last_KEY_CTRL_C = 0;
    static uint16_t last_KEY_CTRL_Z = 0;
    static uint16_t last_KEY_X = 0;

    KEY_CTRL_C = pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    KEY_CTRL_Z = pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    // CTRL+C:̧�� CTRL+Z:����
    if (!last_KEY_CTRL_C && KEY_CTRL_C)
    {
        pwm_control->pwm_mid_up -= 25;
    }
    else if (!last_KEY_CTRL_Z && KEY_CTRL_Z)
    {
        pwm_control->pwm_mid_up += 25;
    }
    // ���pwm��ֵ
    if (servo_flag == -100)
    {
        // ���˶�״̬ˢ��pwmֵ,Ϊ����ϰ�������pwm ,��˳��Ϊ�˶�˳��
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_down);   // �йؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_down);  // �����ؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_down); // ����
        if (!last_KEY_X && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = 1;
        }
    }
    else if (servo_flag == 100)
    {
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_up);  // �����ؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_up);   // �йؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_up); // ����
        if (!last_KEY_X && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = -1;
        }
    }
    /* ̧�� */
    if (servo_flag == 1)
    {
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_up); // �����ؽ�
        servo_last_tick = xTaskGetTickCount();
        servo_flag = 2;

        // ��������ģʽ��־λ
        REMOTE_FIRE_MODE = 1;
    }
    else if (servo_flag == 2)
    {
        if (xTaskGetTickCount() - servo_last_tick > 150)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_up);   // �йؽ�
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_up); // ����
            servo_flag = 100;                                              // ����״̬
        }
    }
    /* �½� */
    if (servo_flag == -1)
    {
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_down); // �йؽ�
        servo_last_tick = xTaskGetTickCount();
        servo_flag = -2;

        // �رյ���ģʽ��־λ
        REMOTE_FIRE_MODE = 0;
    }
    else if (servo_flag == -2)
    {
        if (xTaskGetTickCount() - servo_last_tick > 220)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_down);  // �����ؽ�
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_down); // ����
            servo_flag = -100;                                               // ����״̬
        }
    }

    // ���޷�

    last_KEY_X = pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X;
    last_KEY_CTRL_C = (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C) && (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
    last_KEY_CTRL_Z = (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z) && (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
}

/**
 * @brief          ������ݸ���
 * @param[out]     feedback_update:"shoot_control_t"����ָ��.
 * @retval         none
 */
static void shoot_feedback_update(shoot_control_t *feedback_update)
{
    feedback_update->trigger_speed = feedback_update->trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

    // ����ٶȸ���
    shoot_control.fric_left_speed = shoot_control.fric_left_motor_measure->speed_rpm;
    shoot_control.fric_right_speed = shoot_control.fric_right_motor_measure->speed_rpm;
    shoot_control.fric_bleft_speed = shoot_control.fric_bleft_motor_measure->speed_rpm;
    shoot_control.fric_bright_speed = shoot_control.fric_bright_motor_measure->speed_rpm;

    // �۲�΢������״̬
    if (BUTTEN_TRIG_PIN)
    {
        feedback_update->key_pin = 1;
    }
    else
    {
        feedback_update->key_pin = 0;
    }

    // ���¿ɷ�������
    remain_fire_count_calc(feedback_update);

    // ���ֶ�����Ħ����ת��(�����Ż��ṹʱ����set_control��)
    adjust_wheel_speed(feedback_update);

    // ��¼��갴��
    feedback_update->last_press_l = feedback_update->press_l;
    feedback_update->last_press_r = feedback_update->press_r;
    feedback_update->press_l = feedback_update->shoot_rc->mouse.press_l;
    feedback_update->press_r = feedback_update->shoot_rc->mouse.press_r;
}

/**
 * @brief          �����������ʣ��ɷ�������
 * @param[out]     remain_count_control:"shoot_control_t"����ָ��.
 * @retval         none
 */
static void remain_fire_count_calc(shoot_control_t *remain_count_control)
{
    static uint16_t limit;
    static uint16_t heat;
    limit = robot_state.shooter_barrel_heat_limit;
    heat = power_heat_data.shooter_42mm_barrel_heat;
    // ����ɻ�������, ��ֹ����ϵͳ�ӳٵ��³����� �������Ϊ�ɷ����־λ
    remain_count_control->remain_heat = limit - heat;
    if (remain_count_control->remain_heat < 100)
    {
        remain_count_control->remain_count = 0;
    }
    else if (remain_count_control->remain_heat >= 100 && remain_count_control->remain_heat < 200)
    {
        remain_count_control->remain_count = 1;
    }
    else if (remain_count_control->remain_heat >= 200 && remain_count_control->remain_heat < 300)
    {
        remain_count_control->remain_count = 2;
    }
    else if (remain_count_control->remain_heat >= 300 && remain_count_control->remain_heat < 400)
    {
        remain_count_control->remain_count = 3;
    }
    else if (remain_count_control->remain_heat >= 400 && remain_count_control->remain_heat < 500)
    {
        remain_count_control->remain_count = 4;
    }
    else if (remain_count_control->remain_heat == 500)
    {
        remain_count_control->remain_count = 5;
    }
    else
    {
        remain_count_control->remain_count = 0;
    }
    // ÿ��һ��������ֵ,���¿ɻ�������
    if (remain_count_control->remain_count != remain_count_control->last_remain_count)
    {
        remain_count_control->remain_fire_count = remain_count_control->remain_count;
    }

    // �޸����ӳٵ��µ� ����ʱʶ�𲻵�����������ֵ��˲��
    if (remain_count_control->remain_count > remain_count_control->remain_fire_count)
    {
        if (remain_count_control->heat_delay_start == 0)
        {
            remain_count_control->heat_delay_time = remain_count_control->SHOOT_xTickCount;
            remain_count_control->heat_delay_start = 1;
        }
        if (remain_count_control->heat_delay_start == 1)
        {
            if (remain_count_control->SHOOT_xTickCount - remain_count_control->heat_delay_time > 300)
            {
                remain_count_control->remain_fire_count = remain_count_control->remain_count;
            }
        }
    }
    else
    {
        remain_count_control->heat_delay_start = 0;
    }

    // ��¼��һʱ������
    remain_count_control->last_remain_count = remain_count_control->remain_count;
}
/**
 * @brief          �ֶ�����Ħ����ת��
 * @param[out]     wheel_speed_control:"shoot_control_t"����ָ��.
 * @retval         none
 */
static void adjust_wheel_speed(shoot_control_t *wheel_speed_control)
{
    static uint16_t KEY_CTRL_V = 0;
    static uint16_t KEY_CTRL_B = 0;
    static uint16_t last_KEY_CTRL_V = 0;
    static uint16_t last_KEY_CTRL_B = 0;
    int16_t first_speed_adjst = 0;

    KEY_CTRL_V = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_V && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    KEY_CTRL_B = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_B && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    /* CTRL+V:ǰ���ٶ� CTRL+B:���� */
    // ��¼����ʱ��
    if (!last_KEY_CTRL_V && KEY_CTRL_V)
    {
        wheel_speed_control->CTRL_V_press_time = wheel_speed_control->SHOOT_xTickCount;
    }
    if (!last_KEY_CTRL_B && KEY_CTRL_B)
    {
        wheel_speed_control->CTRL_B_press_time = wheel_speed_control->SHOOT_xTickCount;
    }
    // �̰�����, ��������
    // �ɿ�����ж�
    if (last_KEY_CTRL_V && !KEY_CTRL_V)
    {
        if (wheel_speed_control->SHOOT_xTickCount - wheel_speed_control->CTRL_V_press_time < 500) // �̰�
        {
            first_speed_adjst = -25;
        }
        else if (wheel_speed_control->SHOOT_xTickCount - wheel_speed_control->CTRL_V_press_time >= 500) // ����
        {
            first_speed_adjst = +25;
        }
    }

    // �����ֵ����
    wheel_speed_control->first_speed += first_speed_adjst;
    // ����first�ͱ���һֱ��add
    wheel_speed_control->add_speed = wheel_speed_control->speed_k * wheel_speed_control->first_speed - wheel_speed_control->first_speed;

    last_KEY_CTRL_V = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_V && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    last_KEY_CTRL_B = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_B && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
}

/**
 * @brief          ���״̬������
 * @param[out]     set_mode:"shoot_control_t"����ָ��.
 * @retval         none
 */
extern int8_t R; // ����Ħ����
int16_t x = 0;
int s = 2000, l;
bool_t fireOK;
int8_t datatx_OK = 0;
int32_t datatx_time = 0;
static void shoot_set_mode(shoot_control_t *set_mode)
{
    static uint16_t last_KEY_R = 0;
    static uint16_t CTRL_R_time = 0;

    // ���̿��Ƴ���R������Ħ����
    if (!last_KEY_R && set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R)
    {
        set_mode->R_press_time = set_mode->SHOOT_xTickCount;
    }
    // �ɿ�����ж�
    if (last_KEY_R && !(set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R))
    {
        if (set_mode->SHOOT_xTickCount - set_mode->R_press_time > 500) // ����
        {
            R = !R;
        }
    }
    last_KEY_R = set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R;

    if ((switch_is_up(set_mode->shoot_rc->rc.s[1]) || R))
    {
        STUCK = 1; // ����UI��1 ��������
        // ���ݲ���ϵͳ ���Ƶ���
        set_mode->add_speed = set_mode->speed_k * set_mode->first_speed - set_mode->first_speed;
        shoot_fric(set_mode->first_speed, set_mode->add_speed);

        // �����жϼ��ز��߼�
        trigger_motor_block_control(set_mode);

        if (set_mode->stuck_flag == 0 && set_mode->number_stuck_flag == 0) // ������ ��������������߼�
        {
            STUCK = 0; // ���������㿨��UI

            // ����
            if (set_mode->stuck_trigger_flag == 1)
            {
                trigger_motor(TRIGGER_COMMON_SPEED + 1.0f); // ��ͨ���������ʹ�ø��칩���ٶ�
            }
            else if (BUTTEN_TRIG_PIN == RESET) // �����͵�ƽ
            {
                trigger_motor(0.0f);
            }
            else if (BUTTEN_TRIG_PIN == SET) // Ĭ�ϸߵ�ƽ
            {
                trigger_motor(TRIGGER_COMMON_SPEED); // �����ٶ� // С��5.0
            }

            if (set_mode->stuck_trigger_flag == 1 && BUTTEN_TRIG_PIN == RESET) // ���ٹ����������־λ
            {
                set_mode->stuck_trigger_flag = 0;
            }

            // Ӧ������ ��ֹ�������
            if ((set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL) && (set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R))
            {
                CTRL_R_time++;
                if (CTRL_R_time > 200 && (set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL) && (set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R))
                {
                    trigger_motor(TRIGGER_COMMON_SPEED);
                }
            }
            else
            {
                CTRL_R_time = 0;
            }

            // ����ʱ��¼�򵯸��� ͬʱ�ɻ���������1
            if (set_mode->last_butten_trig_pin == RESET && BUTTEN_TRIG_PIN == SET)
            {
                set_mode->bullet_number++;
                set_mode->remain_fire_count--;

                // ���ڲɼ�����
                datatx_time = 0;
                set_mode->remain_shoot_time = set_mode->SHOOT_xTickCount - set_mode->start_shoot_time;
            }
            // ���ݲɼ�
            if (datatx_time == 10)
            {
                datatx_OK = 1;
            }
            else
            {
                datatx_OK = 0;
            }
            datatx_time++;
            if (datatx_time > 2147483645)
            {
                datatx_time = 11;
            }

            // ����ģʽʱ��shoot_flag����, ���������¿ŵ���
            if (switch_is_up(set_mode->shoot_rc->rc.s[1]))
            {
                if (set_mode->shoot_rc->rc.ch[4] < 120)
                {
                    set_mode->shoot_flag = 0;
                }
            }
            else if (R)
            {
                if (set_mode->press_l == 0)
                {
                    set_mode->shoot_flag = 0;
                }
            }

            if (set_mode->press_r) // ������ʱ,�����ֶ�����(��Ϊ��̨����һֱ������ģʽ)
            {
                // ����
                if ((set_mode->shoot_flag == 0                                                                                             // ����1
                     && ((set_mode->shoot_rc->rc.ch[4] > 600 || set_mode->press_l)                                                         // ��������2:�ֶ�����������־
                         && (vision_control.shoot_vision_control.shoot_command == SHOOT_ATTACK && gimbal_behaviour == GIMBAL_AUTO_ATTACK)) // ��������2:���鴥��������־
                     && BUTTEN_TRIG_PIN == RESET)                                                                                          // ����3
                    && set_mode->remain_fire_count > 0)                                                                                    // �������������
                {
                    set_mode->shoot_flag = 1;
                    set_mode->last_bullet_number = set_mode->bullet_number;
                    set_mode->start_shoot_time = set_mode->SHOOT_xTickCount; // debug���������ӳ�
                }
            }
            else
            {
                // ����
                if ((set_mode->shoot_flag == 0                                    // ����1
                     && (set_mode->shoot_rc->rc.ch[4] > 600 || set_mode->press_l) // ��������2:���鴥��������־
                     && BUTTEN_TRIG_PIN == RESET)                                 // ����3
                    && set_mode->remain_fire_count > 0)                           // �������������
                {
                    set_mode->shoot_flag = 1;
                    set_mode->last_bullet_number = set_mode->bullet_number;
                    set_mode->start_shoot_time = set_mode->SHOOT_xTickCount;
                }
            }
        }
        else if (set_mode->stuck_flag == 1 || set_mode->number_stuck_flag == 1) // ����
        {
            trigger_motor(-5.0f); // �����ز��ٶ�
        }
        else if (set_mode->stuck_flag == 2 || set_mode->number_stuck_flag == 2) // �����ز���ȴ�
        {
            trigger_motor(0); // �����̵ȴ��ٶ�
        }
        else if (set_mode->number_stuck_flag == 3) // ���Ⲧ�����������ٲ�
        {
            trigger_motor(5.0f);
        }
        else if (set_mode->number_stuck_flag == 4) // ���Ⲧ�����Ĳ����ٲ�
        {
            trigger_motor(3.0f);
        }
    } // switch_is_up���ݽ���
    else
    {
        shoot_fric(0, 0);
        trigger_motor(0);
    }

    // �����ź�
    if (set_mode->last_shoot_flag == 0 && set_mode->shoot_flag == 1)
    {
        set_mode->shoot_time = 0;
    }
    // ����
    if (set_mode->shoot_time < set_mode->turn_trigger_time) // ������ʱ��
    {
        trigger_motor(set_mode->turn_trigger_speed); // �������ٶ�
    }
    else if (set_mode->shoot_time == (set_mode->turn_trigger_time))
    {
        trigger_motor(0);
    }

    set_mode->shoot_time++;

    if (set_mode->shoot_time >= 210)
        set_mode->shoot_time = 210;

    set_mode->last_butten_trig_pin = BUTTEN_TRIG_PIN;
    set_mode->last_shoot_flag = set_mode->shoot_flag;
}

/**
 * @brief          ����ٶȼ���
 * @param[out]     control_loop:"shoot_control_t"����ָ��.
 * @retval         none
 */
static void shoot_control_loop(shoot_control_t *control_loop)
{
    // ����pid
    PID_calc(&control_loop->trigger_pid, control_loop->trigger_speed, control_loop->trigger_speed_set);
    // ��Ħ����pid����
    PID_calc(&control_loop->fric_left_pid, control_loop->fric_left_speed, control_loop->fric_left_speed_set);
    PID_calc(&control_loop->fric_right_pid, control_loop->fric_right_speed, control_loop->fric_right_speed_set);
    PID_calc(&control_loop->fric_bleft_pid, control_loop->fric_bleft_speed, control_loop->fric_bleft_speed_set);
    PID_calc(&control_loop->fric_bright_pid, control_loop->fric_bright_speed, control_loop->fric_bright_speed_set);

    trigger_can_set_current = control_loop->trigger_pid.out;

    left_can_set_current = control_loop->fric_left_pid.out;
    right_can_set_current = control_loop->fric_right_pid.out;
    bleft_can_set_current = control_loop->fric_bleft_pid.out;
    bright_can_set_current = control_loop->fric_bright_pid.out;
}

/**
 * @brief          �����̻ز�����
 * @param[out]     block_control:"shoot_control_t"����ָ��.
 * @retval         none
 */
static void trigger_motor_block_control(shoot_control_t *block_control)
{
    if (trigger_can_set_current >= TRIGGER_COMMON_SPEED * 1800.0f - 600.0f)
    {
        // �ж�1 ����ʱ������ȥ
        block_control->block_time++;
        if (block_control->block_time > 600) // ���ʱ��//800
        {
            block_control->stuck_flag = 1;
            block_control->block_time = 0;
        }
    }
    else // ������ʱ���ü�ʱ
    {
        block_control->block_time = 0;
    }

    // �����ز�ʱ�䰲�ţ�2����
    if (block_control->stuck_flag == 1)
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 80) // �ز�ʱ��//40
        {
            block_control->reverse_time = 0;
            block_control->stuck_flag = 2;
        }
    }
    else if (block_control->stuck_flag == 2) // �ز���ȴ�������ز�����
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 100) // �ȴ�ʱ��
        {
            block_control->reverse_time = 0;
            block_control->stuck_flag = 0;
            block_control->stuck_trigger_flag = 1;
        }
    }

    // ���⿨���ز�
    if (block_control->number_stuck_flag == 1)
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 50)
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 2;
        }
    }
    else if (block_control->number_stuck_flag == 2) // �ز���ȴ�������ز�����
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 70) // �ȴ�ʱ��
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 3;
        }
    }
    else if (block_control->number_stuck_flag == 3) // �ز���ȴ�������ز�����
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 80) // ���ϵ�ʱ��
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 4;
        }
    }
    else if (block_control->number_stuck_flag == 4) // �ز���ȴ�������ز�����
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 40) // �ϵ�ʱ��
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 0;
        }
    }
}

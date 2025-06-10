#include "shoot_task.h"
#include "chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
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

/**
 * @brief          ��������ʼ��
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_init(shoot_control_t *shoot_control);

/**
 * @brief          ͼ��̧�������������
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
void servo_pwm_control(shoot_control_t *shoot_control);

/**
 * @brief          ���ģʽ����
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_set_mode(shoot_control_t *shoot_control);

/**
 * @brief          �������ѭ�����������ٶ�
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_control_loop(shoot_control_t *shoot_control);

/**
 * @brief          ����������ݸ���
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_feedback_update(shoot_control_t *shoot_control);

/**
 * @brief          �������̻ز�����
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void trigger_motor_block_control(shoot_control_t *shoot_control);

#define shoot_fric(speed, add)                                    \
    do                                                            \
    {                                                             \
        if (speed == 0)                                           \
        {                                                         \
            shoot_control->fric_left_speed_set = 0;                \
            shoot_control->fric_right_speed_set = 0;               \
            shoot_control->fric_bleft_speed_set = 0;               \
            shoot_control->fric_bright_speed_set = 0;              \
        }                                                         \
        else                                                      \
        {                                                         \
            shoot_control->fric_left_speed_set = speed;            \
            shoot_control->fric_right_speed_set = -speed;          \
            shoot_control->fric_bleft_speed_set = (speed + add);   \
            shoot_control->fric_bright_speed_set = -(speed + add); \
        }                                                         \
    } while (0)

#define trigger_motor(speed) shoot_control->trigger_speed_set = speed
#define third_fric(speed) shoot_control->fric_b_speed_set = -speed

/**
 * @brief          �������������
 * @param[in]      pvParameters: FreeRTOS�������
 * @retval         void
 * @note           ���������ѭ����������ʼ����ģʽ���á�����ѭ����
 */
void shoot_task(void const *pvParameters)
{
    // �ȴ�ϵͳ��ʼ�����
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // ��ʼ���������
    shoot_init(&shoot_control);
    while (1)
    {
        // ���PWM����
        servo_pwm_control(&shoot_control);
        // �������ģʽ
        shoot_set_mode(&shoot_control);
        // ���������������
        shoot_feedback_update(&shoot_control);
        // ִ���������ѭ��
        shoot_control_loop(&shoot_control);

        // ���ң����ͨ��״̬
        if (toe_is_error(DBUS_TOE))
        {
            // ͨ�Ŵ���ʱֹͣ���е��
            CAN_cmd_shoot(0, 0, 0, 0);
            TRIGGER_current = 0;
        }
        // ��鷢�������Դ״̬
        else if (gimbal_control.robot_state->power_management_shooter_output == 0)
        {
            // ��Դ�ر�ʱֹͣ���е��
            CAN_cmd_shoot(0, 0, 0, 0);
            TRIGGER_current = 0;
        }
        else
        {
            // ��������ʱ���͵����������
            CAN_cmd_shoot(left_can_set_current, right_can_set_current, bleft_can_set_current, bright_can_set_current);
            TRIGGER_current = trigger_can_set_current;
        }

        // ������ʱ
        vTaskDelay(SHOOT_CONTROL_TIME_MS);
    }
}

/**
 * @brief          ��������ʼ��
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_init(shoot_control_t *shoot_control)
{
    // ���PID������ʼ��
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 fric_b_pid[3] = {FRIC_S_MOTOR_SPEED_PID_KP, FRIC_S_MOTOR_SPEED_PID_KI, FRIC_S_MOTOR_SPEED_PID_KD};
    static const fp32 fric_left_pid[3] = {FRIC_LEFT_MOTOR_SPEED_PID_KP, FRIC_LEFT_MOTOR_SPEED_PID_KI, FRIC_LEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_right_pid[3] = {FRIC_RIGHT_MOTOR_SPEED_PID_KP, FRIC_RIGHT_MOTOR_SPEED_PID_KI, FRIC_RIGHT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bleft_pid[3] = {FRIC_BLEFT_MOTOR_SPEED_PID_KP, FRIC_BLEFT_MOTOR_SPEED_PID_KI, FRIC_BLEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bright_pid[3] = {FRIC_BRIGHT_MOTOR_SPEED_PID_KP, FRIC_BRIGHT_MOTOR_SPEED_PID_KI, FRIC_BRIGHT_MOTOR_SPEED_PID_KD};

    // ��ȡ�������ָ��
    shoot_control->shoot_data = get_shoot_data_point();
    // ��ȡң����ָ��
    shoot_control->shoot_rc = get_remote_control_point();
    // ��ȡ�����������ָ��
    shoot_control->trigger_motor_measure = get_trigger_motor_measure_point();
    shoot_control->fric_b_motor_measure = get_can_2006_measure_point();
    shoot_control->fric_left_motor_measure = get_can_3508_left_measure_point();
    shoot_control->fric_right_motor_measure = get_can_3508_right_measure_point();
    shoot_control->fric_bleft_motor_measure = get_can_3508_bleft_measure_point();
    shoot_control->fric_bright_motor_measure = get_can_3508_bright_measure_point();

    // ��ʼ�����������PID������
    PID_init(&shoot_control->trigger_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control->bullet_pid, PID_POSITION, fric_b_pid, FRIC_S_MOTOR_SPEED_PID_MAX_OUT, FRIC_S_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_left_pid, PID_POSITION, fric_left_pid, FRIC_LEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_LEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_right_pid, PID_POSITION, fric_right_pid, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_bleft_pid, PID_POSITION, fric_bleft_pid, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_bright_pid, PID_POSITION, fric_bright_pid, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_IOUT);

    // ��ʼ��������Ʋ���
    shoot_control->shoot_flag = 0;
    shoot_control->shoot_continu_flag = 0;
    shoot_control->stuck_flag = 0;
    shoot_control->reverse_time = 0;
    shoot_control->shoot_time = 150;
    shoot_control->last_bullet_number = shoot_control->bullet_number = 0;
    shoot_control->stuck_trigger_flag = 0;

    // ��ʼ������ٶȲ���
    shoot_control->trigger_given_current = 0;
    shoot_control->trigger_speed = 0.0f;
    shoot_control->trigger_speed_set = 0.0f;
    shoot_control->fric_b_speed = 0.0f;
    shoot_control->fric_b_speed_set = 0.0f;
    shoot_control->fric_left_speed = 0.0f;
    shoot_control->fric_left_speed_set = 0.0f;
    shoot_control->fric_right_speed = 0.0f;
    shoot_control->fric_right_speed_set = 0.0f;
    shoot_control->fric_bleft_speed = 0.0f;
    shoot_control->fric_bleft_speed_set = 0.0f;
    shoot_control->fric_bright_speed = 0.0f;
    shoot_control->fric_bright_speed_set = 0.0f;

    // ��ʼ�����PWM����
    shoot_control->pwm_first_up = PWM_FIRST_UP;
    shoot_control->pwm_first_down = PWM_FIRST_DOWN;
    shoot_control->pwm_mid_up = PWM_MID_UP;
    shoot_control->pwm_mid_down = PWM_MID_DOWN;
    shoot_control->pwm_last_up = PWM_LAST_UP;
    shoot_control->pwm_last_down = PWM_LAST_DOWN;

    // ��ʼ�������������������
    shoot_control->remain_heat = 0;
    shoot_control->remain_number = 0;
    shoot_control->last_remain_number = 0;
    shoot_control->remain_shoot_number = 0;

    // ��ʼ�������ǶȲ���
    shoot_control->trigger_angle = 0;
    shoot_control->trigger_angle_set = shoot_control->trigger_angle;

    // ��ʼ������ʱ����ٶȲ���
    shoot_control->turn_trigger_time = TURN_TRIGGER_TIME;
    shoot_control->turn_trigger_speed = TURN_TRIGGER_SPEED;
    shoot_control->first_speed = FIRST_SPEED;
    shoot_control->add_speed = ADD_SPEED;
}

/**
 * @brief          ͼ��̧�������������
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
void servo_pwm_control(shoot_control_t *shoot_control)
{
    // ���״̬��־
    static int8_t servo_flag = -100;
    // �������ʱ���¼
    static TickType_t servo_last_tick = 0;
    // ����״̬��¼
    static int16_t last_KEY_X = 0;
    static int16_t last_KEY_CTRL_C = 0;
    static int16_t last_KEY_CTRL_Z = 0;
    
    // CTRL+C:̧�� CTRL+Z:���� 
    if (!last_KEY_CTRL_C && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        shoot_control->pwm_mid_up -= 50;
    }
    else if (!last_KEY_CTRL_Z && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        shoot_control->pwm_mid_up += 50;
    }
    
    // ���pwm��ֵ
    if (servo_flag == -100)
    {
        // ���˶�״̬ˢ��pwmֵ,Ϊ����ϰ�������pwm ,��˳��Ϊ�˶�˳��
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_down); // �йؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_down); // �����ؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_down); // ����
        if (!last_KEY_X && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = 1;
        }
    }
    else if (servo_flag == 100)
    {
        // ��ȫ̧��״̬
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_up); // �����ؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_up); // �йؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_up); // ����
        if (!last_KEY_X && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = -1;
        }
    }

    /* ̧������ */
    if (servo_flag == 1)
    {
        // ��һ����̧�������ؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_up);
        servo_last_tick = xTaskGetTickCount();
        servo_flag = 2;
    }
    else if (servo_flag == 2)
    {
        // �ڶ������ȴ�150ms��̧���йؽںͱ���
        if (xTaskGetTickCount() - servo_last_tick > 150)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_up);
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_up);
            servo_flag = 100;
        }
    }

    /* �½����� */
    if (servo_flag == -1)
    {
        // ��һ�����½��йؽ�
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_down);
        servo_last_tick = xTaskGetTickCount();
        servo_flag = -2;
    }
    else if (servo_flag == -2)
    {
        // �ڶ������ȴ�220ms���½������ؽںͱ���
        if (xTaskGetTickCount() - servo_last_tick > 220)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_down);
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_down);
            servo_flag = -100;
        }
    }

    // ���°���״̬
    last_KEY_X = shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X;
    last_KEY_CTRL_C = (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
    last_KEY_CTRL_Z = (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
}

/**
 * @brief          ���ģʽ����
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_set_mode(shoot_control_t *shoot_control)
{
    // ����������ʱ��
    static uint16_t press_R_time = 0;
    static uint16_t CTRL_time = 0;
    fp32 fric_speed;

    // ����R���л�Ħ���ֿ���
    if (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_R && !(shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        press_R_time++;
    }
    if (press_R_time > 500)
    {
        R = !R;
        press_R_time = 0;
    }

    // Ħ���ֿ����������������ϻ�R������
    if ((switch_is_up(shoot_control->shoot_rc->rc.s[1]) || R))
    {
        STUCK = 1; // UI������1 ��������
        
        // �����жϼ��ز��߼�
        trigger_motor_block_control(shoot_control);

        if (shoot_control->stuck_flag == 0 && shoot_control->number_stuck_flag == 0) // ������ ��������������߼�
        {
            STUCK = 0; // ���������㿨��UI
            
            // ��������
            if (shoot_control->stuck_trigger_flag == 1)
            {
                trigger_motor(6.0f); //��ͨ������ʹ�ø��칩���ٶ�
            }
            else if (BUTTEN_TRIG_PIN == RESET) // �����͵�ƽ
            {
                trigger_motor(0.0f);
            }
            else if (BUTTEN_TRIG_PIN == SET) // Ĭ�ϸߵ�ƽ
            {
                trigger_motor(3.2f); // �����ٶ�
            }
            
            // ���ٹ����������־λ
            if(shoot_control->stuck_trigger_flag == 1 && BUTTEN_TRIG_PIN == RESET)
            {
                shoot_control->stuck_trigger_flag = 0;
            }
            
            // Ӧ������ ��ֹ�������
            if (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_R)
            {
                CTRL_time ++;
                if (CTRL_time > 200 && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL)
                {
                    trigger_motor(5.0f);
                }
            }
            else
            {
                CTRL_time = 0;
            }
            
            // ��¼�򵯸��� ͬʱ�ɻ���������1
            if (shoot_control->last_butten_trig_pin == RESET && BUTTEN_TRIG_PIN == SET)
            {
                shoot_control->bullet_number++;
                shoot_control->remain_shoot_number--;
                datatx_time = 0;
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
            datatx_time ++;
            if (datatx_time > 2147483645)
            {
                datatx_time = 11;
            }

            // ����ģʽʱ��shoot_flag����, �������������¿ŵ���
            if (switch_is_up(shoot_control->shoot_rc->rc.s[1]))
            {
                if (shoot_control->shoot_rc->rc.ch[4] < 120)
                {
                    shoot_control->shoot_flag = 0;
                }
            }
            else if (R)
            {
                if (shoot_control->press_l == 0)
                {
                    shoot_control->shoot_flag = 0;
                }
            }

            // �ֶ��������
            if (shoot_control->press_r) // �뿪�����ʱ��,�����ֶ�����(��Ϊ��̨����һֱ������ģʽ)
            {
                // ����ģʽ�µķ�������
                if ((shoot_control->shoot_flag == 0                                                                                         // ����1
                     && ((shoot_control->shoot_rc->rc.ch[4] > 600 || shoot_control->press_l)                                                 // ��������2:�ֶ�����������־
                         && (vision_control.shoot_vision_control.shoot_command == SHOOT_ATTACK && gimbal_behaviour == GIMBAL_AUTO_ATTACK)) // ��������2:���鴥��������־
                     && BUTTEN_TRIG_PIN == RESET)                                                                                          // ����3
                    && shoot_control->remain_shoot_number > 0)                                                                              // �������������
                {
                    shoot_control->shoot_flag = 1;
                    shoot_control->last_bullet_number = shoot_control->bullet_number;
                }
            }
            else
            {
                // �ֶ�ģʽ�µķ�������
                if ((shoot_control->shoot_flag == 0                                        // ����1
                     && (shoot_control->shoot_rc->rc.ch[4] > 600 || shoot_control->press_l) // ��������2:���鴥��������־
                     && BUTTEN_TRIG_PIN == RESET)                                         // ����3
                    /*&& shoot_control->remain_shoot_number > 0*/)                             // �������������
                {
                    shoot_control->shoot_flag = 1;
                    shoot_control->last_bullet_number = shoot_control->bullet_number;
                }
            }
        }
        // ��������״̬��
        else if (shoot_control->stuck_flag == 1 || shoot_control->number_stuck_flag == 1) // ����
        {
            third_fric(0);
            trigger_motor(-5.0f); // �����ز��ٶ�
        }
        else if (shoot_control->stuck_flag == 2 || shoot_control->number_stuck_flag == 2) // �����ز���ȴ�
        {
            third_fric(0);
            trigger_motor(0); // �����̵ȴ��ٶ�
        }
        else if (shoot_control->number_stuck_flag == 3)//���Ⲧ�����������ٲ�
        {
            trigger_motor(5.0f);
        }
        else if (shoot_control->number_stuck_flag == 4)//���Ⲧ�����Ĳ����ٲ�
        {
            trigger_motor(3.0f);
        }
    }
    else // Ħ���ֹر�״̬
    {
        laser_off();
        shoot_fric(0, 0);
        third_fric(0);
        trigger_motor(0);
    }

    // �����źŴ���
    if (shoot_control->last_shoot_flag == 0 && shoot_control->shoot_flag == 1)
    {
        shoot_control->shoot_time = 0;
    }

    // ��������
    if (shoot_control->shoot_time < shoot_control->turn_trigger_time) // ������ʱ��
    {
        trigger_motor(shoot_control->turn_trigger_speed); // �������ٶ�
    }
    else if (shoot_control->shoot_time == (shoot_control->turn_trigger_time))
    {
        trigger_motor(0);
    }

    // ���¼�ʱ����״̬
    shoot_control->shoot_time++;
    if (shoot_control->shoot_time >= 210)
        shoot_control->shoot_time = 210;
    
    // ����״̬��¼
    shoot_control->last_butten_trig_pin = BUTTEN_TRIG_PIN;
    shoot_control->last_shoot_flag = shoot_control->shoot_flag;
    shoot_control->last_remain_number = shoot_control->remain_number;
}

/**
 * @brief          �������ѭ�����������ٶ�
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_control_loop(shoot_control_t *shoot_control)
{
    // ���㲦�����PID���
    PID_calc(&shoot_control->trigger_pid, shoot_control->trigger_speed, shoot_control->trigger_speed_set);
    
    // �����ĸ�Ħ���ֵ����PID���
    PID_calc(&shoot_control->fric_left_pid, shoot_control->fric_left_speed, shoot_control->fric_left_speed_set);
    PID_calc(&shoot_control->fric_right_pid, shoot_control->fric_right_speed, shoot_control->fric_right_speed_set);
    PID_calc(&shoot_control->fric_bleft_pid, shoot_control->fric_bleft_speed, shoot_control->fric_bleft_speed_set);
    PID_calc(&shoot_control->fric_bright_pid, shoot_control->fric_bright_speed, shoot_control->fric_bright_speed_set);

    // ����CAN���͵���ֵ
    trigger_can_set_current = shoot_control->trigger_pid.out;
    left_can_set_current = shoot_control->fric_left_pid.out;
    right_can_set_current = shoot_control->fric_right_pid.out;
    bleft_can_set_current = shoot_control->fric_bleft_pid.out;
    bright_can_set_current = shoot_control->fric_bright_pid.out;
}

/**
 * @brief          ����������ݸ���
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void shoot_feedback_update(shoot_control_t *shoot_control)
{
    // �ٶ��˲�����
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    // ���׵�ͨ�˲�ϵ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    // �����ֵ���ٶ��˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control->trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control->trigger_speed = speed_fliter_3;

    // ���¸���������ٶ�����
    shoot_control->fric_b_speed = shoot_control->fric_b_motor_measure->speed_rpm;
    shoot_control->fric_left_speed = shoot_control->fric_left_motor_measure->speed_rpm;
    shoot_control->fric_right_speed = shoot_control->fric_right_motor_measure->speed_rpm;
    shoot_control->fric_bleft_speed = shoot_control->fric_bleft_motor_measure->speed_rpm;
    shoot_control->fric_bright_speed = shoot_control->fric_bright_motor_measure->speed_rpm;
    
    // ���´�������״̬
    shoot_control->key_pin = BUTTEN_TRIG_PIN;

    // ������갴��״̬
    shoot_control->last_press_l = shoot_control->press_l;
    shoot_control->last_press_r = shoot_control->press_r;
    shoot_control->press_l = shoot_control->shoot_rc->mouse.press_l;
    shoot_control->press_r = shoot_control->shoot_rc->mouse.press_r;

    // ���ݲ���ϵͳ���Ƶ���
    fric_speed = shoot_control->first_speed; // Ħ����ת�� 5800
    shoot_fric(fric_speed, shoot_control->add_speed);

    // ���ݰ����л���������ģʽ
    if (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C && !(shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        limit = 300;  // ����ģʽ�µ���������
        heat = 0;     // ����ģʽ�²���������
    }
    else
    {
        limit = robot_state.shooter_barrel_heat_limit;  // ����ģʽ�µ���������
        heat = power_heat_data.shooter_42mm_barrel_heat; // ��ǰ����ֵ
    }

    // ����ɻ�������, ��ֹ����ϵͳ�ӳٵ��³����� �������Ϊ�ɷ����־λ
    shoot_control->remain_heat = limit - heat;
    if (shoot_control->remain_heat < 100)
    {
        shoot_control->remain_number = 0;
    }
    else if (shoot_control->remain_heat >= 100 && shoot_control->remain_heat < 200)
    {
        shoot_control->remain_number = 1;
    }
    else if (shoot_control->remain_heat >= 200 && shoot_control->remain_heat < 300)
    {
        shoot_control->remain_number = 2;
    }
    else if (shoot_control->remain_heat >= 300 && shoot_control->remain_heat < 400)
    {
        shoot_control->remain_number = 3;
    }
    else
    {
        shoot_control->remain_number = 0;
    }

    // ÿ��һ��������ֵ,���¿ɻ�������
    if (shoot_control->last_remain_number != 1 && shoot_control->remain_number == 1)
    {
        shoot_control->remain_shoot_number = 1;
    }
    else if (shoot_control->last_remain_number != 2 && shoot_control->remain_number == 2)
    {
        shoot_control->remain_shoot_number = 2;
    }
    else if (shoot_control->last_remain_number != 3 && shoot_control->remain_number == 3)
    {
        shoot_control->remain_shoot_number = 3;
    }
    else if (shoot_control->last_remain_number != 0 && shoot_control->remain_number == 0)
    {
        shoot_control->remain_shoot_number = 0;
    }

    // ��������Ķ���ȴ�ʱ��
    if (shoot_control->shoot_time == (shoot_control->turn_trigger_time+13))
    {
        if(shoot_control->last_bullet_number == shoot_control->bullet_number)
        {
            shoot_control->number_stuck_flag = 1;
        }
    }
}

/**
 * @brief          �������̻ز�����
 * @param[in]      shoot_control: ������ƽṹ��ָ��
 * @retval         void
 */
static void trigger_motor_block_control(shoot_control_t *shoot_control)
{
    // ��������ʱ��
    static int16_t number_block_time = 0;
    
    // ���ݵ���ֵ��ʱ���ж��Ƿ񿨵�
    if (trigger_can_set_current > 4200.0f)
    {
        // �ж�1������ʱ������ȥ
        shoot_control->block_time++;
        if (shoot_control->block_time > 500) // ���ʱ��
        {
            shoot_control->stuck_flag = 1;
            shoot_control->block_time = 0;
        }
        
        // �ж�2��΢�������е����򲻳�ȥ(�й�������)
        if (trigger_can_set_current > 16000.0f)
        {
            number_block_time++;
            if (number_block_time > TURN_TRIGGER_TIME) // ���ʱ�� ���ݷ���ʱ������
            {
                if(BUTTEN_TRIG_PIN == RESET)
                {
                    shoot_control->number_stuck_flag = 1;
                    number_block_time = 0;
                }
            }
        }
    }
    else // ������ʱ���ü�ʱ��
    {
        shoot_control->block_time = 0;
        number_block_time = 0;
    }

    // �����ز�ʱ�䰲�ţ�2����
    if (shoot_control->stuck_flag == 1)
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 80) // �ز�ʱ��
        {
            shoot_control->reverse_time = 0;
            shoot_control->stuck_flag = 2;
        }
    }
    else if (shoot_control->stuck_flag == 2) // �ز���ȴ�������ز�����
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 80) // �ȴ�ʱ��
        {
            shoot_control->reverse_time = 0;
            shoot_control->stuck_flag = 0;
            shoot_control->stuck_trigger_flag = 1;
        }
    }

    // ���⿨���ز�״̬��
    if(shoot_control->number_stuck_flag == 1)
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 50)
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 2;
        }
    }
    else if (shoot_control->number_stuck_flag == 2) // �ز���ȴ�������ز�����
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 70) // �ȴ�ʱ��
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 3;
        }
    }
    else if (shoot_control->number_stuck_flag == 3) // ���ٲ���
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 80) // ���ϵ�ʱ��
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 4;
        }
    }
    else if (shoot_control->number_stuck_flag == 4) // ��������
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 40) // �ϵ�ʱ��
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 0;
        }
    }
}

// ... existing code ... 
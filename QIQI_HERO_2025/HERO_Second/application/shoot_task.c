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

#define FIRST_SPEED 5300 
#define ADD_SPEED 300

#define trigger_motor(speed) shoot_control.trigger_speed_set = speed // �����������
#define third_fric(speed) shoot_control.fric_b_speed_set = -speed    // ���������������
// �г̿���IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)
#define TURN_TRIGGER_TIME 25  // ����ʱ��
#define TURN_TRIGGER_SPEED 40 // �����ٶ�


int16_t limitnb = 0, a = 0, hh = 0;
extern gimbal_control_t gimbal_control;
shoot_control_t shoot_control; // �������
int16_t s_can_set_current = 0, left_can_set_current = 0, right_can_set_current = 0, bleft_can_set_current = 0, bright_can_set_current = 0, trigger_can_set_current = 0;
fp32 last_speed;
uint8_t shoot_allow_flag = 0, success_flag = 0;
int16_t tim = 0;
fp32 q, w, e, r;
extern int8_t STUCK;
extern TickType_t xTickCount;

/**
 * @brief          �������
 * @param[in]      void-
 * @retval         ����can����ֵ
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
}


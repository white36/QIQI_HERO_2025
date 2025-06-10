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
// 云台控制所有相关数据
gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern int16_t trigger_can_set_current;
extern shoot_control_t shoot_control;
extern ExtY_stm32 stm32_Y_yaw;
extern ExtY_stm32 stm32_Y_pitch;
// 视觉任务结构体
extern vision_control_t vision_control;
////视觉发送任务结构体
// extern vision_send_t vision_send;
////视觉数据
vision_rxfifo_t *vision_rx;
// UI
extern can_feedback_a_typedef get_capA;
extern int16_t R;
//extern int16_t turn_flags;
extern int16_t anglesr;
extern int16_t angle_sin, angle_cos;
extern float angle_radto;
int16_t pitch_dian = 2000;
extern int8_t AUTO_ATTACK; // 自瞄标志位
int hipnuc_flag = 0; // 陀螺仪解码成功flag
int last_hipnuc_flag = 0;
TickType_t xTickCount; // 获取freertos系统时钟滴答数

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
}


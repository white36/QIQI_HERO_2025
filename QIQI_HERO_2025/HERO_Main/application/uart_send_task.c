#include "uart_send_task.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "CAN_comm.h"

//VOFA+发送结构体
SEND_Message send_message;
extern vision_control_t vision_control;
extern shoot_control_t shoot_control;
extern int8_t datatx_OK;

void UART_Send_feedback_update(SEND_Message *sendMessage);

/**
  * @brief          vofa发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void UART_Send_Task(void const * argument)
{

    while (1)
    {
        // 发送数据更新
        UART_Send_feedback_update(&send_message);
        // Vofa发送
        vofa_start(&send_message);
        osDelay(1);
    }
}


void UART_Send_feedback_update(SEND_Message *sendMessage)
{
    // Vofa数据
    send_message.v0 = datatx_OK;
	send_message.v1 = shoot_control.first_speed;
	send_message.v2 = shoot_control.shoot_data->initial_speed;
    send_message.v3 = shoot_control.fric_left_motor_measure->speed_rpm;
    send_message.v4 = shoot_control.fric_right_motor_measure->speed_rpm;
    send_message.v5 = shoot_control.fric_bleft_motor_measure->speed_rpm;
	send_message.v6 = shoot_control.fric_bright_motor_measure->speed_rpm;
	send_message.v7 = shoot_control.trigger_speed;
	send_message.v8 = shoot_control.key_pin;
}



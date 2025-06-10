
#include "uart_send_task.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "CAN_comm.h"

//VOFA+���ͽṹ��
SEND_Message send_message;
extern vision_control_t vision_control;
extern shoot_control_t shoot_control;

void UART_Send_feedback_update(SEND_Message *sendMessage);

/**
  * @brief          led RGB����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void UART_Send_Task(void const * argument)
{

    while (1)
    {
        UART_Send_feedback_update(&send_message);
        vofa_start(&send_message);
        osDelay(1);
    }
}


void UART_Send_feedback_update(SEND_Message *sendMessage)
{
//    send_message.v0 = ;
	send_message.v1 = shoot_control.first_speed;
	send_message.v2 = vision_control.bullet_speed;
}



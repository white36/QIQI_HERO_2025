/**
 * @file can_comm_task.h
 * @author yuanluochen
 * @brief can�豸ͨ���������ö���ʵ��can���ݶ��з���
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H 
#include "can_comm.h"
#include "can.h"

//canͨ�������ʼ��ʱ�� ��λms
#define CAN_COMM_TASK_INIT_TIME 100
//canͨ����������ʱ���� ��λms
#define CAN_COMM_TASK_TIME 1
////��̨can�豸
//#define GIMBAL_CAN hcan1
////˫��canͨ���豸
//#define BOARD_CAN hcan1
////����canͨ���豸
//#define SHOOT_CAN hcan1
////����ϵͳcanͨ��
//#define REFEREE_CAN hcan1
//typedef enum
////{
////	
////		CAP_ID = 0x211,
////	  CAN_UI=0x212,
////		CAN_TUI=0x213,

////} can_msg_id_e;


//canͨ������ṹ��
typedef struct
{
    //canͨ�Ŷ��нṹ��
    can_comm_queue_t *can_comm_queue;
    
}can_comm_task_t;


/**
 * @brief  canͨ������
 * 
 */
void can_comm_task(void const* pvParameters);



/**
 * @brief ˫��ͨ�����ݷ��ͣ���̨���Ƶ��̣���������ӵ�can_comm�߳�ͨ�Ŷ�����
 * 
 * @param relative_angle ��̨��Խ�
 * @param chassis_vx ����x���ٶȷ������
 * @param chassis_vy ����y���ٶȷ������
 * @param chassis_behaviour �����˶�ģʽ
 */
void can_comm_UIT(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour);

/**
 * @brief �������ͨ�����ݷ��ͣ�����ֵΪ�������ֵ��������������ӵ�can_comm�̵߳�ͨ�Ŷ�����
 * 
 * @param fric1 Ħ���ֵ������ֵ
 * @param fric2 Ħ���ֵ������ֵ
 * @param trigger �����̵������ֵ
 */

void can_comm_gimbal(int16_t KEy_1, int16_t KEy_2, int16_t KEy_3 ,int16_t KEy_4);
void can_comm_UIO(int16_t KEY_1, int16_t KEY_2, int16_t KEY_3 ,int16_t KEY_4);

void can_comm_supercap(uint8_t temPower);

bool can_comm_task_init_finish(void);

#endif // !CAN_COMM_TASK_H

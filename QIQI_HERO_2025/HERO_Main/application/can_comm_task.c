/**
 * @file can_comm_task.c
 * @author yuanluochen
 * @brief can�豸ͨ���������ö���ʵ��can����˳����
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "can_comm_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "crc8_crc16.h"
#include "CAN_receive.h"
#define  UIT_CAN hcan1
#define  UIO_CAN hcan1
#define  CAP     hcan1
/**
 * @brief canͨ���̳߳�ʼ��, ��Ҫ����Ϊ�����̶߳���
 * 
 * @param can_comm_init canͨ���̳߳�ʼ���ṹ��
 */
static void can_comm_task_init(can_comm_task_t *can_comm_init);

/**
 * @brief canͨ�������ͺ�����ͨ�����ݶ��з��� 
 * 
 * @param can_comm_transmit canͨ��������ƽṹ��
 */
static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit);

/**
 * @brief canͨ�Ŷ�����Ӻ��� 
 * 
 * @param add_comm_queue canͨ��������
 * @param comm_data canͨ������
 */
static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data);

/**
 * @brief canͨ�Ŷ������ݸ���
 * 
 * @param feedback_update canͨ�Ŷ��нṹ��
 */
static void can_comm_feedback_update(can_comm_task_t *feedback_update);

//��̨canͨ������
//static can_comm_data_t gimbal_can_comm_data = {
//    .can_handle = &GIMBAL_CAN, // ��ʼ����̨ͨ���豸can
//    .can_comm_target = CAN_COMM_GIMBAL,
//};

//˫��canͨ������
static can_comm_data_t board_can_comm_data = {
    .can_handle = &UIT_CAN, // ��ʼ��˫��ͨ���豸can
    .can_comm_target = CAN_COMM_UIT,
};

//����canͨ������
static can_comm_data_t shoot_can_comm_data = {
    .can_handle = &UIO_CAN, // ��ʼ������ͨ���豸can
    .can_comm_target = CAN_COMM_UIO,
};

//����ϵͳͨ������
//static can_comm_data_t cap_data = {
//    .can_handle = &CAP, // ��ʼ������ϵͳͨ���豸can
//    .can_comm_target = CAN_COMM_CAP,
//};

bool init_finish = false;

//ʵ����canͨ���߳̽ṹ��,ȫ�ֱ�������֤����һֱ����
can_comm_task_t can_comm = { 0 };


void can_comm_task(void const* pvParameters)
{ 
    //Ҫ����̨�͵�������ʼ֮ǰ��ɸ�����ĳ�ʼ��
    vTaskDelay(CAN_COMM_TASK_INIT_TIME);
    //canͨ�������ʼ��
    can_comm_task_init(&can_comm);
    init_finish = true;
    while(1)
    {
        //canͨ�Ų�������
        can_comm_feedback_update(&can_comm);
        //canͨ�����ݷ���
        can_comm_task_transmit(&can_comm);
        //can�������ݷ��� 
        vTaskDelay(CAN_COMM_TASK_TIME);
    }
}

static void can_comm_feedback_update(can_comm_task_t *feedback_update)
{
    feedback_update->can_comm_queue->size = can_comm_queue_size(feedback_update->can_comm_queue);
}

static void can_comm_task_init(can_comm_task_t *can_comm_init)
{
    if (can_comm_init == NULL)
        return; 
    //��������ʼ��canͨ�Ŷ���
    can_comm_init->can_comm_queue = can_comm_queue_init(); 
}

static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit)
{
    if (can_comm_transmit == NULL)
        return;
    //���зǿշ���
    if (!can_comm_queue_is_empty(can_comm_transmit->can_comm_queue))
    {
        can_comm_data_t *data = can_comm_queue_pop(can_comm_transmit->can_comm_queue);
        if(data)
        {
            can_transmit(data);
        }
    }
}


static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data)
{
    if (add_comm_queue == NULL || comm_data == NULL)
        return;
    //������ݵ����Ͷ�����
    can_comm_queue_push(add_comm_queue->can_comm_queue, comm_data);
}


//void can_comm_gimbal(int16_t KEy_1, int16_t KEy_2, int16_t KEy_3 ,int16_t KEy_4)
//{
//    //����can��������
//    gimbal_can_comm_data.transmit_message.StdId = 0x1FF;
//    gimbal_can_comm_data.transmit_message.IDE = CAN_ID_STD;
//    gimbal_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
//    gimbal_can_comm_data.transmit_message.DLC = 0x08;
//    gimbal_can_comm_data.data[0] =(KEy_1 >> 8);
//    gimbal_can_comm_data.data[1] = KEy_1;
//    gimbal_can_comm_data.data[2] = (KEy_2 >> 8);
//    gimbal_can_comm_data.data[3] = KEy_2;
//    gimbal_can_comm_data.data[4] = (KEy_3 >> 8);
//    gimbal_can_comm_data.data[5] = KEy_3;
//    gimbal_can_comm_data.data[6] = (KEy_4 >> 8);
//    gimbal_can_comm_data.data[7] = KEy_4;
//    //������ݵ�ͨ�Ŷ���
//    add_can_comm_queue(&can_comm, &gimbal_can_comm_data);
//}

void can_comm_UIT(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour)
{
    //����can��������
    board_can_comm_data.transmit_message.StdId = 0x209;
    board_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    board_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    board_can_comm_data.transmit_message.DLC = 0x08;
    board_can_comm_data.data[0] = (relative_angle >> 8);
    board_can_comm_data.data[1] = relative_angle;
    board_can_comm_data.data[2] = (chassis_vx >> 8);
    board_can_comm_data.data[3] = chassis_vx;
    board_can_comm_data.data[4] = (chassis_vy >> 8);
    board_can_comm_data.data[5] = chassis_vy;
    board_can_comm_data.data[6] = (chassis_behaviour >> 8);
    board_can_comm_data.data[7] = chassis_behaviour;
    //������ݵ�ͨ�Ŷ���
    add_can_comm_queue(&can_comm, &board_can_comm_data);
}

void can_comm_UIO(int16_t KEY_1, int16_t KEY_2, int16_t KEY_3 ,int16_t KEY_4)
{
    //����can��������
    shoot_can_comm_data.transmit_message.StdId = 0x208;
    shoot_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    shoot_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    shoot_can_comm_data.transmit_message.DLC = 0x08;
    shoot_can_comm_data.data[0] = (KEY_1 >> 8);
    shoot_can_comm_data.data[1] = KEY_1;
    shoot_can_comm_data.data[2] = (KEY_2 >> 8);
    shoot_can_comm_data.data[3] = KEY_2;
    shoot_can_comm_data.data[4] = (KEY_3 >> 8);
    shoot_can_comm_data.data[5] = KEY_3;
    shoot_can_comm_data.data[6] = (KEY_4 >> 8);
    shoot_can_comm_data.data[7] = KEY_4;
    //������ݵ�ͨ�Ŷ���
    add_can_comm_queue(&can_comm, &shoot_can_comm_data); 
}

//void can_comm_supercap(uint8_t temPower )
//{
//	can_control_typedef send_cap_data = {0};
////	  uint32_t send_mail_box;
//    cap_data.transmit_message.StdId = 0x21;
//    cap_data.transmit_message.IDE = CAN_ID_STD;
//    cap_data.transmit_message.RTR = CAN_RTR_DATA;
//    cap_data.transmit_message.DLC = 0x08;
//	
//	send_cap_data.p_set = temPower;
//	send_cap_data.power_source = CAPACITY;
//	send_cap_data.out_auto_en = 1;
//	send_cap_data.wireless_en = 1;
//	send_cap_data.freq_feedback = 100;
//	
//	append_CRC8_check_sum((uint8_t *) &send_cap_data,sizeof(send_cap_data));
//	memcpy(cap_data.data,&send_cap_data,8);
//	add_can_comm_queue(&can_comm, &cap_data); 
//}



bool can_comm_task_init_finish(void)
{
    return init_finish;
}


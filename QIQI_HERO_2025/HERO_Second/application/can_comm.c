/**
 * @file can_comm.c
 * @author yuanluochen
 * @brief ���豸ͨ��ģ�飬��Ҫ���ڿ��ư�֮���ͨ�ţ�ʹ��can����ʵ��
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "can_comm.h"
#include "can.h"
#include "main.h"
#include "CRC8_CRC16.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static CAN_TxHeaderTypeDef comm_tx_message_a;
static uint8_t comm_can_send_a[8];

/**
 * @brief          ���ṹ����Ϊ CAN ֡���ݣ�8 �ֽڣ�
 * @param[in]      input_struct: ָ�������Ľṹ��ָ�룬�ṹ����Ϊ __packed__ �Ҳ����� 8 �ֽ�
 * @param[out]     output_buffer: ��Ŵ���������
 * @param[in]      struct_size: �ṹ��ʵ���ֽڳ��ȣ���󲻳��� 8
 * @retval         none
 */
void pack_any_8byte_struct(const void *input_struct, uint8_t *output_buffer, size_t struct_size)
{
    if (struct_size > 8)
        struct_size = 8;
    // �����ͽṹ�忽����can����֡��
    memcpy(output_buffer, input_struct, struct_size);
}

/**
 * @brief          �����յ��� 8 �ֽ� CAN ���ݽ��Ϊָ���ṹ��
 * @param[in]      input_buffer: ָ����ջ������� 8 �ֽ�����
 * @param[out]     output_struct: ָ��Ҫд��Ľṹ��ָ��
 * @param[in]      struct_size: Ŀ��ṹ����ֽڴ�С������ܳ��� 8
 * @retval         none
 *
 * @note           �ṹ�����ȷ���뷢�Ͷ˽ṹ��һ��
 */
void unpack_any_8byte_struct(const uint8_t *input_buffer, void *output_struct, size_t struct_size)
{
    if (struct_size > 8)
        struct_size = 8;
    // �����յ�������֡���������սṹ����
    memcpy(output_struct, input_buffer, struct_size);
}

void CAN_comm_up(uint8_t *out_data)
{
    uint32_t send_mail_box;
    comm_tx_message_a.StdId = CAN_COMM_UP_ID;
    comm_tx_message_a.IDE = CAN_ID_STD;
    comm_tx_message_a.RTR = CAN_RTR_DATA;
    comm_tx_message_a.DLC = 0x08;
    comm_can_send_a[0] = out_data[0];
    comm_can_send_a[1] = out_data[1];
    comm_can_send_a[2] = out_data[2];
    comm_can_send_a[3] = out_data[3];
    comm_can_send_a[4] = out_data[4];
    comm_can_send_a[5] = out_data[5];
    comm_can_send_a[6] = out_data[6];
    comm_can_send_a[7] = out_data[7];

    HAL_CAN_AddTxMessage(&COMM_CAN, &comm_tx_message_a, comm_can_send_a, &send_mail_box);
}

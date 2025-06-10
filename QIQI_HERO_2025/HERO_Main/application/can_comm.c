/**
 * @file can_comm.c
 * @author yuanluochen
 * @brief 多设备通信模块，主要用于控制板之间的通信，使用can总线实现
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

static CAN_TxHeaderTypeDef comm_down_tx_message_A;
static uint8_t comm_down_can_send_data_A[8];
static CAN_TxHeaderTypeDef comm_down_tx_message_B;
static uint8_t comm_down_can_send_data_B[8];
static CAN_TxHeaderTypeDef comm_down_tx_message_C;
static uint8_t comm_down_can_send_data_C[8];

/**
 * @brief          将结构体打包为 CAN 帧数据（8 字节）
 * @param[in]      input_struct: 指向待打包的结构体指针，结构体需为 __packed__ 且不超过 8 字节
 * @param[out]     output_buffer: 存放打包后的数据
 * @param[in]      struct_size: 结构体实际字节长度，最大不超过 8
 * @retval         none
 */
void pack_any_8byte_struct(const void *input_struct, uint8_t *output_buffer, size_t struct_size)
{
    memcpy(output_buffer, input_struct, struct_size);
}

/**
 * @brief          将接收到的 8 字节 CAN 数据解包为指定结构体
 * @param[in]      input_buffer: 指向接收缓冲区的 8 字节数据
 * @param[out]     output_struct: 指向要写入的结构体指针
 * @param[in]      struct_size: 目标结构体的字节大小，最大不能超过 8
 * @retval         none
 *
 * @note           结构体必须确保与发送端结构体一致
 */
void unpack_any_8byte_struct(const uint8_t *input_buffer, void *output_struct, size_t struct_size)
{
    if (struct_size > 8)
        struct_size = 8;
    // 将接收到的数据帧拷贝到接收结构体里
    memcpy(output_struct, input_buffer, struct_size);
}

void CAN_comm_down_A(uint8_t *out_data)
{
    uint32_t send_mail_box;
    comm_down_tx_message_A.StdId = CAN_COMM_DOWN_ID_A;
    comm_down_tx_message_A.IDE = CAN_ID_STD;
    comm_down_tx_message_A.RTR = CAN_RTR_DATA;
    comm_down_tx_message_A.DLC = 0x08;
    comm_down_can_send_data_A[0] = out_data[0];
    comm_down_can_send_data_A[1] = out_data[1];
    comm_down_can_send_data_A[2] = out_data[2];
    comm_down_can_send_data_A[3] = out_data[3];
    comm_down_can_send_data_A[4] = out_data[4];
    comm_down_can_send_data_A[5] = out_data[5];
    comm_down_can_send_data_A[6] = out_data[6];
    comm_down_can_send_data_A[7] = out_data[7];

    HAL_CAN_AddTxMessage(&COMM_CAN, &comm_down_tx_message_A, comm_down_can_send_data_A, &send_mail_box);
}

void CAN_comm_down_B(uint8_t *out_data)
{
    uint32_t send_mail_box;
    comm_down_tx_message_B.StdId = CAN_COMM_DOWN_ID_B;
    comm_down_tx_message_B.IDE = CAN_ID_STD;
    comm_down_tx_message_B.RTR = CAN_RTR_DATA;
    comm_down_tx_message_B.DLC = 0x08;
    comm_down_can_send_data_B[0] = out_data[0];
    comm_down_can_send_data_B[1] = out_data[1];
    comm_down_can_send_data_B[2] = out_data[2];
    comm_down_can_send_data_B[3] = out_data[3];
    comm_down_can_send_data_B[4] = out_data[4];
    comm_down_can_send_data_B[5] = out_data[5];
    comm_down_can_send_data_B[6] = out_data[6];
    comm_down_can_send_data_B[7] = out_data[7];

    HAL_CAN_AddTxMessage(&COMM_CAN, &comm_down_tx_message_B, comm_down_can_send_data_B, &send_mail_box);
}

void CAN_comm_down_C(uint8_t *out_data)
{
    uint32_t send_mail_box;
    comm_down_tx_message_C.StdId = CAN_COMM_DOWN_ID_C;
    comm_down_tx_message_C.IDE = CAN_ID_STD;
    comm_down_tx_message_C.RTR = CAN_RTR_DATA;
    comm_down_tx_message_C.DLC = 0x08;
    comm_down_can_send_data_C[0] = out_data[0];
    comm_down_can_send_data_C[1] = out_data[1];
    comm_down_can_send_data_C[2] = out_data[2];
    comm_down_can_send_data_C[3] = out_data[3];
    comm_down_can_send_data_C[4] = out_data[4];
    comm_down_can_send_data_C[5] = out_data[5];
    comm_down_can_send_data_C[6] = out_data[6];
    comm_down_can_send_data_C[7] = out_data[7];

    HAL_CAN_AddTxMessage(&COMM_CAN, &comm_down_tx_message_C, comm_down_can_send_data_C, &send_mail_box);
}

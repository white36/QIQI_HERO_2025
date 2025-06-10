/**
 * @file can_comm.h
 * @author yuanluochen
 * @brief ���豸ͨ��ģ�飬��Ҫ���ڿ��ư�֮���ͨ�ţ�ʹ��can����ʵ�֣���������ʵ��
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CAN_PACKEET_CONNECTION_H
#define CAN_PACKEET_CONNECTION_H

#include "struct_typedef.h"
#include "main.h"

#define COMM_CAN hcan1

// ������亯���궨��
#define MAX_BITS 64
#define MAX_DATA_ITEMS 20

#define COMM_FLAG_Z ((uint8_t)1 << 7)
#define COMM_FLAG_Q ((uint8_t)1 << 6)

#define COMM_FLAG_TURN_FLAG ((uint8_t)1 << 7)
#define COMM_FLAG_KEY_C ((uint8_t)1 << 6)
#define COMM_FLAG_GIMBAL_MODE ((uint8_t)1 << 6)

#define COMM_FLAG_SHIFT ((uint8_t)1 << 0)
#define COMM_FLAG_R ((uint8_t)1 << 1)
#define COMM_FLAG_QA ((uint8_t)1 << 2)
#define COMM_FLAG_BPIN ((uint8_t)1 << 3)
#define COMM_FLAG_AUTO_ATTACK ((uint8_t)1 << 4)
#define COMM_FLAG_STUCK ((uint8_t)1 << 5)
#define COMM_FLAG_VISION ((uint8_t)1 << 6)
#define COMM_FLAG_DBUS_TOE ((uint8_t)1 << 7)

typedef enum
{
    // �ϰ�����ID
    CAN_COMM_DOWN_ID_A = 0x520,
    CAN_COMM_DOWN_ID_B = 0x521,
    CAN_COMM_DOWN_ID_C = 0x522,

    // �°�����ID
    CAN_COMM_UP_ID = 0x620,

} can_comm_id_e;

// ����
#define PACK_STRUCT_TO_CAN_BUFFER(input_struct, buffer) \
    pack_any_8byte_struct(&(input_struct), (buffer), sizeof(input_struct))
// ���
#define UNPACK_CAN_STRUCT(buf, struct_var) \
    unpack_any_8byte_struct((buf), &(struct_var), sizeof(struct_var))

// ����������
extern void pack_any_8byte_struct(const void *input_struct, uint8_t *output_buffer, size_t struct_size);
extern void unpack_any_8byte_struct(const uint8_t *input_buffer, void *output_struct, size_t struct_size);

extern void CAN_comm_up(uint8_t *out_data);

#endif

#ifndef WEIGHT_DATA_H
#define WEIGHT_DATA_H

#include "bsp_hipnuc.h"
#include "string.h"
// ʹ���ź���
#include "FreeRTOS.h"
#include "semphr.h"

#define HIPNUC_RX_BUF_NUM 256u
// HI91 example data
#define HIPNUC_FRAME_LENGTH 84u

extern int hipnuc_data_flag; //���ݽ��ճɹ���־λ
extern uint16_t uart_rx_index; //��¼��ǰDMA�������ݳ���
extern uint8_t *current_decode_buf; // ȫ�ֱ�����ָ��ǰҪ����Ļ�����
extern SemaphoreHandle_t hipnuc_decode_semaphore; // �ź���

extern void hipnuc_data_IRQ(void);
extern void hipnuc_rx_init(void);

#endif

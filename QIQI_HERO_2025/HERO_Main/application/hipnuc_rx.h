#ifndef WEIGHT_DATA_H
#define WEIGHT_DATA_H

#include "bsp_hipnuc.h"
#include "string.h"
// 使用信号量
#include "FreeRTOS.h"
#include "semphr.h"

#define HIPNUC_RX_BUF_NUM 256u
// HI91 example data
#define HIPNUC_FRAME_LENGTH 84u

extern int hipnuc_data_flag; //数据接收成功标志位
extern uint16_t uart_rx_index; //记录当前DMA接收数据长度
extern uint8_t *current_decode_buf; // 全局变量，指向当前要解码的缓冲区
extern SemaphoreHandle_t hipnuc_decode_semaphore; // 信号量

extern void hipnuc_data_IRQ(void);
extern void hipnuc_rx_init(void);

#endif

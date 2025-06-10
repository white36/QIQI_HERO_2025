#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))

typedef struct
{
    float v0;
    float v1;
    float v2;
    float v3;
    float v4;
    float v5;
    float v6;
    float v7;
    float v8;
    float v9;
    float v10;
    float v11;
    float v12;
    float v13;
    float v14;
    float v15;
}SEND_Message;


void vofa_start(SEND_Message *send_message);
void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);

#endif /* __VOFA_H__ */















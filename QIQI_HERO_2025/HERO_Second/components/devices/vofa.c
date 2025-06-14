#include "vofa.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "math.h"
#include "usart.h"

#define MAX_BUFFER_SIZE 1024
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

/**
***********************************************************************
* @brief:      vofa_start(void)
* @param:		   void
* @retval:     void
* @details:    发送数据给上位机
***********************************************************************
**/
void vofa_start(SEND_Message *send_message)
{
    // Call the function to store the data in the buffer
    vofa_send_data(0, send_message->v0);

    // Call the function to send the frame tail
    vofa_sendframetail();
}

///**
//***********************************************************************
//* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
//* @param:		void
//* @retval:     void
//* @details:    USB
//***********************************************************************
//**/
//void vofa_transmit_USB(uint8_t* buf, uint16_t len)
//{
//    CDC_Transmit_HS((uint8_t *)buf, len);
//}
/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		void
* @retval:     void
* @details:    USART
***********************************************************************
**/
void vofa_transmit_USART(uint8_t* buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 0xFFFF);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void vofa_send_data(uint8_t num, float data)
{
    send_buf[cnt++] = byte0(data);
    send_buf[cnt++] = byte1(data);
    send_buf[cnt++] = byte2(data);
    send_buf[cnt++] = byte3(data);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void vofa_sendframetail(void)
{
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;

    /* 将数据和帧尾打包发送 */
    vofa_transmit_USART((uint8_t *)send_buf, cnt);
    cnt = 0;// 每次发送完帧尾都需要清零
}














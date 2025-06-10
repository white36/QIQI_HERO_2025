/**
 ****************************(C) 版权所有 2025 none****************************
 * @file       hipnuc_data.c
 * @brief      HiPNuc数据接收模块
 *             处理HiPNuc重量数据接收，
 *             使用串口（USART1）双缓冲DMA进行高效数据接收。
 *
 * @details    实现功能：
 *             - HiPNuc通信的接收函数
 *             - 双缓冲数据接收以提高性能（防止占用过多CPU时间）
 *
 * @note       数据解码通过信号量与FreeRTOS任务同步完成，防止中断服务程序占用过多资源。
 *
 * @history    版本        日期            作者           修改内容
 *             V1.0.0     2025-1-10       Bai Shuhao     DMA双缓冲区接收陀螺仪数据，解码见商家封装文件hipnuc_dec.c
 *
 * @verbatim
 * ==============================================================================
 *  通信协议详情（基于Modbus标准协议）：
 *  - 数据帧结构：
 *      - 帧头: 0x5A 0xA5
 *      - 帧长度: 2字节（小端模式）
 *      - 数据: 变长（有效载荷，包含加速度、角速度、磁力计等信息）
 *      - CRC校验: 2字节
 *  - CRC校验算法: Modbus标准CRC16（多项式0x8005）
 *  - 示例帧数据 (十六进制格式)：
 *      0x5A, 0xA5,     (header)
 *      0x4C, 0x00,     (len)
 *      0x14, 0xBB,     (CRC)
 *      0x91,           (package tag )
 *      0x08, 0x15, 0x23, 0x09, 0xA2, 0xC4, 0x47, 0x08, 0x15,   (data)
 *      0x1C, 0x00, 0xCC, 0xE8, 0x61, 0xBE, 0x9A, 0x35, 0x56, 0x3E, 0x65, 0xEA, 0x72, 0x3F, 0x31, 0xD0,
 *      0x7C, 0xBD, 0x75, 0xDD, 0xC5, 0xBB, 0x6B, 0xD7, 0x24, 0xBC, 0x89, 0x88, 0xFC, 0x40, 0x01, 0x00,
 *      0x6A, 0x41, 0xAB, 0x2A, 0x70, 0xC2, 0x96, 0xD4, 0x50, 0x41, 0xED, 0x03, 0x43, 0x41, 0x41, 0xF4,
 *      0xF4, 0xC2, 0xCC, 0xCA, 0xF8, 0xBE, 0x73, 0x6A, 0x19, 0xBE, 0xF0, 0x00, 0x1C, 0x3D, 0x8D, 0x37,
 *      0x5C, 0x3F
 * ==============================================================================
 * @endverbatim
 ****************************(C) 版权所有 none****************************
 */

#include "hipnuc_rx.h"

#include "main.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/*************************************************** 变量定义 **********************************************************/
// 接收原始数据，双缓冲区定义
static uint8_t hipnuc_rx_buf[2][HIPNUC_RX_BUF_NUM];

int hipnuc_data_flag = 0;   //数据接收成功标志位
uint16_t uart_rx_index = 0; //记录当前DMA接收数据长度
uint8_t *current_decode_buf;               // 全局变量，指向当前要解码的缓冲区
SemaphoreHandle_t hipnuc_decode_semaphore; // 信号量

/*************************************************** 函数声明 ***********************************************************/
/**
 * @brief          解析特定格式的HiPNuc重量数据（USART1版本）
 * @details        使用USART1的中断机制和DMA双缓冲区接收HiPNuc数据帧，解析并校验接收到的数据帧。
 *                 在数据长度符合预期并且CRC校验通过的情况下，触发信号量以通知后续处理任务。
 * @note           1. 函数假设数据帧使用特定的HiPNuc协议格式。
 *                 2. 双缓冲区用于连续接收数据，防止数据丢失。
 *                 3. CRC校验确保数据完整性。
 */
void hipnuc_data_IRQ(void);

/**
  * @brief          陀螺仪接收初始化
  * @param[in]      none
  * @retval         none
  */
void hipnuc_rx_init(void);

/****************************************************函数内容******************************************************/
//hipnuc_decode_semaphore = xSemaphoreCreateBinary();
/**
 * @brief          解析特定格式的HiPNuc重量数据（USART1版本）
 * @details        使用USART1的中断机制和DMA双缓冲区接收HiPNuc数据帧，解析并校验接收到的数据帧。
 *                 在数据长度符合预期并且CRC校验通过的情况下，触发信号量以通知后续处理任务。
 * @note           1. 函数假设数据帧使用特定的HiPNuc协议格式。
 *                 2. 双缓冲区用于连续接收数据，防止数据丢失。
 *                 3. CRC校验确保数据完整性。
 */
void hipnuc_data_IRQ(void)
{
	if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    // 检查USART1的空闲中断标志位
    if (huart1.Instance->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0; // 记录本次接收的数据长度

        // 清除空闲中断标志位
        // 执行一个"读操作"：先读取SR（状态寄存器），再读取DR（数据寄存器）
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        // 判断当前使用的缓冲区（DMA双缓冲机制）
        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* 当前使用内存缓冲区0 */

            // 关闭DMA以防止数据传输冲突
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // 获取接收的数据长度
            this_time_rx_len = HIPNUC_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // 重置DMA的数据传输计数寄存器（准备接收新数据）
            hdma_usart1_rx.Instance->NDTR = HIPNUC_RX_BUF_NUM;

            // 切换到缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            // 重新使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            // 处理接收到的数据
			current_decode_buf = hipnuc_rx_buf[0];
			uart_rx_index = this_time_rx_len;

        }
        else
        {
            /* 当前使用内存缓冲区1 */

            // 关闭DMA以防止数据传输冲突
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // 获取接收的数据长度
            this_time_rx_len = HIPNUC_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // 重置DMA的数据传输计数寄存器（准备接收新数据）
            hdma_usart1_rx.Instance->NDTR = HIPNUC_RX_BUF_NUM;

            // 切换到缓冲区0
            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);

            // 重新使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            // 处理接收到的数据
			current_decode_buf = hipnuc_rx_buf[1];
			uart_rx_index = this_time_rx_len;
        }
    }
}

/**
  * @brief          陀螺仪接收初始化
  * @param[in]      none
  * @retval         none
  */
void hipnuc_rx_init(void)
{
    /* 初始化DMA双缓冲接收 */
	HIPNUC_init(hipnuc_rx_buf[0],    // 第一缓冲区
                hipnuc_rx_buf[1],    // 第二缓冲区
                HIPNUC_RX_BUF_NUM);  //缓冲区长度
}


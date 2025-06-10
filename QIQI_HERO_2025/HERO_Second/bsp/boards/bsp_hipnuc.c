
/**
 ****************************(C) 版权所有 2025 none****************************
 * @file       hipnuc_data.c
 * @brief      HiPNuc串口DMA接收初始化模块
 *             配置串口USART6的DMA双缓冲接收功能，
 *             并启用UART的空闲中断以高效处理接收到的数据。
 *
 * @details    实现功能：
 *             配置USART6，DMA双缓冲模式接收hipnuc代码。
 *
 * @note       与官方例程的区别：
 *		   	   确保在完全初始化之后再启用空闲中断，防止在初始化未完成时接收到高频信号，
 *			   导致接收出现问题。
 *
 * @history    版本        日期            作者           修改内容
 *             V1.0.0     2025-01-11      Bai Shuhao     初始化hipnuc双缓冲区接收
 *             V2.0.0     2025-01-13      Bai Shuhao     将各种使能（主要是空闲中断）放在代码最后，防止初始化未完成时接收到高频数据，引发问题。
 *
 * @verbatim
 * ==============================================================================
 *  数据接收初始化说明：
 *  - 串口：USART6
 *  - DMA配置：
 *      - 外设地址：USART6数据寄存器
 *      - 内存地址：双缓冲区
 *      - 缓冲区模式：双缓冲
 * ==============================================================================
 * @endverbatim
 ****************************(C) 版权所有 none****************************
 */

#include "bsp_hipnuc.h"
#include "main.h"

extern UART_HandleTypeDef huart1;         // 使用 UART1
extern DMA_HandleTypeDef hdma_usart1_rx; // 使用 UART1 的 DMA 接收句柄

/**
  * @brief          初始化UART DMA双缓冲接收
  * @param[in]      *rx1_buf: 第一缓冲区地址
  * @param[in]      *rx2_buf: 第二缓冲区地址
  * @param[in]      dma_buf_num: DMA 缓冲区大小
  * @retval         none
  */
void HIPNUC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    // 清理 UART 状态
    __HAL_UART_CLEAR_PEFLAG(&huart1);  // 清空 UART 标志位
    uint32_t tmp = huart1.Instance->DR; // 读 DR 寄存器清除残留数据
    (void)tmp;

    // 配置 DMA 外设地址和缓冲区
    hdma_usart1_rx.Instance->PAR = (uint32_t)&(USART6->DR); // 外设地址
    hdma_usart1_rx.Instance->M0AR = (uint32_t)rx1_buf;      // 缓冲区 1 地址
    hdma_usart1_rx.Instance->M1AR = (uint32_t)rx2_buf;      // 缓冲区 2 地址
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;            // 缓冲区大小

    // 启用双缓冲模式
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    // 启用 DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

    // 启用 DMA 接收模式
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    // 最后启用 UART 空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}


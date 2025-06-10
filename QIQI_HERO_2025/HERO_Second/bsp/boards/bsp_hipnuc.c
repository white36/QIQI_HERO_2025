
/**
 ****************************(C) ��Ȩ���� 2025 none****************************
 * @file       hipnuc_data.c
 * @brief      HiPNuc����DMA���ճ�ʼ��ģ��
 *             ���ô���USART6��DMA˫������չ��ܣ�
 *             ������UART�Ŀ����ж��Ը�Ч������յ������ݡ�
 *
 * @details    ʵ�ֹ��ܣ�
 *             ����USART6��DMA˫����ģʽ����hipnuc���롣
 *
 * @note       ��ٷ����̵�����
 *		   	   ȷ������ȫ��ʼ��֮�������ÿ����жϣ���ֹ�ڳ�ʼ��δ���ʱ���յ���Ƶ�źţ�
 *			   ���½��ճ������⡣
 *
 * @history    �汾        ����            ����           �޸�����
 *             V1.0.0     2025-01-11      Bai Shuhao     ��ʼ��hipnuc˫����������
 *             V2.0.0     2025-01-13      Bai Shuhao     ������ʹ�ܣ���Ҫ�ǿ����жϣ����ڴ�����󣬷�ֹ��ʼ��δ���ʱ���յ���Ƶ���ݣ��������⡣
 *
 * @verbatim
 * ==============================================================================
 *  ���ݽ��ճ�ʼ��˵����
 *  - ���ڣ�USART6
 *  - DMA���ã�
 *      - �����ַ��USART6���ݼĴ���
 *      - �ڴ��ַ��˫������
 *      - ������ģʽ��˫����
 * ==============================================================================
 * @endverbatim
 ****************************(C) ��Ȩ���� none****************************
 */

#include "bsp_hipnuc.h"
#include "main.h"

extern UART_HandleTypeDef huart1;         // ʹ�� UART1
extern DMA_HandleTypeDef hdma_usart1_rx; // ʹ�� UART1 �� DMA ���վ��

/**
  * @brief          ��ʼ��UART DMA˫�������
  * @param[in]      *rx1_buf: ��һ��������ַ
  * @param[in]      *rx2_buf: �ڶ���������ַ
  * @param[in]      dma_buf_num: DMA ��������С
  * @retval         none
  */
void HIPNUC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // ʧЧ DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    // ���� UART ״̬
    __HAL_UART_CLEAR_PEFLAG(&huart1);  // ��� UART ��־λ
    uint32_t tmp = huart1.Instance->DR; // �� DR �Ĵ��������������
    (void)tmp;

    // ���� DMA �����ַ�ͻ�����
    hdma_usart1_rx.Instance->PAR = (uint32_t)&(USART6->DR); // �����ַ
    hdma_usart1_rx.Instance->M0AR = (uint32_t)rx1_buf;      // ������ 1 ��ַ
    hdma_usart1_rx.Instance->M1AR = (uint32_t)rx2_buf;      // ������ 2 ��ַ
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;            // ��������С

    // ����˫����ģʽ
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    // ���� DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

    // ���� DMA ����ģʽ
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    // ������� UART �����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}


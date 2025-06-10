/**
 ****************************(C) ��Ȩ���� 2025 none****************************
 * @file       hipnuc_data.c
 * @brief      HiPNuc���ݽ���ģ��
 *             ����HiPNuc�������ݽ��գ�
 *             ʹ�ô��ڣ�USART1��˫����DMA���и�Ч���ݽ��ա�
 *
 * @details    ʵ�ֹ��ܣ�
 *             - HiPNucͨ�ŵĽ��պ���
 *             - ˫�������ݽ�����������ܣ���ֹռ�ù���CPUʱ�䣩
 *
 * @note       ���ݽ���ͨ���ź�����FreeRTOS����ͬ����ɣ���ֹ�жϷ������ռ�ù�����Դ��
 *
 * @history    �汾        ����            ����           �޸�����
 *             V1.0.0     2025-1-10       Bai Shuhao     DMA˫�������������������ݣ�������̼ҷ�װ�ļ�hipnuc_dec.c
 *
 * @verbatim
 * ==============================================================================
 *  ͨ��Э�����飨����Modbus��׼Э�飩��
 *  - ����֡�ṹ��
 *      - ֡ͷ: 0x5A 0xA5
 *      - ֡����: 2�ֽڣ�С��ģʽ��
 *      - ����: �䳤����Ч�غɣ��������ٶȡ����ٶȡ������Ƶ���Ϣ��
 *      - CRCУ��: 2�ֽ�
 *  - CRCУ���㷨: Modbus��׼CRC16������ʽ0x8005��
 *  - ʾ��֡���� (ʮ�����Ƹ�ʽ)��
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
 ****************************(C) ��Ȩ���� none****************************
 */

#include "hipnuc_rx.h"

#include "main.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/*************************************************** �������� **********************************************************/
// ����ԭʼ���ݣ�˫����������
static uint8_t hipnuc_rx_buf[2][HIPNUC_RX_BUF_NUM];

int hipnuc_data_flag = 0;   //���ݽ��ճɹ���־λ
uint16_t uart_rx_index = 0; //��¼��ǰDMA�������ݳ���
uint8_t *current_decode_buf;               // ȫ�ֱ�����ָ��ǰҪ����Ļ�����
SemaphoreHandle_t hipnuc_decode_semaphore; // �ź���

/*************************************************** �������� ***********************************************************/
/**
 * @brief          �����ض���ʽ��HiPNuc�������ݣ�USART1�汾��
 * @details        ʹ��USART1���жϻ��ƺ�DMA˫����������HiPNuc����֡��������У����յ�������֡��
 *                 �����ݳ��ȷ���Ԥ�ڲ���CRCУ��ͨ��������£������ź�����֪ͨ������������
 * @note           1. ������������֡ʹ���ض���HiPNucЭ���ʽ��
 *                 2. ˫���������������������ݣ���ֹ���ݶ�ʧ��
 *                 3. CRCУ��ȷ�����������ԡ�
 */
void hipnuc_data_IRQ(void);

/**
  * @brief          �����ǽ��ճ�ʼ��
  * @param[in]      none
  * @retval         none
  */
void hipnuc_rx_init(void);

/****************************************************��������******************************************************/
//hipnuc_decode_semaphore = xSemaphoreCreateBinary();
/**
 * @brief          �����ض���ʽ��HiPNuc�������ݣ�USART1�汾��
 * @details        ʹ��USART1���жϻ��ƺ�DMA˫����������HiPNuc����֡��������У����յ�������֡��
 *                 �����ݳ��ȷ���Ԥ�ڲ���CRCУ��ͨ��������£������ź�����֪ͨ������������
 * @note           1. ������������֡ʹ���ض���HiPNucЭ���ʽ��
 *                 2. ˫���������������������ݣ���ֹ���ݶ�ʧ��
 *                 3. CRCУ��ȷ�����������ԡ�
 */
void hipnuc_data_IRQ(void)
{
	if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    // ���USART1�Ŀ����жϱ�־λ
    if (huart1.Instance->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0; // ��¼���ν��յ����ݳ���

        // ��������жϱ�־λ
        // ִ��һ��"������"���ȶ�ȡSR��״̬�Ĵ��������ٶ�ȡDR�����ݼĴ�����
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        // �жϵ�ǰʹ�õĻ�������DMA˫������ƣ�
        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* ��ǰʹ���ڴ滺����0 */

            // �ر�DMA�Է�ֹ���ݴ����ͻ
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // ��ȡ���յ����ݳ���
            this_time_rx_len = HIPNUC_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // ����DMA�����ݴ�������Ĵ�����׼�����������ݣ�
            hdma_usart1_rx.Instance->NDTR = HIPNUC_RX_BUF_NUM;

            // �л���������1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            // ����ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            // ������յ�������
			current_decode_buf = hipnuc_rx_buf[0];
			uart_rx_index = this_time_rx_len;

        }
        else
        {
            /* ��ǰʹ���ڴ滺����1 */

            // �ر�DMA�Է�ֹ���ݴ����ͻ
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // ��ȡ���յ����ݳ���
            this_time_rx_len = HIPNUC_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // ����DMA�����ݴ�������Ĵ�����׼�����������ݣ�
            hdma_usart1_rx.Instance->NDTR = HIPNUC_RX_BUF_NUM;

            // �л���������0
            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);

            // ����ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            // ������յ�������
			current_decode_buf = hipnuc_rx_buf[1];
			uart_rx_index = this_time_rx_len;
        }
    }
}

/**
  * @brief          �����ǽ��ճ�ʼ��
  * @param[in]      none
  * @retval         none
  */
void hipnuc_rx_init(void)
{
    /* ��ʼ��DMA˫������� */
	HIPNUC_init(hipnuc_rx_buf[0],    // ��һ������
                hipnuc_rx_buf[1],    // �ڶ�������
                HIPNUC_RX_BUF_NUM);  //����������
}


//#include "hipnuc_task.h"



///**
//  * @brief          hipnuc���ݽ���
//  * @param[in]      none
//  * @retval         none
//  */
//void HIPNUC_Task(void)
//{
//	if(xSemaphoreTake(hipnuc_decode_semaphore, 0) == pdTRUE) 
//	{
//		for (uint16_t i = 0; i < uart_rx_index; i++)
//		{	
//			//���ֽڽ���������������
//			if(hipnuc_input(&hipnuc_raw, current_decode_buf[i]))
//			{
//				//�������
//				hipnuc_flag = 1;
//			}
//		}
//	}
//}

///**
//  * @brief          ����pitch �������ָ��
//  * @param[in]      none
//  * @retval         pitch
//  */
//const hi91_t* get_HIPNUC_point(void)
//{
//    return &(hipnuc_raw.hi91);
//}

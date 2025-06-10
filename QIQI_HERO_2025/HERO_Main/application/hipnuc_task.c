//#include "hipnuc_task.h"



///**
//  * @brief          hipnuc数据解码
//  * @param[in]      none
//  * @retval         none
//  */
//void HIPNUC_Task(void)
//{
//	if(xSemaphoreTake(hipnuc_decode_semaphore, 0) == pdTRUE) 
//	{
//		for (uint16_t i = 0; i < uart_rx_index; i++)
//		{	
//			//逐字节接收数据流并解析
//			if(hipnuc_input(&hipnuc_raw, current_decode_buf[i]))
//			{
//				//解码完成
//				hipnuc_flag = 1;
//			}
//		}
//	}
//}

///**
//  * @brief          返回pitch 电机数据指针
//  * @param[in]      none
//  * @retval         pitch
//  */
//const hi91_t* get_HIPNUC_point(void)
//{
//    return &(hipnuc_raw.hi91);
//}

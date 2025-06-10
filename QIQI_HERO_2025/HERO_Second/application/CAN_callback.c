/**   ****************************(C) ��Ȩ���� 2024 none****************************
 * @file       CAN_Callback.c
 * @brief      CANͨ�Żص�����
 *
 * @details    ʵ�ֹ��ܣ���װcanͨ�Żص�����
 *
 * @note       ���ص�������ÿ��task�ļ��Ƴ�����װ����ֹ���task��һ��can���µĻص�������ͻ
 *
 * @history    �汾        ����            ����           �޸�����
 *             V1.0.0     2024-12-18      BaiShuhao      ����������������޸ĵļ�¼
 *
 * @verbatim
 * ==============================================================================
 *  ͨ��Э�����飺
 *  - �豸��ַ: 0x01
 *  - ������: 0x03 
 *  - ���ݸ�ʽ: ��ת��ΪASCII�����ʮ����������ֵ 6���ֽ�
 * ==============================================================================
 * @endverbatim
 ****************************(C) ��Ȩ���� none ********************************
 */
 
 #include "CAN_callback.h"
 
    /**
  * @brief CAN1���Ļص�����
  *
  * @param Rx_Buffer CAN���յ���Ϣ�ṹ��
  */
 void CAN_Motor_Call_Back_CAN1(Struct_CAN_Rx_Buffer *Rx_Buffer)
 {
 	switch (Rx_Buffer->Header.StdId)
     {
 		case CAN_3508_LEFT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_left, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_3508_RIGHT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_right, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_3508_BLEFT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_bleft, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_3508_BRIGHT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_bright, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_TRIGGER_MOTOR_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_trigger, Rx_Buffer->Data);
 		break;
 	}
 	}
 }
 
  /**
  * @brief CAN2���Ļص�����
  *
  * @param Rx_Buffer CAN���յ���Ϣ�ṹ��
  */
 void CAN_Motor_Call_Back_CAN2(Struct_CAN_Rx_Buffer *Rx_Buffer)
 {
 	    switch (Rx_Buffer->Header.StdId)
     {
 	case CAN_3508_M1_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[0].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	case CAN_3508_M2_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[1].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	case CAN_3508_M3_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[2].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	case CAN_3508_M4_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[3].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	//
 		case 0X205: // yaw
 	{
         Motor_GM6020_CAN_RxCpltCallback(&gimbal_control.gimbal_yaw_motor.GM6020_measure, Rx_Buffer->Data);
         break;
    }
 	case 0X206: // PITCH
 	{
         Motor_C620_CAN_RxCpltCallback(&gimbal_control.gimbal_pitch_motor.C620_measure, Rx_Buffer->Data);
         break;
    }
    }
 }

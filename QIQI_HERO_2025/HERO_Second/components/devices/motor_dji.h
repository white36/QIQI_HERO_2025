/**
 * @file motor_dji.h
 * @author CHR
 * @brief ��CAN������������
 * @version 0.1
 * @date 2024-11-04
 *
 *
 */

#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Mathh.h"
#include "arm_math.h"
#include "can.h"
#include "PID_Control.h"
#include "CAN_receive.h"
// RPM���㵽rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)

	/* Exported types ------------------------------------------------------------*/

	/**
	 * @brief ���״̬
	 *
	 */
	typedef enum {
		CAN_Motor_Status_DISABLE = 0,
		CAN_Motor_Status_ENABLE,
	} Enum_CAN_Motor_Status;



/**
 * @brief CAN�����ID�������
 *
 */
typedef enum {
    CAN_Motor_ID_Status_FREE = 0,
    CAN_Motor_ID_Status_ALLOCATED,
} Enum_CAN_Motor_ID_Status;

/**
 * @brief ������Ʒ�ʽ
 *
 */
typedef enum {
    Control_Method_OPENLOOP = 0,//����
    Control_Method_TORQUE,      //���ػ�
    Control_Method_OMEGA,       //���ٶȻ�
	Control_Method_Velocity,    //�ٶȻ�
    Control_Method_ANGLE,       //�ǶȻ�
	Control_Method_TRIGGER,       //�ǶȻ�
} Enum_Control_Method;

/**
 * @brief GM6020��ˢ���, ��Ƭ�����������ѹ
 *
 */
typedef struct {
    // PID�ǶȻ�����
    PID_control PID_Angle;
    // PID���ٶȻ�����
    PID_control PID_Omega;
    // PIDŤ�ػ�����
    PID_control PID_Torque;

    // ��ʼ����ر���
    Struct_CAN_Manage_Object *CAN_Manage_Object; // �󶨵�CAN
    Enum_CAN_Motor_ID CAN_ID; // �����ݰ󶨵�CAN ID
    uint8_t *CAN_Tx_Data; // ���ͻ�����
    uint32_t Encoder_Offset; // ������ƫ��
    float Omega_Max; // ����ٶ�

    // ����
    uint16_t Encoder_Num_Per_Round; // һȦ�������̶�
    uint16_t Output_Max; // ��������ѹ

    // �ڲ�����
    uint32_t Flag; // ��ǰʱ�̵ĵ������flag
    uint32_t Pre_Flag; // ǰһʱ�̵ĵ������flag
    uint16_t Rx_Encoder; // ���յı�����λ��
    int16_t Rx_Omega; // ���յ��ٶ�
    int16_t Rx_Torque; // ���յ�Ť��
    uint16_t Rx_Temperature; // ���յ��¶�

    uint16_t Pre_Encoder; // ֮ǰ�ı�����λ��
    int32_t Total_Encoder; // �ܱ�����λ��
    int32_t Total_Round; // ��Ȧ��

    // ������
    Enum_CAN_Motor_Status CAN_Motor_Status; // ���״̬
    float Now_Angle; // ��ǰ�ĽǶ�
    float Now_Omega; // ��ǰ���ٶ�
    float Now_Torque; // ��ǰ��Ť��
    uint8_t Now_Temperature; // ��ǰ���¶�

    // д����
    Enum_Control_Method Control_Method; // ������Ʒ�ʽ
    float Target_Angle; // Ŀ��ĽǶ�
    float Target_Omega; // Ŀ����ٶ�
    float Target_Torque; // Ŀ���Ť��
    float Out; // �����

} Motor_GM6020;

/**
 * @brief C610��ˢ���, �Դ�Ť�ػ�, ��Ƭ���������Ť��
 *
 */
typedef struct {
    // PID�ǶȻ�����
    PID_control PID_Angle;
    // PID���ٶȻ�����
    PID_control PID_Omega;

    // ��ʼ����ر���
    Struct_CAN_Manage_Object *CAN_Manage_Object; // �󶨵�CAN
    Enum_CAN_Motor_ID CAN_ID; // �����ݰ󶨵�CAN ID
    uint8_t *CAN_Tx_Data; // ���ͻ�����
    float Gearbox_Rate; // ���ٱ�
    float Torque_Max; // ���Ť��

    // ����
    uint16_t Encoder_Num_Per_Round; // һȦ�������̶�
    uint16_t Output_Max; // ������Ť��

    // �ڲ�����
    uint32_t Flag; // ��ǰʱ�̵ĵ������flag
    uint32_t Pre_Flag; // ǰһʱ�̵ĵ������flag
    uint16_t Rx_Encoder; // ���յı�����λ��
    int16_t Rx_Omega; // ���յ��ٶ�
    int16_t Rx_Torque; // ���յ�Ť��
    uint16_t Rx_Temperature; // ���յ��¶�

    uint16_t Pre_Encoder; // ֮ǰ�ı�����λ��
    int32_t Total_Encoder; // �ܱ�����λ��
    int32_t Total_Round; // ��Ȧ��

    // ������
    Enum_CAN_Motor_Status CAN_Motor_Status; // ���״̬
    float Now_Angle; // ��ǰ�ĽǶ�
    float Now_Omega; // ��ǰ���ٶ�
    float Now_Torque; // ��ǰ��Ť��
    uint8_t Now_Temperature; // ��ǰ���¶�

    // д����
    Enum_Control_Method Control_Method; // ������Ʒ�ʽ
    float Target_Angle; // Ŀ��ĽǶ�
    float Target_Omega; // Ŀ����ٶ�
    float Target_Torque; // Ŀ���Ť��
    float Out; // �����

} Motor_C610;

/**
 * @brief C620��ˢ���, �Դ�Ť�ػ�, ��Ƭ���������Ť��
 *
 */
typedef struct {
    // PID�ǶȻ�����
    PID_control PID_Angle;
    // PID���ٶȻ�����
    PID_control PID_Omega;
	// PID���ٶȻ�����
	PID_control PID_Velocity;

    // ��ʼ����ر���
    Struct_CAN_Manage_Object *CAN_Manage_Object; // �󶨵�CAN
    Enum_CAN_Motor_ID CAN_ID; // �����ݰ󶨵�CAN ID
    uint8_t *CAN_Tx_Data; // ���ͻ�����
    float Gearbox_Rate; // ���ٱ�
    float Torque_Max; // ���Ť��

    // ����
    uint16_t Encoder_Num_Per_Round; // һȦ�������̶�
    uint16_t Output_Max; // ������Ť��

    // �ڲ�����
    uint32_t Flag; // ��ǰʱ�̵ĵ������flag
    uint32_t Pre_Flag; // ǰһʱ�̵ĵ������flag
    uint16_t Rx_Encoder; // ���յı�����λ��
    int16_t Rx_Omega; // ���յ��ٶ� ��λ rpm
    int16_t Rx_Torque; // ���յ�Ť��
    uint16_t Rx_Temperature; // ���յ��¶�

    uint16_t Pre_Encoder; // ֮ǰ�ı�����λ��
    int32_t Total_Encoder; // �ܱ�����λ��
    int32_t Total_Round; // ��Ȧ��

    // ������
    Enum_CAN_Motor_Status CAN_Motor_Status; // ���״̬
    float Now_Angle; 	// ��ǰ�ĽǶ�
    float Now_Omega; 	// ��ǰ�Ľ��ٶ� //��λ rad/s
	float Now_Velocity; // ��ǰ�����ٶ� //��λ m/s
    float Now_Torque; 	// ��ǰ��Ť��
    uint8_t Now_Temperature; // ��ǰ���¶�

    // д����
    Enum_Control_Method Control_Method; // ������Ʒ�ʽ
    float Target_Angle; 	// Ŀ��ĽǶ�
    float Target_Omega; 	// Ŀ��Ľ��ٶ� //��λ rad/s
	float Target_Velocity;  // Ŀ������ٶ� //��λ m/s
    float Target_Torque; 	// Ŀ���Ť��
    float Out; 				// �����

} Motor_C620;

/* Exported function declarations --------------------------------------------*/

void Motor_GM6020_Init(Motor_GM6020 *motor, CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, int32_t __Encoder_Offset, float __Omega_Max);
void Motor_GM6020_Output(Motor_GM6020 *motor);
uint16_t Motor_GM6020_Get_Output_Max(Motor_GM6020 *motor);
Enum_CAN_Motor_Status Motor_GM6020_Get_CAN_Motor_Status(Motor_GM6020 *motor);
float Motor_GM6020_Get_Now_Angle(Motor_GM6020 *motor);
float Motor_GM6020_Get_Now_Omega(Motor_GM6020 *motor);
float Motor_GM6020_Get_Now_Torque(Motor_GM6020 *motor);
uint8_t Motor_GM6020_Get_Now_Temperature(Motor_GM6020 *motor);
Enum_Control_Method Motor_GM6020_Get_Control_Method(Motor_GM6020 *motor);
float Motor_GM6020_Get_Target_Angle(Motor_GM6020 *motor);
float Motor_GM6020_Get_Target_Omega(Motor_GM6020 *motor);
float Motor_GM6020_Get_Target_Torque(Motor_GM6020 *motor);
float Motor_GM6020_Get_Out(Motor_GM6020 *motor);
void Motor_GM6020_Set_Control_Method(Motor_GM6020 *motor, Enum_Control_Method __Control_Method);
void Motor_GM6020_Set_Target_Angle(Motor_GM6020 *motor, float __Target_Angle);
void Motor_GM6020_Set_Target_Omega(Motor_GM6020 *motor, float __Target_Omega);
void Motor_GM6020_Set_Target_Torque(Motor_GM6020 *motor, float __Target_Torque);
void Motor_GM6020_Set_Out(Motor_GM6020 *motor, float __Out);
void Motor_GM6020_CAN_RxCpltCallback(Motor_GM6020 *motor, uint8_t *Rx_Data);
void Motor_GM6020_TIM_Alive_PeriodElapsedCallback(Motor_GM6020 *motor);
void Motor_GM6020_TIM_PID_PeriodElapsedCallback(Motor_GM6020 *motor);
void Motor_GM6020_give_current(Motor_GM6020 *motor, float current);

void Motor_C610_Init(Motor_C610 *motor, CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method);
void Motor_C610_Output(Motor_C610 *motor);
uint16_t Motor_C610_Get_Output_Max(Motor_C610 *motor);
Enum_CAN_Motor_Status Motor_C610_Get_CAN_Motor_Status(Motor_C610 *motor);
float Motor_C610_Get_Now_Angle(Motor_C610 *motor);
float Motor_C610_Get_Now_Omega(Motor_C610 *motor);
float Motor_C610_Get_Now_Torque(Motor_C610 *motor);
uint8_t Motor_C610_Get_Now_Temperature(Motor_C610 *motor);
Enum_Control_Method Motor_C610_Get_Control_Method(Motor_C610 *motor);
float Motor_C610_Get_Target_Angle(Motor_C610 *motor);
float Motor_C610_Get_Target_Omega(Motor_C610 *motor);
float Motor_C610_Get_Target_Torque(Motor_C610 *motor);
float Motor_C610_Get_Out(Motor_C610 *motor);
void Motor_C610_Set_Control_Method(Motor_C610 *motor, Enum_Control_Method __Control_Method);
void Motor_C610_Set_Target_Angle(Motor_C610 *motor, float __Target_Angle);
void Motor_C610_Set_Target_Omega(Motor_C610 *motor, float __Target_Omega);
void Motor_C610_Set_Target_Torque(Motor_C610 *motor, float __Target_Torque);
void Motor_C610_Set_Out(Motor_C610 *motor, float __Out);
void Motor_C610_CAN_RxCpltCallback(Motor_C610 *motor, uint8_t *Rx_Data);
void Motor_C610_TIM_Alive_PeriodElapsedCallback(Motor_C610 *motor);
void Motor_C610_TIM_PID_PeriodElapsedCallback(Motor_C610 *motor);

void Motor_C620_Init(Motor_C620 *motor, CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method);
void Motor_C620_Output(Motor_C620 *motor);
void Motor_C620_give_current(Motor_C620 *motor, float current);
uint16_t Motor_C620_Get_Output_Max(Motor_C620 *motor);
Enum_CAN_Motor_Status Motor_C620_Get_CAN_Motor_Status(Motor_C620 *motor);
float Motor_C620_Get_Now_Angle(Motor_C620 *motor);
float Motor_C620_Get_Now_Omega(Motor_C620 *motor);
float Motor_C620_Get_Now_Torque(Motor_C620 *motor);
uint8_t Motor_C620_Get_Now_Temperature(Motor_C620 *motor);
Enum_Control_Method Motor_C620_Get_Control_Method(Motor_C620 *motor);
float Motor_C620_Get_Target_Angle(Motor_C620 *motor);
float Motor_C620_Get_Target_Omega(Motor_C620 *motor);
float Motor_C620_Get_Target_Torque(Motor_C620 *motor);
float Motor_C620_Get_Out(Motor_C620 *motor);
void Motor_C620_Set_Control_Method(Motor_C620 *motor, Enum_Control_Method __Control_Method);
void Motor_C620_Set_Target_Angle(Motor_C620 *motor, float __Target_Angle);
void Motor_C620_Set_Target_Omega(Motor_C620 *motor, float __Target_Omega);
void Motor_C620_Set_Target_Torque(Motor_C620 *motor, float __Target_Torque);
void Motor_C620_Set_Out(Motor_C620 *motor, float __Out);
void Motor_C620_CAN_RxCpltCallback(Motor_C620 *motor, uint8_t *Rx_Data);
void Motor_C620_TIM_Alive_PeriodElapsedCallback(Motor_C620 *motor);
void Motor_C620_TIM_PID_PeriodElapsedCallback(Motor_C620 *motor);

#endif

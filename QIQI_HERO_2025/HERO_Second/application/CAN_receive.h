/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "freertos.h"

#define CHASSIS_CAN hcan2
#define CAPID_CAN hcan2

#define GIMBAL_CAN hcan2
#define POWER_CAN hcan2
#define SHOOT_CAN hcan1
#define CAP_CAN hcan2
#define UI_CAN hcan1
#define TUI_CAN hcan1

#define CAN_FEEDBACK_FREAM_ID_A 0x11
#define CAN_FEEDBACK_FREAM_ID_B 0x12
#define CAN_CTRL_FREAM_ID 0x21 // CAN֡ID��
/* CAN send and receive ID */
typedef enum
{
    // Can2
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205, // 1
    CAN_PIT_MOTOR_ID = 0x206,

    CAN_TRIGGER_MOTOR_ID = 0x207,

    CAN_CAP_ID = 0x211, // ����

} can_msg_id_e;

typedef struct
{
    int16_t chassis_power; // ���̹���
    int16_t cap_volt;      // ���ݵ�ѹ
    int16_t cap_curr;      // ���ݵ���
    int16_t cap_energy;    // ����ʣ�������ٷֱ�
} cap_measure_t;           // Ϫ�س���

// ���г���
typedef enum
{
    BATTERY = 1,
    CAPACITY,
    OUT_OFF,
} power_source_enum;
typedef struct
{
    uint16_t input_voltage; // �����ѹ
    uint16_t current;       // �������
    uint16_t cap_voltage;   // ���ݵ�ѹ
    uint8_t p_set;          // �趨����
    uint8_t crc_checksum;
} can_feedback_a_typedef; // CAN��������A
typedef struct
{
    uint16_t output_voltage;  // �����ѹ
    uint8_t power_source : 7; // ��Դ��Դ
    uint8_t out_auto_en : 1;  // ��������Ƿ��Զ�����
    uint8_t nc1;
    uint8_t nc2;
    uint8_t nc3;
    uint8_t nc4; // ��
    uint8_t crc_checksum;
} can_feedback_b_typedef; // CAN��������B

typedef struct
{
    uint8_t p_set;               // �趨����
    uint8_t power_source : 7;    // ���Ƶ�Դ��Դ-1ǰ����Դ  2������  3����ر�
    uint8_t out_auto_en : 1;     // ��������Ƿ��Զ�����
    uint16_t freq_feedback : 15; // ����Ƶ�ʣ�Ĭ��100
    uint16_t wireless_en : 1;    // ���߿���
    uint8_t nc1;
    uint8_t nc2;
    uint8_t nc3; // ��
    uint8_t crc_checksum;
} can_control_typedef; // CAN��������

// RM �������
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    fp32 voltage;
    fp32 cuttent;
    fp32 power;
} pm_measure_t;

extern TickType_t REPLACE_COMM_A_TIME, REPLACE_COMM_B_TIME, REPLACE_COMM_C_TIME;

/**
 * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
 * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
 * @param[in]      rev1: (0x207) ������������Ƶ���
 * @param[in]      rev2: (0x208) ������������Ƶ���
 * @retval         none
 */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);

extern void CAN_cmd_pitch(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);

extern can_feedback_a_typedef get_capA;
extern can_feedback_b_typedef get_capB;
/**
 * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
 * @param[in]      none
 * @retval         none
 */
extern void CAN_cmd_chassis_reset_ID(void);

/**
 * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
 * @param[in]      s0: (0x201) 2006������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      s1: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      s2: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      trigger: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_shoot(int16_t left, int16_t right, int16_t bleft, int16_t bright);
extern void CAN_cmd_trigger(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

///**
// * @brief          ���г���can����
// * @author         LYH
// * @param[in]      target_power��(15-350w)
//                   flag:0����;1����
// * @retval         ����ָ��
// */
// extern void CAN_cmd_cap(int16_t target_power, uint16_t flag);

// Ϫ�س�������can����
void CAN_cmd_cap(int16_t temPower);

extern void CAN_cmd_ui(int16_t key1, int16_t key2, int16_t key3, int16_t key4);
extern void CAN_cmd_ui2(int16_t Key1, int16_t Key2, int16_t Key3, int16_t Key4);

/**
 * @brief          ����meng��������ָ��
 * @param[in]      none
 * @retval         ��������ָ��
 */
extern const cap_measure_t *get_cap_data_point(void);

/**
 * @brief          ����yaw 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
 * @brief          ����pitch 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
 * @brief          ���ز������ 2006�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief          ���ص��̵�� 3508�������ָ��
 * @param[in]      i: ������,��Χ[0,3]
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
 * @brief          ���ز������ 2006�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_can_2006_measure_point(void);

/**
 * @brief          ����Ħ���� 3508�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_can_3508_left_measure_point(void);

/**
 * @brief          ����Ħ���� 3508�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_can_3508_right_measure_point(void);

/**
 * @brief          ����Ħ���� 3508�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_can_3508_bleft_measure_point(void);

/**
 * @brief          ����Ħ���� 3508�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_can_3508_bright_measure_point(void);
#endif

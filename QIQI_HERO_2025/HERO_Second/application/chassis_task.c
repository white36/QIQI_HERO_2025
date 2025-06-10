/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "bsp_usart.h"
#include "Mathh.h"
#include "referee.h"

int anglesr;
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @param[out]     chassis_move_init:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);

static void chassis_get_comm_data(chassis_move_t *receive_comm_data);
// /**
//  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
//  * @param[out]     chassis_move_mode:"chassis_move"����ָ��. 
//  * @retval         none
//  */
// static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
 * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
 * @retval         none
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

// /**
//  * @brief
//  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
//  * @retval         none
//  */
// static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

// ˫��ͨ�Ŵ�����ݺ���
static void comm_data_pack(chassis_move_t *send_data_pack);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
// �����˶�����
chassis_move_t chassis_move;
extern int8_t QA, BPIN, FOLLOW;
int8_t DBUS_error_flag = 0;
extern fp32 RX_PITCH, RX_first_speed, RX_add_speed;
int8_t KEY_shift, KEY_z, KEY_c, KEY_q; // ˫�����ң��������flag
int16_t cnts = 0;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern vision_rxfifo_t *vision_rx;
int8_t turn_flags = 0;
extern int MODE;
/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // ����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    // ���̳�ʼ��
    chassis_init(&chassis_move);
    // �жϵ��̵���Ƿ�����
    /*while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }*/

    while (1)
    {
		chassis_move.CHASSIS_xTickCount = xTaskGetTickCount();
		chassis_get_comm_data(&chassis_move);
		// �������ݷ���
		chassis_feedback_update(&chassis_move);
		// // ���̿���������
		// chassis_set_contorl(&chassis_move);
		// ���̿���PID����
		chassis_control_loop(&chassis_move);
		// ���ʿ���
		CHASSIC_MOTOR_POWER_CONTROL(&chassis_move);
		// ˫�巢��
		comm_data_pack(&chassis_move);
		CAN_comm_up(chassis_move.comm_a_output);


		// ȷ��˫���������,��֤���̵�����������
		if (chassis_move.CHASSIS_xTickCount - REPLACE_COMM_A_TIME < 2000 && chassis_move.CHASSIS_xTickCount - REPLACE_COMM_B_TIME < 2000)
		{
			// ��ң�������ߵ�ʱ�򣬷��͸����̵�������
			if (DBUS_error_flag)
			{
				CAN_cmd_chassis(0, 0, 0, 0);
			}
			else 
			if (chassis_move.robot_state->power_management_chassis_output == 0)
			{
				CAN_cmd_chassis(0, 0, 0, 0);
			}
			else
			{
				// ���Ϳ��Ƶ���
				CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
								chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			}
		}
		else
		{
			CAN_cmd_chassis(0, 0, 0, 0);
		}
	// ϵͳ��ʱ
	vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @param[out]     chassis_move_init:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    memset(chassis_move_init, 0, sizeof(chassis_move_t));

    // �����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    // ���̽Ƕ�pidֵ
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    // ���ʿ���
    const static fp32 power_buffer_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD}; // ���ʻ�PID����

    // ���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    // get gyro sensor euler angle point
    // ��ȡ������������̬��ָ��
    chassis_move_init->chassis_INS_point = get_INS_point();
    //������������
    chassis_move_init->cap_data = get_cap_data_point();
    // ����ϵͳ����
    chassis_move_init->robot_state = get_robot_status_point();
    chassis_move_init->power_heat_data = get_power_heat_data_point();
    chassis_move_init->shoot_data = get_shoot_data_point();

    uint8_t i;
    // ��ȡ���̵������ָ�룬��ʼ��PID
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    // ��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    // ���ʻ�PID
    PID_init(&chassis_move_init->buffer_pid, PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);

    // ��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	//��ʼ�� �������
	chassis_move_init->twist_init_flag = 0;
	chassis_move_init->change_twist_flag = 0;

    // ����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          ���˫���������
 * @param[out]     unpack_comm_data:"chassis_move"����ָ��.
 * @retval         none
 */
extern int8_t R, QA, BPIN, AUTO_ATTACK, STUCK, VISION, FOLLOW, TURN_REMOTE_FIRE;
void chassis_get_comm_data(chassis_move_t *receive_comm_data)
{
    /* A�� */
    receive_comm_data->vx_set = receive_comm_data->comm_rx_A.rx_vx_set;
    receive_comm_data->vy_set = receive_comm_data->comm_rx_A.rx_vy_set;

    /* B�� */
    // vz_set
    receive_comm_data->wz_set = receive_comm_data->comm_rx_B.rx_vz_set;

	if (receive_comm_data->comm_rx_B.rx_flag_b & COMM_FLAG_TURN_FLAG)
		turn_flags = 1;
	else
		turn_flags = 0;
	if (receive_comm_data->comm_rx_B.rx_flag_b & COMM_FLAG_KEY_C)
		KEY_c = 1;
	else
		KEY_c = 0;
	if (receive_comm_data->comm_rx_B.rx_flag_b & COMM_FLAG_GIMBAL_MODE)
		TURN_REMOTE_FIRE = 1;
	else
		TURN_REMOTE_FIRE = 0;
	
    // ����λ

    // ����ģʽ(����ȡ��4λ,�ܴ�16��ģʽ) ǰ��λΪ��־λ
    receive_comm_data->rx_chassis_mode = receive_comm_data->comm_rx_B.rx_chassis_mode & 0x0F;
    if (receive_comm_data->rx_chassis_mode == 0)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_RAW;
    else if (receive_comm_data->rx_chassis_mode == 1)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    else if (receive_comm_data->rx_chassis_mode == 2)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    else if (receive_comm_data->rx_chassis_mode == 3)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    else if (receive_comm_data->rx_chassis_mode == 4)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_BPIN;
    else if (receive_comm_data->rx_chassis_mode == 5)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_TWIST;
	else if (receive_comm_data->rx_chassis_mode == 6)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_TURN_ROUND;
	else
		receive_comm_data->chassis_mode = CHASSIS_VECTOR_RAW;
        // ���ݽ���ģʽ�ж��Ƿ����
        if (receive_comm_data->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
            FOLLOW = 1;
        else
            FOLLOW = 0;
        //����Z
        if (receive_comm_data->comm_rx_B.rx_chassis_mode & COMM_FLAG_Z)
            KEY_z = 1;
        else
            KEY_z = 0;
		//����Q
        if (receive_comm_data->comm_rx_B.rx_chassis_mode & COMM_FLAG_Q)
            KEY_q = 1;
        else
            KEY_q = 0;

    // ���յ���Flag
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_SHIFT)//����SHIFT
        KEY_shift = 1;
    else
        KEY_shift = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_R)
        R = 1;
    else
        R = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_QA)
        QA = 1;
    else
        QA = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_BPIN)
        BPIN = 1;
    else
        BPIN = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_AUTO_ATTACK)
        AUTO_ATTACK = 1;
    else
        AUTO_ATTACK = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_STUCK)
        STUCK = 1;
    else
        STUCK = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_VISION)
        VISION = 1;
    else
        VISION = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_DBUS_TOE)
        DBUS_error_flag = 1;
    else
        DBUS_error_flag = 0;

    /* C�� */
	
    RX_PITCH = receive_comm_data->comm_rx_C.rx_PITCH;
    RX_first_speed = receive_comm_data->comm_rx_C.rx_first_speed;
    RX_add_speed = receive_comm_data->comm_rx_C.rx_add_speed;

}


/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    // UI��ʾ���̽Ƕ�
    anglesr = abs((int)(chassis_move.chassis_yaw_motor->relative_angle * 100));
    if (anglesr > 157 && anglesr < 314)
    {
        anglesr = 314 - anglesr;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        // ���µ���ٶȡ����ٶ� ���ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE; //
    }

	chassis_move_update->chassis_yaw = chassis_move_update->chassis_INS_point->Yaw;
	chassis_move_update->chassis_pitch =  chassis_move_update->chassis_INS_point->Pitch;
    chassis_move_update->chassis_roll = chassis_move_update->chassis_INS_point->Roll;
}
/**
 * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
 *
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     vy_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    // �������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    //    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN+vision_rx->vx;
    //    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN+vision_rx->vy;
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

    // ���̿���
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    // һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    // ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z)
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
    }
}

/**
 * @brief          �ĸ������ٶ���ͨ�������������������
 * @param[in]      vx_set: �����ٶ�
 * @param[in]      vy_set: �����ٶ�
 * @param[in]      wz_set: ��ת�ٶ�
 * @param[out]     wheel_speed: �ĸ������ٶ�
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    // ��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ�� (��c���ĵ� U��ID��)
 wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * (MOTOR_DISTANCE_TO_CENTER_F-0.001f) * wz_set;
 wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER_F * wz_set;
 wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER_B * wz_set;
 wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * (MOTOR_DISTANCE_TO_CENTER_B-0.001f) * wz_set;
}

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    // �����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set, chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {

        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        // raw����ֱ�ӷ���
        return;
    }

    // �������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set); // fab����Ϊ��ǰ����
        if (max_vector < temp)
        {
            max_vector = temp; // ���xС�ڵ�ǰ�趨�ٶȣ�x=��ǰ�ٶ�
        }
    }

    if (max_vector > MAX_WHEEL_SPEED) // xС�ڵ�ǰ�ٶ�ʱ���ڵ�ǰ�ٶȣ�����ǰ�ٶȴ����������
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector; // �����ʼ�Ϊ ����ٶ�/��ǰ�ٶ�
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate; // �������
        }
    }
    // calculate pid
    // ����pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
    // ��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}



void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t *chassis_motor)
{
    static uint8_t can_send_tmp = 0;

    can_send_tmp++;

    uint16_t max_power_limit = 30;
    fp32 input_power = 0; // ���빦��(����������)
    fp32 scaled_motor_power[4];
    fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55  �˲������������ת��ΪŤ��
    fp32 k2 = 1.23e-07;                      // �Ŵ�ϵ��
    fp32 k1 = 1.453e-07;                     // �Ŵ�ϵ��

    fp32 constant = 4.081f;                                // 3508����Ļ�е���
    chassis_motor->power_control.POWER_MAX = 0;            // ���յ��̵������
    chassis_motor->power_control.forecast_total_power = 0; // Ԥ���ܹ���

    // PID_Calc(&chassis_motor->buffer_pid, chassis_motor->power_heat_data->buffer_energy, 30); //ʹ��������ά����һ���ȶ��ķ�Χ,�����PIDû��Ҫ��ֲ�ҵģ�������һ������
    PID_calc(&chassis_motor->buffer_pid, chassis_motor->power_heat_data->buffer_energy, 10);

    if (chassis_motor->robot_state->chassis_power_limit > 120)
    {
        max_power_limit = 120;
    }
    else
    {
        max_power_limit = chassis_motor->robot_state->chassis_power_limit;
    } // ��ò���ϵͳ�Ĺ���������ֵ

    input_power = max_power_limit - chassis_motor->buffer_pid.out; // ͨ������ϵͳ�������

    chassis_motor->power_control.power_charge = input_power; // �������ݵ�����繦��

    if (chassis_motor->power_control.power_charge > 13000)
    {
        chassis_motor->power_control.power_charge = 13000;
    } // �ο�������ư����������繦�ʣ�Ϫ�ذ��ӵ����ϲ�һ��

    if (can_send_tmp > 50)
    {
        CAN_cmd_cap((chassis_motor->power_control.power_charge) * 100); // ���ó���ĳ�繦��
        can_send_tmp = 0;
    }

    // CAN_cmd_cap(chassis_motor->power_control.power_charge);//��������

    if ((chassis_motor->cap_data->cap_volt) > 16) // �������ѹ����ĳ��ֵ(��ֹ����Ƿѹ����C620����)
    {
        if (KEY_shift) // �������磬һ�������𲽼���or���or����or���£�chassis_move.key_CΪ�˴����г��翪������

        {
           chassis_motor->power_control.POWER_MAX = 150;
        }
        else
        {
			chassis_motor->power_control.POWER_MAX = max_power_limit;
        }
    }
    else
    {
        chassis_motor->power_control.POWER_MAX = max_power_limit - 15;
    }

    for (uint8_t i = 0; i < 4; i++) // �������3508����Ĺ��ʺ��ܹ���
    {
        chassis_motor->power_control.forecast_motor_power[i] =
            chassis_motor->motor_chassis[i].give_current * toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm        // ת�ع���
            + k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm // �ٶȹ���
            + k2 * chassis_motor->motor_chassis[i].give_current * chassis_motor->motor_chassis[i].give_current + constant;

        if (chassis_motor->power_control.forecast_motor_power[i] < 0)
            continue; // ���Ը���

        chassis_motor->power_control.forecast_total_power += chassis_motor->power_control.forecast_motor_power[i]; // ����ܹ���+Ԥ�⹦��=ʵ�ʹ���
    }

    if (chassis_motor->power_control.forecast_total_power > chassis_motor->power_control.POWER_MAX) // ������ģ��˥��
    {
        fp32 power_scale = chassis_motor->power_control.POWER_MAX / chassis_motor->power_control.forecast_total_power;
        for (uint8_t i = 0; i < 4; i++)
        {
            scaled_motor_power[i] = chassis_motor->power_control.forecast_motor_power[i] * power_scale; // ���˥����Ĺ���

            if (scaled_motor_power[i] < 0)
                continue;

            fp32 b = toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm;
            fp32 c = k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant;

            if (chassis_motor->motor_chassis[i].give_current > 0) // ���ⳬ��������
            {
                chassis_motor->power_control.MAX_current[i] = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);
                if (chassis_motor->power_control.MAX_current[i] > 16000)
                {
                    chassis_motor->motor_chassis[i].give_current = 16000;
                }
                else
                    chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
            }
            else
            {
                chassis_motor->power_control.MAX_current[i] = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
                if (chassis_motor->power_control.MAX_current[i] < -16000)
                {
                    chassis_motor->motor_chassis[i].give_current = -16000;
                }
                else
                    chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
            }
        }
    }
}

/**
 * @brief          ����˫������֡
 * @param[out]     send_data_pack:"chassis_move"����ָ��.
 * @retval         none
 */
void comm_data_pack(chassis_move_t *send_data_pack)
{
    /* A�� */
    uint8_t send_flag;
    // tx_Flag
    if (send_data_pack->robot_state->power_management_gimbal_output == 1) // ��̨�ϵ�
        send_flag |= (1 << 0);
    else
        send_flag &= ~(1 << 0);
    if (send_data_pack->robot_state->power_management_shooter_output == 1) // �����ϵ�
        send_flag |= (1 << 1);
    else
        send_flag &= ~(1 << 1);
	if (send_data_pack->robot_state->robot_id > 100) // ����ID 0�� 1��
        send_flag |= (1 << 2); // ��
    else
        send_flag &= ~(1 << 2); // ��
	

    // ������������͵Ľṹ��
    send_data_pack->comm_tx_a.tx_current_heat = send_data_pack->power_heat_data->shooter_42mm_barrel_heat;
    send_data_pack->comm_tx_a.tx_robo_level = send_data_pack->robot_state->robot_level;
    send_data_pack->comm_tx_a.tx_initial_speed_x100 = (uint16_t)(send_data_pack->shoot_data->initial_speed*100.0f);
    send_data_pack->comm_tx_a.tx_flag = send_flag;//1
    PACK_STRUCT_TO_CAN_BUFFER(send_data_pack->comm_tx_a, send_data_pack->comm_a_output);
}

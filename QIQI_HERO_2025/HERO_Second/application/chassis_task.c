/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
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
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);

static void chassis_get_comm_data(chassis_move_t *receive_comm_data);
// /**
//  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
//  * @param[out]     chassis_move_mode:"chassis_move"变量指针. 
//  * @retval         none
//  */
// static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
 * @param[out]     chassis_move_transit:"chassis_move"变量指针.
 * @retval         none
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

// /**
//  * @brief
//  * @param[out]     chassis_move_update:"chassis_move"变量指针.
//  * @retval         none
//  */
// static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

// 双板通信打包数据函数
static void comm_data_pack(chassis_move_t *send_data_pack);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
// 底盘运动数据
chassis_move_t chassis_move;
extern int8_t QA, BPIN, FOLLOW;
int8_t DBUS_error_flag = 0;
extern fp32 RX_PITCH, RX_first_speed, RX_add_speed;
int8_t KEY_shift, KEY_z, KEY_c, KEY_q; // 双板接收遥控器按键flag
int16_t cnts = 0;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern vision_rxfifo_t *vision_rx;
int8_t turn_flags = 0;
extern int MODE;
/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    // 底盘初始化
    chassis_init(&chassis_move);
    // 判断底盘电机是否都在线
    /*while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }*/

    while (1)
    {
		chassis_move.CHASSIS_xTickCount = xTaskGetTickCount();
		chassis_get_comm_data(&chassis_move);
		// 底盘数据反馈
		chassis_feedback_update(&chassis_move);
		// // 底盘控制量设置
		// chassis_set_contorl(&chassis_move);
		// 底盘控制PID计算
		chassis_control_loop(&chassis_move);
		// 功率控制
		CHASSIC_MOTOR_POWER_CONTROL(&chassis_move);
		// 双板发送
		comm_data_pack(&chassis_move);
		CAN_comm_up(chassis_move.comm_a_output);


		// 确认双板接收正常,保证底盘电流发送正常
		if (chassis_move.CHASSIS_xTickCount - REPLACE_COMM_A_TIME < 2000 && chassis_move.CHASSIS_xTickCount - REPLACE_COMM_B_TIME < 2000)
		{
			// 当遥控器掉线的时候，发送给底盘电机零电流
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
				// 发送控制电流
				CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
								chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			}
		}
		else
		{
			CAN_cmd_chassis(0, 0, 0, 0);
		}
	// 系统延时
	vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    memset(chassis_move_init, 0, sizeof(chassis_move_t));

    // 底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    // 底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    // 功率控制
    const static fp32 power_buffer_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD}; // 功率环PID参数

    // 底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    // get gyro sensor euler angle point
    // 获取底盘陀螺仪姿态角指针
    chassis_move_init->chassis_INS_point = get_INS_point();
    //超级电容数据
    chassis_move_init->cap_data = get_cap_data_point();
    // 裁判系统数据
    chassis_move_init->robot_state = get_robot_status_point();
    chassis_move_init->power_heat_data = get_power_heat_data_point();
    chassis_move_init->shoot_data = get_shoot_data_point();

    uint8_t i;
    // 获取底盘电机数据指针，初始化PID
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    // 初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    // 功率环PID
    PID_init(&chassis_move_init->buffer_pid, PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);

    // 最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	//初始化 置零变量
	chassis_move_init->twist_init_flag = 0;
	chassis_move_init->change_twist_flag = 0;

    // 更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          解包双板接收数据
 * @param[out]     unpack_comm_data:"chassis_move"变量指针.
 * @retval         none
 */
extern int8_t R, QA, BPIN, AUTO_ATTACK, STUCK, VISION, FOLLOW, TURN_REMOTE_FIRE;
void chassis_get_comm_data(chassis_move_t *receive_comm_data)
{
    /* A包 */
    receive_comm_data->vx_set = receive_comm_data->comm_rx_A.rx_vx_set;
    receive_comm_data->vy_set = receive_comm_data->comm_rx_A.rx_vy_set;

    /* B包 */
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
	
    // 保留位

    // 底盘模式(仅获取后4位,能存16个模式) 前四位为标志位
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
        // 根据接收模式判断是否跟随
        if (receive_comm_data->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
            FOLLOW = 1;
        else
            FOLLOW = 0;
        //按键Z
        if (receive_comm_data->comm_rx_B.rx_chassis_mode & COMM_FLAG_Z)
            KEY_z = 1;
        else
            KEY_z = 0;
		//按键Q
        if (receive_comm_data->comm_rx_B.rx_chassis_mode & COMM_FLAG_Q)
            KEY_q = 1;
        else
            KEY_q = 0;

    // 接收到的Flag
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_SHIFT)//按键SHIFT
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

    /* C包 */
	
    RX_PITCH = receive_comm_data->comm_rx_C.rx_PITCH;
    RX_first_speed = receive_comm_data->comm_rx_C.rx_first_speed;
    RX_add_speed = receive_comm_data->comm_rx_C.rx_add_speed;

}


/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    // UI显示底盘角度
    anglesr = abs((int)(chassis_move.chassis_yaw_motor->relative_angle * 100));
    if (anglesr > 157 && anglesr < 314)
    {
        anglesr = 314 - anglesr;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        // 更新电机速度、加速度 是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE; //
    }

	chassis_move_update->chassis_yaw = chassis_move_update->chassis_INS_point->Yaw;
	chassis_move_update->chassis_pitch =  chassis_move_update->chassis_INS_point->Pitch;
    chassis_move_update->chassis_roll = chassis_move_update->chassis_INS_point->Roll;
}
/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
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
    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    //    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN+vision_rx->vx;
    //    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN+vision_rx->vy;
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

    // 键盘控制
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

    // 一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    // 停止信号，不需要缓慢加速，直接减速到零
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
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    // 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快 (大疆c板文档 U型ID序)
 wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * (MOTOR_DISTANCE_TO_CENTER_F-0.001f) * wz_set;
 wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER_F * wz_set;
 wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER_B * wz_set;
 wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * (MOTOR_DISTANCE_TO_CENTER_B-0.001f) * wz_set;
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    // 麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set, chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {

        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        // raw控制直接返回
        return;
    }

    // 计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set); // fab复制为当前轮速
        if (max_vector < temp)
        {
            max_vector = temp; // 最大x小于当前设定速度，x=当前速度
        }
    }

    if (max_vector > MAX_WHEEL_SPEED) // x小于当前速度时等于当前速度，即当前速度大于最大轮速
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector; // 轮速率即为 最大速度/当前速度
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate; // 结果等于
        }
    }
    // calculate pid
    // 计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
    // 赋值电流值
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
    fp32 input_power = 0; // 输入功率(缓冲能量环)
    fp32 scaled_motor_power[4];
    fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55  此参数将电机电流转换为扭矩
    fp32 k2 = 1.23e-07;                      // 放大系数
    fp32 k1 = 1.453e-07;                     // 放大系数

    fp32 constant = 4.081f;                                // 3508电机的机械损耗
    chassis_motor->power_control.POWER_MAX = 0;            // 最终底盘的最大功率
    chassis_motor->power_control.forecast_total_power = 0; // 预测总功率

    // PID_Calc(&chassis_motor->buffer_pid, chassis_motor->power_heat_data->buffer_energy, 30); //使缓冲能量维持在一个稳定的范围,这里的PID没必要移植我的，用任意一个就行
    PID_calc(&chassis_motor->buffer_pid, chassis_motor->power_heat_data->buffer_energy, 10);

    if (chassis_motor->robot_state->chassis_power_limit > 120)
    {
        max_power_limit = 120;
    }
    else
    {
        max_power_limit = chassis_motor->robot_state->chassis_power_limit;
    } // 获得裁判系统的功率限制数值

    input_power = max_power_limit - chassis_motor->buffer_pid.out; // 通过裁判系统的最大功率

    chassis_motor->power_control.power_charge = input_power; // 超级电容的最大充电功率

    if (chassis_motor->power_control.power_charge > 13000)
    {
        chassis_motor->power_control.power_charge = 13000;
    } // 参考超电控制板允许的最大充电功率，溪地板子的新老不一样

    if (can_send_tmp > 50)
    {
        CAN_cmd_cap((chassis_motor->power_control.power_charge) * 100); // 设置超电的充电功率
        can_send_tmp = 0;
    }

    // CAN_cmd_cap(chassis_motor->power_control.power_charge);//超级电容

    if ((chassis_motor->cap_data->cap_volt) > 16) // 当超电电压大于某个值(防止电容欠压导致C620掉电)
    {
        if (KEY_shift) // 主动超电，一般用于起步加速or冲刺or飞坡or上坡，chassis_move.key_C为此代码中超电开启按键

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

    for (uint8_t i = 0; i < 4; i++) // 获得所有3508电机的功率和总功率
    {
        chassis_motor->power_control.forecast_motor_power[i] =
            chassis_motor->motor_chassis[i].give_current * toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm        // 转矩功率
            + k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm // 速度功率
            + k2 * chassis_motor->motor_chassis[i].give_current * chassis_motor->motor_chassis[i].give_current + constant;

        if (chassis_motor->power_control.forecast_motor_power[i] < 0)
            continue; // 忽略负电

        chassis_motor->power_control.forecast_total_power += chassis_motor->power_control.forecast_motor_power[i]; // 电机总功率+预测功率=实际功率
    }

    if (chassis_motor->power_control.forecast_total_power > chassis_motor->power_control.POWER_MAX) // 超功率模型衰减
    {
        fp32 power_scale = chassis_motor->power_control.POWER_MAX / chassis_motor->power_control.forecast_total_power;
        for (uint8_t i = 0; i < 4; i++)
        {
            scaled_motor_power[i] = chassis_motor->power_control.forecast_motor_power[i] * power_scale; // 获得衰减后的功率

            if (scaled_motor_power[i] < 0)
                continue;

            fp32 b = toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm;
            fp32 c = k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant;

            if (chassis_motor->motor_chassis[i].give_current > 0) // 避免超过最大电流
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
 * @brief          发送双板数据帧
 * @param[out]     send_data_pack:"chassis_move"变量指针.
 * @retval         none
 */
void comm_data_pack(chassis_move_t *send_data_pack)
{
    /* A包 */
    uint8_t send_flag;
    // tx_Flag
    if (send_data_pack->robot_state->power_management_gimbal_output == 1) // 云台上电
        send_flag |= (1 << 0);
    else
        send_flag &= ~(1 << 0);
    if (send_data_pack->robot_state->power_management_shooter_output == 1) // 发射上电
        send_flag |= (1 << 1);
    else
        send_flag &= ~(1 << 1);
	if (send_data_pack->robot_state->robot_id > 100) // 红蓝ID 0红 1蓝
        send_flag |= (1 << 2); // 蓝
    else
        send_flag &= ~(1 << 2); // 红
	

    // 打包数据至发送的结构体
    send_data_pack->comm_tx_a.tx_current_heat = send_data_pack->power_heat_data->shooter_42mm_barrel_heat;
    send_data_pack->comm_tx_a.tx_robo_level = send_data_pack->robot_state->robot_level;
    send_data_pack->comm_tx_a.tx_initial_speed_x100 = (uint16_t)(send_data_pack->shoot_data->initial_speed*100.0f);
    send_data_pack->comm_tx_a.tx_flag = send_flag;//1
    PACK_STRUCT_TO_CAN_BUFFER(send_data_pack->comm_tx_a, send_data_pack->comm_a_output);
}

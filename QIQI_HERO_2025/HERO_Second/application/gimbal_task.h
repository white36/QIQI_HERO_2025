/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "stm32.h"
#include "remote_control.h"
#include "ins_task.h"
#include "vision_task.h"
#include "hipnuc_rx.h"  //陀螺仪通信文件
#include "FreeRTOS.h"
#include "task.h"
// pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 500.0f
#define PITCH_SPEED_PID_KI 0.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT 10000.0f
#define PITCH_SPEED_PID_MAX_IOUT 8000.0f

// yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 2000.0f
#define YAW_SPEED_PID_KI 5.0f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT 20000.0f
#define YAW_SPEED_PID_MAX_IOUT 10000.0f

// pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

// yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP 10.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

// pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 150.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 100.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 50.0f

// yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 0.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define YAW_VISION_PID_KP 0.5f
#define YAW_VISION_PID_KI 0.0f
#define YAW_VISION_PID_KD 0.0f
#define YAW_VISION_PID_MAX_OUT 10.0f
#define YAW_VISION_PID_MAX_IOUT 1.0f

#define PITCH_VISION_PID_KP 0.5f
#define PITCH_VISION_PID_KI 0.0f
#define PITCH_VISION_PID_KD 0.0f
#define PITCH_VISION_PID_MAX_OUT 10.0f
#define PITCH_VISION_PID_MAX_IOUT 1.0f
// 任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

////掉头180 按键
// #define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
// 掉头云台速度
#define TURN_SPEED 0.04f
// 遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10

#define YAW_RC_SEN -0.000006f
// #define YAW_RC_SEN    -0.0005f

#define PITCH_RC_SEN -0.000006f
// #define PITCH_RC_SEN   -0.00005f //0.005

#define YAW_MOUSE_SEN 0.00001f
#define PITCH_MOUSE_SEN 0.00001f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

#define GIMBAL_CONTROL_TIME 1

// 云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN 0
#define YAW_TURN 0

// 电机码盘值最大以及中值
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
// 云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
// 云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED 0.005f

#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

// 云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

// 判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

// 电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

// 线性控制器前馈系数
#define YAW_FEED_FORWARD 0.9f
#define PITCH_FEED_FORWARD 0.95f

// 角度误差项系数
#define K_YAW_ANGLE_ERROR 30000.0f
#define K_PITCH_ANGLE_ERROR 60000.0f // 20000.0f//150000.0f

// 速度项系数
#define K_YAW_ANGLE_SPEED 1000.0f
#define K_PITCH_ANGLE_SPEED 2000.0f //-600.0f//4000.0f

// 最大最小输出
#define YAW_MAX_OUT 32000.0f
#define YAW_MIX_OUT -32000.0f
#define PITCH_MAX_OUT 30000.0f
#define PITCH_MIX_OUT -30000.0f

typedef enum
{
    GIMBAL_MOTOR_RAW = 0,     // 电机原始值控制
    GIMBAL_MOTOR_GYRO,        // 电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE,     // 电机编码值角度控制
    GIMBAL_MOTOR_GYRONOLIMIT, // 小陀螺陀螺仪控制
    GIMBAL_MOTOR_AUTO,
    GIMBAL_MOTOR_BACK,//回头模式绝对角控制
} gimbal_motor_mode_e;

// 云台电机二阶线性控制器
typedef struct
{
    // 设定值
    fp32 set_angle;
    // 当前角度   一阶状态
    fp32 cur_angle;
    // 当前角速度 二阶状态
    fp32 cur_angle_speed;
    // 角度误差项 一阶状态误差
    fp32 angle_error;
    // 前馈项，用于消除系统固有扰动
    fp32 feed_forward;
    // 输出值
    fp32 output;
    // 最大输出值
    fp32 max_out;
    // 最小输出值
    fp32 min_out;

    // 前馈项系数
    fp32 k_feed_forward;
    // 误差项系数
    fp32 k_angle_error;
    // 二阶角速度项系数
    fp32 k_angle_speed;

} gimbal_motor_second_order_linear_controller_t;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{

    const motor_measure_t *gimbal_motor_measure;

    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; // rad
    fp32 min_relative_angle; // rad
    fp32 relative_chassis_angle;

    fp32 relative_angle;     // rad
    fp32 relative_angle_set; // rad
    fp32 absolute_angle;     // rad
    fp32 absolute_angle_set; // rad
    fp32 motor_gyro;         // rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

    ExtU_stm32 stm32_U_YAW;
    ExtU_stm32 stm32_U_PITCH;

    // 二阶线性控制器
    gimbal_motor_second_order_linear_controller_t YAW_SOLC;

    gimbal_motor_second_order_linear_controller_t PITCH_SOLC;

} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    // 获取视觉上位机数据
    const gimbal_vision_control_t *gimbal_vision_point;

    // 场地yaw轴正方向
    fp32 yaw_positive_direction;

    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    pid_type_def yaw_vision_pid;
    pid_type_def pitch_vision_pid;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
    const INS_t *gimbal_INS_point;
    const ext_robot_state_t *robot_state;

    fp32 gimbal_back_init_angle;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
} gimbal_control_t;

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */

extern void gimbal_task(void const *pvParameters);

/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
 * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
#endif

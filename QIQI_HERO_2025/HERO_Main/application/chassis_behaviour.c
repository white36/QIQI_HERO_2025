/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       chassis_behaviour.c/h
* @brief      according to remote control, change the chassis behaviour.
*             根据遥控器的值，决定底盘行为。
* @note
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-26-2018     RM              1. done
*  V1.1.0     Nov-11-2019     RM              1. add some annotation
*
@verbatim
==============================================================================
  add a chassis behaviour mode
  1. in chassis_behaviour.h , add a new behaviour name in chassis_behaviour
  erum
  {
      ...
      ...
      CHASSIS_XXX_XXX, // new add
  }chassis_behaviour_e,
  2. implement new function. chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
      "vx, vy, wz" param is chassis movement contorl input.
      first param: 'vx' usually means  vertical speed,
          positive value means forward speed, negative value means backward speed.
      second param: 'vy' usually means horizotal speed,
          positive value means letf speed, negative value means right speed
      third param: 'wz' can be rotation speed set or angle set,

      in this new function, you can assign speed to "vx","vy",and "wz",as your wish
  3.  in "chassis_behaviour_mode_set" function, add new logical judgement to assign CHASSIS_XXX_XXX to  "chassis_behaviour_mode" variable,
      and in the last of the function, add "else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)"
      choose a chassis control mode.
      four mode:
      CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control relative angle
          between chassis and gimbal. you can name third param to 'xxx_angle_set' other than 'wz'
      CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control absolute angle calculated by gyro
          you can name third param to 'xxx_angle_set.
      CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy' are speed control, 'wz' is rotation speed control.
      CHASSIS_VECTOR_RAW : will use 'vx' 'vy' and 'wz'  to linearly calculate four wheel current set,
          current set will be derectly sent to can bus.
  4. in the last of "chassis_behaviour_control_set" function, add
      else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
      {
          chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
      }


  如果要添加一个新的行为模式
  1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
  erum
  {
      ...
      ...
      CHASSIS_XXX_XXX, // 新添加的
  }chassis_behaviour_e,

  2. 实现一个新的函数 chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
      "vx,vy,wz" 参数是底盘运动控制输入量
      第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
      第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
      第三个参数: 'wz' 可能是角度控制或者旋转速度控制
      在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
  3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
      在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
      4种:
      CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
      你可以命名成"xxx_angle_set"而不是'wz'
      CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
      你可以命名成"xxx_angle_set"
      CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
      CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
  4.  在"chassis_behaviour_control_set" 函数的最后，添加
      else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
      {
          chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
      }
==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2019 DJI****************************
*/

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "referee.h"

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis control mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
 *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
 *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle[-PI, PI]
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
 *                 set value will be sent to can bus.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
/**
 * @brief          底盘陀螺模式
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_bpin_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘摇摆模式
 * @param[in]      angle_set 控制底盘与云台的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_twist_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          一键掉头模式
 * @param[in]      angle_set 控制底盘与云台的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_back_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
// highlight, the variable chassis behaviour mode
// 留意，这个底盘行为模式变量
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
int8_t turn_flags = 0; // 就近对位 回头flag
extern chassis_move_t chassis_move;
extern int8_t QA, EA;
int MODE = 0;
extern int8_t BPIN, FOLLOW;

/**
 * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
 * @param[in]      chassis_move_mode: chassis data
 * @retval         none
 */
/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    static uint16_t last_key_G = 0;
    static uint16_t last_key_E = 0;
    static uint16_t last_key_V = 0;
    static int16_t move_flag = 0;
    static bool_t turn_twist_flag = 0;
    static int chassis_follow = 0; // 底盘两个键鼠模式
    MODE = 0;                      // 遥控器小陀螺标志位

	/* 按键调整模式 */
    if (!last_key_G && chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_G)
    {
        move_flag = !move_flag;
    }
	// 开启不跟随模式
    if (!last_key_V && chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_V && !(chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        chassis_follow = !chassis_follow;
    }


    // remote control  set chassis behaviour mode
    // 遥控器设置模式
    if (gimbal_behaviour == GIMBAL_TURN_ROUND) // 一键掉头时不进入其他模式
    {
        chassis_behaviour_mode = CHASSIS_TURN_ROUND;
    }
	else if(gimbal_behaviour == GIMBAL_REMOTE_FIRE)
	{
		chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
	}
    else // 非一键掉头
    {
        if (!switch_is_down(chassis_move_mode->chassis_RC->rc.s[1])) // 左开启
        {
            if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[0]))
            {
                chassis_behaviour_mode = CHASSIS_BPIN; // 底盘陀螺
                MODE = 1;                              // 证明是遥控器小陀螺
            }
            else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[0]))
            {
                chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW; // 底盘跟随 
            }
            else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[0]))
            {
                chassis_behaviour_mode = CHASSIS_ZERO_FORCE; // 底盘无力
            }
        }
        else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[1]) && move_flag) // 左关，键鼠操作
        {
            if (chassis_follow == 0)
            {
                FOLLOW = 1;
                // 开陀螺
                if (!last_key_E && chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)
                {
                    turn_twist_flag = !turn_twist_flag;
                }

                if (turn_twist_flag == 0)
                {
                    chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
                    BPIN = 0;
                }
                else if (turn_twist_flag == 1)
                {
                    chassis_behaviour_mode = CHASSIS_BPIN;
                    BPIN = 1;
                }
            }
            else if (chassis_follow == 1)
            {
                FOLLOW = 0;
                chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
            }
        }
    }

        // when gimbal in some mode, such as init mode, chassis must's move
        // 当云台在某些模式下，像初始化， 底盘不动
        if (gimbal_cmd_to_chassis_stop())
        {
            chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
        }

        // accord to beheviour mode, choose chassis control mode
        // 根据行为模式选择一个底盘控制模式
        if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
        }
        else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
        }
        else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) // 跟随云台
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
        }
        else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
        }
        else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
        }
        else if (chassis_behaviour_mode == CHASSIS_OPEN)
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
        }
        else if (chassis_behaviour_mode == CHASSIS_BPIN) // 底盘陀螺
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_BPIN;
        }
        else if (chassis_behaviour_mode == CHASSIS_TWIST) // 底盘摇摆
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TWIST;
        }
        else if (chassis_behaviour_mode == CHASSIS_TURN_ROUND) // 一键掉头
        {
            chassis_move_mode->chassis_mode = CHASSIS_VECTOR_TURN_ROUND;
        }

        last_key_G = chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_G;
        last_key_E = chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_E;
        last_key_V = chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_V;
    }

/**
 * @brief          set control set-point. three movement param, according to difference control mode,
 *                 will control corresponding movement.in the function, usually call different control function.
 * @param[out]     vx_set, usually controls vertical speed.
 * @param[out]     vy_set, usually controls horizotal speed.
 * @param[out]     wz_set, usually controls rotation speed.
 * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
 * @retval         none
 */
/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) // 跟随云台
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW) // 比赛用 底盘不跟随，可能用于射击
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_BPIN) // 大陀螺
    {
        chassis_bpin_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_TWIST) // 底盘摇摆
    {
        chassis_twist_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_TURN_ROUND) // 一键掉头
    {
        chassis_back_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
 *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    // channel value and keyboard value change to speed set-point, in general
    // 遥控器的通道值以及键盘按键 赋值一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    fp32 angle_set_channel = 0;

	if (chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle > 1.74f)
	{
		turn_flags = 1;
		angle_set_channel = 3.14f;
	}
	else if (chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle < -1.74f)
	{
		angle_set_channel = -3.14f;
		turn_flags = 0;
	}

    *angle_set = angle_set_channel;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
 *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle[-PI, PI]
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    fp32 wz_set_channel = 0;
    //    if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q)
    //    {
    //        wz_set_channel = 5.0f;
    //    }
    //    else if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)
    //    {
    //        wz_set_channel = -5.0f;
    //    }
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] + wz_set_channel;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
 *                 set value will be sent to can bus.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}
/**
 * @brief          底盘陀螺模式
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_bpin_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	*vx_set*=1.4f;
	*vy_set*=1.4f;
	*wz_set =35.0f;
}

/**
 * @brief          底盘摇摆模式
 * @param[in]      angle_set 控制底盘与云台的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_twist_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    fp32 angle_set_channel = 0;

    // 初始化摇摆角度
    if (chassis_move_rc_to_vector->twist_init_flag == 0)
    {
        chassis_move_rc_to_vector->change_twist_flag = 1;
        chassis_move_rc_to_vector->twist_init_flag = 1;
    }
    // 摇摆中相对角左右交替
    if (chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle > 0.55f)
    {
        chassis_move_rc_to_vector->change_twist_flag = 2;
    }
    else if (chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle < -0.45f)
    {
        chassis_move_rc_to_vector->change_twist_flag = 1;
    }

    if (chassis_move_rc_to_vector->change_twist_flag == 1)
    {
        angle_set_channel = 0.60;
    }
    else if (chassis_move_rc_to_vector->change_twist_flag == 2)
    {
        angle_set_channel = -0.40;
    }

    // 运算结束赋值
    *angle_set = angle_set_channel;
}

/**
 * @brief          一键掉头模式
 * @param[in]      angle_set 控制底盘与云台的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_back_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    // channel value and keyboard value change to speed set-point, in general
    // 遥控器的通道值以及键盘按键 赋值一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    fp32 angle_set_channel = 0;
	//未启用相对云台角度设定值,角速度后续直接置零
    *angle_set = angle_set_channel;
}

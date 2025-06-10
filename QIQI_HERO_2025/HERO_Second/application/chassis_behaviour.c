/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       chassis_behaviour.c/h
* @brief      according to remote control, change the chassis behaviour.
*             ����ң������ֵ������������Ϊ��
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


  ���Ҫ���һ���µ���Ϊģʽ
  1.���ȣ���chassis_behaviour.h�ļ��У� ���һ������Ϊ������ chassis_behaviour_e
  erum
  {
      ...
      ...
      CHASSIS_XXX_XXX, // ����ӵ�
  }chassis_behaviour_e,

  2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
      "vx,vy,wz" �����ǵ����˶�����������
      ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
      �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
      ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
      ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
  3.  ��"chassis_behaviour_mode_set"��������У�����µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
      �ں���������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
      4��:
      CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
      �����������"xxx_angle_set"������'wz'
      CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
      �����������"xxx_angle_set"
      CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
      CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
  4.  ��"chassis_behaviour_control_set" ������������
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


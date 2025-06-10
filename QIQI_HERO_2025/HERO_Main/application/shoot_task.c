/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot_task.h"
#include "chassis_task.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_servo_pwm.h"
#include "bsp_pwm.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "tim.h"
#include "stm32.h"
#include "vision_task.h"

static void shoot_init(shoot_control_t *shoot_init);
/**
 * @brief          图传抬升机构 舵机控制
 * @param[in]      void
 * @retval         void
 */
static void servo_pwm_control(shoot_control_t *pwm_control);
/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(shoot_control_t *feedback_update);
/**
 * @brief          由热量计算出剩余可发弹数量
 * @param[in]      void
 * @retval         void
 */
static void remain_fire_count_calc(shoot_control_t *remain_count_control);
/**
 * @brief          手动调整摩擦轮转速
 * @param[out]     wheel_speed_control:"shoot_control_t"变量指针.
 * @retval         none
 */
static void adjust_wheel_speed(shoot_control_t *wheel_speed_control);
/**
 * @brief          摩擦轮模式切换
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(shoot_control_t *set_mode);
/**
 * @brief          电机速度计算
 * @param[in]      void
 * @retval         void
 */
static void shoot_control_loop(shoot_control_t *control_loop);
/**
 * @brief          卡弹拨盘回拨
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_block_control(shoot_control_t *block_control);

// 开启发弹摩擦轮
#define shoot_fric(speed, add)                                    \
    do                                                            \
    {                                                             \
        if (speed == 0)                                           \
        {                                                         \
            shoot_control.fric_left_speed_set = 0;                \
            shoot_control.fric_right_speed_set = 0;               \
            shoot_control.fric_bleft_speed_set = 0;               \
            shoot_control.fric_bright_speed_set = 0;              \
        }                                                         \
        else                                                      \
        {                                                         \
            shoot_control.fric_left_speed_set = speed;            \
            shoot_control.fric_right_speed_set = -speed;          \
            shoot_control.fric_bleft_speed_set = (speed + add);   \
            shoot_control.fric_bright_speed_set = -(speed + add); \
        }                                                         \
    } while (0)
// 调参宏定义
#define FIRST_SPEED_1 5200
#define SPEED_K 0.75f
#define ADD_SPEED_1 SPEED_K *FIRST_SPEED_1 - FIRST_SPEED_1

#define TURN_TRIGGER_TIME 30  // 拨弹时间
#define TURN_TRIGGER_SPEED 40 // 拨弹速度

#define TRIGGER_COMMON_SPEED 7.4f // 常规供弹速度
// 其他宏定义
#define trigger_motor(speed) shoot_control.trigger_speed_set = speed // 开启拨弹电机
// 微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

extern int8_t STUCK;
int16_t limitnb = 0, a = 0, hh = 0;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
shoot_control_t shoot_control; // 射击数据
int16_t s_can_set_current = 0, left_can_set_current = 0, right_can_set_current = 0, bleft_can_set_current = 0, bright_can_set_current = 0, trigger_can_set_current = 0;
fp32 last_speed;
uint8_t shoot_allow_flag = 0, success_flag = 0;
fp32 q, w, e, r;
extern int16_t TRIGGER_current;
int8_t REMOTE_FIRE_MODE = 0;

/**
 * @brief          射击任务
 * @param[in]      void
 * @retval         返回can控制值
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    shoot_init(&shoot_control);
    while (1)
    {
        shoot_control.SHOOT_xTickCount = xTaskGetTickCount();
        servo_pwm_control(&shoot_control); // 图传舵机控制
        shoot_set_mode(&shoot_control);
        shoot_feedback_update(&shoot_control);
        shoot_control_loop(&shoot_control); // 设置发弹控制量

        if (toe_is_error(DBUS_TOE))
        {
            CAN_cmd_shoot(0, 0, 0, 0);
            TRIGGER_current = 0;
        }
        else
        {
            CAN_cmd_shoot(left_can_set_current, right_can_set_current, bleft_can_set_current, bright_can_set_current);
            TRIGGER_current = trigger_can_set_current;
        }
        vTaskDelay(SHOOT_CONTROL_TIME_MS);
    }
}

/**
 * @brief          初始化"shoot_control"变量.
 * @param[out]     init:"shoot_control"变量指针.
 * @retval         none
 */
static void shoot_init(shoot_control_t *shoot_init)
{
    memset(shoot_init, 0, sizeof(shoot_control_t));
    // 电机PID参数初始化
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 fric_left_pid[3] = {FRIC_LEFT_MOTOR_SPEED_PID_KP, FRIC_LEFT_MOTOR_SPEED_PID_KI, FRIC_LEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_right_pid[3] = {FRIC_RIGHT_MOTOR_SPEED_PID_KP, FRIC_RIGHT_MOTOR_SPEED_PID_KI, FRIC_RIGHT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bleft_pid[3] = {FRIC_BLEFT_MOTOR_SPEED_PID_KP, FRIC_BLEFT_MOTOR_SPEED_PID_KI, FRIC_BLEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bright_pid[3] = {FRIC_BRIGHT_MOTOR_SPEED_PID_KP, FRIC_BRIGHT_MOTOR_SPEED_PID_KI, FRIC_BRIGHT_MOTOR_SPEED_PID_KD};

    shoot_init->robot_state = get_robot_status_point();
    shoot_init->shoot_data = get_shoot_data_point();
    // 遥控器指针
    shoot_init->shoot_rc = get_remote_control_point();
    // 电机指针
    shoot_init->trigger_motor_measure = get_trigger_motor_measure_point();
    shoot_init->fric_left_motor_measure = get_can_3508_left_measure_point();
    shoot_init->fric_right_motor_measure = get_can_3508_right_measure_point();
    shoot_init->fric_bleft_motor_measure = get_can_3508_bleft_measure_point();
    shoot_init->fric_bright_motor_measure = get_can_3508_bright_measure_point();
    // 初始化PID
    PID_init(&shoot_init->trigger_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_left_pid, PID_POSITION, fric_left_pid, FRIC_LEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_LEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_right_pid, PID_POSITION, fric_right_pid, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_bleft_pid, PID_POSITION, fric_bleft_pid, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_init->fric_bright_pid, PID_POSITION, fric_bright_pid, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_IOUT);

    // 更新数据
    shoot_init->shoot_time = 150;

    shoot_init->pwm_first_up = PWM_FIRST_UP;
    shoot_init->pwm_first_down = PWM_FIRST_DOWN;
    shoot_init->pwm_mid_up = PWM_MID_UP;
    shoot_init->pwm_mid_down = PWM_MID_DOWN;
    shoot_init->pwm_last_up = PWM_LAST_UP;
    shoot_init->pwm_last_down = PWM_LAST_DOWN;

    shoot_init->turn_trigger_time = TURN_TRIGGER_TIME;
    shoot_init->turn_trigger_speed = TURN_TRIGGER_SPEED;
    shoot_init->first_speed = FIRST_SPEED_1;
    shoot_init->speed_k = SPEED_K;
    shoot_init->add_speed = ADD_SPEED_1;
}

/**
 * @brief          控制图传抬升机构的舵机
 * @param[out]     pwm_control:"shoot_control_t"变量指针.
 * @retval         none
 */
static void servo_pwm_control(shoot_control_t *pwm_control)
{
    static int8_t servo_flag = -100;
    static TickType_t servo_last_tick = 0;
    static uint16_t KEY_CTRL_C = 0;
    static uint16_t KEY_CTRL_Z = 0;
    static uint16_t last_KEY_CTRL_C = 0;
    static uint16_t last_KEY_CTRL_Z = 0;
    static uint16_t last_KEY_X = 0;

    KEY_CTRL_C = pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    KEY_CTRL_Z = pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    // CTRL+C:抬高 CTRL+Z:降低
    if (!last_KEY_CTRL_C && KEY_CTRL_C)
    {
        pwm_control->pwm_mid_up -= 25;
    }
    else if (!last_KEY_CTRL_Z && KEY_CTRL_Z)
    {
        pwm_control->pwm_mid_up += 25;
    }
    // 舵机pwm赋值
    if (servo_flag == -100)
    {
        // 非运动状态刷新pwm值,为了配合按键更改pwm ,此顺序为运动顺序
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_down);   // 中关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_down);  // 底座关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_down); // 倍镜
        if (!last_KEY_X && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = 1;
        }
    }
    else if (servo_flag == 100)
    {
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_up);  // 底座关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_up);   // 中关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_up); // 倍镜
        if (!last_KEY_X && pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = -1;
        }
    }
    /* 抬升 */
    if (servo_flag == 1)
    {
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_up); // 底座关节
        servo_last_tick = xTaskGetTickCount();
        servo_flag = 2;

        // 开启吊射模式标志位
        REMOTE_FIRE_MODE = 1;
    }
    else if (servo_flag == 2)
    {
        if (xTaskGetTickCount() - servo_last_tick > 150)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_up);   // 中关节
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_up); // 倍镜
            servo_flag = 100;                                              // 重置状态
        }
    }
    /* 下降 */
    if (servo_flag == -1)
    {
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, pwm_control->pwm_mid_down); // 中关节
        servo_last_tick = xTaskGetTickCount();
        servo_flag = -2;

        // 关闭吊射模式标志位
        REMOTE_FIRE_MODE = 0;
    }
    else if (servo_flag == -2)
    {
        if (xTaskGetTickCount() - servo_last_tick > 220)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_4, pwm_control->pwm_last_down);  // 底座关节
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, pwm_control->pwm_first_down); // 倍镜
            servo_flag = -100;                                               // 重置状态
        }
    }

    // 加限幅

    last_KEY_X = pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X;
    last_KEY_CTRL_C = (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C) && (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
    last_KEY_CTRL_Z = (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z) && (pwm_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
}

/**
 * @brief          射击数据更新
 * @param[out]     feedback_update:"shoot_control_t"变量指针.
 * @retval         none
 */
static void shoot_feedback_update(shoot_control_t *feedback_update)
{
    feedback_update->trigger_speed = feedback_update->trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

    // 电机速度更新
    shoot_control.fric_left_speed = shoot_control.fric_left_motor_measure->speed_rpm;
    shoot_control.fric_right_speed = shoot_control.fric_right_motor_measure->speed_rpm;
    shoot_control.fric_bleft_speed = shoot_control.fric_bleft_motor_measure->speed_rpm;
    shoot_control.fric_bright_speed = shoot_control.fric_bright_motor_measure->speed_rpm;

    // 观测微动开关状态
    if (BUTTEN_TRIG_PIN)
    {
        feedback_update->key_pin = 1;
    }
    else
    {
        feedback_update->key_pin = 0;
    }

    // 更新可发弹数量
    remain_fire_count_calc(feedback_update);

    // 可手动调节摩擦轮转速(后续优化结构时放在set_control吧)
    adjust_wheel_speed(feedback_update);

    // 记录鼠标按键
    feedback_update->last_press_l = feedback_update->press_l;
    feedback_update->last_press_r = feedback_update->press_r;
    feedback_update->press_l = feedback_update->shoot_rc->mouse.press_l;
    feedback_update->press_r = feedback_update->shoot_rc->mouse.press_r;
}

/**
 * @brief          由热量计算出剩余可发弹数量
 * @param[out]     remain_count_control:"shoot_control_t"变量指针.
 * @retval         none
 */
static void remain_fire_count_calc(shoot_control_t *remain_count_control)
{
    static uint16_t limit;
    static uint16_t heat;
    limit = robot_state.shooter_barrel_heat_limit;
    heat = power_heat_data.shooter_42mm_barrel_heat;
    // 计算可击打弹丸数, 防止裁判系统延迟导致超热量 将结果作为可发射标志位
    remain_count_control->remain_heat = limit - heat;
    if (remain_count_control->remain_heat < 100)
    {
        remain_count_control->remain_count = 0;
    }
    else if (remain_count_control->remain_heat >= 100 && remain_count_control->remain_heat < 200)
    {
        remain_count_control->remain_count = 1;
    }
    else if (remain_count_control->remain_heat >= 200 && remain_count_control->remain_heat < 300)
    {
        remain_count_control->remain_count = 2;
    }
    else if (remain_count_control->remain_heat >= 300 && remain_count_control->remain_heat < 400)
    {
        remain_count_control->remain_count = 3;
    }
    else if (remain_count_control->remain_heat >= 400 && remain_count_control->remain_heat < 500)
    {
        remain_count_control->remain_count = 4;
    }
    else if (remain_count_control->remain_heat == 500)
    {
        remain_count_control->remain_count = 5;
    }
    else
    {
        remain_count_control->remain_count = 0;
    }
    // 每到一个热量阈值,更新可击打弹丸数
    if (remain_count_control->remain_count != remain_count_control->last_remain_count)
    {
        remain_count_control->remain_fire_count = remain_count_control->remain_count;
    }

    // 修复由延迟导致的 单发时识别不到热量到达阈值的瞬间
    if (remain_count_control->remain_count > remain_count_control->remain_fire_count)
    {
        if (remain_count_control->heat_delay_start == 0)
        {
            remain_count_control->heat_delay_time = remain_count_control->SHOOT_xTickCount;
            remain_count_control->heat_delay_start = 1;
        }
        if (remain_count_control->heat_delay_start == 1)
        {
            if (remain_count_control->SHOOT_xTickCount - remain_count_control->heat_delay_time > 300)
            {
                remain_count_control->remain_fire_count = remain_count_control->remain_count;
            }
        }
    }
    else
    {
        remain_count_control->heat_delay_start = 0;
    }

    // 记录上一时刻数据
    remain_count_control->last_remain_count = remain_count_control->remain_count;
}
/**
 * @brief          手动调整摩擦轮转速
 * @param[out]     wheel_speed_control:"shoot_control_t"变量指针.
 * @retval         none
 */
static void adjust_wheel_speed(shoot_control_t *wheel_speed_control)
{
    static uint16_t KEY_CTRL_V = 0;
    static uint16_t KEY_CTRL_B = 0;
    static uint16_t last_KEY_CTRL_V = 0;
    static uint16_t last_KEY_CTRL_B = 0;
    int16_t first_speed_adjst = 0;

    KEY_CTRL_V = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_V && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    KEY_CTRL_B = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_B && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    /* CTRL+V:前轮速度 CTRL+B:差速 */
    // 记录按下时间
    if (!last_KEY_CTRL_V && KEY_CTRL_V)
    {
        wheel_speed_control->CTRL_V_press_time = wheel_speed_control->SHOOT_xTickCount;
    }
    if (!last_KEY_CTRL_B && KEY_CTRL_B)
    {
        wheel_speed_control->CTRL_B_press_time = wheel_speed_control->SHOOT_xTickCount;
    }
    // 短按减速, 长按加速
    // 松开后进判断
    if (last_KEY_CTRL_V && !KEY_CTRL_V)
    {
        if (wheel_speed_control->SHOOT_xTickCount - wheel_speed_control->CTRL_V_press_time < 500) // 短按
        {
            first_speed_adjst = -25;
        }
        else if (wheel_speed_control->SHOOT_xTickCount - wheel_speed_control->CTRL_V_press_time >= 500) // 长按
        {
            first_speed_adjst = +25;
        }
    }

    // 运算后赋值增量
    wheel_speed_control->first_speed += first_speed_adjst;
    // 根据first和比例一直算add
    wheel_speed_control->add_speed = wheel_speed_control->speed_k * wheel_speed_control->first_speed - wheel_speed_control->first_speed;

    last_KEY_CTRL_V = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_V && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
    last_KEY_CTRL_B = wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_B && wheel_speed_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL;
}

/**
 * @brief          射击状态机设置
 * @param[out]     set_mode:"shoot_control_t"变量指针.
 * @retval         none
 */
extern int8_t R; // 开启摩擦轮
int16_t x = 0;
int s = 2000, l;
bool_t fireOK;
int8_t datatx_OK = 0;
int32_t datatx_time = 0;
static void shoot_set_mode(shoot_control_t *set_mode)
{
    static uint16_t last_KEY_R = 0;
    static uint16_t CTRL_R_time = 0;

    // 键盘控制长按R建开启摩擦轮
    if (!last_KEY_R && set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R)
    {
        set_mode->R_press_time = set_mode->SHOOT_xTickCount;
    }
    // 松开后进判断
    if (last_KEY_R && !(set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R))
    {
        if (set_mode->SHOOT_xTickCount - set_mode->R_press_time > 500) // 长按
        {
            R = !R;
        }
    }
    last_KEY_R = set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R;

    if ((switch_is_up(set_mode->shoot_rc->rc.s[1]) || R))
    {
        STUCK = 1; // 卡弹UI置1 后续清零
        // 根据裁判系统 控制弹速
        set_mode->add_speed = set_mode->speed_k * set_mode->first_speed - set_mode->first_speed;
        shoot_fric(set_mode->first_speed, set_mode->add_speed);

        // 卡弹判断及回拨逻辑
        trigger_motor_block_control(set_mode);

        if (set_mode->stuck_flag == 0 && set_mode->number_stuck_flag == 0) // 不卡弹 则进入正常发弹逻辑
        {
            STUCK = 0; // 不卡弹清零卡弹UI

            // 拨弹
            if (set_mode->stuck_trigger_flag == 1)
            {
                trigger_motor(TRIGGER_COMMON_SPEED + 1.0f); // 普通卡弹后可以使用更快供弹速度
            }
            else if (BUTTEN_TRIG_PIN == RESET) // 触发低电平
            {
                trigger_motor(0.0f);
            }
            else if (BUTTEN_TRIG_PIN == SET) // 默认高电平
            {
                trigger_motor(TRIGGER_COMMON_SPEED); // 供弹速度 // 小弹5.0
            }

            if (set_mode->stuck_trigger_flag == 1 && BUTTEN_TRIG_PIN == RESET) // 快速供弹后清除标志位
            {
                set_mode->stuck_trigger_flag = 0;
            }

            // 应急拨弹 防止特殊情况
            if ((set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL) && (set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R))
            {
                CTRL_R_time++;
                if (CTRL_R_time > 200 && (set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL) && (set_mode->shoot_rc->key.v & KEY_PRESSED_OFFSET_R))
                {
                    trigger_motor(TRIGGER_COMMON_SPEED);
                }
            }
            else
            {
                CTRL_R_time = 0;
            }

            // 发弹时记录打弹个数 同时可击打弹丸数减1
            if (set_mode->last_butten_trig_pin == RESET && BUTTEN_TRIG_PIN == SET)
            {
                set_mode->bullet_number++;
                set_mode->remain_fire_count--;

                // 用于采集数据
                datatx_time = 0;
                set_mode->remain_shoot_time = set_mode->SHOOT_xTickCount - set_mode->start_shoot_time;
            }
            // 数据采集
            if (datatx_time == 10)
            {
                datatx_OK = 1;
            }
            else
            {
                datatx_OK = 0;
            }
            datatx_time++;
            if (datatx_time > 2147483645)
            {
                datatx_time = 11;
            }

            // 发射模式时将shoot_flag置零, 即允许发射下颗弹丸
            if (switch_is_up(set_mode->shoot_rc->rc.s[1]))
            {
                if (set_mode->shoot_rc->rc.ch[4] < 120)
                {
                    set_mode->shoot_flag = 0;
                }
            }
            else if (R)
            {
                if (set_mode->press_l == 0)
                {
                    set_mode->shoot_flag = 0;
                }
            }

            if (set_mode->press_r) // 开自瞄时,不会手动发弹(因为云台并非一直是自瞄模式)
            {
                // 发弹
                if ((set_mode->shoot_flag == 0                                                                                             // 条件1
                     && ((set_mode->shoot_rc->rc.ch[4] > 600 || set_mode->press_l)                                                         // 并列条件2:手动触发发弹标志
                         && (vision_control.shoot_vision_control.shoot_command == SHOOT_ATTACK && gimbal_behaviour == GIMBAL_AUTO_ATTACK)) // 并列条件2:自瞄触发发弹标志
                     && BUTTEN_TRIG_PIN == RESET)                                                                                          // 条件3
                    && set_mode->remain_fire_count > 0)                                                                                    // 最外层热量限制
                {
                    set_mode->shoot_flag = 1;
                    set_mode->last_bullet_number = set_mode->bullet_number;
                    set_mode->start_shoot_time = set_mode->SHOOT_xTickCount; // debug测量发弹延迟
                }
            }
            else
            {
                // 发弹
                if ((set_mode->shoot_flag == 0                                    // 条件1
                     && (set_mode->shoot_rc->rc.ch[4] > 600 || set_mode->press_l) // 并列条件2:自瞄触发发弹标志
                     && BUTTEN_TRIG_PIN == RESET)                                 // 条件3
                    && set_mode->remain_fire_count > 0)                           // 最外层热量限制
                {
                    set_mode->shoot_flag = 1;
                    set_mode->last_bullet_number = set_mode->bullet_number;
                    set_mode->start_shoot_time = set_mode->SHOOT_xTickCount;
                }
            }
        }
        else if (set_mode->stuck_flag == 1 || set_mode->number_stuck_flag == 1) // 卡弹
        {
            trigger_motor(-5.0f); // 卡弹回拨速度
        }
        else if (set_mode->stuck_flag == 2 || set_mode->number_stuck_flag == 2) // 卡弹回拨后等待
        {
            trigger_motor(0); // 拨弹盘等待速度
        }
        else if (set_mode->number_stuck_flag == 3) // 特殊拨弹第三步快速拨
        {
            trigger_motor(5.0f);
        }
        else if (set_mode->number_stuck_flag == 4) // 特殊拨弹第四步快速拨
        {
            trigger_motor(3.0f);
        }
    } // switch_is_up内容结束
    else
    {
        shoot_fric(0, 0);
        trigger_motor(0);
    }

    // 发弹信号
    if (set_mode->last_shoot_flag == 0 && set_mode->shoot_flag == 1)
    {
        set_mode->shoot_time = 0;
    }
    // 拨弹
    if (set_mode->shoot_time < set_mode->turn_trigger_time) // 拨弹轮时间
    {
        trigger_motor(set_mode->turn_trigger_speed); // 拨弹盘速度
    }
    else if (set_mode->shoot_time == (set_mode->turn_trigger_time))
    {
        trigger_motor(0);
    }

    set_mode->shoot_time++;

    if (set_mode->shoot_time >= 210)
        set_mode->shoot_time = 210;

    set_mode->last_butten_trig_pin = BUTTEN_TRIG_PIN;
    set_mode->last_shoot_flag = set_mode->shoot_flag;
}

/**
 * @brief          电机速度计算
 * @param[out]     control_loop:"shoot_control_t"变量指针.
 * @retval         none
 */
static void shoot_control_loop(shoot_control_t *control_loop)
{
    // 计算pid
    PID_calc(&control_loop->trigger_pid, control_loop->trigger_speed, control_loop->trigger_speed_set);
    // 四摩擦轮pid计算
    PID_calc(&control_loop->fric_left_pid, control_loop->fric_left_speed, control_loop->fric_left_speed_set);
    PID_calc(&control_loop->fric_right_pid, control_loop->fric_right_speed, control_loop->fric_right_speed_set);
    PID_calc(&control_loop->fric_bleft_pid, control_loop->fric_bleft_speed, control_loop->fric_bleft_speed_set);
    PID_calc(&control_loop->fric_bright_pid, control_loop->fric_bright_speed, control_loop->fric_bright_speed_set);

    trigger_can_set_current = control_loop->trigger_pid.out;

    left_can_set_current = control_loop->fric_left_pid.out;
    right_can_set_current = control_loop->fric_right_pid.out;
    bleft_can_set_current = control_loop->fric_bleft_pid.out;
    bright_can_set_current = control_loop->fric_bright_pid.out;
}

/**
 * @brief          拨弹盘回拨函数
 * @param[out]     block_control:"shoot_control_t"变量指针.
 * @retval         none
 */
static void trigger_motor_block_control(shoot_control_t *block_control)
{
    if (trigger_can_set_current >= TRIGGER_COMMON_SPEED * 1800.0f - 600.0f)
    {
        // 判断1 供弹时供不上去
        block_control->block_time++;
        if (block_control->block_time > 600) // 检测时间//800
        {
            block_control->stuck_flag = 1;
            block_control->block_time = 0;
        }
    }
    else // 不卡弹时重置计时
    {
        block_control->block_time = 0;
    }

    // 卡弹回拨时间安排（2步）
    if (block_control->stuck_flag == 1)
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 80) // 回拨时间//40
        {
            block_control->reverse_time = 0;
            block_control->stuck_flag = 2;
        }
    }
    else if (block_control->stuck_flag == 2) // 回拨后等待弹丸落回拨弹盘
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 100) // 等待时间
        {
            block_control->reverse_time = 0;
            block_control->stuck_flag = 0;
            block_control->stuck_trigger_flag = 1;
        }
    }

    // 特殊卡弹回拨
    if (block_control->number_stuck_flag == 1)
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 50)
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 2;
        }
    }
    else if (block_control->number_stuck_flag == 2) // 回拨后等待弹丸落回拨弹盘
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 70) // 等待时间
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 3;
        }
    }
    else if (block_control->number_stuck_flag == 3) // 回拨后等待弹丸落回拨弹盘
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 80) // 快上弹时间
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 4;
        }
    }
    else if (block_control->number_stuck_flag == 4) // 回拨后等待弹丸落回拨弹盘
    {
        block_control->reverse_time++;
        if (block_control->reverse_time > 40) // 上弹时间
        {
            block_control->reverse_time = 0;
            block_control->number_stuck_flag = 0;
        }
    }
}

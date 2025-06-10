#include "shoot_task.h"
#include "chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
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

/**
 * @brief          射击任务初始化
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_init(shoot_control_t *shoot_control);

/**
 * @brief          图传抬升机构舵机控制
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
void servo_pwm_control(shoot_control_t *shoot_control);

/**
 * @brief          射击模式设置
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_set_mode(shoot_control_t *shoot_control);

/**
 * @brief          射击控制循环，计算电机速度
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_control_loop(shoot_control_t *shoot_control);

/**
 * @brief          射击反馈数据更新
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_feedback_update(shoot_control_t *shoot_control);

/**
 * @brief          卡弹拨盘回拨控制
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void trigger_motor_block_control(shoot_control_t *shoot_control);

#define shoot_fric(speed, add)                                    \
    do                                                            \
    {                                                             \
        if (speed == 0)                                           \
        {                                                         \
            shoot_control->fric_left_speed_set = 0;                \
            shoot_control->fric_right_speed_set = 0;               \
            shoot_control->fric_bleft_speed_set = 0;               \
            shoot_control->fric_bright_speed_set = 0;              \
        }                                                         \
        else                                                      \
        {                                                         \
            shoot_control->fric_left_speed_set = speed;            \
            shoot_control->fric_right_speed_set = -speed;          \
            shoot_control->fric_bleft_speed_set = (speed + add);   \
            shoot_control->fric_bright_speed_set = -(speed + add); \
        }                                                         \
    } while (0)

#define trigger_motor(speed) shoot_control->trigger_speed_set = speed
#define third_fric(speed) shoot_control->fric_b_speed_set = -speed

/**
 * @brief          射击任务主函数
 * @param[in]      pvParameters: FreeRTOS任务参数
 * @retval         void
 * @note           射击任务主循环，包含初始化、模式设置、控制循环等
 */
void shoot_task(void const *pvParameters)
{
    // 等待系统初始化完成
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 初始化射击控制
    shoot_init(&shoot_control);
    while (1)
    {
        // 舵机PWM控制
        servo_pwm_control(&shoot_control);
        // 设置射击模式
        shoot_set_mode(&shoot_control);
        // 更新射击反馈数据
        shoot_feedback_update(&shoot_control);
        // 执行射击控制循环
        shoot_control_loop(&shoot_control);

        // 检查遥控器通信状态
        if (toe_is_error(DBUS_TOE))
        {
            // 通信错误时停止所有电机
            CAN_cmd_shoot(0, 0, 0, 0);
            TRIGGER_current = 0;
        }
        // 检查发射机构电源状态
        else if (gimbal_control.robot_state->power_management_shooter_output == 0)
        {
            // 电源关闭时停止所有电机
            CAN_cmd_shoot(0, 0, 0, 0);
            TRIGGER_current = 0;
        }
        else
        {
            // 正常工作时发送电机控制命令
            CAN_cmd_shoot(left_can_set_current, right_can_set_current, bleft_can_set_current, bright_can_set_current);
            TRIGGER_current = trigger_can_set_current;
        }

        // 任务延时
        vTaskDelay(SHOOT_CONTROL_TIME_MS);
    }
}

/**
 * @brief          射击任务初始化
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_init(shoot_control_t *shoot_control)
{
    // 电机PID参数初始化
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 fric_b_pid[3] = {FRIC_S_MOTOR_SPEED_PID_KP, FRIC_S_MOTOR_SPEED_PID_KI, FRIC_S_MOTOR_SPEED_PID_KD};
    static const fp32 fric_left_pid[3] = {FRIC_LEFT_MOTOR_SPEED_PID_KP, FRIC_LEFT_MOTOR_SPEED_PID_KI, FRIC_LEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_right_pid[3] = {FRIC_RIGHT_MOTOR_SPEED_PID_KP, FRIC_RIGHT_MOTOR_SPEED_PID_KI, FRIC_RIGHT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bleft_pid[3] = {FRIC_BLEFT_MOTOR_SPEED_PID_KP, FRIC_BLEFT_MOTOR_SPEED_PID_KI, FRIC_BLEFT_MOTOR_SPEED_PID_KD};
    static const fp32 fric_bright_pid[3] = {FRIC_BRIGHT_MOTOR_SPEED_PID_KP, FRIC_BRIGHT_MOTOR_SPEED_PID_KI, FRIC_BRIGHT_MOTOR_SPEED_PID_KD};

    // 获取射击数据指针
    shoot_control->shoot_data = get_shoot_data_point();
    // 获取遥控器指针
    shoot_control->shoot_rc = get_remote_control_point();
    // 获取电机测量数据指针
    shoot_control->trigger_motor_measure = get_trigger_motor_measure_point();
    shoot_control->fric_b_motor_measure = get_can_2006_measure_point();
    shoot_control->fric_left_motor_measure = get_can_3508_left_measure_point();
    shoot_control->fric_right_motor_measure = get_can_3508_right_measure_point();
    shoot_control->fric_bleft_motor_measure = get_can_3508_bleft_measure_point();
    shoot_control->fric_bright_motor_measure = get_can_3508_bright_measure_point();

    // 初始化各个电机的PID控制器
    PID_init(&shoot_control->trigger_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control->bullet_pid, PID_POSITION, fric_b_pid, FRIC_S_MOTOR_SPEED_PID_MAX_OUT, FRIC_S_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_left_pid, PID_POSITION, fric_left_pid, FRIC_LEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_LEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_right_pid, PID_POSITION, fric_right_pid, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_bleft_pid, PID_POSITION, fric_bleft_pid, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BLEFT_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control->fric_bright_pid, PID_POSITION, fric_bright_pid, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_BRIGHT_MOTOR_SPEED_PID_MAX_IOUT);

    // 初始化射击控制参数
    shoot_control->shoot_flag = 0;
    shoot_control->shoot_continu_flag = 0;
    shoot_control->stuck_flag = 0;
    shoot_control->reverse_time = 0;
    shoot_control->shoot_time = 150;
    shoot_control->last_bullet_number = shoot_control->bullet_number = 0;
    shoot_control->stuck_trigger_flag = 0;

    // 初始化电机速度参数
    shoot_control->trigger_given_current = 0;
    shoot_control->trigger_speed = 0.0f;
    shoot_control->trigger_speed_set = 0.0f;
    shoot_control->fric_b_speed = 0.0f;
    shoot_control->fric_b_speed_set = 0.0f;
    shoot_control->fric_left_speed = 0.0f;
    shoot_control->fric_left_speed_set = 0.0f;
    shoot_control->fric_right_speed = 0.0f;
    shoot_control->fric_right_speed_set = 0.0f;
    shoot_control->fric_bleft_speed = 0.0f;
    shoot_control->fric_bleft_speed_set = 0.0f;
    shoot_control->fric_bright_speed = 0.0f;
    shoot_control->fric_bright_speed_set = 0.0f;

    // 初始化舵机PWM参数
    shoot_control->pwm_first_up = PWM_FIRST_UP;
    shoot_control->pwm_first_down = PWM_FIRST_DOWN;
    shoot_control->pwm_mid_up = PWM_MID_UP;
    shoot_control->pwm_mid_down = PWM_MID_DOWN;
    shoot_control->pwm_last_up = PWM_LAST_UP;
    shoot_control->pwm_last_down = PWM_LAST_DOWN;

    // 初始化弹丸计数和热量参数
    shoot_control->remain_heat = 0;
    shoot_control->remain_number = 0;
    shoot_control->last_remain_number = 0;
    shoot_control->remain_shoot_number = 0;

    // 初始化拨弹角度参数
    shoot_control->trigger_angle = 0;
    shoot_control->trigger_angle_set = shoot_control->trigger_angle;

    // 初始化拨弹时间和速度参数
    shoot_control->turn_trigger_time = TURN_TRIGGER_TIME;
    shoot_control->turn_trigger_speed = TURN_TRIGGER_SPEED;
    shoot_control->first_speed = FIRST_SPEED;
    shoot_control->add_speed = ADD_SPEED;
}

/**
 * @brief          图传抬升机构舵机控制
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
void servo_pwm_control(shoot_control_t *shoot_control)
{
    // 舵机状态标志
    static int8_t servo_flag = -100;
    // 舵机动作时间记录
    static TickType_t servo_last_tick = 0;
    // 按键状态记录
    static int16_t last_KEY_X = 0;
    static int16_t last_KEY_CTRL_C = 0;
    static int16_t last_KEY_CTRL_Z = 0;
    
    // CTRL+C:抬高 CTRL+Z:降低 
    if (!last_KEY_CTRL_C && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        shoot_control->pwm_mid_up -= 50;
    }
    else if (!last_KEY_CTRL_Z && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        shoot_control->pwm_mid_up += 50;
    }
    
    // 舵机pwm赋值
    if (servo_flag == -100)
    {
        // 非运动状态刷新pwm值,为了配合按键更改pwm ,此顺序为运动顺序
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_down); // 中关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_down); // 底座关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_down); // 倍镜
        if (!last_KEY_X && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = 1;
        }
    }
    else if (servo_flag == 100)
    {
        // 完全抬升状态
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_up); // 底座关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_up); // 中关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_up); // 倍镜
        if (!last_KEY_X && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
        {
            servo_flag = -1;
        }
    }

    /* 抬升过程 */
    if (servo_flag == 1)
    {
        // 第一步：抬升底座关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_up);
        servo_last_tick = xTaskGetTickCount();
        servo_flag = 2;
    }
    else if (servo_flag == 2)
    {
        // 第二步：等待150ms后抬升中关节和倍镜
        if (xTaskGetTickCount() - servo_last_tick > 150)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_up);
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_up);
            servo_flag = 100;
        }
    }

    /* 下降过程 */
    if (servo_flag == -1)
    {
        // 第一步：下降中关节
        TIM_Set_PWM(&htim1, TIM_CHANNEL_3, shoot_control->pwm_mid_down);
        servo_last_tick = xTaskGetTickCount();
        servo_flag = -2;
    }
    else if (servo_flag == -2)
    {
        // 第二步：等待220ms后下降底座关节和倍镜
        if (xTaskGetTickCount() - servo_last_tick > 220)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_4, shoot_control->pwm_last_down);
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, shoot_control->pwm_first_down);
            servo_flag = -100;
        }
    }

    // 更新按键状态
    last_KEY_X = shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_X;
    last_KEY_CTRL_C = (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
    last_KEY_CTRL_Z = (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_Z) && (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL);
}

/**
 * @brief          射击模式设置
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_set_mode(shoot_control_t *shoot_control)
{
    // 按键长按计时器
    static uint16_t press_R_time = 0;
    static uint16_t CTRL_time = 0;
    fp32 fric_speed;

    // 长按R键切换摩擦轮开关
    if (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_R && !(shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        press_R_time++;
    }
    if (press_R_time > 500)
    {
        R = !R;
        press_R_time = 0;
    }

    // 摩擦轮开启条件：拨杆向上或R键开启
    if ((switch_is_up(shoot_control->shoot_rc->rc.s[1]) || R))
    {
        STUCK = 1; // UI卡弹置1 后续清零
        
        // 卡弹判断及回拨逻辑
        trigger_motor_block_control(shoot_control);

        if (shoot_control->stuck_flag == 0 && shoot_control->number_stuck_flag == 0) // 不卡弹 则进入正常发弹逻辑
        {
            STUCK = 0; // 不卡弹清零卡弹UI
            
            // 拨弹控制
            if (shoot_control->stuck_trigger_flag == 1)
            {
                trigger_motor(6.0f); //普通卡弹后使用更快供弹速度
            }
            else if (BUTTEN_TRIG_PIN == RESET) // 触发低电平
            {
                trigger_motor(0.0f);
            }
            else if (BUTTEN_TRIG_PIN == SET) // 默认高电平
            {
                trigger_motor(3.2f); // 供弹速度
            }
            
            // 快速供弹后清除标志位
            if(shoot_control->stuck_trigger_flag == 1 && BUTTEN_TRIG_PIN == RESET)
            {
                shoot_control->stuck_trigger_flag = 0;
            }
            
            // 应急拨弹 防止特殊情况
            if (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_R)
            {
                CTRL_time ++;
                if (CTRL_time > 200 && shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL)
                {
                    trigger_motor(5.0f);
                }
            }
            else
            {
                CTRL_time = 0;
            }
            
            // 记录打弹个数 同时可击打弹丸数减1
            if (shoot_control->last_butten_trig_pin == RESET && BUTTEN_TRIG_PIN == SET)
            {
                shoot_control->bullet_number++;
                shoot_control->remain_shoot_number--;
                datatx_time = 0;
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
            datatx_time ++;
            if (datatx_time > 2147483645)
            {
                datatx_time = 11;
            }

            // 发射模式时将shoot_flag置零, 即可正常发射下颗弹丸
            if (switch_is_up(shoot_control->shoot_rc->rc.s[1]))
            {
                if (shoot_control->shoot_rc->rc.ch[4] < 120)
                {
                    shoot_control->shoot_flag = 0;
                }
            }
            else if (R)
            {
                if (shoot_control->press_l == 0)
                {
                    shoot_control->shoot_flag = 0;
                }
            }

            // 手动发射控制
            if (shoot_control->press_r) // 想开自瞄的时候,不会手动发弹(因为云台并非一直是自瞄模式)
            {
                // 自瞄模式下的发弹控制
                if ((shoot_control->shoot_flag == 0                                                                                         // 条件1
                     && ((shoot_control->shoot_rc->rc.ch[4] > 600 || shoot_control->press_l)                                                 // 并列条件2:手动触发发弹标志
                         && (vision_control.shoot_vision_control.shoot_command == SHOOT_ATTACK && gimbal_behaviour == GIMBAL_AUTO_ATTACK)) // 并列条件2:自瞄触发发弹标志
                     && BUTTEN_TRIG_PIN == RESET)                                                                                          // 条件3
                    && shoot_control->remain_shoot_number > 0)                                                                              // 最外层热量限制
                {
                    shoot_control->shoot_flag = 1;
                    shoot_control->last_bullet_number = shoot_control->bullet_number;
                }
            }
            else
            {
                // 手动模式下的发弹控制
                if ((shoot_control->shoot_flag == 0                                        // 条件1
                     && (shoot_control->shoot_rc->rc.ch[4] > 600 || shoot_control->press_l) // 并列条件2:自瞄触发发弹标志
                     && BUTTEN_TRIG_PIN == RESET)                                         // 条件3
                    /*&& shoot_control->remain_shoot_number > 0*/)                             // 最外层热量限制
                {
                    shoot_control->shoot_flag = 1;
                    shoot_control->last_bullet_number = shoot_control->bullet_number;
                }
            }
        }
        // 卡弹处理状态机
        else if (shoot_control->stuck_flag == 1 || shoot_control->number_stuck_flag == 1) // 卡弹
        {
            third_fric(0);
            trigger_motor(-5.0f); // 卡弹回拨速度
        }
        else if (shoot_control->stuck_flag == 2 || shoot_control->number_stuck_flag == 2) // 卡弹回拨后等待
        {
            third_fric(0);
            trigger_motor(0); // 拨弹盘等待速度
        }
        else if (shoot_control->number_stuck_flag == 3)//特殊拨弹第三步快速拨
        {
            trigger_motor(5.0f);
        }
        else if (shoot_control->number_stuck_flag == 4)//特殊拨弹第四步快速拨
        {
            trigger_motor(3.0f);
        }
    }
    else // 摩擦轮关闭状态
    {
        laser_off();
        shoot_fric(0, 0);
        third_fric(0);
        trigger_motor(0);
    }

    // 发弹信号处理
    if (shoot_control->last_shoot_flag == 0 && shoot_control->shoot_flag == 1)
    {
        shoot_control->shoot_time = 0;
    }

    // 拨弹控制
    if (shoot_control->shoot_time < shoot_control->turn_trigger_time) // 拨弹轮时间
    {
        trigger_motor(shoot_control->turn_trigger_speed); // 拨弹盘速度
    }
    else if (shoot_control->shoot_time == (shoot_control->turn_trigger_time))
    {
        trigger_motor(0);
    }

    // 更新计时器和状态
    shoot_control->shoot_time++;
    if (shoot_control->shoot_time >= 210)
        shoot_control->shoot_time = 210;
    
    // 更新状态记录
    shoot_control->last_butten_trig_pin = BUTTEN_TRIG_PIN;
    shoot_control->last_shoot_flag = shoot_control->shoot_flag;
    shoot_control->last_remain_number = shoot_control->remain_number;
}

/**
 * @brief          射击控制循环，计算电机速度
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_control_loop(shoot_control_t *shoot_control)
{
    // 计算拨弹电机PID输出
    PID_calc(&shoot_control->trigger_pid, shoot_control->trigger_speed, shoot_control->trigger_speed_set);
    
    // 计算四个摩擦轮电机的PID输出
    PID_calc(&shoot_control->fric_left_pid, shoot_control->fric_left_speed, shoot_control->fric_left_speed_set);
    PID_calc(&shoot_control->fric_right_pid, shoot_control->fric_right_speed, shoot_control->fric_right_speed_set);
    PID_calc(&shoot_control->fric_bleft_pid, shoot_control->fric_bleft_speed, shoot_control->fric_bleft_speed_set);
    PID_calc(&shoot_control->fric_bright_pid, shoot_control->fric_bright_speed, shoot_control->fric_bright_speed_set);

    // 更新CAN发送电流值
    trigger_can_set_current = shoot_control->trigger_pid.out;
    left_can_set_current = shoot_control->fric_left_pid.out;
    right_can_set_current = shoot_control->fric_right_pid.out;
    bleft_can_set_current = shoot_control->fric_bleft_pid.out;
    bright_can_set_current = shoot_control->fric_bright_pid.out;
}

/**
 * @brief          射击反馈数据更新
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void shoot_feedback_update(shoot_control_t *shoot_control)
{
    // 速度滤波变量
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    // 二阶低通滤波系数
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    // 拨弹轮电机速度滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control->trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control->trigger_speed = speed_fliter_3;

    // 更新各个电机的速度数据
    shoot_control->fric_b_speed = shoot_control->fric_b_motor_measure->speed_rpm;
    shoot_control->fric_left_speed = shoot_control->fric_left_motor_measure->speed_rpm;
    shoot_control->fric_right_speed = shoot_control->fric_right_motor_measure->speed_rpm;
    shoot_control->fric_bleft_speed = shoot_control->fric_bleft_motor_measure->speed_rpm;
    shoot_control->fric_bright_speed = shoot_control->fric_bright_motor_measure->speed_rpm;
    
    // 更新触发开关状态
    shoot_control->key_pin = BUTTEN_TRIG_PIN;

    // 更新鼠标按键状态
    shoot_control->last_press_l = shoot_control->press_l;
    shoot_control->last_press_r = shoot_control->press_r;
    shoot_control->press_l = shoot_control->shoot_rc->mouse.press_l;
    shoot_control->press_r = shoot_control->shoot_rc->mouse.press_r;

    // 根据裁判系统控制弹速
    fric_speed = shoot_control->first_speed; // 摩擦轮转速 5800
    shoot_fric(fric_speed, shoot_control->add_speed);

    // 根据按键切换热量限制模式
    if (shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_C && !(shoot_control->shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL))
    {
        limit = 300;  // 测试模式下的热量限制
        heat = 0;     // 测试模式下不计算热量
    }
    else
    {
        limit = robot_state.shooter_barrel_heat_limit;  // 正常模式下的热量限制
        heat = power_heat_data.shooter_42mm_barrel_heat; // 当前热量值
    }

    // 计算可击打弹丸数, 防止裁判系统延迟导致超热量 将结果作为可发射标志位
    shoot_control->remain_heat = limit - heat;
    if (shoot_control->remain_heat < 100)
    {
        shoot_control->remain_number = 0;
    }
    else if (shoot_control->remain_heat >= 100 && shoot_control->remain_heat < 200)
    {
        shoot_control->remain_number = 1;
    }
    else if (shoot_control->remain_heat >= 200 && shoot_control->remain_heat < 300)
    {
        shoot_control->remain_number = 2;
    }
    else if (shoot_control->remain_heat >= 300 && shoot_control->remain_heat < 400)
    {
        shoot_control->remain_number = 3;
    }
    else
    {
        shoot_control->remain_number = 0;
    }

    // 每到一个热量阈值,更新可击打弹丸数
    if (shoot_control->last_remain_number != 1 && shoot_control->remain_number == 1)
    {
        shoot_control->remain_shoot_number = 1;
    }
    else if (shoot_control->last_remain_number != 2 && shoot_control->remain_number == 2)
    {
        shoot_control->remain_shoot_number = 2;
    }
    else if (shoot_control->last_remain_number != 3 && shoot_control->remain_number == 3)
    {
        shoot_control->remain_shoot_number = 3;
    }
    else if (shoot_control->last_remain_number != 0 && shoot_control->remain_number == 0)
    {
        shoot_control->remain_shoot_number = 0;
    }

    // 卡弹检测后的额外等待时间
    if (shoot_control->shoot_time == (shoot_control->turn_trigger_time+13))
    {
        if(shoot_control->last_bullet_number == shoot_control->bullet_number)
        {
            shoot_control->number_stuck_flag = 1;
        }
    }
}

/**
 * @brief          卡弹拨盘回拨控制
 * @param[in]      shoot_control: 射击控制结构体指针
 * @retval         void
 */
static void trigger_motor_block_control(shoot_control_t *shoot_control)
{
    // 卡弹检测计时器
    static int16_t number_block_time = 0;
    
    // 根据电流值和时间判断是否卡弹
    if (trigger_can_set_current > 4200.0f)
    {
        // 判断1：供弹时供不上去
        shoot_control->block_time++;
        if (shoot_control->block_time > 500) // 检测时间
        {
            shoot_control->stuck_flag = 1;
            shoot_control->block_time = 0;
        }
        
        // 判断2：微动开关有弹但打不出去(中供的问题)
        if (trigger_can_set_current > 16000.0f)
        {
            number_block_time++;
            if (number_block_time > TURN_TRIGGER_TIME) // 检测时间 根据发弹时间设置
            {
                if(BUTTEN_TRIG_PIN == RESET)
                {
                    shoot_control->number_stuck_flag = 1;
                    number_block_time = 0;
                }
            }
        }
    }
    else // 不卡弹时重置计时器
    {
        shoot_control->block_time = 0;
        number_block_time = 0;
    }

    // 卡弹回拨时间安排（2步）
    if (shoot_control->stuck_flag == 1)
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 80) // 回拨时间
        {
            shoot_control->reverse_time = 0;
            shoot_control->stuck_flag = 2;
        }
    }
    else if (shoot_control->stuck_flag == 2) // 回拨后等待弹丸落回拨弹盘
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 80) // 等待时间
        {
            shoot_control->reverse_time = 0;
            shoot_control->stuck_flag = 0;
            shoot_control->stuck_trigger_flag = 1;
        }
    }

    // 特殊卡弹回拨状态机
    if(shoot_control->number_stuck_flag == 1)
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 50)
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 2;
        }
    }
    else if (shoot_control->number_stuck_flag == 2) // 回拨后等待弹丸落回拨弹盘
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 70) // 等待时间
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 3;
        }
    }
    else if (shoot_control->number_stuck_flag == 3) // 快速拨弹
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 80) // 快上弹时间
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 4;
        }
    }
    else if (shoot_control->number_stuck_flag == 4) // 正常拨弹
    {
        shoot_control->reverse_time++;
        if (shoot_control->reverse_time > 40) // 上弹时间
        {
            shoot_control->reverse_time = 0;
            shoot_control->number_stuck_flag = 0;
        }
    }
}

// ... existing code ... 
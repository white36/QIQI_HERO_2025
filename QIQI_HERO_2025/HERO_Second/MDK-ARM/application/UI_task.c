#include "UI_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "remote_control.h"
#include "referee.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "math.h"


// 全局UI任务实例
static UI_control_t ui_control;

// 外部变量声明
extern int anglesr;
extern int16_t angle_sin, angle_cos;
extern float angle_radto;
extern ext_robot_state_t robot_state;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern ext_power_heat_data_t power_heat_data_t;
extern chassis_move_t chassis_move;
extern ext_shoot_data_t shoot_data_t;
extern int8_t KEY_z;
extern fp32 RX_PITCH;
extern int8_t R, QA, EA, turn_flags, BPIN, AUTO_ATTACK, STUCK, VISION, FOLLOW;
extern cap_measure_t get_cap;

void UI_task(void const *pvParameters)
{
    // 初始化UI控制结构体
    memset(&ui_control, 0, sizeof(UI_control_t));
    
    // 初始化字符串数据
    memset(&ui_control.str_pitch, 0, sizeof(ui_control.str_pitch));
    memset(&ui_control.str_cap, 0, sizeof(ui_control.str_cap));
    memset(&ui_control.str_shoot, 0, sizeof(ui_control.str_shoot));
    memset(&ui_control.str_angle, 0, sizeof(ui_control.str_angle));
    memset(&ui_control.str_modeq, 0, sizeof(ui_control.str_modeq));
    memset(&ui_control.str_bpin, 0, sizeof(ui_control.str_bpin));
    memset(&ui_control.str_auto, 0, sizeof(ui_control.str_auto));
    memset(&ui_control.str_stuck, 0, sizeof(ui_control.str_stuck));
    memset(&ui_control.str_aim1, 0, sizeof(ui_control.str_aim1));
    memset(&ui_control.str_aim2, 0, sizeof(ui_control.str_aim2));
    
    // 初始化图形数据
    for (int l = 0; l < 10; l++) {
        memset(&ui_control.auto_data[l], 0, sizeof(ui_control.auto_data[l]));
    }
    for (int k = 0; k < 10; k++) {
        memset(&ui_control.aim_data[k], 0, sizeof(ui_control.aim_data[k]));
    }

    while(1)
    {
        if (KEY_z)
        {
            Line_Draw(&ui_control.aim_data[0], "AL1", UI_Graph_ADD, 5, UI_Color_Green, 3, 960, 300, 960, 540); // 竖直瞄准线
            Line_Draw(&ui_control.aim_data[2], "AL3", UI_Graph_ADD, 5, UI_Color_Yellow, 2, 930, 496, 990, 496);
            Line_Draw(&ui_control.aim_data[3], "AL4", UI_Graph_ADD, 5, UI_Color_Yellow, 2, 920, 410, 1000, 410);//5m
            Line_Draw(&ui_control.aim_data[6], "AL7", UI_Graph_ADD, 9, UI_Color_Yellow, 2, 920, 460, 1000, 460);//
            // 可通过宽度
            Line_Draw(&ui_control.aim_data[4], "AL5", UI_Graph_ADD, 7, UI_Color_Green, 6, 440, 0, 627, 443);
            Line_Draw(&ui_control.aim_data[5], "AL6", UI_Graph_ADD, 7, UI_Color_Green, 6, 1480, 0, 1293, 443);
            
            Line_Draw(&ui_control.aim_data[8], "AL9", UI_Graph_ADD, 5, UI_Color_Green, 3, 960, 100, 960, 540);
            //自瞄框
            //Line_Draw(&ui_control.auto_data[0], "AUTO1", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);
            //Line_Draw(&ui_control.auto_data[1], "AUTO2", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);
            //Line_Draw(&ui_control.auto_data[2], "AUTO3", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);
            //Line_Draw(&ui_control.auto_data[3], "AUTO4", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);

            Char_Draw(&ui_control.str_pitch, "PITCH", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(ui_control.tmp_pitch), 2, 860, 200, ui_control.tmp_pitch);
            Char_Draw(&ui_control.str_cap, "CAP", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(ui_control.tmp1), 2, 860, 100, ui_control.tmp1);
            Char_Draw(&ui_control.str_shoot, "SHOOT", UI_Graph_ADD, 9, UI_Color_Green, 30, strlen(ui_control.tmp2), 2, 600, 400, ui_control.tmp2);
            Char_Draw(&ui_control.str_angle, "ANGLE", UI_Graph_ADD, 8, UI_Color_Green, 30, strlen(ui_control.tmp4), 5, 1700, 500, ui_control.tmp4);
            Char_Draw(&ui_control.str_modeq, "mode_Q", UI_Graph_ADD, 6, UI_Color_Green, 30, strlen(ui_control.tmp_modeq), 2, 600, 200, ui_control.tmp_modeq);
            Char_Draw(&ui_control.str_bpin, "BPIN", UI_Graph_ADD, 4, UI_Color_Green, 30, strlen(ui_control.temp_bpin), 2, 600, 300, ui_control.temp_bpin);
            Char_Draw(&ui_control.str_auto, "AUTO", UI_Graph_ADD, 3, UI_Color_Green, 30, strlen(ui_control.temp_auto), 2, 600, 600, ui_control.temp_auto);
            Char_Draw(&ui_control.str_stuck, "STUCK", UI_Graph_ADD, 2, UI_Color_Green, 30, strlen(ui_control.temp_stuck), 2, 1200, 600, ui_control.temp_stuck);
            
            sprintf(ui_control.tmp_aim1, "2m");
            sprintf(ui_control.tmp_aim2, "1m");
            Char_Draw(&ui_control.str_aim1, "2m", UI_Graph_ADD, 5, UI_Color_Yellow, 10, strlen(ui_control.tmp_aim1), 2, 1010, 465, ui_control.tmp_aim1);
            Char_Draw(&ui_control.str_aim2, "1m", UI_Graph_ADD, 5, UI_Color_Yellow, 10, strlen(ui_control.tmp_aim2), 2, 1000, 500, ui_control.tmp_aim2);
            
            Char_ReFresh(ui_control.str_pitch);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_cap);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_shoot);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_angle);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_modeq);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_bpin);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_auto);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_stuck);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_aim1);
            vTaskDelay(25);
            Char_ReFresh(ui_control.str_aim2);
            vTaskDelay(25);
            
            UI_ReFresh(7, ui_control.aim_data[0], ui_control.aim_data[2], ui_control.aim_data[3], ui_control.aim_data[4], ui_control.aim_data[5], ui_control.aim_data[6], ui_control.aim_data[8]);
            vTaskDelay(25);
            UI_ReFresh(2, ui_control.auto_data[0], ui_control.auto_data[1]);
            vTaskDelay(20);
            UI_ReFresh(2, ui_control.auto_data[2], ui_control.auto_data[3]);
            vTaskDelay(20);
        }
        
        // 自瞄是否到可击打距离
        if (ui_control.last_VISION == 0 && VISION == 1)
        {
            Line_Draw(&ui_control.aim_data[0], "AL1", UI_Graph_Change, 5, UI_Color_Green, 3, 960, 100, 960, 540);
            UI_ReFresh(1, ui_control.aim_data[0]);
            vTaskDelay(25);
        }
        else if (ui_control.last_VISION == 1 && VISION == 0)
        {
            Line_Draw(&ui_control.aim_data[0], "AL1", UI_Graph_Change, 5, UI_Color_Black, 3, 960, 100, 960, 540);
            UI_ReFresh(1, ui_control.aim_data[0]);
            vTaskDelay(25);
        }
        ui_control.last_VISION = VISION;
        
        // 车宽UI
        if (FOLLOW == 1)
        {
            Line_Draw(&ui_control.aim_data[4], "AL5", UI_Graph_Change, 7, UI_Color_Green, 6, 440 - anglesr, 0, 627 - anglesr, 443);
            Line_Draw(&ui_control.aim_data[5], "AL6", UI_Graph_Change, 7, UI_Color_Green, 6, 1480 + anglesr, 0, 1293 + anglesr, 443);
            UI_ReFresh(2, ui_control.aim_data[4], ui_control.aim_data[5]);
            vTaskDelay(25);
        }
        else if(FOLLOW == 0)
        {
            Line_Draw(&ui_control.aim_data[4], "AL5", UI_Graph_Change, 7, UI_Color_Pink, 6, 440 - anglesr, 0, 627 - anglesr, 443);
            Line_Draw(&ui_control.aim_data[5], "AL6", UI_Graph_Change, 7, UI_Color_Pink, 6, 1480 + anglesr, 0, 1293 + anglesr, 443);
            UI_ReFresh(2, ui_control.aim_data[4], ui_control.aim_data[5]);
            vTaskDelay(25);
        }

        // 俯仰角显示
        sprintf(ui_control.tmp_pitch, "PITCH:%.4f", RX_PITCH);
        Char_Draw(&ui_control.str_pitch, "PITCH", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(ui_control.tmp_pitch), 2, 860, 200, ui_control.tmp_pitch);
        Char_ReFresh(ui_control.str_pitch);
        vTaskDelay(25);

        // 超电电压显示
        sprintf(ui_control.tmp1, "Cap:%.2f", get_cap.cap_volt);
        if (get_cap.cap_volt > 12.0f)
            Char_Draw(&ui_control.str_cap, "CAP", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(ui_control.tmp1), 2, 860, 100, ui_control.tmp1);
        else
            Char_Draw(&ui_control.str_cap, "CAP", UI_Graph_Change, 8, UI_Color_Orange, 20, strlen(ui_control.tmp1), 2, 860, 100, ui_control.tmp1);
        Char_ReFresh(ui_control.str_cap);
        vTaskDelay(25);

        // 判断shoot模式开关
        if(ui_control.last_R == 0 && R == 1)
        {
            sprintf(ui_control.tmp2, "SHOOT:ON");
            Char_Draw(&ui_control.str_shoot, "SHOOT", UI_Graph_Change, 9, UI_Color_Orange, 30, strlen(ui_control.tmp2), 2, 600, 400, ui_control.tmp2);
            Char_ReFresh(ui_control.str_shoot);
            vTaskDelay(25);
        }
        else if (ui_control.last_R == 1 && R == 0)
        {
            sprintf(ui_control.tmp2, "........");
            Char_Draw(&ui_control.str_shoot, "SHOOT", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(ui_control.tmp2), 2, 600, 400, ui_control.tmp2);
            Char_ReFresh(ui_control.str_shoot);
            vTaskDelay(25);
        }

        // 判断 QA 模式开关
        if (ui_control.last_QA == 0 && QA == 1)
        {
            sprintf(ui_control.tmp_modeq, "MODE_Q");
            Char_Draw(&ui_control.str_modeq, "mode_Q", UI_Graph_Change, 6, UI_Color_Orange, 30, strlen(ui_control.tmp_modeq), 2, 600, 200, ui_control.tmp_modeq);
            Char_ReFresh(ui_control.str_modeq);
            vTaskDelay(25);
        }
        else if (ui_control.last_QA == 1 && QA == 0)
        {
            sprintf(ui_control.tmp_modeq, "......");
            Char_Draw(&ui_control.str_modeq, "mode_Q", UI_Graph_Change, 6, UI_Color_Green, 30, strlen(ui_control.tmp_modeq), 2, 600, 200, ui_control.tmp_modeq);
            Char_ReFresh(ui_control.str_modeq);
            vTaskDelay(25);
        }

        // 判断 BPIN 模式开关
        if (ui_control.last_BPIN == 0 && BPIN == 1)
        {
            sprintf(ui_control.temp_bpin, "BPIN");
            Char_Draw(&ui_control.str_bpin, "BPIN", UI_Graph_Change, 4, UI_Color_Orange, 30, strlen(ui_control.temp_bpin), 2, 600, 300, ui_control.temp_bpin);
            Char_ReFresh(ui_control.str_bpin);
            vTaskDelay(25);
        }
        else if (ui_control.last_BPIN == 1 && BPIN == 0)
        {
            sprintf(ui_control.temp_bpin, ".....");
            Char_Draw(&ui_control.str_bpin, "BPIN", UI_Graph_Change, 4, UI_Color_Green, 30, strlen(ui_control.temp_bpin), 2, 600, 300, ui_control.temp_bpin);
            Char_ReFresh(ui_control.str_bpin);
            vTaskDelay(25);
        }

        // 判断 AUTO_ATTACK 模式开关
        if (ui_control.last_AUTO_ATTACK == 0 && AUTO_ATTACK == 1)
        {
            sprintf(ui_control.temp_auto, "AUTO");
            Char_Draw(&ui_control.str_auto, "AUTO", UI_Graph_Change, 3, UI_Color_Orange, 30, strlen(ui_control.temp_auto), 2, 600, 600, ui_control.temp_auto);
            Char_ReFresh(ui_control.str_auto);
            vTaskDelay(25);
        }
        else if (ui_control.last_AUTO_ATTACK == 1 && AUTO_ATTACK == 0)
        {
            sprintf(ui_control.temp_auto, "....");
            Char_Draw(&ui_control.str_auto, "AUTO", UI_Graph_Change, 3, UI_Color_Green, 30, strlen(ui_control.temp_auto), 2, 600, 600, ui_control.temp_auto);
            Char_ReFresh(ui_control.str_auto);
            vTaskDelay(25);
        }

        // 判断 STUCK 模式开关
        if (ui_control.last_STUCK == 0 && STUCK == 1)
        {
            sprintf(ui_control.temp_stuck, "STUCK");
            Char_Draw(&ui_control.str_stuck, "STUCK", UI_Graph_Change, 2, UI_Color_Orange, 30, strlen(ui_control.temp_stuck), 2, 1200, 600, ui_control.temp_stuck);
            Char_ReFresh(ui_control.str_stuck);
            vTaskDelay(25);
        }
        else if (ui_control.last_STUCK == 1 && STUCK == 0)
        {
            sprintf(ui_control.temp_stuck, ".....");
            Char_Draw(&ui_control.str_stuck, "STUCK", UI_Graph_Change, 2, UI_Color_Green, 30, strlen(ui_control.temp_stuck), 2, 1200, 600, ui_control.temp_stuck);
            Char_ReFresh(ui_control.str_stuck);
            vTaskDelay(25);
        }

        // 更新状态标志
        ui_control.last_R = R;
        ui_control.last_QA = QA;
        ui_control.last_EA = EA;
        ui_control.last_turn_flags = turn_flags;
        ui_control.last_BPIN = BPIN;
        ui_control.last_AUTO_ATTACK = AUTO_ATTACK;
        ui_control.last_STUCK = STUCK;
        
        vTaskDelay(10);
    }
} 
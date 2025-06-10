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
#include "can_receive.h"

// ȫ��UI����ʵ��
static UI_control_t ui_control;
// ��������
static void UI_init(UI_control_t *ui_control);
static void UI_update(UI_control_t *ui_control);
static void UI_draw(UI_control_t *ui_control);

// �ⲿ��������
extern int anglesr;
extern int16_t angle_sin, angle_cos;
extern float angle_radto;
extern ext_robot_state_t robot_state;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern ext_power_heat_data_t power_heat_data_t;
extern chassis_move_t chassis_move;
extern ext_shoot_data_t shoot_data_t;
extern int8_t KEY_z, KEY_c, KEY_q;
int8_t last_KEY_z;


fp32 RX_PITCH, RX_first_speed, RX_add_speed = 0;
int8_t R, QA, BPIN, AUTO_ATTACK, STUCK, VISION, FOLLOW, TURN_REMOTE_FIRE = 0;
extern cap_measure_t get_cap;
extern int8_t turn_flags;

void UI_task(void const *pvParameters)
{
    // UI��ʼ��
    UI_init(&ui_control);
	vTaskDelay(10);
    while (1)
    {
        ui_control.UI_xTickCount = xTaskGetTickCount();
		if (!last_KEY_z && KEY_z)
		{
			ui_control.KEY_z_press_time = ui_control.UI_xTickCount;
		}
		if (last_KEY_z && !KEY_z)
		{
			if (ui_control.UI_xTickCount - ui_control.KEY_z_press_time >= 500) // ����
			{
				// ˢ��UI
				UI_draw(&ui_control);
			}
		}
		last_KEY_z = KEY_z;

        // ��ȡyaw����ֵ
		ui_control.YAW = get_yaw_gimbal_motor_measure_point()->ecd;
        // ��ȡRFID�ṹ��
        ui_control.rfid_status = get_rfid_status();
		// ��ȡϪ�س���ṹ��
		ui_control.cap_data = get_cap_data_point();
		ui_control.robot_pos = get_robot_pos_point();
		
        // ��̬UI����
        UI_update(&ui_control);

        vTaskDelay(10);
    }
}

// ui��ʼ��
void UI_init(UI_control_t *ui_control)
{
    // ��ʼ��UI���ƽṹ��
    memset(ui_control, 0, sizeof(UI_control_t));

    // ��ʼ���ַ�������
	memset(&ui_control->str_x, 0, sizeof(ui_control->str_x));
    memset(&ui_control->str_y, 0, sizeof(ui_control->str_y));
	memset(&ui_control->str_pitch, 0, sizeof(ui_control->str_pitch));
	memset(&ui_control->str_yaw, 0, sizeof(ui_control->str_yaw));
    memset(&ui_control->str_first_speed, 0, sizeof(ui_control->str_first_speed));
    memset(&ui_control->str_add_speed, 0, sizeof(ui_control->str_add_speed));
    memset(&ui_control->str_cap, 0, sizeof(ui_control->str_cap));
	memset(&ui_control->str_RFID, 0, sizeof(ui_control->str_RFID));
    memset(&ui_control->str_shoot, 0, sizeof(ui_control->str_shoot));
    memset(&ui_control->str_angle, 0, sizeof(ui_control->str_angle));
    memset(&ui_control->str_modeq, 0, sizeof(ui_control->str_modeq));
    memset(&ui_control->str_bpin, 0, sizeof(ui_control->str_bpin));
    memset(&ui_control->str_auto, 0, sizeof(ui_control->str_auto));
    memset(&ui_control->str_stuck, 0, sizeof(ui_control->str_stuck));
    memset(&ui_control->str_aim1, 0, sizeof(ui_control->str_aim1));
    memset(&ui_control->str_aim2, 0, sizeof(ui_control->str_aim2));

    // ��ʼ��ͼ������
    for (int l = 0; l < 10; l++)
    {
        memset(&ui_control->auto_data[l], 0, sizeof(ui_control->auto_data[l]));
    }
    for (int k = 0; k < 10; k++)
    {
        memset(&ui_control->aim_data[k], 0, sizeof(ui_control->aim_data[k]));
    }

    // ˢ��һ��
    UI_draw(ui_control);
}

// UIˢ��
void UI_draw(UI_control_t *ui_control)
{
    Line_Draw(&ui_control->aim_data[0], "AL1", UI_Graph_ADD, 5, UI_Color_Green, 3, 960, 300, 960, 540); // ��ֱ��׼��
    Line_Draw(&ui_control->aim_data[2], "AL3", UI_Graph_ADD, 5, UI_Color_Yellow, 2, 1040, 300, 1040, 540); // 1m
    Line_Draw(&ui_control->aim_data[3], "AL4", UI_Graph_ADD, 5, UI_Color_Yellow, 2, 920, 385, 1000, 385); // ����������
    Line_Draw(&ui_control->aim_data[6], "AL7", UI_Graph_ADD, 9, UI_Color_Yellow, 2, 920, 460, 1000, 460); //
    // ��ͨ�����
    Line_Draw(&ui_control->aim_data[4], "AL5", UI_Graph_ADD, 7, UI_Color_Green, 6, 440, 0, 627, 443);
    Line_Draw(&ui_control->aim_data[5], "AL6", UI_Graph_ADD, 7, UI_Color_Green, 6, 1480, 0, 1293, 443);

    Line_Draw(&ui_control->aim_data[8], "AL9", UI_Graph_ADD, 5, UI_Color_Green, 3, 960, 100, 960, 540);
    // �����
    // Line_Draw(&ui_control->auto_data[0], "AUTO1", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);
    // Line_Draw(&ui_control->auto_data[1], "AUTO2", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);
    // Line_Draw(&ui_control->auto_data[2], "AUTO3", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);
    // Line_Draw(&ui_control->auto_data[3], "AUTO4", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 400, 100, 1200, 540);
//    Char_Draw(&ui_control->str_x, "X", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(ui_control->tmp_x), 2, 380, 600, ui_control->tmp_x);
//    Char_Draw(&ui_control->str_y, "Y", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(ui_control->tmp_y), 2, 380, 550, ui_control->tmp_y);
    Char_Draw(&ui_control->str_pitch, "PITCH", UI_Graph_ADD, 8, UI_Color_Cyan, 30, strlen(ui_control->tmp_pitch), 3, 760, 800, ui_control->tmp_pitch);
	Char_Draw(&ui_control->str_yaw, "YAW", UI_Graph_ADD, 8, UI_Color_Cyan, 30, strlen(ui_control->tmp_yaw), 3, 840, 750, ui_control->tmp_yaw);
    Char_Draw(&ui_control->str_first_speed, "First_Speed", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(ui_control->tmp_first_speed), 2, 200, 600, ui_control->tmp_first_speed);
    Char_Draw(&ui_control->str_add_speed, "Add_Speed", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(ui_control->tmp_add_speed), 2, 200, 550, ui_control->tmp_add_speed);
    Char_Draw(&ui_control->str_cap, "CAP", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(ui_control->tmp_cap), 2, 860, 100, ui_control->tmp_cap);
    Char_Draw(&ui_control->str_RFID, "RFID", UI_Graph_ADD, 8, UI_Color_White, 30, strlen(ui_control->tmp_RFID), 4, 1800, 800, ui_control->tmp_RFID);
    Char_Draw(&ui_control->str_shoot, "SHOOT", UI_Graph_ADD, 9, UI_Color_Green, 30, strlen(ui_control->tmp2), 2, 600, 400, ui_control->tmp2);
    Char_Draw(&ui_control->str_angle, "ANGLE", UI_Graph_ADD, 8, UI_Color_Green, 30, strlen(ui_control->tmp4), 5, 1700, 500, ui_control->tmp4);
    Char_Draw(&ui_control->str_modeq, "mode_Q", UI_Graph_ADD, 6, UI_Color_Green, 30, strlen(ui_control->tmp_modeq), 2, 600, 200, ui_control->tmp_modeq);
    Char_Draw(&ui_control->str_bpin, "BPIN", UI_Graph_ADD, 4, UI_Color_Green, 30, strlen(ui_control->tmp_bpin), 2, 600, 300, ui_control->tmp_bpin);
    Char_Draw(&ui_control->str_auto, "AUTO", UI_Graph_ADD, 3, UI_Color_Green, 30, strlen(ui_control->tmp_auto), 2, 600, 600, ui_control->tmp_auto);
    Char_Draw(&ui_control->str_stuck, "STUCK", UI_Graph_ADD, 2, UI_Color_Green, 30, strlen(ui_control->tmp_stuck), 2, 1200, 600, ui_control->tmp_stuck);

//    sprintf(ui_control->tmp_aim1, "TURN_FLAG");
//    Char_Draw(&ui_control->str_aim1, "TURN_FLAG", UI_Graph_ADD, 5, UI_Color_Orange, 10, strlen(ui_control->tmp_aim1), 2, 1010, 465, ui_control->tmp_aim1);
    sprintf(ui_control->tmp_aim2, "1m");
    Char_Draw(&ui_control->str_aim2, "1m", UI_Graph_ADD, 5, UI_Color_Yellow, 10, strlen(ui_control->tmp_aim2), 2, 1000, 500, ui_control->tmp_aim2);

    Char_ReFresh(ui_control->str_x);
    vTaskDelay(33);
	Char_ReFresh(ui_control->str_y);
    vTaskDelay(33);
	Char_ReFresh(ui_control->str_pitch);
    vTaskDelay(33);
	Char_ReFresh(ui_control->str_yaw);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_first_speed);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_add_speed);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_cap);
    vTaskDelay(33);
	Char_ReFresh(ui_control->str_RFID);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_shoot);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_angle);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_modeq);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_bpin);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_auto);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_stuck);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_aim1);
    vTaskDelay(33);
    Char_ReFresh(ui_control->str_aim2);
    vTaskDelay(33);

    UI_ReFresh(7, ui_control->aim_data[0], ui_control->aim_data[2], ui_control->aim_data[3], ui_control->aim_data[4], ui_control->aim_data[5], ui_control->aim_data[6], ui_control->aim_data[8]);
    vTaskDelay(33);
    UI_ReFresh(2, ui_control->auto_data[0], ui_control->auto_data[1]);
    vTaskDelay(20);
    UI_ReFresh(2, ui_control->auto_data[2], ui_control->auto_data[3]);
    vTaskDelay(20);

    // �����Ƿ񵽿ɻ������
    Line_Draw(&ui_control->aim_data[0], "AL1", UI_Graph_Change, 5, UI_Color_Black, 3, 960, 100, 960, 540);
    UI_ReFresh(1, ui_control->aim_data[0]);
    vTaskDelay(33);
    // ����UI
    Line_Draw(&ui_control->aim_data[4], "AL5", UI_Graph_Change, 7, UI_Color_Pink, 6, 440 - anglesr, 0, 627 - anglesr, 443);
    Line_Draw(&ui_control->aim_data[5], "AL6", UI_Graph_Change, 7, UI_Color_Pink, 6, 1480 + anglesr, 0, 1293 + anglesr, 443);
    UI_ReFresh(2, ui_control->aim_data[4], ui_control->aim_data[5]);
    vTaskDelay(33);
    // �ж�shootģʽ����
    sprintf(ui_control->tmp2, "........");
    Char_Draw(&ui_control->str_shoot, "SHOOT", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(ui_control->tmp2), 2, 600, 400, ui_control->tmp2);
    Char_ReFresh(ui_control->str_shoot);
    vTaskDelay(33);
    // �ж� QA ģʽ����
    sprintf(ui_control->tmp_modeq, "......");
    Char_Draw(&ui_control->str_modeq, "mode_Q", UI_Graph_Change, 6, UI_Color_Green, 30, strlen(ui_control->tmp_modeq), 2, 600, 200, ui_control->tmp_modeq);
    Char_ReFresh(ui_control->str_modeq);
    vTaskDelay(33);
    // �ж� BPIN ģʽ����
    sprintf(ui_control->tmp_bpin, "....");
    Char_Draw(&ui_control->str_bpin, "BPIN", UI_Graph_Change, 4, UI_Color_Green, 30, strlen(ui_control->tmp_bpin), 2, 600, 300, ui_control->tmp_bpin);
    Char_ReFresh(ui_control->str_bpin);
    vTaskDelay(33);
    // �ж� AUTO ģʽ����
    sprintf(ui_control->tmp_auto, "....");
    Char_Draw(&ui_control->str_auto, "AUTO", UI_Graph_Change, 3, UI_Color_Green, 30, strlen(ui_control->tmp_auto), 2, 600, 600, ui_control->tmp_auto);
    Char_ReFresh(ui_control->str_auto);
    vTaskDelay(33);
    // �ж� STUCK ģʽ����
    sprintf(ui_control->tmp_stuck, ".....");
    Char_Draw(&ui_control->str_stuck, "STUCK", UI_Graph_Change, 2, UI_Color_Green, 30, strlen(ui_control->tmp_stuck), 2, 1200, 600, ui_control->tmp_stuck);
    Char_ReFresh(ui_control->str_stuck);
    vTaskDelay(33);
}

// ��̬UI����
void UI_update(UI_control_t *ui_control)
{
    // �����Ƿ񵽿ɻ������
    if (ui_control->last_VISION == 0 && VISION == 1)
    {
        Line_Draw(&ui_control->aim_data[0], "AL1", UI_Graph_Change, 5, UI_Color_Green, 3, 960, 100, 960, 540);
        UI_ReFresh(1, ui_control->aim_data[0]);
        vTaskDelay(33);
    }
    else if (ui_control->last_VISION == 1 && VISION == 0)
    {
        Line_Draw(&ui_control->aim_data[0], "AL1", UI_Graph_Change, 5, UI_Color_Black, 3, 960, 100, 960, 540);
        UI_ReFresh(1, ui_control->aim_data[0]);
        vTaskDelay(33);
    }
    ui_control->last_VISION = VISION;

    // ����UI
    if (FOLLOW == 1)
    {
        Line_Draw(&ui_control->aim_data[4], "AL5", UI_Graph_Change, 7, UI_Color_Green, 6, 440 - anglesr, 0, 627 - anglesr, 443);
        Line_Draw(&ui_control->aim_data[5], "AL6", UI_Graph_Change, 7, UI_Color_Green, 6, 1480 + anglesr, 0, 1293 + anglesr, 443);
        UI_ReFresh(2, ui_control->aim_data[4], ui_control->aim_data[5]);
        vTaskDelay(33);
    }
    else if (FOLLOW == 0)
    {
        Line_Draw(&ui_control->aim_data[4], "AL5", UI_Graph_Change, 7, UI_Color_Pink, 6, 440 - anglesr, 0, 627 - anglesr, 443);
        Line_Draw(&ui_control->aim_data[5], "AL6", UI_Graph_Change, 7, UI_Color_Pink, 6, 1480 + anglesr, 0, 1293 + anglesr, 443);
        UI_ReFresh(2, ui_control->aim_data[4], ui_control->aim_data[5]);
        vTaskDelay(33);
    }

//	sprintf(ui_control->tmp_x, "X:%.2f", (fp32)ui_control->robot_pos->x);
//    Char_Draw(&ui_control->str_x, "X", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(ui_control->tmp_x), 2, 380, 600, ui_control->tmp_x);
//    Char_ReFresh(ui_control->str_x);
//    vTaskDelay(33);
//    sprintf(ui_control->tmp_y, "Y:%.2f", (fp32)ui_control->robot_pos->y);
//    Char_Draw(&ui_control->str_y, "Y", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(ui_control->tmp_y), 2, 380, 550, ui_control->tmp_y);
//    Char_ReFresh(ui_control->str_y);
//    vTaskDelay(33);
	if (TURN_REMOTE_FIRE)
	{
	// ��������ʾ
	if (KEY_z)
	{
		sprintf(ui_control->tmp_pitch, "PITCH:%.2f", RX_PITCH);
		Char_Draw(&ui_control->str_pitch, "PITCH", UI_Graph_Change, 8, UI_Color_Cyan, 30, strlen(ui_control->tmp_pitch), 3, 760, 800, ui_control->tmp_pitch);
		Char_ReFresh(ui_control->str_pitch);
		vTaskDelay(25);
	}
	else
	{
		sprintf(ui_control->tmp_pitch, "PITCH:%.2f", RX_PITCH);
		Char_Draw(&ui_control->str_pitch, "PITCH", UI_Graph_Change, 8, UI_Color_Black, 30, strlen(ui_control->tmp_pitch), 3, 760, 800, ui_control->tmp_pitch);
		Char_ReFresh(ui_control->str_pitch);
		vTaskDelay(25);
	}

	// ˮƽ����ʾ
	if (KEY_c)
	{
		sprintf(ui_control->tmp_yaw, "YAW:%.1f", ui_control->YAW);
		Char_Draw(&ui_control->str_yaw, "YAW", UI_Graph_Change, 8, UI_Color_Cyan, 30, strlen(ui_control->tmp_yaw), 3, 840, 750, ui_control->tmp_yaw);
		Char_ReFresh(ui_control->str_yaw);
		vTaskDelay(25);
	}
	else
	{
		sprintf(ui_control->tmp_yaw, "YAW:%.1f", ui_control->YAW);
		Char_Draw(&ui_control->str_yaw, "YAW", UI_Graph_Change, 8, UI_Color_Black, 30, strlen(ui_control->tmp_yaw), 3, 840, 750, ui_control->tmp_yaw);
		Char_ReFresh(ui_control->str_yaw);
		vTaskDelay(25);
	}
}
	else
	{
		sprintf(ui_control->tmp_pitch, "PITCH:%.2f", RX_PITCH);
		Char_Draw(&ui_control->str_pitch, "PITCH", UI_Graph_Change, 8, UI_Color_Black, 30, strlen(ui_control->tmp_pitch), 3, 760, 800, ui_control->tmp_pitch);
		Char_ReFresh(ui_control->str_pitch);
		vTaskDelay(25);
		sprintf(ui_control->tmp_yaw, "YAW:%.1f", ui_control->YAW);
		Char_Draw(&ui_control->str_yaw, "YAW", UI_Graph_Change, 8, UI_Color_Black, 30, strlen(ui_control->tmp_yaw), 3, 840, 750, ui_control->tmp_yaw);
		Char_ReFresh(ui_control->str_yaw);
		vTaskDelay(25);
	}

    // Ħ����ת����ʾ(�趨ֵ)
    sprintf(ui_control->tmp_first_speed, "Speed:%.0f", (fp32)RX_first_speed);
    Char_Draw(&ui_control->str_first_speed, "First_Speed", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(ui_control->tmp_first_speed), 2, 200, 600, ui_control->tmp_first_speed);
    Char_ReFresh(ui_control->str_first_speed);
    vTaskDelay(33);
    sprintf(ui_control->tmp_add_speed, "Add:%.0f", (fp32)RX_add_speed);
    Char_Draw(&ui_control->str_add_speed, "Add_Speed", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(ui_control->tmp_add_speed), 2, 200, 550, ui_control->tmp_add_speed);
    Char_ReFresh(ui_control->str_add_speed);
    vTaskDelay(33);

    // �����ѹ��ʾ
    sprintf(ui_control->tmp_cap, "Cap:%.1f", (float)ui_control->cap_data->cap_volt);
    if (ui_control->cap_data->cap_volt > 17.0f)
        Char_Draw(&ui_control->str_cap, "CAP", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(ui_control->tmp_cap), 2, 860, 100, ui_control->tmp_cap);
    else
        Char_Draw(&ui_control->str_cap, "CAP", UI_Graph_Change, 8, UI_Color_Orange, 20, strlen(ui_control->tmp_cap), 2, 860, 100, ui_control->tmp_cap);
    Char_ReFresh(ui_control->str_cap);
    vTaskDelay(33);

	// RFID��ʾ
    sprintf(ui_control->tmp_RFID, "RFID");
    if (!(ui_control->rfid_status->rfid_status))
        Char_Draw(&ui_control->str_RFID, "RFID", UI_Graph_Change, 8, UI_Color_Green, 30, strlen(ui_control->tmp_RFID), 8, 1800, 800, ui_control->tmp_RFID);
    else
        Char_Draw(&ui_control->str_RFID, "RFID", UI_Graph_Change, 8, UI_Color_White, 30, strlen(ui_control->tmp_RFID), 4, 1800, 800, ui_control->tmp_RFID);
    Char_ReFresh(ui_control->str_RFID);
    vTaskDelay(33);

    // �ж�shootģʽ����
//    if (ui_control->last_R == 0 && R == 1)
	if(R == 1)
    {
        sprintf(ui_control->tmp2, "SHOOT:ON");
        Char_Draw(&ui_control->str_shoot, "SHOOT", UI_Graph_Change, 9, UI_Color_Orange, 30, strlen(ui_control->tmp2), 2, 600, 400, ui_control->tmp2);
        Char_ReFresh(ui_control->str_shoot);
        vTaskDelay(33);
    }
//    else if (ui_control->last_R == 1 && R == 0)
    else
	{
        sprintf(ui_control->tmp2, "........");
        Char_Draw(&ui_control->str_shoot, "SHOOT", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(ui_control->tmp2), 2, 600, 400, ui_control->tmp2);
        Char_ReFresh(ui_control->str_shoot);
        vTaskDelay(33);
    }

//    // �ж� QA ģʽ����
//    if (ui_control->last_QA == 0 && QA == 1)
//    {
//        sprintf(ui_control->tmp_modeq, "MODE_Q");
//        Char_Draw(&ui_control->str_modeq, "mode_Q", UI_Graph_Change, 6, UI_Color_Orange, 30, strlen(ui_control->tmp_modeq), 2, 600, 200, ui_control->tmp_modeq);
//        Char_ReFresh(ui_control->str_modeq);
//        vTaskDelay(33);
//    }
//    else if (ui_control->last_QA == 1 && QA == 0)
//    {
//        sprintf(ui_control->tmp_modeq, ".");
//        Char_Draw(&ui_control->str_modeq, "mode_Q", UI_Graph_Change, 6, UI_Color_Green, 30, strlen(ui_control->tmp_modeq), 2, 600, 200, ui_control->tmp_modeq);
//        Char_ReFresh(ui_control->str_modeq);
//        vTaskDelay(33);
//    }

    // �ж� BPIN ģʽ����
    if (ui_control->last_BPIN == 0 && BPIN == 1)
    {
        sprintf(ui_control->tmp_bpin, "BPIN");
        Char_Draw(&ui_control->str_bpin, "BPIN", UI_Graph_Change, 4, UI_Color_Orange, 30, strlen(ui_control->tmp_bpin), 2, 600, 300, ui_control->tmp_bpin);
        Char_ReFresh(ui_control->str_bpin);
        vTaskDelay(33);
    }
    else if (ui_control->last_BPIN == 1 && BPIN == 0)
    {
        sprintf(ui_control->tmp_bpin, "....");
        Char_Draw(&ui_control->str_bpin, "BPIN", UI_Graph_Change, 4, UI_Color_Green, 30, strlen(ui_control->tmp_bpin), 2, 600, 300, ui_control->tmp_bpin);
        Char_ReFresh(ui_control->str_bpin);
        vTaskDelay(33);
    }

    // �ж� AUTO_ATTACK ģʽ����
    if (ui_control->last_AUTO_ATTACK == 0 && AUTO_ATTACK == 1)
    {
        sprintf(ui_control->tmp_auto, "AUTO");
        Char_Draw(&ui_control->str_auto, "AUTO", UI_Graph_Change, 3, UI_Color_Orange, 30, strlen(ui_control->tmp_auto), 2, 600, 600, ui_control->tmp_auto);
        Char_ReFresh(ui_control->str_auto);
        vTaskDelay(33);
    }
    else if (ui_control->last_AUTO_ATTACK == 1 && AUTO_ATTACK == 0)
    {
        sprintf(ui_control->tmp_auto, "....");
        Char_Draw(&ui_control->str_auto, "AUTO", UI_Graph_Change, 3, UI_Color_Green, 30, strlen(ui_control->tmp_auto), 2, 600, 600, ui_control->tmp_auto);
        Char_ReFresh(ui_control->str_auto);
        vTaskDelay(33);
    }

    // �ж� STUCK ģʽ����
    if (ui_control->last_STUCK == 0 && STUCK == 1)
    {
        sprintf(ui_control->tmp_stuck, "STUCK");
        Char_Draw(&ui_control->str_stuck, "STUCK", UI_Graph_Change, 2, UI_Color_Orange, 30, strlen(ui_control->tmp_stuck), 2, 1200, 600, ui_control->tmp_stuck);
        Char_ReFresh(ui_control->str_stuck);
        vTaskDelay(33);
    }
    else if (ui_control->last_STUCK == 1 && STUCK == 0)
    {
        sprintf(ui_control->tmp_stuck, ".....");
        Char_Draw(&ui_control->str_stuck, "STUCK", UI_Graph_Change, 2, UI_Color_Green, 30, strlen(ui_control->tmp_stuck), 2, 1200, 600, ui_control->tmp_stuck);
        Char_ReFresh(ui_control->str_stuck);
        vTaskDelay(33);
    }

    // ����״̬��־
    ui_control->last_R = R;
    ui_control->last_QA = QA;
    ui_control->last_BPIN = BPIN;
    ui_control->last_AUTO_ATTACK = AUTO_ATTACK;
    ui_control->last_STUCK = STUCK;
}

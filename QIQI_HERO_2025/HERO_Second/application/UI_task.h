#ifndef UI_TASK_H
#define UI_TASK_H
#include "main.h"
#include "RM_Cilent_UI.h"
#include "referee.h"
#include "can_receive.h"

// UI任务控制结构体
typedef struct
{
    // 图形数据
    Graph_Data aim_data[12];
    Graph_Data auto_data[4];

    // 字符串数据
    String_Data str_x;
	String_Data str_y;
	String_Data str_pitch;
	String_Data str_yaw;
    String_Data str_first_speed;
    String_Data str_add_speed;
    String_Data str_cap;
    String_Data str_RFID;
    String_Data str_shoot;
    String_Data str_angle;
    String_Data str_modeq;
    String_Data str_bpin;
    String_Data str_auto;
    String_Data str_stuck;
    String_Data str_aim1;
    String_Data str_aim2;

    // 临时缓冲区
	char tmp_x[30];
	char tmp_y[30];
    char tmp_pitch[30];
	char tmp_yaw[30];
    char tmp_first_speed[30];
    char tmp_add_speed[30];
    char tmp_cap[30];
    char tmp_RFID[30];
    char tmp2[30];
    char tmp4[30];
    char tmp_modeq[30];
    char tmp_bpin[30];
    char tmp_auto[30];
    char tmp_stuck[30];
    char tmp_aim1[30];
    char tmp_aim2[30];

    // 状态标志
    int8_t last_R;
    int8_t last_QA;
    int8_t last_BPIN;
    int8_t last_AUTO_ATTACK;
    int8_t last_STUCK;
    int8_t last_VISION;
	
    // 用于获取数据
    fp32 YAW;
	ext_rfid_status_t* rfid_status;
	const cap_measure_t *cap_data;
	ext_robot_pos_t* robot_pos;
	
	TickType_t UI_xTickCount;
	TickType_t KEY_z_press_time;
	
} UI_control_t;

// 函数声明
void UI_task(void const *pvParameters);


#endif

/*
 * File: stm32.c
 *
 * Code generated for Simulink model :stm32.
 *
 * Model version      : 1.3
 * Simulink Coder version    : 9.3 (R2020a) 18-Nov-2019
 * TLC version       : 9.3 (Jan 23 2020)
 * C/C++ source code generated on  : Sat Feb  5 19:50:19 2022
 *
 * Target selection: stm32.tlc
 * Embedded hardware selection: STM32CortexM
 * Code generation objectives: Unspecified
 * Validation result: Not run
 *
 *
 *
 * ******************************************************************************
 * * attention
 * *
 * * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * *
 * ******************************************************************************
 */

#include "stm32.h"
#include "main.h"
#include "stm32_private.h"
#define PI 3.1415926
/* Block states (default storage) */
DW_stm32 stm32_DW_yaw;
DW_stm32 stm32_DW_pitch;
DW_stm32 stm32_DW_auto_yaw;
DW_stm32 stm32_DW_auto_pitch;
DW_stm32 stm32_DW_trigger;
/* External inputs (root inport signals with default storage) */
ExtU_stm32 stm32_U_yaw;
ExtU_stm32 stm32_U_pitch;
ExtU_stm32 stm32_U_auto_yaw;
ExtU_stm32 stm32_U_auto_pitch;
ExtU_stm32 stm32_U_trigger;
/* External outputs (root outports fed by signals with default storage) */
ExtY_stm32 stm32_Y_yaw;
ExtY_stm32 stm32_Y_pitch;
ExtY_stm32 stm32_Y_auto_yaw;
ExtY_stm32 stm32_Y_auto_pitch;
ExtY_stm32 stm32_Y_trigger;

/* Real-time model */
RT_MODEL_stm32 stm32_M_;
RT_MODEL_stm32 *const stm32_M = &stm32_M_;

/* Model step function */
void stm32_pid_yaw_init(void) // yaw
{
    stm32_U_yaw.P_P = 2200;
    stm32_U_yaw.P_I = 0;
    stm32_U_yaw.P_D = 55;
    stm32_U_yaw.P_N = 35;
    stm32_U_yaw.S_P = 90;
    stm32_U_yaw.S_I = 0;
    stm32_U_yaw.S_D = 5;
    stm32_U_yaw.S_N = 25;
}

void stm32_pid_pitch_init(void) // pitch
{
    stm32_U_pitch.P_P = 1300;
    stm32_U_pitch.P_I = 0;
    stm32_U_pitch.P_D = 40;
    stm32_U_pitch.P_N = 10;
    stm32_U_pitch.S_P = 30;
    stm32_U_pitch.S_I = 0;
    stm32_U_pitch.S_D = 5;
    stm32_U_pitch.S_N = 25;
//	
//	stm32_U_pitch.P_P = 2000;
//    stm32_U_pitch.P_I = 0;
//    stm32_U_pitch.P_D = 20;
//    stm32_U_pitch.P_N = 10;
//    stm32_U_pitch.S_P = 40;
//    stm32_U_pitch.S_I = 0;
//    stm32_U_pitch.S_D = 5;
//    stm32_U_pitch.S_N = 25;
}
/* 自瞄参数 */
void stm32_pid_auto_yaw_init(void) // yaw
{
    stm32_U_auto_yaw.P_P = 2500;
    stm32_U_auto_yaw.P_I = 700;
    stm32_U_auto_yaw.P_D = 55;
    stm32_U_auto_yaw.P_N = 0;
    stm32_U_auto_yaw.S_P = 90;
    stm32_U_auto_yaw.S_I = 2;
    stm32_U_auto_yaw.S_D = 5;
    stm32_U_auto_yaw.S_N = 0;
}

void stm32_pid_auto_pitch_init(void) // pitch
{
    stm32_U_auto_pitch.P_P = 1800;
    stm32_U_auto_pitch.P_I = 800;
    stm32_U_auto_pitch.P_D = 50;
    stm32_U_auto_pitch.P_N = 0;
    stm32_U_auto_pitch.S_P = 60;
    stm32_U_auto_pitch.S_I = 1;
    stm32_U_auto_pitch.S_D = 6;
    stm32_U_auto_pitch.S_N = 0;
}
void stm32_pid_trigger_init(void) // trigger
{
    stm32_U_trigger.P_P = 20;
    stm32_U_trigger.P_I = 0;
    stm32_U_trigger.P_D = 0;
    stm32_U_trigger.P_N = 1;
    stm32_U_trigger.S_P = 80;
    stm32_U_trigger.S_I = 0;
    stm32_U_trigger.S_D = 0;
    stm32_U_trigger.S_N = 1;
}

typedef struct
{
    double rtb_Sum1;
    double rtb_Reciprocal;
    double rtb_FilterDifferentiatorTF;
    double rtb_IProdOut;
    double Integrator;
    double Integrator_d;
    double TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    double TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1;

} stm32_PID_t;

stm32_PID_t stm32_pid_yaw;     // yaw
stm32_PID_t stm32_pid_pitch;   // pitch
stm32_PID_t stm32_pid_auto_yaw;     // yaw
stm32_PID_t stm32_pid_auto_pitch;   // pitch
stm32_PID_t stm32_pid_trigger; // trigger

void stm32_step_yaw(double angle_set, double angle_feedback, double speed_feedback) // yaw
{
    stm32_U_yaw.angle_set = angle_set;
    stm32_U_yaw.angle_feedback = angle_feedback;
    stm32_U_yaw.speed_feedback = speed_feedback;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_U_yaw.P_N * 0.0005;
    stm32_pid_yaw.rtb_Sum1 = 1.0 / (stm32_pid_yaw.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        (stm32_pid_yaw.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_yaw.rtb_Sum1;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_U_yaw.S_N * 0.0005;
    stm32_pid_yaw.rtb_Reciprocal = 1.0 / (stm32_pid_yaw.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
        (stm32_pid_yaw.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_yaw.rtb_Reciprocal;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_U_yaw.angle_set - stm32_U_yaw.angle_feedback;
    if (stm32_pid_yaw.rtb_FilterDifferentiatorTF > 1.5 * PI)
    {
        stm32_pid_yaw.rtb_FilterDifferentiatorTF -= 2 * PI;
    }
    else if (stm32_pid_yaw.rtb_FilterDifferentiatorTF < -1.5 * PI)
    {
        stm32_pid_yaw.rtb_FilterDifferentiatorTF += 2 * PI;
    }
    stm32_pid_yaw.rtb_IProdOut = stm32_pid_yaw.rtb_FilterDifferentiatorTF * stm32_U_yaw.P_I;
    stm32_pid_yaw.Integrator = 0.0005 * stm32_pid_yaw.rtb_IProdOut + stm32_DW_yaw.Integrator_DSTATE;
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        stm32_pid_yaw.rtb_FilterDifferentiatorTF * stm32_U_yaw.P_D -
        stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
            stm32_DW_yaw.FilterDifferentiatorTF_states;
    stm32_pid_yaw.rtb_Sum1 = ((stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                               -stm32_DW_yaw.FilterDifferentiatorTF_states) *
                                  stm32_pid_yaw.rtb_Sum1 * stm32_U_yaw.P_N +
                              (stm32_pid_yaw.rtb_FilterDifferentiatorTF * stm32_U_yaw.P_P + stm32_pid_yaw.Integrator)) -
                             stm32_U_yaw.speed_feedback;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_pid_yaw.rtb_Sum1 * stm32_U_yaw.S_D -
                                               stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
                                                   stm32_DW_yaw.FilterDifferentiatorTF_states_o;
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_yaw.rtb_Sum1 *
                                                                               stm32_U_yaw.S_I;
    stm32_pid_yaw.Integrator_d = 0.0005 *
                                     stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                 stm32_DW_yaw.Integrator_DSTATE_p;
    stm32_Y_yaw.Out1 = (stm32_pid_yaw.rtb_FilterDifferentiatorTF +
                        -stm32_DW_yaw.FilterDifferentiatorTF_states_o) *
                           stm32_pid_yaw.rtb_Reciprocal *
                           stm32_U_yaw.S_N +
                       (stm32_pid_yaw.rtb_Sum1 * stm32_U_yaw.S_P + stm32_pid_yaw.Integrator_d);

    if (stm32_Y_yaw.Out1 >= 30000)
        stm32_Y_yaw.Out1 = 30000;
    else if (stm32_Y_yaw.Out1 <= -30000)
        stm32_Y_yaw.Out1 = -30000;
    stm32_DW_yaw.Integrator_DSTATE = 0.0005 * stm32_pid_yaw.rtb_IProdOut + stm32_pid_yaw.Integrator;
    stm32_DW_yaw.FilterDifferentiatorTF_states =
        stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_yaw.FilterDifferentiatorTF_states_o = stm32_pid_yaw.rtb_FilterDifferentiatorTF;
    stm32_DW_yaw.Integrator_DSTATE_p = 0.0005 *
                                           stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                       stm32_pid_yaw.Integrator_d;
}

void stm32_step_pitch(double angle_set, double angle_feedback, double speed_feedback) // pitch
{

    stm32_U_pitch.angle_set = angle_set;
    stm32_U_pitch.angle_feedback = angle_feedback;
    stm32_U_pitch.speed_feedback = speed_feedback;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.P_N * 0.0005;
    stm32_pid_pitch.rtb_Sum1 = 1.0 / (stm32_pid_pitch.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        (stm32_pid_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_pitch.rtb_Sum1;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.S_N * 0.0005;
    stm32_pid_pitch.rtb_Reciprocal = 1.0 / (stm32_pid_pitch.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
        (stm32_pid_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_pitch.rtb_Reciprocal;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.angle_set - stm32_U_pitch.angle_feedback;

    stm32_pid_pitch.rtb_IProdOut = stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_I;
    stm32_pid_pitch.Integrator = 0.0005 * stm32_pid_pitch.rtb_IProdOut + stm32_DW_pitch.Integrator_DSTATE;
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_D -
        stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
            stm32_DW_pitch.FilterDifferentiatorTF_states;
    stm32_pid_pitch.rtb_Sum1 = ((stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                                 -stm32_DW_pitch.FilterDifferentiatorTF_states) *
                                    stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.P_N +
                                (stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_P + stm32_pid_pitch.Integrator)) -
                               stm32_U_pitch.speed_feedback;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.S_D -
                                                 stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
                                                     stm32_DW_pitch.FilterDifferentiatorTF_states_o;
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_pitch.rtb_Sum1 *
                                                                                 stm32_U_pitch.S_I;
    stm32_pid_pitch.Integrator_d = 0.0005 *
                                       stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                   stm32_DW_pitch.Integrator_DSTATE_p;
    stm32_Y_pitch.Out1 = (stm32_pid_pitch.rtb_FilterDifferentiatorTF +
                          -stm32_DW_pitch.FilterDifferentiatorTF_states_o) *
                             stm32_pid_pitch.rtb_Reciprocal *
                             stm32_U_pitch.S_N +
                         (stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.S_P + stm32_pid_pitch.Integrator_d);

    if (stm32_Y_pitch.Out1 >= 30000)
        stm32_Y_pitch.Out1 = 30000;
    else if (stm32_Y_pitch.Out1 <= -30000)
        stm32_Y_pitch.Out1 = -30000;
    stm32_DW_pitch.Integrator_DSTATE = 0.0005 * stm32_pid_pitch.rtb_IProdOut + stm32_pid_pitch.Integrator;
    stm32_DW_pitch.FilterDifferentiatorTF_states =
        stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_pitch.FilterDifferentiatorTF_states_o = stm32_pid_pitch.rtb_FilterDifferentiatorTF;
    stm32_DW_pitch.Integrator_DSTATE_p = 0.0005 *
                                             stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                         stm32_pid_pitch.Integrator_d;
}

void stm32_step_auto_yaw(double angle_set, double angle_feedback, double speed_feedback) // auto_yaw
{
    stm32_U_auto_yaw.angle_set = angle_set;
    stm32_U_auto_yaw.angle_feedback = angle_feedback;
    stm32_U_auto_yaw.speed_feedback = speed_feedback;
    stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF = stm32_U_auto_yaw.P_N * 0.0005;
    stm32_pid_auto_yaw.rtb_Sum1 = 1.0 / (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_auto_yaw.rtb_Sum1;
    stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF = stm32_U_auto_yaw.S_N * 0.0005;
    stm32_pid_auto_yaw.rtb_Reciprocal = 1.0 / (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
        (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_auto_yaw.rtb_Reciprocal;
    stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF = stm32_U_auto_yaw.angle_set - stm32_U_auto_yaw.angle_feedback;
    if (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF > 1.5 * PI)
    {
        stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF -= 2 * PI;
    }
    else if (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF < -1.5 * PI)
    {
        stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF += 2 * PI;
    }
    stm32_pid_auto_yaw.rtb_IProdOut = stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF * stm32_U_auto_yaw.P_I;
    stm32_pid_auto_yaw.Integrator = 0.0005 * stm32_pid_auto_yaw.rtb_IProdOut + stm32_DW_auto_yaw.Integrator_DSTATE;
    stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF * stm32_U_auto_yaw.P_D -
        stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
            stm32_DW_auto_yaw.FilterDifferentiatorTF_states;
    stm32_pid_auto_yaw.rtb_Sum1 = ((stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                                    -stm32_DW_auto_yaw.FilterDifferentiatorTF_states) *
                                       stm32_pid_auto_yaw.rtb_Sum1 * stm32_U_auto_yaw.P_N +
                                   (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF * stm32_U_auto_yaw.P_P + stm32_pid_auto_yaw.Integrator)) -
                                  stm32_U_auto_yaw.speed_feedback;
    stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF = stm32_pid_auto_yaw.rtb_Sum1 * stm32_U_auto_yaw.S_D -
                                                    stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
                                                        stm32_DW_auto_yaw.FilterDifferentiatorTF_states_o;
    stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_auto_yaw.rtb_Sum1 *
                                                                                    stm32_U_auto_yaw.S_I;
    stm32_pid_auto_yaw.Integrator_d = 0.0005 *
                                          stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                      stm32_DW_auto_yaw.Integrator_DSTATE_p;
    stm32_Y_auto_yaw.Out1 = (stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF +
                             -stm32_DW_auto_yaw.FilterDifferentiatorTF_states_o) *
                                stm32_pid_auto_yaw.rtb_Reciprocal *
                                stm32_U_auto_yaw.S_N +
                            (stm32_pid_auto_yaw.rtb_Sum1 * stm32_U_auto_yaw.S_P + stm32_pid_auto_yaw.Integrator_d);

    if (stm32_Y_auto_yaw.Out1 >= 30000)
        stm32_Y_auto_yaw.Out1 = 30000;
    else if (stm32_Y_auto_yaw.Out1 <= -30000)
        stm32_Y_auto_yaw.Out1 = -30000;
    stm32_DW_auto_yaw.Integrator_DSTATE = 0.0005 * stm32_pid_auto_yaw.rtb_IProdOut + stm32_pid_auto_yaw.Integrator;
    stm32_DW_auto_yaw.FilterDifferentiatorTF_states =
        stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_auto_yaw.FilterDifferentiatorTF_states_o = stm32_pid_auto_yaw.rtb_FilterDifferentiatorTF;
    stm32_DW_auto_yaw.Integrator_DSTATE_p = 0.0005 *
                                                stm32_pid_auto_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                            stm32_pid_auto_yaw.Integrator_d;
}

void stm32_step_auto_pitch(double angle_set, double angle_feedback, double speed_feedback) // auto_pitch
{

    stm32_U_auto_pitch.angle_set = angle_set;
    stm32_U_auto_pitch.angle_feedback = angle_feedback;
    stm32_U_auto_pitch.speed_feedback = speed_feedback;
    stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_U_auto_pitch.P_N * 0.0005;
    stm32_pid_auto_pitch.rtb_Sum1 = 1.0 / (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_auto_pitch.rtb_Sum1;
    stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_U_auto_pitch.S_N * 0.0005;
    stm32_pid_auto_pitch.rtb_Reciprocal = 1.0 / (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
        (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_auto_pitch.rtb_Reciprocal;
    stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_U_auto_pitch.angle_set - stm32_U_auto_pitch.angle_feedback;

    stm32_pid_auto_pitch.rtb_IProdOut = stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF * stm32_U_auto_pitch.P_I;
    stm32_pid_auto_pitch.Integrator = 0.0005 * stm32_pid_auto_pitch.rtb_IProdOut + stm32_DW_auto_pitch.Integrator_DSTATE;
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF * stm32_U_auto_pitch.P_D -
        stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
            stm32_DW_auto_pitch.FilterDifferentiatorTF_states;
    stm32_pid_auto_pitch.rtb_Sum1 = ((stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                                      -stm32_DW_auto_pitch.FilterDifferentiatorTF_states) *
                                         stm32_pid_auto_pitch.rtb_Sum1 * stm32_U_auto_pitch.P_N +
                                     (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF * stm32_U_auto_pitch.P_P + stm32_pid_auto_pitch.Integrator)) -
                                    stm32_U_auto_pitch.speed_feedback;
    stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_pid_auto_pitch.rtb_Sum1 * stm32_U_auto_pitch.S_D -
                                                      stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
                                                          stm32_DW_auto_pitch.FilterDifferentiatorTF_states_o;
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_auto_pitch.rtb_Sum1 *
                                                                                      stm32_U_auto_pitch.S_I;
    stm32_pid_auto_pitch.Integrator_d = 0.0005 *
                                            stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                        stm32_DW_auto_pitch.Integrator_DSTATE_p;
    stm32_Y_auto_pitch.Out1 = (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF +
                               -stm32_DW_auto_pitch.FilterDifferentiatorTF_states_o) *
                                  stm32_pid_auto_pitch.rtb_Reciprocal *
                                  stm32_U_auto_pitch.S_N +
                              (stm32_pid_auto_pitch.rtb_Sum1 * stm32_U_auto_pitch.S_P + stm32_pid_auto_pitch.Integrator_d);

    if (stm32_Y_auto_pitch.Out1 >= 30000)
        stm32_Y_auto_pitch.Out1 = 30000;
    else if (stm32_Y_auto_pitch.Out1 <= -30000)
        stm32_Y_auto_pitch.Out1 = -30000;
    stm32_DW_auto_pitch.Integrator_DSTATE = 0.0005 * stm32_pid_auto_pitch.rtb_IProdOut + stm32_pid_auto_pitch.Integrator;
    stm32_DW_auto_pitch.FilterDifferentiatorTF_states =
        stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_auto_pitch.FilterDifferentiatorTF_states_o = stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF;
    stm32_DW_auto_pitch.Integrator_DSTATE_p = 0.0005 *
                                                  stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                              stm32_pid_auto_pitch.Integrator_d;
}

void stm32_step_trigger(double angle_set, double angle_feedback, double speed_feedback) // pitch
{

    stm32_U_trigger.angle_set = angle_set;
    stm32_U_trigger.angle_feedback = angle_feedback;
    stm32_U_trigger.speed_feedback = speed_feedback;
    stm32_pid_trigger.rtb_FilterDifferentiatorTF = stm32_U_trigger.P_N * 0.0005;
    stm32_pid_trigger.rtb_Sum1 = 1.0 / (stm32_pid_trigger.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        (stm32_pid_trigger.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_trigger.rtb_Sum1;
    stm32_pid_trigger.rtb_FilterDifferentiatorTF = stm32_U_trigger.S_N * 0.0005;
    stm32_pid_trigger.rtb_Reciprocal = 1.0 / (stm32_pid_trigger.rtb_FilterDifferentiatorTF + 1.0);
    stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
        (stm32_pid_trigger.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_trigger.rtb_Reciprocal;
    stm32_pid_trigger.rtb_FilterDifferentiatorTF = stm32_U_trigger.angle_set - stm32_U_trigger.angle_feedback;

    stm32_pid_trigger.rtb_IProdOut = stm32_pid_trigger.rtb_FilterDifferentiatorTF * stm32_U_trigger.P_I;
    stm32_pid_trigger.Integrator = 0.0005 * stm32_pid_trigger.rtb_IProdOut + stm32_DW_trigger.Integrator_DSTATE;
    stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        stm32_pid_trigger.rtb_FilterDifferentiatorTF * stm32_U_trigger.P_D -
        stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
            stm32_DW_trigger.FilterDifferentiatorTF_states;
    stm32_pid_trigger.rtb_Sum1 = ((stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                                   -stm32_DW_trigger.FilterDifferentiatorTF_states) *
                                      stm32_pid_trigger.rtb_Sum1 * stm32_U_trigger.P_N +
                                  (stm32_pid_trigger.rtb_FilterDifferentiatorTF * stm32_U_trigger.P_P + stm32_pid_trigger.Integrator)) -
                                 stm32_U_trigger.speed_feedback;
    stm32_pid_trigger.rtb_FilterDifferentiatorTF = stm32_pid_trigger.rtb_Sum1 * stm32_U_trigger.S_D -
                                                   stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
                                                       stm32_DW_trigger.FilterDifferentiatorTF_states_o;
    stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_trigger.rtb_Sum1 *
                                                                                   stm32_U_trigger.S_I;
    stm32_pid_trigger.Integrator_d = 0.0005 *
                                         stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                     stm32_DW_trigger.Integrator_DSTATE_p;
    stm32_Y_trigger.Out1 = (stm32_pid_trigger.rtb_FilterDifferentiatorTF +
                            -stm32_DW_trigger.FilterDifferentiatorTF_states_o) *
                               stm32_pid_trigger.rtb_Reciprocal *
                               stm32_U_trigger.S_N +
                           (stm32_pid_trigger.rtb_Sum1 * stm32_U_trigger.S_P + stm32_pid_trigger.Integrator_d);

    if (stm32_Y_trigger.Out1 >= 16000)
        stm32_Y_trigger.Out1 = 16000;
    else if (stm32_Y_trigger.Out1 <= -16000)
        stm32_Y_trigger.Out1 = -16000;
    stm32_DW_trigger.Integrator_DSTATE = 0.0005 * stm32_pid_trigger.rtb_IProdOut + stm32_pid_trigger.Integrator;
    stm32_DW_trigger.FilterDifferentiatorTF_states =
        stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_trigger.FilterDifferentiatorTF_states_o = stm32_pid_trigger.rtb_FilterDifferentiatorTF;
    stm32_DW_trigger.Integrator_DSTATE_p = 0.0005 *
                                               stm32_pid_trigger.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                           stm32_pid_trigger.Integrator_d;
}

// 清pid
void stm32_pid_clear_yaw(void)
{
    memset((void *)&stm32_DW_yaw, 0, sizeof(stm32_DW_yaw));
    memset((void *)&stm32_pid_yaw, 0, sizeof(stm32_pid_yaw));
} 
void stm32_pid_clear_pitch(void)
{
    memset((void *)&stm32_DW_pitch, 0, sizeof(stm32_DW_pitch));
    memset((void *)&stm32_pid_pitch, 0, sizeof(stm32_pid_pitch));
}
void stm32_pid_clear_auto_yaw(void)
{
    memset((void *)&stm32_DW_auto_yaw, 0, sizeof(stm32_DW_auto_yaw));
    memset((void *)&stm32_pid_auto_yaw, 0, sizeof(stm32_pid_auto_yaw));
}
void stm32_pid_clear_auto_pitch(void)
{
    memset((void *)&stm32_DW_auto_pitch, 0, sizeof(stm32_DW_auto_pitch));
    memset((void *)&stm32_pid_auto_pitch, 0, sizeof(stm32_pid_auto_pitch));
}

/* Model initialize function */
void stm32_initialize(void)
{
    /* (no initialization code required) */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] stm32.c
 */

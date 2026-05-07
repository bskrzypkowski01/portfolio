/*
 * pi_controller_current.h
 *
 *  Created on: Jun 25, 2025
 *      Author: WODZU
 */

#ifndef INC_PI_CONTROLLER_CURRENT_H_
#define INC_PI_CONTROLLER_CURRENT_H_

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Ts;
    float integral;
    float out_min;
    float out_max;
    float integral_limit;// nowy parametr: maksymalna wartość całki
    float pic_current_output_scaling;
} PI_Controller_current;

void PIC_Init(PI_Controller_current *pic, float Kp, float Ki, float Ts, float out_min, float out_max);
float PIC_Update(PI_Controller_current *pic, float setpoint_c, float current_out);
void PIC_Reset(PI_Controller_current *pic);
void PIC_SetIntegralLimit(PI_Controller_current *pic, float limit); // nowa funkcja
void PIC_ChangeNastawa(PI_Controller_current *pic, float Kp, float Ki);

#endif /* INC_PI_CONTROLLER_CURRENT_H_ */


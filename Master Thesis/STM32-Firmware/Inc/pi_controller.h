/*
 * pi_controller.h
 *
 *  Created on: Jun 4, 2025
 *      Author: birdd
 */

#ifndef INC_PI_CONTROLLER_H_
#define INC_PI_CONTROLLER_H_

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Ts;
    float integral;
    float out_min;
    float out_max;
    float integral_limit; // nowy parametr: maksymalna wartość całki
    float pi_voltage_output_scaling;
} PI_Controller;

void PI_Init(PI_Controller *pi, float Kp, float Ki, float Ts, float out_min, float out_max);
float PI_Update(PI_Controller *pi, float setpoint, float voltage_out);
void PI_Reset(PI_Controller *pi);
void PI_SetIntegralLimit(PI_Controller *pi, float limit); // nowa funkcja
void PI_ChangeNastawa(PI_Controller *pi, float Kp, float Ki);
#endif /* INC_PI_CONTROLLER_H_ */

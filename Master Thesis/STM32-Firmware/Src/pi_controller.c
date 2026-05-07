/*
 * pi_controller.c
 *
 *  Created on: Jun 4, 2025
 *      Author: birdd
 */

#include "pi_controller.h"

void PI_Init(PI_Controller *pi, float Kp, float Ki, float Ts, float out_min, float out_max) {
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->Ts = Ts;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->pi_voltage_output_scaling=1;
    pi->integral_limit = (pi->out_max / pi->Ki);
}

float PI_Update(PI_Controller *pi, float setpoint, float measurement_voltage) {

	float error = setpoint - measurement_voltage;

    // Człon całkujący z ograniczeniem
    pi->integral += error * pi->Ts;



    // Anti-windup (bazujące na wyjściu)
    if (pi->integral * pi->Ki > pi->out_max)
        pi->integral = pi->integral_limit;
    else if (pi->integral * pi->Ki < pi->out_min)
        pi->integral = pi->out_min / pi->Ki;

    float output = pi->Kp * error + pi->Ki * pi->integral;
   // float output = U_reg*pi->pi_voltage_output_scaling;

    if (output > pi->out_max) output = pi->out_max;
    if (output < pi->out_min) output = pi->out_min;

    return output;
}

void PI_Reset(PI_Controller *pi) {
    pi->integral = 0.0f;
}

void PI_SetIntegralLimit(PI_Controller *pi, float limit) {
    pi->integral= limit;
}
void PI_ChangeNastawa(PI_Controller *pi, float Kp, float Ki)
{
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->integral_limit = pi->out_max / pi->Ki;
}


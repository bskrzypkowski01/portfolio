/*
 * pi_controller_current.c
 *
 *  Created on: Jun 25, 2025
 *      Author: WODZU
 */

#include "pi_controller_current.h"

void PIC_Init(PI_Controller_current *pic, float Kp, float Ki, float Ts, float out_min, float out_max) {
    pic->Kp = Kp;
    pic->Ki = Ki;
    pic->Ts = Ts;
    pic->integral = 0.0f;
    pic->out_min = out_min;
    pic->out_max = out_max;
    pic->pic_current_output_scaling=1;
    pic->integral_limit = (pic->out_max / pic->Ki); //
}

float PIC_Update(PI_Controller_current *pic, float setpoint_c, float current_out) {
    		//if(current_out<0) current_out = 0;

	float error = setpoint_c - current_out;

    // Człon całkujący z ograniczeniem
    pic->integral += error * pic->Ts;


    // Anti-windup
    if (pic->integral * pic->Ki > pic->out_max)
        pic->integral = pic->integral_limit;
    else if (pic->integral * pic->Ki < pic->out_min)
        pic->integral = pic->out_min / pic->Ki;

    float output = (pic->Kp * error + pic->Ki * pic->integral);

    if (output > pic->out_max) output = pic->out_max;
       if (output < pic->out_min) output = pic->out_min;




    return output*pic->pic_current_output_scaling;
}

void PIC_Reset(PI_Controller_current *pic) {
    pic->integral = 0.0f;
}

void PIC_SetIntegralLimit(PI_Controller_current *pic, float limit) {
    pic->integral= limit;
}
void PIC_ChangeNastawa(PI_Controller_current *pic, float Kp, float Ki) {
    pic->Kp = Kp;
    pic->Ki = Ki;
    pic->integral_limit = pic->out_max / pic->Ki;
}


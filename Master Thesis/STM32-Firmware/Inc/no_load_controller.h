/*
 * pi_controller.h
 *
 *  Created on: Jun 25, 2025
 *      Author: WODZU
 */

#ifndef INC_NO_LOAD_CONTROLLER_H_
#define INC_NO_LOAD_CONTROLLER_H_


#include "main.h"

typedef struct {

    float limit_high;
    float limit_low;
    int licznik;
    float previous_setpoint;
    int current_case;
} NO_LOAD_CONTROLLER;

void NOL_Init(NO_LOAD_CONTROLLER *nol, float limit_high, float limit_low, int licznik);
float NOL_Update(NO_LOAD_CONTROLLER *nol, float setpoint, float voltage_out, float voltage_in);
void NOL_Reset(NO_LOAD_CONTROLLER *nol);

#endif /* INC_NO_LOAD_CONTROLLER_ */

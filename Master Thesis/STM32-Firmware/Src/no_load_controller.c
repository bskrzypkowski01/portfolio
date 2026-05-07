/*
 * no_load_controller.c
 *
 *  Created on: Jun 25, 2025
 *      Author: WODZU
 */

#include "no_load_controller.h"

void NOL_Init(NO_LOAD_CONTROLLER *nol, float limit_high, float limit_low, int licznik)
		 {
    nol->limit_high = limit_high;
    nol->limit_low = limit_low;
    nol->licznik = licznik;
    nol->previous_setpoint = 0.0f;
}

float NOL_Update(NO_LOAD_CONTROLLER *nol, float setpoint, float voltage_out, float voltage_in) {
    float output = 0.0f;

    int transition = 0;

    if (nol->previous_setpoint != setpoint) {
           // Nowa zmiana — wykryj przejście
           if (fabsf(nol->previous_setpoint - 0.0f) < 0.01f && fabsf(setpoint - 20.0f) < 0.01f)
               transition = 1;
           else if (fabsf(nol->previous_setpoint - 20.0f) < 0.01f && fabsf(setpoint - 60.0f) < 0.01f)
               transition = 2;
           else if (fabsf(nol->previous_setpoint - 60.0f) < 0.01f && fabsf(setpoint - 150.0f) < 0.01f)
               transition = 3;
           else if (fabsf(nol->previous_setpoint - 150.0f) < 0.01f && fabsf(setpoint - 60.0f) < 0.01f)
               transition = 4;
           else if (fabsf(nol->previous_setpoint - 60.0f) < 0.01f && fabsf(setpoint - 20.0f) < 0.01f)
               transition = 5;
           else if (fabsf(nol->previous_setpoint - 20.0f) < 0.01f && fabsf(setpoint - 0.0f) < 0.01f)
               transition = 6;
        // Zapisz aktywny case
        nol->current_case = transition;
        nol->licznik = 0;

        // Zaktualizuj stan poprzedniego setpointu
        nol->previous_setpoint = setpoint;
    }

    // Obsługa wybranego (zapamiętanego) case
    switch (nol->current_case) {
        case 1:  // 0 -> 20
            if (nol->licznik <= 2) {
                output = 830.0f;
                nol->licznik++;
            } else if (nol->licznik < 50) {
                nol->licznik++;
            } else {
                output = (voltage_out < (setpoint - 0.05f)) ? 6.0f : 0.0f;
            }
            break;

        case 2:  // 20 -> 60
        	 if (nol->licznik <= 10) {
        	                output = 1150.0f;
        	                nol->licznik++;
        	            } else if (nol->licznik < 50) {
        	                nol->licznik++;
        	            } else {
        	                output = (voltage_out < (setpoint - 0.05f)) ? 6.0f : 0.0f;
        	            }
        	            break;


        case 3:  // 60 -> 150
        	 if (nol->licznik <= 17) {
        	        	                output = 1160.0f;
        	        	                nol->licznik++;
        	        	            } else if (nol->licznik < 17) {
        	        	                nol->licznik++;
        	        	            } else {
        	        	                output = (voltage_out < (setpoint - 0.05f)) ? 60.0f : 0.0f;
        	        	            }


            break;

        case 4:  // 150 -> 60

        	if (nol->licznik <= 50) {
        	        	     output = -1150.0f;
        	        	    nol->licznik++;}
        	        	 else {
        	          output = (voltage_out < (setpoint - 0.05f)) ? 6.0f : 0.0f;
        	          nol->licznik=nol->licznik +200;
        	        	         	        	            }
            break;

        case 5:  // 60 -> 20
        	 if (nol->licznik <= 50) {
        	     output = -1150.0f;
        	    nol->licznik++;}
        	 else {
          output = (voltage_out < (setpoint - 0.05f)) ? 6.0f : 0.0f;
          nol->licznik=nol->licznik +200;
        	         	        	            }


            break;

        case 6:  // 20 -> 0
            output =-1200;
            break;

        default:
        	if (voltage_out > (setpoint + 0.5f)) {
        	    output = -1100.0f;
        	} else if (voltage_out > (setpoint + 0.1f)) {
        	    output = -100.0f;
        	} else if (voltage_out < (setpoint - 0.05f)) {
        	    output = 6.0f;
        	} else {
        	    output = 0.0f;
        	}
        	break;
    }

    // Dodatkowe warunki niezależne
 //   if (voltage_out > setpoint) {
 //       output = 0.0f;
//    }
    if (fabsf(setpoint) < 0.1f && voltage_out < 0.1f) {
        output = 0.0f;
    }

    return output;
}


void NOL_Reset(NO_LOAD_CONTROLLER *nol) {
	  nol->licznik = 0;
	    nol->previous_setpoint = 0.0f;
}

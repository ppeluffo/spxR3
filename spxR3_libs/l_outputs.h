/*
 * l_outputs.h
 *
 *  Created on: 14 jul. 2018
 *      Author: pablo
 */

#ifndef SRC_SPXR3_LIBS_L_OUTPUTS_H_
#define SRC_SPXR3_LIBS_L_OUTPUTS_H_

#include "stdio.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../spxR3_libs/l_iopines.h"

typedef enum { OUT_ENABLE = 0, OUT_DISABLE, OUT_SLEEP, OUT_AWAKE, OUT_SET_01, OUT_SET_10, V_OPEN, V_CLOSE, PULSE } t_outputs_cmd;

// General
void OUT_config(void);

// Pines
void OUT_power_on(void);
void OUT_power_off(void);
int OUT_sleep_pin( uint8_t modo );
int OUT_reset_pin( uint8_t modo );
int OUT_enable_pin( char driver_id, uint8_t modo );
int OUT_phase_pin( char driver_id, uint8_t modo );

// Driver outputs
void OUT_driver( char driver_id, uint8_t cmd );	// enable,disable,sleep,awake,01,10

// Valvulas
int OUT_valve( char driver_id, uint8_t cmd, uint8_t duracion );// open,close


#endif /* SRC_SPXR3_LIBS_L_OUTPUTS_H_ */

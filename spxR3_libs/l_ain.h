/*
 * l_ain.h
 *
 *  Created on: 14 jul. 2018
 *      Author: pablo
 */

#ifndef SRC_SPXR3_LIBS_L_AIN_H_
#define SRC_SPXR3_LIBS_L_AIN_H_

#include "frtos-io.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <avr/pgmspace.h>
#include "l_ina3221.h"

#include "../spxR3_libs/l_ina3221.h"
#include "../spxR3_libs/l_printf.h"

//------------------------------------------------------------------------------------

void ACH_config( uint8_t ina_id, uint16_t conf_reg_value );
uint16_t ACH_read_channel( uint8_t channel_id );

#define ACH_read_battery()	ACH_read_channel(5)
#define ACH_config_avg128(ina_id)	ACH_config(ina_id, CONF_INAS_AVG128 )
#define ACH_config_sleep(ina_id)	ACH_config(ina_id, CONF_INAS_SLEEP )
#define ACH_prender_12V()	IO_set_SENS_12V_CTL()
#define ACH_apagar_12V()	IO_clr_SENS_12V_CTL()

#endif /* SRC_SPXR3_LIBS_L_AIN_H_ */

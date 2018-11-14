/*
 * sp5KV5_tkGprs_monitor_sqe.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spxR3_tkGprs.h"

// Estoy en modo comando por lo que no importa tanto el wdg.
// Le doy 15 minutos

#define WDG_GPRS_TO_SQE	900

//------------------------------------------------------------------------------------
bool st_gprs_monitor_sqe(void)
{
	// Me quedo en un loop infinito preguntando por el SQE c/10s y
	// mostrando el resultado.

uint8_t MON_timer = 1;

	GPRS_stateVars.state = G_MON_SQE;

	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_SQE );

	while ( GPRS_stateVars.monitor_sqe ) {

		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

		if ( MON_timer > 0) {	// Espero 1s contando
			MON_timer--;
		} else {
			// Expiro: monitoreo el SQE y recargo el timer.
			pub_gprs_ask_sqe();
			MON_timer = 10;
		}

	}

	// No estoy en modo mon_sqe: permite salir y continuar el flujo
	return( bool_CONTINUAR );
}
//------------------------------------------------------------------------------------

/*
 * sp5KV5_tkGprs_esperar_apagado.c
 *
 *  Created on: 28 de abr. de 2017
 *      Author: pablo
 *
 *  En este subestado el modem se apaga y espera aqui hasta que expire el tiempo
 *  del timerdial.
 *  Si esta en modo continuo, (timerdial = 0 ), espero 60s.
 *  En este estado es donde entro en el modo tickless !!!.
 */

#include "spxR3_tkGprs.h"

static int32_t waiting_time;

static bool pv_tkGprs_check_inside_pwrSave(void);
static void pv_tkGprs_calcular_tiempo_espera(void);
static bool pv_tkGprs_procesar_signals_espera( bool *exit_flag );

//------------------------------------------------------------------------------------
bool st_gprs_esperar_apagado(void)
{
	// Calculo el tiempo a esperar y espero. El intervalo no va a considerar el tiempo
	// posterior de proceso.

bool exit_flag = false;

	GPRS_stateVars.state = G_ESPERA_APAGADO;

	// Secuencia para apagar el modem y dejarlo en modo low power.
	xprintf_P( PSTR("GPRS: modem off.\r\n\0"));

	// Actualizo todas las variables para el estado apagado.
	GPRS_stateVars.modem_prendido = false;
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	GPRS_stateVars.signal_redial = false;
	GPRS_stateVars.dcd = 1;

	// Apago
	pub_gprs_modem_pwr_off();

	pv_tkGprs_calcular_tiempo_espera();

	// Espera
	// Salgo porque expiro el tiempo o indicaron un redial ( cmd mode )
//	IO_clr_PWR_SLEEP();
	while ( ! exit_flag )  {

		//xprintf_P(PSTR("DEBUG: signal in %d\r\n\0"),exit_flag );
		// Reinicio el watchdog
		pub_ctl_watchdog_kick(WDG_GPRSTX, ( waiting_time + 300));

		// Espero de a 5s para poder entrar en tickless.
		vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );
		waiting_time -= 5;

		//xprintf_P(PSTR("DEBUG: %d\r\n\0"),waiting_time );
		// Proceso las señales
		if ( pv_tkGprs_procesar_signals_espera( &exit_flag )) {
			// Si recibi alguna senal, debo salir.
			//xprintf_P(PSTR("DEBUG: signal out %d\r\n\0"),exit_flag );
			goto EXIT;
		}

		// Expiro el tiempo de espera. Veo si estoy dentro del intervalo de
		// pwrSave y entonces espero de a 10 minutos mas.
		if ( waiting_time <= 0 ) {
			if ( pv_tkGprs_check_inside_pwrSave() ) {
				waiting_time = 600;
			} else {
				// Salgo con true.
				exit_flag = bool_CONTINUAR;
				goto EXIT;
			}
		}

	}

EXIT:

	// No espero mas y salgo del estado prender.
	GPRS_stateVars.signal_redial = false;
	waiting_time = -1;
//	IO_set_PWR_SLEEP();
	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//----------------------------------------------------------------------------------------
static void pv_tkGprs_calcular_tiempo_espera(void)
{

static bool starting_flag = true;

	// Cuando arranco ( la primera vez) solo espero 10s y disco por primera vez
	if ( starting_flag ) {
		waiting_time = 10;
		starting_flag = false;
		goto EXIT;
	}

	// En modo MONITOR_SQE espero solo 60s
	if ( GPRS_stateVars.monitor_sqe ) {
		waiting_time = 60;
		goto EXIT;
	}

	// En modo CONTINUO ( timerDial = 0 ) espero solo 60s.
	if ( ! MODO_DISCRETO ) {
		waiting_time = 60;
		goto EXIT;
	}

	// En modo DISCRETO ( timerDial > 900 )
	if ( MODO_DISCRETO ) {
		waiting_time = systemVars.timerDial;
		goto EXIT;
	}

	// En cualquier otro caso no considerado, espero 60s
	waiting_time = 60;
	goto EXIT;

EXIT:

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: await %lu s\r\n\0"), waiting_time );
	}

}
//------------------------------------------------------------------------------------
static bool pv_tkGprs_check_inside_pwrSave(void)
{
	// Calculo el waiting time en modo DISCRETO evaluando si estoy dentro
	// del periodo de pwrSave.
	// En caso afirmativo, seteo el tiempo en 10mins ( 600s )
	// En caso negativo, lo seteo en systemVars.timerDial

static bool starting_flag_pws = true;
bool insidePwrSave_flag = false;
RtcTimeType_t rtc;
uint16_t now, pwr_save_start, pwr_save_end ;
int8_t xBytes;;

	// Estoy en modo PWR_DISCRETO con PWR SAVE ACTIVADO
	if ( ( MODO_DISCRETO ) && ( systemVars.pwrSave.modo == modoPWRSAVE_ON )) {

		// Cuando arranco siempre me conecto sin importat si estoy o no en pwr save !!
		if ( starting_flag_pws ) {
			starting_flag_pws = false;
			goto EXIT;
		}

		xBytes = RTC_read_dtime(&rtc);
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_tkGprs_check_inside_pwrSave\r\n\0"));

		now = rtc.hour * 60 + rtc.min;
		pwr_save_start = systemVars.pwrSave.hora_start.hour * 60 + systemVars.pwrSave.hora_start.min;
		pwr_save_end = systemVars.pwrSave.hora_fin.hour * 60 + systemVars.pwrSave.hora_fin.min;

		if ( pwr_save_start < pwr_save_end ) {
			// Caso A:
			if ( now <= pwr_save_start ) { insidePwrSave_flag = false; goto EXIT; }
			// Caso B:
			if ( ( pwr_save_start <= now ) && ( now <= pwr_save_end )) { insidePwrSave_flag = true; goto EXIT; }
			// Caso C:
			if ( now > pwr_save_end ) { insidePwrSave_flag = false; goto EXIT; }
		}

		if (  pwr_save_end < pwr_save_start ) {
			// Caso A:
			if ( now <=  pwr_save_end ) { insidePwrSave_flag = true; goto EXIT; }
			// Caso B:
			if ( ( pwr_save_end <= now ) && ( now <= pwr_save_start )) { insidePwrSave_flag = false; goto EXIT; }
			// Caso C:
			if ( now > pwr_save_start ) { insidePwrSave_flag = true; goto EXIT; }
		}

	}

EXIT:

	if ( systemVars.debug == DEBUG_GPRS) {
		if ( insidePwrSave_flag == true ) {
			xprintf_P( PSTR("GPRS: inside pwrsave\r\n\0"));
		} else {
			xprintf_P( PSTR("GPRS: out pwrsave\r\n\0"));
		}
	}

	return(insidePwrSave_flag);

}
//-----------------------------------------------------------------------------------
static bool pv_tkGprs_procesar_signals_espera( bool *exit_flag )
{

bool ret_f = false;

	//xprintf_P(PSTR("DEBUG: process signal in %d\r\n\0"), *exit_flag );

	if ( GPRS_stateVars.signal_redial) {
		// Salgo a discar inmediatamente.
		*exit_flag = bool_CONTINUAR;
		ret_f = true;
		//xprintf_P(PSTR("DEBUG: process signal out1 %d\r\n\0"), *exit_flag );
		goto EXIT;
	}

/*
	if ( GPRS_stateVars.signal_frameReady) {
		// En modo CONTINUO lo mas que espero son 60s por lo
		// tanto no considero el frame_ready. Salgo a discar solo en DISCRETO
		// que las esperas pueden ser largas.
		if ( ! MODO_DISCRETO ) {
			*exit_flag = bool_CONTINUAR;
			ret_f = true;
			xprintf_P(PSTR("DEBUG: process signal out2 %d\r\n\0"), *exit_flag );
			goto EXIT;
		}
	}
*/
	ret_f = false;
EXIT:

	GPRS_stateVars.signal_redial = false;
//	GPRS_stateVars.signal_frameReady = false;

	return(ret_f);
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//----------------------------------------------------------------------------------------
int32_t pub_gprs_readTimeToNextDial(void)
{
	return(waiting_time);
}
//----------------------------------------------------------------------------------------
void pub_gprs_redial(void)
{
	GPRS_stateVars.signal_redial = true;

}
//----------------------------------------------------------------------------------------

/*
 * spx_analog.c
 *
 *  Created on: 14 de mar. de 2018
 *      Author: pablo
 */

#include "spxR3.h"

//------------------------------------------------------------------------------------
void pub_analog_config_INAS( uint16_t conf_reg_value )
{
	switch ( conf_reg_value ) {
	case CONF_INAS_AVG128:
		// Configuro ambos INA para promediar en 128 medidas.
		ACH_config_avg128(0);
		ACH_config_avg128(1);
		break;
	case CONF_INAS_SLEEP:
		// Configuro ambos INA para promediar en 128 medidas.
		ACH_config_avg128(0);
		ACH_config_avg128(1);
		break;
	default:
		break;
	}

}
//------------------------------------------------------------------------------------
void pub_analog_load_defaults(void)
{

	// Realiza la configuracion por defecto de los canales analogicos.

uint8_t channel;

	systemVars.timerPoll = 300;
	systemVars.pwr_settle_time = 1;

	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		systemVars.coef_calibracion[channel] = 3646;
		systemVars.imin[channel] = 0;
		systemVars.imax[channel] = 20;
		systemVars.mmin[channel] = 0;
		systemVars.mmax[channel] = 6.0;
		snprintf_P( systemVars.a_ch_name[channel], PARAMNAME_LENGTH, PSTR("A%d\0"),channel );

	}
}
//------------------------------------------------------------------------------------
bool pub_analog_config_channel( uint8_t channel,char *s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax )
{

	// Configura los canales analogicos. Es usada tanto desde el modo comando como desde el modo online por gprs.

bool retS = false;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	pub_control_string(s_aname);

	if ( ( channel >=  0) && ( channel < NRO_ANALOG_CHANNELS) ) {
		snprintf_P( systemVars.a_ch_name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_aname );
		if ( s_imin != NULL ) { systemVars.imin[channel] = atoi(s_imin); }
		if ( s_imax != NULL ) { systemVars.imax[channel] = atoi(s_imax); }
		if ( s_mmin != NULL ) { systemVars.mmin[channel] = atoi(s_mmin); }
		if ( s_mmax != NULL ) { systemVars.mmax[channel] = atof(s_mmax); }
		retS = true;
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);
}
//------------------------------------------------------------------------------------
void pub_analog_config_timerpoll ( char *s_timerpoll )
{
	// Configura el tiempo de poleo.
	// Se utiliza desde el modo comando como desde el modo online
	// El tiempo de poleo debe estar entre 15s y 3600s

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	systemVars.timerPoll = atoi(s_timerpoll);

	if ( systemVars.timerPoll < 15 )
		systemVars.timerPoll = 15;

	if ( systemVars.timerPoll > 3600 )
		systemVars.timerPoll = 300;

	xSemaphoreGive( sem_SYSVars );
	return;
}
//------------------------------------------------------------------------------------
void pub_analog_config_sensortime ( char *s_sensortime )
{
	// Configura el tiempo de espera entre que prendo  la fuente de los sensores y comienzo el poleo.
	// Se utiliza solo desde el modo comando.
	// El tiempo de espera debe estar entre 1s y 15s

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	systemVars.pwr_settle_time = atoi(s_sensortime);

	if ( systemVars.pwr_settle_time < 1 )
		systemVars.pwr_settle_time = 1;

	if ( systemVars.pwr_settle_time > 15 )
		systemVars.pwr_settle_time = 15;

	xSemaphoreGive( sem_SYSVars );
	return;
}
//------------------------------------------------------------------------------------
void pub_analog_config_spanfactor ( uint8_t channel, char *s_spanfactor )
{
	// Configura el factor de correccion del span de canales delos INA.
	// Esto es debido a que las resistencias presentan una tolerancia entonces con
	// esto ajustamos que con 20mA den la excursión correcta.
	// Solo de configura desde modo comando.

uint16_t span;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	span = atoi(s_spanfactor);
	systemVars.coef_calibracion[channel] = span;

	xSemaphoreGive( sem_SYSVars );
	return;

}
//------------------------------------------------------------------------------------
void pub_analog_read_battery ( float *mag_val )
{

uint16_t an_raw_val;
float an_mag_val;

	// La bateria esta en el canal 1 (bus) del INA 1.
	an_raw_val = ACH_read_battery();

	// Convierto el raw_value a la magnitud ( 8mV por count del A/D)
	an_mag_val = 0.008 * an_raw_val;
	*mag_val = an_mag_val;

}
//------------------------------------------------------------------------------------
void pub_analog_read_channel ( uint8_t channel, uint16_t *raw_val, float *mag_val )
{

	// Lee un canal analogico y devuelve en raw_val el valor leido del conversor A/D y en
	// mag_val el valor convertido a la magnitud configurada.
	// Se utiliza desde el modo comando como desde el modulo de poleo de las entradas.

uint16_t an_raw_val;
float an_mag_val;
float I,M,P;
uint16_t D;

	an_raw_val =ACH_read_channel(channel);
	*raw_val = an_raw_val;

	// Convierto el raw_value a la magnitud
	// Calculo la corriente medida en el canal
	I = (float)( an_raw_val) * 20 / ( systemVars.coef_calibracion[channel] + 1);

	// Calculo la magnitud
	P = 0;
	D = systemVars.imax[channel] - systemVars.imin[channel];

	an_mag_val = 0.0;
	if ( D != 0 ) {
		// Pendiente
		P = (float) ( systemVars.mmax[channel]  -  systemVars.mmin[channel] ) / D;
		// Magnitud
		M = (float) (systemVars.mmin[channel] + ( I - systemVars.imin[channel] ) * P);
		an_mag_val = M;

	} else {
		// Error: denominador = 0.
		an_mag_val = -999.0;
	}

	*mag_val = an_mag_val;

}
//------------------------------------------------------------------------------------
void pub_analog_read_frame(st_analog_inputs_t *analog_frame )
{

	// Lee todos los canales analogicos y los deja en la estructura st_analog_frame.

uint8_t channel;
uint16_t raw_val;
float mag_val;

	// Leo los canales analogicos de datos.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
		pub_analog_read_channel (channel, &raw_val, &mag_val );
		analog_frame->ainputs[channel] = mag_val;
	}

}
//------------------------------------------------------------------------------------



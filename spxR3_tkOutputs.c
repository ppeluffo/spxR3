/*
 * spx_tkOutputs.c
 *
 *  Created on: 14 de abr. de 2018
 *      Author: pablo
 */

#include "spxR3.h"

static 	uint8_t l_out_A, l_out_B;

static void pv_out_chequear(void);
static void pv_out_check_consignas(void);
static void pv_out_check_outputs_normales(void);
static void pv_out_init(void);
static void pv_out_init_outputs_off(void);
static void pv_out_init_consignas(void);
static void pv_out_init_outputs_normales(void);

static bool reinit_consignas;

//------------------------------------------------------------------------------------
// La tarea pasa por el mismo lugar c/25s.
#define WDG_OUT_TIMEOUT	60

//------------------------------------------------------------------------------------
void tkOutputs(void * pvParameters)
{

( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	pv_out_init();
	reinit_consignas = false;

	xprintf_P( PSTR("starting tkOutputs..\r\n\0"));

	for( ;; )
	{

		pub_ctl_watchdog_kick(WDG_OUT, WDG_OUT_TIMEOUT);

		// Espero con lo que puedo entrar en tickless
		// Con 25s aseguro chequear 2 veces por minuto.
		vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );

		// Si me indican que se reconfiguraron las consignas, debo
		// reiniciarlas.
		if ( reinit_consignas ) {
			reinit_consignas = false;
			pv_out_init();
		}

		// Chequeo y aplico.
		pv_out_chequear();
	}
}
//------------------------------------------------------------------------------------
static void pv_out_chequear(void)
{

	switch(systemVars.outputs.modo) {
	case OUT_OFF:
		break;
	case OUT_CONSIGNA:
		pv_out_check_consignas();
		break;
	case OUT_NORMAL:
		pv_out_check_outputs_normales();
		break;
	}
}
//------------------------------------------------------------------------------------
static void pv_out_check_consignas(void)
{

	// Las consignas se chequean y/o setean en cualquier modo de trabajo, continuo o discreto

RtcTimeType_t rtcDateTime;
int8_t xBytes;

	xBytes = RTC_read_dtime(&rtcDateTime);
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:pv_out_check_consignas\r\n\0"));

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_diurna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_diurna.min )  ) {

		pub_output_set_consigna_diurna();
		return;
	 }

	if ( ( rtcDateTime.hour == systemVars.outputs.consigna_nocturna.hour ) &&
			( rtcDateTime.min == systemVars.outputs.consigna_nocturna.min )  ) {

		pub_output_set_consigna_nocturna();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_out_check_outputs_normales(void)
{


	// Solo cambio las salidas si cambio el systemVars.
	if ( l_out_A != systemVars.outputs.out_A) {
		l_out_A = systemVars.outputs.out_A;
		( l_out_A == 0 ) ?	pub_output_set_outputs( 'A', 0 ) : pub_output_set_outputs( 'A', 1 );
	}

	if ( l_out_B != systemVars.outputs.out_B ) {
		l_out_B = systemVars.outputs.out_B;
		( l_out_B == 0 ) ?	pub_output_set_outputs( 'B', 0 ) : pub_output_set_outputs( 'B', 1 );
	}

}
//------------------------------------------------------------------------------------
static void pv_out_init(void)
{

	OUT_power_off();

	switch(systemVars.outputs.modo) {
	case OUT_OFF:
		pv_out_init_outputs_off();
		break;
	case OUT_CONSIGNA:
		pv_out_init_consignas();
		break;
	case OUT_NORMAL:
		pv_out_init_outputs_normales();
		break;
	}

}
//------------------------------------------------------------------------------------
static void pv_out_init_outputs_off(void)
{

	OUT_driver('A', OUT_SLEEP);
	OUT_driver('B', OUT_SLEEP);

	OUT_power_off();

}
//------------------------------------------------------------------------------------
static void pv_out_init_consignas(void)
{
	// Determino cual consigna corresponde aplicar y la aplico.

RtcTimeType_t rtcDateTime;
uint16_t now, horaConsNoc, horaConsDia ;
uint8_t consigna_a_aplicar = 99;
uint8_t xBytes;

	// Hora actual en minutos.
	xBytes = RTC_read_dtime(&rtcDateTime);
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:pv_out_init_consignas\r\n\0"));

	// Caso 1: C.Diurna < C.Nocturna
	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	now = rtcDateTime.hour * 60 + rtcDateTime.min;
	horaConsDia = systemVars.outputs.consigna_diurna.hour * 60 + systemVars.outputs.consigna_diurna.min;
	horaConsNoc = systemVars.outputs.consigna_nocturna.hour * 60 + systemVars.outputs.consigna_nocturna.min;

	if ( horaConsDia < horaConsNoc ) {
		// Caso A:
		if ( now <= horaConsDia ) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
		// Caso B:
		if ( ( horaConsDia <= now ) && ( now <= horaConsNoc )) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}

		// Caso C:
		if ( now > horaConsNoc ) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
	}

	// Caso 2: C.Nocturna < Diurna
	//           C.Nocturna                      C.diurna
	// |----------|-------------------------------|---------------|
	// 0         hhmm2                          hhmm1            24
	//   diurna             nocturna                 diurna

	if (  horaConsNoc < horaConsDia ) {
		// Caso A:
		if ( now <= horaConsNoc ) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}
		// Caso B:
		if ( ( horaConsNoc <= now ) && ( now <= horaConsDia )) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
		// Caso C:
		if ( now > horaConsDia ) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}
	}

	// Aplico la consigna
	switch (consigna_a_aplicar) {
	case 99:
		// Incompatibilidad: seteo por default.
		xprintf_P( PSTR("OUTPUTS: INIT ERROR al setear consignas: horas incompatibles\r\n\0"));
		systemVars.outputs.modo = OUT_CONSIGNA;
		systemVars.outputs.consigna_diurna.hour = 05;
		systemVars.outputs.consigna_diurna.min = 30;
		systemVars.outputs.consigna_nocturna.hour = 23;
		systemVars.outputs.consigna_nocturna.min = 30;
		break;
	case CONSIGNA_DIURNA:
		pub_output_set_consigna_diurna();
		break;
	case CONSIGNA_NOCTURNA:
		pub_output_set_consigna_nocturna();
		break;
	}

}
//------------------------------------------------------------------------------------
static void pv_out_init_outputs_normales(void)
{
	// Aplica el valor indicado en systemVars a las salidas.

	OUT_power_on();
	vTaskDelay( ( TickType_t)(2000 / portTICK_RATE_MS ) );

	OUT_driver('A', OUT_ENABLE);
	OUT_driver('A', OUT_AWAKE);
	OUT_driver('B', OUT_ENABLE);
	OUT_driver('B', OUT_AWAKE);

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	l_out_A = systemVars.outputs.out_A;
	l_out_B = systemVars.outputs.out_B;

	( l_out_A == 0 ) ?	pub_output_set_outputs( 'A', 0 ) : pub_output_set_outputs( 'A', 1 );
	( l_out_B == 0 ) ?	pub_output_set_outputs( 'B', 0 ) : pub_output_set_outputs( 'B', 1 );

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_output_load_defaults(void)
{

	systemVars.outputs.modo = OUT_OFF;
	systemVars.outputs.out_A = 0;
	systemVars.outputs.out_B = 0;
	systemVars.outputs.consigna_diurna.hour = 05;
	systemVars.outputs.consigna_diurna.min = 30;
	systemVars.outputs.consigna_nocturna.hour = 23;
	systemVars.outputs.consigna_nocturna.min = 30;

}
//------------------------------------------------------------------------------------
void pub_output_config( t_outputs modo, uint16_t hhmm1, uint16_t hhmm2 )
{
	// Configura las salidas en el systemVars.
	// Es Configuracion , NO operacion por lo tanto solo configuro el modo, y en
	// caso de consigna, las horas.

//	xprintf_P( PSTR("DEBUG OUTPUTS (modo=%d,hhmm1=%d,hhmm2=%d\r\n\0"), modo,hhmm1,hhmm2);

	switch ( modo ) {
	case OUT_OFF:
		systemVars.outputs.modo = OUT_OFF;
		break;
	case OUT_CONSIGNA:
		systemVars.outputs.modo = OUT_CONSIGNA;
		u_convert_int_to_time_t(hhmm1, &systemVars.outputs.consigna_diurna);
		u_convert_int_to_time_t(hhmm2, &systemVars.outputs.consigna_nocturna);
		reinit_consignas = true;
//		xprintf_P( PSTR("DEBUG OUTPUTS CONSIGNA: (modo=%d\r\n\0"), systemVars.outputs.modo);
		break;
	case OUT_NORMAL:
		systemVars.outputs.modo = OUT_NORMAL;
		systemVars.outputs.out_A = 0;
		systemVars.outputs.out_B = 0;
//		xprintf_P( PSTR("DEBUG OUTPUTS NORMAL: (modo=%d\r\n\0"), systemVars.outputs.modo);

		break;

	}

}
//----------------------------------------------------------------------------------------
void pub_output_set_consigna_diurna(void)
{
	// En consigna diurna la valvula A (JP28) queda abierta y la valvula B (JP2) cerrada.
	//

	// Proporciono corriente.
	OUT_power_on();
	// Espero 10s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

	OUT_valve( 'A', V_OPEN, 100 );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	OUT_valve( 'B', V_CLOSE, 100 );

	OUT_power_off();

	systemVars.outputs.consigna_aplicada = CONSIGNA_DIURNA;
	xprintf_P( PSTR("OUTPUTS: Aplico Consigna Diurna\r\n\0") );
}
//----------------------------------------------------------------------------------------
void pub_output_set_consigna_nocturna(void)
{

	// Proporciono corriente.
	OUT_power_on();
	// Espero 10s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

	OUT_valve( 'A', V_CLOSE, 100 );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	OUT_valve( 'B', V_OPEN, 100 );

	OUT_power_off();

	systemVars.outputs.consigna_aplicada = CONSIGNA_NOCTURNA;
	xprintf_P( PSTR("OUTPUTS: Aplico Consigna Nocturna\r\n\0") );
}
//----------------------------------------------------------------------------------------
void pub_output_set_outputs( char id_output, uint8_t value)
{

	switch(id_output) {
	case 'A':
		( value == 0 ) ? OUT_driver( id_output, OUT_SET_10 ): OUT_driver( id_output, OUT_SET_01 );
		systemVars.outputs.out_A = value;
		break;
	case 'B':
		( value == 0 ) ? OUT_driver( id_output, OUT_SET_10 ): OUT_driver( id_output, OUT_SET_01 );
		systemVars.outputs.out_B = value;
		break;
	}

	xprintf_P( PSTR("OUTPUTS: Set out_%c=%d\r\n\0"),id_output,value );

}
//----------------------------------------------------------------------------------------


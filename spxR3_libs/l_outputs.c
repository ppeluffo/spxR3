/*
 * l_outputs.c
 *
 *  Created on: 14 jul. 2018
 *      Author: pablo
 */
#include "../spxR3_libs/l_outputs.h"

//------------------------------------------------------------------------------------
void OUT_config(void)
{
	// Configura los pines del micro que son interface del driver DRV8814.

	IO_config_ENA();
	IO_config_PHA();
	IO_config_ENB();
	IO_config_PHB();
	IO_config_V12_OUTS_CTL();
	IO_config_RES();
	IO_config_SLP();

}
//------------------------------------------------------------------------------------
void OUT_power_on(void)
{
	// Prende la fuente de 12V que alimenta el DRV8814

	IO_set_V12_OUTS_CTL();
}
//------------------------------------------------------------------------------------
void OUT_power_off(void)
{
	IO_clr_V12_OUTS_CTL();
}
//------------------------------------------------------------------------------------
int OUT_sleep_pin( uint8_t modo )
{

int xRet = -1;

	switch(modo) {
	case 0:
		IO_clr_SLP(); xRet = 1;
		break;
	case 1:
		IO_set_SLP(); xRet = 1;
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
int OUT_reset_pin( uint8_t modo )
{

int xRet = -1;

	switch(modo) {
	case 0:
		IO_clr_RES(); xRet = 1;
		break;
	case 1:
		IO_set_RES(); xRet = 1;
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
int OUT_enable_pin( char driver_id, uint8_t modo )
{

int xRet = -1;

	switch (driver_id) {

	case 'A':
		switch(modo) {
		case 0:
			IO_clr_ENA(); xRet = 1;
			break;
		case 1:
			IO_set_ENA(); ; xRet = 1;
			break;
		default:
			break;
		}
		break;

	case 'B':
		switch(modo) {
		case 0:
			IO_clr_ENB(); xRet = 1;
			break;
		case 1:
			IO_set_ENB(); xRet = 1;
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}

	return(xRet);

}
//------------------------------------------------------------------------------------
int OUT_phase_pin( char driver_id, uint8_t modo )
{

int xRet = -1;

	switch (driver_id) {

	case 'A':
		switch(modo) {
		case 0:
			IO_clr_PHA(); xRet = 1;
			break;
		case 1:
			IO_set_PHA(); xRet = 1;
			break;
		default:
			break;
		}
		break;

	case 'B':
		switch(modo) {
		case 0:
			IO_clr_PHB(); xRet = 1;
			break;
		case 1:
			IO_set_PHB(); xRet = 1;
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
// Driver outputs
// enable,disable,sleep,awake,set{01,10}
// Driver A: Salidas AOUT1,AOUT2
// Driver B: Salidas BOUT1,BOUT2
// SET_01: xOUT1 = 0, xOUT2 = 1
// SET_10: xOUT1 = 1, xOUT2 = 0

void OUT_driver( char driver_id, uint8_t cmd )
{

	switch (cmd) {
		case OUT_ENABLE:
			OUT_reset_pin(1);
			OUT_enable_pin(driver_id, 1);
			break;
		case OUT_DISABLE:
			OUT_reset_pin(0);
			OUT_enable_pin(driver_id, 0);
			break;
		case OUT_SLEEP:
			OUT_sleep_pin(0);
			break;
		case OUT_AWAKE:
			OUT_reset_pin(1);
			OUT_sleep_pin(1);
			break;
		case OUT_SET_01:
			OUT_reset_pin(1);
			OUT_sleep_pin(1);
			OUT_phase_pin(driver_id, 0);	// SET_01: xOUT1 = 0, xOUT2 = 1
			OUT_enable_pin(driver_id, 1);
			break;
		case OUT_SET_10:
			OUT_reset_pin(1);
			OUT_sleep_pin(1);
			OUT_phase_pin(driver_id, 1);	// SET_10: xOUT1 = 1, xOUT2 = 0
			OUT_enable_pin(driver_id, 1);
			break;
		default:
			break;
	}
}
//------------------------------------------------------------------------------------
// Valvulas
// open,close, pulse
// Los pulsos son de abre-cierra !!!
// Al operar sobre las valvulas se asume que hay fisicamente valvulas conectadas
// por lo tanto se debe propocionar corriente, sacar al driver del estado de reposo, generar
// la apertura/cierre, dejar al driver en reposo y quitar la corriente.
// No se aplica cuando queremos una salida FIJA !!!!
int OUT_valve( char driver_id, uint8_t cmd, uint8_t duracion )
{

int xRet = -1;

	switch (cmd) {
		case V_OPEN:	// Genero en la valvula un pulso de apertura.
			// Saco al driver 8814 de reposo.
			OUT_reset_pin(1);				// El reset y sleep son pines comunes
			OUT_sleep_pin(1);
			OUT_phase_pin(driver_id, 0);	// SET_01: xOUT1 = 0, xOUT2 = 1 ( APERTURA )
			OUT_enable_pin(driver_id, 1);
			vTaskDelay( ( TickType_t)( duracion / portTICK_RATE_MS ) );
			// Pongo el 8814 en reposo
			OUT_reset_pin(0);
			OUT_sleep_pin(0);
			OUT_enable_pin(driver_id, 0);
			xRet = 1;
			break;
		case V_CLOSE: // Genero en la valvula un pulso de cierre.
			// Saco al driver 8814 de reposo.
			OUT_reset_pin(1);				// El reset y sleep son pines comunes
			OUT_sleep_pin(1);
			OUT_phase_pin(driver_id, 1);	// SET_10: xOUT1 = 1, xOUT2 = 0	( CIERRE )
			OUT_enable_pin(driver_id, 1);
			vTaskDelay( ( TickType_t)( duracion / portTICK_RATE_MS ) );
			// Pongo el 8814 en reposo
			OUT_reset_pin(0);
			OUT_sleep_pin(0);
			OUT_enable_pin(driver_id, 0);
			xRet = 1;
			break;
		default:
			break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------




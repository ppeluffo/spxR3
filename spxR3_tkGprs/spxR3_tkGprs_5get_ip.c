/*
 * sp5KV5_tkGprs_ask_ip.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spxR3_tkGprs.h"

static bool pv_gprs_netopen(void);
static void pv_gprs_read_ip_assigned(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_IP	180

//------------------------------------------------------------------------------------
bool st_gprs_get_ip(void)
{
	// El modem esta prendido y configurado.
	// Intento hasta 3 veces pedir la IP.
	// WATCHDOG: En el peor caso demoro 2 mins.
	// La asignacion de la direccion IP es al activar el contexto con el comando AT+CGACT

bool exit_flag = bool_RESTART;

// Entry:
	GPRS_stateVars.state = G_GET_IP;
	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_IP);

	//if ( pg_gprs_activate() ) {
	if ( pv_gprs_netopen() ) {
		exit_flag = bool_CONTINUAR;
	} else {
		// Aqui es que luego de tantos reintentos no consegui la IP.
		exit_flag = bool_RESTART;
		xprintf_P( PSTR("GPRS: ERROR: ip no asignada !!.\r\n\0") );
	}

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pv_gprs_netopen(void)
{
	// Doy el comando para atachearme a la red
	// Puede demorar unos segundos por lo que espero para chequear el resultado
	// y reintento varias veces.

uint8_t reintentos = MAX_TRYES_NET_ATTCH;
uint8_t checks;

	xprintf_P( PSTR("GPRS: netopen (get IP).\r\n\0") );
	// Espero 2s para dar el comando
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	while ( reintentos-- > 0 ) {

		if ( systemVars.debug == DEBUG_GPRS ) {
			xprintf_P( PSTR("GPRS: send NETOPEN cmd (%d)\r\n\0"),reintentos );
		}

		// Envio el comando.
		// AT+NETOPEN
		pub_gprs_flush_RX_buffer();
		xCom_printf_P( fdGPRS,PSTR("AT+NETOPEN\r\0"));
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

		// Intento 5 veces ver si respondio correctamente.
		for ( checks = 0; checks < 5; checks++) {

			if ( systemVars.debug == DEBUG_GPRS ) {
				xprintf_P( PSTR("GPRS: netopen check.(%d)\r\n\0"),checks );
			}

			if ( systemVars.debug == DEBUG_GPRS ) {
				pub_gprs_print_RX_Buffer();
			}

			// Evaluo las respuestas del modem.
			if ( pub_gprs_check_response("+NETOPEN: 0")) {
				xprintf_P( PSTR("GPRS: NETOPEN OK !.\r\n\0") );
				pv_gprs_read_ip_assigned();
				return(true);

			} else if ( pub_gprs_check_response("Network opened")) {
				xprintf_P( PSTR("GPRS: NETOPEN OK !.\r\n\0") );
				pv_gprs_read_ip_assigned();
				return(true);

			} else if ( pub_gprs_check_response("+IP ERROR: Network is already opened")) {
				pv_gprs_read_ip_assigned();
				return(true);

			} else if ( pub_gprs_check_response("ERROR")) {
				break;	// Salgo de la espera

			}
			// Aun no tengo ninguna respuesta esperada.
			// espero 5s para re-evaluar la respuesta.
			vTaskDelay( (portTickType)( 5000 / portTICK_RATE_MS ) );

		}

		// No pude atachearme. Debo mandar de nuevo el comando
		// Pasaron 25s.
	}

	// Luego de varios reintentos no pude conectarme a la red.
	xprintf_P( PSTR("GPRS: NETOPEN FAIL !!.\r\n\0"));
	return(false);

}
//------------------------------------------------------------------------------------
static void pv_gprs_read_ip_assigned(void)
{
	// Tengo la IP asignada: la leo para actualizar systemVars.ipaddress

char *ts = NULL;
int i=0;
char c;

	// AT+CGPADDR para leer la IP
	pub_gprs_flush_RX_buffer();
	//xCom_printf_P( fdGPRS,PSTR("AT+CGPADDR\r\0"));
	xCom_printf_P( fdGPRS,PSTR("AT+IPADDR\r\0"));
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	// Extraigo la IP del token.
	ts = strchr( pv_gprsRxCbuffer.buffer, ':');
	ts++;
	while ( (c= *ts) != '\r') {
		systemVars.dlg_ip_address[i++] = c;
		ts++;
	}
	systemVars.dlg_ip_address[i++] = '\0';

	xprintf_P( PSTR("GPRS: ip address=[%s]\r\n\0"), systemVars.dlg_ip_address);

}
//------------------------------------------------------------------------------------

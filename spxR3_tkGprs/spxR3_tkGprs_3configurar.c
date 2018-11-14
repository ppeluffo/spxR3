/*
 * sp5KV5_tkGprs_configurar.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spxR3_tkGprs.h"

static bool pv_gprs_CPIN(void);
static bool pv_gprs_CGREG(void);
static bool pv_gprs_CGATT(void);
static void pg_gprs_APN(void);
static void pg_gprs_CIPMODE(void);
static void pg_gprs_DCDMODE(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_CONFIG	180
//------------------------------------------------------------------------------------
bool st_gprs_configurar(void)
{

	// Configuro los parametros opertativos, la banda GSM y pido una IP de modo que el
	// modem quede listo para
	// No atiendo mensajes ya que no requiero parametros operativos.
	// WATCHDOG: No demoro mas de 2 minutos en este estado

bool exit_flag = bool_RESTART;

// Entry:

	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_CONFIG);

	GPRS_stateVars.state = G_CONFIGURAR;

	xprintf_P( PSTR("GPRS: configurar.\r\n\0"));

	// Vemos que halla un pin presente.
	if ( !pv_gprs_CPIN() ) {
		exit_flag = bool_RESTART;
		goto EXIT;
	}

	// Vemos que el modem este registrado en la red
	if ( !pv_gprs_CGREG() ) {
		exit_flag = bool_RESTART;
		goto EXIT;
	}

	// Recien despues de estar registrado puedo leer la calidad de señal.
	pub_gprs_ask_sqe();

	// Me debo atachear a la red. Con esto estoy conectado a la red movil pero
	// aun no puedo trasmitir datos.
	if ( !pv_gprs_CGATT() ) {
		exit_flag = bool_RESTART;
		goto EXIT;
	}

	// Configuro parametros operativos
	//pv_gprs_CSPN();
	//pv_gprs_CNSMOD();
	//pv_gprs_CCINFO();
	//pv_gprs_CNTI();
	pg_gprs_CIPMODE();	// modo transparente.
	pg_gprs_DCDMODE();	// UART para utilizar las 7 lineas
	pg_gprs_APN();		// Configuro el APN.

	exit_flag = bool_CONTINUAR;

EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pv_gprs_CPIN(void)
{
	// Chequeo que el SIM este en condiciones de funcionar.
	// AT+CPIN?

bool retS = false;
uint8_t tryes;

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: CPIN\r\n\0"));
	}

	for ( tryes = 0; tryes < 3; tryes++ ) {
		pub_gprs_flush_RX_buffer();
		xCom_printf_P( fdGPRS,PSTR("AT+CPIN?\r\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		if ( systemVars.debug == DEBUG_GPRS ) {
			pub_gprs_print_RX_Buffer();
		}

		vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );

		pub_gprs_check_response("+CPIN: READY\0") ? (retS = true ): (retS = false) ;

		if ( retS ) {
			break;
		}

	}

	return(retS);

}
//------------------------------------------------------------------------------------
static bool pv_gprs_CGREG(void)
{
	/* Chequeo que el TE este registrado en la red.
	 Esto deberia ser automatico.
	 Normalmente el SIM esta para que el dispositivo se registre automaticamente
	 Esto se puede ver con el comando AT+COPS? el cual tiene la red preferida y el modo 0
	 indica que esta para registrarse automaticamente.
	 Este comando se usa para de-registrar y registrar en la red.
	 Conviene dejarlo automatico de modo que si el TE se puede registrar lo va a hacer.
	 Solo chequeamos que este registrado con CGREG.
	 AT+CGREG?
	 +CGREG: 0,1
	*/

bool retS = false;
uint8_t tryes;

	xprintf_P( PSTR("GPRS: NET registation\r\n\0"));

	for ( tryes = 0; tryes < 5; tryes++ ) {
		pub_gprs_flush_RX_buffer();
		xCom_printf_P( fdGPRS,PSTR("AT+CREG?\r\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		if ( systemVars.debug == DEBUG_GPRS ) {
			pub_gprs_print_RX_Buffer();
		}

		vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );

		pub_gprs_check_response("+CREG: 0,1\0") ? (retS = true ): (retS = false) ;

		if ( retS ) {
			break;
		}

	}
	return(retS);

}
//------------------------------------------------------------------------------------
static bool pv_gprs_CGATT(void)
{
	/* Una vez registrado, me atacheo a la red
	 AT+CGATT=1

	 AT+CGATT?
	 +CGATT: 1
	*/

bool retS = false;
uint8_t tryes;

	xprintf_P( PSTR("GPRS: NET attach\r\n\0"));

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CGATT=1\r\0"));
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	for ( tryes = 0; tryes < 3; tryes++ ) {
		pub_gprs_flush_RX_buffer();
		xCom_printf_P( fdGPRS,PSTR("AT+CGATT?\r\0"));
		vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
		if ( systemVars.debug == DEBUG_GPRS ) {
			pub_gprs_print_RX_Buffer();
		}

		pub_gprs_check_response("+CGATT: 1\0") ? (retS = true ): (retS = false) ;

		if ( retS ) {
			break;
		}

	}
	return(retS);

}
//------------------------------------------------------------------------------------
static void pg_gprs_CIPMODE(void)
{
	// Funcion que configura el modo transparente.

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CIPMODE=1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

}
//------------------------------------------------------------------------------------
static void pg_gprs_DCDMODE(void)
{
	// Funcion que configura el UART para utilizar las 7 lineas y el DCD indicando
	// el estado del socket.

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT&D1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CSUART=1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}
/*
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CDCDMD=0\r\0"));
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();
*/
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT&C1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}


}
//------------------------------------------------------------------------------------
static void pg_gprs_APN(void)
{
	//Defino el PDP indicando cual es el APN.

	xprintf_P( PSTR("GPRS: Set APN\r\n\0") );

	// AT+CGDCONT
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r\0"),systemVars.apn);
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	// Como puedo tener varios PDP definidos, indico cual va a ser el que se deba activar
	// al usar el comando NETOPEN.
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CSOCKSETPN=1\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}


}
//------------------------------------------------------------------------------------

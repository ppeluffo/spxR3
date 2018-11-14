/*
 * sp5KV5_tkGprs_data.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spxR3_tkGprs.h"

static bool pv_hay_datos_para_trasmitir(void);
static bool pv_trasmitir_paquete_datos(void );

static void pv_trasmitir_dataHeader( void );
static void pv_trasmitir_dataTail( void );
static void pv_trasmitir_dataRecord( void );

static bool pv_procese_respuesta_server(void);
static void pv_process_response_RESET(void);
static uint8_t pv_process_response_OK(void);
static void pv_process_response_OUTS(void);

static bool pv_check_more_Rcds4Del ( void );

static void pv_tx_dataRecord_modo_SP5K(void);
static void pv_tx_dataRecord_modo_SPX(void);

// La tarea se repite para cada paquete de datos. Esto no puede demorar
// mas de 5 minutos
#define WDG_GPRS_TO_DATA	300
//------------------------------------------------------------------------------------
bool st_gprs_data(void)
{
	// Me quedo en un loop permanente revisando si hay datos y si hay los trasmito
	// Solo salgo en caso que hubiesen problemas para trasmitir
	// Esto es que no puedo abrir un socket mas de 3 veces o que no puedo trasmitir
	// el mismo frame mas de 3 veces.
	// En estos casos salgo con false de modo de hacer un ciclo completo apagando el modem.
	// Si por algun problema no puedo trasmitir, salgo asi me apago y reinicio.
	// Si pude trasmitir o simplemente no hay datos, en modo continuo retorno TRUE.

	// Si hay datos los trasmito todos
	// Solo salgo en caso que hubiesen problemas para trasmitir
	// Esto es que no puedo abrir un socket mas de 3 veces o que no puedo trasmitir
	// el mismo frame mas de 3 veces.
	// En estos casos salgo con false de modo de hacer un ciclo completo apagando el modem.

	// Mientras estoy trasmitiendo datos no atiendo las señales; solo durante la espera en modo
	// continuo.

uint8_t sleep_time;
bool exit_flag = false;
uint8_t intentos;
bool trasmision_OK;

	GPRS_stateVars.state = G_DATA;

	xprintf_P( PSTR("GPRS: data.\r\n\0"));

	//
	while ( 1 ) {

		// Si por algun problema no puedo trasmitir, salgo asi me apago y reinicio.
		// Si pude trasmitir o simplemente no hay datos, en modo continuo retorno TRUE.

		pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_DATA );

		if ( pv_hay_datos_para_trasmitir() ) {			// Si hay datos, intento trasmitir

			for ( intentos = 0; intentos < MAX_INIT_TRYES; intentos++ ) {

				trasmision_OK = false;
				if ( pv_trasmitir_paquete_datos() && pv_procese_respuesta_server() ) {		// Intento madar el frame al servidor
					// Aqui es que anduvo todo bien y debo salir para pasar al modo DATA
					if ( systemVars.debug == DEBUG_GPRS ) {
						xprintf_P( PSTR("GPRS: data packet sent.\r\n\0" ));
					}
					trasmision_OK = true;
					break; // Salgo del FOR

				} else {

					// Espero 3s antes de reintentar
					vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );

					if ( systemVars.debug == DEBUG_GPRS ) {
						xprintf_P( PSTR("GPRS: data packey retry(%d)\r\n\0"),intentos);
					}
				}
			}

			// Veo si pude trasmitir o no.
			if ( trasmision_OK) {
				continue;	// Voy de nuevo a ver si hay mas datos para trasmitir.
			} else {
				// Falle luego de 3 reintentos. Salgo.
				exit_flag = true;
				goto EXIT;
			}

		} else {

			// No hay datos para trasmitir
			// Modo discreto, Salgo a apagarme y esperar
			if ( MODO_DISCRETO ) {
				exit_flag = bool_CONTINUAR ;
				goto EXIT;

			} else {

				// modo continuo: espero 90s antes de revisar si hay mas datos para trasmitir
				sleep_time = 90;
				while( sleep_time-- > 0 ) {
					vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

					// PROCESO LAS SEÑALES
					if ( GPRS_stateVars.signal_redial) {
						// Salgo a discar inmediatamente.
						GPRS_stateVars.signal_redial = false;
						exit_flag = bool_RESTART;
						goto EXIT;
					}

					if ( GPRS_stateVars.signal_frameReady) {
						GPRS_stateVars.signal_frameReady = false;
						break;	// Salgo del while de espera.
					}
				} // while
			} // if

		} // else

	} // while

	// Exit area:
EXIT:
	// No espero mas y salgo del estado prender.
	return(exit_flag);
}
//------------------------------------------------------------------------------------
static bool pv_hay_datos_para_trasmitir(void)
{

	/* Veo si hay datos en memoria para trasmitir
	 * Memoria vacia: rcds4wr = MAX, rcds4del = 0;
	 * Memoria llena: rcds4wr = 0, rcds4del = MAX;
	 * Memoria toda leida: rcds4rd = 0;
	 */

//	l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );

FAT_t l_fat;

	FAT_read(&l_fat);

	// Si hay registros para leer
	if ( l_fat.rcds4rd > 0) {
		return(true);

	} else {

		if ( systemVars.debug == DEBUG_GPRS ) {
			xprintf_P( PSTR("GPRS: bd EMPTY\r\n\0"));
		}
		return(false);
	}

}
//------------------------------------------------------------------------------------
static bool pv_trasmitir_paquete_datos(void )
{
	// Hay datos que intento trasmitir.
	// Leo los mismos hasta completar un TX_WINDOW
	// El socket lo chequeo antes de comenzar a trasmitir. Una vez que comienzo
	// trasmito todo y paso a esperar la respuesta donde chequeo el socket nuevamente.
	// Intento enviar el mismo paquete de datos hasta 3 veces.
	// Entro luego de haber chequeado que hay registros para trasmitir o sea que se que los hay !!

uint8_t registros_trasmitidos = 0;
uint8_t i;
bool exit_flag = false;
uint8_t timeout, await_loops;
t_socket_status socket_status;


	for ( i = 0; i < MAX_TRYES_OPEN_SOCKET; i++ ) {

		socket_status = pub_gprs_check_socket_status();
		if (  socket_status == SOCK_OPEN ) {
			// Envio un window frame
			registros_trasmitidos = 0;
			FF_rewind();
			pv_trasmitir_dataHeader();
			while ( pv_hay_datos_para_trasmitir() && ( registros_trasmitidos < MAX_RCDS_WINDOW_SIZE ) ) {
				pv_trasmitir_dataRecord();
				registros_trasmitidos++;
			}
			pv_trasmitir_dataTail();
			return(true);
		}

		// Doy el comando de abrir el socket
		pub_gprs_open_socket();

		// Y espero que lo abra
		await_loops = ( 10 * 1000 / 3000 ) + 1;
		// Y espero hasta 30s que abra.
		for ( timeout = 0; timeout < await_loops; timeout++) {
			vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );
			socket_status = pub_gprs_check_socket_status();

			// Si el socket abrio, salgo para trasmitir el frame de init.
			if ( socket_status == SOCK_OPEN ) {
				break;
			}

			// Si el socket dio error, salgo para enviar de nuevo el comando.
			if ( socket_status == SOCK_ERROR ) {
				break;
			}
		}

	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
static void pv_trasmitir_dataHeader( void )
{

	pub_gprs_flush_RX_buffer();
	pub_gprs_flush_TX_buffer();

	// Armo el header en el buffer

	if ( systemVars.modo == MODO_SPX ) {
		xCom_printf_P( fdGPRS, PSTR("GET %s?DLGID=%s&PASSWD=%s&VER=%s\0"), systemVars.serverScript, systemVars.dlgId, systemVars.passwd, SPX_FW_REV);
		if ( systemVars.debug == DEBUG_GPRS ) {
			xprintf_P( PSTR("GPRS: sent>GET %s?DLGID=%s&PASSWD=%s&VER=%s\r\n\0"), systemVars.serverScript, systemVars.dlgId, systemVars.passwd, SPX_FW_REV);
		}

	} else {
		// Modo emulacion SP5K
		xCom_printf_P( fdGPRS, PSTR("GET %s?DLGID=%s&PASSWD=%s&VER=6.0.0\0"), systemVars.serverScript, systemVars.dlgId, systemVars.passwd );
		if ( systemVars.debug == DEBUG_GPRS ) {
			xprintf_P( PSTR("GPRS: sent>GET %s?DLGID=%s&PASSWD=%s&VER=6.0.0\r\n\0"), systemVars.serverScript, systemVars.dlgId, systemVars.passwd );
		}
	}
	// Para darle tiempo a vaciar el buffer y que no se corten los datos que se estan trasmitiendo
	// por sobreescribir el gprs_printBuff.
	vTaskDelay( (portTickType)( 250 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
static void pv_trasmitir_dataTail( void )
{
	// TAIL : No mando el close ya que espero la respuesta del server

	//pub_gprs_flush_RX_buffer();

	// TAIL ( No mando el close ya que espero la respuesta y no quiero que el socket se cierre )
	xCom_printf_P( fdGPRS, PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n\0") );
	vTaskDelay( (portTickType)( 250 / portTICK_RATE_MS ) );
	// DEBUG & LOG
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		xprintf_P( PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n\0") );
	}

}
//------------------------------------------------------------------------------------
static void pv_trasmitir_dataRecord( void )
{


	switch ( systemVars.modo ) {
	case MODO_SPX:
		pv_tx_dataRecord_modo_SPX();
		break;
	case MODO_SP5K:
		pv_tx_dataRecord_modo_SP5K();
		break;
	default:
		pv_tx_dataRecord_modo_SP5K();
		break;
	}

	vTaskDelay( (portTickType)( 250 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
static bool pv_procese_respuesta_server(void)
{
	// Me quedo hasta 10s esperando la respuesta del server al paquete de datos.
	// Salgo por timeout, socket cerrado, error del server o respuesta correcta
	// Si la respuesta es correcta, ejecuto la misma

uint8_t timeout;
bool exit_flag = false;

	for ( timeout = 0; timeout < 10; timeout++) {

		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );	// Espero 1s

		if ( pub_gprs_check_socket_status() != SOCK_OPEN ) {		// El socket se cerro
			exit_flag = false;
			goto EXIT;
		}

		if ( pub_gprs_check_response ("</h1>\0")) {	// Recibi una respuesta del server.
			// Log.

			if ( systemVars.debug == DEBUG_GPRS ) {
				pub_gprs_print_RX_Buffer();
			} else {
				pub_gprs_print_RX_response();
			}

			if ( pub_gprs_check_response ("ERROR\0")) {
				// ERROR del server: salgo inmediatamente
				exit_flag = false;
				goto EXIT;
			}

			if ( pub_gprs_check_response ("RESET\0")) {
				// El sever mando la orden de resetearse inmediatamente
				pv_process_response_RESET();
			}

			if ( pub_gprs_check_response ("OUTS\0")) {
				// El sever mando actualizacion de las salidas
				pv_process_response_OUTS();
			}

			if ( pub_gprs_check_response ("RX_OK\0")) {
				// Datos procesados por el server.
				pv_process_response_OK();
				exit_flag = true;
				goto EXIT;
			}
		}
	}

// Exit:
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_process_response_RESET(void)
{
	// El server me pide que me resetee de modo de mandar un nuevo init y reconfigurarme

	xprintf_P( PSTR("GPRS: Config RESET...\r\n\0"));

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	// RESET
	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */

}
//------------------------------------------------------------------------------------
static uint8_t pv_process_response_OK(void)
{
	// Retorno la cantidad de registros procesados ( y borrados )
	// Recibi un OK del server y el ultimo ID de registro a borrar.
	// Los borro de a uno.

FAT_t l_fat;
uint8_t recds_borrados = 0;

	// Borro los registros.
	while ( pv_check_more_Rcds4Del() ) {

		FF_deleteRcd();
		recds_borrados++;
		FAT_read(&l_fat);

		if ( systemVars.debug == DEBUG_GPRS ) {
			xprintf_P( PSTR("GPRS: mem wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );
		}

	}

	return(recds_borrados);
}
//------------------------------------------------------------------------------------
static void pv_process_response_OUTS(void)
{
	// Recibi algo del estilo >RX_OK:285:OUTS=1,0.
	// Extraigo el valor de las salidas y las seteo.
	// Es valido solo si las salidas estan configuradas para modo NORMAL.

char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p1,*p2;
char *p;
uint8_t out_A,out_B;

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "OUTS");
	if ( p == NULL ) {
		return;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//OUTS

	p1 = strsep(&stringp,delim);	// out0
	p2 = strsep(&stringp,delim); 	// out1

	out_A = atoi(p1);
	pub_output_set_outputs('A', out_A);
	//( out_A == 0 )? ( systemVars.outputs.out_A = 0 ) : ( systemVars.outputs.out_A = 1);
	out_B = atoi(p2);
	pub_output_set_outputs('B', out_B);
	//( out_B == 0 )? ( systemVars.outputs.out_B = 0 ) : ( systemVars.outputs.out_B = 1);

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: processOUTS %d %d\r\n\0"), out_A, out_B);
	}

//	if ( ( systemVars.outputs.out_A != out_A ) || ( systemVars.outputs.out_B != out_B )) {
		// Mando una señal a la tarea de OUTPUTS para que procese ensegida el cambio.
		//f_send_signal = true;
//	}

}
//------------------------------------------------------------------------------------
static bool pv_check_more_Rcds4Del ( void )
{
	// Devuelve si aun quedan registros para borrar del FS

FAT_t l_fat;

	FAT_read(&l_fat);

	if ( l_fat.rcds4del > 0 ) {
		return(true);
	} else {
		return(false);
	}

}
//------------------------------------------------------------------------------------
static void pv_tx_dataRecord_modo_SP5K(void)
{

uint8_t channel;
st_data_frame_t pv_data_frame;
FAT_t l_fat;

	// Paso1: Leo un registro de memoria
	FF_readRcd( &pv_data_frame, sizeof(st_data_frame_t));
	FAT_read(&l_fat);

	// Paso2: Armo el frame
	// Siempre trasmito los datos aunque vengan papasfritas.
	//pub_gprs_flush_RX_buffer();

	// Indice de la linea,Fecha y hora
	xCom_printf_P( fdGPRS,PSTR("&CTL=%d&LINE=%04d%02d%02d,%02d%02d%02d\0"), l_fat.rdPTR,pv_data_frame.rtc.year,pv_data_frame.rtc.month,pv_data_frame.rtc.day,pv_data_frame.rtc.hour,pv_data_frame.rtc.min, pv_data_frame.rtc.sec );
	// DEBUG & LOG
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: sent> CTL=%d&LINE=%04d%02d%02d,%02d%02d%02d\0"), l_fat.rdPTR,pv_data_frame.rtc.year,pv_data_frame.rtc.month,pv_data_frame.rtc.day,pv_data_frame.rtc.hour,pv_data_frame.rtc.min, pv_data_frame.rtc.sec );
	}

	// Valores analogicos
	for ( channel = 0; channel < 3; channel++) {
		xCom_printf_P( fdGPRS, PSTR(",%s=%.02f\0"),systemVars.a_ch_name[channel],pv_data_frame.analog_in.ainputs[channel] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR(",%s=%.02f\0"),systemVars.a_ch_name[channel],pv_data_frame.analog_in.ainputs[channel] );
		}
	}

	// Valores digitales.
	// No trasmito en modo SP5K

	// Contadores
	for ( channel = 0; channel < NRO_COUNTER_CHANNELS; channel++) {
		xCom_printf_P( fdGPRS, PSTR(",%s=%.2f\0"),systemVars.c_ch_name[channel],pv_data_frame.counters_in.counters[channel] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR(",%s=%.2f\0"),systemVars.c_ch_name[channel],pv_data_frame.counters_in.counters[channel] );
		}
	}

	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xCom_printf_P( fdGPRS, PSTR(",DIST=%d\0"), pv_data_frame.range );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P(PSTR(",DIST=%d\0"), pv_data_frame.range );
		}
	}

	// Bateria
	xCom_printf_P( fdGPRS, PSTR(",bt=%.2f\0"),pv_data_frame.battery );
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		xprintf_P( PSTR(",bt=%.2f\r\n\0"),pv_data_frame.battery );
	}

}
//------------------------------------------------------------------------------------
static void pv_tx_dataRecord_modo_SPX(void)
{

uint8_t channel;
st_data_frame_t pv_data_frame;
FAT_t l_fat;

	// Paso1: Leo un registro de memoria
	FF_readRcd( &pv_data_frame, sizeof(st_data_frame_t));
	FAT_read(&l_fat);

	// Paso2: Armo el frame
	// Siempre trasmito los datos aunque vengan papasfritas.
	//pub_gprs_flush_RX_buffer();

	// Indice de la linea,Fecha y hora
	xCom_printf_P( fdGPRS,PSTR("&CTL=%d&LINE=%04d%02d%02d,%02d%02d%02d\0"), l_fat.rdPTR,pv_data_frame.rtc.year,pv_data_frame.rtc.month,pv_data_frame.rtc.day,pv_data_frame.rtc.hour,pv_data_frame.rtc.min, pv_data_frame.rtc.sec );
	// DEBUG & LOG
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: sent> CTL=%d&LINE=%04d%02d%02d,%02d%02d%02d\0"), l_fat.rdPTR,pv_data_frame.rtc.year,pv_data_frame.rtc.month,pv_data_frame.rtc.day,pv_data_frame.rtc.hour,pv_data_frame.rtc.min, pv_data_frame.rtc.sec );
	}

	// Valores analogicos
	// Solo muestro los que tengo configurados.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( ! strcmp ( systemVars.a_ch_name[channel], "X" ) )
			continue;

		xCom_printf_P( fdGPRS, PSTR(",%s=%.02f\0"),systemVars.a_ch_name[channel],pv_data_frame.analog_in.ainputs[channel] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR(",%s=%.02f\0"),systemVars.a_ch_name[channel],pv_data_frame.analog_in.ainputs[channel] );
		}

	}

	// Valores digitales. Lo que mostramos depende de lo que tenemos configurado
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {

		if ( ! strcmp ( systemVars.d_ch_name[channel], "X" ) ) {
			continue;
		}
		xCom_printf_P( fdGPRS, PSTR(",%s=%d\0"),systemVars.d_ch_name[channel],pv_data_frame.digital_in.dinputs[channel] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR(",%s=%d\0"),systemVars.d_ch_name[channel],pv_data_frame.digital_in.dinputs[channel] );
		}
	}

	// Contadores
	for ( channel = 0; channel < NRO_COUNTER_CHANNELS; channel++) {

		if ( ! strcmp ( systemVars.c_ch_name[channel], "X" ) ) {
			continue;
		}
		xCom_printf_P( fdGPRS, PSTR(",%s=%.02f\0"),systemVars.c_ch_name[channel],pv_data_frame.counters_in.counters[channel] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR(",%s=%.02f\0"),systemVars.c_ch_name[channel],pv_data_frame.counters_in.counters[channel] );
		}
	}

	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xCom_printf_P( fdGPRS, PSTR("DIST=%d,\0"), pv_data_frame.range );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P(PSTR("DIST=%d,\0"), pv_data_frame.range );
		}
	}

	// Bateria
	xCom_printf_P( fdGPRS, PSTR(",bt=%.2f\0"),pv_data_frame.battery );
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		xprintf_P( PSTR(",bt=%.2f\r\n\0"),pv_data_frame.battery );
	}

}
//------------------------------------------------------------------------------------

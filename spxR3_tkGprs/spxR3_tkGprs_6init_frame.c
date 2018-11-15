/*
 * sp5KV5_tkGprs_init_frame.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spxR3_tkGprs.h"

static bool pv_send_init_frame(void);
static t_init_responses pv_process_init_response(void);
static void pv_TX_init_frame(void);
static void pv_process_server_clock(void);
static void pv_reconfigure_params(void);

static uint8_t pv_gprs_config_dlg_id(void);
static uint8_t pv_gprs_config_pwrSave(void);
static uint8_t pv_gprs_config_timerPoll(void);
static uint8_t pv_gprs_config_timerDial(void);
static uint8_t pv_gprs_config_digitalCh(uint8_t channel);
static uint8_t pv_gprs_config_AnalogCh(uint8_t channel);
static uint8_t pv_gprs_config_Outputs(void);
static uint8_t pv_gprs_config_RangeMeter(void);
static uint8_t pv_gprs_config_counterCh(uint8_t channel);

static void pv_TX_init_parameters_modo_SP5K(void);
static void pv_TX_init_parameters_modo_SPX(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_INIT	180

//------------------------------------------------------------------------------------
bool st_gprs_init_frame(void)
{
	// Debo mandar el frame de init al server, esperar la respuesta, analizarla
	// y reconfigurarme.
	// Intento 3 veces antes de darme por vencido.
	// El socket puede estar abierto o cerrado. Lo debo determinar en c/caso y
	// si esta cerrado abrirlo.
	// Mientras espero la respuesta debo monitorear que el socket no se cierre

uint8_t intentos;
bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_INIT_FRAME;

	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_INIT );

	xprintf_P( PSTR("GPRS: iniframe.\r\n\0" ));

	// Intenteo MAX_INIT_TRYES procesar correctamente el INIT
	for ( intentos = 0; intentos < MAX_INIT_TRYES; intentos++ ) {

		if ( pv_send_init_frame() ) {

			switch( pv_process_init_response() ) {
			case INIT_ERROR:
				// Reintento
				break;
			case INIT_SOCK_CLOSE:
				// Reintento
				break;
			case INIT_OK:
				// Aqui es que anduvo todo bien y debo salir para pasar al modo DATA
				if ( systemVars.debug == DEBUG_GPRS ) {
					xprintf_P( PSTR("\r\nGPRS: Init frame OK.\r\n\0" ));
				}
				exit_flag = true;
				goto EXIT;
				break;
			case INIT_NOT_ALLOWED:
				// Respondio bien pero debo salir a apagarme
				exit_flag = false;
				goto EXIT;
				break;
			}

		} else {

			if ( systemVars.debug == DEBUG_GPRS ) {
				xprintf_P( PSTR("GPRS: iniframe retry(%d)\r\n\0"),intentos);
			}

			// Espero 3s antes de reintentar
			vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );
		}
	}

	// Aqui es que no puede enviar/procesar el INIT correctamente
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Init frame FAIL !!.\r\n\0" ));
	}

// Exit
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pv_send_init_frame(void)
{
	// Intento enviar 1 SOLO frame de init.
	// El socket puede estar cerrado por lo que reintento abrirlo hasta 3 veces.
	// Una vez que envie el INIT, salgo.
	// Al entrar, veo que el socket este cerrado.

uint8_t intentos;
bool exit_flag = false;
uint8_t timeout, await_loops;
t_socket_status socket_status;

	for ( intentos = 0; intentos < MAX_TRYES_OPEN_SOCKET; intentos++ ) {

		socket_status = pub_gprs_check_socket_status();

		if (  socket_status == SOCK_OPEN ) {
			pv_TX_init_frame();		// Escribo en el socket el frame de INIT
			return(true);
		}

		// Doy el comando para abrirlo y espero
		pub_gprs_open_socket();

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
static t_init_responses pv_process_init_response(void)
{

	// Espero la respuesta al frame de INIT.
	// Si la recibo la proceso.
	// Salgo por timeout 10s o por socket closed.

uint8_t timeout;
uint8_t exit_code = INIT_ERROR;

	for ( timeout = 0; timeout < 10; timeout++) {

		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );	// Espero 1s

		if ( pub_gprs_check_socket_status() != SOCK_OPEN ) {		// El socket se cerro
			exit_code = INIT_SOCK_CLOSE;
			goto EXIT;
		}

		if ( pub_gprs_check_response("ERROR") ) {	// Recibi un ERROR de respuesta
			pub_gprs_print_RX_Buffer();
			exit_code = INIT_ERROR;
			goto EXIT;
		}


		if ( pub_gprs_check_response("</h1>") ) {	// Respuesta completa del server
			if ( systemVars.debug == DEBUG_GPRS  ) {
				pub_gprs_print_RX_Buffer();
			} else {
				pub_gprs_print_RX_response();
			}

			if ( pub_gprs_check_response("INIT_OK") ) {	// Respuesta correcta
				// Borro la causa del reset
				wdg_resetCause = 0x00;
				pv_reconfigure_params();
				exit_code = INIT_OK;
				goto EXIT;
			}

			if ( pub_gprs_check_response("NOT_ALLOWED") ) {	// Datalogger esta usando un script incorrecto
				xprintf_P( PSTR("GPRS: SCRIPT ERROR !!.\r\n\0" ));
				exit_code = INIT_NOT_ALLOWED;
				goto EXIT;
			}
		}

	}

// Exit:
EXIT:

	return(exit_code);

}
//------------------------------------------------------------------------------------
static void pv_TX_init_frame(void)
{
	// SP5KV5_3CH
	// Send Init Frame
	// GET /cgi-bin/sp5K/sp5K.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&ALARM&PWRM=CONT&TPOLL=23&TDIAL=234&PWRS=1,1230,2045&A0=pZ,1,20,3,10&D0=qE,3.24&CONS=1,1234,927,1,3 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

	// SP5KV5_8CH
	// Send Init Frame
	// GET /cgi-bin/sp5K8CH.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&CSQ=75 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

	if ( systemVars.debug == DEBUG_GPRS  ) {
		xprintf_P( PSTR("GPRS: iniframe: Sent\r\n\0"));
	}

	// Trasmision: 1r.Parte.
	// HEADER:
	// Envio parcial ( no CR )
	pub_gprs_flush_RX_buffer();
	pub_gprs_flush_TX_buffer();

	if ( systemVars.modo == MODO_SPX ) {
		xCom_printf_P( fdGPRS,PSTR("GET %s?DLGID=%s&PASSWD=%s&IMEI=%s&VER=%s&UID=%s&SIMID=%s\0" ), systemVars.serverScript, systemVars.dlgId, systemVars.passwd, &buff_gprs_imei, SPX_FW_REV, NVMEE_readID(), &buff_gprs_ccid );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("GET %s?DLGID=%s&PASSWD=%s&IMEI=%s&VER=%s&UID=%s&SIMID=%s\0" ), systemVars.serverScript, systemVars.dlgId, systemVars.passwd, &buff_gprs_imei, SPX_FW_REV, NVMEE_readID(), &buff_gprs_ccid );
		}
	} else {
		// Modo emulacion SP5K
		xCom_printf_P( fdGPRS,PSTR("GET %s?DLGID=%s&PASSWD=%s&IMEI=%s&VER=6.0.0&UID=%s&SIMID=%s\0" ), systemVars.serverScript, systemVars.dlgId, systemVars.passwd, &buff_gprs_imei, NVMEE_readID(), &buff_gprs_ccid );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("GET %s?DLGID=%s&PASSWD=%s&IMEI=%s&VER=6.0.0&UID=%s&SIMID=%s\0" ), systemVars.serverScript, systemVars.dlgId, systemVars.passwd, &buff_gprs_imei, NVMEE_readID(), &buff_gprs_ccid );
		}
	}

	// BODY ( 1a parte) :
	// timerpoll,timerdial
	xCom_printf_P( fdGPRS, PSTR("&INIT&TPOLL=%d&TDIAL=%d\0"), systemVars.timerPoll,systemVars.timerDial);
	// DEBUG & LOG
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("&INIT&TPOLL=%d&TDIAL=%d\0"), systemVars.timerPoll,systemVars.timerDial );
	}
	// pwrSave
	xCom_printf_P( fdGPRS, PSTR("&PWRS=%d,%02d%02d,%02d%02d\0"),systemVars.pwrSave.modo,systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min,systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min );
	// DEBUG & LOG
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("&PWRS=%d,%02d%02d,%02d%02d\0"),systemVars.pwrSave.modo,systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min,systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min );
	}

	// outputs,csq
	xCom_printf_P( fdGPRS, PSTR("&OUTS=%d,%02d%02d,%02d%02d&CSQ=%d&WRST=0x%02X\0"),systemVars.outputs.modo,systemVars.outputs.consigna_diurna.hour,systemVars.outputs.consigna_diurna.min,systemVars.outputs.consigna_nocturna.hour,systemVars.outputs.consigna_nocturna.min,systemVars.csq, wdg_resetCause );
	// DEBUG & LOG
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("&OUTS=%d,%02d%02d,%02d%02d&CSQ=%d&WRST=0x%02X\0"), systemVars.outputs.modo,systemVars.outputs.consigna_diurna.hour,systemVars.outputs.consigna_diurna.min,systemVars.outputs.consigna_nocturna.hour,systemVars.outputs.consigna_nocturna.min,systemVars.csq, wdg_resetCause );
	}

	switch(systemVars.modo) {
	case MODO_SP5K:
		pv_TX_init_parameters_modo_SP5K();
		break;
	case MODO_SPX:
		pv_TX_init_parameters_modo_SPX();
		break;
	default:
		pv_TX_init_parameters_modo_SP5K();
		break;
	}

	// TAIL ( No mando el close ya que espero la respuesta y no quiero que el socket se cierre )
	xCom_printf_P( fdGPRS, PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n\0") );

	// DEBUG & LOG
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		xprintf_P( PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n\0") );
	}

	vTaskDelay( (portTickType)( 250 / portTICK_RATE_MS ) );


}
//------------------------------------------------------------------------------------
static void pv_reconfigure_params(void)
{

uint8_t saveFlag = 0;

	// Proceso la respuesta del INIT para reconfigurar los parametros
	pv_process_server_clock();

	saveFlag += pv_gprs_config_dlg_id();

	saveFlag += pv_gprs_config_timerPoll();
	saveFlag += pv_gprs_config_timerDial();
	saveFlag += pv_gprs_config_pwrSave();

	// Canales analogicos.
	saveFlag += pv_gprs_config_AnalogCh(0);
	saveFlag += pv_gprs_config_AnalogCh(1);
	saveFlag += pv_gprs_config_AnalogCh(2);
	saveFlag += pv_gprs_config_AnalogCh(3);
	saveFlag += pv_gprs_config_AnalogCh(4);

	// Canales digitales
	saveFlag += pv_gprs_config_digitalCh(0);
	saveFlag += pv_gprs_config_digitalCh(1);

	// Canales de contadores
	saveFlag += pv_gprs_config_counterCh(0);
	saveFlag += pv_gprs_config_counterCh(1);

	// Outputs/Consignas
	saveFlag += pv_gprs_config_Outputs();

	// RangeMeter
	saveFlag += pv_gprs_config_RangeMeter();

	if ( saveFlag > 0 ) {

		pub_save_params_in_NVMEE();

		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("GPRS: Save params OK\r\n\0"));
		}
	}

}
//------------------------------------------------------------------------------------
static void pv_process_server_clock(void)
{
/* Extraigo el srv clock del string mandado por el server y si el drift con la hora loca
 * es mayor a 5 minutos, ajusto la hora local
 * La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:PWRM=DISC:</h1>
 *
 */

char *p;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char rtcStr[12];
uint8_t i;
char c;
RtcTimeType_t rtc;
int8_t xBytes;

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "CLOCK");
	if ( p  == NULL ) {
		return;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);			// CLOCK

	token = strsep(&stringp,delim);			// rtc
	memset(rtcStr, '\0', sizeof(rtcStr));
	memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora
	for ( i = 0; i<12; i++) {
		c = *token;
		rtcStr[i] = c;
		c = *(++token);
		if ( c == '\0' )
			break;

	}

	RTC_str2rtc(rtcStr, &rtc);	// Convierto el string YYMMDDHHMM a RTC.
	xBytes = RTC_write_dtime(&rtc);		// Grabo el RTC
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:pv_process_server_clock\r\n\0"));

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Update rtc to: %s\r\n\0"), rtcStr );
	}

}
//------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_dlg_id(void)
{
	//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:DLGID=TH001:PWRM=DISC:</h1>

char *p;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "DLGID");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// DLGID
	token = strsep(&stringp,delim);	// TH001

	strncpy(systemVars.dlgId, token,DLGID_LENGTH);

	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig DLGID\r\n\0"));
	}

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_timerPoll(void)
{
//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:PWRM=DISC:</h1>

char *p;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "TPOLL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TPOLL

	token = strsep(&stringp,delim);	// timerPoll

	pub_analog_config_timerpoll(token);

	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig TPOLL\r\n\0"));
	}

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_timerDial(void)
{
	//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:CD=1230:CN=0530</h1>

char *p;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "TDIAL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TDIAL

	token = strsep(&stringp,delim);	// timerDial

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig TDIAL\r\n\0"));
	}

	pub_gprs_config_timerdial(token);
	ret = 1;

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_pwrSave(void)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRS=1,2230,0600:D0=q0,1.00:D1=q1,1.00</h1>
//  Las horas estan en formato HHMM.

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p1,*p2;
uint8_t modo;
char *p;

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "PWRS");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//PWRS

	token = strsep(&stringp,delim);	// modo
	modo = atoi(token);
	p1 = strsep(&stringp,delim);	// startTime
	p2 = strsep(&stringp,delim); 	// endTime

	pub_configPwrSave(modo, p1, p2);
	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig PWRSAVE\r\n\0"));
	}

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_RangeMeter(void)
{
	// ?DIST=1,58

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p1;
uint8_t modo;
uint16_t factor;
char *p;

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "DIST");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//DIST

	token = strsep(&stringp,delim);	// modo
	modo = atoi(token);
	p1 = strsep(&stringp,delim);	// factor
	factor = atoi(p1);

	pub_rangeMeter_config(modo, factor );

	if ( ( ret == 1 ) && ( systemVars.debug == DEBUG_GPRS ) ) {
		xprintf_P( PSTR("GPRS: Reconfig DIST\r\n\0"));
	}

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_AnalogCh(uint8_t channel)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token = NULL;
char *delim = ",=:><";
char *chName,*s_iMin,*s_iMax,*s_mMin,*s_mMax;

	switch (channel) {
	case 0:
		stringp = strstr((const char *)&pv_gprsRxCbuffer.buffer, "A0=");
		break;
	case 1:
		stringp = strstr((const char *)&pv_gprsRxCbuffer.buffer, "A1=");
		break;
	case 2:
		stringp = strstr((const char *)&pv_gprsRxCbuffer.buffer, "A2=");
		break;
	case 3:
		stringp = strstr((const char *)&pv_gprsRxCbuffer.buffer, "A3=");
		break;
	case 4:
		stringp = strstr((const char *)&pv_gprsRxCbuffer.buffer, "A4=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);		//A0

	chName = strsep(&stringp,delim);	//name
	s_iMin = strsep(&stringp,delim);	//iMin
	s_iMax = strsep(&stringp,delim);	//iMax
	s_mMin = strsep(&stringp,delim);	//mMin
	s_mMax = strsep(&stringp,delim);	//mMax

	pub_analog_config_channel( channel, chName,s_iMin,s_iMax,s_mMin,s_mMax );
	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig A%d\r\n\0"), channel);
	}

quit:
	return(ret);
}
//--------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_digitalCh(uint8_t channel)
{

//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=C,q0,1.00:D1=L,q1</h1>
//
// D0=C,q0,1.00:D1=L,q1
//
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName;

	switch (channel) {
	case 0:
		stringp = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "D0=");
		break;
	case 1:
		stringp = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "D1=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		// No viene configuracion de D0 ni de D1.
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado ( solo 32 bytes ) a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp, 31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//D0

	if ( systemVars.modo == MODO_SPX ) {
		chName = strsep(&stringp,delim);	//name
		pub_digital_config_channel( channel, chName );
		ret = 1;
	}

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig D%d\r\n\0"), channel);
	}

quit:

	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_counterCh(uint8_t channel)
{

//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=C,q0,1.00:D1=L,q1</h1>
//
//  C0=q0,1.00:C1=q1,1.45
//
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName, *chMagPP;

	switch (channel) {
	case 0:
		stringp = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "C0=");
		break;
	case 1:
		stringp = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "C1=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		// No viene configuracion de C0 ni de C1.
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado ( solo 32 bytes ) a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp, 31);

	stringp = localStr;
	token = strsep(&stringp,delim);	// C0

	if ( systemVars.modo == MODO_SPX ) {
		chName = strsep(&stringp,delim);	//name
		chMagPP = strsep(&stringp,delim);	//magPP
		pub_counters_config_channel( channel, chName, chMagPP );
		ret = 1;
	}

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig C%d\r\n\0"), channel);
	}

quit:

	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_gprs_config_Outputs(void)
{
	// La linea recibida es del tipo:
	// <h1>INIT_OK:OUTS=modo,param1,param2:</h1>
	// modo=0: OUTS_OFF
	// modo=1: OUTS_NORMAL. param1,param2 indican los valores de las salidas
	// modo=2: OUTS_CONSIGNAS: param1, param2 son las horas de la consigna diurna y la nocturna
	// Las horas estan en formato HHMM.

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p0,*p1,*p2;
t_outputs modo;
uint16_t hhmm1, hhmm2;
char *p;

	p = strstr( (const char *)&pv_gprsRxCbuffer.buffer, "OUTS");
	if ( p == NULL )
		goto quit;

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//OUTS

	p0 = strsep(&stringp,delim);	// modo 0,1,2
	modo = atoi(p0);
	p1 = strsep(&stringp,delim);	// consigna_diurna
	hhmm1 = atoi(p1);
	p2 = strsep(&stringp,delim); 	// consigna_nocturna
	hhmm2 = atoi(p2);

	pub_output_config(modo, hhmm1, hhmm2);
	ret = 1;

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: Reconfig OUTPUTS\r\n\0"));
//		xprintf_P( PSTR("GPRS: Reconfig OUTPUTS (p0=%s,p1=%s,p2=%s)\r\n\0"), p0, p1,p2);
//		xprintf_P( PSTR("GPRS: Reconfig OUTPUTS (modo=%d,hhmm1=%d,hhmm2=%d\r\n\0"), modo,hhmm1,hhmm2);
	}

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
static void pv_TX_init_parameters_modo_SP5K(void)
{

	uint8_t i;

	// BODY ( 2a parte) : Configuracion de canales
	// Configuracion de canales analogicos
	for ( i = 0; i < 3; i++) {
		xCom_printf_P( fdGPRS,PSTR("&A%d=%s,%d,%d,%.02f,%.02f\0"), i,systemVars.a_ch_name[i],systemVars.imin[i], systemVars.imax[i], systemVars.mmin[i], systemVars.mmax[i]);
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("&A%d=%s,%d,%d,%.02f,%.02f\0"), i,systemVars.a_ch_name[i],systemVars.imin[i], systemVars.imax[i], systemVars.mmin[i], systemVars.mmax[i]);
		}
	}

	// Configuracion de canales digitales
	// En modo SP5K no se trasmiten.

	// Contadores
	for ( i = 0; i < NRO_COUNTER_CHANNELS; i++) {
		xCom_printf_P( fdGPRS,PSTR("&D0=%s,%.02f\0"),systemVars.c_ch_name[i], systemVars.c_ch_magpp[i] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("&D0=%s,%.02f\0"),systemVars.c_ch_name[i], systemVars.c_ch_magpp[i] );
		}
	}

	// Configuracion del rangeMeter
	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xCom_printf_P( fdGPRS,PSTR("&DIST=1\0"));
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf( PSTR("&DIST=1\0"));
		}

	} else {
		xCom_printf_P( fdGPRS,PSTR("&DIST=0\0"));
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf( PSTR("&DIST=0\0"));
		}
	}

}
//--------------------------------------------------------------------------------------
static void pv_TX_init_parameters_modo_SPX(void)
{

uint8_t i;

	// BODY ( 2a parte) : Configuracion de canales
	// Configuracion de canales analogicos
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++) {
		// No trasmito los canales que estan con X ( apagados )
		if (!strcmp_P( systemVars.a_ch_name[i], PSTR("X"))) {
			continue;
		}
		xCom_printf_P( fdGPRS,PSTR("&A%d=%s,%d,%d,%.02f,%.02f\0"), i,systemVars.a_ch_name[i],systemVars.imin[i], systemVars.imax[i], systemVars.mmin[i], systemVars.mmax[i]);
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("&A%d=%s,%d,%d,%.02f,%.02f\0"), i,systemVars.a_ch_name[i],systemVars.imin[i], systemVars.imax[i], systemVars.mmin[i], systemVars.mmax[i]);
		}
	}

	// Configuracion de canales digitales
	for (i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		// No trasmito los canales que estan con X ( apagados )
		if (!strcmp_P( systemVars.d_ch_name[i], PSTR("X"))) {
			continue;
		}
		xCom_printf_P( fdGPRS,PSTR("&D%d=%s\0"),i, systemVars.d_ch_name[i] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("&D%d=%s\0"),i, systemVars.d_ch_name[i] );
		}
	}

	// Configuracion de canales contadores
	for (i = 0; i < NRO_COUNTER_CHANNELS; i++) {
		// No trasmito los canales que estan con X ( apagados )
		if (!strcmp_P( systemVars.c_ch_name[i], PSTR("X"))) {
			continue;
		}
		xCom_printf_P( fdGPRS,PSTR("&C%d=%s,%.02f\0"),i, systemVars.c_ch_name[i], systemVars.c_ch_magpp[i] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf_P( PSTR("&C%d=%s,%.02f\0"),i, systemVars.c_ch_name[i], systemVars.c_ch_magpp[i] );
		}
	}


	// Configuracion del rangeMeter
	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xCom_printf_P( fdGPRS,PSTR("&DIST=1\0"));
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf( PSTR("&DIST=1\0"));
		}

	} else {
		xCom_printf_P( fdGPRS,PSTR("&DIST=0\0"));
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			xprintf( PSTR("&DIST=0\0"));
		}
	}

}
//--------------------------------------------------------------------------------------


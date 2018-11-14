/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "FRTOS-CMD.h"

#include "spxR3_tkGprs/spxR3_tkGprs.h"
#include "spxR3.h"

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);
static void cmdPokeFunction(void);
static void cmdPeekFunction(void);

static void pv_cmd_INA(uint8_t cmd_mode );
static void pv_cmd_sens12V(void);
static void pv_cmd_rwEE(uint8_t cmd_mode );
static void pv_cmd_rwNVMEE(uint8_t cmd_mode );
static void pv_cmd_rwRTC(uint8_t cmd_mode );
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode );
static void pv_cmd_rdBATTERY(void);
static void pv_cmd_rdDIN(void);
static void pv_cmd_rdMEMORY(void);
static void pv_cmd_wrOUT8814(void);
static void pv_cmd_rwGPRS(uint8_t cmd_mode );
static void pv_cmd_pulse(void);
static void pv_cmd_range(void);
static void pv_cmd_rwACH(uint8_t cmd_mode );
static void pv_cmd_rangeMeter_config( char *modo, char *factor);
static void pv_cmd_read_fuses(void);
static void pv_cmd_config_outputs( char *param0, char *param1, char *param2 );

#define WR_CMD 0
#define RD_CMD 1

#define WDG_CMD_TIMEOUT	30

static usuario_t tipo_usuario;
RtcTimeType_t rtc;

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );
	FRTOS_CMD_register( "status\0", cmdStatusFunction );
	FRTOS_CMD_register( "config\0", cmdConfigFunction );
	FRTOS_CMD_register( "kill\0", cmdKillFunction );
	FRTOS_CMD_register( "poke\0", cmdPokeFunction );
	FRTOS_CMD_register( "peek\0", cmdPeekFunction );

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdTERM,ioctl_SET_TIMEOUT, &ticks );

	tipo_usuario = USER_TECNICO;

	xprintf_P( PSTR("starting tkCmd..\r\n\0") );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

		// Con la terminal desconectada paso c/5s plt 30s es suficiente.
		pub_ctl_watchdog_kick(WDG_CMD, WDG_CMD_TIMEOUT);

		// Si no tengo terminal conectada, duermo 5s lo que me permite entrar en tickless.
//		if ( IO_read_TERMCTL_PIN() == 0 ) {
//			vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );

//		} else {

			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			//while ( CMD_read( (char *)&c, 1 ) == 1 ) {
			while ( frtos_read( fdTERM, (char *)&c, 1 ) == 1 ) {
				FRTOS_CMD_process(c);
			}
//		}
	}
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

uint8_t channel;
FAT_t l_fat;

	xprintf_P( PSTR("\r\nSpymovil %s %s %s %s \r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
#ifdef PROTO_SPX
	#ifdef APP_SPX_SPYMOVIL
		xprintf_P( PSTR("Compilacion: SPX/SPY\r\n\0") );
	#endif
	#ifdef APP_SPX_LATAHONA
		xprintf_P( PSTR("Compilacion: SPX/TAHONA\r\n\0") );
	#endif
#endif

#ifdef PROTO_SP5K
	#ifdef APP_SP5K_SPYMOVIL
		xprintf_P( PSTR("Compilacion: SP5K/SPY\r\n\0") );
	#endif
	#ifdef APP_SP5K_OSE
		xprintf_P( PSTR("Compilacion: SP5K/OSE\r\n\0") );
	#endif
#endif

	xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );

	// SIGNATURE ID
	xprintf_P( PSTR("uID=%s\r\n\0"), NVMEE_readID() );

	// Protocolo
	switch(systemVars.modo) {
	case MODO_SPX:
		xprintf_P( PSTR("Protocolo SPX.\r\n\0" ));
		break;
	case MODO_SP5K:
		xprintf_P( PSTR("Protocolo SP5K R6.0.0\r\n\0" ));
		break;
	default:
		xprintf_P( PSTR("ERROR !! Protocolo desconocido.\r\n\0" ));
		break;
	}

	// Last reset cause
	xprintf_P( PSTR("WRST=0x%02X\r\n\0") ,wdg_resetCause );

	// Fecha y Hora
	pv_cmd_rwRTC( RD_CMD );

	// DlgId
	xprintf_P( PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );

	// Memoria
	FAT_read(&l_fat);
	xprintf_P( PSTR("memory: wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );

	// SERVER
	xprintf_P( PSTR(">Server:\r\n\0"));
	xprintf_P( PSTR("  apn: %s\r\n\0"), systemVars.apn );
	xprintf_P( PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.server_ip_address,systemVars.server_tcp_port );
	xprintf_P( PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	xprintf_P( PSTR("  passwd: %s\r\n\0"), systemVars.passwd );

	// MODEM
	xprintf_P( PSTR(">Modem:\r\n\0"));
	xprintf_P( PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	xprintf_P( PSTR("  ip address: %s\r\n\0"), systemVars.dlg_ip_address);

	// GPRS STATE
	switch (GPRS_stateVars.state) {
	case G_ESPERA_APAGADO:
		xprintf_P( PSTR("  state: await_off\r\n"));
		break;
	case G_PRENDER:
		xprintf_P( PSTR("  state: prendiendo\r\n"));
		break;
	case G_CONFIGURAR:
		xprintf_P( PSTR("  state: configurando\r\n"));
		break;
	case G_MON_SQE:
		xprintf_P( PSTR("  state: mon_sqe\r\n"));
		break;
	case G_GET_IP:
		xprintf_P( PSTR("  state: ip\r\n"));
		break;
	case G_INIT_FRAME:
		xprintf_P( PSTR("  state: init frame\r\n"));
		break;
	case G_DATA:
		xprintf_P( PSTR("  state: data\r\n"));
		break;
	default:
		xprintf_P( PSTR("  state: ERROR\r\n"));
		break;
	}

	// CONFIG
	xprintf_P( PSTR(">Config:\r\n\0"));

	switch(systemVars.debug) {
	case DEBUG_NONE:
		xprintf_P( PSTR("  debug: none\r\n\0") );
		break;
	case DEBUG_GPRS:
		xprintf_P( PSTR("  debug: gprs\r\n\0") );
		break;
	case DEBUG_RANGEMETER:
		xprintf_P( PSTR("  debug: range\r\n\0") );
		break;
	case DEBUG_COUNTER:
		xprintf_P( PSTR("  debug: counters\r\n\0") );
		break;
	default:
		xprintf_P( PSTR("  debug: ???\r\n\0") );
		break;
	}

	xprintf_P( PSTR("  timerDial: [%lu s]/%li\r\n\0"),systemVars.timerDial, pub_gprs_readTimeToNextDial() );
	xprintf_P( PSTR("  timerPoll: [%d s]/%d\r\n\0"),systemVars.timerPoll, pub_ctl_readTimeToNextPoll() );
	xprintf_P( PSTR("  timerPwrSensor: [%d s]\r\n\0"),systemVars.pwr_settle_time );

	// PULSE WIDTH
	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xprintf_P( PSTR("  rangeMeter: on,%d\r\n"), systemVars.rangeMeter_factor);
	} else {
		xprintf_P( PSTR("  rangeMeter: off\r\n"));
	}

	// PWR SAVE:
	if ( systemVars.pwrSave.modo ==  modoPWRSAVE_OFF ) {
		xprintf_P(  PSTR("  pwrsave: off\r\n\0"));
	} else if ( systemVars.pwrSave.modo ==  modoPWRSAVE_ON ) {
		xprintf_P(  PSTR("  pwrsave: on, start[%02d:%02d], end[%02d:%02d]\r\n\0"), systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min);
	}

	// OUTPUTS:
	switch( systemVars.outputs.modo ) {
	case OUT_OFF:
		xprintf_P( PSTR("  outputs: OFF\r\n"));
		break;
	case OUT_CONSIGNA:
		xprintf_P( PSTR("  outputs: CONSIGNA(c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
		break;
	case OUT_NORMAL:
		xprintf_P( PSTR("  outputs: NORMAL (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	default:
		xprintf_P( PSTR("  outputs: ERROR(%d) (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.modo, systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	}

	// Configuracion de canales analogicos
	if ( systemVars.modo == MODO_SPX ) {

		for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
			xprintf_P( PSTR("  a%d( ) [%d-%d mA/ %.02f,%.02f | %04d | %s]\r\n\0"),channel, systemVars.imin[channel],systemVars.imax[channel],systemVars.mmin[channel],systemVars.mmax[channel], systemVars.coef_calibracion[channel], systemVars.a_ch_name[channel] );
		}

		for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
			xprintf_P( PSTR("  d%d [ %s ]\r\n\0"),channel, systemVars.d_ch_name[channel] );
		}

		for ( channel = 0; channel < NRO_COUNTER_CHANNELS; channel++) {
			xprintf_P( PSTR("  c%d [ %s | %.02f ]\r\n\0"),channel, systemVars.c_ch_name[channel],systemVars.c_ch_magpp[channel] );
		}

	} else {
		// Modo SP5K
		// Canales analogicos
		for ( channel = 0; channel < 3; channel++) {
			xprintf_P( PSTR("  a%d( ) [%d-%d mA/ %.02f,%.02f | %04d | %s]\r\n\0"),channel, systemVars.imin[channel],systemVars.imax[channel],systemVars.mmin[channel],systemVars.mmax[channel], systemVars.coef_calibracion[channel], systemVars.a_ch_name[channel] );
		}

		// Canales digitales

		// Contadores
		for ( channel = 0; channel < NRO_COUNTER_CHANNELS; channel++) {
			xprintf_P( PSTR("  c%d [ %s | %.02f ]\r\n\0"),channel, systemVars.c_ch_name[channel],systemVars.c_ch_magpp[channel] );
		}
	}

	// Valores actuales:
	pub_data_print_frame( false );
}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {

		// Nadie debe usar la memoria !!!
		pub_ctl_watchdog_kick(WDG_CMD, 0xFFFF);

		if (!strcmp_P( strupr(argv[2]), PSTR("SOFT\0"))) {
			FF_format(false );
		} else if (!strcmp_P( strupr(argv[2]), PSTR("HARD\0"))) {
			FF_format(true);
		} else {
			xprintf_P( PSTR("ERROR\r\nUSO: reset memory {hard|soft}\r\n\0"));
			return;
		}
	}

	cmdClearScreen();

	while(1)
		;

	//CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// RTC
	// write rtc YYMMDDhhmm
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(WR_CMD);
		return;
	}

	// EE
	// write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwEE(WR_CMD);
		return;
	}

	// RTC SRAM
	// write rtcram pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0"))  && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwRTC_SRAM(WR_CMD);
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_INA(WR_CMD);
		return;
	}

	// ANALOG
	// write analog {ina_id} conf128
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwACH(WR_CMD);
		return;
	}

	// SENS12V
	// write sens12V {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("SENS12V\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_sens12V();
		return;
	}

	// NVMEE
	// write nvmee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwNVMEE(WR_CMD);
		return;
	}

	// CLRD
	// write clrd {0|1}
	if (!strcmp_P( strupr(argv[1]), PSTR("CLRD\0")) && ( tipo_usuario == USER_TECNICO) ) {
		if ( atoi( argv[2]) == 0 ) { IO_clr_CLRD(); }
		if ( atoi( argv[2]) == 1 ) { IO_set_CLRD(); }
		return;
	}

	// OUT 8814
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	//       out pulse (A/B) (+/-) (ms)
	//       out power {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_wrOUT8814();
		return;
	}

	// GPRS
	// write gprs pwr|sw|rts {on|off}
	// write gprs cmd {atcmd}
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwGPRS(WR_CMD);
		return;
	}

	// PULSE
	// write pulse on|off
	if (!strcmp_P( strupr(argv[1]), PSTR("PULSE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_pulse();
		return;
	}

	// CONSIGNA
	// write consigna {diurna|nocturna}
	if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA\0")) ) {

		if (!strcmp_P( strupr(argv[2]), PSTR("DIURNA\0")) ) {
			pub_output_set_consigna_diurna();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("NOCTURNA\0")) ) {
			pub_output_set_consigna_nocturna();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// OUTPUTS
	// write outputs {x,x}
	// Debo esperar para que se carguen los condensadores
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0")) && ( tipo_usuario == USER_TECNICO) ) {
		if ( systemVars.outputs.modo != OUT_NORMAL ) {
			xprintf_P( PSTR("ERROR\r\nOutput mode no compatible!!\r\n\0"));
			return;
		}
		pub_output_set_outputs( 'A', atoi(argv[2]) );
		pub_output_set_outputs( 'B', atoi(argv[3]) );
		pv_snprintfP_OK();
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

uint16_t raw_val;
float mag_val;

	FRTOS_CMD_makeArgv();

	// FUSES
 	if (!strcmp_P( strupr(argv[1]), PSTR("FUSES\0"))) {
 		pv_cmd_read_fuses();
 		return;
 	}

	// WMK
 	if (!strcmp_P( strupr(argv[1]), PSTR("WMK\0"))) {
 		pub_ctl_print_stack_watermarks();
 		return;
 	}

 	// WDT
 	if (!strcmp_P( strupr(argv[1]), PSTR("WDT\0"))) {
 		pub_ctl_print_wdg_timers();
 		return;
 	}

	// RTC
	// read rtc
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(RD_CMD);
		return;
	}

	// FRAME
	// read frame
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) ) {
		pub_data_read_frame( false );
		pub_data_print_frame( false );
		return;
	}

	// SIGNATURE
	// read id
	if (!strcmp_P( strupr(argv[1]), PSTR("ID\0"))) {
		xprintf_P( PSTR("uID=%s\r\n\0"), NVMEE_readID() );
		return;
	}

	// INA
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_INA(RD_CMD);
		return;
	}

	// NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0"))) {
		pv_cmd_rwNVMEE(RD_CMD);
		return;
	}

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwEE(RD_CMD);
		return;
	}

	// RTC SRAM
	// read rtcram address length
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwRTC_SRAM(RD_CMD);
		return;
	}

	// BATTERY
	// read battery
	if (!strcmp_P( strupr(argv[1]), PSTR("BATTERY\0")) ) {
		pv_cmd_rdBATTERY();
		return;
	}

	// ACH { 0..4}
	// read ach x
	if (!strcmp_P( strupr(argv[1]), PSTR("ACH\0")) && ( tipo_usuario == USER_TECNICO) ) {
		if ( atoi(argv[2]) > 4) {
			pv_snprintfP_ERR();
			return;
		}
		pub_analog_read_channel( atoi(argv[2]),&raw_val, &mag_val );
		xprintf_P( PSTR("CH[%02d] raw=%d,mag=%.02f\r\n\0"),atoi(argv[2]),raw_val, mag_val );
		return;
	}

	// DIN
	// read din {0,1}
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rdDIN();
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rdMEMORY();
		return;
	}

	// GPRS
	// read gprs (rsp,cts,dcd,ri)
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwGPRS(RD_CMD);
		return;
	}

	// DIST
	// read dist
	if (!strcmp_P( strupr(argv[1]), PSTR("DIST\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_range();
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	xprintf_P( PSTR("\x1B[2J\0"));
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{

bool retS = false;

	FRTOS_CMD_makeArgv();

	// USER
	if (!strcmp_P( strupr(argv[1]), PSTR("USER\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("TECNICO\0"))) {
			tipo_usuario = USER_TECNICO;
			pv_snprintfP_OK();
			return;
		}
			if (!strcmp_P( strupr(argv[2]), PSTR("NORMAL\0"))) {
			tipo_usuario = USER_NORMAL;
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
			} else {
				memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
				systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
				retS = true;
			}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// config outputs
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0")) ) {
		pv_cmd_config_outputs( argv[2], argv[3], argv[4] );
		return;
	}

	// rangemeter {on|off} factor
	if (!strcmp_P( strupr(argv[1]), PSTR("RANGEMETER\0"))) {
		pv_cmd_rangeMeter_config( argv[2], argv[3] );
		return;
	}

	// config debug
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("NONE\0"))) {
			systemVars.debug = DEBUG_NONE;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("GPRS\0"))) {
			systemVars.debug = DEBUG_GPRS;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("RANGE\0"))) {
			systemVars.debug = DEBUG_RANGEMETER;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("COUNTER\0"))) {
			systemVars.debug = DEBUG_COUNTER;
			retS = true;
		} else {
			retS = false;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		pub_save_params_in_NVMEE();
		pv_snprintfP_OK();
		return;
	}

	// config analog {0..2} aname imin imax mmin mmax
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) ) {
		retS = pub_analog_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7] );
		retS ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// config digital {0..1} dname
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0")) ) {
		retS = pub_digital_config_channel( atoi(argv[2]), argv[3] );
		retS ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// config counter {0..1} cname magPP
	if (!strcmp_P( strupr(argv[1]), PSTR("COUNTER\0")) ) {
		retS = pub_counters_config_channel( atoi(argv[2]), argv[3], argv[4] );
		retS ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// config timerpoll
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0")) ) {
		pub_analog_config_timerpoll( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config timerdial
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL\0")) ) {
		pub_gprs_config_timerdial( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config sensortime
	if (!strcmp_P( strupr(argv[1]), PSTR("SENSORTIME\0")) ) {
		pub_analog_config_sensortime( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config calibrar
	if (!strcmp_P( strupr(argv[1]), PSTR("CFACTOR\0"))) {
		pub_analog_config_spanfactor( atoi(argv[2]), argv[3] );
		pv_snprintfP_OK();
		return;
	}

	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("SP5K\0"))) {
			pub_load_defaults( MODO_SP5K );
		} else 	if (!strcmp_P( strupr(argv[2]), PSTR("SPX\0"))) {
			pub_load_defaults( MODO_SPX );
		} else {

#ifdef PROTO_SPX
		pub_load_defaults( MODO_SPX );
#endif

#ifdef PROTO_SP5K
		pub_load_defaults( MODO_SP5K );
#endif

		}
		pv_snprintfP_OK();
		return;
	}

	// apn
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// port ( SERVER IP PORT)
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_tcp_port, '\0', sizeof(systemVars.server_tcp_port));
			memcpy(systemVars.server_tcp_port, argv[2], sizeof(systemVars.server_tcp_port));
			systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ip (SERVER IP ADDRESS)
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_ip_address, '\0', sizeof(systemVars.server_ip_address));
			memcpy(systemVars.server_ip_address, argv[2], sizeof(systemVars.server_ip_address));
			systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// script ( SERVER SCRIPT SERVICE )
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// passwd
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PWRSAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRSAVE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR( "ON"))) { pub_configPwrSave ( modoPWRSAVE_ON, argv[3], argv[4] ); }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { pub_configPwrSave ( modoPWRSAVE_OFF, argv[3], argv[4] ); }
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE\0"))) {
		xprintf_P( PSTR("-write\r\n\0"));
		xprintf_P( PSTR("  rtc YYMMDDhhmm\r\n\0"));
		xprintf_P( PSTR("  consigna (diurna|nocturna)\r\n\0"));
		if ( tipo_usuario == USER_TECNICO ) {
			xprintf_P( PSTR("  (ee,nvmee,rtcram {pos} {string}\r\n\0"));
			xprintf_P( PSTR("  ina (id) conf {value}, sens12V {on|off}\r\n\0"));
			xprintf_P( PSTR("  analog {ina_id} conf128 \r\n\0"));
			xprintf_P( PSTR("  clrd {0|1}\r\n\0"));
			xprintf_P( PSTR("  out { (enable|disable),(set|reset),(sleep|awake),(ph01|ph10) } {A/B}\r\n\0"));
			xprintf_P( PSTR("      valve (open|close) (A|B) (ms)\r\n\0"));
			xprintf_P( PSTR("      power {on|off}\r\n\0"));
			xprintf_P( PSTR("  outputs (0|1) (0|1)\r\n\0"));
			xprintf_P( PSTR("  gprs (pwr|sw|cts|dtr) {on|off}\r\n\0"));
			xprintf_P( PSTR("      cmd {atcmd}, redial\r\n\0"));
			xprintf_P( PSTR("  pulse {on|off}\r\n\0"));
		}
		return;
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		xprintf_P( PSTR("-read\r\n\0"));
		xprintf_P( PSTR("  rtc, frame\r\n\0"));
		if ( tipo_usuario == USER_TECNICO ) {
			xprintf_P( PSTR("  id\r\n\0"));
			xprintf_P( PSTR("  ee,nvmee,rtcram {pos} {lenght}\r\n\0"));
			xprintf_P( PSTR("  ina {id} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
			xprintf_P( PSTR("  ach {0..4}, battery\r\n\0"));
			xprintf_P( PSTR("  din\r\n\0"));
			xprintf_P( PSTR("  gprs (rsp,rts,dcd,ri)\r\n\0"));
			xprintf_P( PSTR("  dist\r\n\0"));
			xprintf_P( PSTR("  fuses\r\n\0"));
		}
		return;

	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		xprintf_P( PSTR("-reset\r\n\0"));
		xprintf_P( PSTR("  memory {soft|hard}\r\n\0"));
		return;

	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		xprintf_P( PSTR("-config\r\n\0"));
		xprintf_P( PSTR("  user {normal|tecnico}\r\n\0"));
		if ( systemVars.modo == MODO_SPX ) {
			xprintf_P( PSTR("  analog {0..%d} aname imin imax mmin mmax\r\n\0"),( NRO_ANALOG_CHANNELS - 1 ) );
			xprintf_P( PSTR("  digital {0..%d} dname\r\n\0"), ( NRO_DIGITAL_CHANNELS - 1 ) );
			xprintf_P( PSTR("  counter {0..%d} cname magPP\r\n\0"), ( NRO_COUNTER_CHANNELS - 1 ) );
		}

		if ( systemVars.modo == MODO_SP5K ) {
			xprintf_P( PSTR("  analog {0..2} aname imin imax mmin mmax\r\n\0") );
			xprintf_P( PSTR("  digital {0,1} dname magPP\r\n\0") );
		}

		xprintf_P( PSTR("  cfactor {ch} {coef}\r\n\0"));
		xprintf_P( PSTR("  rangemeter {on|off} factor\r\n\0"));
		xprintf_P( PSTR("  outputs {off}|{normal}|{consigna hhmm_dia hhmm_noche}\r\n\0"));
		xprintf_P( PSTR("  timerpoll, timerdial, dlgid {name}, sensortime\r\n\0"));
		xprintf_P( PSTR("  pwrsave modo [{on|off}] [{hhmm1}, {hhmm2}]\r\n\0"));
		xprintf_P( PSTR("  apn, port, ip, script, passwd\r\n\0"));
		xprintf_P( PSTR("  debug {none,gprs,counter,range}\r\n\0"));
		xprintf_P( PSTR("  default {sp5k | spx}\r\n\0"));
		xprintf_P( PSTR("  save\r\n\0"));
		return;

	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0")) && ( tipo_usuario == USER_TECNICO) ) {
		xprintf_P( PSTR("-kill {data,counter,gprstx,gprsrx,outputs}\r\n\0"));
		return;

	} else {

		// HELP GENERAL
		xprintf_P( PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		xprintf_P( PSTR("Available commands are:\r\n\0"));
		xprintf_P( PSTR("-cls\r\n\0"));
		xprintf_P( PSTR("-help\r\n\0"));
		xprintf_P( PSTR("-status\r\n\0"));
		xprintf_P( PSTR("-reset...\r\n\0"));
		xprintf_P( PSTR("-kill...\r\n\0"));
		xprintf_P( PSTR("-write...\r\n\0"));
		xprintf_P( PSTR("-read...\r\n\0"));
		xprintf_P( PSTR("-config...\r\n\0"));

	}

	xprintf_P( PSTR("\r\n\0"));

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{

	FRTOS_CMD_makeArgv();

	// KILL DATA
	if (!strcmp_P( strupr(argv[1]), PSTR("DATA\0"))) {
		vTaskSuspend( xHandle_tkData );
		pub_ctl_watchdog_kick(WDG_DAT, 0xFFFF);
		return;
	}

	// KILL COUNTER
	if (!strcmp_P( strupr(argv[1]), PSTR("COUNTER\0"))) {
		vTaskSuspend( xHandle_tkCounter );
		pub_ctl_watchdog_kick(WDG_COUNT, 0xFFFF);
		return;
	}

	// KILL GPRS
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSTX\0"))) {
		vTaskSuspend( xHandle_tkGprsTx );
		pub_ctl_watchdog_kick(WDG_GPRSTX, 0xFFFF);
		// Dejo la flag de modem prendido para poder leer comandos
		GPRS_stateVars.modem_prendido = true;
		return;
	}

	// KILL RX
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSRX\0"))) {
		vTaskSuspend( xHandle_tkGprsRx );
		pub_ctl_watchdog_kick(WDG_GPRSRX, 0xFFFF);
		return;
	}

	// KILL OUTPUTS
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0"))) {
		vTaskSuspend( xHandle_tkOutputs );
		pub_ctl_watchdog_kick(WDG_OUT, 0xFFFF);
		return;
	}

	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------
static void cmdPokeFunction(void)
{
	// Guarda una variable en la memoria
	// Es para usar para programar todo el systemVars desde un programa externo.

	// poke 1 DLG01

	FRTOS_CMD_makeArgv();

	switch ( atoi(argv[1])) {
	case 0:
		memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
		systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
		break;
	case 1:
		memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
		systemVars.apn[APN_LENGTH - 1] = '\0';
		break;
	case 2:
		memcpy(systemVars.server_tcp_port, argv[2], sizeof(systemVars.server_tcp_port));
		systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
		break;
	case 3:
		memcpy(systemVars.server_ip_address, argv[2], sizeof(systemVars.server_ip_address));
		systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
		break;
	case 4:
		memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
		systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
		break;
	case 5:
		memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
		systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
		break;
	case 6:
		//xprintf_P(PSTR("A0=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[0],systemVars.mmin[0],systemVars.mmax[0],systemVars.imin[0],systemVars.imax[0],systemVars.coef_calibracion[0],systemVars.a_ch_modo[0]  );
		break;
	case 7:
		//xprintf_P(PSTR("A1=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[1],systemVars.mmin[1],systemVars.mmax[1],systemVars.imin[1],systemVars.imax[1],systemVars.coef_calibracion[1],systemVars.a_ch_modo[1]  );
		break;
	case 8:
		//xprintf_P(PSTR("A2=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[2],systemVars.mmin[2],systemVars.mmax[2],systemVars.imin[2],systemVars.imax[2],systemVars.coef_calibracion[2],systemVars.a_ch_modo[2]  );
		break;
	case 9:
		//xprintf_P(PSTR("A3=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[3],systemVars.mmin[3],systemVars.mmax[3],systemVars.imin[3],systemVars.imax[3],systemVars.coef_calibracion[3],systemVars.a_ch_modo[3]  );
		break;
	case 10:
		//xprintf_P(PSTR("A4=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[4],systemVars.mmin[4],systemVars.mmax[4],systemVars.imin[4],systemVars.imax[4],systemVars.coef_calibracion[4],systemVars.a_ch_modo[4]  );
		break;
	case 11:
		//xprintf_P(PSTR("D0=%s,%0.2f,%c,%c\r\n\0"), systemVars.d_ch_name[0],systemVars.d_ch_magpp[0],systemVars.d_ch_modo[0],systemVars.d_ch_type[0] );
		break;
	case 12:
		//xprintf_P(PSTR("D1=%s,%0.2f,%c,%c\r\n\0"), systemVars.d_ch_name[1],systemVars.d_ch_magpp[1],systemVars.d_ch_modo[1],systemVars.d_ch_type[1] );
		break;
	case 13:
		//xprintf_P(PSTR("D2=%s,%0.2f,%c,%c\r\n\0"), systemVars.d_ch_name[1],systemVars.d_ch_magpp[1],systemVars.d_ch_modo[1],systemVars.d_ch_type[1] );
		break;
	case 14:
		//xprintf_P(PSTR("D3=%s,%0.2f,%c\r\n\0"), systemVars.d_ch_name[1],systemVars.d_ch_magpp[1],systemVars.d_ch_modo[1] );
		break;
	case 15:
		//pub_analog_config_sensortime( argv[2] );
		break;
	case 16:
		//pub_analog_config_timerpoll( argv[2] );
		break;
	case 17:
		//pub_gprs_config_timerdial( argv[2] );
		break;
	case 18:
		//xprintf_P(PSTR("DEBUG=%d\r\n\0"), systemVars.debug );
		break;
	case 19:
		systemVars.rangeMeter_enabled = atoi(argv[2]);
		break;
	case 20:
		//systemVars.xbee = atoi(argv[2]);
		break;
	case 21:
		//xprintf_P(PSTR("PWRS=%d,%d,%d,%d,%d\r\n\0"), systemVars.pwrSave.modo, systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min );
		break;
	case 22:
		//xprintf_P(PSTR("OUTS=%d,%d,%d,%d,%d,%d,%d\r\n\0"), systemVars.outputs.modo,systemVars.outputs.out_A,systemVars.outputs.out_B, systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
		break;
	default:
		xprintf_P(PSTR("ERR\r\n\0"));
		return;
	}

	xprintf_P(PSTR("OK\r\n\0"));
}
//------------------------------------------------------------------------------------
static void cmdPeekFunction(void)
{

	FRTOS_CMD_makeArgv();

	xprintf_P(PSTR("peek: \0"));

	switch ( atoi(argv[1])) {
	case 0:
		xprintf_P(PSTR("DLGID=%s\r\n\0"), systemVars.dlgId);
		break;
	case 1:
		xprintf_P(PSTR("APN=%s\r\n\0"), systemVars.apn);
		break;
	case 2:
		xprintf_P(PSTR("IPPORT=%s\r\n\0"), systemVars.server_tcp_port);
		break;
	case 3:
		xprintf_P(PSTR("IPADDR=%s\r\n\0"), systemVars.server_ip_address);
		break;
	case 4:
		xprintf_P(PSTR("SCRIPT=%s\r\n\0"), systemVars.serverScript);
		break;
	case 5:
		xprintf_P(PSTR("PASSWD=%s\r\n\0"), systemVars.passwd);
		break;
	case 6:
		//xprintf_P(PSTR("A0=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[0],systemVars.mmin[0],systemVars.mmax[0],systemVars.imin[0],systemVars.imax[0],systemVars.coef_calibracion[0],systemVars.an_ch_modo[0]  );
		break;
	case 7:
		//xprintf_P(PSTR("A1=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[1],systemVars.mmin[1],systemVars.mmax[1],systemVars.imin[1],systemVars.imax[1],systemVars.coef_calibracion[1],systemVars.an_ch_modo[1]  );
		break;
	case 8:
		//xprintf_P(PSTR("A2=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[2],systemVars.mmin[2],systemVars.mmax[2],systemVars.imin[2],systemVars.imax[2],systemVars.coef_calibracion[2],systemVars.an_ch_modo[2]  );
		break;
	case 9:
		//xprintf_P(PSTR("A3=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[3],systemVars.mmin[3],systemVars.mmax[3],systemVars.imin[3],systemVars.imax[3],systemVars.coef_calibracion[3],systemVars.an_ch_modo[3]  );
		break;
	case 10:
		//xprintf_P(PSTR("A4=%s,%0.2f,%0.2f,%d,%d,%d,%c\r\n\0"), systemVars.an_ch_name[4],systemVars.mmin[4],systemVars.mmax[4],systemVars.imin[4],systemVars.imax[4],systemVars.coef_calibracion[4],systemVars.an_ch_modo[4]  );
		break;
	case 11:
		//xprintf_P(PSTR("D0=%s,%0.2f,%c\r\n\0"), systemVars.d_ch_name[0],systemVars.d_ch_magpp[0],systemVars.d_ch_modo[0] );
		break;
	case 12:
		//xprintf_P(PSTR("D1=%s,%0.2f,%c\r\n\0"), systemVars.d_ch_name[1],systemVars.d_ch_magpp[1],systemVars.d_ch_modo[1] );
		break;
	case 13:
		//xprintf_P(PSTR("D2=%s,%0.2f,%c\r\n\0"), systemVars.d_ch_name[1],systemVars.d_ch_magpp[1],systemVars.d_ch_modo[1] );
		break;
	case 14:
		//xprintf_P(PSTR("D3=%s,%0.2f,%c\r\n\0"), systemVars.d_ch_name[1],systemVars.d_ch_magpp[1],systemVars.d_ch_modo[1] );
		break;
	case 15:
		xprintf_P(PSTR("PWRST=%d\r\n\0"), systemVars.pwr_settle_time );
		break;
	case 16:
		xprintf_P(PSTR("TPOLL=%d\r\n\0"), systemVars.timerPoll );
		break;
	case 17:
		xprintf_P(PSTR("TDIAL=%lu\r\n\0"), systemVars.timerDial );
		break;
	case 18:
		xprintf_P(PSTR("DEBUG=%d\r\n\0"), systemVars.debug );
		break;
	case 19:
		xprintf_P(PSTR("DIST=%d\r\n\0"), systemVars.rangeMeter_enabled );
		break;
	case 20:
		//xprintf_P(PSTR("XBEE=%d\r\n\0"), systemVars.xbee );
		break;
	case 21:
		xprintf_P(PSTR("PWRS=%d,%d,%d,%d,%d\r\n\0"), systemVars.pwrSave.modo, systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min );
		break;
	case 22:
		xprintf_P(PSTR("OUTS=%d,%d,%d,%d,%d,%d,%d\r\n\0"), systemVars.outputs.modo,systemVars.outputs.out_A,systemVars.outputs.out_B, systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
		break;
	default:
		xprintf_P(PSTR("ERR\r\n\0"));
		break;
	}
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf_P( PSTR("ok\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf_P( PSTR("error\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_cmd_INA(uint8_t cmd_mode )
{

uint16_t val;
uint8_t ina_id;
char data[3];
int8_t xBytes;

	// write ina id conf {value}
	if ( cmd_mode == WR_CMD ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0")) ) {
			ina_id = atoi(argv[2]);
			val = atoi( argv[4]);
			data[0] = ( val & 0xFF00 ) >> 8;
			data[1] = ( val & 0x00FF );
			xBytes = INA_write( ina_id, INA3231_CONF, data, 2 );
			if ( xBytes == -1 )
				xprintf_P(PSTR("ERROR: I2C:INA:pv_cmd_INA\r\n\0"));

			pv_snprintfP_OK();
			return;
		}
	}

	// read ina id {conf|chxshv|chxbusv|mfid|dieid}
	if ( cmd_mode == RD_CMD ) {

		ina_id = atoi(argv[2]);

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0"))) {
			xBytes = INA_read(  ina_id, INA3231_CONF, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH1_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH1_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH2_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH2_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH3_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH3_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("MFID\0"))) {
			xBytes = INA_read(  ina_id, INA3221_MFID, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("DIEID\0"))) {
			xBytes = INA_read(  ina_id, INA3221_DIEID, data, 2 );
		} else {
			pv_snprintfP_ERR();
			return;
		}

		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:INA:pv_cmd_INA\r\n\0"));

		val = ( data[0]<< 8 ) + data	[1];
		xprintf_P( PSTR("INAID=%d\r\n\0"), ina_id);
		xprintf_P( PSTR("VAL=0x%04x\r\n\0"), val);
		pv_snprintfP_OK();
		return;

	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_sens12V(void)
{
	// sens12V on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	xprintf_P( PSTR("cmd ERROR: ( write sens12V on{off} )\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_pulse(void)
{
	// pulse on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_UPULSE_RUN();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_UPULSE_RUN();
		pv_snprintfP_OK();
		return;
	}

	xprintf_P( PSTR("cmd ERROR: ( write pulse on{off} )\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwEE(uint8_t cmd_mode )
{

int xBytes = 0;
uint8_t length = 0;
char buffer[32];
char *p;

	// read ee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		xBytes = EE_read( (uint32_t)(atol(argv[2])), buffer, (uint8_t)(atoi(argv[3]) ) );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:EE:pv_cmd_rwEE\r\n\0"));

		if ( xBytes > 0 ) {
			xprintf_P( PSTR( "%s\r\n\0"),buffer);
		}
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write ee pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		xBytes = EE_write( (uint32_t)(atol(argv[2])), argv[3], length );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:EE:pv_cmd_rwEE\r\n\0"));

		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwNVMEE(uint8_t cmd_mode )
{
	// Hace prueba de lectura y escritura de la memoria internan EE del micro
	// que es la que usamos para guardar la configuracion.

int xBytes = 0;
uint8_t length = 0;
char buffer[32];
char *p;

	// read nvmee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {

		xBytes = NVMEE_read( (uint16_t)(atoi(argv[2])), buffer, (uint8_t)(atoi(argv[3]) ) );
		if ( xBytes > 0 ) {
			xprintf_P( PSTR( "%s\r\n\0"),buffer);
		}
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write nvmee pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		xBytes = NVMEE_write( (uint16_t)(atoi(argv[2])), argv[3], length );
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC(uint8_t cmd_mode )
{

char datetime[24];
RtcTimeType_t rtc;
int8_t xBytes;

	if ( cmd_mode == WR_CMD ) {
		RTC_str2rtc(argv[2], &rtc);				// Convierto el string YYMMDDHHMM a RTC.
		xBytes = RTC_write_dtime(&rtc);		// Grabo el RTC
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC\r\n\0"));

		( xBytes > 0)? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if ( cmd_mode == RD_CMD ) {
		xBytes = RTC_read_dtime(&rtc);
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC\r\n\0"));

		RTC_rtc2str(datetime,&rtc);
		xprintf_P( PSTR("%s\r\n\0"), datetime );
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode )
{
	// Como se usa para leer memoria, la impresion la hacemos en hex
	// La RTCram comienza en RTC79410_SRAM_INIT.

uint8_t rtc_sram_buffer[32];
uint8_t i;
int8_t xBytes;
uint8_t length = 0;
char *p;

	// read rtcram {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset(rtc_sram_buffer, '\0', sizeof(rtc_sram_buffer));
		xBytes = RTC_read( ( RTC79410_SRAM_INIT + (uint8_t)(atoi(argv[2]))), (char *)&rtc_sram_buffer, (uint8_t)(atoi(argv[3])) );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC_SRAM\r\n\0"));

		if ( xBytes > 0 ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			xprintf_P ( PSTR( "\r\n\0 ") );
			for (i=0; i < atoi(argv[3]); i++ ) {
				xprintf_P (PSTR("[0x%02x]"),rtc_sram_buffer[i]);
			}
			xprintf_P ( PSTR( "\r\n\0 ") );
		}
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write rtcram pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		xBytes = RTC_write( ( RTC79410_SRAM_INIT + (uint32_t)(atol(argv[2]))), argv[3], length );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC_SRAM\r\n\0"));

		( xBytes > 0 )? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rdBATTERY(void)
{

float battery;

	pub_analog_read_battery(&battery);
	xprintf_P( PSTR("BATTERY=%.02f\r\n\0"), battery );
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_rdDIN(void)
{
	// Leo todas las entradas digitales(2).

st_digital_inputs_t dframe;
uint8_t i;

	// Leo los canales digitales.
	dframe.dinputs[0] = IO_read_D0();
	dframe.dinputs[1] = IO_read_D1();

	// Armo la respuesta
	for (i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		xprintf_P( PSTR("D%d(%s) = %d\r\n\0"), i, systemVars.d_ch_name[i], dframe.dinputs[i] );
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rdMEMORY(void)
{
	// Leemos la memoria e imprimo los datos.
	// El problema es que si hay muchos datos puede excederse el tiempo de watchdog y
	// resetearse el dlg.
	// Para esto, cada 32 registros pateo el watchdog.

	// No implementada
}
//------------------------------------------------------------------------------------
static void pv_cmd_wrOUT8814(void)
{
	// write out { (enable|disable),(set|reset),(sleep|awake),(ph01|ph10) } {A/B}
	//             power {on|off}
	//             valve (open|close) (A|B) (ms)

	// write out enable (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) ) {
		( OUT_enable_pin( toupper(argv[3][0]), 1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out disable (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("DISABLE\0")) ) {
		( OUT_enable_pin( toupper(argv[3][0]), 0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out set
	if (!strcmp_P( strupr(argv[2]), PSTR("SET\0")) ) {
		( OUT_reset_pin (1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out reset
	if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
		( OUT_reset_pin (0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out sleep
	if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
		( OUT_sleep_pin (1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out awake
	if (!strcmp_P( strupr(argv[2]), PSTR("AWAKE\0")) ) {
		( OUT_sleep_pin (0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out ph01 (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("PH01\0")) ) {
		( OUT_phase_pin( toupper(argv[3][0]), 1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out ph10 (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("PH10\0")) ) {
		( OUT_phase_pin( toupper(argv[3][0]), 0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out power on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("POWER\0")) ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
			OUT_power_on();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
			OUT_power_off();
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	//  write out valve (open|close) (A|B) (ms)
	if (!strcmp_P( strupr(argv[2]), PSTR("VALVE\0")) ) {

		// Proporciono corriente.
		OUT_power_on();
		// Espero 10s que se carguen los condensasores
		vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

		if (!strcmp_P( strupr(argv[3]), PSTR("OPEN\0")) ) {
			( OUT_valve( toupper(argv[4][0]), V_OPEN, atoi(argv[5]) )  > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
			OUT_power_off();
			return;
		}
		if (!strcmp_P( strupr(argv[3]), PSTR("CLOSE\0")) ) {
			( OUT_valve( toupper(argv[4][0]), V_CLOSE, atoi(argv[5]) )  > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
			OUT_power_off();
			return;
		}

		OUT_power_off();
		pv_snprintfP_ERR();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwGPRS(uint8_t cmd_mode )
{

uint8_t pin;
//char *p;

	if ( cmd_mode == WR_CMD ) {

		// write gprs (pwr|sw|rts|dtr) {on|off}

		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("SW\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_SW();
				pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_SW(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("CTS\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// Por ahora cableo DTR a CTS.

		if (!strcmp_P( strupr(argv[2]), PSTR("DTR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write gprs redial
		if (!strcmp_P( strupr(argv[2]), PSTR("REDIAL\0")) ) {
			pub_gprs_redial();
			return;
		}
		// ATCMD
		// // write gprs cmd {atcmd}
		if (!strcmp_P(strupr(argv[2]), PSTR("CMD\0"))) {
			xprintf_P( PSTR("%s\r\0"),argv[3] );

			pub_gprs_flush_RX_buffer();
			xCom_printf_P( fdGPRS,PSTR("%s\r\0"),argv[3] );

			xprintf_P( PSTR("sent->%s\r\n\0"),argv[3] );
			return;
		}

		return;
	}

	if ( cmd_mode == RD_CMD ) {
		// read gprs (rsp,cts,dcd,ri)

			// ATCMD
			// read gprs rsp
			if (!strcmp_P(strupr(argv[2]), PSTR("RSP\0"))) {
				pub_gprs_print_RX_Buffer();
				//p = pub_gprs_rxbuffer_getPtr();
				//xprintf_P( PSTR("rx->%s\r\n\0"),p );
				return;
			}

			// DCD
			if (!strcmp_P( strupr(argv[2]), PSTR("DCD\0")) ) {
				pin = IO_read_DCD();
				xprintf_P( PSTR("DCD=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}

			// RI
			if (!strcmp_P( strupr(argv[2]), PSTR("RI\0")) ) {
				pin = IO_read_RI();
				xprintf_P( PSTR("RI=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}

			// RTS
			if (!strcmp_P( strupr(argv[2]), PSTR("RTS\0")) ) {
				pin = IO_read_RTS();
				xprintf_P( PSTR("RTS=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}


			pv_snprintfP_ERR();
			return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_range(void)
{
int16_t range;

	pub_rangeMeter_ping(&range);
	xprintf_P( PSTR("RANGE=%d\r\n\0"),range);
	pv_snprintfP_OK();
	return;

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwACH(uint8_t cmd_mode )
{

uint16_t val = 0;
uint8_t channel;

	// read aCh {ch}, bat
	if ( cmd_mode == RD_CMD ) {
		// Bateria
		if (!strcmp_P( strupr( (char *)argv[2]), PSTR("BAT\0"))) {
			val = ACH_read_battery();
			xprintf_P( PSTR("BAT=%d\r\n\0"), val);
			pv_snprintfP_OK();
			return;
		}

		// Canales
		channel = atoi(argv[2]);
		if (  channel > 4 ) {
			pv_snprintfP_ERR();
			return;
		} else {
			val = ACH_read_channel( channel );
			xprintf_P( PSTR("CH%0d=%d\r\n\0"), channel, val);
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// write ach {id} conf128
	if ( cmd_mode == WR_CMD ) {
		ACH_config_avg128(atoi(argv[2]));
		pv_snprintfP_OK();
		return;
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_rangeMeter_config( char *s_modo, char *s_factor)
{

	// Configura el rangeMeter.
	// Reacomoda los parametros para invocar a pub_rangeMeter_config.

uint8_t modo;
uint16_t factor;

	if ( !strcmp_P( strupr(s_modo), PSTR("ON\0"))) {
		modo = modoRANGEMETER_ON;
	} else if ( !strcmp_P( strupr(s_modo), PSTR("OFF\0"))) {
		modo = modoRANGEMETER_OFF;
	} else {
		goto quit;
	}

	factor = atoi(s_factor);

	if ( pub_rangeMeter_config (modo, factor) == true ) {
		pv_snprintfP_OK();
	} else {
		pv_snprintfP_ERR();
	}

quit:
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_config_outputs( char *param0, char *param1, char *param2 )
{
	// Convierte los parametros char al modo estandar de la funcion interna
	// Ej: config outputs consigna 1030 1240

t_outputs modo = OUT_OFF;
uint16_t hhmm1 = 0;
uint16_t hhmm2 = 0;


	if ( !strcmp_P( strupr(param0), PSTR("OFF\0")) ) {
		modo = OUT_OFF;
	} else if (!strcmp_P( strupr(param0), PSTR("CONSIGNA\0")) ) {
		modo = OUT_CONSIGNA;
	} else if (!strcmp_P( strupr(param0), PSTR("NORMAL\0")) ) {
		modo = OUT_NORMAL;
	} else {
		pv_snprintfP_ERR();
		return;
	}

	if ( param1 != NULL ) {
		hhmm1 = atoi(param1);
	}
	if ( param2 != NULL ) {
		hhmm2 = atoi(param2);
	}

	pub_output_config( modo, hhmm1, hhmm2 );
	pv_snprintfP_OK();
	return;

}
//------------------------------------------------------------------------------------
static void pv_cmd_read_fuses(void)
{
	// Lee los fuses.

uint8_t fuse0,fuse1,fuse2,fuse4,fuse5;

	fuse0 = nvm_fuses_read(0x00);	// FUSE0
	xprintf_P( PSTR("FUSE0=0x%x\r\n\0"),fuse0);

	fuse1 = nvm_fuses_read(0x01);	// FUSE1
	xprintf_P( PSTR("FUSE1=0x%x\r\n\0"),fuse1);

	fuse2 = nvm_fuses_read(0x02);	// FUSE2
	xprintf_P( PSTR("FUSE2=0x%x\r\n\0"),fuse2);

	fuse4 = nvm_fuses_read(0x04);	// FUSE4
	xprintf_P( PSTR("FUSE4=0x%x\r\n\0"),fuse4);

	fuse5 = nvm_fuses_read(0x05);	// FUSE5
	xprintf_P( PSTR("FUSE5=0x%x\r\n\0"),fuse5);

	if ( (fuse0 != 0xFF) || ( fuse1 != 0xAA) || (fuse2 != 0xFD) || (fuse4 != 0xF5) || ( fuse5 != 0xD4) ) {
		xprintf_P( PSTR("FUSES ERROR !!!.\r\n\0"));
		xprintf_P( PSTR("Los valores deben ser: FUSE0=0xFF,FUSE1=0xAA,FUSE2=0xFD,FUSE4=0xF5,FUSE5=0xD4\r\n\0"));
		xprintf_P( PSTR("Deben reconfigurarse !!.\r\n\0"));
		pv_snprintfP_ERR();
		return;
	}
	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------


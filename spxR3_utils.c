/*
 * xmega01_utils.c
 *
 *  Created on: 1 de nov. de 2016
 *      Author: pablo
 */

#include "spxR3.h"

#define RTC32_ToscBusy()        !( VBAT.STATUS & VBAT_XOSCRDY_bm )
void RTC32_ToscEnable( bool use1khz );

//-----------------------------------------------------------
void initMCU(void)
{
	// Inicializa los pines del micro

	// LEDS:
	IO_config_LED_KA();
	IO_config_LED_COMMS();

	// ANALOG: SENSOR VCC CONTROL
	IO_config_SENS_12V_CTL();

	// TERMINAL CTL PIN
	IO_config_TERMCTL_PIN();

	// INPUTS DIGITAL
	IO_config_PA0();		// D0
	IO_config_PA2();
	IO_config_PB2();
	IO_config_PB7();		// D1
	IO_config_CLRD();

	// OUT8814
	OUT_config();

	// GPRS
	IO_config_GPRS_SW();
	IO_config_GPRS_PWR();
	IO_config_GPRS_RTS();
	IO_config_GPRS_CTS();
	IO_config_GPRS_DCD();
	IO_config_GPRS_RI();
	IO_config_GPRS_RX();
	IO_config_GPRS_TX();

	// BLUETOOTH POWER CTL
//	IO_config_BT_PWR_CTL();

	// INTERFASE DE PULSOS ( sensor de ultrasonido )
	IO_config_UPULSE_RUN();
	IO_config_UPULSE_WIDTH();

	// TICK:
	//IO_config_TICK();

	// PWR_SLEEP
	IO_config_PWR_SLEEP();
//	IO_set_PWR_SLEEP();

	// XBEE
	IO_config_XBEE_PWR();
	IO_config_XBEE_SLEEP();
	IO_config_XBEE_RESET();

}
//-----------------------------------------------------------
void u_configure_systemMainClock(void)
{
/*	Configura el clock principal del sistema
	Inicialmente se arranca en 2Mhz.
	La configuracion del reloj tiene 2 componentes: el clock y el oscilador.
	OSCILADOR:
	Primero debo elejir cual oscilador voy a usar para alimentar los prescalers que me den
	el clock del sistema y esperar a que este este estable.
	CLOCK:
	Elijo cual oscilador ( puedo tener mas de uno prendido ) va a ser la fuente principal
	del closck del sistema.
	Luego configuro el prescaler para derivar los clocks de los perifericos.
	Puedo por ultimo 'lockear' esta configuracion para que no se cambie por error.
	Los registros para configurar el clock son 'protegidos' por lo que los cambio
	utilizando la funcion CCPwrite.

	Para nuestra aplicacion vamos a usar un clock de 32Mhz.
	Como vamos a usar el ADC debemos prestar atencion al clock de perifericos clk_per ya que luego
	el ADC clock derivado del clk_per debe estar entre 100khz y 1.4Mhz ( AVR1300 ).

	Opcionalmente podriamos deshabilitar el oscilador de 2Mhz para ahorrar energia.
*/

#if SYSMAINCLK == 32
	// Habilito el oscilador de 32Mhz
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 32 Mhz ).
	//
#endif

#if SYSMAINCLK == 8
	// Habilito el oscilador de 32Mhz y lo divido por 4
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// Pongo el prescaler A por 4 y el B y C en 0.
	CLKSYS_Prescalers_Config( CLK_PSADIV_4_gc, CLK_PSBCDIV_1_1_gc );

	//
#endif

#if SYSMAINCLK == 2
	// Este es el oscilador por defecto por lo cual no tendria porque configurarlo.
	// Habilito el oscilador de 2Mhz
	OSC.CTRL |= OSC_RC2MEN_bm;
	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC2MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 2Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC2M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 2 Mhz ).
	//
#endif

//#ifdef configUSE_TICKLESS_IDLE
	// Para el modo TICKLESS
	// Configuro el RTC con el osc externo de 32Khz
	// Pongo como fuente el xtal externo de 32768 contando a 32Khz.
	//CLK.RTCCTRL = CLK_RTCSRC_TOSC32_gc | CLK_RTCEN_bm;
	//do {} while ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) );

	// Disable RTC interrupt.
	// RTC.INTCTRL = 0x00;
	//
	// Si uso el RTC32, habilito el oscilador para 1ms.

	RTC32_ToscEnable(true);
//#endif

	// Lockeo la configuracion.
	CCPWrite( &CLK.LOCK, CLK_LOCK_bm );

}
//-----------------------------------------------------------
void RTC32_ToscEnable( bool use1khz )
{
	/* Enable 32 kHz XTAL oscillator, with 1 kHz or 1 Hz output. */
	if (use1khz)
		VBAT.CTRL |= ( VBAT_XOSCEN_bm | VBAT_XOSCSEL_bm );
	else
		VBAT.CTRL |= ( VBAT_XOSCEN_bm );

	RTC32.PER = 10;
	RTC32.CNT = 0;

	/* Wait for oscillator to stabilize before returning. */
//	do { } while ( RTC32_ToscBusy() );
}
//-----------------------------------------------------------
void pub_load_defaults(modo_t modo)
{

	switch ( modo ) {
	case MODO_SP5K:
		systemVars.modo = MODO_SP5K;
		break;
	case MODO_SPX:
		systemVars.modo = MODO_SPX;
		break;
	default:
		systemVars.modo = MODO_SP5K;
		break;
	}

	strncpy_P(systemVars.dlgId, PSTR("SPX001\0"),DLGID_LENGTH);
	systemVars.debug = DEBUG_NONE;

	// PwrSave
	systemVars.pwrSave.modo = modoPWRSAVE_ON;
	systemVars.pwrSave.hora_start.hour = 23;
	systemVars.pwrSave.hora_start.min = 30;
	systemVars.pwrSave.hora_fin.hour = 5;
	systemVars.pwrSave.hora_fin.min = 30;

	pub_analog_load_defaults();
	pub_digital_load_defaults();
	pub_counters_load_defaults();
	pub_gprs_load_defaults(modo);
	pub_output_load_defaults();
	pub_rangeMeter_load_defaults();


}
//-----------------------------------------------------------
void u_configure_RTC32(void)
{
	// El RTC32 lo utilizo para desperarme en el modo tickless.
	// V-bat needs to be reset, and activated
	VBAT.CTRL |= VBAT_ACCEN_bm;
	// Este registro esta protegido de escritura con CCP.
	CCPWrite(&VBAT.CTRL, VBAT_RESET_bm);

	// Pongo el reloj en 1.024Khz.
	VBAT.CTRL |=  VBAT_XOSCSEL_bm | VBAT_XOSCFDEN_bm ;

	// wait for 200us see AVR1321 Application note page 8
	_delay_us(200);

	// Turn on 32.768kHz crystal oscillator
	VBAT.CTRL |= VBAT_XOSCEN_bm;

	// Wait for stable oscillator
	while(!(VBAT.STATUS & VBAT_XOSCRDY_bm));

	// Disable RTC32 module before setting counter values
	RTC32.CTRL = 0;

	// Wait for sync
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );

	// EL RTC corre a 1024 hz y quiero generar un tick de 10ms,
	RTC32.PER = 1024;
	RTC32.CNT = 0;

	// Interrupt: on Overflow
	RTC32.INTCTRL = RTC32_OVFINTLVL_LO_gc;

	// Enable RTC32 module
	RTC32.CTRL = RTC32_ENABLE_bm;

	/* Wait for sync */
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );
}
//-----------------------------------------------------------
uint8_t pub_save_params_in_NVMEE(void)
{
	// Calculo el checksum del systemVars.
	// Considero el systemVars como un array de chars.

uint8_t *p;
uint8_t checksum;
uint16_t data_length;
uint16_t i;

	// Calculo el checksum del systemVars.
	systemVars.checksum = 0;
	data_length = sizeof(systemVars);
	p = (uint8_t *)&systemVars;
	checksum = 0;
	// Recorro todo el systemVars considerando c/byte como un char, hasta
	// llegar al ultimo ( checksum ) que no lo incluyo !!!.
	for ( i = 0; i < (data_length - 1) ; i++) {
		checksum += p[i];
	}
	checksum = ~checksum;
	systemVars.checksum = checksum;

	// Guardo systemVars en la EE
	NVMEE_write_buffer(0x00, &systemVars, sizeof(systemVars));

	return(checksum);

}
//------------------------------------------------------------------------------------
bool u_load_params_from_NVMEE(void)
{
	// Leo el systemVars desde la EE.
	// Calculo el checksum. Si no coincide es que hubo algun
	// error por lo que cargo el default.


uint8_t *p;
uint8_t stored_checksum, checksum;
uint16_t data_length;
uint16_t i;

	// Leo de la EE es systemVars.
	NVMEE_read_buffer(0x00, (char *)&systemVars, sizeof(systemVars));

	// Guardo el checksum que lei.
	stored_checksum = systemVars.checksum;

	// Calculo el checksum del systemVars leido
	systemVars.checksum = 0;
	data_length = sizeof(systemVars);
	p = ( uint8_t *)&systemVars;	// Recorro el systemVars como si fuese un array de int8.
	checksum = 0;
	for ( i = 0; i < ( data_length - 1 ); i++) {
		checksum += p[i];
	}
	checksum = ~checksum;

	if ( stored_checksum != checksum ) {
		return(false);
	}

	return(true);
}
//------------------------------------------------------------------------------------
void u_convert_str_to_time_t ( char *time_str, time_t *time_struct )
{

	// Convierte un string hhmm en una estructura time_type que tiene
	// un campo hora y otro minuto

uint16_t time_num;

	time_num = atol(time_str);
	time_struct->hour = (uint8_t) (time_num / 100);
	time_struct->min = (uint8_t)(time_num % 100);

//	xprintf_P( PSTR("DEBUG: u_convert_str_to_time_t (hh=%d,mm=%d)\r\n\0"), time_struct->hour,time_struct->min );

}
//------------------------------------------------------------------------------------
void u_convert_int_to_time_t ( int int16time, time_t *time_struct )
{

	// Convierte un int16  hhmm en una estructura time_type que tiene
	// un campo hora y otro minuto

	time_struct->hour = (uint8_t) (int16time / 100);
	time_struct->min = (uint8_t)(int16time % 100);

//	xprintf_P( PSTR("DEBUG: u_convert_str_to_time_t (hh=%d,mm=%d)\r\n\0"), time_struct->hour,time_struct->min );

}
//------------------------------------------------------------------------------------
void pub_configPwrSave(uint8_t modoPwrSave, char *s_startTime, char *s_endTime)
{
	// Recibe como parametros el modo ( 0,1) y punteros a string con las horas de inicio y fin del pwrsave
	// expresadas en minutos.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	switch ( modoPwrSave ) {
	case modoPWRSAVE_OFF:
		systemVars.pwrSave.modo = modoPWRSAVE_OFF;
		break;
	case modoPWRSAVE_ON:
		systemVars.pwrSave.modo = modoPWRSAVE_ON;
		break;
	default:
		xprintf_P(PSTR("pub_configPwrSave ERRRO: modo=%d\r\n\0"), modoPwrSave);
		return;
	}

	if ( s_startTime != NULL ) { pub_convert_str_to_time_t( s_startTime, &systemVars.pwrSave.hora_start); }
	if ( s_endTime != NULL ) { pub_convert_str_to_time_t( s_endTime, &systemVars.pwrSave.hora_fin); }

	xSemaphoreGive( sem_SYSVars );

}
//------------------------------------------------------------------------------------
void pub_convert_str_to_time_t ( char *time_str, time_t *time_struct )
{

	// Convierte un string hhmm en una estructura time_type que tiene
	// un campo hora y otro minuto

uint16_t time_num;

	time_num = (uint16_t)atol(time_str);
	time_struct->hour = (uint8_t) (time_num / 100);
//	xprintf_P(PSTR("hour: %d - %d\r\n\0"), time_num, time_struct->hour);
	time_struct->min = (uint8_t)(time_num % 100);
//	xprintf_P(PSTR("min: %d - %d\r\n\0"), time_num, time_struct->min);

}
//------------------------------------------------------------------------------------
void pub_control_string( char *s_name )
{
	// Controlo que el string terminado en \0 tenga solo letras o digitos.
	// Es porque si en un nombre de canal se cuela un caracter extranio, me
	// despelota los logs.
	// Si encuentro un caracter extraño, lo sustituyo por un \0 y salgo

uint8_t max_length = PARAMNAME_LENGTH;
char *p;
uint8_t cChar;

	p = (char *)s_name;
	while (*p && (max_length-- > 0) ) {

		cChar = (uint8_t)*p;
		if (  ! isalnum(cChar) )	{
			*p = '\0';
			return;
		}
		p++;
	}

}
//------------------------------------------------------------------------------------
bool pub_digital_config_channel( uint8_t channel,char *s_param0 )
{

	// {0..1} dname

bool retS = false;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	if ( ( channel >=  0) && ( channel < NRO_DIGITAL_CHANNELS ) ) {

		// NOMBRE
		pub_control_string(s_param0);
		snprintf_P( systemVars.d_ch_name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_param0 );
		retS = true;
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);

}
//------------------------------------------------------------------------------------
void pub_digital_load_defaults(void)
{
	// Realiza la configuracion por defecto de los canales digitales.

	snprintf_P( systemVars.d_ch_name[0], PARAMNAME_LENGTH, PSTR("D0\0") );
	snprintf_P( systemVars.d_ch_name[1], PARAMNAME_LENGTH, PSTR("D1\0") );

}
//------------------------------------------------------------------------------------


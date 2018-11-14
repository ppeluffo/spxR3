/*
 * sp5KFRTOS_rtc.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 * Funciones del RTC DS1340-33 modificadas para usarse con FRTOS.
 *
 *
 */
// --------------------------------------------------------------------------------

#include "../spxR3_libs/l_rtc79410.h"

static char pv_bcd2dec(char num);
static char pv_dec2bcd(char num);

//------------------------------------------------------------------------------------
// Funciones de uso general
//------------------------------------------------------------------------------------
void RTC_start(void)
{

RtcTimeType_t rtc;
uint8_t data;
int8_t xBytes;

	// cuando arranca el RTC de una situacion de pwr_off no battery, esta apagado.
	// Para que comienze a correr debemos poner el bit7 de RTCSEC en 1.
	// Por otro lado, puede estar reseteado con lo que la fecha aparece en 01 01 2000.

	// Leo la hora
	xBytes = RTC_read_dtime( &rtc);
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:RTC_start\r\n\0"));

	// Si esta reseteado la reconfiguro
	if ( ( rtc.year < 2018 ) || ( rtc.year > 2040) ){
		RTC_str2rtc("1801010000", &rtc);
		xBytes = RTC_write_dtime(&rtc);
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:RTC_start\r\n\0"));

	} else {
	// Escribo ST = 1 para asegurarme de haber activado el RTC
		// Habilito el OSCILADOR
		data = 0x80;
		xBytes = RTC_write(0x00, (char *)&data , 1);
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:RTC_start\r\n\0"));
	}
	return;

}
//------------------------------------------------------------------------------------
bool RTC_read_dtime(RtcTimeType_t *rtc)
{
	// Retorna la hora formateada en la estructura RtcTimeType_t
	// No retornamos el valor de EOSC ni los bytes adicionales.

uint8_t data[8];
uint8_t rdBytes = 0;

	rdBytes = RTC_read(0x00, (char *)&data, 7);
	if ( rdBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:RTC_read_dtime\r\n\0"));

	if (rdBytes != 7 ) {
		return ( false );
	}

	// Decodifico los resultados del buffer para ponerlos en la estructura RTC

	rtc->sec = pv_bcd2dec(data[0] & 0x7F);
	rtc->min = pv_bcd2dec(data[1]);
	rtc->hour = pv_bcd2dec(data[2] & 0x3F);
	rtc->day = pv_bcd2dec(data[4] & 0x3F);
	rtc->month = pv_bcd2dec(data[5] & 0x1F);
	rtc->year = pv_bcd2dec(data[6]) + 2000;

	return(true);
}
//------------------------------------------------------------------------------------
bool RTC_write_dtime(RtcTimeType_t *rtc)
{
	// Setea el RTC con la hora pasada en la estructura RtcTimeType
	// El procedimiento es:
	// Pongo ST en 0.
	// Espero que el bit OSCRUN este en 0.
	// Grabo la nueva hora
	// Arranco el reloj poniendo ST en 1.

uint8_t data[8];
uint8_t rdBytes;

	// Pongo ST en 0.
	// Como está en el registro que corresponde a los segundos, pongo todo el
	// byte en 0.
	data[0] = 0x00;
	rdBytes = RTC_write(0x00, (char *)&data, 1 );
	if ( rdBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:RTC_write_dtime\r\n\0"));

	//
	// Espero que el OSCRUN quede en 0.
	while ( ( data[0] & 0x20 ) != 0 ) {
		rdBytes = RTC_read( 0x03, (char *)&data, 1 );
		if ( rdBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:RTC_write_dtime\r\n\0"));

		vTaskDelay( ( TickType_t) 5 );
	}

	// Ahora puedo grabar la nueva hora.
	// Grabo los datos sin habilitar aun el oscilador.
	data[0] = 0x00;								// RTCSEC: ST = 0, Oscillator disabled.
	data[1] = pv_dec2bcd(rtc->min & 0x7F);		// RTCMIN
	data[2] = pv_dec2bcd(rtc->hour & 0x1F);		// RTCHOUR, 12/24 = 0: 24 hours.
	data[3] = 0x09;								// RTCWKDAY:, VBAT enable
	data[4] = pv_dec2bcd(rtc->day);				// RTCDATE
	data[5] = pv_dec2bcd(rtc->month & 0x1F);	// RTCMONTH
	data[6] = pv_dec2bcd(rtc->year - 2000);		// RTCYEAR

	rdBytes = RTC_write(0x00, (char *)&data, 7);
	if ( rdBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:RTC_write_dtime\r\n\0"));

	// Habilito el OSCILADOR
	data[0] = 0x80;
	rdBytes = RTC_write(0x00, (char *)&data, 1 );
	if ( rdBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:RTC_write_dtime\r\n\0"));

	return(true);

}
//------------------------------------------------------------------------------------
void RTC_rtc2str(char *str, RtcTimeType_t *rtc)
{
	// Convierte los datos del RTC a un string con formato DD/MM/YYYY hh:mm:ss

	snprintf( str, 32 ,"%02d/%02d/%04d %02d:%02d:%02d\r\n",rtc->day,rtc->month, rtc->year, rtc->hour,rtc->min, rtc->sec );

}
//------------------------------------------------------------------------------------
bool RTC_str2rtc(char *str, RtcTimeType_t *rtc)
{
	// Convierto los datos de un string con formato YYMMDDhhmm a RTC

char dateTimeStr[11];
char tmp[3];
bool retS;


	/* YYMMDDhhmm */
	if ( str == NULL )
		return(false);

		memcpy(dateTimeStr, str, 10);
		// year
		tmp[0] = dateTimeStr[0]; tmp[1] = dateTimeStr[1];	tmp[2] = '\0';
		rtc->year = 2000 + atoi(tmp);
		// month
		tmp[0] = dateTimeStr[2]; tmp[1] = dateTimeStr[3];	tmp[2] = '\0';
		rtc->month = atoi(tmp);
		// day of month
		tmp[0] = dateTimeStr[4]; tmp[1] = dateTimeStr[5];	tmp[2] = '\0';
		rtc->day = atoi(tmp);
		// hour
		tmp[0] = dateTimeStr[6]; tmp[1] = dateTimeStr[7];	tmp[2] = '\0';
		rtc->hour = atoi(tmp);
		// minute
		tmp[0] = dateTimeStr[8]; tmp[1] = dateTimeStr[9];	tmp[2] = '\0';
		rtc->min = atoi(tmp);

		return(retS);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static char pv_dec2bcd(char num)
{
	// Convert Decimal to Binary Coded Decimal (BCD)
	return ((num/10 * 16) + (num % 10));
}
//------------------------------------------------------------------------------------
static char pv_bcd2dec(char num)
{
	// Convert Binary Coded Decimal (BCD) to Decimal
	return ((num/16 * 10) + (num % 16));
}
//------------------------------------------------------------------------------------

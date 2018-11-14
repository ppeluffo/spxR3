/*
 * l_iopines.c
 *
 *  Created on: 7 de jul. de 2017
 *      Author: pablo
 */

#include "../spxR3_libs/l_iopines.h"

//------------------------------------------------------------------------------------
// ENTRADAS DIGITALES DEL MODEM
//------------------------------------------------------------------------------------
uint8_t IO_read_DCD(void)
{
	return( PORT_GetBitValue(&GPRS_DCD_PORT, GPRS_DCD_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_RI(void)
{
	return( PORT_GetBitValue(&GPRS_RI_PORT, GPRS_RI_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_RTS(void)
{
	return( PORT_GetBitValue(&GPRS_RTS_PORT, GPRS_RTS_BITPOS));
}
//------------------------------------------------------------------------------------
// ENTRADAS DIGITALES DE CONTEO DE PULSOS Y NIVELES
//------------------------------------------------------------------------------------
uint8_t IO_read_PA0(void)
{
	return( PORT_GetBitValue(&PA0_PORT, PA0_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_PB7(void)
{
	return( PORT_GetBitValue(&PB7_PORT, PB7_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_PB2(void)
{
	return( PORT_GetBitValue(&PB2_PORT, PB2_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_PA2(void)
{
	return( PORT_GetBitValue(&PA2_PORT, PA2_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_SLEEP_CTL(void)
{
	return( PORT_GetBitValue(&SLEEP_CTL_PORT, SLEEP_CTL_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_TERMCTL_PIN(void)
{
	return( PORT_GetBitValue(&TERMCTL_PIN_PORT, TERMCTL_PIN_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_BAUD_PIN(void)
{
	return( PORT_GetBitValue(&BAUD_PIN_PORT, BAUD_PIN_BITPOS));
}
//------------------------------------------------------------------------------------
void IO_config_PB2(void)
{
	// ( PB2 ) se utiliza para generar una interrupcion por flanco por lo tanto
	// hay que configurarlo.

	PORT_SetPinAsInput( &PB2_PORT, PB2_BITPOS);
	PB2_PORT.PIN2CTRL = 0x01;	// sense rising edge
	PB2_PORT.INTCTRL = 0x01;	// Dispara la interrupcion 0.
	PB2_PORT.INT0MASK = 0x04;	// Asocio el pin 2 a dicha interrupcion
}
//------------------------------------------------------------------------------------
void IO_config_PA2(void)
{
	// ( PA2 ) se utiliza para generar una interrupcion por flanco por lo tanto
	// hay que configurarlo.

	PORT_SetPinAsInput( &PA2_PORT, PA2_BITPOS);
	PA2_PORT.PIN2CTRL = 0x01;	// sense rising edge
	PA2_PORT.INTCTRL = 0x01;	// Dispara la interrupcion 0 con level 1
	PA2_PORT.INT0MASK = 0x04;	// Asocio el pin 2 a dicha interrupcion


}
//------------------------------------------------------------------------------------
// SENSOR DE ULTRASONIDO. MIDO ANCHO DE PULSO

uint8_t IO_read_UPULSE_WIDTH(void)
{
	return( PORT_GetBitValue(&UPULSE_WIDTH_PORT, UPULSE_WIDTH_BITPOS));
}
//------------------------------------------------------------------------------------

/*
 *  This function configures interrupt 1 to be associated with a set of pins and
 *  sets the desired interrupt level.
 *
 *  port       The port to configure.
 *  intLevel   The desired interrupt level for port interrupt 1.
 *  pinMask    A mask that selects the pins to associate with port interrupt 1.
 */
/*
void PORT_ConfigureInterrupt0( PORT_t * port,PORT_INT0LVL_t intLevel,uint8_t pinMask )
{
	port->INTCTRL = ( port->INTCTRL & ~PORT_INT0LVL_gm ) | intLevel;
	port->INT0MASK = pinMask;
}

//------------------------------------------------------------------------------------
void PORT_ConfigureInterrupt1( PORT_t * port, PORT_INT1LVL_t intLevel,uint8_t pinMask )
{
	port->INTCTRL = ( port->INTCTRL & ~PORT_INT1LVL_gm ) | intLevel;
	port->INT1MASK = pinMask;
}
*/

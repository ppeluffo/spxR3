/*
 * drv_uart_spx.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "drv_uart_spxR3.h"

//----------------------------------------------------------------------------------------
uart_control_t *drv_uart_init( uart_id_t iUART, uint32_t baudrate )
{
	// El puerto del USB es PORTD:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uart_control_t *pUart = NULL;

	switch(iUART) {
	case iUART_GPRS:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_gprs_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_gprs.RXringBuffer, &gprs_rxStorage[0], GPRS_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_gprs.TXringBuffer, &gprs_txStorage[0], GPRS_RXSTORAGE_SIZE );
		// Asigno el identificador
		uart_gprs.uart_id = iUART_GPRS;
		// Devuelvo la direccion de uart_gprs para que la asocie al dispositvo GPRS el frtos.
		pUart = (uart_control_t *)&uart_gprs;
		break;
	case iUART_XBEE:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_xbee_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_xbee.RXringBuffer, &xbee_rxStorage[0], XBEE_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_xbee.TXringBuffer, &xbee_txStorage[0], XBEE_RXSTORAGE_SIZE );
		// Asigno el identificador
		uart_xbee.uart_id = iUART_XBEE;
		// Devuelvo la direccion de uart_gprs para que la asocie al dispositvo GPRS el frtos.
		pUart = (uart_control_t *)&uart_xbee;
		break;
	case iUART_TERM:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_term_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_term.RXringBuffer, &term_rxStorage[0], TERM_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_term.TXringBuffer, &term_txStorage[0], TERM_RXSTORAGE_SIZE );
		// Asigno el identificador
		uart_term.uart_id = iUART_TERM;
		// Devuelvo la direccion de uart_gprs para que la asocie al dispositvo GPRS el frtos.
		pUart = (uart_control_t *)&uart_term;
		break;
	}

	return(pUart);
}
//----------------------------------------------------------------------------------------
void drv_uart_interruptOn(uart_id_t iUART)
{
	// Habilito la interrupcion TX del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

uint8_t tempCTRLA;

	switch(iUART) {
	case iUART_GPRS:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTE0.CTRLA = tempCTRLA;
		break;
	case iUART_XBEE:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTC0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTC0.CTRLA = tempCTRLA;
		break;
	case iUART_TERM:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTF0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTF0.CTRLA = tempCTRLA;
		break;
	}

}
//----------------------------------------------------------------------------------------
void drv_uart_interruptOff(uart_id_t iUART)
{

uint8_t tempCTRLA;

	switch(iUART) {
	case iUART_GPRS:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTE0.CTRLA = tempCTRLA;
		break;
	case iUART_XBEE:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTC0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTC0.CTRLA = tempCTRLA;
		break;
	case iUART_TERM:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTF0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTF0.CTRLA = tempCTRLA;
		break;
	}

}
//----------------------------------------------------------------------------------------
void drv_set_baudrate(uint32_t baudRate, uint8_t *baudA, uint8_t *baudB, uint8_t *ctl )
{
#if F_CPU == 32000000

	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock frequency that is 32 MHz.
	 * Los valores los extraigo de la planilla provista por Atmel
	 * 32Mhz
	 * BSEL = 2094
	 * BSCALE = -7
	 * CLK2X = 0
	 * %error = 0,01%
	 */
	switch(baudRate) {
	case 115200:
		*baudA = (uint8_t) 2094;
		*baudB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
		break;
	case 9600:
		// 9600
		*baudA = (uint8_t) 3317;
		*baudB = ( -4 << USART_BSCALE0_bp)|(3317 >> 8);
		break;
	}

#endif

#if F_CPU == 8000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 8Mhz
		 * BSEL = 983
		 * BSCALE = -7
		 * CLK2X = 1
		 * %error = 0,01%
		 */
	*baudA = (uint8_t) 983;
	*baudB = ( -7 << USART_BSCALE0_bp)|(983 >> 8);
		// Habilito CLK2X
	*ctl |= USART_CLK2X_bm;
#endif

#if F_CPU == 2000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 2 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 2Mhz
		 * BSEL = 11
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,08%
		 */
		*baudA = (uint8_t) 11;
		*baudB = ( -7 << USART_BSCALE0_bp)|(11 >> 8);
#endif
}
//----------------------------------------------------------------------------------------
// UART GPRS:
//----------------------------------------------------------------------------------------
void drv_uart_gprs_open( uint32_t baudrate )
{
	// El puerto del GPRS es PORTE:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA, baudB, ctl;

	PORTE.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTE.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTE0, 8 Data bits, No Parity, 1 Stop bit.
	USARTE0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTE0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTE0.BAUDCTRLA = baudA;
	USARTE0.BAUDCTRLB = baudB;
	USARTE0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTE0.CTRLB |= USART_RXEN_bm;
	USARTE0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTE0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTE0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTE0.CTRLA = ( USARTE0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	return;
}
//----------------------------------------------------------------------------------------
ISR(USARTE0_DRE_vect)
{

char cChar;
int8_t res = false;

	res = rBufferPop( &uart_gprs.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTE0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_gprs.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTE0_RXC_vect)
{

char cChar;

	cChar = USARTE0.DATA;

	if( rBufferPokeFromISR( &uart_gprs.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------
// UART XBEE:
//----------------------------------------------------------------------------------------
void drv_uart_xbee_open( uint32_t baudrate )
{
	// El puerto del XBEE es PORTC:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA, baudB, ctl;

	PORTC.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTC.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTC0, 8 Data bits, No Parity, 1 Stop bit.
	USARTC0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTC0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTC0.BAUDCTRLA = baudA;
	USARTC0.BAUDCTRLB = baudB;
	USARTC0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTC0.CTRLB |= USART_RXEN_bm;
	USARTC0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTC0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTC0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTC0.CTRLA = ( USARTE0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	return;

}
//----------------------------------------------------------------------------------------
ISR(USARTC0_DRE_vect)
{

char cChar;
int8_t res = false;

	res = rBufferPop( &uart_xbee.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTC0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_xbee.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTC0_RXC_vect)
{

char cChar;

	cChar = USARTC0.DATA;

	if( rBufferPokeFromISR( &uart_xbee.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------
// UART TERM:
//----------------------------------------------------------------------------------------
void drv_uart_term_open( uint32_t baudrate )
{
	// El puerto del TERM es PORTF:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX
	// Debe operar a 9600 si tiene el BT y a 115200 si es el USB para que se pueda conectar con el celular.

uint8_t baudA, baudB, ctl;

	PORTF.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTF.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTF0, 8 Data bits, No Parity, 1 Stop bit.
	USARTF0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTF0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTF0.BAUDCTRLA = baudA;
	USARTF0.BAUDCTRLB = baudB;
	USARTF0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTF0.CTRLB |= USART_RXEN_bm;
	USARTF0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTF0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTF0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTF0.CTRLA = ( USARTF0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	return;
}
//----------------------------------------------------------------------------------------
ISR(USARTF0_DRE_vect)
{

char cChar;
int8_t res = false;

	res = rBufferPop( &uart_term.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTF0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_term.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTF0_RXC_vect)
{

char cChar;

	cChar = USARTF0.DATA;

	if( rBufferPokeFromISR( &uart_term.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------



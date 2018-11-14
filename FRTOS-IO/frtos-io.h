/*
 * frtos-io.h
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 *
 * Funcionamiento:
 * Usamos los servicios que nos brindan los drivers.
 * Definimos para cada periferico una estructura de control que depende del periferico.
 * En el caso del los puertos seriales es periferico_serial_port_t.
 * Este tiene un elemento que es un puntero a una uart definida en el driver.
 *
 * Cada periferico se asocia a un file descriptor de modo que las funciones genericas
 * frtos_open/ioctl/read/write por medio de un switch redirigen a funciones mas especializadas
 * en cada tipo de periferico.
 *
 */

#ifndef SRC_FRTOS_IO_FRTOS_IO_H_
#define SRC_FRTOS_IO_FRTOS_IO_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "../drivers/drv_i2c_spxR3.h"
#include "../drivers/drv_uart_spxR3.h"

// Identificador de los file descriptor.
typedef enum {
	fdTERM = 1,
	fdGPRS,
	fdXBEE,
	fdI2C,

} file_descriptor_t;

// Estructuctura generica de un periferico tipo serial port.
typedef struct {
	file_descriptor_t fd;
	SemaphoreHandle_t xBusSemaphore;		//
	uint8_t xBlockTime;						// ticks to block in read operations. Set by ioctl
	uart_control_t *uart;					// puntero a una estructura con los miembros adecuados al periferico
	                                        // La uart pertenece al driver. El periferico la apunta.
} periferico_serial_port_t;

// Estructuctura generica de un periferico tipo bus i2c.
typedef struct {
	file_descriptor_t fd;
	SemaphoreHandle_t xBusSemaphore;		//
	uint8_t xBlockTime;						// ticks to block in read operations. Set by ioctl
	uint8_t devAddress;
	uint8_t byteAddressLength;
	uint16_t byteAddress;
	uint8_t i2c_error_code;

} periferico_i2c_port_t;

// Periferico real.
periferico_serial_port_t xComGPRS, xComXBEE, xComTERM;
StaticSemaphore_t GPRS_xMutexBuffer,XBEE_xMutexBuffer,TERM_xMutexBuffer;

periferico_i2c_port_t xBusI2C;
StaticSemaphore_t I2C_xMutexBuffer;

#define ioctl_OBTAIN_BUS_SEMPH			1
#define ioctl_RELEASE_BUS_SEMPH			2
#define ioctl_SET_TIMEOUT				3
#define ioctl_UART_CLEAR_RX_BUFFER		4
#define ioctl_UART_CLEAR_TX_BUFFER		5

#define ioctl_I2C_SET_DEVADDRESS		6
#define ioctl_I2C_SET_BYTEADDRESS		7
#define ioctl_I2C_SET_BYTEADDRESSLENGTH	8
#define ioctl_I2C_GET_LAST_ERROR		9

#define I2C_OK			0
#define I2C_RD_ERROR	1
#define I2C_WR_ERROR	2

//-----------------------------------------------------------------------
int frtos_open( file_descriptor_t fd, uint32_t flags);
int frtos_uart_open( periferico_serial_port_t *xCom, file_descriptor_t fd, StaticSemaphore_t *xCom_semph, uart_id_t uart_id, uint32_t flags);
int frtos_i2c_open( periferico_i2c_port_t *xI2c, file_descriptor_t fd, StaticSemaphore_t *i2c_semph, uint32_t flags);

int frtos_write( file_descriptor_t fd ,const char *pvBuffer, const uint16_t xBytes );
int frtos_uart_write( periferico_serial_port_t *xCom, const char *pvBuffer, const uint16_t xBytes );
int frtos_i2c_write( periferico_i2c_port_t *xI2c, const char *pvBuffer, const uint16_t xBytes );

int frtos_ioctl( file_descriptor_t fd, uint32_t ulRequest, void *pvValue );
int frtos_uart_ioctl( periferico_serial_port_t *xCom, uint32_t ulRequest, void *pvValue );
int frtos_i2c_ioctl( periferico_i2c_port_t *xI2c, uint32_t ulRequest, void *pvValue );

int frtos_read( file_descriptor_t fd , char *pvBuffer, uint16_t xBytes );
int frtos_uart_read( periferico_serial_port_t *xCom, char *pvBuffer, uint16_t xBytes );
int frtos_i2c_read( periferico_i2c_port_t *xI2c, char *pvBuffer, uint16_t xBytes );

#endif /* SRC_FRTOS_IO_FRTOS_IO_H_ */

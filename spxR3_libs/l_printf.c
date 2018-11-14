/*
 * l_printf.c
 *
 *  Created on: 13 jul. 2018
 *      Author: pablo
 */


#include "../spxR3_libs/l_printf.h"


#define PRINTF_BUFFER_SIZE        256U

static uint8_t stdout_buff[PRINTF_BUFFER_SIZE];
xSemaphoreHandle sem_STDOUT;
StaticSemaphore_t STDOUT_xMutexBuffer;

//-----------------------------------------------------------------------------------
int xprintf_P( PGM_P fmt, ...)
{
	// Imprime formateando en el uart fd.usando el buffer stdout_buff.
	// Como se controla con semaforo, nos permite ahorrar los buffers de c/tarea.
	// Si bien vsnprintf no es thread safe, al usarla aqui con el semaforo del buffer
	// queda thread safe !!!
	// Al usar esta funcion no es necesario controlar el tamaño de los buffers ya que
	// los limita a PRINTF_BUFFER_SIZE

va_list args;
int i;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_STDOUT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 5 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	memset(stdout_buff,'\0',PRINTF_BUFFER_SIZE);
	va_start(args, fmt);
	vsnprintf_P( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	i = frtos_write(fdTERM, (char *)stdout_buff, PRINTF_BUFFER_SIZE );

	xSemaphoreGive( sem_STDOUT );
	return(i);

}
//-----------------------------------------------------------------------------------
int xprintf( const char *fmt, ...)
{
	// Imprime formateando en el uart fd.usando el buffer stdout_buff.
	// Como se controla con semaforo, nos permite ahorrar los buffers de c/tarea.
	// Si bien vsnprintf no es thread safe, al usarla aqui con el semaforo del buffer
	// queda thread safe !!!

va_list args;
int i;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_STDOUT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 5 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	memset(stdout_buff,'\0',PRINTF_BUFFER_SIZE);
	va_start(args, fmt);
	vsnprintf( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	i = frtos_write(fdTERM, (char *)stdout_buff, PRINTF_BUFFER_SIZE );

	xSemaphoreGive( sem_STDOUT );
	return(i);

}
//-----------------------------------------------------------------------------------
int xnprint( const char *pvBuffer, const uint16_t xBytes )
{
	// Imprime en fdUSB sin formatear

int bytes2wr = 0;

	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 0 )
		return(bytes2wr);

//	frtos_ioctl (fdUSB,ioctl_OBTAIN_BUS_SEMPH, NULL );
//	frtos_ioctl (fdBT,ioctl_OBTAIN_BUS_SEMPH, NULL );

	bytes2wr = frtos_write( fdTERM, pvBuffer, xBytes );

//	frtos_ioctl (fdBT,ioctl_RELEASE_BUS_SEMPH, NULL);
//	frtos_ioctl (fdUSB,ioctl_RELEASE_BUS_SEMPH, NULL);

	return(bytes2wr);

}
//-----------------------------------------------------------------------------------
void xputChar(unsigned char c)
{
	// Funcion intermedia necesaria por cmdline para escribir de a 1 caracter en consola
	// El tema es que el prototipo de funcion que requiere cmdlineSetOutputFunc no se ajusta
	// al de FreeRTOS_UART_write, por lo que defino esta funcion intermedia.

char cChar;

	cChar = c;
	xnprint( &cChar, sizeof(char));
}
//-----------------------------------------------------------------------------------
int xCom_printf_P( file_descriptor_t fd, PGM_P fmt, ...)
{
	// Idem que xprintf_P pero imprime sobre el descriptor tipo uart indicado con fd.

va_list args;
int i;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_STDOUT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 5 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	memset(stdout_buff,'\0',PRINTF_BUFFER_SIZE);
	va_start(args, fmt);
	vsnprintf_P( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	i = frtos_write(fd, (char *)stdout_buff, PRINTF_BUFFER_SIZE );

	xSemaphoreGive( sem_STDOUT );
	return(i);

}
//-----------------------------------------------------------------------------------
int xCom_printf( file_descriptor_t fd, const char *fmt, ...)
{
	// Idem que xCom_printf_P pero el formato esta en RAM.

va_list args;
int i;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_STDOUT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 5 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	memset(stdout_buff,'\0',PRINTF_BUFFER_SIZE);
	va_start(args, fmt);
	vsnprintf( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	i = frtos_write(fd, (char *)stdout_buff, PRINTF_BUFFER_SIZE );

	xSemaphoreGive( sem_STDOUT );
	return(i);

}
//-----------------------------------------------------------------------------------
int xCom_nprint( file_descriptor_t fd, const char *pvBuffer, const uint16_t xBytes )
{
	// Imprime en fd sin formatear

int bytes2wr = 0;

	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 0 )
		return(bytes2wr);

	frtos_ioctl (fdTERM,ioctl_OBTAIN_BUS_SEMPH, NULL );
	bytes2wr = frtos_write( fdTERM, pvBuffer, xBytes );
	frtos_ioctl (fdTERM,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(bytes2wr);

}
//-----------------------------------------------------------------------------------
void xCom_putChar(file_descriptor_t fd, unsigned char c)
{

char cChar;

	cChar = c;
	xCom_nprint( fd, &cChar, sizeof(char));
}
//-----------------------------------------------------------------------------------
void xprintf_init(void)
{
	sem_STDOUT = xSemaphoreCreateMutexStatic( &STDOUT_xMutexBuffer );
}
//------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// Formatea e imprime en el fdUSB
//-----------------------------------------------------------------------------------
/*

//-----------------------------------------------------------------------------------
// Imprime sin formatear en el fdUSB
//-----------------------------------------------------------------------------------
// Formatea e imprime en un fd tipo uart
//-----------------------------------------------------------------------------------
void xCom_printf( const xComPortHandlePtr pxPort, const char * format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) taskYIELD();
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, (const char *)format, arg);
	xSerialxPrint(pxPort, (uint8_t *)(pxPort->serialWorkBuffer));

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}
//-----------------------------------------------------------------------------------
void xCom_printf_P( const xComPortHandlePtr pxPort, PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) taskYIELD();
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf_P((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, format, arg);
	xSerialxPrint(pxPort, (uint8_t *)(pxPort->serialWorkBuffer));

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}
//-----------------------------------------------------------------------------------
// Imprime sin formatear en un fd tipo uart
//-----------------------------------------------------------------------------------
void xCom_print( const xComPortHandlePtr pxPort, const uint8_t * str)
{
	int16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		xSerialPutChar( pxPort, str[i++]);
}
//-----------------------------------------------------------------------------------
void xCom_print_P( const xComPortHandlePtr pxPort, PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		xSerialPutChar( pxPort, pgm_read_byte(&str[i++]) );
}
//-----------------------------------------------------------------------------------
*/

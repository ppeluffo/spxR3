/*
 * l_file.c
 *
 *  Created on: 31/10/2015
 *      Author: pablo
 */


#include "l_file.h"
#include "l_i2c.h"
#include "l_rtc79410.h"

#include "spxR3.h"

static bool pv_FAT_load( FAT_t *fat );
static bool pv_FAT_save( FAT_t *fat );
static uint8_t pv_chksum8(const char *buff, size_t len);

xSemaphoreHandle sem_FAT;
StaticSemaphore_t FAT_xMutexBuffer;

//-----------------------------------------------------------------------------------
void FAT_init(void)
{
	sem_FAT = xSemaphoreCreateMutexStatic( &FAT_xMutexBuffer );
}
//------------------------------------------------------------------------------------
bool FF_open(void)
{
	// Inicializa el sistema del file system leyendo de la SRAM del RTC la FAT
	// y cargandola en memoria.
	// Debo chequear la consistencia y si esta mal indicarlo y reiniciarla

bool retS = false;

	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	retS = pv_FAT_load( &FCB.fat);

	// Ajustes iniciales.
	// 1: No hay registros leidos aun.
	FCB.fat.rdPTR = FCB.fat.delPTR;
	// 2: No hay registros para borrar aun.
	FCB.fat.rcds4del = 0;
	// 3: Todos los registros escritos estan disponibles para leer
	FCB.fat.rcds4rd = FF_MAX_RCDS - FCB.fat.rcds4wr;

	pv_FAT_save( &FCB.fat);

	FCB.errno = pdFF_ERRNO_NONE;

	xSemaphoreGive( sem_FAT);

	return(retS);

}
//------------------------------------------------------------------------------------
int8_t FF_writeRcd( const void *pvBuffer, uint8_t xSize )
{
	// El archivo es del tipo circular FirstIN-LastOUT.
	// Escribe un registro en la posicion apuntada por el HEAD y avanzo el puntero.
	// Decremento el rcdsFree.
	// El registro que se pasa en pvBuffer es del tipo 'frameData_t' de x bytes
	// pero en la memoria voy a escribir de a paginas de 64 bytes.
	// Retorna el nro.de bytes escritos y setea la variable 'errno' del FCB
	// En la posicion 63 grabo un tag con el valor 0xC5 para poder luego
	// determinar si el registro esta ocupado o vacio.
	// En la posicion 62 grabo el checksum.
	// Esto implica que xSize debe ser menor de 62. Por ahora NO LO CONTROLO pero deberia
	//
	// Sigue la logica de la Tabla de movimientos de fat.
	//       | rcdsXwr | rcdsXrd | rcdsXdel
	//  -------------------------------------
	// WRITE |   -1    |   +1    |    x
	//
	// Memoria llena: rcds4wr = 0, rcds4del = MAX;

	// TESTING:
	// Memoria vacia: OK
	// Memoria llena: OK
	// Memoria con HEAD(10) > TAIL(4), Free(10) OK
	// Memoria con HEAD(3) < TAIL(8), Free(5) OK
	// Condicion de borde 1: HEAD(15), TAIL(0), Free(1) OK
	// Condicion de borde 2: HEAD(0), TAIL(1), Free(1) OK

uint16_t wrAddress;
int8_t bytes_written = -1;
//uint8_t tryes;
//bool write_ok;

	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	FCB.errno = pdFF_ERRNO_NONE;

	// Si la memoria esta llena no puedo escribir: salgo
	if ( FCB.fat.rcds4wr == 0 ) {
		FCB.errno = pdFF_ERRNO_MEMFULL;
		goto quit;
	}

	// inicializo la estructura lineal temporal en el FCB para copiar ahi los datos y
	// calcular el checksum antes de grabarlo en memoria.
	memset( FCB.rw_buffer,0xFF, FF_RECD_SIZE );
	// copio los datos recibidos del frame al buffer ( 0..(xSize-1))
	memcpy ( FCB.rw_buffer, pvBuffer, xSize );
	// Calculo y grabo el checksum a continuacion del frame (en la pos.xSize)
	// El checksum es solo del dataFrame por eso paso dicho size.
	FCB.rw_buffer[FF_RECD_SIZE - 2] = pv_chksum8(FCB.rw_buffer, (FF_RECD_SIZE - 2) );
	// Grabo el tag para indicar que el registro esta escrito.
	FCB.rw_buffer[FF_RECD_SIZE - 1] = FF_WRTAG;

	// EE WRITE:
	// Direccion interna en la EE.(comienzo del registro / frontera)
	// Necesito esperar un poco entre c/ciclo
	vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
	wrAddress = FF_ADDR_START + FCB.fat.wrPTR * FF_RECD_SIZE;
	bytes_written = EE_write( wrAddress, (char *)&FCB.rw_buffer, FF_RECD_SIZE );
	if ( bytes_written == -1 )
		xprintf_P(PSTR("ERROR: I2C:EE:FF_writeRcd\r\n\0"));

	if ( bytes_written != FF_RECD_SIZE ) {
		// Errores de escritura ?
		FCB.errno = pdFF_ERRNO_MEMWR;
		bytes_written = -1;
//		FRTOS_snprintf_P( stdout_buff,sizeof(stdout_buff),PSTR("DEBUG: BD WR ERROR: (%d)\r\n\0"),tryes );
//		CMD_write(stdout_buff, sizeof(stdout_buff) );
		goto quit;
	}

//	FRTOS_snprintf_P( stdout_buff,sizeof(stdout_buff),PSTR("DEBUG: WR_OK: Address=%d, rcdNbr=%d, cks=0x%02x\r\n\0"),wrAddress,FCB.fat.wrPTR, FCB.rw_buffer[FF_RECD_SIZE - 2]  );
//	CMD_write(stdout_buff, sizeof(stdout_buff) );

	// Avanzo el puntero de WR en modo circular
	FCB.fat.wrPTR = (++FCB.fat.wrPTR == FF_MAX_RCDS) ?  0 : FCB.fat.wrPTR;
	FCB.fat.rcds4wr--;
	FCB.fat.rcds4rd++;
	// Actualizo la fat
	pv_FAT_save(&FCB.fat);

quit:
	// libero los semaforos
	xSemaphoreGive( sem_FAT);

	return(bytes_written);

}
//------------------------------------------------------------------------------------
int8_t FF_readRcd( void *pvBuffer, uint8_t xSize )
{
	// Lee un registro apuntado por RD.
	// Retorna la cantidad de bytes leidos.
	// Las condiciones de lectura son:
	// - la memoria debe tener al menos algun dato
	// - el puntero RD debe apuntar dentro del bloque 'leible'
	//
	// Lee un registro ( como string ) y lo copia al espacio de memoria apuntado
	// *pvBuffer. El puntero es void y entonces debemos pasar el tamaño de la estructura
	// a la que apunta.
	// En caso que tenga problemas para leer o de checksum, debo continuar e indicar
	// el error.
	//
	// Sigue la logica de la Tabla de movimientos de fat.
	//       | rcdsXwr | rcdsXrd | rcdsXdel
	//  -------------------------------------
	// READ  |    x    |   -1    |    +1
	//
	// Memoria vacia: rcds4wr = MAX, rcds4del = 0;
	// Memoria toda leida: rcds4rd = 0;

uint8_t rdCheckSum;
uint16_t rdAddress;
int8_t bytes_read = 0U;
//uint16_t rcdPos = 0;

	// Lo primero es obtener el semaforo
	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	FCB.errno = pdFF_ERRNO_NONE;

	// Si la memoria no tiene registros para leer salgo
	if ( FCB.fat.rcds4rd == 0 ) {
		FCB.errno = pdFF_ERRNO_MEMEMPTY;
		goto quit;
	}

	// inicializo la estructura lineal temporal en el FCB.
	memset( FCB.rw_buffer,0xFF, FF_RECD_SIZE );
	// EE READ:
	// Direccion interna en la EE.(comienzo del registro / frontera)
	//rcdPos = FCB.fat.rdPTR;
	rdAddress = FF_ADDR_START + FCB.fat.rdPTR * FF_RECD_SIZE;
	bytes_read = EE_read( rdAddress, (char *)&FCB.rw_buffer, FF_RECD_SIZE);
	if (bytes_read == -1 )
		xprintf_P(PSTR("ERROR: I2C:EE:FF_readRcd\r\n\0"));

	// Avanzo el puntero de RD en modo circular siempre !!
	FCB.fat.rdPTR = (++FCB.fat.rdPTR == FF_MAX_RCDS) ?  0 : FCB.fat.rdPTR;
	FCB.fat.rcds4rd--;
	FCB.fat.rcds4del++;
	// Actualizo la fat
	pv_FAT_save(&FCB.fat);

	// Copio los datos a la estructura de salida.: aun no se si estan correctos
	memcpy( pvBuffer, &FCB.rw_buffer, xSize );

	// Errores de lectura ?
	// Solo indico los errores, pero igual devuelvo el recd. para no trancarme
	if (bytes_read != FF_RECD_SIZE ) {
		FCB.errno = pdFF_ERRNO_MEMRD;
		bytes_read = -1;
		goto quit;
	}

	// Verifico los datos leidos ( checksum )
	// El checksum es solo del dataFrame por eso paso dicho size.
	rdCheckSum = pv_chksum8( FCB.rw_buffer, (FF_RECD_SIZE - 2) );
	if ( rdCheckSum != FCB.rw_buffer[(FF_RECD_SIZE - 2)] ) {
		FCB.errno = pdFF_ERRNO_RDCKS;
		bytes_read = -1;
		goto quit;
	}

	// Vemos si la ultima posicion tiene el tag de ocupado.
	if ( ( FCB.rw_buffer[FF_RECD_SIZE - 1] )  != FF_WRTAG ) {
		FCB.errno = pdFF_ERRNO_RDNOTAG;
		bytes_read = -1;
		goto quit;
	}

//	FRTOS_snprintf_P( stdout_buff,sizeof(stdout_buff),PSTR("DEBUG: RD_OK: Address=%d, rcdNbr=%d, cks=0x%02x\r\n\0"),rdAddress,rcdPos, rdCheckSum  );
//	CMD_write(stdout_buff, sizeof(stdout_buff) );


quit:
	// libero el semaforo
	xSemaphoreGive( sem_FAT);

	return(bytes_read);
}
//------------------------------------------------------------------------------------
void FF_rewind(void)
{
	// Como leo con un puntero, escribo con otro y borro con otro, con esto inicializo
	// el puntero de lectura, por ejemplo luego de un reset.
	// Ajusta la posicion del puntero de lectura FAT.RD al primer registro FAT.DEL.

	// Lo primero es obtener el semaforo
	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	FCB.fat.rdPTR = FCB.fat.delPTR;
	// 2: No hay registros para borrar aun.
	FCB.fat.rcds4del = 0;
	// 3: Todos los registros escritos estan disponibles para leer
	FCB.fat.rcds4rd = FF_MAX_RCDS - FCB.fat.rcds4wr;

	pv_FAT_save(&FCB.fat);

	// libero el semaforo
	xSemaphoreGive( sem_FAT);
	return;

}
//------------------------------------------------------------------------------------
void FF_deleteRcd(void)
{
	// Borra el registro apuntado por TAIL y lo avanzo circularmente.
	// Los registros que se pueden borrar son aquellos que ya hallan sido leidos
	// ( trasmitidos ), Esto lo indica el contador rcds4Del.
	// Sigue la logica de la Tabla de movimientos de fat.
	//       | rcdsXwr | rcdsXrd | rcdsXdel
	//  -------------------------------------
	// DEL   |   +1    |    x    |    -1
	//
	// Memoria vacia: rcds4wr = MAX, rcds4del = 0

uint16_t delAddress = 0;
int8_t xBytes;

	// Lo primero es obtener el semaforo
	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	if ( FCB.fat.rcds4del == 0 ) {
		// Memoria vacia.
		return;
	}

	// Borro fisicamente el registro
	memset( FCB.rw_buffer,0xFF, FF_RECD_SIZE );
	delAddress = FF_ADDR_START + FCB.fat.delPTR * FF_RECD_SIZE;
	xBytes = EE_write( delAddress, (char *)&FCB.rw_buffer, FF_RECD_SIZE );
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:EE:FF_deleteRcd\r\n\0"));

	// Ajusto la FAT
	FCB.fat.rcds4wr++;
	FCB.fat.rcds4del--;
	FCB.fat.delPTR = (++FCB.fat.delPTR == FF_MAX_RCDS) ?  0 : FCB.fat.delPTR;
	pv_FAT_save(&FCB.fat);

	// libero el semaforo
	xSemaphoreGive( sem_FAT);
	return;
}
//------------------------------------------------------------------------------------
void FF_format(bool fullformat )
{
	// Inicializa la memoria reseteando solo la FAT.

uint16_t page;
uint16_t wrAddress;
int8_t xBytes;

	// Lo primero es obtener el semaforo
	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	FCB.fat.wrPTR = 0;
	FCB.fat.rdPTR = 0;
	FCB.fat.delPTR = 0;
	FCB.fat.rcds4wr = FF_MAX_RCDS;
	FCB.fat.rcds4rd = 0;
	FCB.fat.rcds4del = 0;

	pv_FAT_save(&FCB.fat);

	if ( fullformat ) {
		// Para que no salga por watchdog, apago las tareas
		vTaskSuspend( xHandle_tkData );
		pub_ctl_watchdog_kick(WDG_DAT, 0xFFFF);

		vTaskSuspend( xHandle_tkCounter );
		pub_ctl_watchdog_kick(WDG_COUNT, 0xFFFF);

		vTaskSuspend( xHandle_tkGprsTx );
		pub_ctl_watchdog_kick(WDG_GPRSTX, 0xFFFF);

		vTaskSuspend( xHandle_tkGprsRx );
		pub_ctl_watchdog_kick(WDG_GPRSRX, 0xFFFF);

		vTaskSuspend( xHandle_tkOutputs );
		pub_ctl_watchdog_kick(WDG_OUT, 0xFFFF);

		// Borro fisicamente los registros
		memset( FCB.rw_buffer,0xFF, FF_RECD_SIZE );
		for ( page = 0; page < FF_MAX_RCDS; page++) {
			wrAddress = FF_ADDR_START + page * FF_RECD_SIZE;
			xBytes = EE_write( wrAddress, (char *)&FCB.rw_buffer, FF_RECD_SIZE );
			if ( xBytes == -1 )
				xprintf_P(PSTR("ERROR: I2C:EE:FF_deleteRcd\r\n\0"));

			vTaskDelay( ( TickType_t)( 10 ) );

			if ( ( page > 0 ) && (page % 32) == 0 ) {
				xprintf_P( PSTR(" %04d\0"),page);
				if ( ( page > 0 ) && (page % 256) == 0 ) {
					xprintf_P( PSTR("\r\n\0"));
				}
			}


		}

	}

	// libero el semaforo
	xSemaphoreGive( sem_FAT);
}
//----------------------------------------------------------------------------------
uint8_t FF_errno( void )
{
	// Retorna el codigo de error de errno.
	return( FCB.errno);
}
//----------------------------------------------------------------------------------
void FAT_read( FAT_t *fat )
{
	// Lo primero es obtener el semaforo
	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	memcpy( fat, &FCB.fat, sizeof(FAT_t));

	// libero el semaforo
	xSemaphoreGive( sem_FAT);

}
//----------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
// No usan el semaforo de la FAT.
//----------------------------------------------------------------------------------
static bool pv_FAT_load( FAT_t *fat )
{
	// Lee la FAT de la SRAM del RTC. Verifica su integridad con el checksum
	// No uso el semaforo ya que es una funcion privada.

uint8_t cks = 0x00;
bool retS = false;
int8_t xBytes;

	xBytes = RTC_read( FAT_ADDRESS, (char *)fat, (uint8_t)sizeof(FAT_t) );
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:pv_FAT_load\r\n\0"));


	cks = pv_chksum8( (char *)fat, (sizeof( FAT_t) - 1));
	if ( cks != fat->checksum ) {
		// Error al cargar la fat.
		retS = false;
		goto quit;
	}

	retS = true;
quit:

	return(retS);

}
//----------------------------------------------------------------------------------
static bool pv_FAT_save( FAT_t *fat )
{
	// Escribe la FAT en la SRAM del RTC a partir de la direccion 0x20. Calcula y escribe el checksum
	// Por ahora no controlo errores

uint8_t cks = 0x00;
int8_t xBytes;

	cks = pv_chksum8( (char *)fat, (sizeof( FAT_t) - 1));
	fat->checksum = cks;
	xBytes = RTC_write( FAT_ADDRESS, (char *)fat, (uint8_t)sizeof(FAT_t) );
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:pv_FAT_save\r\n\0"));

	return(true);
}
//----------------------------------------------------------------------------------
static uint8_t pv_chksum8(const char *buff, size_t len)
{
uint8_t checksum = 0;

	for ( checksum = 0 ; len != 0 ; len-- )
		checksum += *(buff++);   // parenthesis not required!

	checksum = ~checksum;
	return (checksum);
}
//----------------------------------------------------------------------------------


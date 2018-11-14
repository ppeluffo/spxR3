/*
 * file_sp5K.h
 *
 *  Created on: 31/10/2015
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KLIBS_L_FILE_H_
#define SRC_SP5KLIBS_L_FILE_H_

#include <l_eeprom.h>
#include "FreeRTOS.h"
#include "frtos-io.h"
#include "spxR3.h"
//#include "../spxR3_libs/l_printf.h"
#include "l_printf.h"

#define FF_SIZE_IN_KB	32	// Tamanio en KB de la eeprom externa.
#define FF_RECD_SIZE	64	// Tamanio del registro
#define FF_ADDR_START	0	// Posicion inicial
#define FF_MAX_RCDS		1024	// Cantidad de registros ( max 4096 en M24CM02 ).
//#define FF_MAX_RCDS		128

#define FF_WRTAG	0xC5	// 1100 0101
#define FF_EMPTYTAG	0xFF

/*
 * **************************************************************************************
 * Tabla de movimientos de la FAT
 *
 *       | rcdsXwr | rcdsXrd | rcdsXdel
 * -------------------------------------
 * WRITE |   -1    |   +1    |    x
 * READ  |    x    |   -1    |    +1
 * DEL   |   +1    |    x    |    -1
 * -------------------------------------
 *
 * Memoria vacia: rcds4wr = MAX, rcds4del = 0;
 * Memoria llena: rcds4wr = 0, rcds4del = MAX;
 * Memoria toda leida: rcds4rd = 0;
 *
 */
typedef struct {		// Estructura de control de archivo
	uint16_t wrPTR;		// Puntero a la primera posicion libre.
	uint16_t delPTR;	// Puntero a la primera posicion ocupada
	uint16_t rdPTR;		// Puntero de lectura. Se mueve entre la posicion ocupada y la libre
	uint16_t rcds4wr;	// Registros libres para escribir.
	uint16_t rcds4del;	// rcds. para borrar ( espacio ocupado y leido )
	uint16_t rcds4rd;	// rcds. para leer
	uint8_t checksum;
} FAT_t;

typedef struct {					// File Control Block
	FAT_t fat;						// Estructura de control de archivo
	uint8_t errno;					// Codigo del ultimo error
	char rw_buffer[FF_RECD_SIZE];	//
	char check_buffer[FF_RECD_SIZE];
} FCB_t;

FCB_t FCB;

#define pdFF_ERRNO_NONE		0
#define pdFF_ERRNO_MEMFULL	1
#define pdFF_ERRNO_MEMWR	2
#define pdFF_ERRNO_MEMEMPTY	3
#define pdFF_ERRNO_MEMRD	4
#define pdFF_ERRNO_RDCKS	5
#define pdFF_ERRNO_RDNOTAG	6
#define pdFF_ERRNO_INIT		7

//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void FAT_init(void);
bool FF_open(void);
int8_t FF_writeRcd( const void *pvBuffer, uint8_t xSize );
int8_t FF_readRcd( void *pvBuffer, uint8_t xSize );
uint8_t FF_errno( void );
void FF_rewind(void);
void FF_deleteRcd(void);
void FF_format( bool fullformat );
void FAT_read( FAT_t *fat );
//------------------------------------------------------------------------------------

#endif /* SRC_SP5KLIBS_L_FILE_H_ */

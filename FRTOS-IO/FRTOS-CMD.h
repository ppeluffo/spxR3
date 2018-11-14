/*
 * FRTOS-CMD.h
 *
 *  Created on: 5 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_FRTOS_IO_FRTOS_CMD_H_
#define SRC_FRTOS_IO_FRTOS_CMD_H_

/* Creo una lista de comandos como un array de estructuras.
 * Cada una tiene 2 elementos, el nombre del comando, y un puntero de funcion
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "l_printf.h"

#define CMDLINE_MAX_CMD_LENGTH	10
#define CMDLINE_MAX_COMMANDS	12
#define CMDLINE_HISTORYSIZE		3

typedef void (*CmdlineFuncPtrType)(void);

#define MAX_INPUT_LENGTH	64

#define ASCII_BEL				0x07
#define ASCII_BS				0x08
#define ASCII_CR				0x0D
#define ASCII_LF				0x0A
#define ASCII_ESC				0x1B
#define ASCII_DEL				0x7F
#define VT100_ARROWUP           'A'
#define VT100_ARROWDOWN         'B'

#define CMDLINE_HISTORY_SAVE    0
#define CMDLINE_HISTORY_PREV    1
#define CMDLINE_HISTORY_NEXT    2

char *argv[16];

void FRTOS_CMD_init( void );
void FRTOS_CMD_register( char *newCmdString, void (*fnptr)(void) );
void FRTOS_CMD_process( char cRxedChar );
void FRTOS_CMD_History( uint8_t action);
void FRTOS_CMD_regtest(void);
uint8_t FRTOS_CMD_makeArgv(void);

#endif /* SRC_FRTOS_IO_FRTOS_CMD_H_ */

/*
 * FRTOS-CMD.c
 *
 *  Created on: 5 de oct. de 2017
 *      Author: pablo
 */

#include "FRTOS-CMD.h"

// Lista de comandos:
// Estructura que define la base de datos nombre,funcion de c/comando aceptado.
// Son 2 listas; una del comando y otra de la funcion
static char BDCMD_commandList[CMDLINE_MAX_COMMANDS][CMDLINE_MAX_CMD_LENGTH];	// -commands are null-terminated strings
static CmdlineFuncPtrType BDCMD_commandFuntions[CMDLINE_MAX_COMMANDS];			// command function pointer list
uint8_t BDCMD_numCommands;	// number of commands currently registered

CmdlineFuncPtrType CmdlineExecFunction;

// Estructura que define la linea de comandos.
static char cmdLine_buffer[MAX_INPUT_LENGTH];
static char cmdLine_History_buffer[MAX_INPUT_LENGTH];
static uint8_t cmdLine_ptr;


void pv_CMD_print_prompt(void);
void pv_CMD_execute(void);
void pv_CMD_print_error(void);
void pv_CMD_init( void );

// Estado: 0: normal, 1 recibi ESC, 2 recibi ESC [ lo que indica que el proximo caracters es
// un codigo VT100.
uint8_t VT100State;

//----------------------------------------------------------------------------------------
void FRTOS_CMD_register(char *newCmdString, void (*fnptr)(void) )
{
	// Registro el comando ( nombre, funcion ) en la lista de comandos.
	// No controlo overflow !!!

	// Inicializo la memoria del CMD.
	memset(BDCMD_commandList[BDCMD_numCommands], '\0', CMDLINE_MAX_CMD_LENGTH);

	// Agrego el nombre del comando
	strcpy( BDCMD_commandList[BDCMD_numCommands], newCmdString);

	// Agrego el puntero a la funcion que lo ejecuta
	BDCMD_commandFuntions[BDCMD_numCommands] = fnptr;

	BDCMD_numCommands++;
}
//----------------------------------------------------------------------------------------
void FRTOS_CMD_regtest(void)
{
//uint8_t i;

//	for ( i = 0; i < BDCMD_numCommands; i++) {
//		FRTOS_snprintf_P( d_printfBuff,sizeof(d_printfBuff),PSTR("REG[%d]: %s\r\n\0"),i, BDCMD_commandList[i]);
//		CMD_write(d_printfBuff, sizeof(d_printfBuff) );
//	}

}
//----------------------------------------------------------------------------------------
void FRTOS_CMD_process( char cRxedChar )
{

	// Procesa el caracter recibido.
	// Si es CR, ejecuta la funcion correspondiente
	// Si no lo guarda ( editado ) en el buffer.


	if ( VT100State == 2 ) {
		// recibi ESC [ de modo que el proximo caracter es de control
		// Solo proceso flecha arriba y abajo.
		switch (cRxedChar) {
		case VT100_ARROWUP:
			FRTOS_CMD_History(CMDLINE_HISTORY_PREV);
			break;
		case VT100_ARROWDOWN:
			FRTOS_CMD_History(CMDLINE_HISTORY_NEXT);
			break;
		default:
			break;
		}
		// Done
		VT100State = 0;
		return;
	}

	if ( VT100State == 1 ) {
		// Recibi el ESC.
		if(cRxedChar == '[') {
			VT100State = 2;
			return;
		}
		VT100State = 0;
		return;
	}

	if ( VT100State == 0 ) {
		// Proceso normal
		switch (cRxedChar) {

		case ASCII_ESC:
			VT100State = 1;
			break;

		case ASCII_CR:
			// NEWLINE: Proceso el comando.
			FRTOS_CMD_History(CMDLINE_HISTORY_SAVE);
			xnprint ( "\r\n\0", strlen( "\r\n\0" ));
			pv_CMD_execute();
			pv_CMD_init();
			pv_CMD_print_prompt();
			break;

		case ASCII_BS:
			// BACKSPACE: Borro el ultimo caracter entrado
			if( cmdLine_ptr > 0 ) {
				cmdLine_ptr--;
				cmdLine_buffer[ cmdLine_ptr ] = '\0';
				xputChar(ASCII_BS);
				xputChar(' ');
				xputChar(ASCII_BS);
			}
			break;

		default:
			// Si el caracter es imprimible lo almaceno
			if( (cRxedChar >= 0x20) && (cRxedChar < 0x7F) ) {
				// Echo local
				xputChar(cRxedChar);
				if( cmdLine_ptr < MAX_INPUT_LENGTH ) {
					cmdLine_buffer[ cmdLine_ptr ] = cRxedChar;
					cmdLine_ptr++;
				}
			}
			break;
		}
	}
}
//----------------------------------------------------------------------------------------
void FRTOS_CMD_History( uint8_t action)
{
	switch(action) {
	case CMDLINE_HISTORY_SAVE:
	       // copy CmdlineBuffer to history if not null string
		if( strlen(cmdLine_buffer) ) {
			memset(cmdLine_History_buffer, '\0', sizeof(cmdLine_History_buffer) );
			strcpy(cmdLine_History_buffer, cmdLine_buffer);
		}
		//CMD_write ( "SAVE\r\n\0", strlen( "SAVE\r\n\0" ));
		break;
	case CMDLINE_HISTORY_PREV:
		memset(cmdLine_buffer, '\0', sizeof(cmdLine_buffer) );
		strcpy(cmdLine_buffer,cmdLine_History_buffer);
		cmdLine_ptr = strlen(cmdLine_buffer);
		//CMD_write ( "UP\r\n\0", strlen( "UP\r\n\0" ));
		xnprint ( cmdLine_buffer, strlen( cmdLine_buffer ));
		break;
	case CMDLINE_HISTORY_NEXT:
		//CMD_write ( "DOWN\r\n\0", strlen( "DOWN\r\n\0" ));
		break;
	}
	return;
}
//----------------------------------------------------------------------------------------
void FRTOS_CMD_init(void)
{
	pv_CMD_init();

	VT100State = 0;
	// Inicializo la memoria.
	BDCMD_numCommands = 0;
	pv_CMD_print_prompt();

}
//----------------------------------------------------------------------------------------
uint8_t FRTOS_CMD_makeArgv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, '\0', sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(cmdLine_buffer, parseDelimiters);
	argv[i++] = token;

	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}

	return(( i - 1));
}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//----------------------------------------------------------------------------------------
void pv_CMD_print_prompt(void)
{
	// Muestra el prompt
	xnprint( "cmd>\0", sizeof("cmd>\0") );
}
//----------------------------------------------------------------------------------------
void pv_CMD_print_error(void)
{
	// Muestra el mensaje de error cuando no encontro el comando
	xnprint( "command not found\r\n\0", sizeof("command not found\r\n\0") );
}
//----------------------------------------------------------------------------------------
void pv_CMD_init( void )
{
	// Preparo el buffer de entrada para nuevos comandos.
	cmdLine_ptr = 0;
    memset( cmdLine_buffer, 0x00, MAX_INPUT_LENGTH );

}
//----------------------------------------------------------------------------------------
void pv_CMD_execute(void)
{
	// Busco en el array comandos si hay alguno que coincide con el
	// ingresado en la linea de comandos.

uint8_t i = 0;
uint8_t cmdIndex;

	// Primero debo quedarme solo con el comando y no los argumentos
	// de la linea de comandos.
	// Busco el primer caracter en blanco
	while( !((cmdLine_buffer[i] == ' ') || (cmdLine_buffer[i] == 0)) ) i++;

	if(!i)
	{
		// command was null or empty
		return;
	}

	// Muestro el cmdLine
	//FRTOS_snprintf_P( d_printfBuff,sizeof(d_printfBuff),PSTR("CL[%d]: %s\r\n\0"),i, cmdLine_buffer);
	//CMD_write(d_printfBuff, sizeof(d_printfBuff) );
	//
	// Recorro la BD de comandos ( 2 listas )
	for( cmdIndex = 0; cmdIndex < BDCMD_numCommands; cmdIndex++) {

		// Comparo el nombre del comando
		//FRTOS_snprintf_P( d_printfBuff,sizeof(d_printfBuff),PSTR("LN[%d]: %s\r\n\0"),cmdIndex, BDCMD_commandList[cmdIndex]);
		//CMD_write(d_printfBuff, sizeof(d_printfBuff) );

		if( !strncmp(BDCMD_commandList[cmdIndex], cmdLine_buffer, i) ) {
			// user-entered command matched a command in the list (database)
			// run the corresponding function
			CmdlineExecFunction = BDCMD_commandFuntions[cmdIndex];
			CmdlineExecFunction();
			CmdlineExecFunction = 0;
			return;

		}
	}
	// No hay comando para la linea de entrada: error
	pv_CMD_print_error();

}
//----------------------------------------------------------------------------------------

/*
 * l_printf.h
 *
 *  Created on: 13 jul. 2018
 *      Author: pablo
 */

#ifndef SRC_SPXR3_LIBS_L_PRINTF_H_
#define SRC_SPXR3_LIBS_L_PRINTF_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "task.h"

#include "frtos-io.h"

void xprintf_init(void);
int xprintf_P( PGM_P fmt, ...);
int xprintf( const char *fmt, ...);
void xputChar(unsigned char c);
int xCom_printf_P( file_descriptor_t fd, PGM_P fmt, ...);
int xCom_printf( file_descriptor_t fd, const char *fmt, ...);
int xnprint( const char *pvBuffer, const uint16_t xBytes );
int xCom_nprint( file_descriptor_t fd, const char *pvBuffer, const uint16_t xBytes );
void xCom_putChar(file_descriptor_t fd, unsigned char c);


#endif /* SRC_SPXR3_LIBS_L_PRINTF_H_ */

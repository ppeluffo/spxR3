/*
 * ee_sp5K.h
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#ifndef SRC_SPXR3_LIBS_L_EEPROM_H_
#define SRC_SPXR3_LIBS_L_EEPROM_H_

#include "frtos-io.h"
#include "stdint.h"
#include "../spxR3_libs/l_i2c.h"

//------------------------------------------------------------------------------------
// Identificacion en el bus I2C en el board sp6KX_LOGICA
// La memoria EE M24M02 es de 1024 paginas de 256 bytes o sea de 256Kbytes.
// Se accede con 16 bits ( 2 bytes de direcciones ) por lo que con los bit A16 y A17
// se indican en la direccion de la memoria en el bus I2C
// EEADDRESS = 0xA | 0 A17 A16 r/w
// Esto lo controlamos en I2C.C

//--------------------------------------------------------------------------------
// API START

#define EE_read( rdAddress, data, length ) I2C_read( BUSADDR_EEPROM_M2402, rdAddress, data, length );
#define EE_write( wrAddress, data, length ) I2C_write( BUSADDR_EEPROM_M2402, wrAddress, data, length );

// API END
//--------------------------------------------------------------------------------

#endif /* SRC_SPXR3_LIBS_L_EEPROM_H_ */

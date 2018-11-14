/*
 * l_i2c.h
 *
 *  Created on: 26 de mar. de 2018
 *      Author: pablo
 */

#ifndef SPX_LIBS_L_I2C_H_
#define SPX_LIBS_L_I2C_H_

#include "frtos-io.h"

#include "../spxR3_libs/l_printf.h"

#define BUSADDR_EEPROM_M2402	0xA0
#define BUSADDR_RTC_M79410		0xDE
#define INA0_DEVADDR			0x80
#define INA1_DEVADDR			0x82

int8_t I2C_read( uint8_t i2c_bus_address, uint32_t rdAddress, char *data, uint8_t length );
int8_t I2C_write( uint8_t i2c_bus_address, uint32_t wrAddress, char *data, uint8_t length );

#endif /* SPX_LIBS_L_I2C_H_ */

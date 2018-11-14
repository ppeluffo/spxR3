/*
 * l_ina3221.c
 *
 *  Created on: 13 de oct. de 2017
 *      Author: pablo
 */

#include "../spxR3_libs/l_ina3221.h"


//------------------------------------------------------------------------------------
uint8_t INA_id2busaddr( uint8_t id )
{
	switch(id) {
	case 0:
		return(INA3221_ADDR_0);	// Canales 0,1,2
		break;
	case 1:
		return(INA3221_ADDR_1); // Canales 3,4,5
		break;
	default:
		return(99);
		break;

	}

	return(99);

}
//------------------------------------------------------------------------------------

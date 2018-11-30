/*
 * l_ringBuffer.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */


#include "../spxR3_libs/l_ringBuffer.h"

//------------------------------------------------------------------------------------
ringBufferHandle_t ringBufferCreate ( const uint16_t length,int flags  )
{
ringBuffer_s *pxNewRingBuffer;
ringBufferHandle_t xReturn = NULL;
int8_t *pcAllocatedBuffer;
uint8_t *dataBuffer;

	// Aloco el espacio para el buffer de datos.
	dataBuffer = ( uint8_t * ) pvPortMalloc( length + 1);

	pcAllocatedBuffer = ( int8_t * ) pvPortMalloc( sizeof(ringBuffer_s ));
	if( pcAllocatedBuffer != NULL )
	{
		pxNewRingBuffer = ( ringBufferHandle_t * ) pcAllocatedBuffer;
		pxNewRingBuffer->head = 0;	// start
		pxNewRingBuffer->tail = 0;	// end
		pxNewRingBuffer->count = 0;
		pxNewRingBuffer->buff = dataBuffer;
		pxNewRingBuffer->length = length;		// REVISAR
		xReturn = pxNewRingBuffer;
	}

	return(xReturn);

}
//------------------------------------------------------------------------------------
void ringBufferFlush( ringBufferHandle_t ringBufferHandle )
{
ringBuffer_s *pxRingBuffer;

	taskENTER_CRITICAL();

	pxRingBuffer = ringBufferHandle;
	pxRingBuffer->head = 0;
	pxRingBuffer->tail = 0;
	pxRingBuffer->count = 0;
	memset(pxRingBuffer->buff,'\0', pxRingBuffer->length );	// REVISAR

	taskEXIT_CRITICAL();
}
//------------------------------------------------------------------------------------
bool ringBufferPoke( ringBufferHandle_t ringBufferHandle,const char *cChar )
{
	// Inserta un elemento en el ringBuffer.
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

ringBuffer_s *pxRingBuffer = ringBufferHandle;
bool ret = false;

	taskENTER_CRITICAL();
	// Si el buffer esta vacio ajusto los punteros
	if( pxRingBuffer->count == 0) {
		pxRingBuffer->head = pxRingBuffer->tail = 0;
	}

	if ( pxRingBuffer->count < pxRingBuffer->length ) {
		pxRingBuffer->buff[pxRingBuffer->head] = *cChar;
		++pxRingBuffer->count;
		// Avanzo en modo circular
		pxRingBuffer->head = ( pxRingBuffer->head  + 1 ) % ( pxRingBuffer->length );
		ret = true;
    }
	taskEXIT_CRITICAL();
	return(ret);

}
//------------------------------------------------------------------------------------
bool ringBufferPokeFromISR( ringBufferHandle_t ringBufferHandle,const char *cChar )
{
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

ringBuffer_s *pxRingBuffer = ringBufferHandle;
bool ret = false;

	// Si el buffer esta vacio ajusto los punteros !!!
	if( pxRingBuffer->count == 0) {
		pxRingBuffer->head = pxRingBuffer->tail = 0;
	}

	if ( pxRingBuffer->count < pxRingBuffer->length ) {
		pxRingBuffer->buff[pxRingBuffer->head] = *cChar;
		++pxRingBuffer->count;
		// Avanzo en modo circular
		pxRingBuffer->head = ( pxRingBuffer->head  + 1 ) % ( pxRingBuffer->length );
		ret = true;
    }
	return(ret);

}
//------------------------------------------------------------------------------------
bool ringBufferPop( ringBufferHandle_t ringBufferHandle, char *cChar )
{

ringBuffer_s *pxRingBuffer = ringBufferHandle;
bool ret = false;

	taskENTER_CRITICAL();
	//  Si el buffer esta vacio retorno.
	if( pxRingBuffer->count == 0) {
		pxRingBuffer->head = pxRingBuffer->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*cChar = pxRingBuffer->buff[pxRingBuffer->tail];
	--pxRingBuffer->count;
	// Avanzo en modo circular
	pxRingBuffer->tail = ( pxRingBuffer->tail  + 1 ) % ( pxRingBuffer->length );
	ret = true;
	taskEXIT_CRITICAL();

	return(ret);
}
//------------------------------------------------------------------------------------
bool ringBufferPopFromISR( ringBufferHandle_t ringBufferHandle, char *cChar )
{

ringBuffer_s *pxRingBuffer = ringBufferHandle;
bool ret = false;

	// Cannot block in an ISR

	//  Si el buffer esta vacio retorno.
	if( pxRingBuffer->count == 0) {
		pxRingBuffer->head = pxRingBuffer->tail = 0;
		return(ret);
	}

	*cChar = pxRingBuffer->buff[pxRingBuffer->tail];
	--pxRingBuffer->count;
	// Avanzo en modo circular
	pxRingBuffer->tail = ( pxRingBuffer->tail  + 1 ) % ( pxRingBuffer->length );
	ret = true;

	return(ret);
}
//------------------------------------------------------------------------------------
uint16_t ringBufferGetCount( ringBufferHandle_t ringBufferHandle )
{

ringBuffer_s *pxRingBuffer = ringBufferHandle;

	return(pxRingBuffer->count);

}
//------------------------------------------------------------------------------------
uint16_t ringBufferGetFreeCount( ringBufferHandle_t ringBufferHandle )
{

ringBuffer_s *pxRingBuffer = ringBufferHandle;

	return(pxRingBuffer->length - pxRingBuffer->count);

}
//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------
bool rBufferPoke( ringBuffer_s *rB, char *cChar )
{

bool ret = false;

	taskENTER_CRITICAL();

	// Si el buffer esta vacio ajusto los punteros
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
	}

	if ( rB->count < rB->length ) {
		rB->buff[rB->head] = *cChar;
		++rB->count;
		// Avanzo en modo circular
		rB->head = ( rB->head  + 1 ) % ( rB->length );
		ret = true;
    }

	taskEXIT_CRITICAL();
	return(ret);

}
//------------------------------------------------------------------------------------
bool rBufferPokeFromISR( ringBuffer_s *rB, char *cChar )
{

bool ret = false;

	// Si el buffer esta vacio ajusto los punteros
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
	}

	if ( rB->count < rB->length ) {
		rB->buff[rB->head] = *cChar;
		++rB->count;
		// Avanzo en modo circular
		rB->head = ( rB->head  + 1 ) % ( rB->length );
		ret = true;
    }

	// Indico que estan llegando datos
	rB->arriving = true;

	return(ret);

}
//------------------------------------------------------------------------------------
bool rBufferPop( ringBuffer_s *rB, char *cChar )
{

bool ret = false;

	// Voy a leer un dato. Si estan llegando, espero.
	if ( rB->arriving == true ) {
		rB->arriving = false;
		vTaskDelay( ( TickType_t)10 );
		// Si siguen llegando, me voy
		if ( rB->arriving == true ) {
			return(false);
		}
	}

	taskENTER_CRITICAL();

	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*cChar = rB->buff[rB->tail];
	--rB->count;
	// Avanzo en modo circular
	rB->tail = ( rB->tail  + 1 ) % ( rB->length );
	ret = true;

	taskEXIT_CRITICAL();
	return(ret);
}
//------------------------------------------------------------------------------------
bool rBufferPopFromISR( ringBuffer_s *rB, char *cChar )
{

bool ret = false;

	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		return(ret);
	}

	*cChar = rB->buff[rB->tail];
	--rB->count;
	// Avanzo en modo circular
	rB->tail = ( rB->tail  + 1 ) % ( rB->length );
	ret = true;

	return(ret);
}
//------------------------------------------------------------------------------------
void rBufferFlush( ringBuffer_s *rB )
{

	rB->head = 0;
	rB->tail = 0;
	rB->count = 0;
	memset(rB->buff,'\0', rB->length );
}
//------------------------------------------------------------------------------------
void rBufferCreateStatic ( ringBuffer_s *rB, uint8_t *storage_area, uint16_t size  )
{
	rB->buff = storage_area;
	rB->head = 0;	// start
	rB->tail = 0;	// end
	rB->count = 0;
	rB->length = size;
	rB->arriving = false;
}
//------------------------------------------------------------------------------------
uint16_t rBufferGetCount( ringBuffer_s *rB )
{

	return(rB->count);

}
//------------------------------------------------------------------------------------
uint16_t rBufferGetFreeCount( ringBuffer_s *rB )
{

	return(rB->length - rB->count);

}
//------------------------------------------------------------------------------------
bool rBufferReachLowWaterMark( ringBuffer_s *rB)
{
bool retS = false;

	if ( rB->count  < (int)(0.2 * rB->length  ))
		retS = true;
	return(retS);

}
//------------------------------------------------------------------------------------
bool rBufferReachHighWaterMark( ringBuffer_s *rB )
{
bool retS = false;

	if ( rB->count  > (int)(0.8 * rB->length ))
		retS = true;

	return(retS);

}
//------------------------------------------------------------------------------------


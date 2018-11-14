/*
 * l_ringBuffer.h
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#ifndef SRC_SPXR3_LIBS_L_RINGBUFFER_H_
#define SRC_SPXR3_LIBS_L_RINGBUFFER_H_


#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"

typedef void * ringBufferHandle_t;

typedef struct {
	uint8_t *buff;
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	uint16_t length;
	bool arriving;
} ringBuffer_s;

//------------------------------------------------------------------------------------

ringBufferHandle_t ringBufferCreate ( const uint16_t length, int flags );
bool ringBufferPoke( ringBufferHandle_t ringBufferHandle,const char *cChar );
bool ringBufferPokeFromISR( ringBufferHandle_t ringBufferHandle,const char *cChar );
bool ringBufferPop( ringBufferHandle_t ringBufferHandle,char *cChar );
bool ringBufferPopFromISR( ringBufferHandle_t ringBufferHandle, char *cChar );
void ringBufferFlush( ringBufferHandle_t ringBufferHandle );
uint16_t ringBufferGetFreeCount( ringBufferHandle_t ringBufferHandle );
uint16_t ringBufferGetCount( ringBufferHandle_t ringBufferHandle );


void rBufferCreateStatic ( ringBuffer_s *rB, uint8_t *storage_area, uint16_t size  );
bool rBufferPoke( ringBuffer_s *rB, char *cChar );
bool rBufferPokeFromISR( ringBuffer_s *rB, char *cChar );
bool rBufferPop( ringBuffer_s *rB, char *cChar );
bool rBufferPopFromISR( ringBuffer_s *rB, char *cChar );
void rBufferFlush( ringBuffer_s *rB );
uint16_t rBufferGetCount( ringBuffer_s *rB );
uint16_t rBufferGetFreeCount( ringBuffer_s *rB );
bool rBufferReachLowWaterMark( ringBuffer_s *rB);
bool rBufferReachHighWaterMark( ringBuffer_s *rB );

#endif /* SRC_SPXR3_LIBS_L_RINGBUFFER_H_ */

/*
 * l_nvm.c
 *
 *  Created on: 29 de set. de 2017
 *      Author: pablo
 */

#include "../spxR3_libs/l_nvm.h"

#include <stdio.h>

static void pv_NVMEE_FlushBuffer( void );
static void pv_NVMEE_WaitForNVM( void );

static uint8_t pv_ReadSignatureByte(uint16_t Address);
bool signature_ok = false;
char nvmid_str[32];

//------------------------------------------------------------------------------------
// NVM EEPROM
//------------------------------------------------------------------------------------
int NVMEE_write_buffer(eeprom_addr_t address, const void *buf, uint16_t len)
{
	// Escribe un buffer en la internal EEprom de a 1 byte. No considero paginacion.
	// Utiliza una funcion que hace erase&write.

int bytes2wr = 0;

	if ( address >= NVMEEPROM_SIZE) return(bytes2wr);

	taskENTER_CRITICAL();

	while (len > 0) {
		NVMEE_write_byte(address++, *(uint8_t*)buf);
        buf = (uint8_t*)buf + 1;
        len--;
        bytes2wr++;
    }

	taskEXIT_CRITICAL();

	return(bytes2wr);
}
//------------------------------------------------------------------------------------
void NVMEE_write_byte(eeprom_addr_t address, uint8_t value)
{
	// Escribe de a 1 byte en la internal EEprom

	if ( address >= NVMEEPROM_SIZE ) return;

    /*  Flush buffer to make sure no unintentional data is written and load
     *  the "Page Load" command into the command register.
     */
    pv_NVMEE_FlushBuffer();
    NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;

    // Set address to write to
    NVM.ADDR2 = 0x00;
    NVM.ADDR1 = (address >> 8) & 0xFF;
    NVM.ADDR0 = address & 0xFF;

	/* Load data to write, which triggers the loading of EEPROM page buffer. */
	NVM.DATA0 = value;

	/*  Issue EEPROM Atomic Write (Erase&Write) command. Load command, write
	 *  the protection signature and execute command.
	 */
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
	NVM_EXEC();

}
//------------------------------------------------------------------------------------
uint8_t NVMEE_ReadByte( eeprom_addr_t address )
{
	/* Wait until NVM is not busy. */
	pv_NVMEE_WaitForNVM();

	/* Set address to read from. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0xFF;
	NVM.ADDR2 = 0x00;

	/* Issue EEPROM Read command. */
	NVM.CMD = NVM_CMD_READ_EEPROM_gc;
	NVM_EXEC();

	return NVM.DATA0;
}
//------------------------------------------------------------------------------------
int NVMEE_read_buffer(eeprom_addr_t address, char *buf, uint16_t len)
{

uint8_t rb;
int bytes2rd = 0;

	if ( address >= NVMEEPROM_SIZE) return(bytes2rd);

	taskENTER_CRITICAL();

	while (len--) {
		rb = NVMEE_ReadByte(address++);
		*buf++ = rb;
		bytes2rd++;
    }

	taskEXIT_CRITICAL();

	return(bytes2rd);
}
//------------------------------------------------------------------------------------
void NVMEE_EraseAll( void )
{
	/* Wait until NVM is not busy. */
	pv_NVMEE_WaitForNVM();

	/* Issue EEPROM Erase All command. */
	NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
	NVM_EXEC();
}
//------------------------------------------------------------------------------------
static void pv_NVMEE_FlushBuffer( void )
{
	// Flushea el eeprom page buffer.

	/* Wait until NVM is not busy. */
	pv_NVMEE_WaitForNVM();

	/* Flush EEPROM page buffer if necessary. */
	if ((NVM.STATUS & NVM_EELOAD_bm) != 0) {
		NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
		NVM_EXEC();
	}
}
//------------------------------------------------------------------------------------
static void pv_NVMEE_WaitForNVM( void )
{
	/* Wait for any NVM access to finish, including EEPROM.
 	 *
 	 *  This function is blcoking and waits for any NVM access to finish,
 	 *  including EEPROM. Use this function before any EEPROM accesses,
 	 *  if you are not certain that any previous operations are finished yet,
 	 *  like an EEPROM write.
 	 */
	do {
		/* Block execution while waiting for the NVM to be ready. */
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}
//------------------------------------------------------------------------------------
// NVM ID
//------------------------------------------------------------------------------------
char *NVMEE_readID( void )
{

	if ( ! signature_ok ) {
		// Paso los bytes de identificacion a un string para su display.
		signature[ 0]=pv_ReadSignatureByte(LOTNUM0);
		signature[ 1]=pv_ReadSignatureByte(LOTNUM1);
		signature[ 2]=pv_ReadSignatureByte(LOTNUM2);
		signature[ 3]=pv_ReadSignatureByte(LOTNUM3);
		signature[ 4]=pv_ReadSignatureByte(LOTNUM4);
		signature[ 5]=pv_ReadSignatureByte(LOTNUM5);
		signature[ 6]=pv_ReadSignatureByte(WAFNUM );
		signature[ 7]=pv_ReadSignatureByte(COORDX0);
		signature[ 8]=pv_ReadSignatureByte(COORDX1);
		signature[ 9]=pv_ReadSignatureByte(COORDY0);
		signature[10]=pv_ReadSignatureByte(COORDY1);

		snprintf( nvmid_str, 32 ,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\0",signature[0],signature[1],signature[2],signature[3],signature[4],signature[5],signature[6],signature[7],signature[8],signature[9],signature[10]  );
		signature_ok = true;

	}

	return(&nvmid_str);
}
//------------------------------------------------------------------------------------
static uint8_t pv_ReadSignatureByte(uint16_t Address) {

	// Funcion que lee la memoria NVR, la calibration ROW de a una posicion ( de 16 bits )

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	//  __asm__ ("lpm \n  mov %0, r0 \n" : "=r" (Result) : "z" (Address) : "r0");
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}
//----------------------------------------------------------------------------------------
static inline void nvm_wait_until_ready( void )
{
	do {
		// Block execution while waiting for the NVM to be ready
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}
//----------------------------------------------------------------------------------------
static inline void nvm_issue_command(NVM_CMD_t nvm_command)
{
	uint8_t old_cmd;

	old_cmd = NVM.CMD;
	NVM.CMD = nvm_command;
	//ccp_write_io((uint8_t *)&NVM.CTRLA, NVM_CMDEX_bm);
	CCPWrite((uint8_t *)&NVM.CTRLA, NVM_CMDEX_bm);

	NVM.CMD = old_cmd;
}
//----------------------------------------------------------------------------------------
uint8_t nvm_fuses_read(uint8_t fuse)
{
	// Wait until NVM is ready
	nvm_wait_until_ready();

	// Set address
	NVM.ADDR0 = fuse;

	// Issue READ_FUSES command
	nvm_issue_command(NVM_CMD_READ_FUSES_gc);

	return NVM.DATA0;
}
//----------------------------------------------------------------------------------------



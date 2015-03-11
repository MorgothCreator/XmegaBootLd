//#######################################################################################
// Usart Driver
//
// File Name            :"IO_Usart.c"
// Location             :"Drivers/IO_Usart.c"
// Title                :Usart IO Driver
// Date                 :10.07.2011
// Version              :1.0
// Target MCU           :All ATXmega
// AUTHOR				:Iulian Gheorghiu
//			Romania
//			morgoth.creator@gmail.com
//			http://digitalelectronicsandprograming.blogspot.com/
//			http://morgothatxmegaprograming.blogspot.com/
//			http://devboardshop.servehttp.com
//
// DESCRIPTION:
//  This is a Hardware Usart driver
//	
//
//#######################################################################################
//#######################################################################################
//#######################################################################################
#ifndef __IO_EEP_Internal__
#define __IO_EEP_Internal__
//-----------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "IO_EEP_Internal.h"
//#######################################################################################
//#######################################################################################

/*
 * Please note that no other interrupts can have the highest
 * interrupt level than the EEPROM interrupt!
 */

/* Interrupt handler for the EEPROM write "done" interrupt */
//#######################################################################################
ISR(NVM_EE_vect)
{
	/* Disable the EEPROM interrupt */
	NVM.INTCTRL = (NVM.INTCTRL & ~NVM_EELVL_gm);
}
//#######################################################################################
/*@
Function for writing a page to EEPROM while putting the device to sleep
page: pointer to the first address of the page to be written
page_no: which page number to write to EEPROM.
*/
#if defined(_AVR_ATXMEGA8E5_H_INCLUDED) || defined(_AVR_ATXMEGA16E5_H_INCLUDED) || defined(_AVR_ATXMEGA32E5_H_INCLUDED)
void EEPROM_write_page (uint8_t * page, uint8_t page_no) {
	uint16_t ByteStart = page_no * EEPROM_PAGE_SIZE;
	for(int i = 0; i < EEPROM_PAGE_SIZE; i++)
	{
		eeprom_write_byte((uint8_t *)ByteStart + i, page[i]);
	}
}
#else
void EEPROM_write_page (uint8_t * page, uint8_t page_no)
{
	/* Set the NVM command to load to the EEPROM buffer */
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;
	
	/* Max buffersize is PAGESIZE, hence the upper addresses are not needed */
	NVM.ADDR2 = 0;
	NVM.ADDR1 = 0;

	/* Load buffer with data */
	for(int i = 0; i < EEPROM_PAGE_SIZE; i++)
	{
		/* Set the location where data is going to be put in the buffer */
		NVM.ADDR0 = i;
		/* Writing to the DATA register will trigger a write to the buffer */
		NVM.DATA0 = page[i];
	}
	
	/* Load the NVM command with erase and then write EEPROM page */
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;

	/* Calculate address */
	uint16_t address = (uint16_t)(page_no*EEPROM_PAGE_SIZE);

	/* Set address to write to. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0x00;

	/* Save the Sleep register */
	uint8_t sleepCtr = SLEEP.CTRL;
	/* Set sleep mode to IDLE */
	SLEEP.CTRL = (SLEEP.CTRL & ~SLEEP.CTRL) | SLEEP_SMODE_IDLE_gc;
	/* Save the PMIC Status and control registers */
	uint8_t statusStore = PMIC.STATUS;
	uint8_t pmicStore = PMIC.CTRL;
	
	/* Enable only the highest level of interrupts */
	PMIC.CTRL = (PMIC.CTRL & ~(PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm)) | PMIC_HILVLEN_bm;
	/* Save SREG for later use */
	uint8_t globalInt = SREG;
	/* Enable global interrupts */
	sei();
	/* Set sleep enabled */
	SLEEP.CTRL |= SLEEP_SEN_bm;
	/* Save eeprom interrupt settings for later */
	uint8_t eepromintStore = NVM.INTCTRL;
	
	
	/* Write the "safety code" to the CCP regiter */
	/* EEPROM write has to be executed within 4 cycles */
	CCP = CCP_IOREG_gc;
	/* Execute command to write buffer page to EEPROM */
	NVM.CTRLA = NVM_CMDEX_bm;
	/* Enable EEPROM interrupt */
	NVM.INTCTRL =  NVM_EELVL0_bm | NVM_EELVL1_bm;
	/* Sleep before 2.5uS has passed */
	//__sleep();
	asm("sleep");
	
	
	/* Restore sleep settings */
	SLEEP.CTRL = sleepCtr;
	/* Restore PMIC status and control registers */
	PMIC.STATUS = statusStore;
	PMIC.CTRL = pmicStore;
	/* Restore EEPROM interruptsettings */
	NVM.INTCTRL = eepromintStore;
	/* Restore global interrupt settings */
	SREG = globalInt;
}
#endif
//#######################################################################################
void EEPROM_WaitForNVM( void )
{
	do {
		/* Block execution while waiting for the NVM to be ready. */
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}
//#######################################################################################
#if defined(_AVR_ATXMEGA8E5_H_INCLUDED) || defined(_AVR_ATXMEGA16E5_H_INCLUDED) || defined(_AVR_ATXMEGA32E5_H_INCLUDED)
uint8_t EEPROM_ReadByte( uint8_t pageAddr, uint8_t byteAddr ) {
	uint16_t ByteStart = pageAddr * EEPROM_PAGE_SIZE;
	return eeprom_read_byte((const uint8_t *)ByteStart + byteAddr);
}
#else
uint8_t EEPROM_ReadByte( uint8_t pageAddr, uint8_t byteAddr )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();
	/* Calculate address */
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGE_SIZE)
	|(byteAddr & (EEPROM_PAGE_SIZE-1));
	/* Set address to read from. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0x00;
	/* Issue EEPROM Read command. */
	NVM.CMD = NVM_CMD_READ_EEPROM_gc;
	NVM_EXEC();
	return NVM.DATA0;
}
#endif






#endif

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
#ifndef __IO_Flash_Internal__
#define __IO_Flash_Internal__
//-----------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include "IO_Flash_Internal.h"
//#######################################################################################

/* Temporary storage used for NVM-workaround. */
uint8_t sleepCtr;
uint8_t statusStore;
uint8_t pmicStore;
uint8_t globalInt;
uint8_t spmintStore;

/* SPM wakeup interrupt */
ISR(NVM_SPM_vect)
{
	/* Disable the SPM interrupt */
	NVM.INTCTRL = (NVM.INTCTRL & ~NVM_SPMLVL_gm);
	/* Restore sleep settings */
	SLEEP.CTRL = sleepCtr;
	/* Restore PMIC status and control registers */
	PMIC.STATUS = statusStore;
	PMIC.CTRL = pmicStore;
	/* Restore SPM interruptsettings */
	NVM.INTCTRL = spmintStore;
	/* Restore global interrupt settings */
	SREG = globalInt;
}

/* Set interrupt vector location to boot section of flash */
void PMIC_SetVectorLocationToBoot( void )
{
	uint8_t temp = PMIC.CTRL | PMIC_IVSEL_bm;
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = temp;
}

/*Set interrupt vector location to application section of flash */
void PMIC_SetVectorLocationToApplication( void )
{
	uint8_t temp = PMIC.CTRL & ~PMIC_IVSEL_bm;
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = temp;
}

/* Save register settings before entering sleep mode */
void Prepare_to_Sleep( void )
{
	sleepCtr = SLEEP.CTRL;
	/* Set sleep mode to IDLE */
	SLEEP.CTRL =  0x00; 
	/* Save the PMIC Status and control registers */
	statusStore = PMIC.STATUS;								
	pmicStore = PMIC.CTRL;		
	/* Enable only the highest level of interrupts */									
	PMIC.CTRL = (PMIC.CTRL & ~(PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm)) | PMIC_HILVLEN_bm;
	/* Save SREG for later use */
	globalInt = SREG;
	/* Enable global interrupts */
	asm("sei");
	/* Save SPM interrupt settings for later */ 
	spmintStore = NVM.INTCTRL;
}

/* New function declarations used for the NVM-workaround */
void EraseApplicationPage(uint32_t address)
{
	/*Set the correct settings and store critical registers before NVM-workaround*/
	Prepare_to_Sleep();
	/*Assembly "function" to preform page erase*/
	SP_EraseApplicationPage(address);
}

void EraseWriteApplicationPage(uint32_t address)
{
	/*Set the correct settings and store critical registers before NVM-workaround*/
	Prepare_to_Sleep(); 
	/*Assembly "function" to preform page erase-write*/
	SP_EraseWriteApplicationPage(address);
}

void ClearFlashBuffer(void)
{
	/*Set the correct settings and store critical registers before NVM-workaround*/
	Prepare_to_Sleep(); 
	/*Assembly "function" to erase flash buffer*/
	SP_EraseFlashBuffer();
}

void LoadFlashWord(uint32_t address, uint16_t word)
{
	/*Set the correct settings and store critical registers before NVM-workaround*/
	Prepare_to_Sleep();   
	/*Assembly "function" to load flash buffer*/
   	SP_LoadFlashWord(address, word);
}




























#endif
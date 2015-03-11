#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
//#include <util/delay.h>
//#define sei( ) (__enable_interrupt( ))
//#define cli( ) (__disable_interrupt( ))
#define nop( ) (__no_operation())
//#######################################################################################
#define NVM_EXEC()	asm("push r30"      "\n\t"	\
			    "push r31"      "\n\t"	\
    			    "push r16"      "\n\t"	\
    			    "push r18"      "\n\t"	\
			    "ldi r30, 0xCB" "\n\t"	\
			    "ldi r31, 0x01" "\n\t"	\
			    "ldi r16, 0xD8" "\n\t"	\
			    "ldi r18, 0x01" "\n\t"	\
			    "out 0x34, r16" "\n\t"	\
			    "st Z, r18"	    "\n\t"	\
    			    "pop r18"       "\n\t"	\
			    "pop r16"       "\n\t"	\
			    "pop r31"       "\n\t"	\
			    "pop r30"       "\n\t"	\
			    )
//#######################################################################################
void EEPROM_write_page (uint8_t * page, uint8_t page_no);
uint8_t EEPROM_ReadByte( uint8_t pageAddr, uint8_t byteAddr );
//#######################################################################################

/*
 * Xmega_BootLoader.c
 *
 * Created: 8/13/2011 4:42:34 PM
 *  Author: Morgoth
 */ 
//#pragma orgall 0x010000

// -combine
//sp_driver.s
#define F_CPU							(32000000UL)
#define _BaudRate						(115200)
#define UsartNr							(0)
#define HexBufferLenght					(100)//43 Generic
#define BinBufferLength					(HexBufferLenght/2)
#define TimeToWaitEnterInBootLoader		(1)
//-----------------------------------------------------
//Intel hex definitions
#define Data_Record						0//Contains data and 16-bit address. The format described above.
#define EndOfFile_Record				1//A file termination record. No data. Has to be the last line of the file, only one per file permitted. Usually ':00000001FF'. Originally the End Of File record could contain a start address for the program being loaded, e.g. :00AB2F0125 would make a jump to address AB2F. This was convenient when programs were loaded from punched paper tape.
#define ExtendedSegmentAddress_Record	2//Segment-base address. Used when 16 bits are not enough, identical to 80x86 real mode addressing. The address specified by the 02 record is multiplied by 16 (shifted 4 bits left) and added to the subsequent 00 record addresses. This allows addressing of up to a megabyte of address space. The address field of this record has to be 0000, the byte count is 02 (the segment is 16-bit). The least significant hex digit of the segment address is always 0.
#define StartSegmentAddress_Record		3//For 80x86 processors, it specifies the initial content of the CS:IP registers. The address field is 0000, the byte count is 04, the first two bytes are the CS value, the latter two are the IP value.
#define ExtendedLinearAddress_Record	4//Allowing for fully 32 bit addressing. The address field is 0000, the byte count is 02. The two data bytes represent the upper 16 bits of the 32 bit address, when combined with the address of the 00 type record.
#define StartLinearAddress_Record		5//The address field is 0000, the byte count is 04. The 4 data bytes represent the 32-bit value loaded into the EIP register of the 80386 and higher CPU.
//-----------------------------------------------------
//Errors 
#define Error_LineDefError				'a'
#define Error_SecondHexCharNotFound		'b'
#define Error_CheckSum					'c'
#define Error_LineMismach				'd'
#define Error_NoMemorySelected			'e'
#define Error_NoError					'k'
//-----------------------------------------------------
#define EnterToFlashWrite				(1)
#define EnterToEepWrite					(2)
#define EnterInUndefinedWrite			(255)
//-----------------------------------------------------
#include <util/delay.h>
#include <avr/io.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "string.h"
#include "Unions.h"
#include "IO_EEP_Internal.h"
#include "IO_Flash_Internal.h"
//-----------------------------------------------------
#if APP_SECTION_PAGE_SIZE == 32
	#define FlashPageMask	(5)
#elif APP_SECTION_PAGE_SIZE == 64
	#define FlashPageMask	(6)
#elif APP_SECTION_PAGE_SIZE == 128
	#define FlashPageMask	(7)
#elif APP_SECTION_PAGE_SIZE == 256
	#define FlashPageMask	(8)
#elif APP_SECTION_PAGE_SIZE == 512
	#define FlashPageMask	(9)
#elif APP_SECTION_PAGE_SIZE == 1024
	#define FlashPageMask	(10)
#elif APP_SECTION_PAGE_SIZE == 2040
	#define FlashPageMask	(11)
#endif
//-----------------------------------------------------
#if EEPROM_PAGE_SIZE == 32
	#define EEpromPageMask	(5)
#elif EEPROM_PAGE_SIZE == 64
	#define EEpromPageMask	(6)
#elif EEPROM_PAGE_SIZE == 128
	#define EEpromPageMask	(7)
#elif EEPROM_PAGE_SIZE == 256
	#define EEpromPageMask	(8)
#elif EEPROM_PAGE_SIZE == 512
	#define EEpromPageMask	(9)
#elif EEPROM_PAGE_SIZE == 1024
	#define EEpromPageMask	(10)
#elif EEPROM_PAGE_SIZE == 2040
	#define EEpromPageMask	(11)
#endif
//-----------------------------------------------------
#ifndef False
#define False	0
#endif

#ifndef True
#define True	1
#endif

#ifndef Null
#define Null	-1
#endif
//-Wl,-Ttext,0x010000
//#####################################################
uint8_t FlashPageBuffer[APP_SECTION_PAGE_SIZE];
char RxHexBuffer[HexBufferLenght];
uint8_t RxBinBuffer[BinBufferLength];
uint32_t ExtendedSegmentAddressRecord;
uint32_t RegPageInBuffer;
uint8_t RegFlashEEPromWriteMode;

uint8_t CountProcessedBytes = 0;
uint8_t CountToExtractBytesFromBinBuffer = 0;
uint8_t CheckSum = 0;
//#####################################################
void usart_init(void)
{
#if UsartNr == 0
	PORTC.DIRSET = 1<<3;
	PORTC.OUTSET = 1<<3;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTC0.BAUDCTRLA = BaudRate.Byte0;
	USARTC0.BAUDCTRLB = BaudRate.Byte1;
	USARTC0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTC0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#elif UsartNr == 1
	PORTC.DIRSET = 1<<7;
	PORTC.OUTSET = 1<<7;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTC1.BAUDCTRLA = BaudRate.Byte0;
	USARTC1.BAUDCTRLB = BaudRate.Byte1;
	USARTC1.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTC1.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTC1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#elif UsartNr == 2
	PORTD.DIRSET = 1<<3;
	PORTD.OUTSET = 1<<3;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTD0.BAUDCTRLA = BaudRate.Byte0;
	USARTD0.BAUDCTRLB = BaudRate.Byte1;
	USARTD0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTD0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#elif UsartNr == 3
	PORTD.DIRSET = 1<<7;
	PORTD.OUTSET = 1<<7;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTD1.BAUDCTRLA = BaudRate.Byte0;
	USARTD1.BAUDCTRLB = BaudRate.Byte1;
	USARTD1.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTD1.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTD1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#elif UsartNr == 4
	PORTE.DIRSET = 1<<3;
	PORTE.OUTSET = 1<<3;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTE0.BAUDCTRLA = BaudRate.Byte0;
	USARTE0.BAUDCTRLB = BaudRate.Byte1;
	USARTE0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTE0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTE0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#elif UsartNr == 5
	PORTE.DIRSET = 1<<7;
	PORTE.OUTSET = 1<<7;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTE1.BAUDCTRLA = BaudRate.Byte0;
	USARTE1.BAUDCTRLB = BaudRate.Byte1;
	USARTE1.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTE1.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTE1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#elif UsartNr == 6
	PORTF.DIRSET = 1<<3;
	PORTF.OUTSET = 1<<3;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTF0.BAUDCTRLA = BaudRate.Byte0;
	USARTF0.BAUDCTRLB = BaudRate.Byte1;
	USARTF0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTF0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTF0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#elif UsartNr == 7
	PORTF.DIRSET = 1<<7;
	PORTF.OUTSET = 1<<7;
	convert16to8 BaudRate;
	BaudRate.i16 = (F_CPU/8/_BaudRate)-1;
	USARTF1.BAUDCTRLA = BaudRate.Byte0;
	USARTF1.BAUDCTRLB = BaudRate.Byte1;
	USARTF1.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTF1.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm;
	USARTF1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
#endif
}
//#####################################################
inline uint8_t usart_check_send_busy(void)
{
#if UsartNr == 0
	if(USARTC0.STATUS&USART_DREIF_bm) return False;
		else return True;
#elif UsartNr == 1
	if(USARTC1.STATUS&USART_DREIF_bm) return False;
		else return True;
#elif UsartNr == 2
	if(USARTD0.STATUS&USART_DREIF_bm) return False;
		else return True;
#elif UsartNr == 3
	if(USARTD1.STATUS&USART_DREIF_bm) return False;
		else return True;
#elif UsartNr == 4
	if(USARTE0.STATUS&USART_DREIF_bm) return False;
		else return True;
#elif UsartNr == 5
	if(USARTE1.STATUS&USART_DREIF_bm) return False;
		else return True;
#elif UsartNr == 6
	if(USARTF0.STATUS&USART_DREIF_bm) return False;
		else return True;
#elif UsartNr == 7
	if(USARTF1.STATUS&USART_DREIF_bm) return False;
		else return True;
#endif
}
//#####################################################
inline uint8_t usart_check_rec_data(void)
{
#if UsartNr == 0
	if(USARTC0.STATUS&USART_RXCIF_bm) return True;
		else return False;
#elif UsartNr == 1
	if(USARTC1.STATUS&USART_RXCIF_bm) return True;
		else return False;
#elif UsartNr == 2
	if(USARTD0.STATUS&USART_RXCIF_bm) return True;
		else return False;
#elif UsartNr == 3
	if(USARTD1.STATUS&USART_RXCIF_bm) return True;
		else return False;
#elif UsartNr == 4
	if(USARTE0.STATUS&USART_RXCIF_bm) return True;
		else return False;
#elif UsartNr == 5
	if(USARTE1.STATUS&USART_RXCIF_bm) return True;
		else return False;
#elif UsartNr == 6
	if(USARTF0.STATUS&USART_RXCIF_bm) return True;
		else return False;
#elif UsartNr == 7
	if(USARTF1.STATUS&USART_RXCIF_bm) return True;
		else return False;
#endif
}
//#####################################################
void usart_putc(char Chr)
{
	while(usart_check_send_busy());
#if UsartNr == 0
	USARTC0.DATA = Chr;
#elif UsartNr == 1
	USARTC1.DATA = Chr;
#elif UsartNr == 2
	USARTD0.DATA = Chr;
#elif UsartNr == 3
	USARTD1.DATA = Chr;
#elif UsartNr == 4
	USARTE0.DATA = Chr;
#elif UsartNr == 5
	USARTE1.DATA = Chr;
#elif UsartNr == 6
	USARTF0.DATA = Chr;
#elif UsartNr == 7
	USARTF1.DATA = Chr;
#endif
}
//#####################################################
char usart_getc(void)
{
#if UsartNr == 0
	return USARTC0.DATA;
#elif UsartNr == 1
	return USARTC1.DATA;
#elif UsartNr == 2
	return USARTD0.DATA;
#elif UsartNr == 3
	return USARTD1.DATA;
#elif UsartNr == 4
	return USARTE0.DATA;
#elif UsartNr == 5
	return USARTE1.DATA;
#elif UsartNr == 6
	return USARTF0.DATA;
#elif UsartNr == 7
	return USARTF1.DATA;
#endif
}
//#####################################################
void clear_buff(void)
{
	for(uint16_t ClearCount = 0; ClearCount < HexBufferLenght; ClearCount++)
	{
		RxHexBuffer[ClearCount] = 0x00;
	}
}
//#####################################################
void read_eeprom_page_in_buffer()
{
	uint16_t ByteStart = RegPageInBuffer * EEPROM_PAGE_SIZE;
	for (uint8_t EEpReadPageCount = 0; EEpReadPageCount < EEPROM_PAGE_SIZE; EEpReadPageCount++)
	{
		FlashPageBuffer[EEpReadPageCount] = EEPROM_ReadByte(ByteStart, EEpReadPageCount);
	}
}
//#####################################################
void read_flash_page_in_buffer()
{
	uint32_t ByteStart = RegPageInBuffer * APP_SECTION_PAGE_SIZE;
	for(uint16_t ReadCount = 0; ReadCount < APP_SECTION_PAGE_SIZE; ReadCount++)
	{
		FlashPageBuffer[ReadCount] = SP_ReadByte(ByteStart++);
	}
}
//#####################################################
void write_flash_page_from_buffer()
{
	uint32_t ByteStart = RegPageInBuffer * APP_SECTION_PAGE_SIZE;
	convert8to16 Union16;
	PMIC_SetVectorLocationToBoot();
	ClearFlashBuffer(); //Clear the flash buffer first to avoid data corruption
	SP_WaitForSPM();
	for (uint16_t WriteCount = 0; WriteCount < APP_SECTION_PAGE_SIZE; WriteCount += 2) 
	{
		Union16.Byte0 = FlashPageBuffer[WriteCount];
		Union16.Byte1 = FlashPageBuffer[WriteCount+1];
		LoadFlashWord((uint32_t)WriteCount, Union16.ShortReturn);
		SP_WaitForSPM();
	}
	EraseWriteApplicationPage(ByteStart);
	SP_WaitForSPM();
}
//#####################################################
uint8_t strcmp_enter_to_boot(void)
{
	if(RxHexBuffer[0] != 'B') return False;
	if(RxHexBuffer[1] != 'o') return False;
	if(RxHexBuffer[2] != 'o') return False;
	if(RxHexBuffer[3] != 't') return False;
	if(RxHexBuffer[4] != 'I') return False;
	if(RxHexBuffer[5] != 'n') return False;
	if(RxHexBuffer[6] != 'i') return False;
	if(RxHexBuffer[7] != 't') return False;
	return True;
}
//#####################################################
uint8_t strcmp_enter_to_flash(void)
{
	if(RxHexBuffer[0] != 'F') return False;
	if(RxHexBuffer[1] != 'l') return False;
	if(RxHexBuffer[2] != 'a') return False;
	if(RxHexBuffer[3] != 's') return False;
	if(RxHexBuffer[4] != 'h') return False;
	if(RxHexBuffer[5] != 'W') return False;
	return True;
}
//#####################################################
uint8_t strcmp_enter_to_eep(void)
{
	if(RxHexBuffer[0] != 'E') return False;
	if(RxHexBuffer[1] != 'E') return False;
	if(RxHexBuffer[2] != 'P') return False;
	if(RxHexBuffer[3] != 'r') return False;
	if(RxHexBuffer[4] != 'o') return False;
	if(RxHexBuffer[5] != 'm') return False;
	if(RxHexBuffer[6] != 'W') return False;
	return True;
}
//#####################################################
uint8_t strcmp_enter_to_exit(void)
{
	if(RxHexBuffer[0] != 'E') return False;
	if(RxHexBuffer[1] != 'x') return False;
	if(RxHexBuffer[2] != 'i') return False;
	if(RxHexBuffer[3] != 't') return False;
	return True;
}
//#####################################################
uint8_t receive_data()
{
	uint32_t TimeCount = 0;
	uint8_t CharCount = 0;
	char Char;
	do 
	{
		TimeCount++;
		if(usart_check_rec_data())
		{
			TimeCount = 0;
			Char = usart_getc();
			if(Char == 13) 
			{
				RxHexBuffer[CharCount] = 0;
				break;
			}
			RxHexBuffer[CharCount] = Char;
			CharCount++;
		}
		_delay_us(10);
	} while (TimeCount != (TimeToWaitEnterInBootLoader*100000));
	return CharCount;
}
//#####################################################
uint8_t check_if_is_hex_char_and_convert(char Ch)
{
	uint8_t Tmp = (uint8_t)Ch - '0';
	if(Tmp < (':'-'0')) return Tmp;
	Tmp -= ('@'-'9');
	if(Tmp < 16) return Tmp;
	return 255;
}
//#####################################################
typedef struct 
{
	uint8_t nr_of_bytes;
	uint8_t error;
} hex_to_bin_return;
//#####################################################
hex_to_bin_return hex_to_bin()
{
	uint8_t ConvertedByte = 0;
	uint8_t Tmp = 0;
	uint8_t BytesCount = 0;
	hex_to_bin_return Return;
	for (uint8_t ConvertCount = 1; ConvertCount < strlen(RxHexBuffer); ConvertCount += 2)
	{
		ConvertedByte = check_if_is_hex_char_and_convert(RxHexBuffer[ConvertCount])<<4;
		Tmp = check_if_is_hex_char_and_convert(RxHexBuffer[ConvertCount+1]);
		if(Tmp == 255) 
		{
			Return.error = Error_SecondHexCharNotFound;
			return Return;
		}
		ConvertedByte |= Tmp;
		RxBinBuffer[BytesCount] = ConvertedByte;
		BytesCount++;
	}
	Return.error = Error_NoError;
	Return.nr_of_bytes = BytesCount;
	return Return;
}
//#####################################################
uint8_t check_sum()
{
	if(CountProcessedBytes != 0)
	{
		do
		{
			CheckSum += RxBinBuffer[CountToExtractBytesFromBinBuffer++];
		}while(--CountProcessedBytes);						
	}
	if(RxBinBuffer[CountToExtractBytesFromBinBuffer] == (uint8_t)(0-CheckSum)) return Error_NoError;
	return Error_CheckSum;
}
//#####################################################
void append_line_in_eeprom_buffer(uint32_t Addr)
{
	if(CountProcessedBytes)
	{
		uint32_t PageAddr = 0;
		uint8_t AppendCount = 0;
		uint8_t Byte = 0;
		do 
		{
			PageAddr = ((Addr + AppendCount)>>EEpromPageMask);
			if(PageAddr != RegPageInBuffer)
			{
				EEPROM_write_page(FlashPageBuffer, RegPageInBuffer);
				RegPageInBuffer = PageAddr;
				read_eeprom_page_in_buffer();
			}
			Byte = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
			CheckSum += Byte;
			FlashPageBuffer[(Addr + AppendCount) & (EEPROM_PAGE_SIZE - 1)] = Byte;
			AppendCount++;
		} while (--CountProcessedBytes);
	}	
}
//#####################################################
void append_line_in_flash_buffer(uint32_t Addr)
{
	if(CountProcessedBytes)
	{
		uint32_t PageAddr = 0;
		uint8_t AppendCount = 0;
		uint8_t Byte = 0;
		do 
		{
			PageAddr = ((Addr + AppendCount)>>FlashPageMask);
			if(PageAddr != RegPageInBuffer)
			{
				write_flash_page_from_buffer();
				RegPageInBuffer = PageAddr;
				read_flash_page_in_buffer();
			}
			Byte = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
			CheckSum += Byte;
			FlashPageBuffer[(Addr + AppendCount) & (APP_SECTION_PAGE_SIZE - 1)] = Byte;
			AppendCount++;
		} while (--CountProcessedBytes);
	}		
}
//#####################################################
int main(void)
{
#if defined(_AVR_ATXMEGA8E5_H_INCLUDED) || defined(_AVR_ATXMEGA16E5_H_INCLUDED) || defined(_AVR_ATXMEGA32E5_H_INCLUDED)
#else
  	OSC_XOSCCTRL =		OSC_XOSCSEL1_bm;// | OSC_X32KLPM_bm
  	//Setup DFLL
  	OSC_CTRL = OSC_XOSCEN_bm;
  	OSC_DFLLCTRL = OSC_RC2MCREF_bm;
  	DFLLRC2M_CTRL = DFLL_ENABLE_bm;
  	//Setup PLL
  	OSC_PLLCTRL = (OSC_PLLSRC_RC2M_gc) | (F_CPU/2000000);
  	//Set osc
  	OSC_CTRL = OSC_XOSCEN_bm|OSC_PLLEN_bm;
  	CPU_CCP = CCP_IOREG_gc;
  	// //Security Signature to modify clock
  	CLK_PSCTRL = (CLK_PSADIV_1_gc) | (CLK_PSBCDIV_1_1_gc);
  	while((OSC_STATUS & OSC_PLLRDY_bm) == 0);
  	//Select system clock source PLL
  	CPU_CCP = CCP_IOREG_gc;
  	// //Security Signature to modify clock
  	CLK_CTRL = CLK_SCLKSEL_PLL_gc;
  	PMIC_CTRL = PMIC_HILVLEN_bm|PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
#endif

	uint8_t ReceivedCharNr = 0;
	hex_to_bin_return Return;
	uint8_t DataBytesNrInLine = 0;
	uint8_t AddressByte0 = 0;
	uint8_t AddressByte1 = 0;
	uint8_t LineFunction = 0;
	uint32_t WriteAddr = 0;
	convert8to32 Union32;

	CCP = CCP_IOREG_gc;
	PMIC.CTRL = PMIC_IVSEL_bm;
	usart_init();
	clear_buff();
	receive_data();
	if(strcmp_enter_to_boot());
		else 
		{
			CCP = CCP_IOREG_gc;
			PMIC.CTRL = 0;
			asm("JMP 0x0000");
		}
	RegFlashEEPromWriteMode = EnterInUndefinedWrite;
	do
	{
		clear_buff();
		usart_putc('k');
		ReceivedCharNr = receive_data();
		if(strcmp_enter_to_exit())
		{
			break;
		}			
		else if(strcmp_enter_to_flash()) 
		{
			RegFlashEEPromWriteMode = EnterToFlashWrite;
			RegPageInBuffer = 0;
			read_flash_page_in_buffer();
		}
		else if(strcmp_enter_to_eep()) 
		{
			RegFlashEEPromWriteMode = EnterToEepWrite;
			RegPageInBuffer = 0;
			read_eeprom_page_in_buffer();
		}
		else
		{
			CountToExtractBytesFromBinBuffer = 0;
			Return = hex_to_bin();
			if(Return.error == Error_SecondHexCharNotFound)
			{
				usart_putc(Error_SecondHexCharNotFound);
				break;
			}
			//Verify integrity of bin line
			DataBytesNrInLine = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
			CountProcessedBytes = Return.nr_of_bytes - 5;
			if(CountProcessedBytes != DataBytesNrInLine)
			{
				usart_putc(Error_LineMismach);
				break;
			}
			AddressByte1 = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
			AddressByte0 = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
			LineFunction = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
			CheckSum = DataBytesNrInLine;
			CheckSum += AddressByte1;
			CheckSum += AddressByte0;
			CheckSum += LineFunction;
			if(LineFunction == Data_Record)
			{
				Union32.Byte0 = AddressByte0;
				Union32.Byte1 = AddressByte1;
				Union32.Byte2 = 0;
				Union32.Byte3 = 0;
				WriteAddr = /*ExtendedSegmentAddressRecord + */Union32.LongReturn;
				if(RegFlashEEPromWriteMode == EnterToFlashWrite)
				{
					append_line_in_flash_buffer(WriteAddr);
					uint8_t Tmp = check_sum();
					if(Tmp == Error_CheckSum) 
					{
						usart_putc(Error_CheckSum);
						break;
					}
				}
				else if(RegFlashEEPromWriteMode == EnterToEepWrite)
				{
					append_line_in_eeprom_buffer(WriteAddr);
					uint8_t Tmp = check_sum();
					if(Tmp == Error_CheckSum) 
					{
						usart_putc(Error_CheckSum);
						break;
					}						
				}
				else 
				{
					usart_putc(Error_NoMemorySelected);
					break;
				}					
			}
			else if(LineFunction == EndOfFile_Record)
			{
				if(RegFlashEEPromWriteMode == EnterToFlashWrite)
				{
					write_flash_page_from_buffer();
				}
				else if(RegFlashEEPromWriteMode == EnterToEepWrite)	
				{
					EEPROM_write_page(FlashPageBuffer, RegPageInBuffer);
				}				
				uint8_t Tmp = check_sum();
				if(Tmp == Error_CheckSum) 
				{
					usart_putc(Error_CheckSum);
					break;
				}						
			}
			else if(LineFunction == ExtendedSegmentAddress_Record)
			{
				uint8_t Tmp1 = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
				uint8_t Tmp0 = RxBinBuffer[CountToExtractBytesFromBinBuffer++];
				CheckSum += Tmp0;
				CheckSum += Tmp1;
				CountProcessedBytes -= 2;
				Union32.Byte0 = Tmp0;
				Union32.Byte1 = Tmp1;
				Union32.Byte2 = 0;
				Union32.Byte3 = 0;
				ExtendedSegmentAddressRecord = Union32.LongReturn<<4;
			}
			else usart_putc(Error_LineDefError);

		}
	} while(True);
	//write_page_from_buffer();
	usart_putc('f');
	_delay_ms(50);
	CCP = CCP_IOREG_gc;
	RST.CTRL = RST_SWRST_bm;
}
#ifndef ___IO_Flash_Internal__
#define ___IO_Flash_Internal__

#include "sp_driver.h"


void PMIC_SetVectorLocationToBoot( void );
void PMIC_SetVectorLocationToApplication( void );
void Prepare_to_Sleep( void );
void EraseApplicationPage(uint32_t address);
void EraseWriteApplicationPage(uint32_t address);
void ClearFlashBuffer(void);
void LoadFlashWord(uint32_t address, uint16_t word);

#endif
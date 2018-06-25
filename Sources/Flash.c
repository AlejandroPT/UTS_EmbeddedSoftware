/*
 * Flash.c
 *
 *  @brief Routines to implement a Flash HAL
 *
 *  This contains the implementation for modifying (read/write) flash memory
 *  Created on: 11 Apr 2018
 *      Author: 13113117, 11989668
 */

#include "Flash.h"
#define FLASH_CMD_ERASE_SECTOR 0x09LU //Flash command for erasing a sector
#define FLASH_CMD_PROGRAM_PHRASE 0x07LU //Flash command for erasing a sector

static uint8_t AllocationMap[FLASH_SIZE] = {0}; //Map of the allocation space, initialiazed to 0

//Private functions

/*! @brief Waits for the CCIF flag.
 *
 */
static void WaitCCIF()
{
  while (!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK));
}

/*! @brief Sets the CCIF flag.
 *
 */
static void SetCCIF()
{
  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;
  WaitCCIF();
}

/*! @brief Reads a phrase from flash memory.
 *
 *  @param phrase The address of where to store the read phrase.
 *  @return bool - TRUE if the phrase was read successfully.
 */
static bool ReadPhrase(uint64_t *const phrase)
{
  WaitCCIF();
  *phrase = _FP(FLASH_DATA_START);
}



/*! @brief Writes a phrase from flash memory.
 *
 *  @param phrase The phrase to be written
 *  @return bool - TRUE if the phrase was written successfully.
 */
static bool WritePhrase(const uint64_t phrase)
{
  WaitCCIF();

  if (!Flash_Erase())
    return false;

  uint32_8union_t flashStart;
  flashStart.l = FLASH_DATA_START;

  FTFE_FCCOB0 = FLASH_CMD_PROGRAM_PHRASE;  //Command to write
  FTFE_FCCOB1 = flashStart.s.Byte2;      //sets the flash address of the correct sector
  FTFE_FCCOB2 = flashStart.s.Byte3;
  FTFE_FCCOB3 = flashStart.s.Byte4 & 0xF8;

  uint8_t *data = (uint8_t *) &phrase;

  FTFE_FCCOB4 = data[3];
  FTFE_FCCOB5 = data[2];
  FTFE_FCCOB6 = data[1];
  FTFE_FCCOB7 = data[0];
  FTFE_FCCOB8 = data[7];
  FTFE_FCCOB9 = data[6];
  FTFE_FCCOBA = data[5];
  FTFE_FCCOBB = data[4];

  //WaitCCIF();
  SetCCIF();
  //WaitCCIF();
  return true; //Check if everything is actually OK
}

//Public Functions

bool Flash_ReadByte(uint8_t offset, uint8_t *const byte)
{
  WaitCCIF();
  if (offset > 7 || offset < 0)
    return false;
  volatile uint32_t* const address = FLASH_DATA_START + offset;
  *byte = _FP(address);
}

bool Flash_Init(void)
{
  SIM_SCGC3 |= SIM_SCGC3_NFC_MASK;

  WaitCCIF();

  return true;
}

bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  uint8_t NbContiniousUnallocatedBytes = 0;

  uint8_t position = 0;

  for (uint8_t i = 0; i < FLASH_SIZE; i++) {
      if (AllocationMap[i] == 0)
        NbContiniousUnallocatedBytes++;
      else
        NbContiniousUnallocatedBytes = 0;

      if (i % size == 0)
        position = i;

      if (size < NbContiniousUnallocatedBytes && (i + 1) % size == 0) {
        for (uint8_t j = position; j <= i; j++) {
          AllocationMap[j] = 1;
        }

        *variable = (void*) (FLASH_DATA_START + position);
        return true;
      }
  }

  return false;
}


bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  uint32_t index = (uint32_t) (address - FLASH_DATA_START);

  if (index >= FLASH_SIZE || index < 0)
    return false;
  if (index % 4 != 0)
    return  false;
  index /= 4;

  uint64_t tempPhrase;

  ReadPhrase(&tempPhrase);

  uint32_t *tempArray = (uint32_t *) &tempPhrase;
  tempArray[index] = data;

  return WritePhrase(tempPhrase);
}


bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  uint16_t index = (uint16_t) (address - FLASH_DATA_START);

  if (index >= FLASH_SIZE || index < 0)
    return false;
  if (index % 2 != 0)
    return  false;
  index /= 2;

  uint64_t tempPhrase;

  ReadPhrase(&tempPhrase);

  uint16_t *tempArray = (uint16_t *) &tempPhrase;
  tempArray[index] = data;

  return WritePhrase(tempPhrase);
}


bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  uint8_t index = (uint8_t) (address - FLASH_DATA_START);

  if (index >= FLASH_SIZE || index < 0)
    return false;

  uint64_t tempPhrase;

  ReadPhrase(&tempPhrase);

  uint8_t *tempArray = (uint8_t *) &tempPhrase;
  tempArray[index] = data;

  return WritePhrase(tempPhrase);
}

bool Flash_Erase(void)
{
  uint32_8union_t flashStart;
  flashStart.l = FLASH_DATA_START;

  WaitCCIF();

  FTFE_FCCOB0 = FLASH_CMD_ERASE_SECTOR;  //Command to erase
  FTFE_FCCOB1 = flashStart.s.Byte2;      //sets the flash address of the correct sector
  FTFE_FCCOB2 = flashStart.s.Byte3;
  FTFE_FCCOB3 = flashStart.s.Byte4 & 0xF0;
  //WaitCCIF();
  SetCCIF();
 // WaitCCIF();

  return true;
}

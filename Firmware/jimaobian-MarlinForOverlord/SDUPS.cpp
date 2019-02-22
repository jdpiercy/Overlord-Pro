#include "Configuration.h"

#include "Marlin.h"
#include "SDUPS.h"
#include "cardreader.h"
#include "temperature.h"
#include "UltiLCD2_menu_print.h"

//uint32_t SDUPSFilePosition=0;


static int8_t SDUPSMaxIndex=-1,SDUPSMinIndex=-1;
static uint8_t SDUPSPositionIndex=0;


boolean SDUPSIsWorking()
{
  return(eeprom_read_byte((const uint8_t*)EEPROM_SDUPS_WORKING_OFFSET) == 'H');
}

boolean SDUPSIsStarting()
{
  return(eeprom_read_byte((const uint8_t*)EEPROM_SDUPS_WORKING_OFFSET) == 'S');
}

void SDUPSStart()
{
  SDUPSPositionIndex=((uint8_t)millis())%SDUPSPositionSize;
  SDUPSMinIndex=SDUPSPositionIndex;
  eeprom_write_dword(((uint32_t *)EEPROM_SDUPS_POSITION_OFFSET) + SDUPSPositionIndex, 1);
  eeprom_write_byte((uint8_t*)EEPROM_SDUPS_WORKING_OFFSET, 'S');
//  SERIAL_DEBUGLNPGM("SDUPSStart...");
//  SERIAL_DEBUGLN((int)SDUPSPositionIndex);
}

void SDUPSDone()
{
  eeprom_write_byte((uint8_t*)EEPROM_SDUPS_WORKING_OFFSET, 'L');
}

void SDUPSStorePosition(uint32_t theFilePosition)
{
  SDUPSPositionIndex=(SDUPSPositionIndex+1)% SDUPSPositionSize;
  if (SDUPSMinIndex == SDUPSPositionIndex) {
    SDUPSMinIndex = SDUPSMaxIndex;
  }
  SDUPSMaxIndex = SDUPSPositionIndex;

  if ((SDUPSMaxIndex+SDUPSPositionSize - SDUPSMinIndex)%SDUPSPositionSize == 3) {
    eeprom_write_byte((uint8_t*)EEPROM_SDUPS_WORKING_OFFSET, 'H');
    //    SERIAL_DEBUGLNPGM("SDUPSworking...");
  }
  eeprom_write_dword(((uint32_t *)EEPROM_SDUPS_POSITION_OFFSET) + SDUPSPositionIndex, theFilePosition);
  eeprom_write_word(((uint16_t *)EEPROM_SDUPS_PAUSE_TIME_OFFSET) + SDUPSPositionIndex, (millis() - starttime + pausetime)/60000UL);

  //  SERIAL_DEBUGLNPGM("SDUPSPositionIndex:");
  SERIAL_DEBUGLN((int)SDUPSPositionIndex);
  //  SERIAL_DEBUGLNPGM("the FilePosition:");
  //  SERIAL_DEBUGLN(theFilePosition);
}

void SDUPSOverlapPosition(uint32_t theFilePosition)
{
  eeprom_write_dword(((uint32_t *)EEPROM_SDUPS_POSITION_OFFSET) + SDUPSPositionIndex, theFilePosition);
  eeprom_write_word(((uint16_t *)EEPROM_SDUPS_PAUSE_TIME_OFFSET) + SDUPSPositionIndex, millis() - starttime + pausetime);

  //  SERIAL_DEBUGLNPGM("SDUPSOverlapPositionIndex:");
  //  SERIAL_DEBUGLN((int)SDUPSPositionIndex);
  //  SERIAL_DEBUGLNPGM("the FilePosition:");
  //  SERIAL_DEBUGLN(theFilePosition);
}

void SDUPSScanPosition()
{
  SDUPSMaxIndex=0;
  SDUPSMinIndex=0;
  uint32_t maxBuffer=0,minBuffer=0xffffffffUL,eepromReadBuffer=0;
  SERIAL_DEBUGLNPGM("eepromReadBuffer:");
  for (int i=0; i<SDUPSPositionSize; i++) {
    eepromReadBuffer=eeprom_read_dword((uint32_t *)(EEPROM_SDUPS_POSITION_OFFSET)+i);

//    SERIAL_DEBUG(i);
//    SERIAL_DEBUGPGM(":");
//    SERIAL_DEBUGLN(eepromReadBuffer);

    if (eepromReadBuffer) {
      if (eepromReadBuffer>=maxBuffer) {
        maxBuffer=eepromReadBuffer;
        SDUPSMaxIndex=i;
      }
      if (eepromReadBuffer<minBuffer) {
        minBuffer=eepromReadBuffer;
        SDUPSMinIndex=i;
      }
    }
  }


//  SERIAL_DEBUGLNPGM("SDUPSScanIndex");
//  SERIAL_DEBUGLN(SDUPSMaxIndex);
//  SERIAL_DEBUGLN(SDUPSMinIndex);
//
//  SERIAL_DEBUGLNPGM("SDUPSScanPosition:");
//  SERIAL_DEBUGLN(eeprom_read_dword((uint32_t *)(EEPROM_SDUPS_POSITION_OFFSET)+SDUPSMaxIndex));
//  SERIAL_DEBUGLN(eeprom_read_dword((uint32_t *)(EEPROM_SDUPS_POSITION_OFFSET)+SDUPSMinIndex));
}

uint32_t SDUPSRetrievePosition(int8_t theIndex)
{
  uint32_t eepromReadBuffer;

  if (theIndex>=-SDUPSPositionSize && theIndex<0) {
    theIndex=(SDUPSMinIndex-theIndex-1)%SDUPSPositionSize;
    SDUPSPositionIndex=theIndex;
    eepromReadBuffer=eeprom_read_dword((uint32_t *)(EEPROM_SDUPS_POSITION_OFFSET)+theIndex);
    pausetime = eeprom_read_word((uint16_t *)(EEPROM_SDUPS_PAUSE_TIME_OFFSET)+theIndex) * 60000UL;
    if (eepromReadBuffer==0) {
      eepromReadBuffer=0xffffffffUL;
    }
  }
  else if (theIndex<=SDUPSPositionSize && theIndex>0) {
    theIndex=(SDUPSMaxIndex-theIndex+1+SDUPSPositionSize)%SDUPSPositionSize;
    SDUPSPositionIndex=theIndex;
    eepromReadBuffer=eeprom_read_dword((uint32_t *)(EEPROM_SDUPS_POSITION_OFFSET)+theIndex);
    pausetime = eeprom_read_word((uint16_t *)(EEPROM_SDUPS_PAUSE_TIME_OFFSET)+theIndex) * 60000UL;
    if (eepromReadBuffer==0) {
      eepromReadBuffer=0xffffffffUL;
    }
  }
  else{
    eepromReadBuffer=0xffffffffUL;
  }

//  SERIAL_DEBUGLN("SDUPSRetrievePosition");
//  SERIAL_DEBUGLN(eepromReadBuffer);

  return eepromReadBuffer;
}

void SDUPSRemovePosition(int8_t theIndex)
{
  if (theIndex>=-SDUPSPositionSize && theIndex<0) {
    theIndex=(SDUPSMinIndex-theIndex-1)%SDUPSPositionSize;
    eeprom_write_dword((uint32_t *)(EEPROM_SDUPS_POSITION_OFFSET)+theIndex,0);
  }
  else if (theIndex<=SDUPSPositionSize && theIndex>0) {
    theIndex=(SDUPSMaxIndex-theIndex+1+SDUPSPositionSize)%SDUPSPositionSize;
    eeprom_write_dword((uint32_t *)(EEPROM_SDUPS_POSITION_OFFSET)+theIndex,0);
  }
}


bool SDUPSRetrieveClass()
{
  uint16_t SDUPSValidate;

  uint16_t SDUPSValidateStored;

  char* bufferPtr;

  uint8_t cardErrorTimes=0;

  int bufferLengthStored;

  int index=EEPROM_SDUPS_CLASS_OFFSET;


  EEPROM_READ_VAR(index, lcd_cache);
  EEPROM_READ_VAR(index, SDUPSValidateStored);

  bufferPtr = (char *)&lcd_cache;

  SDUPSValidate = 0;
  for (int i=0; i<sizeof(lcd_cache); i++) {
    SDUPSValidate += *(bufferPtr+i);
  }

  if (SDUPSValidateStored != SDUPSValidate) {
//    SERIAL_DEBUGLNPGM("lcd_cache_Validate:");
//    SERIAL_DEBUGLN(SDUPSValidateStored);
//    SERIAL_DEBUGLN(SDUPSValidate);
    return false;
  }

  uint32_t fileSizeStored;

  _EEPROM_readData(index, (uint8_t*)card.longFilename, 20);
  card.longFilename[20]=0;
  EEPROM_READ_VAR(index, fileSizeStored);
  EEPROM_READ_VAR(index, SDUPSValidateStored);
  EEPROM_READ_VAR(index, bufferLengthStored);

  bufferPtr = (char*)malloc(bufferLengthStored+1);
  if (bufferPtr != NULL) {
    _EEPROM_readData(index, (uint8_t*)bufferPtr, bufferLengthStored);
    bufferPtr[bufferLengthStored]=0;


    SDUPSValidate = 0;
    for (int i=0; i<bufferLengthStored; i++) {
      SDUPSValidate += *(bufferPtr+i);
    }

    if (SDUPSValidate == SDUPSValidateStored) {
      card.openFile(bufferPtr, true);
      if (card.isFileOpen()) {
        if (fileSizeStored ==  card.getFileSize()) {
          free(bufferPtr);
          return true;
        }
        else{
          SERIAL_DEBUGLNPGM("file not same");
        }
      }
      else{
        SERIAL_DEBUGLNPGM("card cannot open");
      }
    }
    else{
//      SERIAL_DEBUGLNPGM("buffer_Validate:");
//      SERIAL_DEBUGLN(SDUPSValidateStored);
//      SERIAL_DEBUGLN(SDUPSValidate);
    }
    free(bufferPtr);
    return false;
  }
  else{
    Stop(STOP_REASON_OUT_OF_MEMORY);
    return false;
  }



}


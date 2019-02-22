#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "Configuration.h"


#define EEPROM_MATERIAL_SETTINGS_OFFSET 0xe00

#define EEPROM_FIRST_RUN_DONE_OFFSET 0x3ff  //1 Byte

#define EEPROM_SDUPS_WORKING_OFFSET 0x3fe   //1 Byte

#define EEPROM_PRIMED_OFFSET 0x3fd          //1 Byte

#define EEPROM_LANGUAGE_OFFSET 0x3fc        //1 Byte

#define EEPROM_SDUPS_HEATING_OFFSET 0x3fa        //2 Bytes

#define EEPROM_SDUPS_HEATING_BED_OFFSET 0x3f8        //2 Bytes

#define EEPROM_SDUPS_CLASS_OFFSET 0x400     //1720 Bytes to   0xAB8

#define EEPROM_SDUPS_POSITION_OFFSET  0xC00   //256 Bytes

#define EEPROM_SDUPS_PAUSE_TIME_OFFSET  0xD00   //128 Bytes

#define EEPROM_DEVICE_OFFSET 0xB00 //3Bytes

#define EEPROM_FITTING_OFFSET 0xB10 //

#define EEPROM_DEVICE_ID 0xD80

#define EEPROM_OFFSET 100


void Config_ResetDefault();

#ifdef EEPROM_CHITCHAT
void Config_PrintSettings();
#else
FORCE_INLINE void Config_PrintSettings() {
}
#endif

#ifdef EEPROM_SETTINGS
void Config_StoreSettings();
void Config_RetrieveSettings();
#else
FORCE_INLINE void Config_StoreSettings() {
}
FORCE_INLINE void Config_RetrieveSettings() {
  Config_ResetDefault(); Config_PrintSettings();
}
#endif



#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))

void _EEPROM_writeData(int &pos, uint8_t* value, uint16_t size);
void _EEPROM_readData(int &pos, uint8_t* value, uint16_t size);



#endif//CONFIG_STORE_H

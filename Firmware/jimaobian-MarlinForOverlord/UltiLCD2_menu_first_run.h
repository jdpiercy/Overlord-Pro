#ifndef ULTI_LCD2_MENU_FIRST_RUN_H
#define ULTI_LCD2_MENU_FIRST_RUN_H
#include "ConfigurationStore.h"

#define IS_FIRST_RUN_DONE() (eeprom_read_byte((const uint8_t*)EEPROM_FIRST_RUN_DONE_OFFSET) == 'U')
#define SET_FIRST_RUN_DONE() do { eeprom_write_byte((uint8_t*)EEPROM_FIRST_RUN_DONE_OFFSET, 'U'); } while(0)

void lcd_menu_first_run_init();
void lcd_menu_first_run_temperature_error();
void lcd_menu_first_run_language();


#endif//ULTI_LCD2_MENU_FIRST_RUN_H

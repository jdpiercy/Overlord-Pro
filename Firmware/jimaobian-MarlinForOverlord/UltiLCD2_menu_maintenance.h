#ifndef ULTI_LCD2_MENU_MAINTENANCE_H
#define ULTI_LCD2_MENU_MAINTENANCE_H

void lcd_menu_maintenance();
void lcd_menu_advanced_settings();
void lcd_menu_maintenance_advanced_bed_heatup();
void manualLevelRoutine();
void lcd_menu_advanced_version();
void lcd_menu_maintenance_advanced_heatup();
void lcd_menu_device();


void readSensorOffset();
void writeSensorOffset();
bool isFactorySensorOffsetMode();
void restoreFactorySensorOffset();
void clearFactorySensorOffset();

#define EMoveSpeed  10

#endif//ULTI_LCD2_MENU_MAINTENANCE_H

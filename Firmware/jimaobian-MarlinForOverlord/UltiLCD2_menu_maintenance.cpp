#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
  #include "UltiLCD2.h"
  #include "UltiLCD2_hi_lib.h"
  #include "UltiLCD2_gfx.h"
  #include "UltiLCD2_menu_maintenance.h"
  #include "UltiLCD2_menu_first_run.h"
  #include "UltiLCD2_menu_material.h"
  #include "cardreader.h"
  #include "lifetime_stats.h"
  #include "ConfigurationStore.h"
  #include "temperature.h"
  #include "pins.h"
  #include "UltiLCD2_menu_maintenance.h"
  #include "fitting_bed.h"
  #include "stepper.h"
#include "ConfigurationStore.h"

static void lcd_menu_maintenance_retraction();
static void lcd_retraction_details(uint8_t nr);
static char* lcd_retraction_item(uint8_t nr);
static void lcd_menu_advanced_factory_reset();
static void doFactoryReset();
static void lcd_menu_advanced_state();
void lcd_menu_advanced_version();
static void lcd_menu_advanced_level();

static void doAdvancedLevelDone();
static void doAdvancedLevel();
void lcd_menu_maintenance_advanced_bed_heatup();
static void lcd_menu_maintenance_extrude();
void lcd_menu_maintenance_advanced_heatup();
static void lcd_menu_advanced_about();
static void lcd_advanced_about_details(uint8_t nr);
static char* lcd_advanced_about_item(uint8_t nr);
static void lcd_advanced_settings_details(uint8_t nr);
static char* lcd_advanced_settings_item(uint8_t nr);
static void lcd_menu_advanced_movement();
static void lcd_advanced_movement_details(uint8_t nr);
static char* lcd_advanced_movement_item(uint8_t nr);
static void lcd_menu_advanced_temperature();
static void lcd_advanced_temperature_details(uint8_t nr);
static char* lcd_advanced_temperature_item(uint8_t nr);
static void lcd_menu_maintenance_advanced();
static void lcd_advanced_details(uint8_t nr);
static char* lcd_advanced_item(uint8_t nr);
static void lcd_menu_maintenance_details(uint8_t nr);
static char* lcd_menu_maintenance_item(uint8_t nr);
static void lcd_menu_maintenance_waiting_auto_level();

static void lcd_menu_maintenance_select_level();
static void lcd_menu_maintenance_select_manual_level();
static void lcd_menu_maintenance_heat_for_level();
static void lcd_menu_maintenance_heat_for_manual_level();
static void lcd_menu_maintenance_manual_level();
void lcd_menu_device();

static void lcd_menu_maintenance_auto_level_refine_option();
static void lcd_menu_maintenance_auto_level_refine_process();


  #define CIRCLE_RADIUS 55.0
  #define CIRCLE_RADIUS_STRING "55"

static float circleDegree=0;
static float circleRadius=CIRCLE_RADIUS;
static float maintenanceLevelZ=ADDING_Z_FOR_POSITIVE;
static uint8_t linePosition = 0;

  #define MANUAL_LEVEL_NONE 0
  #define MANUAL_LEVEL_ENABLE 1
  #define MANUAL_LEVEL_STOP 2
uint8_t manualLevelState=MANUAL_LEVEL_NONE;

/****************************************************************************************
* Maintenance Menu
*
****************************************************************************************/
static char* lcd_menu_maintenance_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_UP "Reload"),
                                   PSTR(CHINESE_POINT "\xA2" "\x81"  "\xF6" "\x80"  ),
                                   PSTR(CHINESE_POINT "\x96" "\x83"  "\x97" "\x83"  " ""\xA3" "\x84"  "\xA9" "\x84"  "\x83" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_POINT "Calibrate"),
                                   PSTR(CHINESE_POINT "\xFB" "\x80"  "\xFC" "\x80"  "\xFE" "\x80"  "\xFF" "\x80"  ),
                                   PSTR(CHINESE_POINT "\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xA4" "\x83"  "\x85" "\x83"  )) );
  else if (nr == 2)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_DOWN "Clean"),
                                   PSTR(CHINESE_POINT "\x97" "\x82"  "\x98" "\x82"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                                   PSTR(CHINESE_POINT "\xD5" "\x82"  "\xD6" "\x82"  " ""\x90" "\x84"  "\xC6" "\x82"  )) );
  else
    strcpy_P(card.longFilename, PSTR("???"));

  return card.longFilename;
}

static void lcd_menu_maintenance_details(uint8_t nr)
{
  char buffer[22];

  if (nr == 0)
  {
#ifdef FilamentDetection
    if (isFilamentDetectionEnable) {
      if (FilamentAvailable()) {
        lcd_draw_detailP(LS(PSTR("Material Yes."),
                            PSTR("\xC8" "\x80"  "\xF6" "\x80"  ),
                            PSTR("\x96" "\x83"  "\x97" "\x83"  "Yes")) );
      }
      else{
        lcd_draw_detailP(LS(PSTR("Material No."),
                            PSTR("\x85" "\x82"  "\xF6" "\x80"  ),
                            PSTR("\x96" "\x83"  "\x97" "\x83"  "No")) );
      }
    }
    else{
      lcd_draw_detailP(LS(PSTR("Change the Material."),
                          PSTR("\xA3" "\x81"  "\xA2" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                          PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA3" "\x84"  "\xA9" "\x84"  "\x83" "\x83"  )) );
    }
#else
    lcd_draw_detailP(LS(PSTR("Change the Material."),
                        PSTR("\xA3" "\x81"  "\xA2" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                        PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA3" "\x84"  "\xA9" "\x84"  "\x83" "\x83"  )) );
#endif
  }
  else if (nr == 1)
    lcd_draw_detailP(LS(PSTR("Level the Platform."),
                        PSTR("\xA4" "\x81"  "\xFB" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  ),
                        PSTR("\xB7" "\x83"  "\xB8" "\x83"  " ""\xB9" "\x83"  "\x85" "\x83"  )) );
  else if (nr == 2)
    lcd_draw_detailP(LS(PSTR("Clean the Nozzle."),
                        PSTR("\x97" "\x82"  "\x98" "\x82"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                        PSTR("\x90" "\x84"  "\xC6" "\x82"  " ""\xD5" "\x82"  "\xD6" "\x82"  )) );
  else
    lcd_draw_detailP(PSTR("???"));
}

void lcd_menu_maintenance()
{
  LED_NORMAL();
  lcd_normal_menu(LS(PSTR("Maintenance Menu"),
                     PSTR("\xC4" "\x80"  "\xA3" "\x80"  ),
                     PSTR("\xBA" "\x83"  "\xAC" "\x83"  "\xD7" "\x82"  "\xB7" "\x83"  )) , 3, lcd_menu_maintenance_item, lcd_menu_maintenance_details);
  lcd_lib_draw_gfx(0, 0, maintenanceGfx);

  if (IS_SELECTED_SCROLL(-1)) {
    lcd_change_to_menu(lcd_menu_main, SCROLL_MENU_ITEM_POS(0), MenuUp);
  }
  if (IS_SELECTED_SCROLL(3)) {
    lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(0), MenuDown);
  }
  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0)) {
      setTargetHotend(material[active_extruder].temperature, active_extruder);
      isReload=true;
      lcd_change_to_menu(lcd_menu_change_material_preheat, SCROLL_MENU_ITEM_POS(0), MenuForward);
      menuTimer = millis();
    }
    else if (IS_SELECTED_SCROLL(1)) {
      lcd_change_to_menu(lcd_menu_maintenance_select_level, SCROLL_MENU_ITEM_POS(0), MenuForward);
    }
    else if (IS_SELECTED_SCROLL(2)) {
      isReload=false;
      lcd_change_to_menu(lcd_material_clean_nozzle_option, SCROLL_MENU_ITEM_POS(0), MenuForward);
    }
  }
}

/****************************************************************************************
* Level select Menu
*
****************************************************************************************/
void doStartHeatForLevel(){
  menuTimer = millis();
}

static void lcd_menu_maintenance_select_level()
{
  lcd_question_screen(lcd_menu_maintenance_heat_for_level, doStartHeatForLevel, LS(PSTR("YES"),
                                                                    PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                                    PSTR("\x91" "\x84"  "\xB6" "\x83"  )) , lcd_menu_maintenance_select_manual_level, NULL, LS(PSTR("NO"),
                                                                                                                                   PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                                                                                   PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) , MenuForward, MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Would you like to"),
                                                  PSTR("\xA5" "\x81"  "\xA6" "\x81"  "\x84" "\x80"  "\x85" "\x80"  "\xA7" "\x81"  "\xB1" "\x80"  ),
                                                  PSTR("\xBB" "\x83"  "\xBC" "\x83"  " ""\xB7" "\x83"  "\xB8" "\x83"  " ""\xB9" "\x83"  "\x85" "\x83"  " ""\xDE" "\x82"  "\xB4" "\x83"  "\xF2" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Automatically level"),
                                                  PSTR("\xA4" "\x81"  "\xFB" "\x80"  "\xA8" "\x81"  "\xA9" "\x81"  "\xAA" "\x81"  "?"),
                                                  PSTR("\xBD" "\x83"  "\xBE" "\x83"  " ""\x9E" "\x83"  "\xBF" "\x83"  "\xF3" "\x82"  "\xC0" "\x83"  "\xC1" "\x83"  "\x9C" "\x83"  "\xC2" "\x83"  "?")) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("the buildplate?"),
                                                  PSTR(""),
                                                  PSTR("")) );
}

/****************************************************************************************
* Level select Manual Menu
*
****************************************************************************************/

static void doManualLevelHeat()
{
  fanSpeedPercent = 0;
  for(uint8_t e=0; e<EXTRUDERS; e++)
  {
    target_temperature[e] = material[e].temperature;
    fanSpeedPercent = max(fanSpeedPercent, material[0].fan_speed);
    volume_to_filament_length[e] = 1.0 / (M_PI * (material[e].diameter / 2.0) * (material[e].diameter / 2.0));
    extrudemultiply[e] = material[e].flow;
  }

  target_temperature_bed = 0;
  fanSpeed = 0;
  SERIAL_DEBUGPGM("target_temperature_bed");
  SERIAL_DEBUGLN(target_temperature_bed);
}

static void doCancelManualLevel()
{
  nextEncoderPos=1;
}

static void lcd_menu_maintenance_select_manual_level()
{
  lcd_question_screen(lcd_menu_maintenance_heat_for_manual_level, doManualLevelHeat, LS(PSTR("YES"),
                                                                                        PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                                                        PSTR("\x91" "\x84"  "\xB6" "\x83"  )) ,
                      lcd_menu_maintenance, doCancelManualLevel, LS(PSTR("NO"),
                                                                    PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                    PSTR("\xFF" "\x82"  "\xC6" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Would you like to"),
                                                  PSTR(""),
                                                  PSTR("\xB7" "\x83"  "\xBC" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  " ""\xB7" "\x83"  "\xB8" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Manually level"),
                                                  PSTR("\xA5" "\x81"  "\xA6" "\x81"  "\xAB" "\x81"  "\xB1" "\x80"  "\xA4" "\x81"  "\xFB" "\x80"  "\xAA" "\x81"  "?"),
                                                  PSTR("\xB9" "\x83"  "\x85" "\x83"  "\xF3" "\x82"  "\xC0" "\x83"  "\xC1" "\x83"  "\x9C" "\x83"  "\xC2" "\x83"  "?")) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("the buildplate?"),
                                                  PSTR(""),
                                                  PSTR("")) );
}

/****************************************************************************************
* Auto level :waiting heating
*
****************************************************************************************/
static bool willEnterSensorCompensation = false;

static void doMaintenanceCancelAutoLevel()
{
  lcd_change_to_menu(lcd_menu_maintenance,1,MenuBackward);
  setTargetHotend(0, 0);
}

static void lcd_menu_maintenance_heat_for_level()
{
  LED_GLOW_HEAT();
  setTargetHotend(160, 0);
  int16_t temp = degHotend(0) - 20;
  int16_t target = degTargetHotend(0) - 10 - 20;
  if (temp < 0) temp = 0;
  if (temp > target && millis()-menuTimer>800) {
    setTargetHotend(0, 0);
    setTargetBed(0);
    enquecommand_P(PSTR("G29\nM84"));
    currentMenu = lcd_menu_maintenance_waiting_auto_level;
    temp = target;
    if (isFactorySensorOffsetMode()) {
      willEnterSensorCompensation = true ;
    }
    else{
      willEnterSensorCompensation = false;
    }
  }

  uint8_t progress = uint8_t(temp * 125 / target);
  if (progress < minProgress)
    progress = minProgress;
  else
    minProgress = progress;

  lcd_info_screen(NULL, doMaintenanceCancelAutoLevel, LS(PSTR("CANCEL"),
                                                         PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                         PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) , MenuForward);
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Printhead heating in"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\xAE" "\x80"  "\x90" "\x80"  "\x8B" "\x80"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\x90" "\x83"  "\x91" "\x83"  "\xC0" "\x82"  " ""\x94" "\x83"  "\x95" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("case that material"),
                                                  PSTR("\xEE" "\x80"  "\xEF" "\x80"  "\xF2" "\x80"  "\xF3" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  "\x98" "\x83"  " ""\xE3" "\x82"  "\x99" "\x83"  "\x9A" "\x83"  "\x9B" "\x83"  "\x9C" "\x83"  "\x9D" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("is left around it."),
                                                  PSTR(""),
                                                  PSTR("")) );
  lcd_progressbar(progress);
}


/****************************************************************************************
* Auto level :waiting Auto leveling finishing
*
****************************************************************************************/

static void doMaintenanceCancelAutoLevelProcession()
{
  doMaintenanceCancelAutoLevel();
  printing_state=PRINT_STATE_HOMING_ABORT;
}

static void lcd_menu_maintenance_waiting_auto_level()
{
  LED_NORMAL();

  if (lcd_lib_encoder_pos >= 2){
    if (Device_isLevelSensor) {
      willEnterSensorCompensation = true;
    }
  }
  
  if (printing_state!=PRINT_STATE_NORMAL || is_command_queued() || isCommandInBuffer()) {
    if (willEnterSensorCompensation) {
      lcd_info_screen(NULL, NULL, LS(PSTR("Sensor Calibrate"),
                                     PSTR("Sensor Calibrate"),
                                     PSTR("Sensor Calibrate")) , MenuForward);
    }
    else{
      lcd_info_screen(NULL, doMaintenanceCancelAutoLevelProcession, LS(PSTR("ABORT"),
                                                                       PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                                                       PSTR("\xE5" "\x83"  "\xD7" "\x82"  "\xDB" "\x82"  )) , MenuForward);
    }
  }
  else{
    if (willEnterSensorCompensation) {
      lcd_change_to_menu(lcd_menu_maintenance_auto_level_refine_option,0,MenuForward);
    }
    else{
      lcd_change_to_menu(lcd_menu_maintenance,1,MenuBackward);
    }
    
  }
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11), LS(PSTR("The Nozzle will"),
                                                 PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x81" "\x81"  "\xE3" "\x80"  "\xF8" "\x80"  ),
                                                 PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\x9D" "\x83"  "\x9E" "\x83"  "\x9F" "\x83"  "\xA0" "\x83"  "\xC8" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24), LS(PSTR("touch the buildplate"),
                                                 PSTR("\xED" "\x80"  "\xF9" "\x80"  "\xFA" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  ),
                                                 PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xCA" "\x82"  "\xD2" "\x82"  "\xA3" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37), LS(PSTR("gently to do the"),
                                                 PSTR("\x99" "\x82"  "\xAD" "\x81"  "\xC8" "\x80"  "\"""\xAE" "\x81"  "\xAE" "\x81"  "\"""\xAF" "\x81"  ),
                                                 PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xA4" "\x83"  "\x85" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64), LS(PSTR("calibration process."),
                                                 PSTR(""),
                                                 PSTR("")));
}




/****************************************************************************************
 * Auto level :waiting Auto leveling refine option
 *
 ****************************************************************************************/

static void resetSensorOffset(){
  fittingBedOffSensorsetInit();
  int index=EEPROM_FITTING_OFFSET;
  uint8_t isValid = 'F';
  EEPROM_WRITE_VAR(index, isValid);
  EEPROM_WRITE_VAR(index, fittingBedSensorOffset);
  EEPROM_WRITE_VAR(index, fittingBedSensorOffset);
}

void readSensorOffset(){
  SERIAL_DEBUGLNPGM("readSensorOffset");
  int index=EEPROM_FITTING_OFFSET;
  uint8_t isValid;
  EEPROM_READ_VAR(index, isValid);
  if (isValid == 'F' || isValid == 'U') {
    EEPROM_READ_VAR(index, fittingBedSensorOffset);
  }else{
    resetSensorOffset();
  }
}

void writeSensorOffset(){
  SERIAL_DEBUGLNPGM("writeSensorOffset");
  int index=EEPROM_FITTING_OFFSET;
  uint8_t isValid;
  EEPROM_READ_VAR(index, isValid);
  if (isValid == 'F') {
    index=EEPROM_FITTING_OFFSET;
    isValid = 'U';
    EEPROM_WRITE_VAR(index, isValid);
    EEPROM_WRITE_VAR(index, fittingBedSensorOffset);
    EEPROM_WRITE_VAR(index, fittingBedSensorOffset);
  }
  else if (isValid == 'U'){
    index=EEPROM_FITTING_OFFSET;
    EEPROM_WRITE_VAR(index, isValid);
    EEPROM_WRITE_VAR(index, fittingBedSensorOffset);
  }
  else{
    resetSensorOffset();
  }
}

bool isFactorySensorOffsetMode(){
//  SERIAL_DEBUGLNPGM("isFactorySensorOffsetMode");
  int index=EEPROM_FITTING_OFFSET;
  uint8_t isValid;
  EEPROM_READ_VAR(index, isValid);
  return isValid == 'F' && Device_isLevelSensor;
}

void restoreFactorySensorOffset(){
  SERIAL_DEBUGLNPGM("restoreFactorySensorOffset");

  int index=EEPROM_FITTING_OFFSET;
  uint8_t isValid;
  EEPROM_READ_VAR(index, isValid);
  if (isValid == 'F' || isValid == 'U') {
    index=EEPROM_FITTING_OFFSET;
    EEPROM_READ_VAR(index, isValid);
    EEPROM_READ_VAR(index, fittingBedSensorOffset);
    EEPROM_READ_VAR(index, fittingBedSensorOffset);
    
    index=EEPROM_FITTING_OFFSET;
    EEPROM_WRITE_VAR(index, isValid);
    EEPROM_WRITE_VAR(index, fittingBedSensorOffset);
  }else{
    resetSensorOffset();
  }
}

void clearFactorySensorOffset(){
  SERIAL_DEBUGLNPGM("clearFactorySensorOffset");

  int index=EEPROM_FITTING_OFFSET;
  uint8_t isValid = 0;
  EEPROM_WRITE_VAR(index, isValid);
}

static int8_t autoLevelRefineIndex;
static void doAutoLevelRefine();

static void doCancelAutoLevelRefine(){
  nextEncoderPos = 1;
}

static void doPrepareAutoLevelRefine(){
  dropsegments = 0;
  autoLevelRefineIndex = -1;
  fittingBedReset();
  fittingBedResetK();
  add_homeing[Z_AXIS]=0;
  enquecommand_P(PSTR("G28"));
  doAutoLevelRefine();
  lcd_lib_encoder_pos = 0;
}

static void lcd_menu_maintenance_auto_level_refine_option(){
  lcd_question_screen(lcd_menu_maintenance_auto_level_refine_process, doPrepareAutoLevelRefine,LS(PSTR("YES"),
                                                                                                  PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                                                                  PSTR("\x91" "\x84"  "\xB6" "\x83"  )) ,
                      lcd_menu_maintenance, doCancelAutoLevelRefine,LS(PSTR("NO"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(10, 11, 11), LS(PSTR("Sensor offset."),
                                                 PSTR("Sensor offset."),
                                                 PSTR("Sensor offset.")));
  lcd_lib_draw_string_centerP(LS(20, 24, 24), LS(PSTR("Prepare A4 paper"),
                                                 PSTR("Prepare A4 paper"),
                                                 PSTR("Prepare A4 paper")) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37), LS(PSTR("on the plate"),
                                                 PSTR("on the plate"),
                                                 PSTR("on the plate")));
}

/****************************************************************************************
 * Auto level :waiting Auto leveling refine resume
 *
 ****************************************************************************************/

static void doAutoLevelRefine(){
  
  char buffer[64];
  
  char *bufferPtr;
  
  
  autoLevelRefineIndex++;
  
  if (autoLevelRefineIndex == NodeNum) {
    doCooldown();
    manualLevelState=MANUAL_LEVEL_STOP;

    writeSensorOffset();
    
    SERIAL_BED_DEBUGLNPGM("fittingBedArraydoAutoLevelRefine");
    for (int index=0; index<NodeNum; index++) {
      SERIAL_BED_DEBUG("G1 F4000 X");
      SERIAL_BED_DEBUG(dtostrf(fittingBedArray[index][X_AXIS], 10, 10, buffer));
      SERIAL_BED_DEBUG(" Y");
      SERIAL_BED_DEBUG(dtostrf(fittingBedArray[index][Y_AXIS], 10, 10, buffer));
      SERIAL_BED_DEBUG(" Z");
      SERIAL_BED_DEBUGLN(dtostrf(fittingBedArray[index][Z_AXIS]-ADDING_Z_FOR_POSITIVE, 10, 10, buffer));
    }

    SERIAL_BED_DEBUGLNPGM("fittingBedArraydoAutoLevelRefine");
    for (int index=0; index<NodeNum; index++) {
      SERIAL_BED_DEBUGLN(dtostrf(fittingBedSensorOffset[index], 10, 10, buffer));
    }
    
    SERIAL_BED_DEBUGLNPGM("togethertoLevelRefine");
    for (int index=0; index<NodeNum; index++) {
      SERIAL_BED_DEBUG("G1 F4000 X");
      SERIAL_BED_DEBUG(dtostrf(fittingBedArray[index][X_AXIS], 10, 10, buffer));
      SERIAL_BED_DEBUG(" Y");
      SERIAL_BED_DEBUG(dtostrf(fittingBedArray[index][Y_AXIS], 10, 10, buffer));
      SERIAL_BED_DEBUG(" Z");
      SERIAL_BED_DEBUGLN(dtostrf(fittingBedArray[index][Z_AXIS] + fittingBedSensorOffset[index] - ADDING_Z_FOR_POSITIVE, 10, 10, buffer));
    }
    
    fittingBed();
    add_homeing[Z_AXIS] = 1.0/plainFactorC;
    add_homeing[Z_AXIS] -= -ADDING_Z_FOR_POSITIVE;
    add_homeing[Z_AXIS] -= touchPlateOffset;
    fittingBedUpdateK();
    SERIAL_BED_DEBUGLNPGM("add_homeing[Z_AXIS]");
    SERIAL_BED_DEBUGLN(add_homeing[Z_AXIS]);
    Config_StoreSettings();
    
    enquecommand_P(PSTR("G28\nM84"));
    lcd_change_to_menu(lcd_menu_maintenance,1, MenuForward);
    dropsegments = DROP_SEGMENTS;
    nextEncoderPos=1;
    
    return;
  }
  
  enquecommand_P(PSTR("G1 F3000 X0 Y0 Z5"));

}

static void lcd_menu_maintenance_auto_level_refine_process(){
  
  char buffer[64];
  
  static bool error = 0;
  
  char *bufferPtr;
  
  if (printing_state == PRINT_STATE_NORMAL && commands_queued() <3) {
    
    if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0) {
      //Only move E.
      
      fittingBedSensorOffset[autoLevelRefineIndex] -= 0.03* lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM;
      lcd_lib_encoder_pos = 0;
      menuTimer = 0;
    }
    
    if (millis()-menuTimer>350) {
      error = !error;
      menuTimer = millis();
      bufferPtr=buffer;
      
      strcpy_P(bufferPtr, PSTR("G1 X"));
      bufferPtr+=strlen_P(PSTR("G1 X"));
      
      bufferPtr=float_to_string(fittingBedArray[autoLevelRefineIndex][X_AXIS], bufferPtr);
      
      strcpy_P(bufferPtr, PSTR(" Y"));
      bufferPtr+=strlen_P(PSTR(" Y"));
      
      bufferPtr=float_to_string(fittingBedArray[autoLevelRefineIndex][Y_AXIS], bufferPtr);
      
      strcpy_P(bufferPtr, PSTR(" Z"));
      bufferPtr+=strlen_P(PSTR(" Z"));
      
      bufferPtr=float_to_string(fittingBedSensorOffset[autoLevelRefineIndex] +fittingBedArray[autoLevelRefineIndex][Z_AXIS] + (error?0.06:0)  - ADDING_Z_FOR_POSITIVE, bufferPtr);
      
      enquecommand(buffer);
    }
    
  }
  
  if (isFactorySensorOffsetMode()) {
    lcd_info_screen(NULL, doAutoLevelRefine, LS(PSTR("Factory DONE"),
                                                PSTR("Factory DONE"),
                                                PSTR("Factory DONE")) , MenuForward);
  }
  else{
    lcd_info_screen(NULL, doAutoLevelRefine, LS(PSTR("DONE"),
                                                PSTR("DONE"),
                                                PSTR("DONE")) , MenuForward);
  }
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Press up and down."),
                                                  PSTR("Press up and down."),
                                                  PSTR("Press up and down.")));
  
  lcd_lib_draw_string_centerP(LS(20, 24, 24), LS(PSTR("Move paper until feel"),
                                                 PSTR("Move paper until feel"),
                                                 PSTR("Move paper until feel")));
  
  lcd_lib_draw_string_centerP(LS(30, 37, 37), LS(PSTR("resistance repeatly"),
                                                 PSTR("resistance repeatly"),
                                                 PSTR("resistance repeatly")));
}


/****************************************************************************************
* Manual Level :heat_for_manual_level
****************************************************************************************/
static void doManualLevel()
{
  char buffer[64];
  char *bufferPtr;
  
  volume_to_filament_length[active_extruder] = 1.0 / (M_PI * (material[active_extruder].diameter / 2.0) * (material[active_extruder].diameter / 2.0));

    circleDegree=0;
    circleRadius=CIRCLE_RADIUS;
    manualLevelState=MANUAL_LEVEL_ENABLE;
  
    fittingBedArrayInit();
    plainFactorA=plainFactorABackUp;
    plainFactorB=plainFactorBBackUp;
    plainFactorC=plainFactorCBackUp;
    
    for (uint8_t index=0; index<NodeNum; index++) {
      fittingBedArray[index][Z_AXIS]=(-1.0-plainFactorA*fittingBedArray[index][X_AXIS]-plainFactorB*fittingBedArray[index][Y_AXIS])/plainFactorC;
      fittingBedArray[index][Z_AXIS]+=fittingBedOffset[index];
    }
    
    fittingBedRaw(NodeNum);
    
    add_homeing[Z_AXIS] = 1.0/plainFactorC;
    add_homeing[Z_AXIS] -= -ADDING_Z_FOR_POSITIVE;
    add_homeing[Z_AXIS] -= touchPlateOffset;
    
    maintenanceLevelZ=-1.0/plainFactorC;
  
  //  SERIAL_DEBUGLNPGM("add_homeing[Z_AXIS]");
//  SERIAL_DEBUGLN(add_homeing[Z_AXIS]);

  enquecommand_P(PSTR("G28"));
  enquecommand_P(PSTR("G1 F6000 Z50\nG1 X0 Y"Y_MIN_POS_STR " Z" PRIMING_HEIGHT_STRING));

  for(uint8_t e = 0; e<EXTRUDERS; e++)
  {

    bufferPtr=buffer;

    if (!readPrimed()) {

      if (e>0) {
        //Todo
      }
      else{
        strcpy_P(bufferPtr, PSTR("G92 E"));
        bufferPtr+=strlen_P(PSTR("G92 E"));

        bufferPtr=int_to_string((0.0 - END_OF_PRINT_RETRACTION) / volume_to_filament_length[e] - PRIMING_MM3, bufferPtr);

        strcpy_P(bufferPtr, PSTR("\nG1 F"));
        bufferPtr+=strlen_P(PSTR("\nG1 F"));

        bufferPtr=int_to_string(END_OF_PRINT_RECOVERY_SPEED*60, bufferPtr);

        strcpy_P(bufferPtr, PSTR(" E-" PRIMING_MM3_STRING "\nG1 F"));
        bufferPtr+=strlen_P(PSTR(" E-" PRIMING_MM3_STRING "\nG1 F"));

        bufferPtr=int_to_string(PRIMING_MM3_PER_SEC * volume_to_filament_length[e]*60, bufferPtr);

        strcpy_P(bufferPtr, PSTR("X10 E0\n"));
        bufferPtr+=strlen_P(PSTR("X10 E0\n"));
      }
      setPrimed();
    }
    else{
      if (e>0) {
        //Todo
      }
      else{
        strcpy_P(bufferPtr, PSTR("G92 E-" PRIMING_MM3_STRING "\nG1 F"));
        bufferPtr+=strlen_P(PSTR("G92 E-" PRIMING_MM3_STRING "\nG1 F"));

        bufferPtr=int_to_string(PRIMING_MM3_PER_SEC * volume_to_filament_length[e]*60, bufferPtr);

        strcpy_P(bufferPtr, PSTR("X10 E0\n"));
        bufferPtr+=strlen_P(PSTR("X10 E0\n"));
      }
    }
    enquecommand(buffer);
  }
  
  enquecommand_P(PSTR("G1 X"CIRCLE_RADIUS_STRING " Y0 Z0.3 F3000"));
}

static void doCancelHeatManualLevel()
{
  nextEncoderPos=1;
  abortPrint();
}

static void lcd_menu_maintenance_heat_for_manual_level()
{
  LED_GLOW_HEAT();

  lcd_info_screen(lcd_menu_maintenance, doCancelHeatManualLevel, LS(PSTR("ABORT"),
                                                                    PSTR("\xC1" "\x80"  "\x83" "\x80"  ),
                                                                    PSTR("\xE5" "\x83"  "\xD7" "\x82"  "\xDB" "\x82"  )) , MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Heating up..."),
                                                  PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\x90" "\x80"  "\x8B" "\x80"  "..."),
                                                  PSTR("\x90" "\x83"  "\x91" "\x83"  "\xC0" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Preparing for"),
                                                  PSTR("\xFF" "\x80"  "\xEB" "\x80"  "\xA4" "\x81"  "\xFB" "\x80"  ),
                                                  PSTR("\xB7" "\x83"  "\xB8" "\x83"  " ""\xB9" "\x83"  "\x85" "\x83"  " ""\xC3" "\x83"  "\x8F" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("Calibration"),
                                                  PSTR(""),
                                                  PSTR("")) );
  for(uint8_t e=0; e<EXTRUDERS; e++)
  {
    if (current_temperature[e] < target_temperature[e] - TEMP_HYSTERESIS || current_temperature[e] > target_temperature[e] + TEMP_HYSTERESIS) {
      menuTimer=millis();
    }
  }

  if (millis()-menuTimer>=TEMP_RESIDENCY_TIME*1000UL && printing_state == PRINT_STATE_NORMAL)
  {
    doManualLevel();
    lcd_change_to_menu(lcd_menu_maintenance_manual_level);
  }

  uint8_t progress = 125;
  for(uint8_t e=0; e<EXTRUDERS; e++)
  {
    if (current_temperature[e] > 20)
      progress = min(progress, (current_temperature[e] - 20) * 125 / (target_temperature[e] - 20 - TEMP_WINDOW));
    else
      progress = 0;
  }

  if (progress < minProgress)
    progress = minProgress;
  else
    minProgress = progress;
  lcd_progressbar(progress);
}


/****************************************************************************************
* Manual Level :Manual_level
****************************************************************************************/
static void doFinishManualLevel()
{
  manualLevelState=MANUAL_LEVEL_NONE;

  add_homeing[Z_AXIS] = 1.0/plainFactorC;
  add_homeing[Z_AXIS] -= -ADDING_Z_FOR_POSITIVE;
  add_homeing[Z_AXIS] -= touchPlateOffset;

//  SERIAL_DEBUGLNPGM("add_homeing[Z_AXIS]");
//  SERIAL_DEBUGLN(add_homeing[Z_AXIS]);
  Config_StoreSettings();

  doCooldown();
  enquecommand_P(PSTR("G28\nM84"));
  nextEncoderPos=1;
}


void manualLevelRoutine()
{
  if (manualLevelState == MANUAL_LEVEL_ENABLE) {

    if (commands_queued() || is_command_queued()) {
      return;
    }

    if (feedrate>=1000) {
      menuTimer=millis();
    }
    else{
      if (millis()-menuTimer>1000) {
        feedrate=3000;
      }
    }

    while (movesplanned()<=3) {
      circleDegree+=1/(circleRadius*2*PI);

      if (circleDegree>=2*PI) {
        circleDegree-=2*PI;
        circleRadius-=0.4;
        plan_set_e_position(0);
        if (circleRadius<2) {
          manualLevelState = MANUAL_LEVEL_STOP;
          return;
        }
      }

      current_position[X_AXIS]=circleRadius*cos(circleDegree);
      current_position[Y_AXIS]=circleRadius*sin(circleDegree);
      current_position[E_AXIS]=circleRadius*circleDegree/10;

      //x y small move
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    }
  }
}


static void lcd_menu_maintenance_manual_level()
{
  if (printing_state == PRINT_STATE_NORMAL && !commands_queued()) {
    
    float circleDegreeBuffer;
    
    if (lcd_lib_encoder_pos != 0) {
      
      feedrate=500;
      
      circleDegreeBuffer=circleDegree-(movesplanned()/(circleRadius*2*PI));
      
      fittingBedArray[0][X_AXIS]=circleRadius*cos(circleDegreeBuffer);
      
      //      SERIAL_DEBUGLNPGM("fittingBedArray");
      
      for (uint8_t index=0; index<NodeNum; index++) {
        fittingBedArray[index][X_AXIS]=circleRadius*cos(circleDegreeBuffer);
        fittingBedArray[index][Y_AXIS]=circleRadius*sin(circleDegreeBuffer);
        circleDegreeBuffer+=2*PI/NodeNum;
        fittingBedArray[index][Z_AXIS]=(-1.0-plainFactorA*fittingBedArray[index][X_AXIS]-plainFactorB*fittingBedArray[index][Y_AXIS])/plainFactorC;
        
        //      SERIAL_ECHOLN(int(degreeIndex));
        //      SERIAL_ECHOLN(fittingBedArray[index][Z_AXIS]);
      }
      
      if (lcd_lib_encoder_pos>0) {
        fittingBedArray[0][Z_AXIS] -= 0.05;
        fittingBedArray[1][Z_AXIS] -= 0.0375;
        fittingBedArray[2][Z_AXIS] -= 0.0125;
        fittingBedArray[3][Z_AXIS] -= 0;
        fittingBedArray[4][Z_AXIS] -= 0.0125;
        fittingBedArray[5][Z_AXIS] -= 0.0375;
      }
      else{
        fittingBedArray[0][Z_AXIS] += 0.05;
        fittingBedArray[1][Z_AXIS] += 0.0375;
        fittingBedArray[2][Z_AXIS] += 0.0125;
        fittingBedArray[3][Z_AXIS] += 0;
        fittingBedArray[4][Z_AXIS] += 0.0125;
        fittingBedArray[5][Z_AXIS] += 0.0375;
      }
      
      if (fittingBedArray[0][Z_AXIS]<0.5) {
        for (uint8_t index=0; index<NodeNum; index++) {
          fittingBedArray[index][Z_AXIS]=0.5;
        }
      }
      
      //      SERIAL_DEBUGLNPGM("plainFactor old:");
      //      SERIAL_DEBUGLN(plainFactorA*1000000.0);
      //      SERIAL_DEBUGLN(plainFactorB*1000000.0);
      //      SERIAL_DEBUGLN(plainFactorC*1000000.0);
      
      fittingBedRaw(NodeNum);
      fittingBedUpdateK();
      
      //      SERIAL_DEBUGLNPGM("plainFactor new:");
      //      SERIAL_DEBUGLN(plainFactorA*1000000.0);
      //      SERIAL_DEBUGLN(plainFactorB*1000000.0);
      //      SERIAL_DEBUGLN(plainFactorC*1000000.0);
      
      fittingBedArrayInit();
      
      for (uint8_t index=0; index<NodeNum; index++) {
        fittingBedOffset[index]=(-1.0-plainFactorA*fittingBedArray[index][X_AXIS]-plainFactorB*fittingBedArray[index][Y_AXIS])/plainFactorC-(-1.0-plainFactorABackUp*fittingBedArray[index][X_AXIS]-plainFactorBBackUp*fittingBedArray[index][Y_AXIS])/plainFactorCBackUp;
      }
      
      current_position[Z_AXIS]=-1.0/plainFactorC-maintenanceLevelZ+0.3;
      
      lcd_lib_encoder_pos=0;
    }
  }
  
  
  if (manualLevelState == MANUAL_LEVEL_STOP) {
    doFinishManualLevel();
    lcd_change_to_menu(lcd_menu_maintenance);
  }
  
  lcd_info_screen(lcd_menu_maintenance, doFinishManualLevel, LS(PSTR("DONE"),
                                                                PSTR("\xDA" "\x81"  "\xBB" "\x80"  ),
                                                                PSTR("\xDD" "\x83"  "\xCE" "\x82"  "\xC7" "\x83"  )) , MenuBackward);
  lcd_lib_draw_string_centerP(LS(10, 11, 11), LS(PSTR("Press Down or UP"),
                                                 PSTR("\xE2" "\x80"  "\xD2" "\x80"  "\xE3" "\x80"  "\xE4" "\x80"  ",""\xA4" "\x81"  "\xB1" "\x81"  "\xA0" "\x80"  "\xEC" "\x80"  "\x89" "\x81"  ),
                                                 PSTR("Down or UP""\x86" "\x83"  "\x87" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24), LS(PSTR("to move the hotend"),
                                                 PSTR("\xFB" "\x80"  "\xFC" "\x80"  "\xB3" "\x81"  "\xB4" "\x81"  ",""\x91" "\x82"  "\xB5" "\x81"  "\x89" "\x81"  ),
                                                 PSTR("\xC9" "\x83"  "\xCA" "\x83"  "\xCB" "\x83"  " ""\xCC" "\x83"  "\xCD" "\x83"  "\xBE" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37), LS(PSTR("until the print"),
                                                 PSTR("\xB6" "\x81"  "\xA6" "\x81"  "\xF4" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xB7" "\x81"  "\xB8" "\x81"  ),
                                                 PSTR("\xC5" "\x82"  "\xCE" "\x83"  "\xCF" "\x83"  "\xB7" "\x83"  "\xD0" "\x83"  "\xC1" "\x83"  "\x9C" "\x83"  "\x9D" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64), LS(PSTR("works well."),
                                                 PSTR(""),
                                                 PSTR("")));
}

/****************************************************************************************
* Advanced Menu
*
****************************************************************************************/

static char* lcd_advanced_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_UP "Temperature"),
                                   PSTR(CHINESE_POINT "\x88" "\x80"  "\x89" "\x80"  ),
                                   PSTR(CHINESE_POINT "\xC4" "\x82"  "\xC5" "\x82"  )));
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_POINT "Movement"),
                                   PSTR(CHINESE_POINT "\xB9" "\x81"  "\xBA" "\x81"  ),
                                   PSTR(CHINESE_POINT "\x9F" "\x84"  "\xA0" "\x84"  "\xA1" "\x84"  "\xDB" "\x82"  )));
  else if (nr == 2)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_POINT "Settings"),
                                   PSTR(CHINESE_POINT "\xE0" "\x80"  "\xE1" "\x80"  ),
                                   PSTR(CHINESE_POINT "\x84" "\x83"  "\x85" "\x83"  )));
  else if (nr == 3)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_POINT "About"),
                                   PSTR(CHINESE_POINT "\x9B" "\x80"  "\xBB" "\x81"  "\xBC" "\x81"  "\xD3" "\x80"  ),
                                   PSTR(CHINESE_POINT "\xE5" "\x83"  "\xA3" "\x84"  "\xA4" "\x84"  )));
  else
    strcpy_P(card.longFilename, PSTR("???"));
  
  return card.longFilename;
}

static void lcd_advanced_details(uint8_t nr)
{
  char buffer[22];
  
  if (nr == 0)
    lcd_draw_detailP(LS(PSTR("Change the Temperature."),
                        PSTR("\xA4" "\x81"  "\xB1" "\x81"  "\x88" "\x80"  "\x89" "\x80"  ),
                        PSTR("\xC4" "\x82"  "\xC5" "\x82"  " ""\xB9" "\x83"  "\xD1" "\x83"  )) );
  else if (nr == 1)
    lcd_draw_detailP(LS(PSTR("Movement."),
                        PSTR("\xB9" "\x81"  "\xBA" "\x81"  ),
                        PSTR("\x9F" "\x84"  "\xA0" "\x84"  "\xA1" "\x84"  "\xDB" "\x82"  )));
  else if (nr == 2)
    lcd_draw_detailP(LS(PSTR("Settings."),
                        PSTR("\xE0" "\x80"  "\xE1" "\x80"  ),
                        PSTR("\x84" "\x83"  "\x85" "\x83"  )));
  else if (nr == 3)
    lcd_draw_detailP(LS(PSTR("About."),
                        PSTR("\x9B" "\x80"  "\xBB" "\x81"  "\xBC" "\x81"  "\xD3" "\x80"  ),
                        PSTR("\xE5" "\x83"  "\xA3" "\x84"  "\xA4" "\x84"  )));
  else
    lcd_draw_detailP(PSTR("???"));
}

static void lcd_menu_maintenance_advanced()
{
  LED_NORMAL();

  if (lcd_lib_encoder_pos >= 3 * ENCODER_TICKS_PER_SCROLL_MENU_ITEM)
    lcd_lib_encoder_pos = 3 * ENCODER_TICKS_PER_SCROLL_MENU_ITEM;

  lcd_normal_menu(LS(PSTR("Maintenance Menu"),
                     PSTR("\xC4" "\x80"  "\xA3" "\x80"  "\xD0" "\x80"  "\xD1" "\x80"  ),
                     PSTR("\xBA" "\x83"  "\xAC" "\x83"  "\xD7" "\x82"  "\xB7" "\x83"  " ""\xFB" "\x82"  "\xF1" "\x82"  )) , 4, lcd_advanced_item, lcd_advanced_details);

  lcd_lib_draw_gfx(0, 0, advancedGfx);


  if (IS_SELECTED_SCROLL(-1)) {
    lcd_change_to_menu(lcd_menu_maintenance, SCROLL_MENU_ITEM_POS(2), MenuUp);
  }

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0)){
      lcd_change_to_menu(lcd_menu_advanced_temperature, SCROLL_MENU_ITEM_POS(1), MenuForward);
    }
    else if (IS_SELECTED_SCROLL(1)){
      lcd_change_to_menu(lcd_menu_advanced_movement, SCROLL_MENU_ITEM_POS(1), MenuForward);
    }
    else if (IS_SELECTED_SCROLL(2)){
      lcd_change_to_menu(lcd_menu_advanced_settings, SCROLL_MENU_ITEM_POS(1), MenuForward);
    }
    else if (IS_SELECTED_SCROLL(3)){
      lcd_change_to_menu(lcd_menu_advanced_about, SCROLL_MENU_ITEM_POS(1), MenuForward);
    }
  }
}

/****************************************************************************************
* Temperature Menu
*
****************************************************************************************/
static char* lcd_advanced_temperature_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR("Return"),
                                   PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR("Heatup nozzle"),
                                   PSTR("\x90" "\x80"  "\x8B" "\x80"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                                   PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\x90" "\x83"  "\x91" "\x83"  )) );
  else if (nr == 2 && Device_isBedHeat)
    strcpy_P(card.longFilename, LS(PSTR("Heatup buildplate"),
                                   PSTR("\x90" "\x80"  "\x8B" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  ),
                                   PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\x90" "\x83"  "\x91" "\x83"  )) );
  else if (nr == (3 - !Device_isBedHeat))
    strcpy_P(card.longFilename, LS(PSTR("Fan speed"),
                                   PSTR("\xC0" "\x81"  "\xC1" "\x81"  "\xC2" "\x81"  "\xC3" "\x81"  ),
                                   PSTR("\xD2" "\x83"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) );
  else
    strcpy_P(card.longFilename, PSTR("???"));
  return card.longFilename;
}

static void lcd_advanced_temperature_details(uint8_t nr)
{

  char buffer[22];

  char* c = (char*)buffer;

  if (nr == 0)
    lcd_draw_detailP(LS(PSTR("Return to Advanced."),
                        PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                        PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
  {
    c = int_to_string(current_temperature[0], c, PSTR("`C"));
    *c++ = '/';
    c = int_to_string(target_temperature[0], c, PSTR("`C"));
    
    lcd_draw_detail((char*)buffer);
  }
  else if (nr == 2 && Device_isBedHeat)
  {
    c = int_to_string(current_temperature_bed, c, PSTR("`C"));
    *c++ = '/';
    c = int_to_string(target_temperature_bed, c, PSTR("`C"));
    
    lcd_draw_detail((char*)buffer);
  }
  else if (nr == (3 - !Device_isBedHeat))
  {
    c = int_to_string(lround((int(fanSpeed) * 100) / 255.0), c, PSTR("%"));
    
    lcd_draw_detail((char*)buffer);
  }
  else
    lcd_draw_detailP(PSTR("???"));
}

static void lcd_menu_advanced_temperature()
{

  LED_NORMAL();
  if (lcd_lib_encoder_pos < 0) lcd_lib_encoder_pos = 0;

  lcd_advance_menu(LS(PSTR("Temperature"),
                      PSTR("\x88" "\x80"  "\x89" "\x80"  ),
                      PSTR("\xC4" "\x82"  "\xC5" "\x82"  )) , 4 - !Device_isBedHeat, lcd_advanced_temperature_item, lcd_advanced_temperature_details);

  if (IS_SELECTED_SCROLL(4 - !Device_isBedHeat)) {
    lcd_change_to_menu(lcd_menu_advanced_movement, SCROLL_MENU_ITEM_POS(1), MenuDown);
  }

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
      lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(0), MenuBackward);
    else if (IS_SELECTED_SCROLL(1))
    {
      if (degTargetHotend(0) == 0) {
        setTargetHotend(220, 0);
      }
      active_extruder = 0;
      lcd_change_to_menu(lcd_menu_maintenance_advanced_heatup, 0);
      lcd_lib_button_up_down_reversed = true;
    }
    else if (IS_SELECTED_SCROLL(2) && Device_isBedHeat)
    {
      if (degTargetBed() == 0) {
        setTargetBed(40);
      }
      lcd_change_to_menu(lcd_menu_maintenance_advanced_bed_heatup, 0);
      lcd_lib_button_up_down_reversed = true;
    }
    else if (IS_SELECTED_SCROLL(3 - !Device_isBedHeat))
    {
      LCD_EDIT_SETTING_BYTE_PERCENT(fanSpeed,LS(PSTR("Fan speed"),
                                                PSTR("\xC0" "\x81"  "\xC1" "\x81"  "\xC2" "\x81"  "\xC3" "\x81"  ),
                                                PSTR("\xD2" "\x83"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) , LS(PSTR("%"),
                                                               PSTR("%"),
                                                               PSTR("%")) , 0, 100);
    }
  }

}

/****************************************************************************************
* Hotend Heatup
****************************************************************************************/
void lcd_menu_maintenance_advanced_heatup()
{
  LED_NORMAL();
  if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
  {
    target_temperature[active_extruder] = int(target_temperature[active_extruder]) + (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM);
    if (target_temperature[active_extruder] < 0)
      target_temperature[active_extruder] = 0;
    if (target_temperature[active_extruder] > HEATER_0_MAXTEMP - 15)
      target_temperature[active_extruder] = HEATER_0_MAXTEMP - 15;
    lcd_lib_encoder_pos = 0;
  }
  if (lcd_lib_button_pressed)
  {
    lcd_lib_button_up_down_reversed = false;
    lcd_change_to_menu(previousMenu, previousEncoderPos, MenuBackward);
    target_temperature[active_extruder] = 0;
  }
  lcd_lib_clear();
  char buffer[16];

  int_to_string(int(current_temperature[active_extruder]), buffer, PSTR("`C/"));
  int_to_string(int(target_temperature[active_extruder]), buffer+strlen(buffer), PSTR("`C"));

  lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("Nozzle temperature:"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ":"),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  ":")) );
  
  if (temp_error_handle) {
    if (temp_error_handle & 0x01) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37) ,LS(PSTR("Error: PT1 Max Temp"),
                                                     PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT1" "\xC4" "\x81"  "\x88" "\x80"  ),
                                                     PSTR("Error: PT1 Max Temp")) );
    }
    else if (temp_error_handle & 0x02) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37),LS(PSTR("Error: PT1 Min Temp"),
                                                    PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT1" "\xC5" "\x81"  "\x88" "\x80"  ),
                                                    PSTR("Error: PT1 Min Temp")) );
    }
    else if (temp_error_handle & 0x04) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37),LS(PSTR("Error: PT2 Max Temp"),
                                                    PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT2" "\xC4" "\x81"  "\x88" "\x80"  ),
                                                    PSTR("Error: PT2 Max Temp")) );
    }
    else if (temp_error_handle & 0x08) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37),LS(PSTR("Error: PT2 Min Temp"),
                                                    PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT2" "\xC5" "\x81"  "\x88" "\x80"  ),
                                                    PSTR("Error: PT2 Min Temp")) );
    }
    else{
      //Should not come here.
      lcd_lib_draw_string_centerP(LS(40, 37, 37),LS(PSTR("Error: Unknown"),
                                                    PSTR("\x80" "\x80"  "\x81" "\x80"  ":" "\xC6" "\x81"  "\xC7" "\x81"  ),
                                                    PSTR("Error: Unknown")) );
    }
  }
  lcd_lib_draw_string_centerP(LS(56, 56-3, 56-3),LS(PSTR("Click to return"),
                                                    PSTR("\x9E" "\x81"  "\x9F" "\x81"  "OK""\xE4" "\x80"  "\xBD" "\x81"  "\xBE" "\x81"  ),
                                                    PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  
  lcd_lib_draw_string_center(LS(30, 24, 24), buffer);
}

/****************************************************************************************
* bed Heatup
****************************************************************************************/
void lcd_menu_maintenance_advanced_bed_heatup()
{
  LED_NORMAL();
  if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
  {
    target_temperature_bed = int(target_temperature_bed) + (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM);
    if (target_temperature_bed < 0)
      target_temperature_bed = 0;
    if (target_temperature_bed > BED_MAXTEMP - 15)
      target_temperature_bed = BED_MAXTEMP - 15;
    lcd_lib_encoder_pos = 0;
  }
  if (lcd_lib_button_pressed)
  {
    lcd_change_to_menu(previousMenu, previousEncoderPos, MenuBackward);
    lcd_lib_button_up_down_reversed = false;
//    target_temperature_bed = 0;
  }

  char buffer[16];

  lcd_lib_clear();

  lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("Buildplate temp:"),
                                                  PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ":"),
                                                  PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(56, 56-3, 56-3) , LS(PSTR("Click to return"),
                                                      PSTR("\x9E" "\x81"  "\x9F" "\x81"  "OK""\xE4" "\x80"  "\xBD" "\x81"  "\xBE" "\x81"  ),
                                                      PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  int_to_string(int(current_temperature_bed), buffer, PSTR("`C/"));
  int_to_string(int(target_temperature_bed), buffer+strlen(buffer), PSTR("`C"));
  lcd_lib_draw_string_center(LS(30, 24, 24) , buffer);
}
/****************************************************************************************
* Movement Menu
*
****************************************************************************************/
static char* lcd_advanced_movement_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR("Return"),
                                   PSTR("\xBE" "\x81"  "\x89" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR("Home Head"),
                                   PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\xC8" "\x81"  "\x99" "\x80"  ),
                                   PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xE9" "\x82"  "\xD4" "\x83"  )) );
  else if (nr == 2)
    strcpy_P(card.longFilename, LS(PSTR("Move Material"),
                                   PSTR("\xAB" "\x81"  "\xB1" "\x80"  "\xDC" "\x80"  "\xF6" "\x80"  "\xC9" "\x81"  "\xF6" "\x80"  ),
                                   PSTR("\xBB" "\x83"  "\x96" "\x83"   "\xC9" "\x83"  "\xBC" "\x83"  )) );
  else
    strcpy_P(card.longFilename, PSTR("???"));
  
  return card.longFilename;
}

static void lcd_advanced_movement_details(uint8_t nr)
{
  if (nr == 0)
    lcd_draw_detailP(LS(PSTR("Return to Advanced."),
                        PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                        PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
  {
    lcd_draw_detailP(LS(PSTR("Home the Head."),
                        PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\xC8" "\x81"  "\x99" "\x80"  ),
                        PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xE9" "\x82"  "\xD4" "\x83"  )) );
  }
  else if (nr == 2)
  {
    lcd_draw_detailP(LS(PSTR("Move the Material."),
                        PSTR("\xAB" "\x81"  "\xB1" "\x80"  "\xDC" "\x80"  "\xF6" "\x80"  "\xC9" "\x81"  "\xF6" "\x80"  ),
                        PSTR("\xB7" "\x83"  "\xBC" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  " ""\x96" "\x83"  "\x97" "\x83"  " ""\xA6" "\x83"  "\xA7" "\x83"  "\xA3" "\x83"  " ""\xD5" "\x83"  "\xA9" "\x83"  )) );
  }
  else
    lcd_draw_detailP(PSTR("???"));
}

static void lcd_menu_advanced_movement()
{
  LED_NORMAL();

  lcd_advance_menu(LS(PSTR("Movement"),
                      PSTR("\xB9" "\x81"  "\xBA" "\x81"  ),
                      PSTR("\x9F" "\x84"  "\xA0" "\x84"  "\xA1" "\x84"  "\xDB" "\x82"  )) , 3, lcd_advanced_movement_item, lcd_advanced_movement_details);

  if (IS_SELECTED_SCROLL(-1)) {
    lcd_change_to_menu(lcd_menu_advanced_temperature, SCROLL_MENU_ITEM_POS(3 - !Device_isBedHeat), MenuUp);
  }

  if (IS_SELECTED_SCROLL(3)) {
    lcd_change_to_menu(lcd_menu_advanced_settings, SCROLL_MENU_ITEM_POS(1), MenuDown);
  }

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
      lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(1), MenuBackward);
    else if (IS_SELECTED_SCROLL(1))
    {
      lcd_lib_beep();
      enquecommand_P(PSTR("G28\nM84"));
    }
    else if (IS_SELECTED_SCROLL(2))
    {
      //            set_extrude_min_temp(0);
      active_extruder = 0;
      target_temperature[active_extruder] = material[active_extruder].temperature;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      lcd_change_to_menu(lcd_menu_maintenance_extrude, 0);
    }
  }
}
/****************************************************************************************
* extrude
****************************************************************************************/
static void lcd_menu_maintenance_extrude()
{
  LED_NORMAL();
  if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
  {
    if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 3)
    {
      current_position[E_AXIS] += lcd_lib_encoder_pos * 0.1;
      //Only move E.
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], EMoveSpeed, active_extruder);
      lcd_lib_encoder_pos = 0;
    }
    menuTimer = millis();
    target_temperature[active_extruder] = material[active_extruder].temperature;
  }
  
  if (current_temperature[active_extruder]<200) {
    menuTimer = millis();
  }
  
  if (millis()-menuTimer > 30000UL) {
      target_temperature[active_extruder] = 0;
  }
  
  if (lcd_lib_button_pressed)
  {
    //        set_extrude_min_temp(EXTRUDE_MINTEMP);
    target_temperature[active_extruder] = 0;
    lcd_change_to_menu(previousMenu, previousEncoderPos, MenuBackward);
  }

  char buffer[16];

  lcd_lib_clear();

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Nozzle temperature:"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ":"),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  ":")));
  lcd_lib_draw_string_centerP(LS(30, 37, 37), LS(PSTR("Press UP to retract"),
                                                 PSTR("\xE2" "\x80"  "\xD2" "\x80"  "\xE4" "\x80"  "\xC9" "\x81"  "\xF6" "\x80"   "," "\xE2" "\x80"  "\xE3" "\x80"  "\xE4" "\x80"  "\xDC" "\x80"  "\xF6" "\x80"  ),
                                                 PSTR("UP""\x96" "\x83"  "\x97" "\x83"  " ""\x96" "\x83"  "\x97" "\x83"  ",Down""\xA6" "\x83"  "\xA7" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64), LS(PSTR("Press Down to extrude"),
                                                 PSTR(""),
                                                 PSTR("")));
  lcd_lib_draw_string_centerP(LS(56, 56-3, 56-3), LS(PSTR("Click to return"),
                                                     PSTR("\x9E" "\x81"  "\x9F" "\x81"  "OK""\xE4" "\x80"  "\xBD" "\x81"  "\xBE" "\x81"  ),
                                                     PSTR("\xAA" "\x84"  "\xB4" "\x83"  )));
  int_to_string(int(current_temperature[active_extruder]), buffer, PSTR("`C/"));
  int_to_string(int(target_temperature[active_extruder]), buffer+strlen(buffer), PSTR("`C"));
  lcd_lib_draw_string_center(LS(20, 24, 24) , buffer);
}

/****************************************************************************************
* Settings Menu
*
****************************************************************************************/
static char* lcd_advanced_settings_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR("Return"),
                                   PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR("Retraction settings"),
                                   PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xBE" "\x81"  "\xCB" "\x81"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                                   PSTR("\xDE" "\x82"  "\xE9" "\x82"  "\xDB" "\x82"  " ""\xB7" "\x83"  "\xD7" "\x83"  " ""\x84" "\x83"  "\x85" "\x83"  )) );
  else if (nr == 2)
    strcpy_P(card.longFilename, LS(PSTR("Material settings"),
                                   PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                                   PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\x96" "\x83"  "\x97" "\x83"  " ""\x84" "\x83"  "\x85" "\x83"  )) );
#ifdef FilamentDetection
  else if (nr == 3)
    strcpy_P(card.longFilename, LS(PSTR("Material Detection"),
                                   PSTR("\xA7" "\x81"  "\xB1" "\x80"  "\xCC" "\x81"  "\xCD" "\x81"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                   PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xBB" "\x83"  "\xBC" "\x83"  " ""\xD8" "\x83"  "\x85" "\x83"  )) );
#endif
  else
    strcpy_P(card.longFilename, PSTR("???"));
  return card.longFilename;
}

static void lcd_advanced_settings_details(uint8_t nr)
{
  if (nr == 0)
    lcd_draw_detailP(LS(PSTR("Return to Advanced."),
                        PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                        PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
  {
    lcd_draw_detailP(LS(PSTR("Retraction settings."),
                        PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xBE" "\x81"  "\xCB" "\x81"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                        PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\xFF" "\x82"  "\xC6" "\x82"  " ""\x84" "\x83"  "\x85" "\x83"  )) );
  }
  else if (nr == 2)
  {
    lcd_draw_detailP(LS(PSTR("Material settings."),
                        PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                        PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\x96" "\x83"  "\x97" "\x83"  " ""\x84" "\x83"  "\x85" "\x83"  )));
  }
#ifdef FilamentDetection
  else if (nr == 3)
  {
    if (isFilamentDetectionEnable) {
      lcd_draw_detailP(LS(PSTR("Detection Sensor  ON"),
                          PSTR("\xCC" "\x81"  "\xCD" "\x81"  "\xBC" "\x80"  "\xCE" "\x81"  "\xAA" "\x80"  "\x9A" "\x80"  ),
                          PSTR("\xD8" "\x83"  "\x85" "\x83"  " ""\x83" "\x83"  "\xD9" "\x83"  " ON")));
    }
    else{
      lcd_draw_detailP(LS(PSTR("Detection Sensor OFF"),
                          PSTR("\xCC" "\x81"  "\xCD" "\x81"  "\xBC" "\x80"  "\xCE" "\x81"  "\x9B" "\x80"  "\xCF" "\x81"  ),
                          PSTR("\xD8" "\x83"  "\x85" "\x83"  " ""\x83" "\x83"  "\xD9" "\x83"  " OFF")));
    }
  }
#endif
  else
    lcd_draw_detailP(PSTR("???"));
}

void lcd_menu_advanced_settings()
{
  LED_NORMAL();

#ifdef FilamentDetection
  lcd_advance_menu(LS(PSTR("Settings"),
                      PSTR("\xE0" "\x80"  "\xE1" "\x80"  ),
                      PSTR("\x84" "\x83"  "\x85" "\x83"  )) , 4, lcd_advanced_settings_item, lcd_advanced_settings_details);
#else
  lcd_advance_menu(LS(PSTR("Settings"),
                      PSTR("\xE0" "\x80"  "\xE1" "\x80"  ),
                      PSTR("\x84" "\x83"  "\x85" "\x83"  )) , 3, lcd_advanced_settings_item, lcd_advanced_settings_details);
#endif
  
  if (IS_SELECTED_SCROLL(-1)) {
    lcd_change_to_menu(lcd_menu_advanced_movement, SCROLL_MENU_ITEM_POS(2), MenuUp);
  }

  #ifdef FilamentDetection
  if (IS_SELECTED_SCROLL(4))
  #else
  if (IS_SELECTED_SCROLL(3))
  #endif
  {
    lcd_change_to_menu(lcd_menu_advanced_about, SCROLL_MENU_ITEM_POS(1), MenuDown);
  }

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
      lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(2), MenuBackward);
    else if (IS_SELECTED_SCROLL(1))
    {
      lcd_change_to_menu(lcd_menu_maintenance_retraction, SCROLL_MENU_ITEM_POS(0));
    }
    else if (IS_SELECTED_SCROLL(2))
    {
      lcd_change_to_menu(lcd_menu_material_select, SCROLL_MENU_ITEM_POS(0));
    }
  #ifdef FilamentDetection
    else if (IS_SELECTED_SCROLL(3))
    {
      isFilamentDetectionEnable = !isFilamentDetectionEnable;
      Config_StoreSettings();
    }
  #endif
  }

}

/****************************************************************************************
* retraction
****************************************************************************************/

static char* lcd_retraction_item(uint8_t nr)
{
  
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR("Return"),
                                   PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR("Retract length"),
                                   PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xD0" "\x81"  "\x89" "\x80"  ),
                                   PSTR("\xDA" "\x83"  "\xC9" "\x83"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) );
  else if (nr == 2)
    strcpy_P(card.longFilename, LS(PSTR("Retract speed"),
                                   PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xC3" "\x81"  "\x89" "\x80"  ),
                                   PSTR("\xD0" "\x82"  "\xE8" "\x82"  "\xEC" "\x82"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) );
  else if (nr == 3)
    strcpy_P(card.longFilename, LS(PSTR("Recover speed"),
                                   PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xD1" "\x81"  "\xC8" "\x81"  "\xC3" "\x81"  "\x89" "\x80"  ),
                                   PSTR("\xD0" "\x82"  "\xE8" "\x82"  "\xEC" "\x82"  " ""\x92" "\x84"  "\xFA" "\x83"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) );
  else
    strcpy_P(card.longFilename, PSTR("???"));
  return card.longFilename;
}

static void lcd_retraction_details(uint8_t nr)
{
  char buffer[16];
  if (nr == 0)
    return;
  else if(nr == 1)
    float_to_string(retract_length, buffer, PSTR("mm"));
  else if(nr == 2)
    int_to_string(retract_feedrate / 60 + 0.5, buffer, PSTR("mm/sec"));
  else if(nr == 3)
    int_to_string(retract_recover_feedrate / 60 + 0.5, buffer, PSTR("mm/sec"));
  lcd_draw_detail(buffer);
  //    lcd_lib_draw_string(5, 57, buffer);
}

static void lcd_menu_maintenance_retraction()
{
  LED_NORMAL();

  lcd_scroll_menu(LS(PSTR("RETRACTION"),
                     PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xBE" "\x81"  "\xCB" "\x81"  ),
                     PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) , 4, lcd_retraction_item, lcd_retraction_details);
  if (lcd_lib_button_pressed){
    if (IS_SELECTED_SCROLL(0)){
      Config_StoreSettings();
      lcd_change_to_menu(lcd_menu_advanced_settings, 1, MenuBackward);
    }
    else if (IS_SELECTED_SCROLL(1)){
      LCD_EDIT_SETTING_FLOAT001(retract_length, LS(PSTR("Retract length") ,
                                                   PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xD0" "\x81"  "\x89" "\x80"  ),
                                                   PSTR("\xDA" "\x83"  "\xC9" "\x83"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) ,LS(PSTR("mm"),
                                                                 PSTR("mm"),
                                                                 PSTR("mm")) , 0, 50);
    }
    else if (IS_SELECTED_SCROLL(2)){
      LCD_EDIT_SETTING_SPEED(retract_feedrate, LS(PSTR("Retract speed"),
                                                  PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xC3" "\x81"  "\x89" "\x80"  ),
                                                  PSTR("\xD0" "\x82"  "\xE8" "\x82"  "\xEC" "\x82"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) ,LS(PSTR("mm/sec"),
                                                                PSTR("mm/sec"),
                                                                PSTR("mm/sec")), 0, max_feedrate[E_AXIS] * 60);
    }
    else if (IS_SELECTED_SCROLL(3)){
      LCD_EDIT_SETTING_SPEED(retract_recover_feedrate, LS(PSTR("Recover speed"),
                                                          PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xD1" "\x81"  "\xC8" "\x81"  "\xC3" "\x81"  "\x89" "\x80"  ),
                                                          PSTR("\xD0" "\x82"  "\xE8" "\x82"  "\xEC" "\x82"  " ""\x92" "\x84"  "\xFA" "\x83"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) , LS(PSTR("mm/sec"),
                                                                         PSTR("mm/sec"),
                                                                         PSTR("mm/sec")), 0, max_feedrate[E_AXIS] * 60);
    }
  }

}

/****************************************************************************************
* About Menu
*
****************************************************************************************/
static char* lcd_advanced_about_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR("Return"),
                                   PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR("Factory Reset"),
                                   PSTR("\xD1" "\x81"  "\xC8" "\x81"  "\x8B" "\x81"  "\xD2" "\x81"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                                   PSTR("\x82" "\x83"  "\x83" "\x83"  "\x99" "\x83"  )) );
  else if (nr == 2)
    strcpy_P(card.longFilename, LS(PSTR("Machine State"),
                                   PSTR("\xE0" "\x80"  "\xEB" "\x80"  "\xD3" "\x81"  "\xD4" "\x81"  ),
                                   PSTR("\x84" "\x83"  "\x8F" "\x83"  " ""\xCA" "\x83"  "\xDB" "\x83"  )) );
  else if (nr == 3)
    strcpy_P(card.longFilename, LS(PSTR("Version"),
                                   PSTR("\xD5" "\x81"  "\xD6" "\x81"  "\xD7" "\x81"  "\xBC" "\x81"  ),
                                   PSTR("\xE4" "\x83"  "\xAB" "\x84"  " ""\x86" "\x83"  "\xDC" "\x83"  )) );
  else
    strcpy_P(card.longFilename, PSTR("???"));
  
  return card.longFilename;
}

static void lcd_advanced_about_details(uint8_t nr)
{
  char buffer[32];
  char* bufferPtr;
  bufferPtr = buffer;
  strcpy_P(bufferPtr, PSTR(STRING_CONFIG_H_AUTHOR));
  bufferPtr += strlen_P(PSTR(STRING_CONFIG_H_AUTHOR));
  
  if (nr == 0){
    lcd_draw_detailP(LS(PSTR("Return to Advanced."),
                        PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                        PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  }
  else if (nr == 1){
    lcd_draw_detailP(LS(PSTR("Factory Reset."),
                        PSTR("\xD1" "\x81"  "\xC8" "\x81"  "\x8B" "\x81"  "\xD2" "\x81"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                        PSTR("\x82" "\x83"  "\x83" "\x83"  "\x99" "\x83"  )) );
  }
  else if (nr == 2){
    lcd_draw_detailP(LS(PSTR("Machine State."),
                        PSTR("\xE0" "\x80"  "\xEB" "\x80"  "\xD3" "\x81"  "\xD4" "\x81"  ),
                        PSTR("\x84" "\x83"  "\x8F" "\x83"   "\xCA" "\x83"  "\xDB" "\x83"  )) );
  }
  else if (nr == 3){
    switch (Device_type) {
      case OVERLORD_TYPE_PNHW:
        strcpy_P(bufferPtr, PSTR("PNHW"));
        break;
      case OVERLORD_TYPE_PNHL:
        strcpy_P(bufferPtr, PSTR("PNHL"));
        break;
      case OVERLORD_TYPE_PNH:
        strcpy_P(bufferPtr, PSTR("PNH"));
        break;
      case OVERLORD_TYPE_P:
        strcpy_P(bufferPtr, PSTR("P"));
        break;
      case OVERLORD_TYPE_MBNH:
        strcpy_P(bufferPtr, PSTR("MBNH"));
        break;
      case OVERLORD_TYPE_MNH:
        strcpy_P(bufferPtr, PSTR("MNH"));
        break;
      case OVERLORD_TYPE_MB:
        strcpy_P(bufferPtr, PSTR("MB"));
        break;
      case OVERLORD_TYPE_M:
        strcpy_P(bufferPtr, PSTR("M"));
        break;
      case OVERLORD_TYPE_PS:
        strcpy_P(bufferPtr, PSTR("PS"));
        break;
      case OVERLORD_TYPE_MS:
        strcpy_P(bufferPtr, PSTR("MS"));
        break;
      case OVERLORD_TYPE_PSD:
        strcpy_P(bufferPtr, PSTR("PSD"));
        break;
      default:
        break;
    }
    
    lcd_draw_detail(buffer);
  }
  else
    lcd_draw_detailP(PSTR("???"));
}

static void lcd_menu_advanced_about()
{
  LED_NORMAL();

  if (lcd_lib_encoder_pos >= 3 * ENCODER_TICKS_PER_SCROLL_MENU_ITEM)
    lcd_lib_encoder_pos = 3 * ENCODER_TICKS_PER_SCROLL_MENU_ITEM;

  lcd_advance_menu(LS(PSTR("About"),
                      PSTR("\x9B" "\x80"  "\xBB" "\x81"  ),
                      PSTR("\xE5" "\x83"  "\xA3" "\x84"  "\xA4" "\x84"  )) , 4, lcd_advanced_about_item, lcd_advanced_about_details);

  if (IS_SELECTED_SCROLL(-1)) {
    lcd_change_to_menu(lcd_menu_advanced_settings, SCROLL_MENU_ITEM_POS(2), MenuUp);
  }

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
      lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(3), MenuBackward);
    else if (IS_SELECTED_SCROLL(1))
    {
      lcd_change_to_menu(lcd_menu_advanced_factory_reset, SCROLL_MENU_ITEM_POS(1));
    }
    else if (IS_SELECTED_SCROLL(2))
    {
      lcd_change_to_menu(lcd_menu_advanced_state, SCROLL_MENU_ITEM_POS(0));
    }
    else if (IS_SELECTED_SCROLL(3))
    {
      lcd_change_to_menu(lcd_menu_advanced_version, SCROLL_MENU_ITEM_POS(0));
    }
  }
}


/****************************************************************************************
* version
****************************************************************************************/
void lcd_menu_advanced_version()
{
  char buffer[32]={0};
  if (lcd_lib_encoder_pos <= -2) {
    doAdvancedLevel();
    lcd_change_to_menu(lcd_menu_advanced_level);
  }
  else if (lcd_lib_encoder_pos >= 2){
    lcd_change_to_menu(lcd_menu_device);
  }

  char* bufferPtr;

  LED_NORMAL();

  lcd_info_screen(lcd_menu_advanced_about, NULL, LS(PSTR("Return"),
                                                    PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                                    PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  
  strcpy_P(buffer, LS(PSTR("Device ID:"),
                      PSTR("\xE0" "\x80"  "\xEB" "\x80"  "\xD8" "\x81"  "\xD9" "\x81"  ),
                      PSTR("\x84" "\x83"  "\x8F" "\x83"  " ID")) );
  bufferPtr = buffer + LS(strlen_P(PSTR("Device ID:")),
                          strlen_P(PSTR("\xE0" "\x80"  "\xEB" "\x80"  "\xD8" "\x81"  "\xD9" "\x81"  )),
                          strlen_P(PSTR("\x84" "\x83"  "\x8F" "\x83"  " ID"))) ;
  eeprom_read_block(bufferPtr, (uint8_t*)EEPROM_DEVICE_ID, 8);
  *(bufferPtr + 9) = 0;
  
  if (*bufferPtr <' ' || *bufferPtr >'}') {
    strcpy_P(bufferPtr, PSTR("Unknown"));
  }
  
//  lcd_lib_draw_string_center(LS(30,
//                                37,
//                                37) , buffer);
  
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , PSTR(STRING_VERSION_CONFIG_H));
  strcpy_P(buffer, PSTR(STRING_CONFIG_H_AUTHOR));
  bufferPtr = buffer + strlen_P(PSTR(STRING_CONFIG_H_AUTHOR));
  
  switch (Device_type) {
    case OVERLORD_TYPE_PNHW:
      strcpy_P(bufferPtr, PSTR("PNHW"));
      break;
    case OVERLORD_TYPE_PNHL:
      strcpy_P(bufferPtr, PSTR("PNHL"));
      break;
    case OVERLORD_TYPE_PNH:
      strcpy_P(bufferPtr, PSTR("PNH"));
      break;
    case OVERLORD_TYPE_P:
      strcpy_P(bufferPtr, PSTR("P"));
      break;
    case OVERLORD_TYPE_MBNH:
      strcpy_P(bufferPtr, PSTR("MBNH"));
      break;
    case OVERLORD_TYPE_MNH:
      strcpy_P(bufferPtr, PSTR("MNH"));
      break;
    case OVERLORD_TYPE_MB:
      strcpy_P(bufferPtr, PSTR("MB"));
      break;
    case OVERLORD_TYPE_M:
      strcpy_P(bufferPtr, PSTR("M"));
      break;
    case OVERLORD_TYPE_PS:
      strcpy_P(bufferPtr, PSTR("PS"));
      break;
    case OVERLORD_TYPE_MS:
      strcpy_P(bufferPtr, PSTR("MS"));
      break;
    case OVERLORD_TYPE_PSD:
      strcpy_P(bufferPtr, PSTR("PSD"));
      break;
    default:
      break;
  }
  
  lcd_lib_draw_string_center(LS(20, 24, 24) , buffer);

  nextEncoderPos=3;

}

  #define SCREW_NORMAL 0
  #define SCREW_UP 1
  #define SCREW_DOWN 2

  #define SCREW_STATE_FIRST 0
  #define SCREW_STATE_TOUCH 1
  #define SCREW_STATE_UNTOUCH 2

static uint8_t touchState=SCREW_STATE_FIRST;

static void doAdvancedLevel()
{
  enable_endstops(true);

  touchState=SCREW_STATE_FIRST;

  current_position[X_AXIS]=0;
  current_position[Y_AXIS]=0;
  current_position[Z_AXIS]=0;
  current_position[E_AXIS]=0;
  plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  if (Device_isPro) {
    current_position[X_AXIS]=Z_HOME_POS_PRO;
    current_position[Y_AXIS]=Z_HOME_POS_PRO;
    current_position[Z_AXIS]=Z_HOME_POS_PRO;
  }
  else{
    current_position[X_AXIS]=Z_HOME_POS_MINI;
    current_position[Y_AXIS]=Z_HOME_POS_MINI;
    current_position[Z_AXIS]=Z_HOME_POS_MINI;
  }

  current_position[E_AXIS]=0;
  feedrate=4000;

  plan_buffer_line_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
}

static void doAdvancedLevelDone()
{
  endstops_hit_on_purpose();
}

static void lcd_menu_advanced_level()
{
  lcd_info_screen(lcd_menu_advanced_version, doAdvancedLevelDone, LS(PSTR("DONE"),
                                                                     PSTR("\xDA" "\x81"  "\xBB" "\x80"  ),
                                                                     PSTR("\xDD" "\x83"  "\x97" "\x83"  )) );

  char buffer[16];
  int_to_string(int(st_get_position(X_AXIS)), buffer, NULL);
  lcd_lib_draw_string_center(10, buffer);

  int_to_string(int(READ(X_MAX_PIN)), buffer, PSTR(" "));
  int_to_string(int(READ(Y_MAX_PIN)), buffer+strlen(buffer), PSTR(" "));
  int_to_string(int(READ(Z_MAX_PIN)), buffer+strlen(buffer), PSTR(" "));
  lcd_lib_draw_string_center(20, buffer);

  static uint8_t xState=SCREW_NORMAL;
  static uint8_t yState=SCREW_NORMAL;
  static uint8_t zState=SCREW_NORMAL;

  switch (xState) {
    case SCREW_NORMAL:
      lcd_lib_draw_stringP(0, LS(40, 37, 37), LS(PSTR("O"),
                                                 PSTR(CHINESE_POINT),
                                                 PSTR(CHINESE_POINT)) );
      break;
    case SCREW_UP:
      lcd_lib_draw_stringP(0, LS(40, 37, 37), LS(PSTR(ENGLISH_UP),
                                                 PSTR(CHINESE_UP),
                                                 PSTR(CHINESE_UP)) );
      break;
    case SCREW_DOWN:
      lcd_lib_draw_stringP(0, LS(40, 37, 37), LS(PSTR(ENGLISH_DOWN),
                                                 PSTR(CHINESE_DOWN),
                                                 PSTR(CHINESE_DOWN)) );
      break;
    default:
      break;
  }
  
  switch (yState) {
    case SCREW_NORMAL:
      lcd_lib_draw_stringP(100, LS(40, 37, 37), LS(PSTR("O"),
                                                   PSTR(CHINESE_POINT),
                                                   PSTR(CHINESE_POINT)));
      break;
    case SCREW_UP:
      lcd_lib_draw_stringP(100, LS(40, 37, 37), LS(PSTR(ENGLISH_UP),
                                                   PSTR(CHINESE_UP),
                                                   PSTR(CHINESE_UP)));
      break;
    case SCREW_DOWN:
      lcd_lib_draw_stringP(100, LS(40, 37, 37), LS(PSTR(ENGLISH_DOWN),
                                                   PSTR(CHINESE_DOWN),
                                                   PSTR(CHINESE_DOWN)) );
      break;
    default:
      break;
  }
  
  switch (zState) {
    case SCREW_NORMAL:
      lcd_lib_draw_stringP(50, LS(30, 24, 24) , LS(PSTR("O"),
                                                   PSTR(CHINESE_POINT),
                                                   PSTR(CHINESE_POINT)));
      break;
    case SCREW_UP:
      lcd_lib_draw_stringP(50, LS(30, 24, 24), LS(PSTR(ENGLISH_UP),
                                                  PSTR(CHINESE_UP),
                                                  PSTR(CHINESE_UP)));
      break;
    case SCREW_DOWN:
      lcd_lib_draw_stringP(50, LS(30, 24, 24), LS(PSTR(ENGLISH_DOWN),
                                                  PSTR(CHINESE_DOWN),
                                                  PSTR(CHINESE_DOWN)) );
      break;
    default:
      break;
  }
  
  if (blocks_queued()) {
    menuTimer=millis();
  }

  if (millis()-menuTimer>300) {
    menuTimer=millis();


    switch (touchState) {
    case SCREW_STATE_FIRST:
      if (st_get_position(X_AXIS)>=(lround(STEPS_PER_UNIT_DELTA*261))) {
        enable_endstops(false);
        touchState=SCREW_STATE_TOUCH;
      }
      else{

        if (!READ(X_MAX_PIN)) {                    // touched
          xState=SCREW_DOWN;
          yState=SCREW_NORMAL;
          zState=SCREW_NORMAL;
        }

        if (!READ(Y_MAX_PIN)) {                    // touched
          xState=SCREW_NORMAL;
          yState=SCREW_DOWN;
          zState=SCREW_NORMAL;
        }

        if (!READ(Z_MAX_PIN)) {                    // touched
          xState=SCREW_NORMAL;
          yState=SCREW_NORMAL;
          zState=SCREW_DOWN;
        }


        current_position[X_AXIS]=(st_get_position(X_AXIS))/STEPS_PER_UNIT_DELTA;
        current_position[Y_AXIS]=current_position[X_AXIS];
        current_position[Z_AXIS]=current_position[X_AXIS];
        current_position[E_AXIS]=0;
        plan_set_position_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

        if ((READ(X_MAX_PIN))&&(READ(Y_MAX_PIN))&&(READ(Z_MAX_PIN))) {
          current_position[X_AXIS]=262;
          current_position[Y_AXIS]=262;
          current_position[Z_AXIS]=262;
          current_position[E_AXIS]=0;
          feedrate=4000;
        }
      }
      break;

    case SCREW_STATE_TOUCH:

      if (READ(X_MAX_PIN)) {                    //not touched
        xState=SCREW_UP;
      }
      else{
        if (xState==SCREW_UP) {
          xState=SCREW_NORMAL;
        }
      }

      if (READ(Y_MAX_PIN)) {                    //not touched
        yState=SCREW_UP;
      }
      else{
        if (yState==SCREW_UP) {
          yState=SCREW_NORMAL;
        }
      }

      if (READ(Z_MAX_PIN)) {                    //not touched
        zState=SCREW_UP;
      }
      else{
        if (zState==SCREW_UP) {
          zState=SCREW_NORMAL;
        }
      }

      current_position[X_AXIS]=261.5;
      current_position[Y_AXIS]=261.5;
      current_position[Z_AXIS]=261.5;
      current_position[E_AXIS]=0;
      feedrate=4000;
      touchState=SCREW_STATE_UNTOUCH;
      break;

    case SCREW_STATE_UNTOUCH:

      if (READ(X_MAX_PIN)) {                    //not touched
        if (xState==SCREW_DOWN) {
          xState=SCREW_NORMAL;
        }
      }
      else{
        xState=SCREW_DOWN;
      }

      if (READ(Y_MAX_PIN)) {                    //not touched
        if (yState==SCREW_DOWN) {
          yState=SCREW_NORMAL;
        }
      }
      else{
        yState=SCREW_DOWN;
      }

      if (READ(Z_MAX_PIN)) {                    //not touched
        if (zState==SCREW_DOWN) {
          zState=SCREW_NORMAL;
        }
      }
      else{
        zState=SCREW_DOWN;
      }

      current_position[X_AXIS]=262;
      current_position[Y_AXIS]=262;
      current_position[Z_AXIS]=262;
      current_position[E_AXIS]=0;
      feedrate=4000;
      touchState=SCREW_STATE_TOUCH;

      break;

    default:
      break;
    }
    //Only move z.
    plan_buffer_line_old(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
  }
}

/****************************************************************************************
* state
****************************************************************************************/
static void lcd_menu_advanced_state()
{
  LED_NORMAL();

  char buffer[32];
  char* c;
  
  lcd_info_screen(previousMenu, NULL, LS(PSTR("Return"),
                                         PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                         PSTR("\xAA" "\x84"  "\xB4" "\x83"  ) ));
  if (Device_isBattery) {
    if (READ(BatteryPin)) {
      lcd_lib_draw_stringP(5, LS(10, 1, 1) , LS(PSTR("Battery:    Charging"),
                                                PSTR("\x95" "\x80"  "\xDB" "\x81"  ":  ""\xAE" "\x80"  "\xAF" "\x80"  "\xDC" "\x81"  "\x95" "\x80"  ),
                                                PSTR("\xDE" "\x83"  "\xCA" "\x82"  "\xE9" "\x82"  ": ""\xDF" "\x83"  "\xCE" "\x82"  "\xC0" "\x82"  )) );
    }
    else{
      lcd_lib_draw_stringP(5, LS(10, 1, 1), LS(PSTR("Battery:     Charged"),
                                               PSTR("\x95" "\x80"  "\xDB" "\x81"  ":    " "\xDD" "\x81"  "\x95" "\x80"  "\xDE" "\x81"  ),
                                               PSTR("\xDE" "\x83"  "\xCA" "\x82"  "\xE9" "\x82"  ": ""\xDF" "\x83"  "\xCE" "\x82"  " ""\xDD" "\x83"  "\x97" "\x83"  )) );
    }
  }
  
  lcd_lib_draw_stringP(5, LS(20, 13, 13) , LS(PSTR("Life time:"),
                                              PSTR("\xA6" "\x80"  "\xA7" "\x80"  "\xDF" "\x81"  "\xE0" "\x81"   ":"),
                                              PSTR("Life time:")) );
  c = buffer;
  c = int_to_string(lifetime_minutes / 60, c, PSTR(":"));
  if (lifetime_minutes % 60 < 10)
    *c++ = '0';
  c = int_to_string(lifetime_minutes % 60, c);
  
  lcd_lib_draw_string((21-strlen(buffer))*6-1, LS(20, 13, 13) , buffer);
  
  
  lcd_lib_draw_stringP(5, LS(30, 25, 25) , LS(PSTR("Print time:"),
                                              PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xE1" "\x81"  "\xE2" "\x81"  ":"),
                                              PSTR("Print time:")) );
  c = buffer;
  c = int_to_string(lifetime_print_minutes / 60, c, PSTR(":"));
  if (lifetime_print_minutes % 60 < 10)
    *c++ = '0';
  c = int_to_string(lifetime_print_minutes % 60, c);
  lcd_lib_draw_string((21-strlen(buffer))*6-1, LS(30, 25, 25) , buffer);
  
  lcd_lib_draw_stringP(5, LS(40, 37, 37), LS(PSTR("Material:"),
                                 PSTR("\xF5" "\x80"  "\xF6" "\x80"  ":"),
                                 PSTR("\x96" "\x83"  "\x97" "\x83"  ":")) );
  c = buffer;
  c = int_to_string(lifetime_print_centimeters / 100, c, PSTR("m"));
  lcd_lib_draw_string((21-strlen(buffer))*6-1, LS(40, 37, 37) , buffer);

  nextEncoderPos=2;
}

/****************************************************************************************
* Factory Reset
****************************************************************************************/
static void doFactoryReset()
{
  //Clear the EEPROM settings so they get read from default.
  eeprom_write_byte((uint8_t*)100, 0);
  eeprom_write_byte((uint8_t*)101, 0);
  eeprom_write_byte((uint8_t*)102, 0);
  eeprom_write_byte((uint8_t*)EEPROM_FIRST_RUN_DONE_OFFSET, 0);
  eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), 0);

  cli();
  //NOTE: Jumping to address 0 is not a fully proper way to reset.
  // Letting the watchdog timeout is a better reset, but the bootloader does not continue on a watchdog timeout.
  // So we disable interrupts and hope for the best!
  //Jump to address 0x0000
  #ifdef __AVR__
  asm volatile (
    "clr	r30		\n\t"
    "clr	r31		\n\t"
    "ijmp	\n\t"
    );
  #else
  //TODO
  #endif
}

static void lcd_menu_advanced_factory_reset()
{
  LED_GLOW();
  
  lcd_question_screen(NULL, doFactoryReset, LS(PSTR("YES"),
                                               PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                               PSTR("\x91" "\x84"  "\xB6" "\x83"  )) , previousMenu, NULL, LS(PSTR("NO"),
                                                                                  PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                                  PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Reset everything"),
                                                  PSTR("\x81" "\x81"  "\x94" "\x80"  "\xE3" "\x81"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                                                  PSTR("\xE9" "\x82"  "\xD4" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("to default?"),
                                                  PSTR("\xD1" "\x81"  "\xC8" "\x81"  "\x89" "\x81"  "\xDA" "\x80"  "\xDB" "\x80"  "?"),
                                                  PSTR("\x80" "\x83"  "\x81" "\x83"  "\xDB" "\x82"  "?")) );
  nextEncoderPos=1;
}



/****************************************************************************************
 * device setting
 ****************************************************************************************/
static char* lcd_menu_device_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, PSTR("Return"));
  else if (nr == OVERLORD_TYPE_P)
    strcpy_P(card.longFilename, PSTR("P"));
  else if (nr == OVERLORD_TYPE_M)
    strcpy_P(card.longFilename, PSTR("M"));
  else if (nr == OVERLORD_TYPE_MB)
    strcpy_P(card.longFilename, PSTR("MB"));
  else if (nr == OVERLORD_TYPE_PNH)
    strcpy_P(card.longFilename, PSTR("PNH"));
  else if (nr == OVERLORD_TYPE_MNH)
    strcpy_P(card.longFilename, PSTR("MNH"));
  else if (nr == OVERLORD_TYPE_MBNH)
    strcpy_P(card.longFilename, PSTR("MBNH"));
  else if (nr == OVERLORD_TYPE_PNHW)
    strcpy_P(card.longFilename, PSTR("PNHW"));
  else if (nr == OVERLORD_TYPE_PNHL)
    strcpy_P(card.longFilename, PSTR("PNHL"));
  else if (nr == OVERLORD_TYPE_PS)
    strcpy_P(card.longFilename, PSTR("PS"));
  else if (nr == OVERLORD_TYPE_MS)
    strcpy_P(card.longFilename, PSTR("MS"));
  else if (nr == OVERLORD_TYPE_PSD)
    strcpy_P(card.longFilename, PSTR("PSD"));
  else
    strcpy_P(card.longFilename, PSTR("???"));

  return card.longFilename;
}

static void lcd_menu_device_details(uint8_t nr)
{
  if (nr == 0)
    lcd_draw_detailP(PSTR("Return"));
  else if(nr == OVERLORD_TYPE_P)
    lcd_draw_detailP(PSTR("Pro"));
  else if(nr == OVERLORD_TYPE_M)
    lcd_draw_detailP(PSTR("Mini"));
  else if(nr == OVERLORD_TYPE_MB)
    lcd_draw_detailP(PSTR("Mini Bed Heater"));
  else if(nr == OVERLORD_TYPE_PNH)
    lcd_draw_detailP(PSTR("Pro New Heater"));
  else if(nr == OVERLORD_TYPE_MNH)
    lcd_draw_detailP(PSTR("Mini New Heater"));
  else if(nr == OVERLORD_TYPE_MBNH)
    lcd_draw_detailP(PSTR("Mini Bed New Heater"));
  else if(nr == OVERLORD_TYPE_PNHW)
    lcd_draw_detailP(PSTR("Pro New Heater Wifi"));
  else if(nr == OVERLORD_TYPE_PNHL)
    lcd_draw_detailP(PSTR("Pro New Heater Lifeng"));
  else if(nr == OVERLORD_TYPE_PS)
    lcd_draw_detailP(PSTR("Pro with Sensor"));
  else if(nr == OVERLORD_TYPE_MS)
    lcd_draw_detailP(PSTR("Mini with Sensor"));
  else if(nr == OVERLORD_TYPE_PSD)
    lcd_draw_detailP(PSTR("Pro with Sensor and Door"));
}

void lcd_menu_device()
{
  LED_NORMAL();
  
  lcd_scroll_menu(LS(PSTR("Device"),
                     PSTR("\xE0" "\x80"  "\xEB" "\x80"  ),
                     PSTR("\xCC" "\x82"  "\x8F" "\x83"  )) , OVERLORD_TYPE_MAX - OVERLORD_TYPE_MIN+2, lcd_menu_device_item, lcd_menu_device_details);
  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
    {
      lcd_change_to_menu(lcd_menu_advanced_version, 0, MenuBackward);
    }
    else{
      storeDevice(lcd_lib_encoder_pos);
      retrieveDevice();
      if (Device_isLevelSensor) {
        clearFactorySensorOffset();
      }
      delay(20);
      doFactoryReset();
    }
  }
}
#endif//ENABLE_ULTILCD2

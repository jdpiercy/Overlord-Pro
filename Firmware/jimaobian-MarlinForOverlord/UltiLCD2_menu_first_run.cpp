#include <avr/pgmspace.h>

#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
  #include "Marlin.h"
  #include "cardreader.h"//This code uses the card.longFilename as buffer to store data, to save memory.
  #include "temperature.h"
  #include "ConfigurationStore.h"
  #include "UltiLCD2.h"
  #include "UltiLCD2_hi_lib.h"
  #include "UltiLCD2_menu_material.h"
  #include "UltiLCD2_menu_first_run.h"
  #include "UltiLCD2_menu_print.h"
  #include "UltiLCD2_menu_maintenance.h"
  #include "fitting_bed.h"
  #include "stepper.h"

static void lcd_menu_first_run_print_card_detect();
static void lcd_menu_first_run_print();
static void lcd_menu_first_run_material_PLA();
static void setFirstRunDone();
static void lcd_menu_first_run_material_load_wait();
static void doAfterMaterialLoad();
static void lcd_menu_first_run_material_load_forward();
static void lcd_menu_first_run_material_load_insert();
static void runMaterialForward();
static void lcd_menu_first_run_material_load_heatup();
static void lcd_menu_first_run_material_load();
static void doMaterialReset();
static void lcd_menu_first_run_waiting_auto_level();
static void lcd_menu_first_run_heat_for_level();
static void doCancelAutoLevel();
static void skipAll();
void lcd_menu_first_run_temperature_error();


  #define DRAW_PROGRESS_NR(nr) do { lcd_lib_draw_stringP((nr < 10) ? 100 : 94, 0, PSTR( #nr "/11")); } while(0)

//Run the first time you start-up the machine or after a factory reset.

/****************************************************************************************
* Language page
*
****************************************************************************************/
char* lcd_menu_first_run_language_callback(uint8_t nr)
{
  if (nr==0) {
    strcpy_P(card.longFilename, PSTR("English"));
  }
  else if (nr==1) {
    strcpy_P(card.longFilename, PSTR("\xC1" "\x80"  "\xF6" "\x81"  ));
  }
  else if (nr==2){
    strcpy_P(card.longFilename, PSTR("\xC7" "\x83"  "\x8F" "\x84"  "\xE5" "\x83"  ));
  }
}

void lcd_menu_first_run_language_details_callback(uint8_t nr)
{
  if (nr==0) {
    lcd_draw_detailP(PSTR("English"));
  }
  else if (nr==1) {
    lcd_draw_detailP(PSTR("\xC1" "\x80"  "\xF6" "\x81"  ));
  }
  else if (nr==2) {
    lcd_draw_detailP(PSTR("\xC7" "\x83"  "\x8F" "\x84"  "\xE5" "\x83"  ));
  }
}

void lcd_menu_first_run_language()
{
  LED_GLOW();

  lcd_advance_menu(PSTR("Language"), 3, lcd_menu_first_run_language_callback, lcd_menu_first_run_language_details_callback);

  if (lcd_lib_button_pressed) {
    if (IS_SELECTED_SCROLL(0)) {
      storeLanguage(LANGUAGE_ENGLISH);
    }
    else if (IS_SELECTED_SCROLL(1))
    {
      storeLanguage(LANGUAGE_CHINESE);
    }
    else if (IS_SELECTED_SCROLL(2)){
      storeLanguage(LANGUAGE_KOREAN);
    }
    lcd_change_to_menu(lcd_menu_first_run_init);
  }
}


/****************************************************************************************
* Welcome page
*
****************************************************************************************/
static void skipAll()
{
  Config_ResetDefault();
  doCancelAutoLevel();
  doMaterialReset();
  setFirstRunDone();
}

static void doHeatWait()
{
  menuTimer = millis();
}

void lcd_menu_first_run_init()
{
  LED_NORMAL();

  if (degHotend(0)<5 || degHotend(0)>28) {
    lcd_question_screen(lcd_menu_first_run_temperature_error, NULL, LS(PSTR("CONTINUE"),
                                                                       PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                       PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) , lcd_menu_first_run_temperature_error, skipAll, LS(PSTR("DEFAULT"),
                                                                                                                                     PSTR("\xDA" "\x80"  "\xDB" "\x80"  ),
                                                                                                                                     PSTR("\x80" "\x83"  "\x81" "\x83"  "\xDB" "\x82"  )) ,MenuForward,MenuForward);
  }
  else{
    lcd_question_screen(lcd_menu_first_run_heat_for_level, doHeatWait, LS(PSTR("CONTINUE"),
                                                                    PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                    PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) , lcd_menu_main, skipAll, LS(PSTR("DEFAULT"),
                                                                                                           PSTR("\xDA" "\x80"  "\xDB" "\x80"  ),
                                                                                                           PSTR("\x80" "\x83"  "\x81" "\x83"  "\xDB" "\x82"  )) ,MenuForward,MenuForward);
  }

  DRAW_PROGRESS_NR(1);

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Welcome to the first"),
                                                  PSTR("\xBD" "\x80"  "\xBE" "\x80"  "\xDC" "\x80"  "\xDD" "\x80"  "OverLord"),
                                                  PSTR("Welcome")));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("startup of OverLord!"),
                                                  PSTR("\xDE" "\x80"  "\xDF" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                                                  PSTR("\x82" "\x83"  "\x83" "\x83"  " ""\x84" "\x83"  "\x85" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("Press middle button"),
                                                  PSTR("\xE2" "\x80"  "\xE3" "\x80"  "OK""\xE4" "\x80"  "\xC6" "\x80"  "\xC7" "\x80"  ),
                                                  PSTR("OK ""\x86" "\x83"  "\x87" "\x83"  " ""\x88" "\x83"  "\x89" "\x83"  " ""\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("to continue"),
                                                  PSTR(""),
                                                  PSTR("")));
}

/****************************************************************************************
* Temperature Error page
*
****************************************************************************************/
void lcd_menu_first_run_temperature_error()
{
  LED_NORMAL();

  if (IS_FIRST_RUN_DONE()) {
    lcd_info_screen(lcd_menu_main, NULL, LS(PSTR("OK"),
                                            PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                            PSTR("OK")));
  }
  else{
    lcd_info_screen(lcd_menu_first_run_heat_for_level, doHeatWait, LS(PSTR("OK"),
                                                                PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                                PSTR("OK")));
  }
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Warning!!!"),
                                                  PSTR("\xE5" "\x80"  "\xE6" "\x80"  "!!!"),
                                                  PSTR("\x9C" "\x84"  "\x9D" "\x84"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Recommended"),
                                                  PSTR("\xE7" "\x80"  "\xE8" "\x80"  "\xE9" "\x80"  "\xEA" "\x80"  "\x88" "\x80"  "\x89" "\x80"  "\xAF" "\x80"  ),
                                                  PSTR("\x8D" "\x83"  "\x8E" "\x83"  " ""\xC4" "\x82"  "\xC5" "\x82"  " 5`C""\xC0" "\x82"  "\xBE" "\x82"  "\xC8" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("temperature is"),
                                                  PSTR("5`C""\x89" "\x81"  "28`C""\xE3" "\x80"  "\xA6" "\x80"  "\xA7" "\x80"  "\xE0" "\x80"  "\xEB" "\x80"  ),
                                                  PSTR("28`C""\xBE" "\x82"  "\xC8" "\x82"  " ""\x84" "\x83"  "\x8F" "\x83"  " ""\xDC" "\x82"  "\xDD" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("between 5" "`C-28" "`C!"),
                                                  PSTR(""),
                                                  PSTR("")));
}


/****************************************************************************************
* Auto level waiting heating
*
****************************************************************************************/
static void doCancelAutoLevel()
{
  setTargetHotend(0, 0);
}

static void lcd_menu_first_run_heat_for_level()
{
  LED_GLOW_HEAT();
  setTargetHotend(160, 0);
  int16_t temp = degHotend(0) - 20;
  int16_t target = degTargetHotend(0) - 10 - 20;
  if (temp < 0) temp = 0;
  if (temp > target && millis()-menuTimer>800) {
    setTargetHotend(0, 0);
    setTargetBed(0);
    enquecommand_P(PSTR("G29"));
    lcd_change_to_menu(lcd_menu_first_run_waiting_auto_level);
    temp = target;
  }

  uint8_t progress = uint8_t(temp * 125 / target);
  if (progress < minProgress)
    progress = minProgress;
  else
    minProgress = progress;

  lcd_info_screen(lcd_menu_first_run_material_load, doCancelAutoLevel,LS(PSTR("CANCEL"),
                                                                         PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                         PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) , MenuForward);
  
  
  DRAW_PROGRESS_NR(2);

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Printhead heating in"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\xAE" "\x80"  "\x90" "\x80"  "\x8B" "\x80"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\x90" "\x83"  "\x91" "\x83"  "\xC0" "\x82"  " ""\x94" "\x83"  "\x95" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("case that material"),
                                                  PSTR("\xEE" "\x80"  "\xEF" "\x80"  "\xF2" "\x80"  "\xF3" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  "\x98" "\x83"  " ""\xE3" "\x82"  "\x99" "\x83"  "\x9A" "\x83"  "\x9B" "\x83"  "\x9C" "\x83"  "\x9D" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("is left around it."),
                                                  PSTR(""),
                                                  PSTR("")));
  lcd_progressbar(progress);
}


/****************************************************************************************
* waiting Auto leveling finishing
*
****************************************************************************************/
static void doCancelAutoLevelProcession()
{
  doCancelAutoLevel();
  printing_state=PRINT_STATE_HOMING_ABORT;
}

static void lcd_menu_first_run_waiting_auto_level()
{
  LED_NORMAL();
  if (printing_state!=PRINT_STATE_NORMAL || is_command_queued() || isCommandInBuffer()) {
    lcd_info_screen(lcd_menu_first_run_material_load, doCancelAutoLevelProcession, LS(PSTR("ABORT"),
                                                                                      PSTR("\xC1" "\x80"  "\x83" "\x80"  ),
                                                                                      PSTR("\xE5" "\x83"  "\xD7" "\x82"  "\xDB" "\x82"  )) , MenuForward);
  }
  else{
    lcd_change_to_menu(lcd_menu_first_run_material_load,0,MenuForward);
  }

  DRAW_PROGRESS_NR(3);

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("The Nozzle will"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\xF7" "\x80"  "\xE3" "\x80"  "\xF8" "\x80"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\x9D" "\x83"  "\x9E" "\x83"  "\x9F" "\x83"  "\xA0" "\x83"  "\xC8" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("touch the buildplate"),
                                                  PSTR("\xED" "\x80"  "\xF9" "\x80"  "\xFA" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  ),
                                                  PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xCA" "\x82"  "\xD2" "\x82"  "\xA3" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("gently to do the"),
                                                  PSTR("\xFD" "\x80"  "\xDC" "\x80"  "\x85" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  "\xFE" "\x80"  "\xFF" "\x80"  ),
                                                  PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xA4" "\x83"  "\x85" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("calibration process."),
                                                  PSTR(""),
                                                  PSTR("")));
}


/****************************************************************************************
* Insert material
*
****************************************************************************************/

static void doMaterialReset()
{
  menuTimer = millis();
  lcd_material_reset_defaults();
  for(uint8_t e=0; e<EXTRUDERS; e++)
    lcd_material_set_material(0, e);
}

static void lcd_menu_first_run_material_load()
{
  LED_NORMAL();

  lcd_question_screen(lcd_menu_first_run_material_load_heatup, doMaterialReset, LS(PSTR("CONTINUE"),
                                                                                   PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                                   PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) , lcd_menu_first_run_print, doMaterialReset, LS(PSTR("SKIP"),
                                                                                                                                             PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                                                                                                                             PSTR("\xD0" "\x82"  "\x9B" "\x83"  )) ,MenuForward,MenuForward);
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Next step is to"),
                                                  PSTR("\xE3" "\x80"  "\xC9" "\x80"  "\x80" "\x81"  ),
                                                  PSTR("\x9D" "\x83"  "\xA5" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("insert material"),
                                                  PSTR("\xD2" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA6" "\x83"  "\xA7" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("push middle button"),
                                                  PSTR("\xB2" "\x80"  "\xE2" "\x80"  "\xE3" "\x80"  "OK" "\xE4" "\x80"  "\xC6" "\x80"  "\xC7" "\x80"  ),
                                                  PSTR("OK ""\x86" "\x83"  "\x87" "\x83"  " ""\x88" "\x83"  "\x89" "\x83"  " ""\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("to continue."),
                                                  PSTR(""),
                                                  PSTR("")));
  DRAW_PROGRESS_NR(4);

}

/****************************************************************************************
* Insert material preheat
*
****************************************************************************************/
static void lcd_menu_first_run_material_load_heatup()
{
  LED_GLOW_HEAT();
  setTargetHotend(210, 0);
  int16_t temp = degHotend(0) - 20;
  int16_t target = degTargetHotend(0) - 10 - 20;
  if (temp < 0) temp = 0;
  if (temp > target && millis()-menuTimer>800)
  {
    for(uint8_t e=0; e<EXTRUDERS; e++)
      volume_to_filament_length[e] = 1.0;       //Set the extrusion to 1mm per given value, so we can move the filament a set distance.

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
    currentMenu = lcd_menu_first_run_material_load_insert;
    menuTimer = millis();
    temp = target;
  }

  uint8_t progress = uint8_t(temp * 125 / target);
  if (progress < minProgress)
    progress = minProgress;
  else
    minProgress = progress;

  lcd_basic_screen();
  DRAW_PROGRESS_NR(5);

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Please wait,"),
                                                  PSTR(""),
                                                  PSTR("")));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("printhead heating for"),
                                                  PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\x90" "\x80"  "\x8B" "\x80"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                                                  PSTR("\x90" "\x83"  "\x91" "\x83"  "\xC0" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("material loading"),
                                                  PSTR(""),
                                                  PSTR("")));
  lcd_progressbar(progress);
}

/****************************************************************************************
* Insert material first inserting
*
****************************************************************************************/
static void runMaterialForward()
{
  LED_NORMAL();
  //Override the max feedrate and acceleration values to get a better insert speed and speedup/slowdown
  float old_max_feedrate_e = max_feedrate[E_AXIS];
  float old_retract_acceleration = retract_acceleration;
  max_feedrate[E_AXIS] = FILAMENT_INSERT_FAST_SPEED;
  retract_acceleration = FILAMENT_LONG_MOVE_ACCELERATION;

  current_position[E_AXIS] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
  for(uint8_t n=0; n<6; n++)
  {
    current_position[E_AXIS] += FILAMENT_FORWARD_LENGTH  / volume_to_filament_length[active_extruder] / 6;
    //Only move E.
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_FAST_SPEED, 0);
  }

  //Put back origonal values.
  max_feedrate[E_AXIS] = old_max_feedrate_e;
  retract_acceleration = old_retract_acceleration;
}

static void lcd_menu_first_run_material_load_insert()
{
  LED_GLOW();

  if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 2 && !is_command_queued() && !isCommandInBuffer())
  {
    current_position[E_AXIS] -= 9;
    //Only move E.
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_SPEED, active_extruder);
    current_position[E_AXIS] += 10;
    //Only move E.
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_SPEED, active_extruder);
  }

  if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 4 && !is_command_queued() && !isCommandInBuffer()) {
    char buffer[10];
    char* c = buffer;

    int leftTime = FILAMENT_INSERT_TIME - (millis()-menuTimer)/1000;
    leftTime = constrain(leftTime, 0, FILAMENT_INSERT_TIME);

    int_to_string(leftTime, buffer);

    lcd_info_screen(lcd_menu_first_run_material_load_forward, runMaterialForward, LS(PSTR("READY   "),
                                                                                     PSTR("\x9A" "\x80"  "\xDF" "\x80"  "   "),
                                                                                     PSTR("\x9A" "\x83"  "\xA1" "\x83"  "   ")) ,MenuForward);
    lcd_lib_clear_string(65 - LS(strlen_P(PSTR("READY   ")),
                                 strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "   ")),
                                 strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "   ")))  * 3 + 6 * 6, LS(56, 53, 53) , buffer);
    if (leftTime == 0) {
      lcd_change_to_menu(lcd_menu_first_run_material_load_forward);
      runMaterialForward();
    }
  }
  else{
    lcd_info_screen(NULL, NULL, LS(PSTR("WAITING"),
                                   PSTR("\xB3" "\x80"  "\xB4" "\x80"  ),
                                   PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )) ,MenuForward);
    menuTimer = millis();
  }

  DRAW_PROGRESS_NR(6);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Insert material into"),
                                                  PSTR("\xB2" "\x80"  "\x81" "\x81"  "\x82" "\x81"  "\x83" "\x81"  "\xF4" "\x80"  "\x84" "\x81"  "\xF5" "\x80"  "\x85" "\x81"  "\xDD" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA6" "\x83"  "\xA7" "\x83"  " ""\x96" "\x83"  "\x97" "\x83"  "\x90" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("extruder until it is"),
                                                  PSTR("\x86" "\x81"  "\x87" "\x81"  ",""\x8E" "\x80"  "\x88" "\x81"  "\x89" "\x81"  "\x8A" "\x81"  "\x8B" "\x81"  "\x8C" "\x81"  "\x8D" "\x81"  ),
                                                  PSTR("\xA8" "\x83"  "\xA9" "\x83"  "\xAA" "\x83"  "\xAB" "\x83"  "\xC2" "\x83"  "\xAC" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("drived by extruder"),
                                                  PSTR("\x8E" "\x81"  "\x8F" "\x81"  "\x84" "\x81"  "\xF5" "\x80"  ",""\xE2" "\x80"  "OK""\xE4" "\x80"  "\xC6" "\x80"  "\xC7" "\x80"  ),
                                                  PSTR("OK ""\x86" "\x83"  "\x87" "\x83"  " ""\x88" "\x83"  "\x89" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("and then push button."),
                                                  PSTR(""),
                                                  PSTR("")));
}

/****************************************************************************************
* Insert material fast inserting
*
****************************************************************************************/
static void lcd_menu_first_run_material_load_forward()
{
  LED_NORMAL();
  lcd_basic_screen();
  DRAW_PROGRESS_NR(7);

  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Loading material..."),
                                                  PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\xD2" "\x80"  "\xF6" "\x80"  "..."),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA6" "\x83"  "\xA7" "\x83"  "\xC0" "\x82"  )));
  if (!blocks_queued())
  {
    lcd_lib_beep();
    led_glow_dir = led_glow = 0;
  #if MOTOR_CURRENT_PWM_XY_PIN > -1
    digipot_current(2, motor_current_setting[2]*2/3);    //Set E motor power lower so the motor will skip instead of grind.
  #endif
    currentMenu = lcd_menu_first_run_material_load_wait;
    menuTimer = millis();
    SELECT_MAIN_MENU_ITEM(0);
  }

  long pos = st_get_position(E_AXIS);
  long targetPos = lround(FILAMENT_FORWARD_LENGTH*axis_steps_per_unit[E_AXIS]);
  uint8_t progress = (pos * 125 / targetPos);
  lcd_progressbar(progress);
}

/****************************************************************************************
* Insert material last inserting
*
****************************************************************************************/
static void doAfterMaterialLoad()
{
  current_position[E_AXIS] = END_OF_PRINT_RETRACTION / volume_to_filament_length[active_extruder];
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS]);
  current_position[E_AXIS] = 0;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], retract_feedrate/60, 0);
  clearPrimed();
  enquecommand_P(PSTR("M84"));
  setTargetHotend(0, 0);
}

static void lcd_menu_first_run_material_load_wait()
{
  LED_GLOW();

  if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 2 && !is_command_queued() && !isCommandInBuffer()) {
    current_position[E_AXIS] += 0.5;
    //Only move E.
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_EXTRUDE_SPEED, 0);
  }

  if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 3 && !is_command_queued() && !isCommandInBuffer()) {

    char buffer[10];
    char* c = buffer;

    int leftTime = FILAMENT_INSERT_TIME - (millis()-menuTimer)/1000;
    leftTime = constrain(leftTime, 0, FILAMENT_INSERT_TIME);

    int_to_string(leftTime, buffer);

    lcd_info_screen(lcd_menu_first_run_material_PLA, doAfterMaterialLoad, LS(PSTR("READY   "),
                                                                      PSTR("\x9A" "\x80"  "\xDF" "\x80"  "   "),
                                                                      PSTR("\x9A" "\x83"  "\xA1" "\x83"  "   ")) , MenuForward);
    lcd_lib_clear_string(65 - LS(strlen_P(PSTR("READY   ")),
                                 strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "   ")),
                                 strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "   "))) * 3 + 6 * 6, LS(56, 53, 53), buffer);
    if (leftTime == 0) {
      lcd_change_to_menu(lcd_menu_first_run_material_PLA);
      doAfterMaterialLoad();
    }
  }
  else{
    menuTimer = millis();
    lcd_info_screen(NULL, NULL, LS(PSTR("WAITING"),
                                   PSTR("\xB3" "\x80"  "\xB4" "\x80"  "\xC1" "\x80"  ),
                                   PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )) , MenuForward);
  }
  DRAW_PROGRESS_NR(8);

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Push button when"),
                                                  PSTR("\x90" "\x81"  "\x91" "\x81"  "\x92" "\x81"  "\x89" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  "\x90" "\x83"  " ""\xD5" "\x82"  "\xD6" "\x82"  "\xBE" "\x82"  "\xC8" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("material exits"),
                                                  PSTR("\x93" "\x81"  "\xA0" "\x80"  "\xEC" "\x80"  "\x96" "\x80"  "\x94" "\x81"  "\x95" "\x81"  "\x96" "\x81"  "\x8B" "\x81"  ),
                                                  PSTR("\xFC" "\x82"  "\xAD" "\x83"  "\xF3" "\x82"  "\xAE" "\x83"  " ""\xF4" "\x82"  "\xAF" "\x83"  "\xAB" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("from nozzle..."),
                                                  PSTR("\xB2" "\x80"  "\xE2" "\x80"  "\xE3" "\x80"  "OK" "\xE4" "\x80"  ),
                                                  PSTR("OK ""\x86" "\x83"  "\x87" "\x83"  " ""\x88" "\x83"  "\x89" "\x83"  )));
}


/****************************************************************************************
 * Insert material PLA tips
 *
 ****************************************************************************************/
static void lcd_menu_first_run_material_PLA()
{
  LED_GLOW();
  
  lcd_info_screen(lcd_menu_first_run_print, NULL, LS(PSTR("CONTINUE"),
                                 PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                 PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) , MenuForward);
  DRAW_PROGRESS_NR(9);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Default material is"),
                                                  PSTR("\xDA" "\x80"  "\xDB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  "\xFC" "\x81"  "PLA,""\xF3" "\x81"  "\x91" "\x82"  "\xAF" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\x83" "\x83"  "\xB5" "\x84"  " ""\x84" "\x83"  "\x85" "\x83"  " PLA")));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("PLA. Define material"),
                                                  PSTR("\"" "\xE0" "\x80"  "\xE1" "\x80"   ">>" "\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"   "\""),
                                                  PSTR("\"""\x84" "\x83"  "\x85" "\x83"   ">>" "\x96" "\x83"  "\x97" "\x83"   "\"""\xC0" "\x82"  "\xBE" "\x82"  "\xC8" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("in \"Settings >>"),
                                                  PSTR("\xC1" "\x80"  "\xA7" "\x81"  "\xF7" "\x81"  "\xF8" "\x81"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xE9" "\x83"  "\xEA" "\x83"  " ""\x84" "\x83"  "\x85" "\x83"  "\x90" "\x83"  "\xD9" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("Material settings\""),
                                                  PSTR(""),
                                                  PSTR("")));
}

/****************************************************************************************
* Insert material last inserting
*
****************************************************************************************/
static void setFirstRunDone()
{
  SET_FIRST_RUN_DONE();
  lcd_clear_cache();
}

static void lcd_menu_first_run_print()
{
  LED_NORMAL();

  lcd_question_screen(lcd_menu_first_run_print_card_detect, setFirstRunDone, LS(PSTR("CONTINUE"),
                                                                     PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                     PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) , lcd_menu_main, setFirstRunDone,LS(PSTR("SKIP"),
                                                                                                                   PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                                                                                                   PSTR("\xD0" "\x82"  "\x9B" "\x83"  )) , MenuForward, MenuForward);
  lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("I'm ready. let's"),
                                                  PSTR(""),
                                                  PSTR("")));
  lcd_lib_draw_string_centerP(LS(30, 24, 24) , LS(PSTR("make a 3D Print!"),
                                                  PSTR("\x97" "\x81"  "\x98" "\x81"  "!""\x9A" "\x80"  "\xDF" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\x99" "\x81"  "!"),
                                                  PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\x9A" "\x83"  "\xA1" "\x83"  )));
  DRAW_PROGRESS_NR(10);
}

/****************************************************************************************
* card detect
*
****************************************************************************************/
static void lcd_menu_first_run_print_card_detect()
{
  LED_NORMAL();
  if (!card.sdInserted)
  {
    LED_GLOW();
    lcd_info_screen(lcd_menu_main,NULL,NULL, MenuForward);
    DRAW_PROGRESS_NR(11);
    
    lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("Please insert SD-card"),
                                                    PSTR("\xB2" "\x80"  "\x85" "\x81"  "\xDD" "\x80"  "\x9A" "\x81"  "\xD3" "\x80"  ),
                                                    PSTR("SD" "\xEB" "\x82"  "\xEC" "\x82"  "\x98" "\x83"  )));
    lcd_lib_draw_string_centerP(LS(30, 24, 24) , LS(PSTR("that came with"),
                                                    PSTR("\x9B" "\x81"  "\x9C" "\x81"  "\xF4" "\x80"  "SD" "\x9E" "\x80"  ),
                                                    PSTR("\xA6" "\x83"  "\xA7" "\x83"  "\xB0" "\x83"  "\x92" "\x83"  "\xF9" "\x82"  "\xFA" "\x82"  )));
    lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("your OverLord..."),
                                                    PSTR(""),
                                                    PSTR("")));
    return;
  }

  if (!card.isOk())
  {
    lcd_info_screen(lcd_menu_main,NULL,NULL, MenuForward);
    DRAW_PROGRESS_NR(11);
    lcd_lib_draw_string_centerP(LS(30, 24, 24) , LS(PSTR("Reading card..."),
                                                    PSTR("SD" "\x9E" "\x80"  "\x9D" "\x81"  "\xD8" "\x80"  "\xC1" "\x80"  "..."),
                                                    PSTR("Reading card...")));
    return;
  }
  
  lcd_info_screen(lcd_menu_print_select, NULL, LS(PSTR("LET'S PRINT"),
                                                             PSTR("\x9A" "\x80"  "\xDF" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "!"),
                                                             PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\x9A" "\x83"  "\xA1" "\x83"  )) , MenuForward);
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Select a print file"),
                                                  PSTR("\x93" "\x81"  "SD""\x9E" "\x80"  "\xC1" "\x80"  "\x9E" "\x81"  "\x9F" "\x81"  "\xA0" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  ),
                                                  PSTR("SD""\xEB" "\x82"  "\xEC" "\x82"  "\xC0" "\x82"  "\xBE" "\x82"  "\xC8" "\x82"  " ""\xED" "\x82"  "\xEE" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("on the SD-card"),
                                                  PSTR("\xF4" "\x80"  "\xC2" "\x80"  "\xC3" "\x80"  ",""\xA1" "\x81"  "\xCE" "\x80"  "\xE2" "\x80"  "\xE3" "\x80"  "OK""\xE4" "\x80"  ),
                                                  PSTR("\xB1" "\x83"  "\xDA" "\x82"  "\xB2" "\x83"  " OK ""\x86" "\x83"  "\x87" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("and press the button"),
                                                  PSTR("\x9A" "\x80"  "\xDF" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "!"),
                                                  PSTR("\x88" "\x83"  "\x89" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  " ""\xA9" "\x83"  "\xA7" "\x84"  " ""\x9A" "\x83"  "\xA1" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("to print it!"),
                                                  PSTR(""),
                                                  PSTR("")));
  DRAW_PROGRESS_NR(11);
}
#endif//ENABLE_ULTILCD2

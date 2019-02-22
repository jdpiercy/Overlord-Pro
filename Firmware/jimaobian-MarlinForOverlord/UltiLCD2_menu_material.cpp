#include <avr/pgmspace.h>

#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
  #include "Marlin.h"
  #include "cardreader.h"//This code uses the card.longFilename as buffer to store data, to save memory.
  #include "temperature.h"
  #include "UltiLCD2.h"
  #include "UltiLCD2_hi_lib.h"
  #include "UltiLCD2_menu_material.h"
  #include "UltiLCD2_menu_print.h"
  #include "UltiLCD2_menu_maintenance.h"
#include "stepper.h"

  #ifndef eeprom_read_float
//Arduino IDE compatibility, lacks the eeprom_read_float function
float inline eeprom_read_float(float* addr)
{
  union { uint32_t i; float f; } n;
  n.i = eeprom_read_dword((uint32_t*)addr);
  return n.f;
}
void inline eeprom_write_float(float* addr, float f)
{
  union { uint32_t i; float f; } n;
  n.f = f;
  eeprom_write_dword((uint32_t*)addr, n.i);
}
  #endif

struct materialSettings material[EXTRUDERS] = {0,0,0,0.0};

void doCooldown();//TODO
static void lcd_menu_material_main();
void lcd_menu_change_material_preheat();
static void lcd_menu_change_material_remove();
static void lcd_menu_change_material_remove_wait_user();
static void lcd_menu_change_material_remove_wait_user_ready();
static void lcd_menu_change_material_insert_wait_user();
static void lcd_menu_change_material_insert_wait_user_ready();
static void lcd_menu_change_material_insert_forward();
static void lcd_menu_change_material_insert();
static void lcd_menu_change_material_select_material();
void lcd_menu_material_select();
static void lcd_menu_material_selected();
static void lcd_menu_material_settings();
void lcd_material_clean_nozzle_option();
void lcd_material_clean_nozzle_remove_option();
void lcd_material_clean_nozzle_check_option();
void lcd_material_clean_nozzle_open();
void lcd_material_clean_nozzle_prepare();
void lcd_material_clean_nozzle_heat();
void lcd_material_clean_nozzle_heated();
void lcd_material_clean_nozzle_cool();
void lcd_material_clean_nozzle_cooled();
void lcd_material_clean_nozzle_repeat();
  

static void lcd_menu_select_skip_remove();
static void lcd_menu_change_material_remove_process();

bool isReload;

static void finishMaterialInsert()
{
  enquecommand_P("M84");

  #if MOTOR_CURRENT_PWM_XY_PIN > -1
  digipot_current(2, motor_current_setting[2]);//Set E motor power to default.
  #endif
  
  if (isReload) {
    nextEncoderPos=0;
  }
  else{
    nextEncoderPos=2;
  }

  doCooldown();
  #ifdef FilamentDetection
  resumeState&= ~RESUME_STATE_FILAMENT;
  #endif
}

static void cancelMaterialInsert()
{
  quickStop();
  finishMaterialInsert();
}


void lcd_menu_change_material_preheat()
{
  LED_GLOW_HEAT();
  int16_t temp = degHotend(active_extruder) - 20;
  int16_t target = degTargetHotend(active_extruder) - 20 - 10;
  if (temp < 0) temp = 0;
  if (temp > target && millis()-menuTimer>800 && !is_command_queued() && !isCommandInBuffer())
  {
    if (isReload) {
      lcd_change_to_menu(lcd_menu_select_skip_remove,MAIN_MENU_ITEM_POS(0),MenuForward);
    }
    else{
      setTargetHotend(0, active_extruder);
      fanSpeed=255;
      lcd_menu_change_material_remove_process();
      lcd_change_to_menu(lcd_menu_change_material_remove,MAIN_MENU_ITEM_POS(0),MenuForward);
    }
    
    menuTimer = millis();
    temp = target;
  }

  uint8_t progress = uint8_t(temp * 125 / target);
  if (progress < minProgress)
    progress = minProgress;
  else
    minProgress = progress;
  lcd_info_screen(lcd_menu_maintenance, cancelMaterialInsert);

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Please wait,"),
                                                  PSTR("\xB2" "\x80"  "\xB3" "\x80"  "\xB4" "\x80"  ","),
                                                  PSTR("Please wait")) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("printhead heating for"),
                                                  PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\x90" "\x80"  "\x8B" "\x80"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"   "\x90" "\x83"  "\x91" "\x83"  "\xC0" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("material loading"),
                                                  PSTR(""),
                                                  PSTR("")) );

  lcd_progressbar(progress);
}

static void lcd_menu_select_skip_remove()
{
  LED_NORMAL();
  
  if (printing_state == PRINT_STATE_NORMAL && movesplanned() ==0 && !is_command_queued() && !isCommandInBuffer()) {
    
    char buffer[10];
    char* c = buffer;
    
    int leftTime = FILAMENT_HEATING_WAIT_TIME - (millis()-menuTimer)/1000;
    leftTime = constrain(leftTime, 0, FILAMENT_HEATING_WAIT_TIME);
    
    int_to_string(leftTime, buffer);
    
    lcd_question_screen(lcd_menu_change_material_remove,lcd_menu_change_material_remove_process,LS(PSTR("READY"),
                                                                                                   PSTR("\x9A" "\x80"  "\xDF" "\x80"  ),
                                                                                                   PSTR("\x9A" "\x83"  "\xA1" "\x83"  )) ,
                        lcd_menu_change_material_insert_wait_user,lcd_menu_change_material_remove_wait_user_ready,LS(PSTR("SKIP   "),
                                                                                                                     PSTR("\xAC" "\x80"  "\xAD" "\x80"  "    "),
                                                                                                                     PSTR("\xD0" "\x82"  "\x9B" "\x83"  "    ")) ,MenuForward,MenuForward);


    if (IS_SELECTED_MAIN(0)) {
      lcd_lib_draw_string(34 + 61 - LS(strlen_P(PSTR("SKIP   ")),
                                   strlen_P(PSTR("\xAC" "\x80"  "\xAD" "\x80"  "    ")),
                                   strlen_P(PSTR("\xD0" "\x82"  "\x9B" "\x83"  "    ")))  * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }else{
      lcd_lib_clear_string(34 + 61 - LS(strlen_P(PSTR("SKIP   ")),
                                  strlen_P(PSTR("\xAC" "\x80"  "\xAD" "\x80"  "    ")),
                                  strlen_P(PSTR("\xD0" "\x82"  "\x9B" "\x83"  "    "))) * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }
    
    if (leftTime == 0) {
      lcd_change_to_menu(lcd_menu_maintenance);
      cancelMaterialInsert();
    }
  }
  else{
    lcd_question_screen(NULL, NULL, LS(PSTR("WAITING"),
                                       PSTR("\xB3" "\x80"  "\xB4" "\x80"  ),
                                       PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )) ,
                        lcd_menu_change_material_insert_wait_user, lcd_menu_change_material_remove_wait_user_ready, LS(PSTR("SKIP"),
                                                                                                                       PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                                                                                                       PSTR("\xD0" "\x82"  "\x9B" "\x83"  )) ,MenuForward,MenuForward);
  }
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Heating finished."),
                                                  PSTR("\x90" "\x80"  "\x8B" "\x80"  "\xE4" "\x81"  "\xE5" "\x81"  ),
                                                  PSTR("\x90" "\x83"  "\x91" "\x83"  "\xDD" "\x83"  "\x97" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("The material will"),
                                                  PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  "\x81" "\x81"  "\xE6" "\x81"  "\xC9" "\x81"  "\x8B" "\x81"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xE0" "\x83"  "\x97" "\x83"  "\xAA" "\x83"  "\xE1" "\x83"  "\xA7" "\x83"  "\x9C" "\x83"  "\x9D" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("be removed."),
                                                  PSTR(""),
                                                  PSTR("")) );
}

static void lcd_menu_change_material_remove_process()
{
  //    set_extrude_min_temp(0);
  for(uint8_t e=0; e<EXTRUDERS; e++)
    volume_to_filament_length[e] = 1.0; //Set the extrusion to 1mm per given value, so we can move the filament a set distance.

  current_position[E_AXIS] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  current_position[E_AXIS]=20.0;
  //Only move E.
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], (1000/60), active_extruder);

  float old_max_feedrate_e = max_feedrate[E_AXIS];
  float old_retract_acceleration = retract_acceleration;
  max_feedrate[E_AXIS] = FILAMENT_REVERSAL_SPEED;
  retract_acceleration = FILAMENT_LONG_MOVE_ACCELERATION;

  current_position[E_AXIS] = 0;
  //Only move E.
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], (1000/60), active_extruder);
  for(uint8_t n=0; n<6; n++)
  {
    current_position[E_AXIS] -= FILAMENT_REVERSAL_LENGTH/6;
    //Only move E.
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_REVERSAL_SPEED, active_extruder);
  }
  max_feedrate[E_AXIS] = old_max_feedrate_e;
  retract_acceleration = old_retract_acceleration;

  //    currentMenu = lcd_menu_select_skip_remove;
}


static void lcd_menu_change_material_remove()
{
  LED_NORMAL();

  if (!blocks_queued())
  {
    lcd_lib_beep();
    led_glow_dir = led_glow = 0;
    if (isReload) {
      currentMenu = lcd_menu_change_material_remove_wait_user;
    }
    else{
      currentMenu = lcd_material_clean_nozzle_check_option;
    }
    menuTimer=millis();
    SELECT_MAIN_MENU_ITEM(0);
    //Disable the extruder motor so you can pull out the remaining filament.
    disable_e0();
    disable_e1();
    disable_e2();
  }

  lcd_info_screen(lcd_menu_maintenance, cancelMaterialInsert, NULL, MenuBackward);

  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Reversing material"),
                                                  PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\xAE" "\x80"  "\xAF" "\x80"  "\xC9" "\x81"  "\x8B" "\x81"  ),
                                                  PSTR("\xE0" "\x83"  "\x97" "\x83"  "\xC0" "\x82"  )) );
  
  long pos = -st_get_position(E_AXIS);
  long targetPos = lround(FILAMENT_REVERSAL_LENGTH*axis_steps_per_unit[E_AXIS]);
  pos = constrain(pos, 0, targetPos);
  uint8_t progress = (pos * 125 / targetPos);
  lcd_progressbar(progress);
}

static void lcd_menu_change_material_remove_wait_user_ready()
{
  current_position[E_AXIS] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
  menuTimer = millis();
}

static void lcd_menu_change_material_remove_wait_user()
{



  LED_GLOW();
  if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 4 && !is_command_queued() && !isCommandInBuffer()) {

    char buffer[10];
    char* c = buffer;

    int leftTime = FILAMENT_INSERT_TIME - (millis()-menuTimer)/1000;
    leftTime = constrain(leftTime, 0, FILAMENT_INSERT_TIME);

    int_to_string(leftTime, buffer);

    lcd_question_screen(lcd_menu_change_material_insert_wait_user, lcd_menu_change_material_remove_wait_user_ready, LS(PSTR("READY   "),
                                                                                                                       PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    "),
                                                                                                                       PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    ")) ,
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
    
    if (IS_SELECTED_MAIN(0))
    {
      lcd_lib_clear_string(34 - LS(strlen_P(PSTR("READY   ")),
                                   strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    ")),
                                   strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    ")))  * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }else{
      lcd_lib_draw_string(34 - LS(strlen_P(PSTR("READY   ")),
                                  strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    ")),
                                  strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    "))) * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }

    if (leftTime == 0) {
      lcd_change_to_menu(lcd_menu_change_material_insert_wait_user);
      lcd_menu_change_material_remove_wait_user_ready();
    }
  }
  else{
    lcd_question_screen(NULL, NULL, LS(PSTR("WAITING"),
                                       PSTR("\xB3" "\x80"  "\xB4" "\x80"  ),
                                       PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )) ,
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
    menuTimer = millis();
  }

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Please remove"),
                                                  PSTR("\xB2" "\x80"  "\x93" "\x81"  "\x8A" "\x81"  "\x8B" "\x81"  "\xD3" "\x80"  "\xE7" "\x81"  "\xE8" "\x81"  ),
                                                  PSTR("\xA8" "\x83"  "\xA9" "\x83"  "\x83" "\x83"  "\xE2" "\x83"  "\xE3" "\x83"  "\xF5" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("material from"),
                                                  PSTR("\xE9" "\x81"  "\x8B" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xE4" "\x83"  "\xC8" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("extruder."),
                                                  PSTR(""),
                                                  PSTR("")) );

  #ifdef FilamentDetection

  if (isFilamentDetectionEnable && (currentMenu==lcd_menu_change_material_remove_wait_user)) {
    static unsigned long FilamentDetectionTimer=millis();
    if (READ(FilamentDetectionPin)) {
      if (millis()-FilamentDetectionTimer>500) {
        lcd_menu_change_material_remove_wait_user_ready();
        lcd_change_to_menu(lcd_menu_change_material_insert_wait_user,0,MenuForward);
      }
    }
    else{
      FilamentDetectionTimer=millis();
    }
  }

  #endif
}

static void lcd_menu_change_material_insert_wait_user()
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
    
    lcd_question_screen(lcd_menu_change_material_insert_forward, lcd_menu_change_material_insert_wait_user_ready, LS(PSTR("READY   "),
                                                                                                                     PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    "),
                                                                                                                     PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    ")) ,
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
    if (IS_SELECTED_MAIN(0)) {
      lcd_lib_clear_string(34 - LS(strlen_P(PSTR("READY   ")),
                                   strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    ")),
                                   strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    ")))  * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }else{
      lcd_lib_draw_string(34 - LS(strlen_P(PSTR("READY   ")),
                                  strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    ")),
                                  strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    "))) * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }

    if (leftTime == 0) {
      lcd_change_to_menu(lcd_menu_change_material_insert_forward);
      lcd_menu_change_material_insert_wait_user_ready();
    }

  }
  else{
    lcd_question_screen(NULL, NULL, LS(PSTR("WAITING"),
                                       PSTR("\xB3" "\x80"  "\xB4" "\x80"  "\xC1" "\x80"  ),
                                       PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )) ,
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
    menuTimer = millis();
  }


  #ifdef FilamentDetection
  if (isFilamentDetectionEnable && (currentMenu==lcd_menu_change_material_insert_wait_user)) {
    static unsigned long FilamentDetectionTimer=millis();
    if (FilamentAvailable()) {
      if (millis()-FilamentDetectionTimer>5000) {
        lcd_menu_change_material_insert_wait_user_ready();
        lcd_change_to_menu(lcd_menu_change_material_insert_forward,0,MenuForward);
      }
    }
    else{
      FilamentDetectionTimer=millis();
    }
  }
  #endif

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Insert material into"),
                                                  PSTR("\xB2" "\x80"  "\x81" "\x81"  "\x82" "\x81"  "\x83" "\x81"  "\xF4" "\x80"  "\x84" "\x81"  "\xF5" "\x80"  "\x85" "\x81"  "\xDD" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA6" "\x83"  "\xA7" "\x83"  " ""\x96" "\x83"  "\x97" "\x83"  "\x90" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("extruder until it is"),
                                                  PSTR("\x86" "\x81"  "\x87" "\x81"  ",""\x8E" "\x80"  "\x88" "\x81"  "\x89" "\x81"  "\x8A" "\x81"  "\x8B" "\x81"  "\x8C" "\x81"  "\x8D" "\x81"  ),
                                                  PSTR("\xA8" "\x83"  "\xA9" "\x83"  "\xAA" "\x83"  "\xAB" "\x83"  "\xC2" "\x83"  "\xAC" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37), LS(PSTR("drived by extruder"),
                                                 PSTR("\x8E" "\x81"  "\x8F" "\x81"  "\x84" "\x81"  "\xF5" "\x80"  ",""\xE2" "\x80"  "OK""\xE4" "\x80"  "\xC6" "\x80"  "\xC7" "\x80"  ),
                                                 PSTR("OK ""\x86" "\x83"  "\x87" "\x83"  " ""\x88" "\x83"  "\x89" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(40, 64, 64), LS(PSTR("and then push button."),
                                                 PSTR(""),
                                                 PSTR("")));
}

static void lcd_menu_change_material_insert_wait_user_ready()
{
  for(uint8_t e=0; e<EXTRUDERS; e++)
    volume_to_filament_length[e] = 1.0; //Set the extrusion to 1mm per given value, so we can move the filament a set distance.

  //Override the max feedrate and acceleration values to get a better insert speed and speedup/slowdown
  float old_max_feedrate_e = max_feedrate[E_AXIS];
  float old_retract_acceleration = retract_acceleration;
  max_feedrate[E_AXIS] = FILAMENT_INSERT_FAST_SPEED;
  retract_acceleration = FILAMENT_LONG_MOVE_ACCELERATION;

  current_position[E_AXIS] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
  for(uint8_t n=0; n<6; n++)
  {
    current_position[E_AXIS] += FILAMENT_FORWARD_LENGTH / 6;
    //Only move E.
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_FAST_SPEED, active_extruder);
  }

  //Put back origonal values.
  max_feedrate[E_AXIS] = old_max_feedrate_e;
  retract_acceleration = old_retract_acceleration;

  //    lcd_change_to_menu(lcd_menu_change_material_insert_forward);
}

static void lcd_menu_change_material_insert_forward()
{
  LED_NORMAL();

  if (!blocks_queued())
  {
    lcd_lib_beep();
    led_glow_dir = led_glow = 0;
  #if MOTOR_CURRENT_PWM_XY_PIN > -1

    digipot_current(2, motor_current_setting[2]*2/3);//Set the E motor power lower to we skip instead of grind.
  #endif
    currentMenu = lcd_menu_change_material_insert;
    menuTimer = millis();
    SELECT_MAIN_MENU_ITEM(0);
  }

  lcd_info_screen(lcd_menu_main, cancelMaterialInsert);

  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Forwarding material"),
                                                  PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\xD2" "\x80"  "\xF6" "\x80"  "..."),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA6" "\x83"  "\xA7" "\x83"  "\xC0" "\x82"  )) );
  
  long pos = st_get_position(E_AXIS);
  long targetPos = lround(FILAMENT_FORWARD_LENGTH*axis_steps_per_unit[E_AXIS]);
  uint8_t progress = (pos * 125 / targetPos);
  lcd_progressbar(progress);
}

static void materialInsertReady()
{
  current_position[E_AXIS] -= END_OF_PRINT_RETRACTION;

  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 25, active_extruder);
  
  // no longer primed
  clearPrimed();
  
  finishMaterialInsert();
}

static void lcd_menu_change_material_insert()
{
  LED_GLOW();


  if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 2 && !is_command_queued() && !isCommandInBuffer())
  {
    current_position[E_AXIS] += 0.5;
    //Only move E.
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_EXTRUDE_SPEED, active_extruder);
  }

  #ifdef FilamentDetection

  if ((resumeState&RESUME_STATE_FILAMENT)==RESUME_STATE_FILAMENT) {
    lcd_question_screen(lcd_menu_print_resume_manual, materialInsertReady, LS(PSTR("READY"),
                                                                              PSTR("\x9A" "\x80"  "\xDF" "\x80"  ),
                                                                              PSTR("\x9A" "\x83"  "\xA1" "\x83"  )) ,
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
  }
  else{
    lcd_question_screen(lcd_menu_change_material_select_material, materialInsertReady, LS(PSTR("READY"),
                                                                                          PSTR("\x9A" "\x80"  "\xDF" "\x80"  ),
                                                                                          PSTR("\x9A" "\x83"  "\xA1" "\x83"  )) ,
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )));
  }

  #else

  if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 3 && !is_command_queued() && !isCommandInBuffer()) {
    char buffer[10];
    char* c = buffer;

    int leftTime = FILAMENT_INSERT_TIME - (millis()-menuTimer)/1000;
    leftTime = constrain(leftTime, 0, FILAMENT_INSERT_TIME);

    int_to_string(leftTime, buffer);

    lcd_question_screen(lcd_menu_change_material_select_material, materialInsertReady, LS(PSTR("READY   "),
                                                                                            PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    "),
                                                                                            PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    ")),
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                         PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                         PSTR("\xFF" "\x82"  "\xC6" "\x82"  )));

    
    if (IS_SELECTED_MAIN(0)) {
      lcd_lib_clear_string(34 - LS(strlen_P(PSTR("READY   ")),
                                   strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    ")),
                                   strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    "))) * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }else{
      lcd_lib_draw_string(34 - LS(strlen_P(PSTR("READY   ")),
                                  strlen_P(PSTR("\x9A" "\x80"  "\xDF" "\x80"  "    ")),
                                  strlen_P(PSTR("\x9A" "\x83"  "\xA1" "\x83"  "    "))) * 3 + 6 * 6, LS(56, 53, 53), buffer);
    }

    if (leftTime == 0) {
      lcd_change_to_menu(lcd_menu_change_material_select_material);
      materialInsertReady();
    }
  }
  else{
    lcd_question_screen(NULL, NULL, LS(PSTR("WAITING"),
                                       PSTR("\xB3" "\x80"  "\xB4" "\x80"  "\xC1" "\x80"  ),
                                       PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )),
                        lcd_menu_maintenance, cancelMaterialInsert, LS(PSTR("CANCEL"),
                                                                       PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                       PSTR("\xFF" "\x82"  "\xC6" "\x82"  )));
    menuTimer = millis();
  }

  #endif

  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Push button when"),
                                                  PSTR("\x90" "\x81"  "\x91" "\x81"  "\x92" "\x81"  "\x89" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  "\x90" "\x83"  " ""\xD5" "\x82"  "\xD6" "\x82"  "\xBE" "\x82"  "\xC8" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("material exits"),
                                                  PSTR("\x93" "\x81"  "\xA0" "\x80"  "\xEC" "\x80"  "\x96" "\x80"  "\x94" "\x81"  "\x95" "\x81"  "\x96" "\x81"  "\x8B" "\x81"  ),
                                                  PSTR("\xFC" "\x82"  "\xAD" "\x83"  "\xF3" "\x82"  "\xAE" "\x83"  " ""\xF4" "\x82"  "\xAF" "\x83"  "\xAB" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("from nozzle..."),
                                                  PSTR("\xB2" "\x80"  "\xE2" "\x80"  "\xE3" "\x80"  "OK" "\xE4" "\x80"  ),
                                                  PSTR("OK ""\x86" "\x83"  "\x87" "\x83"  " ""\x88" "\x83"  "\x89" "\x83"  )) );
}

static char* lcd_menu_change_material_select_material_callback(uint8_t nr)
{
  eeprom_read_block(card.longFilename, EEPROM_MATERIAL_NAME_OFFSET(nr), 8);
  card.longFilename[8] = '\0';
  return card.longFilename;
}

static void lcd_menu_change_material_select_material_details_callback(uint8_t nr)
{
  char buffer[34];
  char* c = buffer;

  if (led_glow_dir)
  {
    c = float_to_string(eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr)), c, PSTR("mm"));
    while(c < buffer + 10) *c++ = ' ';
    strcpy_P(c, LS(PSTR("Flow:"),
                   PSTR("\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  ":"),
                   PSTR("\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  ":")) );
    c += strlen_P(LS(PSTR("Flow:"),
                     PSTR("\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  ":"),
                     PSTR("\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  ":")));
    c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(nr)), c, PSTR("%"));
  }else{
    c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr)), c, PSTR("`C"));
    *c++ = ' ';
    c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr)), c, PSTR("`C"));
    while(c < buffer + 10) *c++ = ' ';
    strcpy_P(c, LS(PSTR(" Fan:"),
                   PSTR(" ""\xC0" "\x81"  "\xC3" "\x81"  ":"),
                   PSTR(" ""\x93" "\x84"  "\xD3" "\x83"  ":")) );
    c += strlen_P(LS(PSTR(" Fan:"),
                     PSTR(" ""\xC0" "\x81"  "\xC3" "\x81"  ":"),
                     PSTR(" ""\x93" "\x84"  "\xD3" "\x83"  ":")) );
    c = int_to_string(eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr)), c, PSTR("%"));
  }
  lcd_draw_detail(buffer);
}

static void lcd_menu_change_material_select_material()
{
  LED_NORMAL();
  uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());

  lcd_advance_menu(LS(PSTR("MATERIAL"),
                      PSTR("\xF5" "\x80"  "\xF6" "\x80"  ),
                      PSTR("\x96" "\x83"  "\x97" "\x83"  )), count, lcd_menu_change_material_select_material_callback, lcd_menu_change_material_select_material_details_callback);

  if (lcd_lib_button_pressed)
  {
    lcd_material_set_material(SELECTED_SCROLL_MENU_ITEM(), active_extruder);

    lcd_change_to_menu(lcd_menu_maintenance, MAIN_MENU_ITEM_POS(0));
  }
}

static void lcd_menu_material_export_done()
{
  lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);

  lcd_info_screen(lcd_menu_material_select, NULL, LS(PSTR("OK"),
                                                     PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                     PSTR("OK")) );
  lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("Saved materials"),
                                                  PSTR("\xAF" "\x80"  "SD" "\x9E" "\x80"  "MATERIAL.TXT "),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xE9" "\x83"  "\xEA" "\x83"  "\xBE" "\x83"  "SD" "\xEB" "\x82"  "\xEC" "\x82"  "\xF5" "\x82"   )) );
  lcd_lib_draw_string_centerP(LS(30, 24, 24) , LS(PSTR("to the SD card"),
                                                  PSTR("\xC1" "\x80"  "\xA2" "\x80"  "\xED" "\x81"  "\xF4" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  "\xEB" "\x81"  "\xEC" "\x81"  ),
                                                  PSTR("MATERIAL.TXT""\xB4" "\x83"  " ""\xD6" "\x83"  "\xCC" "\x82"   )) );
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("in MATERIAL.TXT"),
                                                  PSTR(""),
                                                  PSTR("")) );
}

static void lcd_menu_material_export()
{
  if (!card.sdInserted)
  {
    LED_GLOW();
    lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);
    lcd_info_screen(lcd_menu_material_select);
    
    lcd_lib_draw_string_centerP(LS(15, 11, 11) , LS(PSTR("No SD-CARD!"),
                                                    PSTR("\xC6" "\x81"  "\xCC" "\x81"  "\xCD" "\x81"  "\x89" "\x81"  "SD" "\x9E" "\x80"  ),
                                                    PSTR("No SD-CARD")) );
    lcd_lib_draw_string_centerP(25, LS(PSTR("Please insert card"),
                                       PSTR("\xB2" "\x80"  "\x85" "\x81"  "\xDD" "\x80"  "SD" "\x9E" "\x80"  ),
                                       PSTR("SD ""\xEB" "\x82"  "\xEC" "\x82"  " ""\xA6" "\x83"  "\xA7" "\x83"  )) );
    return;
  }
  if (!card.isOk())
  {
    lcd_info_screen(lcd_menu_material_select);
    lcd_lib_draw_string_centerP(LS(16, 24, 24) , LS(PSTR("Reading card..."),
                                                    PSTR("SD" "\x9E" "\x80"  "\x9D" "\x81"  "\xD8" "\x80"  "\xC1" "\x80"  "..."),
                                                    PSTR("Reading card...")) );
    return;
  }

  card.setroot();
  card.openFile("MATERIAL.TXT", false);
  uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
  for(uint8_t n=0; n<count; n++)
  {
    char buffer[32];
    strcpy_P(buffer, PSTR("[material]\n"));
    card.write_string(buffer);

    strcpy_P(buffer, PSTR("name="));
    char* ptr = buffer + strlen(buffer);
    eeprom_read_block(ptr, EEPROM_MATERIAL_NAME_OFFSET(n), 8);
    ptr[8] = '\0';
    strcat_P(buffer, PSTR("\n"));
    card.write_string(buffer);

    strcpy_P(buffer, PSTR("temperature="));
    ptr = buffer + strlen(buffer);
    int_to_string(eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(n)), ptr, PSTR("\n"));
    card.write_string(buffer);

    strcpy_P(buffer, PSTR("bed_temperature="));
    ptr = buffer + strlen(buffer);
    int_to_string(eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(n)), ptr, PSTR("\n"));
    card.write_string(buffer);

    strcpy_P(buffer, PSTR("fan_speed="));
    ptr = buffer + strlen(buffer);
    int_to_string(eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(n)), ptr, PSTR("\n"));
    card.write_string(buffer);

    strcpy_P(buffer, PSTR("flow="));
    ptr = buffer + strlen(buffer);
    int_to_string(eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(n)), ptr, PSTR("\n"));
    card.write_string(buffer);

    strcpy_P(buffer, PSTR("diameter="));
    ptr = buffer + strlen(buffer);
    float_to_string(eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(n)), ptr, PSTR("\n\n"));
    card.write_string(buffer);
  }
  card.closefile();

  currentMenu = lcd_menu_material_export_done;
}

static void lcd_menu_material_import_done()
{
  lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);

  lcd_info_screen(lcd_menu_material_select, NULL, LS(PSTR("OK"),
                                                     PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                     PSTR("OK")) );
  lcd_lib_draw_string_centerP(LS(11, 20, 20) , LS(PSTR("Loaded materials"),
                                                  PSTR("\x93" "\x81"  "SD" "\x9E" "\x80"  "\xC1" "\x80"  "\x9D" "\x81"  "\xD8" "\x80"  ),
                                                  PSTR("SD""\xEB" "\x82"  "\xEC" "\x82"  "\xC0" "\x82"  "\xBE" "\x82"  "\xC8" "\x82"   )) );
  lcd_lib_draw_string_centerP(LS(30, 24, 24) , LS(PSTR("from the SD card"),
                                                  PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\xEB" "\x81"  "\xEC" "\x81"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xE9" "\x83"  "\xEA" "\x83"  "\xBE" "\x83"  " ""\xB6" "\x83"  "\xEE" "\x83"  "\xC7" "\x83"  "\x9D" "\x83"  )) );
}

static void lcd_menu_material_import()
{
  if (!card.sdInserted)
  {
    LED_GLOW();
    lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);
    lcd_info_screen(lcd_menu_material_select);

    lcd_lib_draw_string_centerP(LS(15, 11, 11) , LS(PSTR("No SD-CARD!"),
                                                    PSTR("\xC6" "\x81"  "\xCC" "\x81"  "\xCD" "\x81"  "\x89" "\x81"  "SD" "\x9E" "\x80"  ),
                                                    PSTR("No SD-CARD")) );
    lcd_lib_draw_string_centerP(LS(25, 24, 24) , LS(PSTR("Please insert card"),
                                                    PSTR("\xB2" "\x80"  "\x85" "\x81"  "\xDD" "\x80"  "SD" "\x9E" "\x80"  ),
                                                    PSTR("SD ""\xEB" "\x82"  "\xEC" "\x82"  " ""\xA6" "\x83"  "\xA7" "\x83"  )) );
    return;
  }
  if (!card.isOk())
  {
    lcd_info_screen(lcd_menu_material_select);
    
    lcd_lib_draw_string_centerP(16, LS(PSTR("Reading card..."),
                                       PSTR("SD" "\x9E" "\x80"  "\x9D" "\x81"  "\xD8" "\x80"  "\xC1" "\x80"   "..."),
                                       PSTR("Reading card...")) );
    return;
  }

  card.setroot();
  card.openFile("MATERIAL.TXT", true);
  if (!card.isFileOpen())
  {
    lcd_info_screen(lcd_menu_material_select);

    lcd_lib_draw_string_centerP(LS(15, 11, 11) , LS(PSTR("No import file"),
                                                    PSTR("\xC6" "\x81"  "\xAF" "\x80"  "SD""\x9E" "\x80"  "\xD2" "\x80"  "\xF2" "\x81"  "\x89" "\x81"  ),
                                                    PSTR("SD""\xEB" "\x82"  "\xEC" "\x82"  "\xC0" "\x82"  "\xBE" "\x82"  )) );
    lcd_lib_draw_string_centerP(LS(25, 24, 24) , LS(PSTR("Found on card."),
                                                    PSTR("\xF3" "\x81"  "\xF4" "\x81"  "\xF5" "\x81"  "\xF6" "\x81"  "\xD6" "\x81"  ),
                                                    PSTR("\xB6" "\x83"  "\xEE" "\x83"  "\xEF" "\x83"  " ""\xF0" "\x83"  "\xF1" "\x83"  "\xC9" "\x83"  " ""\xF2" "\x83"  "\xC1" "\x83"  "\x9C" "\x83"  "\x9D" "\x83"  )) );
    return;
  }

  char buffer[32];
  uint8_t count = 0xFF;
  while(card.fgets(buffer, sizeof(buffer)) > 0)
  {
    buffer[sizeof(buffer)-1] = '\0';
    char* c = strchr(buffer, '\n');
    if (c) *c = '\0';

    if(strcmp_P(buffer, PSTR("[material]")) == 0)
    {
      count++;
    }else if (count < EEPROM_MATERIAL_SETTINGS_MAX_COUNT)
    {
      c = strchr(buffer, '=');
      if (c)
      {
        *c++ = '\0';
        if (strcmp_P(buffer, PSTR("name")) == 0)
        {
          eeprom_write_block(c, EEPROM_MATERIAL_NAME_OFFSET(count), 8);
        }else if (strcmp_P(buffer, PSTR("temperature")) == 0)
        {
          eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(count), strtol(c, NULL, 10));
        }else if (strcmp_P(buffer, PSTR("bed_temperature")) == 0)
        {
          eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(count), strtol(c, NULL, 10));
        }else if (strcmp_P(buffer, PSTR("fan_speed")) == 0)
        {
          eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(count), strtol(c, NULL, 10));
        }else if (strcmp_P(buffer, PSTR("flow")) == 0)
        {
          eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(count), strtol(c, NULL, 10));
        }else if (strcmp_P(buffer, PSTR("diameter")) == 0)
        {
          eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(count), strtod(c, NULL));
        }
      }
    }
  }
  count++;
  if (count > 0)
  {
    eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), count);
  }
  card.closefile();

  currentMenu = lcd_menu_material_import_done;
}

static char* lcd_material_select_callback(uint8_t nr)
{
  uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
  
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR("Return"),
                                   PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == count+1)
    strcpy_P(card.longFilename, LS(PSTR("Customize"),
                                   PSTR("\xA7" "\x81"  "\xF7" "\x81"  "\xF8" "\x81"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                   PSTR("Customize")) );
  else if (nr == count+2)
    strcpy_P(card.longFilename, LS(PSTR("Load Default"),
                                   PSTR("\xD1" "\x81"  "\xC8" "\x81"  "\xDA" "\x80"  "\xDB" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                                   PSTR("Load Default")) );
  else{
    eeprom_read_block(card.longFilename, EEPROM_MATERIAL_NAME_OFFSET(nr - 1), 8);
    card.longFilename[8] = '\0';
  }
  return card.longFilename;
}

static void lcd_material_select_details_callback(uint8_t nr)
{
  uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
  if (nr == 0)
  {

  }
  else if (nr <= count)
  {
    char buffer[34];
    char* c = buffer;
    nr -= 1;

    if (led_glow_dir)
    {
      c = float_to_string(eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr)), c, PSTR("mm"));
      while(c < buffer + 9) *c++ = ' ';
      strcpy_P(c, LS(PSTR("Flow:"),
                     PSTR("\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  ":"),
                     PSTR("\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  ":")));
      c += strlen_P(LS(PSTR("Flow:"),
                       PSTR("\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  ":"),
                       PSTR("\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  ":")));
      c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(nr)), c, PSTR("%"));
    }else{
      c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr)), c, PSTR("`C"));
      *c++ = ' ';
      c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr)), c, PSTR("`C"));
      while(c < buffer + 9) *c++ = ' ';
      strcpy_P(c, LS(PSTR(" Fan:"),
                     PSTR(" ""\xC0" "\x81"  "\xC3" "\x81"  ":"),
                     PSTR(" ""\x93" "\x84"  "\xD3" "\x83"  ":")));
      c += strlen_P(LS(PSTR(" Fan:"),
                       PSTR(" " "\xC0" "\x81"  "\xC3" "\x81"  ":"),
                       PSTR(" ""\x93" "\x84"  "\xD3" "\x83"  ":")));
      c = int_to_string(eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr)), c, PSTR("%"));
    }
    lcd_draw_detail(buffer);
  }else{
    lcd_draw_detailP(LS(PSTR("Modify the settings"),
                        PSTR("\xA3" "\x81"  "\xF9" "\x81"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                        PSTR("\x84" "\x83"  "\x85" "\x83"  " ""\x93" "\x83"  "\x8E" "\x83"  )) );
  }
}

void lcd_menu_material_select()
{
  LED_NORMAL();

  uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());

  lcd_scroll_menu(LS(PSTR("Material Settings"),
                     PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                     PSTR("\x84" "\x83"  "\x85" "\x83"  " ""\x96" "\x83"  "\x97" "\x83"  )) , count + 3, lcd_material_select_callback, lcd_material_select_details_callback);

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
      lcd_change_to_menu(lcd_menu_advanced_settings, MAIN_MENU_ITEM_POS(2), MenuBackward);
    else if (IS_SELECTED_SCROLL(count + 1))
      lcd_change_to_menu(lcd_menu_material_settings);
    else if (IS_SELECTED_SCROLL(count + 2))
    {
      lcd_material_reset_defaults();
      for(uint8_t e=0; e<EXTRUDERS; e++)
        lcd_material_set_material(0, e);
    }
    else{
      lcd_material_set_material(SELECTED_SCROLL_MENU_ITEM() - 1, active_extruder);
      lcd_change_to_menu(lcd_menu_material_selected, MAIN_MENU_ITEM_POS(0));
    }
  }
}

static void lcd_menu_material_selected()
{
  LED_NORMAL();

  lcd_info_screen(lcd_menu_advanced_settings, NULL, LS(PSTR("OK"),
                                                       PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                       PSTR("OK")) );
  lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("Selected material:"),
                                                  PSTR("\xF5" "\x80"  "\xF6" "\x80"  ":"),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  ":")) );
  lcd_lib_draw_string_center(LS(30, 24, 24) , card.longFilename);
#if EXTRUDERS > 1
  if (active_extruder == 0)
    lcd_lib_draw_string_centerP(LS(40, 37, 37) , LS(PSTR("for primary nozzle"),
                                                    PSTR("\xCF" "\x80"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                                                    PSTR("")) );
  else if (active_extruder == 1)
    lcd_lib_draw_string_centerP(LS(40, 37, 37), LS(PSTR("for secondary nozzle"),
                                                   PSTR("\x93" "\x81"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                                                   PSTR("")) );
#endif
  
  nextEncoderPos=2;
}

static char* lcd_material_settings_callback(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR("Return"),
                                   PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(card.longFilename, LS(PSTR("Temperature"),
                                   PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ),
                                   PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  )) );
  else if (nr == 2  && Device_isBedHeat)
    strcpy_P(card.longFilename, LS(PSTR("Heated buildplate"),
                                   PSTR("\xFB" "\x80"  "\xFC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ),
                                   PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  )) );
  else if (nr == 3 - !Device_isBedHeat)
    strcpy_P(card.longFilename, LS(PSTR("Diameter"),
                                   PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\x9A" "\x82"  "\xF1" "\x81"  ),
                                   PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xED" "\x83"  "\x8E" "\x83"  )) );
  else if (nr == 4 - !Device_isBedHeat)
    strcpy_P(card.longFilename, LS(PSTR("Fan"),
                                   PSTR("\xC0" "\x81"  "\xC1" "\x81"  "\xC3" "\x81"  "\xEA" "\x81"  ),
                                   PSTR("\xD2" "\x83"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) );
  else if (nr == 5 - !Device_isBedHeat)
    strcpy_P(card.longFilename, LS(PSTR("Flow %"),
                                   PSTR("\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  "%"),
                                   PSTR("\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  "%")) );
  else if (nr == 6 - !Device_isBedHeat)
    strcpy_P(card.longFilename, LS(PSTR("Store as preset"),
                                   PSTR("\xA2" "\x80"  "\xED" "\x81"  "\xC5" "\x80"  "\xF9" "\x81"  "\xED" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"  ),
                                   PSTR("\xDC" "\x82"  "\xCE" "\x82"  " ""\x84" "\x83"  "\x85" "\x83"  "\xEF" "\x83"  "\xE1" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  " ""\xD6" "\x83"  "\xCC" "\x82"  )) );
  else
    strcpy_P(card.longFilename, PSTR("???"));

  return card.longFilename;
}

static void lcd_material_settings_details_callback(uint8_t nr)
{
  char buffer[12];
  buffer[0] = '\0';

  if (nr == 0){
    return;
  }
  else if (nr == 1){
    int_to_string(material[active_extruder].temperature, buffer, PSTR("`C"));
  }
  else if (nr == 2){
    int_to_string(material[active_extruder].bed_temperature, buffer, PSTR("`C"));
  }
  else if (nr == 3){
    float_to_string(material[active_extruder].diameter, buffer, PSTR("mm"));
  }
  else if (nr == 4){
    int_to_string(material[active_extruder].fan_speed, buffer, PSTR("%"));
  }
  else if (nr == 5){
    int_to_string(material[active_extruder].flow, buffer, PSTR("%"));
  }

  lcd_draw_detail(buffer);
}

static void lcd_menu_material_settings()
{
  LED_NORMAL();

  lcd_scroll_menu(LS(PSTR("Material Details"),
                     PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\xEB" "\x81"  "\xEC" "\x81"  ),
                     PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xE9" "\x83"  "\xEA" "\x83"  )) , 7 - !Device_isBedHeat, lcd_material_settings_callback, lcd_material_settings_details_callback);
  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
    {
      lcd_change_to_menu(lcd_menu_material_select, MAIN_MENU_ITEM_POS(0), MenuBackward);
      lcd_material_store_current_material();
    }else if (IS_SELECTED_SCROLL(1))
      LCD_EDIT_SETTING(material[active_extruder].temperature, LS(PSTR("Temperature"),
                                                                 PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ),
                                                                 PSTR("\xD5" "\x82"  "\xD6" "\x82"   "\xC4" "\x82"  "\xC5" "\x82"  )) , LS(PSTR("`C"),
                                                                                PSTR("`C"),
                                                                                PSTR("`C")) , 0, HEATER_0_MAXTEMP - 15);
    else if (IS_SELECTED_SCROLL(2) && Device_isBedHeat)
      LCD_EDIT_SETTING(material[active_extruder].bed_temperature, LS(PSTR("Buildplate Temp."),
                                                                     PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ),
                                                                     PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  )) , LS(PSTR("`C"),
                                                                                    PSTR("`C"),
                                                                                    PSTR("`C")), 0, BED_MAXTEMP - 15);
    else if (IS_SELECTED_SCROLL(3 - !Device_isBedHeat))
      LCD_EDIT_SETTING_FLOAT001(material[active_extruder].diameter, LS(PSTR("Material Diameter"),
                                                                       PSTR("\xF0" "\x81"  "\xF5" "\x80"  "\x9A" "\x82"  "\xF1" "\x81"  ),
                                                                       PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xED" "\x83"  "\x8E" "\x83"  )) , LS(PSTR("mm"),
                                                                                      PSTR("mm"),
                                                                                      PSTR("mm")), 0, 100);
    else if (IS_SELECTED_SCROLL(4 - !Device_isBedHeat))
      LCD_EDIT_SETTING(material[active_extruder].fan_speed, LS(PSTR("Fan speed"),
                                                               PSTR("\xC0" "\x81"  "\xC1" "\x81"  "\xC3" "\x81"  "\xEA" "\x81"  ),
                                                               PSTR("\xD2" "\x83"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) , LS(PSTR("%"),
                                                                              PSTR("%"),
                                                                              PSTR("%")), 0, 100);
    else if (IS_SELECTED_SCROLL(5 - !Device_isBedHeat))
      LCD_EDIT_SETTING(material[active_extruder].flow, LS(PSTR("Material flow") ,
                                                          PSTR("\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  ),
                                                          PSTR("\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  )) , LS(PSTR("%"),
                                                                         PSTR("%"),
                                                                         PSTR("%")), 1, 1000);
    else if (IS_SELECTED_SCROLL(6 - !Device_isBedHeat))
    {
      uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
      if (count == EEPROM_MATERIAL_SETTINGS_MAX_COUNT)
        count--;
      char buffer[9] = "CUSTOM";
      int_to_string(count, buffer + 6);
      eeprom_write_block(buffer, EEPROM_MATERIAL_NAME_OFFSET(count), 8);
      eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), count + 1);
      lcd_material_store_material(count);
      lcd_material_store_current_material();
      lcd_change_to_menu(lcd_menu_material_select,count+1,MenuBackward);
    }
  }
}

void lcd_material_reset_defaults()
{
  //Fill in the defaults
  char buffer[8];

  strcpy_P(buffer, PSTR("PLA"));
  eeprom_write_block(buffer, EEPROM_MATERIAL_NAME_OFFSET(0), 4);
  eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(0), 210);
  if (Device_isBedHeat) {
    if (Device_isGate) {
      eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(0), 35);
    }
    else{
      eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(0), 60);
    }
  }
  else{
    eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(0), 0);
  }
  eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(0), 100);
  eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(0), 100);
  eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(0), 1.75);

  if (Device_isABS) {
    strcpy_P(buffer, PSTR("ABS"));
    eeprom_write_block(buffer, EEPROM_MATERIAL_NAME_OFFSET(1), 4);
    eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(1), 260);
    if (Device_isBedHeat) {
      eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(1), 75);
    }
    else{
      eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(1), 0);
    }
    eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(1), 50);
    eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(1), 107);
    eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(1), 1.75);
    eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), 2);
  }
  else{
    eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), 1);
  }
}

void lcd_material_set_material(uint8_t nr, uint8_t e)
{
  material[e].temperature = eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr));
  material[e].bed_temperature = eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr));
  material[e].flow = eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(nr));

  material[e].fan_speed = eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr));
  material[e].diameter = eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr));
  eeprom_read_block(card.longFilename, EEPROM_MATERIAL_NAME_OFFSET(nr), 8);
  card.longFilename[8] = '\0';
  if (material[e].temperature > HEATER_0_MAXTEMP - 15)
    material[e].temperature = HEATER_0_MAXTEMP - 15;
  if (Device_isBedHeat) {
    if (material[e].bed_temperature > BED_MAXTEMP - 15)
      material[e].bed_temperature = BED_MAXTEMP - 15;
  }
  lcd_material_store_current_material();
}

void lcd_material_store_material(uint8_t nr)
{
  eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr), material[active_extruder].temperature);
  eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr), material[active_extruder].bed_temperature);
  eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(nr), material[active_extruder].flow);

  eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr), material[active_extruder].fan_speed);
  eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr), material[active_extruder].diameter);
  //eeprom_write_block(card.longFilename, EEPROM_MATERIAL_NAME_OFFSET(nr), 8);
}

void lcd_material_read_current_material()
{
  SERIAL_DEBUGLNPGM("Material:");
  
  for(uint8_t e=0; e<EXTRUDERS; e++)
  {
    material[e].temperature = eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
    SERIAL_DEBUGPGM("temperature:");
    SERIAL_DEBUGLN(material[e].temperature);
    material[e].bed_temperature = eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
    SERIAL_DEBUGPGM("bed_temperature:");
    SERIAL_DEBUGLN(material[e].bed_temperature);
    material[e].flow = eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
    SERIAL_DEBUGPGM("flow:");
    SERIAL_DEBUGLN(material[e].flow);
    material[e].fan_speed = eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
    SERIAL_DEBUGPGM("fan_speed:");
    SERIAL_DEBUGLN(material[e].fan_speed);
    material[e].diameter = eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
    SERIAL_DEBUGPGM("diameter:");
    SERIAL_DEBUGLN(material[e].diameter);
  }
}

void lcd_material_store_current_material()
{
  for(uint8_t e=0; e<EXTRUDERS; e++)
  {
    eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].temperature);
    eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].bed_temperature);
    eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].fan_speed);
    eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].flow);
    eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].diameter);
  }
}

bool lcd_material_verify_material_settings()
{
  uint8_t cnt = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
  if (cnt < 2 || cnt > 16)
    return false;
  while(cnt > 0)
  {
    cnt--;
    if (eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(cnt)) > HEATER_0_MAXTEMP)
      return false;
    if (Device_isBedHeat) {
      if (eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(cnt)) > BED_MAXTEMP)
        return false;
    }
    if (eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(cnt)) > 100)
      return false;
    if (eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(cnt)) > 1000)
      return false;
    if (eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(cnt)) > 10.0)
      return false;
    if (eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(cnt)) < 0.1)
      return false;
  }
  return true;
}



int8_t cleanTimes=0;

void doCancelClean(){
  setTargetHotend(0, active_extruder);
  nextEncoderPos = 2;
}

void doStartClean(){
  cleanTimes = 0;
}

void lcd_material_clean_nozzle_option(){
  lcd_question_screen(lcd_material_clean_nozzle_remove_option, doStartClean, LS(PSTR("YES"),
                                                                    PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                                    PSTR("\x91" "\x84"  "\xB6" "\x83"  )) , lcd_menu_maintenance, doCancelClean, LS(PSTR("NO"),
                                                                                                                                   PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                                                                                   PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) , MenuForward, MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Start to clean"),
                                                  PSTR("\x9E" "\x82"  "\x81" "\x81"  "\x9A" "\x80"  "\xDF" "\x80"  "\x97" "\x82"  "\xB6" "\x81"  "\xA0" "\x80"  "\xA1" "\x80"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\x90" "\x84"  "\xC6" "\x82"  " ""\x9A" "\x83"  "\xA1" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("the nozzle"),
                                                  PSTR("(""\x9F" "\x82"  "\xC0" "\x80"  "\xA0" "\x82"  "\xBC" "\x80"  "\xCE" "\x81"  ")"),
                                                  PSTR("(""\xAC" "\x84"  "\xD0" "\x82"  "\xDB" "\x82"  " ""\x83" "\x83"  "\xD9" "\x83"  ")")) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("(experimental)"),
                                                  PSTR(""),
                                                  PSTR("")) );
}

void doCleanPreheat(){
  setTargetHotend(220, active_extruder);
  menuTimer = millis();
}

void lcd_material_clean_nozzle_remove_option(){
  lcd_question_screen(lcd_menu_change_material_preheat, doCleanPreheat, LS(PSTR("REMOVE"),
                                                                 PSTR("\xC9" "\x81"  "\xF6" "\x80"  ),
                                                                 PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\x97" "\x84"  "\xA9" "\x83"  )) , lcd_material_clean_nozzle_check_option, NULL, LS(PSTR("SKIP"),
                                                                                                                                PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                                                                                                                PSTR("\xD0" "\x82"  "\x9B" "\x83"  )) , MenuForward, MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Please remove"),
                                                  PSTR("\xB2" "\x80"  "\xC9" "\x81"  "\x8B" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xD3" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\x97" "\x84"  "\xA9" "\x83"  " ""\x95" "\x83"  "\xF8" "\x83"   )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("material from 3D"),
                                                  PSTR("\xC1" "\x80"  "\xF4" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x91" "\x84"  "\xB6" "\x83"  " ""\xA3" "\x84"  "\xAD" "\x84"  )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("printer"),
                                                  PSTR(""),
                                                  PSTR("")) );
}

void doCleanHeat(){
  fanSpeed = 0;
  setTargetHotend(220, active_extruder);
  menuTimer = millis();
}

void lcd_material_clean_nozzle_check_option(){
  lcd_question_screen(lcd_material_clean_nozzle_heat, doCleanHeat, LS(PSTR("SKIP"),
                                                                      PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                                                      PSTR("\xD0" "\x82"  "\x9B" "\x83"  )) , lcd_material_clean_nozzle_open, NULL, LS(PSTR("DETAILS"),
                                                                                        PSTR("\xA4" "\x82"  "\xA5" "\x82"  ),
                                                                                        PSTR("\xCA" "\x83"  "\xF9" "\x82"  )) , MenuForward, MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Warning: please obey"),
                                                  PSTR("\xA6" "\x82"  "\xA7" "\x82"  ":""\x91" "\x82"  "\xE3" "\x80"  "\x80" "\x81"  "\xA8" "\x82"  "\xB2" "\x80"  ),
                                                  PSTR("Warning:""\xAE" "\x84"  "\xEC" "\x82"  "\x9A" "\x83"  " ""\xDC" "\x82"  "\xDD" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("user manual to do"),
                                                  PSTR("\xA9" "\x82"  "\xAA" "\x82"  "\xEB" "\x81"  "\xAB" "\x82"  "\xA7" "\x80"  "\xAC" "\x82"  "\xAB" "\x81"  "\xAD" "\x82"  "\xB9" "\x81"  "\xBA" "\x81"  "."),
                                                  PSTR("\x84" "\x83"  "\xEB" "\x83"  "\xC8" "\x82"  "\xAF" "\x84"  "\xDD" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("the following steps"),
                                                  PSTR("\xA1" "\x82"  "\xA2" "\x82"  "\xAC" "\x80"  "\xAD" "\x80"  "\xE5" "\x80"  "\xE6" "\x80"  "?"),
                                                  PSTR("\xC3" "\x82"  "\xB4" "\x83"  " ""\xB0" "\x84"  "\xBF" "\x83"  "\xA3" "\x84"  "\xAD" "\x84"  )) );
}

void lcd_material_clean_nozzle_open(){
  lcd_info_screen(lcd_material_clean_nozzle_prepare, NULL, LS(PSTR("CONTINUE"),
                                                              PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                              PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )), MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Open the filament"),
                                                  PSTR("\xAB" "\x81"  "\xB1" "\x80"  "\xAA" "\x80"  "\x9A" "\x80"  "\xA0" "\x80"  "\xA1" "\x80"  "\xD2" "\x80"  ),
                                                  PSTR("\xB4" "\x84"  "\xB3" "\x83"  "\xB4" "\x83"  " ""\xD5" "\x82"  "\xD6" "\x82"  "\xCA" "\x83"  " ""\x96" "\x83"  "\x97" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("tube above the"),
                                                  PSTR("\x9C" "\x81"  "\xF6" "\x80"  "\x87" "\x81"  "\xAE" "\x82"  "\xAF" "\x82"  ),
                                                  PSTR("\xA6" "\x83"  "\xA7" "\x83"  " ""\xFC" "\x83"  "\x86" "\x83"  "\x98" "\x83"  " ""\xE3" "\x82"  "\x9D" "\x83"  ".")) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("nozzle by hand"),
                                                  PSTR(""),
                                                  PSTR("")) );
}

void lcd_material_clean_nozzle_prepare(){
  lcd_info_screen(lcd_material_clean_nozzle_heat, doCleanHeat, LS(PSTR("CONTINUE"),
                                                                  PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                  PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )), MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Prepare one meter"),
                                                  PSTR("\xFF" "\x80"  "\xEB" "\x80"  "\xB0" "\x82"  "\xAD" "\x80"  "\xC9" "\x80"  "\xB1" "\x82"  "\xD0" "\x81"  "\xF4" "\x80"  ),
                                                  PSTR("1M ""\xC9" "\x83"  "\xCA" "\x83"  "\xF5" "\x82"  " ""\x96" "\x83"  "\x97" "\x83"  "\x98" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("of PLA filament"),
                                                  PSTR("PLA""\xAA" "\x80"  "\xAB" "\x80"  "\x84" "\x81"  "\xF5" "\x80"  ),
                                                  PSTR("\xC3" "\x83"  "\x8F" "\x83"  "\xC7" "\x83"  "\x9D" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR(""),
                                                  PSTR(""),
                                                  PSTR("")) );
}


void lcd_material_clean_nozzle_heat(){
  LED_GLOW_HEAT();
  int16_t temp = degHotend(active_extruder) - 20;
  int16_t target = degTargetHotend(active_extruder) - 20 - 10;
  if (temp < 0) temp = 0;
  if (temp > target  && millis()-menuTimer>800 && !is_command_queued() && !isCommandInBuffer())
  {
    if (cleanTimes<0) {
      lcd_change_to_menu(lcd_menu_change_material_insert_wait_user,MAIN_MENU_ITEM_POS(0),MenuForward);
      current_position[E_AXIS] = 0;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    }
    else{
      lcd_change_to_menu(lcd_material_clean_nozzle_heated,MAIN_MENU_ITEM_POS(0),MenuForward);
    }
    menuTimer = millis();
    temp = target;
  }
  
  uint8_t progress = uint8_t(temp * 125 / target);
  if (progress < minProgress)
    progress = minProgress;
  else
    minProgress = progress;
  lcd_info_screen(lcd_menu_maintenance, doCancelClean, LS(PSTR("CANCEL"),
                                                          PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                          PSTR("\xFF" "\x82"  "\xC6" "\x82"  )), MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Nozzle is heating to"),
                                                  PSTR("\xA0" "\x80"  "\xA1" "\x80"  "\x90" "\x80"  "\x88" "\x80"  "\xB2" "\x82"  "\xB3" "\x82"  "\xF7" "\x81"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  "\x98" "\x83"  " ""\xAC" "\x83"  "\x85" "\x83"  "\xEF" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("220`C, please wait."),
                                                  PSTR("\x88" "\x80"  "\x89" "\x80"  ",""\xB2" "\x80"  "\xB4" "\x82"  "\xB3" "\x80"  ),
                                                  PSTR("\xC4" "\x82"  "\xC5" "\x82"  "\xC2" "\x83"  "\xAC" "\x83"  " ""\x90" "\x83"  "\x91" "\x83"  "\xC7" "\x83"  "\x9D" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR(""),
                                                  PSTR(""),
                                                  PSTR("")) );
  
  lcd_progressbar(progress);
}

void doCleanCool(){
  setTargetHotend(0, active_extruder);
  fanSpeed=255;
}

void lcd_material_clean_nozzle_heated(){
  lcd_info_screen(lcd_material_clean_nozzle_repeat, doCleanCool, LS(PSTR("CONTINUE"),
                                                                  PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                  PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) ,MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Insert filament into"),
                                                  PSTR("\xB0" "\x81"  "\xA0" "\x80"  "\xA1" "\x80"  "\xC1" "\x80"  "\x85" "\x81"  "\xDD" "\x80"  "\x84" "\x81"  "\xF5" "\x80"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  "\xB1" "\x84"  "\xC1" "\x82"  "\xF8" "\x83"  "\xBE" "\x82"  " ""\x96" "\x83"  "\x97" "\x83"  "\x90" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("nozzle until melted"),
                                                  PSTR("\x9A" "\x82"  "\x89" "\x81"  "\xA0" "\x80"  "\xA1" "\x80"  "\x8B" "\x81"  "\xF6" "\x80"  ),
                                                  PSTR("\xD7" "\x82"  "\xC9" "\x83"  "\xC5" "\x82"  "\xB2" "\x84"  " ""\x96" "\x83"  "\x97" "\x83"  "\x98" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("filament comes out."),
                                                  PSTR("\xA1" "\x81"  "\xCE" "\x80"  "\xE2" "\x80"  "\x8C" "\x82"  "\xDB" "\x80"  "\xE4" "\x80"  ),
                                                  PSTR("\xA6" "\x83"  "\xA7" "\x83"  "\xC7" "\x83"  "\x9D" "\x83"  ".")) );
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("Press OK to continue"),
                                                  PSTR(""),
                                                  PSTR("")) );
}


void doRepeatClean(){
  cleanTimes++;
}

void doFinishClean(){
  cleanTimes = -1;
}

void lcd_material_clean_nozzle_repeat(){
  lcd_question_screen(lcd_material_clean_nozzle_cool, doRepeatClean, LS(PSTR("REPEAT"),
                                                                      PSTR("\xB5" "\x82"  "\xC8" "\x81"  ),
                                                                      PSTR("REPEAT")) , lcd_material_clean_nozzle_cool, doFinishClean, LS(PSTR("DONE"),
                                                                                                                                          PSTR("\xDA" "\x81"  "\xBB" "\x80"  ),
                                                                                                                                          PSTR("\xDD" "\x83"  "\xCE" "\x82"  "\xC7" "\x83"  )) , MenuForward, MenuForward);
  
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Compare result in "),
                                                  PSTR("\xEB" "\x81"  "\xAB" "\x82"  "\xA7" "\x80"  "\xAC" "\x82"  "\xAB" "\x81"  "\xAD" "\x82"  ),
                                                  PSTR("\xDC" "\x82"  "\xDD" "\x82"  " ""\x84" "\x83"  "\xEB" "\x83"  "\xC8" "\x82"  "\xC4" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("user manual. Do you"),
                                                  PSTR("\xA1" "\x82"  "\xA2" "\x82"  "\xA0" "\x81"  "\xB5" "\x82"  "\xC8" "\x81"  "\x97" "\x82"  "\xB6" "\x81"  "\xA0" "\x80"  "\xA1" "\x80"  "?"),
                                                  PSTR("\x8F" "\x83"  "\xA4" "\x83"  "\xF3" "\x82"  "\x95" "\x83"  " ""\xD5" "\x82"  "\xD6" "\x82"   )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("need to clean"),
                                                  PSTR(""),
                                                  PSTR("\x90" "\x84"  "\xC6" "\x82"  " ""\x95" "\x83"  "\xF8" "\x83"  " ""\x91" "\x84"  "\xB6" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("the nozzle again?"),
                                                  PSTR(""),
                                                  PSTR("")) );
}


void lcd_material_clean_nozzle_cool(){
  
  LED_GLOW_END();
  
  lcd_info_screen(lcd_menu_maintenance, doCancelClean, LS(PSTR("CANCEL"),
                                                          PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                          PSTR("\xFF" "\x82"  "\xC6" "\x82"  )),MenuForward);
  if (current_temperature[0] > 90)
  {
    lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Nozzle is cooling to"),
                                                    PSTR("\xA0" "\x80"  "\xA1" "\x80"  "\xF8" "\x80"  "\x88" "\x80"  "\xB2" "\x82"  "\xB3" "\x82"  "\xF7" "\x81"  ),
                                                    PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  "\x98" "\x83"  " ""\xAC" "\x83"  "\x85" "\x83"  "\xEF" "\x83"   )) );
    lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("90`C, please wait"),
                                                    PSTR("\x88" "\x80"  "\x89" "\x80"  ",""\xB2" "\x80"  "\xB4" "\x82"  "\xB3" "\x80"  ),
                                                    PSTR("\xC4" "\x82"  "\xC5" "\x82"  "\xC2" "\x83"  "\xAC" "\x83"  " ""\x83" "\x84"  "\xB3" "\x84"  "\x9D" "\x83"  )) );
    lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR(""),
                                                    PSTR(""),
                                                    PSTR("")) );
    
    int16_t progress = 124 - (current_temperature[0] - 90);
    if (progress < 0) progress = 0;
    if (progress > 124) progress = 124;
    
    if (progress < minProgress)
      progress = minProgress;
    else
      minProgress = progress;
    
    lcd_progressbar(progress);

  }else{
    currentMenu = lcd_material_clean_nozzle_cooled;
    setTargetHotend(90, active_extruder);
    fanSpeed=0;
  }
}

void lcd_material_clean_nozzle_cooled(){

  lcd_info_screen(lcd_material_clean_nozzle_heat, doCleanHeat, LS(PSTR("CONTINUE"),
                                                                  PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                  PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) ,MenuForward);

  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Slowly pull out the"),
                                                  PSTR("\xCA" "\x81"  "\xCA" "\x81"  "\x81" "\x81"  "\x84" "\x81"  "\xF5" "\x80"  "\xB6" "\x82"  "\x8B" "\x81"  ),
                                                  PSTR("\xDC" "\x82"  "\xDD" "\x82"  " ""\x84" "\x83"  "\xEB" "\x83"  "\xC8" "\x82"  "\xC4" "\x83"   )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("filament and cut the"),
                                                  PSTR("\xB7" "\x82"  "\xB8" "\x82"  "\xB9" "\x82"  "\xBA" "\x82"  "\xF4" "\x80"  "\xEC" "\x80"  "\xE3" "\x81"  ),
                                                  PSTR("\x8F" "\x83"  "\xA4" "\x83"  "\xF3" "\x82"  "\x95" "\x83"  " ""\xD5" "\x82"  "\xD6" "\x82"   )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("end off."),
                                                  PSTR(""),
                                                  PSTR("\x90" "\x84"  "\xC6" "\x82"  " ""\x95" "\x83"  "\xF8" "\x83"  " ""\x91" "\x84"  "\xB6" "\x83"  )) );
}


#endif//ENABLE_ULTILCD2

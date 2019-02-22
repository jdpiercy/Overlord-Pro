#include "configuration.h"
#ifdef ENABLE_ULTILCD2
  #include "UltiLCD2.h"
  #include "UltiLCD2_hi_lib.h"
  #include "UltiLCD2_gfx.h"
  #include "UltiLCD2_menu_material.h"
  #include "UltiLCD2_menu_print.h"
  #include "UltiLCD2_menu_first_run.h"
  #include "UltiLCD2_menu_maintenance.h"
  #include "cardreader.h"
  #include "ConfigurationStore.h"
  #include "temperature.h"
  #include "pins.h"
  #include "UltiLCD2_low_lib.h"
#include "UltiLCD2_menu_material.h"
  #include "SDUPS.h"

  #define SERIAL_CONTROL_TIMEOUT 0

unsigned long lastSerialCommandTime;
bool serialScreenShown;
uint8_t led_brightness_level = 100;
uint8_t led_mode = LED_MODE_ALWAYS_ON;

static void lcd_menu_startup();

void lcd_menu_wait();

void lcd_init()
{
  lcd_lib_init();
  if (!lcd_material_verify_material_settings())
  {
    lcd_material_reset_defaults();
    for(uint8_t e=0; e<EXTRUDERS; e++)
      lcd_material_set_material(0, e);
  }
  lcd_material_read_current_material();
  if (Device_isWifi) {
    menuTimer = 100;
    currentMenu = lcd_menu_wait;
  }
  currentMenu = lcd_menu_startup;
  analogWrite(LED_PIN, 0);
  lastSerialCommandTime = millis() - SERIAL_CONTROL_TIMEOUT;
}


void lcd_update()
{
  if (!lcd_lib_update_ready()) return;
  lcd_lib_buttons_update();
  card.updateSDInserted();

  if (led_glow_dir)
  {
    led_glow-=2;
    if (led_glow == 0) led_glow_dir = 0;
  }else{
    led_glow+=2;
    if (led_glow == 126) led_glow_dir = 1;
  }

  if (IsStopped())
  {
    lcd_lib_clear();

    
    lcd_lib_draw_string_centerP(LS(10, 11, 11),
                                LS(PSTR("ERROR - STOPPED"),
                                   PSTR("\x80" "\x80"  "\x81" "\x80"  " - " "\x82" "\x80"  "\x83" "\x80"  "\x84" "\x80"  "\x85" "\x80"  ),
                                   PSTR("\xBE" "\x82"  "\xBF" "\x82"   " - " "\xC0" "\x82"  "\xC1" "\x82"  )));
    switch(StoppedReason())
    {
      case STOP_REASON_MAXTEMP:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Temp Max"),
                                       PSTR("\x86" "\x80"  "\x87" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ),
                                       PSTR("\xC2" "\x82"  "\xC3" "\x82"  "\xC4" "\x82"  "\xC5" "\x82"  )));
        break;
      case STOP_REASON_MINTEMP:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Temp MIM"),
                                       PSTR("\x86" "\x80"  "\x8A" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ),
                                       PSTR("\xC2" "\x82"  "\xC6" "\x82"  "\xC4" "\x82"  "\xC5" "\x82"  )));
        break;
      case STOP_REASON_MAXTEMP_BED:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Temp sensor BED"),
                                       PSTR("\x8B" "\x80"  "\x8C" "\x80"  "\x88" "\x80"  "\x89" "\x80"  "\x8D" "\x80"  "\x8E" "\x80"  "\x8F" "\x80"  ),
                                       PSTR("BED""\xC4" "\x82"  "\xC5" "\x82"  " ""\xC7" "\x82"  "\xC8" "\x82"  )));
        break;
      case STOP_REASON_HEATER_ERROR:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Heater error"),
                                       PSTR("\x90" "\x80"  "\x8B" "\x80"  "\x8F" "\x80"  "\x91" "\x80"  "\x92" "\x80"  ),
                                       PSTR("\xC9" "\x82"  "\xCA" "\x82"  " ""\xCB" "\x82"  "\xCC" "\x82"  )));
        break;
      case STOP_REASON_SAFETY_TRIGGER:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Safety circuit"),
                                       PSTR("\x93" "\x80"  "\x94" "\x80"  "\x95" "\x80"  "\x96" "\x80"  ),
                                       PSTR("\xCD" "\x82"  "\xCE" "\x82"  " ""\xCE" "\x82"  "\xCF" "\x82"  )));
        break;
      case STOP_REASON_Z_ENDSTOP_BROKEN_ERROR:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Z switch broken"),
                                       PSTR("Z""\x97" "\x80"  "\x98" "\x80"  "\x99" "\x80"  "\x9A" "\x80"  "\x9B" "\x80"  "\x9C" "\x80"  "\x9D" "\x80"  ),
                                       PSTR("Z ""\xD0" "\x82"  "\xD1" "\x82"  "\xD2" "\x82"  " ""\xCB" "\x82"  "\xCC" "\x82"  )));
        break;
      case STOP_REASON_Z_ENDSTOP_STUCK_ERROR:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Z switch stuck"),
                                       PSTR("Z""\x97" "\x80"  "\x98" "\x80"  "\x99" "\x80"  "\x9A" "\x80"  "\x9B" "\x80"  "\x9E" "\x80"  "\x9F" "\x80"  ),
                                       PSTR("Z ""\xD0" "\x82"  "\xD1" "\x82"  "\xD2" "\x82"  " ""\xD3" "\x82"  "\xD4" "\x82"  )));
        break;
      case STOP_REASON_X_ENDSTOP_BROKEN_ERROR:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("X switch broken"),
                                       PSTR("X""\x97" "\x80"  "\x98" "\x80"  "\x99" "\x80"  "\x9A" "\x80"  "\x9B" "\x80"  "\x9C" "\x80"  "\x9D" "\x80"  ),
                                       PSTR("X ""\xD0" "\x82"  "\xD1" "\x82"  "\xD2" "\x82"  " ""\xCB" "\x82"  "\xCC" "\x82"  )));
        break;
      case STOP_REASON_X_ENDSTOP_STUCK_ERROR:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("X switch stuck"),
                                       PSTR("X""\x97" "\x80"  "\x98" "\x80"  "\x99" "\x80"  "\x9A" "\x80"  "\x9B" "\x80"  "\x9E" "\x80"  "\x9F" "\x80"  ),
                                       PSTR("X ""\xD0" "\x82"  "\xD1" "\x82"  "\xD2" "\x82"  " ""\xD3" "\x82"  "\xD4" "\x82"  )));
        break;
      case STOP_REASON_Y_ENDSTOP_BROKEN_ERROR:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Y switch broken"),
                                       PSTR("Y""\x97" "\x80"  "\x98" "\x80"  "\x99" "\x80"  "\x9A" "\x80"  "\x9B" "\x80"  "\x9C" "\x80"  "\x9D" "\x80"  ),
                                       PSTR("Y ""\xD0" "\x82"  "\xD1" "\x82"  "\xD2" "\x82"  " ""\xCB" "\x82"  "\xCC" "\x82"  )));
        break;
      case STOP_REASON_Y_ENDSTOP_STUCK_ERROR:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Y switch stuck"),
                                       PSTR("Y""\x97" "\x80"  "\x98" "\x80"  "\x99" "\x80"  "\x9A" "\x80"  "\x9B" "\x80"  "\x9E" "\x80"  "\x9F" "\x80"  ),
                                       PSTR("Y ""\xD0" "\x82"  "\xD1" "\x82"  "\xD2" "\x82"  " ""\xD3" "\x82"  "\xD4" "\x82"  )));
        break;
      case STOP_REASON_REDUNDANT_TEMP:
        lcd_lib_draw_string_centerP(LS(20, 24, 24) ,
                                    LS(PSTR("Temp Redundant"),
                                       PSTR("\xA0" "\x80"  "\xA1" "\x80"  "\x88" "\x80"  "\x89" "\x80"  "\xA2" "\x80"  "\xA3" "\x80"  ),
                                       PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  " ""\xD7" "\x82"  "\xD8" "\x82"  )));
        break;
    }
    lcd_lib_draw_string_centerP(LS(40, 37, 37) ,
                                LS(PSTR("Contact:"),
                                   PSTR("\xA4" "\x80"  "\xA5" "\x80"  ":"),
                                   PSTR("\xD9" "\x82"  "\xDA" "\x82"  "\xDB" "\x82"  )));
    
    lcd_lib_draw_string_centerP(LS(50, 49, 49) ,
                                LS(PSTR("support@dreammaker.cc"),
                                   PSTR("support@dreammaker.cc"),
                                   PSTR("support@dreammaker.cc")));

    LED_ERROR();
    lcd_lib_update_screen();
  }else if (millis() - lastSerialCommandTime < SERIAL_CONTROL_TIMEOUT)
  {
    if (!serialScreenShown)
    {
      lcd_lib_clear();
      
      lcd_lib_draw_string_centerP(LS(24, 24, 24) ,
                                  LS(PSTR("Printing with USB..."),
                                     PSTR("\xA6" "\x80"  "\xA7" "\x80"  "USB""\xA8" "\x80"  "\xA9" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "..."),
                                     PSTR("USB ""\xDC" "\x82"  "\xDD" "\x82"  " ""\xA9" "\x83"  "\xA7" "\x84"  )));
      serialScreenShown = true;
    }
    if (printing_state == PRINT_STATE_HEATING || printing_state == PRINT_STATE_HEATING_BED || printing_state == PRINT_STATE_HOMING)
      lastSerialCommandTime = millis();
    lcd_lib_update_screen();
  }else{
    serialScreenShown = false;
    currentMenu();
    lcd_lib_update_screen();
    if (postMenuCheck) postMenuCheck();
  }
}

void lcd_menu_wait()
{
  LED_OFF();

  if (lcd_lib_button_down) {
    if (--menuTimer == 0) {
      lcd_change_to_menu(lcd_menu_startup,MenuForward);
    }
  }
  else{
    menuTimer = 100;
  }
  
  lcd_info_screen(NULL, NULL, LS(PSTR("SKIP"),
                                 PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                 PSTR("\xD0" "\x82"  "\x9B" "\x83"  )));
  lcd_lib_draw_string_centerP(LS(20, 11, 11) ,
                              LS(PSTR("Starting Up..."),
                                 PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\xB0" "\x80"  "\xB1" "\x80"   "..."),
                                 PSTR("Starting Up...")));
  lcd_lib_draw_string_centerP(LS(30, 24, 24) ,
                              LS(PSTR("Please wait"),
                                 PSTR("\xB2" "\x80"  "\xB3" "\x80"  "\xB4" "\x80"  ),
                                 PSTR("Please wait")));
  if (isWindowsServerStarted) {
    lcd_change_to_menu(lcd_menu_startup);
  }
}

void lcd_menu_bluetooth()
{
  LED_NORMAL();
  lcd_info_screen(lcd_menu_main, NULL, LS(PSTR("DISMISS"),
                                          PSTR("\xB5" "\x80"  "\xB6" "\x80"  ),
                                          PSTR("\x80" "\x83"  "\xD0" "\x82"  "\x9A" "\x84"  "\xD0" "\x82"  )) , MenuForward);
  lcd_lib_draw_string_centerP(LS(20, 11, 11) ,
                              LS(PSTR("Blutooth"),
                                 PSTR("\xB7" "\x80"  "\xB8" "\x80"  ),
                                 PSTR("\xE0" "\x82"  "\xE1" "\x82"  "\xE2" "\x82"  "\xD0" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(30, 24, 24) ,
                              LS(PSTR("Connected!"),
                                 PSTR("\xB9" "\x80"  "\xBA" "\x80"  "\xBB" "\x80"  "\xBC" "\x80"  ),
                                 PSTR("\xE3" "\x82"  "\xE4" "\x82"  " ""\xE5" "\x82"  "\xE6" "\x82"  )));
}

void lcd_menu_wifi()
{

}


  #define RefreshFrequency 20
  #define DropFrequency 50
  #define DropAcc 32
  #define DropVel 32

void lcd_menu_startup()
{
  LED_NORMAL();

  static boolean isFirstTime=true;
  static unsigned long startTime;
  unsigned long allTime;

  if (isFirstTime) {
    isFirstTime=false;
    startTime=millis();
  }

  allTime=millis()-startTime + 1024;

  lcd_lib_clear();
  lcd_lib_draw_gfx(0, 22, overlordTextGfx);

  int endLine=64;
  int i,j;

  for (j=0; j<endLine; j++) {
    int lineStartTime = ((63-j)<<6) - allTime;

    if (lineStartTime + 127*20 < 0) {
      if (j==0) {

        isFirstTime=true;

        if (led_mode == LED_MODE_ALWAYS_ON)
          analogWrite(LED_PIN, 255 * led_brightness_level / 100);
        led_glow = led_glow_dir = 0;
        LED_NORMAL();
        lcd_lib_beep();
        
        if (Device_type<OVERLORD_TYPE_MIN || Device_type>OVERLORD_TYPE_MAX) {
          languageType= LANGUAGE_ENGLISH;
          currentMenu = lcd_menu_device;
        }
        else{
          if (!IS_FIRST_RUN_DONE())
          {
            languageType= LANGUAGE_CHINESE;
            currentMenu = lcd_menu_first_run_language;
          }
          else{
            if (degHotend(0)<5) {
              currentMenu = lcd_menu_first_run_temperature_error;
            }
            else{
              currentMenu = lcd_menu_main;
            }
          }
        }
      }
      continue;
    }

    for (i=0; i<128; i++) {

      if (lineStartTime>=0) {

        //            int s= DropVel*lineStartTime+DropAcc*lineStartTime*lineStartTime;
        long s= j - (((long)lineStartTime*(long)lineStartTime)>>12); //+lineStartTime*lineStartTime>14;

        if (s>=0) {
          lcd_lib_move_point(i, s, i, j);
        }
        else{
          lcd_lib_clear(i, j, 127, j);
          break;
        }
      }
      lineStartTime+=20;
    }
  }

  if (led_mode == LED_MODE_ALWAYS_ON)
    analogWrite(LED_PIN, int(led_glow << 1) * led_brightness_level / 100);
  if (lcd_lib_button_pressed)
  {
    isFirstTime=true;
    if (led_mode == LED_MODE_ALWAYS_ON)
      analogWrite(LED_PIN, 255 * led_brightness_level / 100);
    led_glow = led_glow_dir = 0;
    LED_NORMAL();
    lcd_lib_beep();

    if (Device_type<1 || Device_type>20) {
      languageType= LANGUAGE_ENGLISH;
      currentMenu = lcd_menu_device;
    }
    else{
      if (!IS_FIRST_RUN_DONE()) {
        languageType= LANGUAGE_CHINESE;
        currentMenu = lcd_menu_first_run_language;
      }
      else{
        if (degHotend(0)<5) {
          currentMenu = lcd_menu_first_run_temperature_error;
        }
        else{
          currentMenu = lcd_menu_main;
        }
      }
    }
  }
}

void doCooldown()
{
  for(uint8_t n=0; n<EXTRUDERS; n++)
    setTargetHotend(0, n);
  setTargetBed(0);
}

static char* lcd_menu_main_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P(card.longFilename, LS(PSTR(ENGLISH_DOWN "Print"),
                                   PSTR(CHINESE_POINT "\xAA" "\x80"  "\xAB" "\x80"  ),
                                   PSTR(CHINESE_POINT "\xA9" "\x83"  "\xA7" "\x84"  )) );
  else
    strcpy_P(card.longFilename, PSTR("???"));
  return card.longFilename;
}

static void lcd_menu_main_details(uint8_t nr)
{
  char buffer[22];
  
  if (nr == 0)
    lcd_draw_detailP(LS(PSTR("Print from SD card."),
                        PSTR("\xAA" "\x80"  "\xAB" "\x80"  "SD" "\x9E" "\x80"  "\xC1" "\x80"  "\xC2" "\x80"  "\xC3" "\x80"  ),
                        PSTR("SD""\xEB" "\x82"  "\xEC" "\x82"  "\x98" "\x83"  " ""\xA8" "\x84"  "\xC7" "\x83"  " ""\xED" "\x82"  "\xEE" "\x82"  " ""\xA9" "\x83"  "\xA7" "\x84"  )));
}

static void doSelectPrint()
{
  SDUPSGetCoordinateLastZ=0.0;
  SDUPSGetCoordinateZ=0.0;
  
  if (Device_isBedHeat) {
    if (material[0].bed_temperature) {
      target_temperature_bed = max(target_temperature_bed, material[0].bed_temperature);
    }
    else{
      target_temperature_bed = 40;
    }
    
  }
  else{
    target_temperature_bed = 0;
  }
  
  isBedPreheat = true;
  
  if (SDUPSIsWorking()) {
    lcd_change_to_menu(lcd_menu_print_resume_option,SCROLL_MENU_ITEM_POS(0), MenuForward);
  }
  else{
    lcd_clear_cache();
    lcd_change_to_menu(lcd_menu_print_select, SCROLL_MENU_ITEM_POS(0), MenuForward);
  }
}

void lcd_menu_temp_warning()
{
  if (temp_error_handle) {
    
    lcd_question_screen(lcd_menu_maintenance_advanced_heatup, NULL, LS(PSTR("FIX"),
                                                                       PSTR("\xC4" "\x80"  "\xC5" "\x80"  ),
                                                                       PSTR("\x9B" "\x84"  "\xD0" "\x82"  )) , NULL, doSelectPrint, LS(PSTR("CONTINUE"),
                                                                                                           PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                                                           PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) ,MenuForward,MenuForward);
    lcd_lib_draw_string_centerP(LS(10, 11, 11), LS(PSTR("One of the temp"),
                                                   PSTR("\xC8" "\x80"  "\xC9" "\x80"  "\xCA" "\x80"  ),
                                                   PSTR("\xF2" "\x82"  "\xC0" "\x82"  " ""\xF3" "\x82"  "\xF4" "\x82"  "\xF5" "\x82"  )));
    lcd_lib_draw_string_centerP(LS(20, 24, 24), LS(PSTR("sensors seems not"),
                                                   PSTR("\x88" "\x80"  "\x89" "\x80"  "\x8D" "\x80"  "\x8E" "\x80"  "\x8F" "\x80"  ),
                                                   PSTR("\xC4" "\x82"  "\xC5" "\x82"  " ""\xC7" "\x82"  "\xC8" "\x82"  " ""\xCB" "\x82"  "\xCC" "\x82"  " ""\xF6" "\x82"  "\xF7" "\x82"  )));
    lcd_lib_draw_string_centerP(LS(30, 37, 37), LS(PSTR("working. Try"),
                                                   PSTR("\xCB" "\x80"  "\xAE" "\x80"  "\xCC" "\x80"  ",""\xB2" "\x80"  "\xA4" "\x80"  "\xA5" "\x80"  "\xCD" "\x80"  "\xCE" "\x80"  "\xC4" "\x80"  "\xC5" "\x80"  ),
                                                   PSTR(",A/S""\xBE" "\x82"  " ""\xE3" "\x82"  "\xF8" "\x82"  "\xF3" "\x82"  "\xF9" "\x82"  "\xFA" "\x82"  )));
    lcd_lib_draw_string_centerP(LS(40, 64, 64), LS(PSTR("to fix it"),
                                                   PSTR(""),
                                                   PSTR("")));
  }
  else{
    doSelectPrint();
  }

}

void lcd_menu_main()
{
  LED_NORMAL();
  
  if (isBedPreheat) {
    target_temperature_bed = 0;
    isBedPreheat = false;
  }
  
  if (lcd_lib_encoder_pos < 0) lcd_lib_encoder_pos = 0;
  
  lcd_normal_menu(LS(PSTR("Main Menu"),
                     PSTR("\xCF" "\x80"  "\xD0" "\x80"  "\xD1" "\x80"  ),
                     PSTR("\xFB" "\x82"  "\xF1" "\x82"  )) , 1, lcd_menu_main_item, lcd_menu_main_details);
  
  lcd_lib_draw_gfx(0, 0, printGfx);

  if (IS_SELECTED_SCROLL(1))
  {
    lcd_change_to_menu(lcd_menu_maintenance, SCROLL_MENU_ITEM_POS(0), MenuDown);
  }

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
    {
      doSelectPrint();
    }
  }
}

void lcd_menu_power_check()
{
  LED_GLOW_POWERERROR();
  lcd_lib_clear();
  
  lcd_lib_draw_string_centerP(LS(20, 20, 20), LS(PSTR("Connect the Power"),
                                                      PSTR("\xBA" "\x80"  "\xD2" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xD3" "\x80"  "\x95" "\x80"  "\xD4" "\x80"  ),
                                                      PSTR("\xCE" "\x82"  "\xFC" "\x82"  " ""\xE3" "\x82"  "\xE4" "\x82"  )));
  lcd_lib_draw_string_centerP(LS(30, 30, 30), LS(PSTR("or switch off"),
                                                 PSTR("\xD5" "\x80"  "\xD6" "\x80"  "\x9B" "\x80"  "\xD7" "\x80"  "\x9A" "\x80"  "\x9B" "\x80"  ),
                                                 PSTR("\xFD" "\x82"  "\xFE" "\x82"  " OFF")));

}

/* Warning: This function is called from interrupt context */
void lcd_buttons_update()
{
  lcd_lib_buttons_update_interrupt();
}

#endif//ENABLE_ULTILCD2

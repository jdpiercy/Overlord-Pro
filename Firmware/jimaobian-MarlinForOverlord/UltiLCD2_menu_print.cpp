#include <avr/pgmspace.h>

#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
#include "Marlin.h"
#include "cardreader.h"
#include "temperature.h"
#include "lifetime_stats.h"
#include "UltiLCD2.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_print.h"
#include "UltiLCD2_menu_material.h"
#include "UltiLCD2_menu_maintenance.h"
#include "SDUPS.h"

uint8_t lcd_cache[LCD_CACHE_SIZE];


static void lcd_menu_print_out_of_filament();
static void doOutOfFilament();
static void checkPrintFinished();
static void lcd_menu_print_resume_error();
static void lcd_menu_print_resume_manual_search_sd_card();
static void lcd_menu_print_resume_manual_height();
static void doResumeManualStoreZ();
static void lcd_menu_print_resume_search_sd_card();
static void lcd_menu_print_resume_manual_option();
static void doResumeManualInit();
static void doResumeInit();
static void lcd_menu_print_ready_cooled_down();
static void lcd_menu_print_ready();
static void postPrintReady();
static void lcd_menu_print_classic_warning_again();
static void lcd_menu_print_classic_warning();
static void lcd_menu_print_error();
void lcd_menu_print_abort();
static void lcd_menu_print_tune_retraction();
static void lcd_retraction_details(uint8_t nr);
static char* lcd_retraction_item(uint8_t nr);
static void lcd_menu_print_tune_heatup_nozzle1();
static void lcd_menu_print_tune_heatup_nozzle0();
static void lcd_menu_print_tune();
static void tune_item_details_callback(uint8_t nr);
static char* tune_item_callback(uint8_t nr);
static void lcd_menu_print_printing();
static void lcd_menu_print_heatup();
static void doStartPrint();
void lcd_menu_print_select();
static void lcd_sd_menu_details_callback(uint8_t nr);
static char* lcd_sd_menu_filename_callback(uint8_t nr);
static void cardUpdir();
void lcd_menu_print_resume_option();
static void doResumeStateResume();
static void doResumeStateNormal();

static void lcd_menu_print_prepare_printing();
static void lcd_menu_print_resume_manual_search_sd_card_eeprom();

static void doHotendHome();

float SDUPSGetCoordinateZ;
float SDUPSGetCoordinateLastZ;
static float SDUPSCurrentPosition[NUM_AXIS];
static float SDUPSFeedrate;
static float SDUPSLastZ;
static float SDUPSLastZForCheck;
uint32_t SDUPSFilePosition;
int8_t SDUPSPositionIndex;

uint8_t resumeState=RESUME_STATE_NORMAL;

#define SDUPSStateFindNone 0x00
#define SDUPSStateFindX 0x01
#define SDUPSStateFindY 0x02
#define SDUPSStateFindZ 0x04
#define SDUPSStateFindE 0x08
#define SDUPSStateFindF 0x10

static uint8_t SDUPSState;

#define RESUME_ERROR_NONE 0
#define RESUME_ERROR_Z_RANGE 1
#define RESUME_ERROR_M25 2
#define RESUME_ERROR_SDUPSState 3
#define RESUME_ERROR_SD_VALIDATE 4
#define RESUME_ERROR_SD_FILEPOSITION 5
#define RESUME_ERROR_SD_READ_CARD 6
#define RESUME_ERROR_SD_SEARCH_TOO_LONG 7

static uint8_t resumeError=RESUME_ERROR_NONE;


#ifdef FilamentDetection
boolean isFilamentDetectionEnable;
#endif

void setPrimed()
{
  eeprom_write_byte((uint8_t*)EEPROM_PRIMED_OFFSET, 'H');
}

void clearPrimed()
{
  eeprom_write_byte((uint8_t*)EEPROM_PRIMED_OFFSET, 'L');
}

bool readPrimed()
{
  return (eeprom_read_byte((const uint8_t*)EEPROM_PRIMED_OFFSET) == 'H');
}

void lcd_clear_cache()
{
  for(uint8_t n=0; n<LCD_CACHE_COUNT; n++)
    LCD_CACHE_ID(n) = 0xFF;
  LCD_DETAIL_CACHE_ID() = 0;
  LCD_CACHE_NR_OF_FILES() = 0xFF;
}

void abortPrint()
{
  isWindowsPrinting = false;
  lcd_lib_button_up_down_reversed=false;
  postMenuCheck = NULL;
  lifetime_stats_print_end();
  doCooldown();
  discardEnqueueingCommand();
  discardCommandInBuffer();
  
  char buffer[32];
  if (card.sdprinting) {
    card.sdprinting = false;
  }
  
  if (card.isFileOpen()) {
    card.closefile();
  }
  
  card.pause = false;
  
  if (readPrimed())
  {
    sprintf_P(buffer, PSTR("G92 E%i\nG1 F%i E0"), int(((float)END_OF_PRINT_RETRACTION) / volume_to_filament_length[active_extruder]),int(retract_feedrate));
    enquecommand(buffer);
    
    // no longer primed
    clearPrimed();
  }
  
  enquecommand_P(PSTR("G28\nM84"));
  
  feedmultiply=100;
  
}


/****************************************************************************************
 * Choose whether to resume
 *
 ****************************************************************************************/
static void doResumeStateNormal()
{
  resumeState=RESUME_STATE_NORMAL;
  lcd_clear_cache();
}

static void doResumeStateResume()
{
  resumeState=RESUME_STATE_RESUME;
}

void lcd_menu_print_resume_option()
{
  LED_NORMAL();
  
  if (!card.sdInserted)
  {
    LED_GLOW();
    lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);
    lcd_info_screen(lcd_menu_main);
    
    
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
    lcd_info_screen(lcd_menu_main);
    lcd_lib_draw_string_centerP(16, LS(PSTR("Reading card..."),
                                       PSTR("SD""\x9E" "\x80"  "\x9D" "\x81"  "\xD8" "\x80"  "\xC1" "\x80"  "..." ),
                                       PSTR("Reading card")) );
    return;
  }
  
  lcd_question_screen(lcd_menu_print_resume_manual_option, doResumeStateResume, LS(PSTR("YES"),
                                                                                   PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                                                   PSTR("\x91" "\x84"  "\xB6" "\x83"  )) ,
                      lcd_menu_print_select, doResumeStateNormal, LS(PSTR("NO"),
                                                                     PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                     PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) ,MenuForward,MenuForward);
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Last print is not"),
                                                  PSTR("\xD2" "\x80"  "\xC9" "\x80"  "\xFE" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xE6" "\x81"  "\xC1" "\x80"  "\xFF" "\x81"  ),
                                                  PSTR("\xC5" "\x83"  "\xAC" "\x83"  "\xF7" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  " ""\xC0" "\x82"  "\xC1" "\x82"  "\xEF" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("finished. Do you"),
                                                  PSTR("\x80" "\x82"  "\x81" "\x82"  "\xD1" "\x81"  "\xC8" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xAA" "\x81"  "?"),
                                                  PSTR("\xF8" "\x83"  "\xF9" "\x83"  "\xBE" "\x82"  " ""\xC3" "\x82"  "\xB0" "\x83"  "\x95" "\x83"  " ""\x9A" "\x83"  "\x9D" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("want to resume"),
                                                  PSTR(""),
                                                  PSTR("\xFA" "\x83"  "\xFB" "\x83"  "\xF3" "\x82"  "\xC0" "\x83"  "\xC1" "\x83"  "\x9C" "\x83"  "\xC2" "\x83"  "?")) );
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("print?"),
                                                  PSTR(""),
                                                  PSTR("")) );
}


/****************************************************************************************
 * SD select
 *
 ****************************************************************************************/
static void cardUpdir()
{
  card.updir();
}

static char* lcd_sd_menu_filename_callback(uint8_t nr)
{
  //This code uses the card.longFilename as buffer to store the filename, to save memory.
  if (nr == 0)
  {
    if (card.atRoot())
    {
      strcpy_P(card.longFilename, LS(PSTR("Return"),
                                     PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                     PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
    }else{
      strcpy_P(card.longFilename, LS(PSTR("Back"),
                                     PSTR("\xD2" "\x80"  "\xC9" "\x80"  "\xE3" "\x81"  ),
                                     PSTR("Back")) );
    }
  }else{
    card.longFilename[0] = '\0';
    for(uint8_t idx=0; idx<LCD_CACHE_COUNT; idx++)
    {
      if (LCD_CACHE_ID(idx) == nr){
        strncpy(card.longFilename, LCD_CACHE_FILENAME(idx), LONG_FILENAME_LENGTH -1);
        card.longFilename[LONG_FILENAME_LENGTH -1] = '\0';
      }
    }
    if (card.longFilename[0] == '\0')
    {
      card.getfilename(nr - 1);
      if (!card.longFilename[0])
        strcpy(card.longFilename, card.filename);
      if (!card.filenameIsDir)
      {
        if (strchr(card.longFilename, '.')) strrchr(card.longFilename, '.')[0] = '\0';
      }
      
      uint8_t idx = nr % LCD_CACHE_COUNT;
      LCD_CACHE_ID(idx) = nr;
      strncpy(LCD_CACHE_FILENAME(idx), card.longFilename, LONG_FILENAME_LENGTH -1);
      LCD_CACHE_FILENAME(idx)[LONG_FILENAME_LENGTH -1] = '\0';
      LCD_CACHE_TYPE(idx) = card.filenameIsDir ? 1 : 0;
      if (card.errorCode() && card.sdInserted)
      {
        //On a read error reset the file position and try to keep going. (not pretty, but these read errors are annoying as hell)
#ifdef ClearError
        card.clearError();
#endif
        LCD_CACHE_ID(idx) = 255;
        card.longFilename[0] = '\0';
      }
    }
  }
  return card.longFilename;
}

static void lcd_sd_menu_details_callback(uint8_t nr)
{
  if (nr == 0)
  {
    return;
  }
  for(uint8_t idx=0; idx<LCD_CACHE_COUNT; idx++)
  {
    if (LCD_CACHE_ID(idx) == nr)
    {
      if (LCD_CACHE_TYPE(idx) == 1)
      {
        lcd_draw_detailP(LS(PSTR("Folder"),
                            PSTR("\xF6" "\x81"  "\xD6" "\x81"  "\x82" "\x82"  ),
                            PSTR("\xF0" "\x83"  "\xF1" "\x83"  )) );
      }else{
        char buffer[64];
        if (LCD_DETAIL_CACHE_ID() != nr)
        {
          card.getfilename(nr - 1);
          if (card.errorCode())
          {
#ifdef ClearError
            card.clearError();
#endif
            return;
          }
          LCD_DETAIL_CACHE_ID() = nr;
          LCD_DETAIL_CACHE_TIME() = 0;
          for(uint8_t e=0; e<EXTRUDERS; e++)
            LCD_DETAIL_CACHE_MATERIAL(e) = 0;
          card.openFile(card.filename, true);
          if (card.isFileOpen())
          {
            for(uint8_t n=0; n<8; n++)
            {
              card.fgets(buffer, sizeof(buffer));
              buffer[sizeof(buffer)-1] = '\0';
              while (strlen(buffer) > 0 && buffer[strlen(buffer)-1] < ' ') buffer[strlen(buffer)-1] = '\0';
              if (strncmp_P(buffer, PSTR(";TIME:"), 6) == 0)
                LCD_DETAIL_CACHE_TIME() = atol(buffer + 6);
              else if (strncmp_P(buffer, PSTR(";MATERIAL:"), 10) == 0)
                LCD_DETAIL_CACHE_MATERIAL(0) = atol(buffer + 10);
#if EXTRUDERS > 1
              else if (strncmp_P(buffer, PSTR(";MATERIAL2:"), 11) == 0)
                LCD_DETAIL_CACHE_MATERIAL(1) = atol(buffer + 11);
#endif
            }
            card.closefile();
          }
          if (card.errorCode())
          {
            //On a read error reset the file position and try to keep going. (not pretty, but these read errors are annoying as hell)
#ifdef ClearError
            card.clearError();
#endif
            LCD_DETAIL_CACHE_ID() = 255;
          }
        }
        
        if (LCD_DETAIL_CACHE_TIME() > 0)
        {
          char* c = buffer;
          
          if (led_glow_dir)
          {
            strcpy_P(c, LS(PSTR("Time: "),
                           PSTR("\xE1" "\x81"  "\xE2" "\x81"  ": "),
                           PSTR("\x9A" "\x83"  "\x94" "\x84"  ": ")) );
            c += strlen_P(LS(PSTR("Time: "),
                             PSTR("\xE1" "\x81"  "\xE2" "\x81"  ": "),
                             PSTR("\x9A" "\x83"  "\x94" "\x84"  ": ")) );
            c = int_to_time_string(LCD_DETAIL_CACHE_TIME(), c);
          }else{
            strcpy_P(c, LS(PSTR("Material: "),
                           PSTR("\xF5" "\x80"  "\xF6" "\x80"  ": "),
                           PSTR("\x96" "\x83"  "\x97" "\x83"  ": ")) );
            c += strlen_P(LS(PSTR("Material: "),
                             PSTR("\xF5" "\x80"  "\xF6" "\x80"  ": "),
                             PSTR("\x96" "\x83"  "\x97" "\x83"  ": ")));
            float length = float(LCD_DETAIL_CACHE_MATERIAL(0)) / (M_PI * (material[0].diameter / 2.0) * (material[0].diameter / 2.0));
            if (length < 10000)
              c = float_to_string(length / 1000.0, c, PSTR("m"));
            else
              c = int_to_string(length / 1000.0, c, PSTR("m"));
#if EXTRUDERS > 1
            if (LCD_DETAIL_CACHE_MATERIAL(1))
            {
              *c++ = '/';
              float length = float(LCD_DETAIL_CACHE_MATERIAL(1)) / (M_PI * (material[1].diameter / 2.0) * (material[1].diameter / 2.0));
              if (length < 10000)
                c = float_to_string(length / 1000.0, c, PSTR("m"));
              else
                c = int_to_string(length / 1000.0, c, PSTR("m"));
            }
#endif
          }
          
          lcd_draw_detail(buffer);
        }else{
          lcd_draw_detailP(LS(PSTR("No info available"),
                              PSTR("\x85" "\x82"  "\xC8" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\x9C" "\x82"  "\x9D" "\x82"  ),
                              PSTR("\xE5" "\x83"  "\x95" "\x84"  " ""\xB6" "\x83"  "\x96" "\x84"  " ""\x85" "\x83"  "\xD7" "\x82"  " ""\xF2" "\x83"  "\xA5" "\x83"  )) );
        }
      }
    }
  }
}

void doPreparePrint(){
  active_extruder = 0;
  pausetime = 0;
  SDUPSStart();
  SDUPSGetCoordinateLastZ=0.0;
  SDUPSGetCoordinateZ=0.0;
  eeprom_write_word((uint16_t*)EEPROM_SDUPS_HEATING_OFFSET, 0);
  eeprom_write_word((uint16_t*)EEPROM_SDUPS_HEATING_BED_OFFSET, 0);
  isWindowsPrinting = true;
  if (led_mode == LED_MODE_WHILE_PRINTING || led_mode == LED_MODE_BLINK_ON_DONE)
    analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);
  if (!card.longFilename[0])
    strcpy(card.longFilename, card.filename);
  card.longFilename[20] = '\0';
  if (strchr(card.longFilename, '.')) strchr(card.longFilename, '.')[0] = '\0';
  
  char buffer[64];
  
  for(uint8_t e=0; e<EXTRUDERS; e++)
    LCD_DETAIL_CACHE_MATERIAL(e) = 0;
  
  for(uint8_t n=0; n<8; n++)
  {
    card.fgets(buffer, sizeof(buffer));
    buffer[sizeof(buffer)-1] = '\0';
    while (strlen(buffer) > 0 && buffer[strlen(buffer)-1] < ' ') buffer[strlen(buffer)-1] = '\0';
    if (strncmp_P(buffer, PSTR(";TIME:"), 6) == 0)
      LCD_DETAIL_CACHE_TIME() = atol(buffer + 6);
    else if (strncmp_P(buffer, PSTR(";MATERIAL:"), 10) == 0)
      LCD_DETAIL_CACHE_MATERIAL(0) = atol(buffer + 10);
#if EXTRUDERS > 1
    else if (strncmp_P(buffer, PSTR(";MATERIAL2:"), 11) == 0)
      LCD_DETAIL_CACHE_MATERIAL(1) = atol(buffer + 11);
#endif
  }
  
  card.setIndex(0);
  
  card.fgets(buffer, sizeof(buffer));
  
  if (strncmp_P(buffer, PSTR(";FLAVOR:UltiGCode"), 17) != 0) {
    card.fgets(buffer, sizeof(buffer));
  }
  
  card.setIndex(0);
  if (strncmp_P(buffer, PSTR(";FLAVOR:UltiGCode"), 17) == 0)
  {
    //New style GCode flavor without start/end code.
    // Temperature settings, filament settings, fan settings, start and end-code are machine controlled.
    target_temperature_bed = 0;
    fanSpeedPercent = 0;
    for(uint8_t e=0; e<EXTRUDERS; e++)
    {
      if (LCD_DETAIL_CACHE_MATERIAL(e) < 1)
        continue;
      target_temperature[e] = 0;//material[e].temperature;
      if (Device_isBedHeat) {
        target_temperature_bed = max(target_temperature_bed, material[e].bed_temperature);
      }
      else{
        target_temperature_bed = 0;
      }
      fanSpeedPercent = max(fanSpeedPercent, material[0].fan_speed);
      volume_to_filament_length[e] = 1.0 / (M_PI * (material[e].diameter / 2.0) * (material[e].diameter / 2.0));
      extrudemultiply[e] = material[e].flow;
    }
    
    fanSpeed = 0;
    resumeState=RESUME_STATE_NORMAL;
    
    enquecommand_P(PSTR("G28\nG1 F6000 Z50\nG1 X-10 Y"Y_MIN_POS_STR " Z" PRIMING_HEIGHT_STRING));
    
    lcd_change_to_menu(lcd_menu_print_heatup);
  }else{
    //Classic gcode file
    card.setIndex(0);
    //Set the settings to defaults so the classic GCode has full control
    fanSpeedPercent = 100;
    for(uint8_t e=0; e<EXTRUDERS; e++)
    {
      volume_to_filament_length[e] = 1.0;
      extrudemultiply[e] = 100;
    }
    
    doHotendHome();
    currentMenu = lcd_menu_print_prepare_printing;
    nextEncoderPos = 0;
  }
  
}

void lcd_menu_print_gate()
{
  if (!gateOpened()) {
    if (SDUPSIsWorking()) {
      lcd_change_to_menu(lcd_menu_print_resume_option,SCROLL_MENU_ITEM_POS(0), MenuForward);
    }
    else{
      lcd_clear_cache();
      lcd_change_to_menu(lcd_menu_print_select, SCROLL_MENU_ITEM_POS(0), MenuForward);
    }
  }
  
  lcd_info_screen(lcd_menu_main, NULL, LS(PSTR("CANCEL"),
                                          PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                          PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(15, 11, 11) , LS(PSTR("Please close"),
                                                  PSTR("\xB2" "\x80"  "\x9B" "\x80"  "\xCF" "\x81"  ),
                                                  PSTR("\xD7" "\x82"  "\xD8" "\x82"  " ""\xFC" "\x83"  "\x86" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(25, 24, 24) , LS(PSTR("the gate!"),
                                                  PSTR("\x83" "\x82"  "\xA3" "\x80"  "\x84" "\x82"  "\xF4" "\x80"  "\x9B" "\x82"  "!"),
                                                  PSTR("\xFD" "\x83"  "\xFE" "\x83"  "\x92" "\x83"  "\xF9" "\x82"  "\xFA" "\x82"  "!")) );
}

void lcd_menu_print_select()
{
  LED_NORMAL();
  if (!card.sdInserted)
  {
    LED_GLOW();
    lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);
    lcd_info_screen(lcd_menu_main);
    
    lcd_lib_draw_string_centerP(LS(15, 11, 11) , LS(PSTR("No SD-CARD!"),
                                                    PSTR("\xC6" "\x81"  "\xCC" "\x81"  "\xCD" "\x81"  "\x89" "\x81"  "SD" "\x9E" "\x80"  ),
                                                    PSTR("No SD-CARD!")) );
    lcd_lib_draw_string_centerP(LS(25, 24, 24), LS(PSTR("Please insert card"),
                                                   PSTR("\xB2" "\x80"  "\x85" "\x81"  "\xDD" "\x80"  "SD" "\x9E" "\x80"  ),
                                                   PSTR("SD ""\xEB" "\x82"  "\xEC" "\x82"  " ""\xA6" "\x83"  "\xA7" "\x83"  )) );
    lcd_clear_cache();
    return;
  }
  if (!card.isOk())
  {
    lcd_info_screen(lcd_menu_main);
    lcd_lib_draw_string_centerP(LS(16, 16, 16), LS(PSTR("Reading card..."),
                                                   PSTR("SD""\x9E" "\x80"  "\x9D" "\x81"  "\xD8" "\x80"  "\xC1" "\x80"  "..." ),
                                                   PSTR("Reading card...")) );
    lcd_clear_cache();
    return;
  }
  
  if (LCD_CACHE_NR_OF_FILES() == 0xFF)
    LCD_CACHE_NR_OF_FILES() = card.getnrfilenames();
  if (card.errorCode())
  {
    LCD_CACHE_NR_OF_FILES() = 0xFF;
    return;
  }
  uint8_t nrOfFiles = LCD_CACHE_NR_OF_FILES();
  if (nrOfFiles == 0)
  {
    if (card.atRoot())
      lcd_info_screen(lcd_menu_main, NULL, LS(PSTR("OK"),
                                              PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                              PSTR("OK")) );
    else
      lcd_info_screen(lcd_menu_print_select, cardUpdir, LS(PSTR("OK"),
                                                           PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                                           PSTR("OK")) );
    lcd_lib_draw_string_centerP(25, LS(PSTR("No files found!"),
                                       PSTR("\x85" "\x82"  "\xC8" "\x80"  "\xF2" "\x81"  "\x89" "\x81"  "\x86" "\x82"  "\x87" "\x82"  "\xF6" "\x81"  "\xD6" "\x81"  "!"),
                                       PSTR("\xB0" "\x83"  "\xFF" "\x83"  " ""\xF0" "\x83"  "\xF1" "\x83"  " ""\xF2" "\x83"  "\xC1" "\x83"  "\x9C" "\x83"  "\x9D" "\x83"  )) );
    lcd_clear_cache();
    return;
  }
  
  if (lcd_lib_button_pressed)
  {
    uint8_t selIndex = uint16_t(SELECTED_SCROLL_MENU_ITEM());
    if (selIndex == 0)
    {
      if (card.atRoot())
      {
        lcd_change_to_menu(lcd_menu_main, SCROLL_MENU_ITEM_POS(0), MenuBackward);
      }else{
        lcd_clear_cache();
        lcd_lib_beep();
        card.updir();
        
        lcd_change_to_menu(lcd_menu_print_select,SCROLL_MENU_ITEM_POS(0),MenuBackward);
      }
    }else{
      card.getfilename(selIndex - 1);
      if (!card.filenameIsDir)
      {
        if (Device_isGate) {
          if (gateOpened()) {
            lcd_change_to_menu(lcd_menu_print_gate);
            return;
          }
        }
        //Start print
        card.openFile(card.filename, true);
        SERIAL_DEBUGLNPGM("file start:");
        SERIAL_DEBUGLN(card.filename);
        if (card.isFileOpen() && !is_command_queued() && !isCommandInBuffer())
        {
          doPreparePrint();
        }
      }else{
        lcd_lib_beep();
        lcd_clear_cache();
        card.chdir(card.filename);
        lcd_change_to_menu(lcd_menu_print_select,SCROLL_MENU_ITEM_POS(0),MenuForward);
      }
      return;//Return so we do not continue after changing the directory or selecting a file. The nrOfFiles is invalid at this point.
    }
  }
  
  lcd_scroll_menu(LS(PSTR("SD CARD"),
                     PSTR("SD" "\x9E" "\x80"  ),
                     PSTR("SD" "\xEB" "\x82"  "\xEC" "\x82"  )) , nrOfFiles+1, lcd_sd_menu_filename_callback, lcd_sd_menu_details_callback);
}

/****************************************************************************************
 * heating up
 *
 ****************************************************************************************/

#define PREPARE_PRINTING_INIT 0
#define PREPARE_PRINTING_CLEAR_SDUPS_POSITION 1
#define PREPARE_PRINTING_CLEAR_SDUPS_POSITION_VALIDATE 2
#define PREPARE_PRINTING_STORE_SD_CLASS 3
#define PREPARE_PRINTING_STORE_SD_CLASS_VALIDATE 4
#define PREPARE_PRINTING_STORE_LCD_CACHE 5
#define PREPARE_PRINTING_STORE_FINISH 6
#define PREPARE_PRINTING_STORE_LCD_CACHE_VALIDATE 7

static uint8_t preparePrintingState=PREPARE_PRINTING_INIT;

static void doHotendHome()
{
  preparePrintingState=PREPARE_PRINTING_INIT;
  starttime = millis();
}

static void lcd_menu_print_heatup()
{
  LED_GLOW_HEAT();
  
  lcd_info_screen(lcd_menu_print_abort, NULL, LS(PSTR("ABORT"),
                                                 PSTR("\xAC" "\x80"  "\xAD" "\x80"  ),
                                                 PSTR("\xE5" "\x83"  "\xD7" "\x82"  "\xDB" "\x82"  )) , MenuForward);
  
  if (Device_isGate) {
    if (gateOpened()) {
      abortPrint();
      lcd_change_to_menu(lcd_menu_print_gate);
      return;
    }
  }
  
  if (Device_isBedHeat) {
    if (current_temperature_bed > target_temperature_bed - 6)
    {
      for(uint8_t e=0; e<EXTRUDERS; e++)
      {
        if (LCD_DETAIL_CACHE_MATERIAL(e) < 1 || target_temperature[e] > 0)
          continue;
        target_temperature[e] = material[e].temperature;
      }
      
      if (current_temperature_bed >= target_temperature_bed - TEMP_WINDOW * 2 && !is_command_queued() && !isCommandInBuffer())
      {
        for(uint8_t e=0; e<EXTRUDERS; e++)
        {
          if (current_temperature[e] < target_temperature[e] - TEMP_HYSTERESIS || current_temperature[e] > target_temperature[e] + TEMP_HYSTERESIS) {
            menuTimer=millis();
          }
        }
        
        if (millis()-menuTimer>=TEMP_RESIDENCY_TIME*1000UL && printing_state == PRINT_STATE_NORMAL)
        {
          doHotendHome();
          currentMenu = lcd_menu_print_prepare_printing;
          nextEncoderPos = 0;
        }
      }
    }
  }
  else {
    for(uint8_t e=0; e<EXTRUDERS; e++)
    {
      if (LCD_DETAIL_CACHE_MATERIAL(e) < 1 || target_temperature[e] > 0)
        continue;
      target_temperature[e] = material[e].temperature;
    }
    
    for(uint8_t e=0; e<EXTRUDERS; e++)
    {
      if (current_temperature[e] < target_temperature[e] - TEMP_HYSTERESIS || current_temperature[e] > target_temperature[e] + TEMP_HYSTERESIS) {
        menuTimer=millis();
      }
    }
    
    if (millis()-menuTimer>=TEMP_RESIDENCY_TIME*1000UL && printing_state == PRINT_STATE_NORMAL)
    {
      doHotendHome();
      currentMenu = lcd_menu_print_prepare_printing;
      nextEncoderPos = 0;
    }
  }
  
  uint8_t progress = 125;
  for(uint8_t e=0; e<EXTRUDERS; e++)
  {
    if (LCD_DETAIL_CACHE_MATERIAL(e) < 1 || target_temperature[e] < 1)
      continue;
    if (current_temperature[e] > 20)
      progress = min(progress, (current_temperature[e] - 20) * 125 / (target_temperature[e] - 20 - TEMP_WINDOW));
    else
      progress = 0;
  }
  if (Device_isBedHeat) {
    
    if (current_temperature_bed > 20)
      progress = min(progress, (current_temperature_bed - 20) * 125 / (target_temperature_bed - 20 - TEMP_WINDOW));
    else
      progress = 0;
  }
  if (progress < minProgress)
    progress = minProgress;
  else
    minProgress = progress;
  
  lcd_lib_draw_string_centerP(LS(10, 25, 25) , LS(PSTR("Heating up..."),
                                                  PSTR("\x90" "\x80"  "\x8B" "\x80"  "\xC1" "\x80"  "..."),
                                                  PSTR("\x90" "\x83"  "\x91" "\x83"  "\xC0" "\x82"  "...")) );
  lcd_lib_draw_string_centerP(LS(20, 1, 1) , LS(PSTR("Preparing to print:"),
                                                PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\xFF" "\x80"  "\xEB" "\x80"  "\x9A" "\x80"  "\xDF" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  ":"),
                                                PSTR("\xA9" "\x83"  "\xA7" "\x84"   "\xC3" "\x83"  "\x8F" "\x83"  "\xC0" "\x82"  ":")) );
  lcd_lib_draw_string_center(LS(30, 13, 13) , card.longFilename);
  
  lcd_progressbar(progress);
}

/****************************************************************************************
 * prepare printing
 *
 ****************************************************************************************/
static void doStartPrint()
{
  char buffer[64];
  char *bufferPtr;
  uint8_t cardErrorTimes=0;
  
  retracted = false;
  // note that we have primed, so that we know to de-prime at the end
  
  current_position[E_AXIS]=0;
  plan_set_e_position(current_position[E_AXIS]);
  
  for(uint8_t e = 0; e<EXTRUDERS; e++)
  {
    if (!LCD_DETAIL_CACHE_MATERIAL(e))
      continue;
    
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
        
        if (resumeState) {
          strcpy_P(bufferPtr, PSTR("E0\n"));
          bufferPtr+=strlen_P(PSTR("E0\n"));
        }
        else{
          strcpy_P(bufferPtr, PSTR("X10 E0\n"));
          bufferPtr+=strlen_P(PSTR("X10 E0\n"));
        }
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
        
        if (resumeState) {
          strcpy_P(bufferPtr, PSTR("E0\n"));
          bufferPtr+=strlen_P(PSTR("E0\n"));
        }
        else{
          strcpy_P(bufferPtr, PSTR("X10 E0\n"));
          bufferPtr+=strlen_P(PSTR("X10 E0\n"));
        }
        
      }
    }
    enquecommand(buffer);
  }
  
  
  if (resumeState) {
    
    bufferPtr=buffer;
    
    strcpy_P(bufferPtr, PSTR("G92 E"));
    bufferPtr+=strlen_P(PSTR("G92 E"));
    
    ltoa(long(SDUPSCurrentPosition[E_AXIS]), bufferPtr, 10);
    bufferPtr=buffer+strlen(buffer);
    
    
    strcpy_P(bufferPtr, PSTR("\nG1 F3000 X"));
    bufferPtr+=strlen_P(PSTR("\nG1 F3000 X"));
    
    bufferPtr=float_to_string(SDUPSCurrentPosition[X_AXIS], bufferPtr);
    
    strcpy_P(bufferPtr, PSTR(" Y"));
    bufferPtr+=strlen_P(PSTR(" Y"));
    
    bufferPtr=float_to_string(SDUPSCurrentPosition[Y_AXIS], bufferPtr);
    
    strcpy_P(bufferPtr, PSTR(" Z"));
    bufferPtr+=strlen_P(PSTR(" Z"));
    
    bufferPtr=float_to_string(SDUPSCurrentPosition[Z_AXIS], bufferPtr);
    
    if (SDUPSFeedrate>0.0) {
      strcpy_P(bufferPtr, PSTR("\nG1 F"));
      bufferPtr+=strlen_P(PSTR("\nG1 F"));
      
      bufferPtr=int_to_string(SDUPSFeedrate, bufferPtr);
    }
    else{
      strcpy_P(bufferPtr, PSTR("\nG1 F1000"));
      bufferPtr+=strlen_P(PSTR("\nG1 F1000"));
    }
    
    enquecommand(buffer);
    
    enquecommand_P(PSTR("M107\nM220 S40\nM780 S255\nM781 S100"));
    
  }else{
    if (LCD_DETAIL_CACHE_MATERIAL(0)) {
      enquecommand_P(PSTR("G1 F400 Z1"));
    }
  }
  
  SDUPSCurrentPosition[Z_AXIS] = 0.0;
  
  postMenuCheck = checkPrintFinished;
  lifetime_stats_print_start();
  card.startFileprint();
  starttime = millis();
}



static void lcd_menu_print_prepare_printing()
{
  
  LED_PRINT();
  
  lcd_info_screen(NULL, NULL, LS(PSTR("WAITING"),
                                 PSTR("\xB3" "\x80"  "\xB4" "\x80"  "\xC1" "\x80"  ),
                                 PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(10, 1, 1) , LS(PSTR("Prepare Printing:"),
                                                PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\xFF" "\x80"  "\xEB" "\x80"  "\x9A" "\x80"  "\xDF" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  ":"),
                                                PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\xC3" "\x83"  "\x8F" "\x83"  ":")) );
  lcd_lib_draw_string_center(LS(20, 13, 13) , card.longFilename);

  static int EEPROMSDUPSClassIndex=EEPROM_SDUPS_CLASS_OFFSET;
  
  uint16_t SDUPSValidate;
  uint8_t cardErrorTimes;
  char* bufferPtr;
  
  if (printing_state==PRINT_STATE_NORMAL && is_command_queued()==false && isCommandInBuffer()==false && movesplanned()==0) {
    
    if (resumeState) {
      preparePrintingState=PREPARE_PRINTING_STORE_FINISH;
    }
    
    switch (preparePrintingState) {
      case PREPARE_PRINTING_INIT:
//        SERIAL_DEBUGLNPGM("PREPARE_PRINTING_INIT");
        EEPROMSDUPSClassIndex=EEPROM_SDUPS_CLASS_OFFSET;
        preparePrintingState=PREPARE_PRINTING_CLEAR_SDUPS_POSITION;
        break;
      case PREPARE_PRINTING_CLEAR_SDUPS_POSITION:
//        SERIAL_DEBUGLNPGM("PREPARE_PRINTING_CLEAR_SDUPS_POSITION");
        for (int index=0; index<(SDUPSPositionSize*(sizeof(uint32_t)+sizeof(uint16_t))); index++) {
          eeprom_write_byte((uint8_t*)EEPROM_SDUPS_POSITION_OFFSET+index, (uint8_t)0x00);
        }
        preparePrintingState=PREPARE_PRINTING_CLEAR_SDUPS_POSITION_VALIDATE;
        break;
      case PREPARE_PRINTING_CLEAR_SDUPS_POSITION_VALIDATE:
//        SERIAL_DEBUGLNPGM("PREPARE_PRINTING_CLEAR_SDUPS_POSITION_VALIDATE");
        for (int index=0; index<(SDUPSPositionSize*(sizeof(uint32_t)+sizeof(uint16_t))); index++) {
          if (eeprom_read_byte((uint8_t*)EEPROM_SDUPS_POSITION_OFFSET+index)) {
            SERIAL_ERRORLN("eeprom broken!!!");
            SERIAL_ERRORLN(index);
          }
        }
        preparePrintingState=PREPARE_PRINTING_STORE_LCD_CACHE;
        break;
      case PREPARE_PRINTING_STORE_LCD_CACHE:
//        SERIAL_DEBUGLNPGM("PREPARE_PRINTING_STORE_LCD_CACHE");
        EEPROM_WRITE_VAR(EEPROMSDUPSClassIndex, lcd_cache);
        bufferPtr = (char *)&lcd_cache;
        SDUPSValidate = 0;
        for (int i=0; i<sizeof(lcd_cache); i++) {
          SDUPSValidate += *(bufferPtr+i);
        }
        EEPROM_WRITE_VAR(EEPROMSDUPSClassIndex, SDUPSValidate);
        preparePrintingState=PREPARE_PRINTING_STORE_SD_CLASS;
        break;
      case PREPARE_PRINTING_STORE_SD_CLASS:
//        SERIAL_DEBUGLNPGM("PREPARE_PRINTING_STORE_SD_CLASS");
        bufferPtr = card.getFullPath();
        if (bufferPtr != NULL) {
          int bufferLength = strlen(bufferPtr);
          SDUPSValidate = 0;
          for (int i=0; i<bufferLength; i++) {
            SDUPSValidate += *(bufferPtr+i);
          }
          uint32_t fileSize = card.getFileSize();
          _EEPROM_writeData(EEPROMSDUPSClassIndex, (uint8_t*)card.longFilename, 20);
          EEPROM_WRITE_VAR(EEPROMSDUPSClassIndex, fileSize);
          EEPROM_WRITE_VAR(EEPROMSDUPSClassIndex, SDUPSValidate);
          EEPROM_WRITE_VAR(EEPROMSDUPSClassIndex, bufferLength);
          _EEPROM_writeData(EEPROMSDUPSClassIndex, (uint8_t*)bufferPtr, bufferLength);
          free(bufferPtr);
        }
        preparePrintingState=PREPARE_PRINTING_STORE_FINISH;
        break;
      case PREPARE_PRINTING_STORE_FINISH:
//        SERIAL_DEBUGLNPGM("PREPARE_PRINTING_STORE_FINISH");
        doStartPrint();
        currentMenu=lcd_menu_print_printing;
        break;
      default:
        break;
    }
  }
  
  lcd_lib_draw_string_center_atP(25, LS(30, 25, 25) , PSTR("--:--:--"));
  lcd_lib_draw_string_center_atP(95, LS(30, 25, 25), PSTR("---:--:--"));

  lcd_progressbar(0);
}

/****************************************************************************************
 * printing
 *
 ****************************************************************************************/
static void lcd_menu_print_printing()
{
  
  LED_PRINT();
  
  uint8_t progress = card.getFilePos() / ((card.getFileSize() + 123) / 124);
  char buffer[32];
  char* c;
  unsigned long totalTimeSec;
  
  lcd_question_screen(lcd_menu_print_tune, NULL, LS(PSTR("TUNE"),
                                                    PSTR("\xA4" "\x81"  "\xB1" "\x81"  ),
                                                    PSTR("\xA5" "\x84"  "\xB9" "\x83"  )) ,
                      lcd_menu_print_abort, NULL, LS(PSTR("PAUSE"),
                                                     PSTR("\x89" "\x82"  "\x82" "\x80"  ),
                                                     PSTR("\x80" "\x84"  "\x81" "\x84"  )) );
  switch(printing_state)
  {
    default:
      lcd_lib_draw_string_centerP(LS(10, 1, 1) , LS(PSTR("Printing:"),
                                                    PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  ":"),
                                                    PSTR("\xDE" "\x82"  "\xDF" "\x82"  "\xF3" "\x83"  ":")) );
      lcd_lib_draw_string_center(LS(20, 13, 13) , card.longFilename);
      break;
    case PRINT_STATE_WAIT_USER:
      lcd_lib_draw_string_centerP(LS(10, 1, 1), LS(PSTR("Press button"),
                                                   PSTR("\xE2" "\x80"  "OK" "\xE4" "\x80"  "\xC6" "\x80"  "\xC7" "\x80"  ),
                                                   PSTR("OK ""\x86" "\x83"  "\x87" "\x83"  " ""\x88" "\x83"  "\x89" "\x83"  )) );
      lcd_lib_draw_string_centerP(LS(20, 13, 13), LS(PSTR("to continue"),
                                                     PSTR(""),
                                                     PSTR("\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  )) );
      break;
    case PRINT_STATE_HEATING:
      lcd_lib_draw_string_centerP(LS(10, 1, 1), LS(PSTR("Heating"),
                                                   PSTR("\x90" "\x80"  "\x8B" "\x80"  "\xC1" "\x80"  ),
                                                   PSTR("\x90" "\x83"  "\x91" "\x83"  "\xC0" "\x82"  )) );
      c = int_to_string(current_temperature[0], buffer, PSTR("`C"));
      *c++ = '/';
      c = int_to_string(target_temperature[0], c, PSTR("`C"));
      lcd_lib_draw_string_center(LS(20, 13, 13), buffer);
      break;
      if (Device_isBedHeat) {
      case PRINT_STATE_HEATING_BED:
        lcd_lib_draw_string_centerP(LS(10, 1, 1), LS(PSTR("Heating buildplate"),
                                                     PSTR("\x90" "\x80"  "\x8B" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\xFC" "\x80"  "\xC1" "\x80"  ),
                                                     PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"   "\x90" "\x83"  "\x91" "\x83"  )) );
        c = int_to_string(current_temperature_bed, buffer, PSTR("`C"));
        *c++ = '/';
        c = int_to_string(target_temperature_bed, c, PSTR("`C"));
        lcd_lib_draw_string_center(LS(20, 13, 13), buffer);
        break;
      }
  }
  
  float printTimeMs = (millis() - starttime + pausetime);
  float printTimeSec = printTimeMs / 1000L;
  
  unsigned long timeLeftSec;
  
  float totalTimeMs = float(printTimeMs) * float(card.getFileSize()) / float(card.getFilePos());
  static float totalTimeSmoothSec;
  
  if (millis() - starttime < 50000UL) {
    totalTimeSmoothSec = totalTimeMs /1000;
  }
  totalTimeSmoothSec = (totalTimeSmoothSec * 999.0 + totalTimeMs / 1000.0) / 1000.0;
  if (isinf(totalTimeSmoothSec))
    totalTimeSmoothSec = totalTimeMs /1000;
  
  int_to_time_string(printTimeSec, buffer);
  
  if (printing_state == PRINT_STATE_HEATING || printing_state == PRINT_STATE_HEATING_BED) {
    lcd_lib_draw_string_center_atP(25, LS(30, 25, 25) , PSTR("--:--:--"));
  }
  else{
    lcd_lib_draw_string_center_at(25, LS(30, 25, 25) , buffer);
  }
  
  if (LCD_DETAIL_CACHE_TIME() == 0)
  {
    if (millis() - starttime < 180000UL || printing_state == PRINT_STATE_HEATING || printing_state == PRINT_STATE_HEATING_BED) {
      lcd_lib_draw_string_center_atP(95, LS(30, 25, 25) , PSTR("---:--:--"));
    }
    else{
      totalTimeSec = max(totalTimeSmoothSec, printTimeSec);
      
      int_to_time_string(totalTimeSec - printTimeSec, buffer+1);
      buffer[0]='-';
      
      lcd_lib_draw_string_center_at(95, LS(30, 25, 25) , buffer);
    }
    
  }else{
    
    totalTimeSmoothSec = constrain(totalTimeSmoothSec, LCD_DETAIL_CACHE_TIME()/2, 4*LCD_DETAIL_CACHE_TIME());
    
    if (card.getFilePos() < card.getFileSize() / 2) {
      totalTimeSec= max(LCD_DETAIL_CACHE_TIME(), printTimeSec);
    }
    else if (card.getFilePos() >= card.getFileSize() / 2 && card.getFilePos() < card.getFileSize() * 3 / 4) {
      float f = float(card.getFileSize()) / float(card.getFilePos()) - 1.0;
      f=constrain(f, 0.0, 1.0);
      totalTimeSec = max(float(totalTimeSmoothSec) * (1-f) + float(LCD_DETAIL_CACHE_TIME()) * (f), printTimeSec);
    }
    else{
      totalTimeSec = max(totalTimeSmoothSec, printTimeSec);
    }
    
    int_to_time_string(totalTimeSec - printTimeSec, buffer+1);
    buffer[0]='-';
    
    lcd_lib_draw_string_center_at(95, LS(30, 25, 25) , buffer);
  }
  
  lcd_progressbar(progress);
}

/****************************************************************************************
 * tune
 *
 ****************************************************************************************/
static char* tune_item_callback(uint8_t nr)
{
  char* c = (char*)lcd_cache;
  
  if (nr == 0)
    strcpy_P(c, LS(PSTR("Return"),
                   PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                   PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P(c, LS(PSTR("Speed"),
                   PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xC3" "\x81"  "\x89" "\x80"  ),
                   PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) );
  else if (nr == 2)
    strcpy_P(c, LS(PSTR("Temperature"),
                   PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ),
                   PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  )) );
#if EXTRUDERS > 1
  else if (nr == 3)
    strcpy_P(c, LS(PSTR("Temperature 2"),
                   PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  " 2"),
                   PSTR("\xD5" "\x82"  "\xD6" "\x82"  "2 ""\xC4" "\x82"  "\xC5" "\x82"  )) );
#endif
  else if (nr == 2 + EXTRUDERS && Device_isBedHeat)
    strcpy_P(c, LS(PSTR("Buildplate temp."),
                   PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xFB" "\x80"  "\x8A" "\x82"  "\x88" "\x80"  "\x89" "\x80"  ),
                   PSTR("\xA1" "\x83"  "\xA2" "\x83"  "\xC3" "\x82"  " ""\x90" "\x83"  "\x91" "\x83"  )) );
  else if (nr == 3 + EXTRUDERS - !Device_isBedHeat)
    strcpy_P(c, LS(PSTR("Fan speed"),
                   PSTR("\xC0" "\x81"  "\xC1" "\x81"  "\xC3" "\x81"  "\xEA" "\x81"  ),
                   PSTR("\xD2" "\x83"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) );
  else if (nr == 4 + EXTRUDERS - !Device_isBedHeat)
    strcpy_P(c, LS(PSTR("Material flow"),
                   PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  ),
                   PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  )) );
#if EXTRUDERS > 1
  else if (nr == 5 + EXTRUDERS - !Device_isBedHeat)
    strcpy_P(c, LS(PSTR("Material flow 2"),
                   PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  "2"),
                   PSTR("\x96" "\x83"  "\x97" "\x83"  "2 ""\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  )) );
#endif
  else if (nr == 4 + EXTRUDERS * 2 - !Device_isBedHeat)
    strcpy_P(c, LS(PSTR("Retraction"),
                   PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xBE" "\x81"  "\xCB" "\x81"  ),
                   PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
  return c;
}

static void tune_item_details_callback(uint8_t nr)
{
  char* c = (char*)lcd_cache;
  if (nr == 1)
    c = int_to_string(feedmultiply, c, PSTR("%"));
  else if (nr == 2)
  {
    c = int_to_string(current_temperature[0], c, PSTR("`C"));
    *c++ = '/';
    c = int_to_string(target_temperature[0], c, PSTR("`C"));
  }
#if EXTRUDERS > 1
  else if (nr == 3)
  {
    c = int_to_string(current_temperature[1], c, PSTR("`C"));
    *c++ = '/';
    c = int_to_string(target_temperature[1], c, PSTR("`C"));
  }
#endif
  else if (nr == 2 + EXTRUDERS  && Device_isBedHeat)
  {
    c = int_to_string(current_temperature_bed, c, PSTR("`C"));
    *c++ = '/';
    c = int_to_string(target_temperature_bed, c, PSTR("`C"));
  }
  else if (nr == 3 + EXTRUDERS - !Device_isBedHeat)
    c = int_to_string(lround((int(fanSpeed) * 100) / 255.0), c, PSTR("%"));
  else if (nr == 4 + EXTRUDERS - !Device_isBedHeat)
    c = int_to_string(extrudemultiply[0], c, PSTR("%"));
#if EXTRUDERS > 1
  else if (nr == 5 + EXTRUDERS - !Device_isBedHeat)
    c = int_to_string(extrudemultiply[1], c, PSTR("%"));
#endif
  else
    return;
  lcd_draw_detail((char*)lcd_cache);
}

static void lcd_menu_print_tune()
{
  LED_PRINT();
  
  lcd_scroll_menu(LS(PSTR("TUNE"),
                     PSTR("\xA4" "\x81"  "\xB1" "\x81"  ),
                     PSTR("\xA5" "\x84"  "\xB9" "\x83"  )) , 5 + EXTRUDERS * 2 - !Device_isBedHeat, tune_item_callback, tune_item_details_callback);
  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
    {
      if (card.sdprinting)
        lcd_change_to_menu(lcd_menu_print_printing, MAIN_MENU_ITEM_POS(0), MenuBackward);
      else
        lcd_change_to_menu(lcd_menu_print_heatup, MAIN_MENU_ITEM_POS(0), MenuBackward);
    }
    else if (IS_SELECTED_SCROLL(1))
    {
      targetFeedmultiply=0;
      LCD_EDIT_SETTING(feedmultiply, LS(PSTR("Print speed"),
                                        PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xC3" "\x81"  "\x89" "\x80"  ),
                                        PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) , LS(PSTR("%"),
                                                       PSTR("%"),
                                                       PSTR("%")) , 10, 1000);
    }
    else if (IS_SELECTED_SCROLL(2))
    {
      lcd_lib_button_up_down_reversed = true;
      lcd_change_to_menu(lcd_menu_print_tune_heatup_nozzle0, 0);
    }
#if EXTRUDERS > 1
    else if (IS_SELECTED_SCROLL(3))
      lcd_change_to_menu(lcd_menu_print_tune_heatup_nozzle1, 0);
#endif
    else if (IS_SELECTED_SCROLL(2 + EXTRUDERS)  && Device_isBedHeat)
    {
      lcd_change_to_menu(lcd_menu_maintenance_advanced_bed_heatup, 0);//Use the maintainace heatup menu, which shows the current temperature.
      lcd_lib_button_up_down_reversed = true;
    }
    else if (IS_SELECTED_SCROLL(3 + EXTRUDERS - !Device_isBedHeat))
    {
      targetFanSpeed=0;
      LCD_EDIT_SETTING_BYTE_PERCENT(fanSpeed, LS(PSTR("Fan speed"),
                                                 PSTR("\xC0" "\x81"  "\xC1" "\x81"  "\xC3" "\x81"  "\xEA" "\x81"  ),
                                                 PSTR("\xD2" "\x83"  " ""\xD3" "\x83"  "\xC5" "\x82"  )) ,  LS(PSTR("%"),
                                                                 PSTR("%"),
                                                                 PSTR("%")), 0, 100);
    }
    else if (IS_SELECTED_SCROLL(4 + EXTRUDERS - !Device_isBedHeat))
      LCD_EDIT_SETTING(extrudemultiply[0], LS(PSTR("Material flow"),
                                              PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  ),
                                              PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  )) , LS(PSTR("%"),
                                                             PSTR("%"),
                                                             PSTR("%")), 10, 1000);
#if EXTRUDERS > 1
    else if (IS_SELECTED_SCROLL(5 + EXTRUDERS - !Device_isBedHeat))
      LCD_EDIT_SETTING(extrudemultiply[1], LS(PSTR("Material flow 2"),
                                              PSTR("\xF5" "\x80"  "\xF6" "\x80"  "\x8A" "\x81"  "\x8B" "\x81"  "\xEA" "\x81"  "2"),
                                              PSTR("\x96" "\x83"  "\x97" "\x83"  "2 ""\xA8" "\x83"  "\xD7" "\x83"  "\xE8" "\x83"  )) , LS(PSTR("%"),
                                                             PSTR("%"),
                                                             PSTR("%")), 10, 1000);
#endif
    else if (IS_SELECTED_SCROLL(4 + EXTRUDERS * 2 - !Device_isBedHeat))
      lcd_change_to_menu(lcd_menu_print_tune_retraction);
  }
}

/****************************************************************************************
 * nozzle temperature
 *
 ****************************************************************************************/
static void lcd_menu_print_tune_heatup_nozzle0()
{
  LED_PRINT();
  if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
  {
    target_temperature[0] = int(target_temperature[0]) + (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM);
    if (target_temperature[0] < 0)
      target_temperature[0] = 0;
    if (target_temperature[0] > HEATER_0_MAXTEMP - 15)
      target_temperature[0] = HEATER_0_MAXTEMP - 15;
    lcd_lib_encoder_pos = 0;
  }
  if (lcd_lib_button_pressed)
  {
    lcd_change_to_menu(previousMenu, previousEncoderPos, MenuBackward);
    lcd_lib_button_up_down_reversed = false;
  }
  
  
  char buffer[18];
  int_to_string(int(current_temperature[0]), buffer, PSTR("`C/"));
  int_to_string(int(target_temperature[0]), buffer+strlen(buffer), PSTR("`C"));
  
  lcd_lib_clear();
  
  lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("Nozzle temperature:"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x88" "\x80"  "\x89" "\x80"  ":"),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xC4" "\x82"  "\xC5" "\x82"  )) );
  
  if (temp_error_handle) {
    if (temp_error_handle & 0x01) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37) , LS(PSTR("Error: PT1 Max Temp"),
                                                      PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT1" "\xC4" "\x81"  "\x88" "\x80"  ),
                                                      PSTR("Error: PT1 Max Temp")) );
    }
    else if (temp_error_handle & 0x02) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37), LS(PSTR("Error: PT1 Min Temp"),
                                                     PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT1" "\xC5" "\x81"  "\x88" "\x80"  ),
                                                     PSTR("Error: PT1 Min Temp")) );
    }
    else if (temp_error_handle & 0x04) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37), LS(PSTR("Error: PT2 Max Temp"),
                                                     PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT2" "\xC4" "\x81"  "\x88" "\x80"  ),
                                                     PSTR("Error: PT2 Max Temp")) );
    }
    else if (temp_error_handle & 0x08) {
      lcd_lib_draw_string_centerP(LS(40, 37, 37), LS(PSTR("Error: PT2 Min Temp"),
                                                     PSTR("\x80" "\x80"  "\x81" "\x80"   ":PT2" "\xC5" "\x81"  "\x88" "\x80"  ),
                                                     PSTR("Error: PT2 Min Temp")) );
    }
    else{
      //Should not come here.
      lcd_lib_draw_string_centerP(LS(40, 37, 37), LS(PSTR("Error: Unknown"),
                                                     PSTR("\x80" "\x80"  "\x81" "\x80"  ":" "\xC6" "\x81"  "\xC7" "\x81"  ),
                                                     PSTR("Error: Unknown")) );
    }
  }
  lcd_lib_draw_string_centerP(LS(56, 56-3, 56-3) , LS(PSTR("Click to return"),
                                                      PSTR("\x9E" "\x81"  "\x9F" "\x81"  "OK""\xE4" "\x80"  "\xBD" "\x81"  "\xBE" "\x81"  ),
                                                      PSTR("Click to return")) );
  lcd_lib_draw_string_center(LS(30, 24, 24) , buffer);
}
#if EXTRUDERS > 1
static void lcd_menu_print_tune_heatup_nozzle1()
{
  if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
  {
    target_temperature[1] = int(target_temperature[1]) + (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM);
    if (target_temperature[1] < 0)
      target_temperature[1] = 0;
    if (target_temperature[1] > HEATER_0_MAXTEMP - 15)
      target_temperature[1] = HEATER_0_MAXTEMP - 15;
    lcd_lib_encoder_pos = 0;
  }
  if (lcd_lib_button_pressed)
    lcd_change_to_menu(previousMenu, previousEncoderPos, false);
  char buffer[18];
  int_to_string(int(current_temperature[1]), buffer, PSTR("`C/"));
  int_to_string(int(target_temperature[1]), buffer+strlen(buffer), PSTR("`C"));
  
  lcd_lib_clear();
  
  lcd_lib_draw_string_centerP(LS(20, 11, 11) , LS(PSTR("Nozzle2 temperature:"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "2 ""\x88" "\x80"  "\x89" "\x80"  ":"),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  "2 ""\xC4" "\x82"  "\xC5" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(53, 53 - 3, 53 - 3) , LS(PSTR("Click to return"),
                                                          PSTR("\x9E" "\x81"  "\x9F" "\x81"  "OK""\xE4" "\x80"  "\xBD" "\x81"  "\xBE" "\x81"  ),
                                                          PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  lcd_lib_draw_string_center(LS(30, 23, 23) , buffer);
}
#endif

/****************************************************************************************
 * retraction setting
 *
 ****************************************************************************************/
static char* lcd_retraction_item(uint8_t nr)
{
  if (nr == 0)
    strcpy_P((char*)lcd_cache, LS(PSTR("Return"),
                                  PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xD2" "\x80"  "\xC9" "\x80"  "\xBF" "\x81"  ),
                                  PSTR("\xAA" "\x84"  "\xB4" "\x83"  )) );
  else if (nr == 1)
    strcpy_P((char*)lcd_cache, LS(PSTR("Retract length"),
                                  PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xD0" "\x81"  "\x89" "\x80"  ),
                                  PSTR("\xDA" "\x83"  "\xC9" "\x83"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) );
  else if (nr == 2)
    strcpy_P((char*)lcd_cache, LS(PSTR("Retract speed"),
                                  PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xC3" "\x81"  "\x89" "\x80"  ),
                                  PSTR("\xD3" "\x83"  "\xC5" "\x82"  " ""\x83" "\x84"  "\x84" "\x84"  )) );
#if EXTRUDERS > 1
  else if (nr == 3)
    strcpy_P((char*)lcd_cache, LS(PSTR("Extruder change len"),
                                  PSTR("Extruder change len"),
                                  PSTR("Extruder change len")) );
#endif
  else if (nr == EXTRUDERS+2)
    strcpy_P((char*)lcd_cache, LS(PSTR("Retract Zlift"),
                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x8B" "\x82"  "\xC4" "\x81"  "\xBE" "\x81"  "\xCB" "\x81"  ),
                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) );
  else
    strcpy_P((char*)lcd_cache, PSTR("???"));
  
  return (char*)lcd_cache;
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
#if EXTRUDERS > 1
  else if(nr == 3)
    int_to_string(extruder_swap_retract_length, buffer, PSTR("mm"));
#endif
  else if(nr == EXTRUDERS+2)
    float_to_string(retract_zlift, buffer, PSTR("mm"));
  lcd_draw_detail(buffer);
  //    lcd_lib_draw_string(5, 57, buffer);
}

static void lcd_menu_print_tune_retraction()
{
  LED_PRINT();
  
  lcd_scroll_menu(LS(PSTR("RETRACTION"),
                     PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xBE" "\x81"  "\xCB" "\x81"  ),
                     PSTR("\xB6" "\x83"  "\x96" "\x84"  " ""\xB2" "\x83"  "\x97" "\x84"  )) , 3 + (EXTRUDERS > 1 ? 1 : 0), lcd_retraction_item, lcd_retraction_details);
  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_SCROLL(0))
      lcd_change_to_menu(lcd_menu_print_tune, SCROLL_MENU_ITEM_POS(6), MenuBackward);
    else if (IS_SELECTED_SCROLL(1))
      LCD_EDIT_SETTING_FLOAT001(retract_length, LS(PSTR("Retract length"),
                                                   PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xD0" "\x81"  "\x89" "\x80"  ),
                                                   PSTR("\xDA" "\x83"  "\xC9" "\x83"  " ""\xB7" "\x83"  "\xD7" "\x83"  )) , LS(PSTR("mm"),
                                                                  PSTR("mm"),
                                                                  PSTR("mm")), 0, 50);
    else if (IS_SELECTED_SCROLL(2))
      LCD_EDIT_SETTING_SPEED(retract_feedrate, LS(PSTR("Retract speed"),
                                                  PSTR("\xBE" "\x81"  "\xCB" "\x81"  "\xC3" "\x81"  "\x89" "\x80"  ),
                                                  PSTR("\xD3" "\x83"  "\xC5" "\x82"  " ""\x83" "\x84"  "\x84" "\x84"  )) , LS(PSTR("mm/sec"),
                                                                 PSTR("mm/sec"),
                                                                 PSTR("mm/sec")), 0, max_feedrate[E_AXIS] * 60);
#if EXTRUDERS > 1
    else if (IS_SELECTED_SCROLL(3))
      LCD_EDIT_SETTING_FLOAT001(extruder_swap_retract_length, LS(PSTR("Extruder change"),
                                                                 PSTR("Extruder change"),
                                                                 PSTR("Extruder change")) , LS(PSTR("mm"),
                                                                                PSTR("mm"),
                                                                                PSTR("mm")), 0, 50);
#endif
    else if (IS_SELECTED_SCROLL(EXTRUDERS+2))
      LCD_EDIT_SETTING_FLOAT001(retract_zlift, LS(PSTR("retract_zlift"),
                                                  PSTR("\xA0" "\x80"  "\xEC" "\x80"  "\x8B" "\x82"  "\xC4" "\x81"  "\xBE" "\x81"  "\xCB" "\x81"  ),
                                                  PSTR("\xD5" "\x82"  "\xD6" "\x82"  " ""\xB7" "\x83"  "\xD7" "\x83"  ))  , LS(PSTR("mm"),
                                                                  PSTR("mm"),
                                                                  PSTR("mm")), 0, 50);
  }
}


/****************************************************************************************
 * Pause
 *
 ****************************************************************************************/

void doPausePrint()
{
  if (card.sdprinting) {
    card.sdprinting = false;
    postMenuCheck = NULL;
    SDUPSStorePosition(card.getFilePos());
  }
}


void lcd_menu_print_abort()
{
  LED_GLOW();
  
  if (card.sdprinting) {
    lcd_question_screen(NULL, doPausePrint, LS(PSTR("YES"),
                                               PSTR("\x8C" "\x82"  "\xDB" "\x80"  ),
                                               PSTR("\x91" "\x84"  "\xB6" "\x83"  )) , previousMenu, NULL, LS(PSTR("NO"),
                                                                                  PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                                                  PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
    lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Pause the print?"),
                                                    PSTR("\x8C" "\x82"  "\xDB" "\x80"  "\x89" "\x82"  "\x82" "\x80"  "\xAA" "\x80"  "\xAB" "\x80"  "\xAA" "\x81"  "?"),
                                                    PSTR("\xC0" "\x82"  "\xAC" "\x83"  " ""\x9A" "\x83"  "\x85" "\x84"  "\xC0" "\x83"  "\xC1" "\x83"  "\x9C" "\x83"  "\xC2" "\x83"  "?")) );
  }
  else{
    lcd_info_screen(NULL,NULL,LS(PSTR("Waiting"),
                                 PSTR("\xB3" "\x80"  "\xB4" "\x80"  ),
                                 PSTR("\x9E" "\x84"  "\xC9" "\x83"  "\xF3" "\x83"  )) );
    lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Pausing..."),
                                                    PSTR("\xAE" "\x80"  "\xAF" "\x80"  "\x89" "\x82"  "\x82" "\x80"  "..."),
                                                    PSTR("\x80" "\x84"  "\xA6" "\x84"  "...")) );
  }
  
  if (!(isCommandInBuffer()||is_command_queued()||movesplanned()) || ((printing_state == PRINT_STATE_HEATING || printing_state == PRINT_STATE_HEATING_BED) && !card.sdprinting)) {
    lcd_change_to_menu(lcd_menu_print_ready);
    fanSpeed = lround(255 * int(fanSpeedPercent) / 100.0);
    abortPrint();
  }
}

/****************************************************************************************
 * Print Error
 *
 ****************************************************************************************/

static void lcd_menu_print_error()
{
  LED_ERROR();
  
  char buffer[12];
  strcpy_P(buffer, PSTR("Code:"));
  int_to_string(card.errorCode(), buffer+5);
  
  lcd_info_screen(lcd_menu_main, NULL, LS(PSTR("RETURN TO MAIN"),
                                          PSTR("\xBD" "\x81"  "\xBE" "\x81"  "\xCF" "\x80"  "\xD0" "\x80"  "\xD1" "\x80"  ),
                                          PSTR("\xFB" "\x82"  "\xF1" "\x82"  "\xB4" "\x83"  " ""\x86" "\x84"  "\xFE" "\x83"  "\x90" "\x83"  "\x83" "\x83"  )) );
  
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Error while"),
                                                  PSTR("\xAF" "\x80"  "\x9D" "\x81"  "\xD8" "\x80"  "SD" "\x9E" "\x80"  "\xE1" "\x81"  ),
                                                  PSTR("SD ""\xEB" "\x82"  "\xEC" "\x82"  " ""\xB6" "\x83"  "\xEE" "\x83"  "\x9A" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("reading"),
                                                  PSTR("\x8D" "\x82"  "\x8E" "\x82"  "\x80" "\x80"  "\x81" "\x80"  ),
                                                  PSTR("\xBE" "\x82"  "\xB4" "\x83"  " ""\xF6" "\x82"  "\xF7" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(30, 64, 64) , LS(PSTR("SD-card!"),
                                                  PSTR(""),
                                                  PSTR("")) );
  lcd_lib_draw_string_center(LS(40, 37, 37) , buffer);
}

/****************************************************************************************
 * Print Done
 *
 ****************************************************************************************/
static void postPrintReady()
{
  if (led_mode == LED_MODE_BLINK_ON_DONE)
    analogWrite(LED_PIN, 0);
  
  fanSpeed=0;
  LED_NORMAL();
}

static void lcd_menu_print_ready()
{
  LED_GLOW_END();
  
  if (led_mode == LED_MODE_WHILE_PRINTING)
    analogWrite(LED_PIN, 0);
  else if (led_mode == LED_MODE_BLINK_ON_DONE)
    analogWrite(LED_PIN, (led_glow << 1) * int(led_brightness_level) / 100);
  
  lcd_info_screen(lcd_menu_main, postPrintReady, LS(PSTR("BACK TO MENU"),
                                                    PSTR("\xBE" "\x81"  "\x89" "\x81"  "\xD0" "\x80"  "\xD1" "\x80"  ),
                                                    PSTR("\xFB" "\x82"  "\xF1" "\x82"  "\xB4" "\x83"  " ""\x86" "\x84"  "\xFE" "\x83"  "\x90" "\x83"  "\x83" "\x83"  )) );
  if (current_temperature[0] > 60 || (current_temperature_bed > 40 && Device_isBedHeat))
  {
    lcd_lib_draw_string_centerP(LS(15, 11, 11) , LS(PSTR("Printer cooling down"),
                                                    PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xD3" "\x80"  "\xAE" "\x80"  "\xAF" "\x80"  "\xF8" "\x80"  "\x88" "\x80"  ),
                                                    PSTR("\xDE" "\x82"  "\xDF" "\x82"  "\xCA" "\x82"  " ""\x89" "\x84"  "\x8A" "\x84"  "\xC0" "\x82"  )) );
    
    int16_t progress = 124 - (current_temperature[0] - 60);
    if (progress < 0) progress = 0;
    if (progress > 124) progress = 124;
    
    if (progress < minProgress)
      progress = minProgress;
    else
      minProgress = progress;
    
    lcd_progressbar(progress);
    char buffer[18];
    char* c = buffer;
    for(uint8_t e=0; e<EXTRUDERS; e++)
      c = int_to_string(current_temperature[e], c, PSTR("`C   "));
    if (Device_isBedHeat) {
      int_to_string(current_temperature_bed, c, PSTR("`C"));
    }
    lcd_lib_draw_string_center(25, buffer);
  }else{
    if (SDUPSIsWorking() || SDUPSIsStarting()) {
      currentMenu = lcd_menu_main;
    }
    else{
      currentMenu = lcd_menu_print_ready_cooled_down;
    }
    fanSpeed=0;
  }
}

static void lcd_menu_print_ready_cooled_down()
{
  if (led_mode == LED_MODE_WHILE_PRINTING)
    analogWrite(LED_PIN, 0);
  else if (led_mode == LED_MODE_BLINK_ON_DONE)
    analogWrite(LED_PIN, (led_glow << 1) * int(led_brightness_level) / 100);
  
  LED_GLOW();
  
  lcd_info_screen(lcd_menu_main, postPrintReady, LS(PSTR("BACK TO MENU"),
                                                    PSTR("\xBE" "\x81"  "\x89" "\x81"  "\xD0" "\x80"  "\xD1" "\x80"  ),
                                                    PSTR("\xFB" "\x82"  "\xF1" "\x82"  "\xB4" "\x83"  " ""\x86" "\x84"  "\xFE" "\x83"  "\x90" "\x83"  "\x83" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Print finished"),
                                                  PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\x8F" "\x82"  "\x90" "\x82"  "\xDA" "\x81"  "\xBB" "\x80"  "!"),
                                                  PSTR("\xA9" "\x83"  "\xA7" "\x84"  " ""\xDD" "\x83"  "\x97" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 24, 24) , LS(PSTR("You can remove"),
                                                  PSTR("\xA5" "\x81"  "\xF3" "\x81"  "\x91" "\x82"  "\xD8" "\x80"  "\xE3" "\x80"  "\xC2" "\x80"  "\xC3" "\x80"  "\x98" "\x81"  ),
                                                  PSTR("\xED" "\x82"  "\xEE" "\x82"  " ""\xE4" "\x83"  "\xC8" "\x83"  "\xF3" "\x82"  "\x87" "\x84"  "\x9A" "\x83"  "\x88" "\x84"  )) );
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("the model."),
                                                  PSTR(""),
                                                  PSTR("")) );
}


/****************************************************************************************
 * Select Manual resume or not
 *
 ****************************************************************************************/


static bool handleResumeError(uint8_t theErrorCode)
{
  if ((resumeError=theErrorCode)!=RESUME_ERROR_NONE) {
    lcd_change_to_menu(lcd_menu_print_resume_error);
    return true;
  }
  else{
    return false;
  }
}

static uint8_t doRetriveClass()
{
  if (!SDUPSRetrieveClass()) {
    return RESUME_ERROR_SD_VALIDATE;
  }
  SDUPSScanPosition();
  return RESUME_ERROR_NONE;
}

static uint8_t doStableReadSD(uint32_t theFilePosition, char *theBuffer, uint8_t theSize)
{
  uint8_t cardErrorTimes=0;
  
  do {
    card.clearError();
    cardErrorTimes++;
    card.setIndex(theFilePosition);
    card.fgets(theBuffer, theSize);
  } while (card.errorCode() && cardErrorTimes<5);
  
  if (card.errorCode()) {
    return RESUME_ERROR_SD_READ_CARD;
  }
  
  theBuffer = strchr(theBuffer, ';');
  *theBuffer = 0;
  
  return RESUME_ERROR_NONE;
}

static uint8_t doSearchZLayer(uint32_t theFilePosition, char *theBuffer, uint8_t theSize)
{
  menuTimer=millis();
  
  char* searchPtr;
  
  SDUPSState=SDUPSStateFindNone;
  card.setIndex(theFilePosition);
  
  SDUPSFeedrate = 0.0;
  
  do {
    
    if (millis()-menuTimer>=2000) {
      previous_millis_cmd = millis();
      return RESUME_ERROR_SD_SEARCH_TOO_LONG;
    }
    
    card.fgets(theBuffer, theSize);
    
    searchPtr = strchr(theBuffer, ';');
    *searchPtr = 0;
    
    if (card.errorCode())
    {
//      SERIAL_ECHOLNPGM("sd error");
      if (!card.sdInserted)
      {
        return RESUME_ERROR_SD_READ_CARD;
      }
      //On an error, reset the error, reset the file position and try again.
#ifdef ClearError
      card.clearError();
#endif
      card.setIndex(card.getFilePos());
      continue;
    }
    
    if (theBuffer[0]=='G') {
      
      searchPtr=strchr(theBuffer, 'X');
      if (searchPtr!=NULL && (SDUPSState & SDUPSStateFindX) == 0) {
        SDUPSCurrentPosition[X_AXIS]=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindX;
//        SERIAL_DEBUGPGM("x ");
      }
      
      searchPtr=strchr(theBuffer, 'Y');
      if (searchPtr!=NULL && (SDUPSState & SDUPSStateFindY) == 0) {
        SDUPSCurrentPosition[Y_AXIS]=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindY;
//        SERIAL_DEBUGPGM("y ");
      }
      
      searchPtr=strchr(theBuffer, 'Z');
      if (searchPtr!=NULL && (SDUPSState & SDUPSStateFindZ) == 0) {
        SDUPSCurrentPosition[Z_AXIS]=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindZ;
//        SERIAL_DEBUGPGM("z ");
      }
      
      searchPtr=strchr(theBuffer, 'E');
      if (searchPtr!=NULL && (SDUPSState & SDUPSStateFindE) == 0) {
        SDUPSCurrentPosition[E_AXIS]=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindE;
//        SERIAL_DEBUGPGM("e ");
      }
      
      searchPtr=strchr(theBuffer, 'F');
      if (searchPtr!=NULL && (SDUPSState & SDUPSStateFindF) == 0) {
        SDUPSFeedrate=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindF;
//        SERIAL_DEBUGPGM("f ");
      }
      
      if ((SDUPSState&(SDUPSStateFindX|SDUPSStateFindY|SDUPSStateFindZ|SDUPSStateFindE))==(SDUPSStateFindX|SDUPSStateFindY|SDUPSStateFindZ|SDUPSStateFindE)) {
        break;
      }
    }
  } while (true);
//  SERIAL_DEBUGLN("");
  return RESUME_ERROR_NONE;
}


static uint8_t doSearchPause(uint32_t theFilePosition, char *theBuffer, uint8_t theSize)
{
  menuTimer=millis();
  
  char* searchPtr;
  
  SDUPSState=SDUPSStateFindNone;
  card.setIndex(theFilePosition);
  
  SDUPSFeedrate = 0.0;
  
  do {
    
    if (millis()-menuTimer>=2000) {
      previous_millis_cmd = millis();
      return RESUME_ERROR_SD_SEARCH_TOO_LONG;
    }
    
    card.fgets(theBuffer, theSize);
    
    searchPtr = strchr(theBuffer, ';');
    *searchPtr = 0;
    
    if (card.errorCode())
    {
      SERIAL_ERRORLNPGM("sd error");
      if (!card.sdInserted)
      {
        return RESUME_ERROR_SD_READ_CARD;
      }
      //On an error, reset the error, reset the file position and try again.
#ifdef ClearError
      card.clearError();
#endif
      card.setIndex(card.getFilePos());
      continue;
    }
    
    if (theBuffer[0]=='G') {
      
      searchPtr=strchr(theBuffer, 'X');
      if ((searchPtr!=NULL)&&((SDUPSState&SDUPSStateFindX)==0)) {
        SDUPSCurrentPosition[X_AXIS]=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindX;
//        SERIAL_DEBUGPGM("x ");
      }
      
      searchPtr=strchr(theBuffer, 'Y');
      if (searchPtr!=NULL&&((SDUPSState&SDUPSStateFindY)==0)) {
        SDUPSCurrentPosition[Y_AXIS]=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindY;
//        SERIAL_DEBUGPGM("y ");
      }
      
      searchPtr=strchr(theBuffer, 'E');
      if (searchPtr!=NULL&&((SDUPSState&SDUPSStateFindE)==0)) {
        SDUPSCurrentPosition[E_AXIS]=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindE;
//        SERIAL_DEBUGPGM("e ");
      }
      
      searchPtr=strchr(theBuffer, 'F');
      if (searchPtr!=NULL&&((SDUPSState&SDUPSStateFindF)==0)) {
        SDUPSFeedrate=strtod(searchPtr+1, NULL);
        SDUPSState|=SDUPSStateFindF;
//        SERIAL_DEBUGPGM("f ");
      }
      
      if ((SDUPSState&(SDUPSStateFindX|SDUPSStateFindY|SDUPSStateFindE))==(SDUPSStateFindX|SDUPSStateFindY|SDUPSStateFindE)) {
        break;
      }
    }
  } while (true);
//  SERIAL_DEBUGLN("");
  return RESUME_ERROR_NONE;
}

static void doManualPosition(char *theBuffer)
{
  char *bufferPtr;
  enquecommand_P(PSTR("G28\nG11\nG1 F6000 Z200"));
  
  bufferPtr=theBuffer;
  
  strcpy_P(bufferPtr, PSTR("G1 Z"));
  bufferPtr+=strlen_P(PSTR("G1 Z"));
  
  bufferPtr=float_to_string(min(SDUPSCurrentPosition[Z_AXIS]+20, 260), bufferPtr);
  
  strcpy_P(bufferPtr, PSTR("\nG1 X"));
  bufferPtr+=strlen_P(PSTR("\nG1 X"));
  
  bufferPtr=float_to_string(SDUPSCurrentPosition[X_AXIS], bufferPtr);
  
  strcpy_P(bufferPtr, PSTR(" Y"));
  bufferPtr+=strlen_P(PSTR(" Y"));
  
  bufferPtr=float_to_string(SDUPSCurrentPosition[Y_AXIS], bufferPtr);
  
  strcpy_P(bufferPtr, PSTR(" Z"));
  bufferPtr+=strlen_P(PSTR(" Z"));
  
  bufferPtr=float_to_string(SDUPSCurrentPosition[Z_AXIS]+1.0, bufferPtr);
  
  enquecommand(theBuffer);
}

static void doResumeHeatUp(uint32_t& theSDUPSFilePosition)
{
  char buffer[64];
  char *bufferPtr;
  
  card.setIndex(theSDUPSFilePosition);
  
  target_temperature_bed = 0;
  fanSpeedPercent = 0;
  
  fanSpeed = 0;
  
  bufferPtr=buffer;
  
  strcpy_P(bufferPtr, PSTR("G28\nG1 F3000 Z"));
  bufferPtr+=strlen_P(PSTR("G28\nG1 F3000 Z"));
  
  bufferPtr=float_to_string(min(SDUPSCurrentPosition[Z_AXIS]+PRIMING_HEIGHT+50, 260), bufferPtr);
  
  strcpy_P(bufferPtr, PSTR("\nG1 X0 Y"Y_MIN_POS_STR " Z"));
  bufferPtr+=strlen_P(PSTR("\nG1 X0 Y"Y_MIN_POS_STR " Z"));
  
  bufferPtr=float_to_string(min(SDUPSCurrentPosition[Z_AXIS]+PRIMING_HEIGHT+30, 260), bufferPtr);
  
  
  if (eeprom_read_word((uint16_t*)EEPROM_SDUPS_HEATING_OFFSET) == 0) {
    for(uint8_t e=0; e<EXTRUDERS; e++)
    {
      if (LCD_DETAIL_CACHE_MATERIAL(e) < 1)
        continue;
      target_temperature[e] = 0;//material[e].temperature;
      if (Device_isBedHeat) {
        target_temperature_bed = max(target_temperature_bed, material[e].bed_temperature);
      }
      else{
        target_temperature_bed = 0;
      }
      fanSpeedPercent = max(fanSpeedPercent, material[0].fan_speed);
      volume_to_filament_length[e] = 1.0 / (M_PI * (material[e].diameter / 2.0) * (material[e].diameter / 2.0));
      extrudemultiply[e] = material[e].flow;
    }
    currentMenu = lcd_menu_print_heatup;
    nextEncoderPos = 0;
    enquecommand(buffer);
    
  }
  else{
    fanSpeedPercent = 100;
    
    if (eeprom_read_word((uint16_t*)EEPROM_SDUPS_HEATING_BED_OFFSET)) {
      strcpy_P(bufferPtr, PSTR("\nM190 S"));
      bufferPtr+=strlen_P(PSTR("\nM190 S"));
      
      bufferPtr = int_to_string(eeprom_read_word((uint16_t*)EEPROM_SDUPS_HEATING_BED_OFFSET), bufferPtr);
    }
    
    if (eeprom_read_word((uint16_t*)EEPROM_SDUPS_HEATING_OFFSET)) {
      strcpy_P(bufferPtr, PSTR("\nM109 S"));
      bufferPtr+=strlen_P(PSTR("\nM109 S"));
      
      bufferPtr = int_to_string(eeprom_read_word((uint16_t*)EEPROM_SDUPS_HEATING_OFFSET), bufferPtr);
    }
    enquecommand(buffer);
    
    doStartPrint();
    currentMenu = lcd_menu_print_printing;
    nextEncoderPos = 0;
  }
}

static void doResumeInit()
{
  char buffer[64];
  
  if (handleResumeError(doRetriveClass())) return;
  
  for (uint8_t index=1; index<SDUPSPositionSize; index++) {
    SDUPSFilePosition=SDUPSRetrievePosition(index);
    if (SDUPSFilePosition==0xffffffffUL) {
      if (handleResumeError(RESUME_ERROR_SD_READ_CARD)) return;
    }
    if (handleResumeError(doStableReadSD(SDUPSFilePosition, buffer, sizeof(buffer)))) return;
    if (strchr(buffer, 'Z') != NULL) {
      if (index==1) {
        resumeState|=RESUME_STATE_RESUME;
//        SERIAL_DEBUGLNPGM("RESUME_STATE_RESUME");
      }
      else{
        resumeState|=RESUME_STATE_PAUSE;
//        SERIAL_DEBUGLNPGM("RESUME_STATE_PAUSE");
      }
//      SERIAL_DEBUGLNPGM("RetriveIndex:");
//      SERIAL_DEBUGLN((int)index);
//      SERIAL_DEBUGLN(buffer);
      break;
    }
    if (index==SDUPSPositionSize-1) {
      if (handleResumeError(RESUME_ERROR_SD_READ_CARD)) return;
    }
  }
  
  if (handleResumeError(doSearchZLayer(SDUPSFilePosition, buffer, sizeof(buffer)))) return;
  
  if ((resumeState&RESUME_STATE_MANUAL)==RESUME_STATE_MANUAL) {
    doManualPosition(buffer);
  }
  else{
    if ((resumeState&RESUME_STATE_PAUSE)==RESUME_STATE_PAUSE) {
      SDUPSFilePosition=SDUPSRetrievePosition(1);
      if (SDUPSFilePosition==0xffffffffUL) {
        if (handleResumeError(RESUME_ERROR_SD_READ_CARD)) return;
      }
      doSearchPause(SDUPSFilePosition, buffer, sizeof(buffer));
    }
    doResumeHeatUp(SDUPSFilePosition);
  }
}

static void doResumeManualInit()
{
  resumeState|=RESUME_STATE_MANUAL;
  doResumeInit();
}

static void lcd_menu_print_resume_manual_option()
{
  LED_NORMAL();
  
  lcd_question_screen(NULL, doResumeInit, LS(PSTR("CONTINUE"),
                                             PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                             PSTR("\x98" "\x84"  "\xD3" "\x83"  )) ,
                      lcd_menu_print_resume_manual_height, doResumeManualInit, LS(PSTR("MANUAL"),
                                                                                  PSTR("\xAB" "\x81"  "\xB1" "\x80"  ),
                                                                                  PSTR("\xB7" "\x83"  "\xBC" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  )) ,MenuForward,MenuForward);
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("Continue to resume"),
                                                  PSTR("\x9E" "\x81"  "\x9F" "\x81"  "\"""\xC6" "\x80"  "\xC7" "\x80"  "\"""\xA7" "\x81"  "\xB1" "\x80"  "\xD1" "\x81"  "\xC8" "\x81"  ),
                                                  PSTR("\"""\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  "\"""\xB1" "\x83"  "\xDA" "\x82"  "\xB3" "\x83"  "\xB4" "\x83"   )));
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("or manually set the"),
                                                  PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xD5" "\x80"  "\xD6" "\x80"  "\xAB" "\x81"  "\xB1" "\x80"  "\xE0" "\x80"  "\xE1" "\x80"  "\xA0" "\x80"  "\xEC" "\x80"  ),
                                                  PSTR("\xBB" "\x83"  "\xBC" "\x83"  " ""\xA9" "\x83"  "\xA7" "\x84"  " ""\xFD" "\x82"  "\xFE" "\x82"   )));
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("height."),
                                                  PSTR("\xC4" "\x81"  "\x89" "\x80"  "\xD1" "\x81"  "\xC8" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  ),
                                                  PSTR("\xB7" "\x83"  "\xBC" "\x83"  "\xB3" "\x83"  "\xB4" "\x83"  " ""\x84" "\x83"  "\x85" "\x83"  )) );
}


/****************************************************************************************
 * resume manually set height
 *
 ****************************************************************************************/
static void doResumeManualStoreZ()
{
  SDUPSLastZ=current_position[Z_AXIS];
  SDUPSLastZForCheck = 0.0;
  
  enquecommand_P(PSTR("G28"));
  
  //The latest one may not be confirmed to be valid Z position for height. So use the 2 instead.
  SDUPSPositionIndex=2;
  SDUPSRemovePosition(1);
}

static void lcd_menu_print_resume_manual_height()
{
  char buffer[32];
  char *bufferPtr;
  
  LED_NORMAL();
  
  if (printing_state == PRINT_STATE_NORMAL && lcd_lib_encoder_pos != 0 && movesplanned() < 4 && !is_command_queued() && !isCommandInBuffer() )
  {
    current_position[Z_AXIS] -= float(lcd_lib_encoder_pos) * 0.1;
    
    if (current_position[Z_AXIS]<0.0) {
      current_position[Z_AXIS]=0.0;
    }
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 1500/60, 0);
  }
  lcd_lib_encoder_pos = 0;
  
  if (blocks_queued())
    lcd_info_screen(NULL, NULL, LS(PSTR("CONTINUE"),
                                   PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                   PSTR("\x98" "\x84"  "\xD3" "\x83"  )) );
  else
    lcd_info_screen(lcd_menu_print_resume_manual_search_sd_card_eeprom, doResumeManualStoreZ, LS(PSTR("CONTINUE"),
                                                                                                 PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                                                 PSTR("\x98" "\x84"  "\xD3" "\x83"  )) , MenuForward);
  
  bufferPtr=buffer;
  strcpy_P(bufferPtr, LS(PSTR("Last Printed Z:"),
                         PSTR("\xD2" "\x80"  "\xC9" "\x80"  "\xFE" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xC4" "\x81"  "\x89" "\x80"  ":"),
                         PSTR("\xCE" "\x82"  "\xBE" "\x82"  " ""\xB6" "\x83"  "\x96" "\x84"  " ""\x99" "\x84"  "\xC9" "\x83"  ":")) );
  bufferPtr+=LS(strlen_P(PSTR("Last Printed Z:")),
                strlen_P(PSTR("\xD2" "\x80"  "\xC9" "\x80"  "\xFE" "\x81"  "\xAA" "\x80"  "\xAB" "\x80"  "\xC4" "\x81"  "\x89" "\x80"  ":")),
                strlen_P(PSTR("\xCE" "\x82"  "\xBE" "\x82"  " ""\xB6" "\x83"  "\x96" "\x84"  " ""\x99" "\x84"  "\xC9" "\x83"  ":"))) ;
  bufferPtr=float_to_string(SDUPSCurrentPosition[Z_AXIS], bufferPtr);
  lcd_lib_draw_string_center(LS(10, 11, 11) , buffer);
  
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Press Down or UP"),
                                                  PSTR("\xE2" "\x80"  "\xD2" "\x80"  "\xE3" "\x80"  "\xE4" "\x80"  "\xA4" "\x81"  "\xB1" "\x81"  "\xC4" "\x81"  "\x89" "\x80"  ),
                                                  PSTR("Press Down or UP")) );
  
  bufferPtr=buffer;
  strcpy_P(bufferPtr, LS(PSTR("Current Z:"),
                         PSTR("\x94" "\x82"  "\xAF" "\x80"  "\xC4" "\x81"  "\x89" "\x80"  ":"),
                         PSTR("\x8D" "\x84"  "\x96" "\x83"  " Z""\x8B" "\x84"  "\x8C" "\x84"  )) );
  bufferPtr+=LS(strlen_P(PSTR("Current Z:")),
                strlen_P(PSTR("\x94" "\x82"  "\xAF" "\x80"  "\xC4" "\x81"  "\x89" "\x80"  ":")),
                strlen_P(PSTR("\x8D" "\x84"  "\x96" "\x83"  " Z""\x8B" "\x84"  "\x8C" "\x84"  ))) ;
  
  bufferPtr=float_to_string(current_position[Z_AXIS], bufferPtr);
  
  lcd_lib_draw_string_center(LS(30, 37, 37) , buffer);
}

static void lcd_menu_print_resume_manual_search_sd_card_eeprom()
{
  char buffer[64];
  
  lcd_info_screen(lcd_menu_print_abort,NULL,LS(PSTR("ABORT"),
                                               PSTR("\xC1" "\x80"  "\x83" "\x80"  ),
                                               PSTR("\xE5" "\x83"  "\xD7" "\x82"  "\xDB" "\x82"  )) ,MenuForward);
  
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Searching SD card..."),
                                                  PSTR("\x9D" "\x81"  "\xD8" "\x80"  "SD" "\x9E" "\x80"  "..." ),
                                                  PSTR("SD ""\xEB" "\x82"  "\xEC" "\x82"  " ""\xB6" "\x83"  "\xEE" "\x83"  "\xC0" "\x82"  )) );
  lcd_progressbar(0);
  
  SDUPSFilePosition=SDUPSRetrievePosition(SDUPSPositionIndex);
  if (SDUPSFilePosition==0xffffffffUL) {
    SDUPSFilePosition=0;
    card.setIndex(SDUPSFilePosition);
    SDUPSCurrentPosition[Z_AXIS]=0;
    lcd_change_to_menu(lcd_menu_print_resume_manual_search_sd_card);
    return;
  }
  
  if (handleResumeError(doStableReadSD(SDUPSFilePosition, buffer, sizeof(buffer)))) return;
  
  
  if (strchr(buffer, 'Z') != NULL) {
//    SERIAL_DEBUGLNPGM("Z Layer:");
    SERIAL_DEBUGLN(buffer);
    if(handleResumeError(doSearchZLayer(SDUPSFilePosition, buffer, sizeof(buffer)))) return;
    if (SDUPSCurrentPosition[Z_AXIS]<=SDUPSLastZ) {
      if (SDUPSPositionIndex==2) {
        card.setIndex(SDUPSFilePosition);
        lcd_change_to_menu(lcd_menu_print_resume_manual_search_sd_card);
//        SERIAL_DEBUGLNPGM("manual_search_sd_card");
        return;
      }
      else{
        doResumeHeatUp(SDUPSFilePosition);
        return;
      }
    }
  }
  
  SDUPSRemovePosition(SDUPSPositionIndex);
  
  if (SDUPSPositionIndex==SDUPSPositionSize) {
    SDUPSFilePosition=0;
    card.setIndex(SDUPSFilePosition);
    SDUPSCurrentPosition[Z_AXIS]=0;
    lcd_change_to_menu(lcd_menu_print_resume_manual_search_sd_card);
//    SERIAL_DEBUGLNPGM("manual_search_sd_card");
  }
  
  SDUPSPositionIndex++;
}

/****************************************************************************************
 * resume manually Search SD card
 *
 ****************************************************************************************/
static void lcd_menu_print_resume_manual_search_sd_card()
{
  LED_NORMAL();
  
  lcd_info_screen(lcd_menu_print_abort,NULL,LS(PSTR("ABORT"),
                                               PSTR("\xC1" "\x80"  "\x83" "\x80"  ),
                                               PSTR("\xE5" "\x83"  "\xD7" "\x82"  "\xDB" "\x82"  )) ,MenuForward);
  
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("Searching SD card..."),
                                                  PSTR("\x9D" "\x81"  "\xD8" "\x80"  "SD" "\x9E" "\x80"  "..."),
                                                  PSTR("SD ""\xEB" "\x82"  "\xEC" "\x82"  " ""\xB6" "\x83"  "\xEE" "\x83"  "\xC0" "\x82"  )) );
  
  lcd_progressbar((uint8_t)(125*SDUPSCurrentPosition[Z_AXIS]/SDUPSLastZ));
  
  if (printing_state!=PRINT_STATE_NORMAL || is_command_queued() || isCommandInBuffer()) {
    return;
  }
  
  
  char buffer[64];
  char *bufferPtr=NULL;
  
  menuTimer=millis();
  
  do {
    if (millis()-menuTimer>=2000) {
      previous_millis_cmd = millis();
      return;
    }
    
    card.fgets(buffer, sizeof(buffer));
    
    bufferPtr = strchr(buffer, ';');
    if (bufferPtr != NULL) {
      *bufferPtr = 0;
    }
    
    if (card.errorCode())
    {
      SERIAL_DEBUGLNPGM("sd error");
      if (!card.sdInserted)
      {
        return;
      }
      
#ifdef ClearError
      card.clearError();
#endif
      card.setIndex(card.getFilePos());
      
      return;
    }
    
    if (strchr(buffer, 'Z') != NULL) {
      SERIAL_DEBUGLNPGM("Z Layer:");
      SERIAL_DEBUGLN(buffer);
      if (handleResumeError(doSearchZLayer(card.getFilePos(), buffer, sizeof(buffer)))) return;
      if (SDUPSLastZForCheck >= SDUPSLastZ && SDUPSCurrentPosition[Z_AXIS] > SDUPSLastZForCheck) {
        SDUPSStorePosition(SDUPSFilePosition);
        doResumeHeatUp(SDUPSFilePosition);
        break;
      }
      SDUPSFilePosition=card.getFilePos();
      SDUPSLastZForCheck = SDUPSCurrentPosition[Z_AXIS];
    }
    
    if (card.eof()) {
      if (handleResumeError(RESUME_ERROR_M25)) return;
    }
    
  } while (true);
  
  previous_millis_cmd = millis();
}

/****************************************************************************************
 * resume error
 *
 ****************************************************************************************/
static void doClearResumeError()
{
  resumeError=RESUME_ERROR_NONE;
  if (card.isFileOpen()) {
    card.closefile();
  }
  card.release();
}

static void lcd_menu_print_resume_error()
{
  LED_GLOW();
  char buffer[32];
  
  lcd_info_screen(lcd_menu_main,doClearResumeError,LS(PSTR("CANCEL"),
                                                      PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                                      PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) ,MenuForward);
  
  
  if (resumeError) {
    switch (resumeError) {
      case RESUME_ERROR_M25:
        lcd_lib_draw_string_centerP(20, PSTR("M25 found"));
        break;
      case RESUME_ERROR_SD_VALIDATE:
        lcd_lib_draw_string_centerP(20, PSTR("SD card file changed"));
        break;
      case RESUME_ERROR_SD_READ_CARD:
        lcd_lib_draw_string_centerP(20, PSTR("SD read error"));
        break;
      case RESUME_ERROR_SD_SEARCH_TOO_LONG:
        lcd_lib_draw_string_centerP(20, PSTR("SD search too long"));
        break;
      default:
        break;
    }
  }
}


/****************************************************************************************
 * Check whether the print is finished
 *
 ****************************************************************************************/
static void checkPrintFinished()
{
  if ((!card.sdprinting || (card.pause && ((card.getFilePos() + 512) > card.getFileSize()))) && !is_command_queued() && !isCommandInBuffer()){
    SDUPSDone();
    abortPrint();
    fanSpeed = lround(255 * int(fanSpeedPercent) / 100.0);
    currentMenu = lcd_menu_print_ready;
    SELECT_MAIN_MENU_ITEM(0);
  }
  if (card.errorCode())
  {
    abortPrint();
    currentMenu = lcd_menu_print_error;
    SELECT_MAIN_MENU_ITEM(0);
  }
  
  if (Device_isGate) {
    if (gateOpened()) {
      quickStop();
      doPausePrint();
      abortPrint();
      lcd_change_to_menu(lcd_menu_print_gate);
      return;
    }
  }
  
  
#ifdef FilamentDetection
  
  if (isFilamentDetectionEnable) {
    if (READ(FilamentDetectionPin)) {
      if (millis()-menuTimer>500) {
        abortPrint();
        currentMenu = lcd_menu_print_out_of_filament;
        SELECT_MAIN_MENU_ITEM(0);
      }
    }
    else{
      menuTimer=millis();
    }
  }
  
#endif
}

/****************************************************************************************
 * Check the filament
 *
 ****************************************************************************************/
#ifdef FilamentDetection
static void doOutOfFilament()
{
  resumeState=RESUME_STATE_FILAMENT;
  menuTimer = millis();
}

static void lcd_menu_print_out_of_filament()
{
  LED_GLOW();
  
  lcd_question_screen(lcd_menu_change_material_preheat, doOutOfFilament, LS(PSTR("CONTINUE"),
                                                                            PSTR("\xC6" "\x80"  "\xC7" "\x80"  ),
                                                                            PSTR("\x98" "\x84"  "\xD3" "\x83"  )) ,
                      lcd_menu_main, NULL, LS(PSTR("CANCEL"),
                                              PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                              PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) );
  lcd_lib_draw_string_centerP(LS(10, 11, 11) , LS(PSTR("OverLord is out"),
                                                  PSTR("\xAA" "\x80"  "\xAB" "\x80"  "\xF5" "\x80"  "\xF6" "\x80"  "\x8F" "\x82"  "\xA7" "\x80"  "\x96" "\x82"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xDC" "\x82"  "\xDD" "\x82"  " ""\xDD" "\x83"  "\x97" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(20, 24, 24) , LS(PSTR("of material. Press"),
                                                  PSTR("\x9E" "\x81"  "\x9F" "\x81"  "\xC6" "\x80"  "\xC7" "\x80"  ),
                                                  PSTR("\"""\xEF" "\x82"  "\xF0" "\x82"  "\xF1" "\x82"  "\" ""\xB1" "\x83"  "\xDA" "\x82"  "\xB3" "\x83"  "\xB4" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(30, 37, 37) , LS(PSTR("CONTINUE to change"),
                                                  PSTR("\x9A" "\x80"  "\xDF" "\x80"  "\xA3" "\x81"  "\xA2" "\x81"  "\xF5" "\x80"  "\xF6" "\x80"  ),
                                                  PSTR("\x96" "\x83"  "\x97" "\x83"  " ""\xA3" "\x84"  "\xA9" "\x84"  "\x83" "\x83"  )) );
  lcd_lib_draw_string_centerP(LS(40, 64, 64) , LS(PSTR("the material."),
                                                  PSTR(""),
                                                  PSTR("")) );
}
#endif
      
      
#endif//ENABLE_ULTILCD2

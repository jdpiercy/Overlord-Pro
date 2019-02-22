#ifndef ULTI_LCD2_HI_LIB_H
#define ULTI_LCD2_HI_LIB_H

#include "UltiLCD2_low_lib.h"
#include "UltiLCD2_gfx.h"

typedef void (*menuFunc_t)();
typedef char* (*entryNameCallback_t)(uint8_t nr);
typedef void (*entryDetailsCallback_t)(uint8_t nr);

#define ENCODER_TICKS_PER_MAIN_MENU_ITEM 1
#define ENCODER_TICKS_PER_SCROLL_MENU_ITEM 1
#define MAIN_MENU_ITEM_POS(n)  (ENCODER_TICKS_PER_MAIN_MENU_ITEM * (n) + ENCODER_TICKS_PER_MAIN_MENU_ITEM / 2)
#define SCROLL_MENU_ITEM_POS(n)  (ENCODER_TICKS_PER_SCROLL_MENU_ITEM * (n) + ENCODER_TICKS_PER_SCROLL_MENU_ITEM / 2)
#define SELECT_MAIN_MENU_ITEM(n)  do { lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(n); } while(0)
#define SELECT_SCROLL_MENU_ITEM(n)  do { lcd_lib_encoder_pos = SCROLL_MENU_ITEM_POS(n); } while(0)
#define SELECTED_MAIN_MENU_ITEM() (lcd_lib_encoder_pos / ENCODER_TICKS_PER_MAIN_MENU_ITEM)
#define SELECTED_SCROLL_MENU_ITEM() (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM)
#define IS_SELECTED_MAIN(n) ((n) == SELECTED_MAIN_MENU_ITEM())
#define IS_SELECTED_SCROLL(n) ((n) == SELECTED_SCROLL_MENU_ITEM())

#define MenuBackward 0
#define MenuForward 1
#define MenuUp 2
#define MenuDown 3
void lcd_change_to_menu(menuFunc_t nextMenu, int16_t newEncoderPos = MAIN_MENU_ITEM_POS(0),  uint8_t direction = MenuForward);

void lcd_tripple_menu(const char* left, const char* right, const char* bottom);
void lcd_basic_screen();
void lcd_info_screen(menuFunc_t cancelMenu, menuFunc_t callbackOnCancel = NULL, const char* cancelButtonText = NULL, uint8_t direction = MenuBackward);
void lcd_question_screen(menuFunc_t optionAMenu, menuFunc_t callbackOnA, const char* AButtonText, menuFunc_t optionBMenu, menuFunc_t callbackOnB, const char* BButtonText, uint8_t directionA=MenuForward, uint8_t directionB=MenuBackward);
void lcd_scroll_menu(const char* menuNameP, int8_t entryCount, entryNameCallback_t entryNameCallback, entryDetailsCallback_t entryDetailsCallback);

void lcd_draw_detail(char* pstr);
void lcd_draw_detailP(const char* pstr);
void lcd_normal_menu(const char* menuNameP, int8_t entryCount, entryNameCallback_t entryNameCallback, entryDetailsCallback_t entryDetailsCallback);
void lcd_advance_menu(const char* menuNameP, int8_t entryCount, entryNameCallback_t entryNameCallback, entryDetailsCallback_t entryDetailsCallback);

void lcd_progressbar(uint8_t progress);

void lcd_menu_edit_setting();

extern const char* lcd_setting_name;
extern const char* lcd_setting_postfix;
extern void* lcd_setting_ptr;
extern uint8_t lcd_setting_type;
extern int16_t lcd_setting_min;
extern int16_t lcd_setting_max;

extern menuFunc_t currentMenu;
extern menuFunc_t previousMenu;
extern menuFunc_t postMenuCheck;
extern int16_t previousEncoderPos;
extern int16_t nextEncoderPos;
extern uint8_t minProgress;
extern unsigned long menuTimer;

#define LCD_EDIT_SETTING(_setting, _name, _postfix, _min, _max) do { \
    lcd_change_to_menu(lcd_menu_edit_setting, _setting); \
    lcd_setting_name = (_name); \
    lcd_setting_postfix = (_postfix); \
    lcd_setting_ptr = &_setting; \
    lcd_setting_type = sizeof(_setting); \
    lcd_setting_min = _min; \
    lcd_setting_max = _max; \
    lcd_lib_button_up_down_reversed = true; \
} while(0)
#define LCD_EDIT_SETTING_STORE(_setting, _name, _postfix, _min, _max) do { \
    lcd_change_to_menu(lcd_menu_edit_setting, _setting); \
    lcd_setting_name = (_name); \
    lcd_setting_postfix = (_postfix); \
    lcd_setting_ptr = &_setting; \
    lcd_setting_type = 9; \
    lcd_setting_min = _min; \
    lcd_setting_max = _max; \
    lcd_lib_button_up_down_reversed = true; \
} while(0)
#define LCD_EDIT_SETTING_BYTE_PERCENT(_setting, _name, _postfix, _min, _max) do { \
    lcd_change_to_menu(lcd_menu_edit_setting,(lround((int(_setting) * 100) / 255.0))); \
    lcd_setting_name = (_name); \
    lcd_setting_postfix = (_postfix); \
    lcd_setting_ptr = &_setting; \
    lcd_setting_type = 5; \
    lcd_setting_min = _min; \
    lcd_setting_max = _max; \
    lcd_lib_button_up_down_reversed = true; \
} while(0)
#define LCD_EDIT_SETTING_FLOAT001(_setting, _name, _postfix, _min, _max) do { \
    lcd_change_to_menu(lcd_menu_edit_setting,((_setting) * 100.0 + 0.5)); \
    lcd_setting_name = (_name); \
    lcd_setting_postfix = (_postfix); \
    lcd_setting_ptr = &_setting; \
    lcd_setting_type = 3; \
    lcd_setting_min = (_min) * 100; \
    lcd_setting_max = (_max) * 100; \
    lcd_lib_button_up_down_reversed = true; \
} while(0)
#define LCD_EDIT_SETTING_FLOAT100(_setting, _name, _postfix, _min, _max) do { \
    lcd_change_to_menu(lcd_menu_edit_setting,((_setting) / 100 + 0.5)); \
    lcd_setting_name = (_name); \
    lcd_setting_postfix = ("00" _postfix); \
    lcd_setting_ptr = &(_setting); \
    lcd_setting_type = 7; \
    lcd_setting_min = (_min) / 100 + 0.5; \
    lcd_setting_max = (_max) / 100 + 0.5; \
    lcd_lib_button_up_down_reversed = true; \
} while(0)
#define LCD_EDIT_SETTING_FLOAT1(_setting, _name, _postfix, _min, _max) do { \
    lcd_change_to_menu(lcd_menu_edit_setting,((_setting) + 0.5)); \
    lcd_setting_name = (_name); \
    lcd_setting_postfix = (_postfix); \
    lcd_setting_ptr = &(_setting); \
    lcd_setting_type = 8; \
    lcd_setting_min = (_min) + 0.5; \
    lcd_setting_max = (_max) + 0.5; \
    lcd_lib_button_up_down_reversed = true; \
} while(0)
#define LCD_EDIT_SETTING_SPEED(_setting, _name, _postfix, _min, _max) do { \
    lcd_change_to_menu(lcd_menu_edit_setting,((_setting) / 60 + 0.5)); \
    lcd_setting_name = (_name); \
    lcd_setting_postfix = (_postfix); \
    lcd_setting_ptr = &(_setting); \
    lcd_setting_type = 6; \
    lcd_setting_min = (_min) / 60 + 0.5; \
    lcd_setting_max = (_max) / 60 + 0.5; \
    lcd_lib_button_up_down_reversed = true; \
} while(0)



extern uint8_t led_glow;
extern uint8_t led_glow_dir;
#define LED_NORMAL() lcd_lib_led_color(48,48,60)

#define LED_GLOW() lcd_lib_led_color(48*led_glow/128, 48*led_glow/128, 60*led_glow/128)
#define LED_PRINT() lcd_lib_led_color(48, 80, 48)
#define LED_GLOW_HEAT() lcd_lib_led_color(255*led_glow/128, 70*led_glow/128, 70*led_glow/128)
#define LED_GLOW_START() lcd_lib_led_color(5 + led_glow, 41 + led_glow, 41 + led_glow)
#define LED_GLOW_END() lcd_lib_led_color(250*led_glow/128, 130*led_glow/128, 15*led_glow/128)
//#define LED_GLOW_ERROR() lcd_lib_led_color(51 + led_glow, 0 + led_glow, 0 + led_glow)
//#define LED_GLOW_SLEEP() lcd_lib_led_color(5 + led_glow/4, 5 + led_glow/4, 5 + led_glow/4)
#define LED_GLOW_POWERERROR() lcd_lib_led_color(35*led_glow/128, 16*led_glow/128, 2*led_glow/128)
#define LED_ERROR() lcd_lib_led_color(255, 0, 0)

#define LED_OFF() lcd_lib_led_color(0, 0, 0)

//#define LED_GLOW_ERROR() lcd_lib_led_color(led_glow,128-led_glow,led_glow);

#endif//ULTI_LCD2_HI_LIB_H

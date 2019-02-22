#ifndef ULTI_LCD2_LOW_LIB_H
#define ULTI_LCD2_LOW_LIB_H

#include <stdint.h>
#include <stddef.h>

void lcd_lib_init();
void lcd_lib_update_screen();   /* Start sending out the display buffer to the screen. Wait till lcd_lib_update_ready before issuing any draw functions */
bool lcd_lib_update_ready();

void lcd_lib_draw_string(int16_t x, int16_t y, const char* str);
void lcd_lib_clear_string(int16_t x, int16_t y, const char* str);
void lcd_lib_draw_string_center(int16_t y, const char* str);
void lcd_lib_clear_string_center(int16_t y, const char* str);
void lcd_lib_draw_stringP(int16_t x, int16_t y, const char* pstr);
void lcd_lib_clear_stringP(int16_t x, int16_t y, const char* pstr);
void lcd_lib_draw_string_centerP(int16_t y, const char* pstr);
void lcd_lib_clear_string_centerP(int16_t y, const char* pstr);
void lcd_lib_draw_string_center_atP(int16_t x, int16_t y, const char* pstr);
void lcd_lib_clear_string_center_atP(int16_t x, int16_t y, const char* pstr);
void lcd_lib_draw_string_center_at(int16_t x, int16_t y, const char* pstr);

void lcd_lib_draw_vline(int16_t x, int16_t y0, int16_t y1);
void lcd_lib_draw_hline(int16_t x0, int16_t x1, int16_t y);
void lcd_lib_draw_box(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void lcd_lib_draw_shade(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void lcd_lib_clear();
void lcd_lib_clear(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void lcd_lib_invert(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void lcd_lib_set();
void lcd_lib_set(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void lcd_lib_draw_gfx(int16_t x, int16_t y, const uint8_t* gfx);
void lcd_lib_clear_gfx(int16_t x, int16_t y, const uint8_t* gfx);

void lcd_lib_move_horizontal(int16_t x);
void lcd_lib_move_vertical(int16_t y);
void lcd_lib_set_contrast(uint8_t theContrast);

void lcd_lib_copy_hline(int16_t yDst, int16_t ySrc);
void lcd_lib_move_point(uint8_t xDst, uint8_t yDst, uint8_t xSrc, uint8_t ySrc);
void lcd_lib_remove_point(uint8_t xSrc, uint8_t ySrc);

void lcd_lib_beep();
void lcd_lib_buttons_update();
void lcd_lib_buttons_update_interrupt();
void lcd_lib_led_color(uint8_t r, uint8_t g, uint8_t b);

extern int16_t lcd_lib_encoder_pos;
extern bool lcd_lib_button_pressed;
extern bool lcd_lib_button_down;

void lcd_lib_oled_on();
void lcd_lib_oled_off();
void lcd_lib_RGB_off();

#define CHINESE_POINT "\xB6" "\x84"
#define CHINESE_DOWN  "\xB7" "\x84"
#define CHINESE_UP    "\xB8" "\x84"

#define ENGLISH_POINT "\x1C"
#define ENGLISH_UP    "\x1E"
#define ENGLISH_DOWN  "\x1F"

#define LANGUAGE_NUMBER 2

#define LANGUAGE_X 0
#define LANGUAGE_Y 1

extern const char* languagePointer[LANGUAGE_NUMBER];
extern int16_t languageOffsetX[LANGUAGE_NUMBER];
extern int16_t languageOffsetY[LANGUAGE_NUMBER];

extern bool lcd_lib_button_up_down_reversed;
char* int_to_string(int i, char* temp_buffer, const char* p_postfix = NULL);
char* int_to_time_string(unsigned long i, char* temp_buffer);
char* float_to_string(float f, char* temp_buffer, const char* p_postfix = NULL);

void lcd_lib_i2c_stop();

#endif//ULTI_LCD2_LOW_LIB_H

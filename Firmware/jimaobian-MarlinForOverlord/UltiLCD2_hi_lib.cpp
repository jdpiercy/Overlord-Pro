#include <avr/pgmspace.h>
#include <string.h>

#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
  #include "UltiLCD2_hi_lib.h"
  #include "ConfigurationStore.h"

menuFunc_t currentMenu;
menuFunc_t previousMenu;
menuFunc_t postMenuCheck;
menuFunc_t currentBackUpMenu;

int16_t previousEncoderPos;
int16_t nextEncoderPos;
uint8_t led_glow = 0;
uint8_t led_glow_dir;
uint8_t minProgress;

const char* lcd_setting_name;
const char* lcd_setting_postfix;
void* lcd_setting_ptr;
uint8_t lcd_setting_type;
int16_t lcd_setting_min;
int16_t lcd_setting_max;
unsigned long menuTimer;

void lcd_draw_detail(char* pstr)
{
  static int detailStringIndex=0;
  static char* pstrBack=pstr;

  uint8_t detailStringLength=strlen(pstr);

  uint8_t yOffset = LS(56, 53, 53);

  if (detailStringLength<=20) {
    lcd_lib_draw_string_center(yOffset, pstr);
  }
  else{
    if (pstrBack!=pstr) {
      detailStringIndex=0;
      pstrBack=pstr;
    }
    lcd_lib_draw_string(-detailStringIndex, yOffset, pstr);
    detailStringIndex++;
    if (detailStringIndex>=6*detailStringLength) {
      detailStringIndex=-128;
    }
  }
}

void lcd_draw_detailP(const char* pstr)
{
  static int detailStringIndex=0;
  static const char* pstrBack=pstr;
  if (pstrBack!=pstr) {
    detailStringIndex=0;
    pstrBack=pstr;
  }

  uint8_t yOffset = LS(56, 53, 53);

  uint8_t detailStringLength=strlen_P(pstr);

  if (detailStringLength<=20) {
    lcd_lib_draw_string_centerP(yOffset, pstr);
  }
  else{
    lcd_lib_draw_stringP(-detailStringIndex, yOffset, pstr);
    detailStringIndex++;
    if (detailStringIndex>=6*detailStringLength) {
      detailStringIndex=-128;
    }

  }
}

static uint8_t swiftStep=0;
static int16_t swiftWiewPos = 0;
static int16_t swiftWiewPosBackUp = 0;

void lcd_swift_down()
{
  if (lcd_lib_button_pressed) {
    currentMenu=currentBackUpMenu;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep=0;
    return;
  }

  switch (swiftStep) {
  case 0:
    swiftWiewPos = 0;
    swiftWiewPosBackUp = 0;
    swiftStep++;
  case 1:
    swiftWiewPos += swiftWiewPos * 4 / 5;
    if (swiftWiewPos <= -63) { swiftStep++; }
    lcd_lib_move_vertical(swiftWiewPos-swiftWiewPosBackUp);
    swiftWiewPos -=1;
    swiftWiewPosBackUp=swiftWiewPos;

    break;
  case 2:
    swiftWiewPos = 63;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep++;
  case 3:
    lcd_lib_clear();
    currentBackUpMenu();
    swiftWiewPos -= swiftWiewPos * 4 / 5;
    if (swiftWiewPos <= 0) {
      currentMenu=currentBackUpMenu;
      swiftStep=0;
    }

    lcd_lib_move_vertical(swiftWiewPos);
    swiftWiewPos -=1;

    break;
  default:
    swiftStep=0;
    break;
  }

}


void lcd_swift_up()
{

  if (lcd_lib_button_pressed) {
    currentMenu=currentBackUpMenu;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep=0;
    return;
  }

  switch (swiftStep) {
  case 0:
    swiftWiewPos = 0;
    swiftWiewPosBackUp = 0;
    swiftStep++;
  case 1:
    swiftWiewPos += swiftWiewPos * 4 / 5;
    if (swiftWiewPos >= 63) {
      swiftStep++;
    }
    lcd_lib_move_vertical(swiftWiewPos-swiftWiewPosBackUp);
    swiftWiewPos +=1;
    swiftWiewPosBackUp=swiftWiewPos;

    break;
  case 2:
    swiftWiewPos = -63;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep++;
  case 3:
    lcd_lib_clear();
    currentBackUpMenu();
    swiftWiewPos -= swiftWiewPos * 4 / 5;
    if (swiftWiewPos >= 0) {
      currentMenu=currentBackUpMenu;
      swiftStep=0;
    }
    lcd_lib_move_vertical(swiftWiewPos);
    swiftWiewPos +=1;

    break;
  default:
    swiftStep=0;
    break;
  }

}


void lcd_swift_forward()
{

  if (lcd_lib_button_pressed) {
    currentMenu=currentBackUpMenu;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep=0;
    return;
  }

  switch (swiftStep) {
  case 0:
    swiftWiewPos = 0;
    swiftWiewPosBackUp = 0;
    swiftStep++;
  case 1:
    swiftWiewPos += swiftWiewPos * 4 / 5;
    if (swiftWiewPos <= -127) { swiftStep++; }
    lcd_lib_move_horizontal(swiftWiewPos-swiftWiewPosBackUp);
    swiftWiewPos -=1;
    swiftWiewPosBackUp=swiftWiewPos;

    break;
  case 2:
    swiftWiewPos = 127;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep++;
  case 3:
    lcd_lib_clear();
    currentBackUpMenu();
    swiftWiewPos -= swiftWiewPos * 4 / 5;
    if (swiftWiewPos <= 0) {
      currentMenu=currentBackUpMenu;
      swiftStep=0;
    }

    lcd_lib_move_horizontal(swiftWiewPos);
    swiftWiewPos -=1;

    break;
  default:
    swiftStep=0;
    break;
  }

}

void lcd_swift_backward()
{

  if (lcd_lib_button_pressed) {
    currentMenu=currentBackUpMenu;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep=0;
    return;
  }

  switch (swiftStep) {
  case 0:
    swiftWiewPos = 0;
    swiftWiewPosBackUp = 0;
    swiftStep++;
  case 1:
    swiftWiewPos += swiftWiewPos * 4 / 5;
    if (swiftWiewPos >= 127) {
      swiftStep++;
    }
    lcd_lib_move_horizontal(swiftWiewPos-swiftWiewPosBackUp);
    swiftWiewPos +=1;
    swiftWiewPosBackUp=swiftWiewPos;

    break;
  case 2:
    swiftWiewPos = -127;
    lcd_lib_encoder_pos = nextEncoderPos;
    swiftStep++;
  case 3:
    lcd_lib_clear();
    currentBackUpMenu();
    swiftWiewPos -= swiftWiewPos * 4 / 5;
    if (swiftWiewPos >= 0) {
      currentMenu=currentBackUpMenu;
      swiftStep=0;
    }
    lcd_lib_move_horizontal(swiftWiewPos);
    swiftWiewPos +=1;

    break;
  default:
    swiftStep=0;
    break;
  }

}

void lcd_change_to_menu(menuFunc_t nextMenu, int16_t newEncoderPos, uint8_t direction)
{
  minProgress = 5;
  led_glow = led_glow_dir = 0;
  lcd_lib_beep();
  if (currentMenu==lcd_swift_down || currentMenu==lcd_swift_up || currentMenu==lcd_swift_forward || currentMenu==lcd_swift_backward) {
    previousMenu = currentBackUpMenu;
    SERIAL_DEBUGLNPGM("inside swift");
  }
  else{
    previousMenu = currentMenu;
  }
  previousEncoderPos = lcd_lib_encoder_pos;

  switch (direction) {
  case MenuBackward:
    currentMenu = lcd_swift_backward;
    break;
  case MenuForward:
    currentMenu = lcd_swift_forward;
    break;
  case MenuUp:
    currentMenu = lcd_swift_up;
    break;
  case MenuDown:
    currentMenu = lcd_swift_down;
    break;

  default:
    break;
  }

  currentBackUpMenu = nextMenu;
  nextEncoderPos = newEncoderPos;

  swiftStep=0;
}

void lcd_basic_screen()
{
  lcd_lib_clear();
  lcd_lib_draw_hline(3, 124, LS(53, 50, 50));
}

void lcd_info_screen(menuFunc_t cancelMenu, menuFunc_t callbackOnCancel, const char* cancelButtonText, uint8_t direction)
{
  lcd_lib_encoder_pos = 0;

  if (lcd_lib_button_pressed && IS_SELECTED_MAIN(0))
  {
    if (cancelMenu) lcd_change_to_menu(cancelMenu,MAIN_MENU_ITEM_POS(0), direction);
    if (callbackOnCancel) callbackOnCancel();
  }

  lcd_basic_screen();

  if (!cancelButtonText) cancelButtonText = LS(PSTR("CANCEL"),
                                               PSTR("\xD8" "\x80"  "\xD9" "\x80"  ),
                                               PSTR("\xFF" "\x82"  "\xC6" "\x82"  )) ;
  
  

  switch (languageType) {
  case LANGUAGE_CHINESE:
  case LANGUAGE_KOREAN:
    if (IS_SELECTED_MAIN(0))
    {
      lcd_lib_draw_box(3+3, 54-3+1, 63+61-3, 64-1);
      lcd_lib_set(3+4, 54-3+2, 63+61-4, 64-2);
      lcd_lib_clear_stringP(65 - strlen_P(cancelButtonText) * 3, 56-3, cancelButtonText);
    }else{
      lcd_lib_draw_stringP(65 - strlen_P(cancelButtonText) * 3, 56-3, cancelButtonText);
    }
    break;
  case LANGUAGE_ENGLISH:
    if (IS_SELECTED_MAIN(0))
    {
      lcd_lib_draw_box(3+3, 54+1, 63+61-3, 64-1);
      lcd_lib_set(3+4, 54+2, 63+61-4, 64-2);
      lcd_lib_clear_stringP(65 - strlen_P(cancelButtonText) * 3, 56, cancelButtonText);
    }else{
      lcd_lib_draw_stringP(65 - strlen_P(cancelButtonText) * 3, 56, cancelButtonText);
    }
    break;
  default:
    break;
  }


}

void lcd_question_screen(menuFunc_t optionAMenu, menuFunc_t callbackOnA, const char* AButtonText, menuFunc_t optionBMenu, menuFunc_t callbackOnB, const char* BButtonText, uint8_t directionA, uint8_t directionB)
{
  if (lcd_lib_encoder_pos <= 0) {
    lcd_lib_encoder_pos=0;
  }
  else{
    lcd_lib_encoder_pos=1;
  }

  if (lcd_lib_button_pressed)
  {
    if (IS_SELECTED_MAIN(0))
    {
      if (optionAMenu) lcd_change_to_menu(optionAMenu,MAIN_MENU_ITEM_POS(0),directionA);
      if (callbackOnA) callbackOnA();
    }else if (IS_SELECTED_MAIN(1))
    {
      if (optionBMenu) lcd_change_to_menu(optionBMenu,MAIN_MENU_ITEM_POS(0),directionB);
      if (callbackOnB) callbackOnB();
    }
  }

  lcd_basic_screen();

  switch (languageType) {
  case LANGUAGE_CHINESE:
  case LANGUAGE_KOREAN:
    if (IS_SELECTED_MAIN(0))
    {
      lcd_lib_draw_box(3+3, 54-3+1, 63-3, 64-1);
      lcd_lib_set(3+4, 54-3+2, 63-4, 64-2);
      lcd_lib_clear_stringP(34 - strlen_P(AButtonText) * 3, 56-3, AButtonText);
    }else{
      lcd_lib_draw_stringP(34 - strlen_P(AButtonText) * 3, 56-3, AButtonText);
    }
    if (IS_SELECTED_MAIN(1))
    {
      lcd_lib_draw_box(3+61+3, 54-3+1, 63+61-3, 64-1);
      lcd_lib_set(3+61+4, 54-3+2, 63+61-4, 64-2);
      lcd_lib_clear_stringP(34+61 - strlen_P(BButtonText) * 3, 56-3, BButtonText);
    }else{
      lcd_lib_draw_stringP(34+61 - strlen_P(BButtonText) * 3, 56-3, BButtonText);
    }
    break;
  case LANGUAGE_ENGLISH:
    if (IS_SELECTED_MAIN(0))
    {
      lcd_lib_draw_box(3+3, 54+1, 63-3, 64-1);
      lcd_lib_set(3+4, 54+2, 63-4, 64-2);
      lcd_lib_clear_stringP(34 - strlen_P(AButtonText) * 3, 56, AButtonText);
    }else{
      lcd_lib_draw_stringP(34 - strlen_P(AButtonText) * 3, 56, AButtonText);
    }
    if (IS_SELECTED_MAIN(1))
    {
      lcd_lib_draw_box(3+61+3, 54+1, 63+61-3, 64-1);
      lcd_lib_set(3+61+4, 54+2, 63+61-4, 64-2);
      lcd_lib_clear_stringP(34+61 - strlen_P(BButtonText) * 3, 56, BButtonText);
    }else{
      lcd_lib_draw_stringP(34+61 - strlen_P(BButtonText) * 3, 56, BButtonText);
    }
    break;
  default:
    break;
  }



}

void lcd_progressbar(uint8_t progress)
{
  lcd_lib_draw_box(3, 38, 124, 46);

  static uint8_t moveContinue=0;

  for(uint8_t n=0; n<progress; n++)
  {
    if (n>120) break;
    uint8_t m = (progress-n-1+moveContinue) % 12;
    if (m < 5)
      lcd_lib_draw_vline(4 + n, 40, 40+m);
    else if (m < 10)
      lcd_lib_draw_vline(4 + n, 40+m-5, 44);
  }

  moveContinue++;
  if (moveContinue==12) {
    moveContinue=0;
  }

}

void lcd_scroll_menu(const char* menuNameP, int8_t entryCount, entryNameCallback_t entryNameCallback, entryDetailsCallback_t entryDetailsCallback)
{
  if (lcd_lib_button_pressed)
    return;    //Selection possibly changed the menu, so do not update it this cycle.

  if (lcd_lib_encoder_pos < 0) lcd_lib_encoder_pos = 0;
  if (lcd_lib_encoder_pos >= entryCount * ENCODER_TICKS_PER_SCROLL_MENU_ITEM) lcd_lib_encoder_pos = entryCount * ENCODER_TICKS_PER_SCROLL_MENU_ITEM - 1;

  uint8_t selIndex = uint16_t(lcd_lib_encoder_pos/ENCODER_TICKS_PER_SCROLL_MENU_ITEM);

  lcd_lib_clear();

  int16_t targetViewPos;
  int16_t viewDiff;
  int16_t drawOffset;
  uint8_t itemOffset;

  switch (languageType) {
  case LANGUAGE_CHINESE:
  case LANGUAGE_KOREAN:
  {
    static int16_t viewPos = 8 + 12;
    targetViewPos = selIndex * 12;
    viewDiff = targetViewPos - viewPos;

    if (viewDiff<0) {
      viewDiff += 18;
      if (viewDiff > 0) {
        viewDiff=0;
      }
    }
    else if (viewDiff>0) {
      viewDiff -= 18;
      if (viewDiff<0) {
        viewDiff=0;
      }
    }

    viewPos += viewDiff / 4;
    if (viewDiff > 0) { viewPos++; led_glow = led_glow_dir = 0; }
    if (viewDiff < 0) { viewPos--; led_glow = led_glow_dir = 0; }

    if (viewPos<0) {
      viewPos=0;
    }

    drawOffset = 7 + 12 -viewPos % 12;
    itemOffset = viewPos / 12;
    for(int8_t n=-2; n<4; n++)
    {
      uint8_t itemIdx = n + itemOffset;
      if (itemIdx >= entryCount)
        continue;

      char* ptr = entryNameCallback(itemIdx);
      ptr[20] = '\0';
      if (itemIdx == selIndex)
      {
        lcd_lib_draw_string(0, drawOffset+12*n, CHINESE_POINT);
        lcd_lib_draw_string(12, drawOffset+12*n, ptr);
      }else{
        lcd_lib_draw_string(12, drawOffset+12*n, ptr);
      }
    }
    lcd_lib_clear(0, 0, 127,0);

    lcd_lib_clear(0, 50, 127, 63);
    lcd_lib_clear(127-strlen_P(menuNameP)*6-6, 0, 127, 13);
    lcd_lib_draw_hline(127-strlen_P(menuNameP)*6-6, 127, 13);

    lcd_lib_clear(0, 49, 127,49);
    lcd_lib_draw_hline(0, 127, 50);
    lcd_lib_draw_stringP(127-strlen_P(menuNameP)*6, 1, menuNameP);
  }
  break;

  case LANGUAGE_ENGLISH:
  {
    static int16_t viewPos = 8 +20;

    targetViewPos = selIndex * 10;
    viewDiff = targetViewPos - viewPos;

    if (viewDiff<0) {
      viewDiff += 15;
      if (viewDiff > 0) {
        viewDiff=0;
      }
    }
    else if (viewDiff>0) {
      viewDiff -= 15;
      if (viewDiff<0) {
        viewDiff=0;
      }
    }

    viewPos += viewDiff / 4;
    if (viewDiff > 0) { viewPos++; led_glow = led_glow_dir = 0; }
    if (viewDiff < 0) { viewPos--; led_glow = led_glow_dir = 0; }

    if (viewPos<0) {
      viewPos=0;
    }

    drawOffset = 8 +20 -viewPos % 10;
    itemOffset = viewPos / 10;
    for(int8_t n=-2; n<4; n++)
    {
      uint8_t itemIdx = n + itemOffset;
      if (itemIdx >= entryCount)
        continue;

      char* ptr = entryNameCallback(itemIdx);
      ptr[20] = '\0';
      if (itemIdx == selIndex)
      {
        lcd_lib_draw_string(0, drawOffset+10*n, ENGLISH_POINT);
        lcd_lib_draw_string(10, drawOffset+10*n, ptr);
      }else{
        lcd_lib_draw_string(10, drawOffset+10*n, ptr);
      }
    }

    lcd_lib_clear(0, 53, 127, 63);
    lcd_lib_clear(0, 0, 127, 9);
    lcd_lib_draw_hline(127-strlen_P(menuNameP)*6-12, 127, 10);
    lcd_lib_draw_hline(0, 127, 53);
    lcd_lib_draw_stringP(127-strlen_P(menuNameP)*6, 1, menuNameP);
  }
  default:
    break;
  }

  entryDetailsCallback(selIndex);
}


void lcd_normal_menu(const char* menuNameP, int8_t entryCount, entryNameCallback_t entryNameCallback, entryDetailsCallback_t entryDetailsCallback)
{
  if (lcd_lib_encoder_pos < 0) lcd_lib_encoder_pos = -1;
  if (lcd_lib_encoder_pos >= entryCount * ENCODER_TICKS_PER_SCROLL_MENU_ITEM) lcd_lib_encoder_pos = entryCount * ENCODER_TICKS_PER_SCROLL_MENU_ITEM;

  uint8_t selIndex =constrain(lcd_lib_encoder_pos/ENCODER_TICKS_PER_SCROLL_MENU_ITEM, 0, entryCount-1);
  uint8_t drawOffset;

  lcd_lib_clear();

  switch (languageType) {
  case LANGUAGE_CHINESE:
  case LANGUAGE_KOREAN:
    drawOffset = (49-12*entryCount)/2+1;
    for(int8_t n=0; n<entryCount; n++)
    {
      char* ptr = entryNameCallback(n);
      ptr[20] = '\0';

      if (n == selIndex) {
        lcd_lib_draw_string(54-6, drawOffset+12*n, ptr);
      }else{
        lcd_lib_draw_string(54+6, drawOffset+12*n, ptr+2);
      }
    }
    lcd_lib_draw_hline(0, 127, 53-3);

    break;
  case LANGUAGE_ENGLISH:
    drawOffset = (52-13*entryCount)/2+3;
    for(int8_t n=0; n<entryCount; n++)
    {
      char* ptr = entryNameCallback(n);
      ptr[20] = '\0';
      if (n == selIndex) {
        lcd_lib_draw_string(54, drawOffset+13*n, ptr);
      }else{
        lcd_lib_draw_string(60, drawOffset+13*n, ptr+1);
      }
    }
    lcd_lib_draw_hline(0, 127, 53);
    break;
  default:
    break;
  }

  entryDetailsCallback(selIndex);
}

void lcd_advance_menu(const char* menuNameP, int8_t entryCount, entryNameCallback_t entryNameCallback, entryDetailsCallback_t entryDetailsCallback)
{

  if (lcd_lib_encoder_pos < 0) lcd_lib_encoder_pos = -1;
  if (lcd_lib_encoder_pos >= entryCount * ENCODER_TICKS_PER_SCROLL_MENU_ITEM) lcd_lib_encoder_pos = entryCount * ENCODER_TICKS_PER_SCROLL_MENU_ITEM;

  uint8_t selIndex =constrain(lcd_lib_encoder_pos/ENCODER_TICKS_PER_SCROLL_MENU_ITEM, 0, entryCount-1);

  uint8_t drawOffset;

  lcd_lib_clear();

  switch (languageType) {
  case LANGUAGE_CHINESE:
  case LANGUAGE_KOREAN:
    drawOffset = (49-12*entryCount)/2+1;

    for(int8_t n=0; n<entryCount; n++)
    {

      char* ptr = entryNameCallback(n);
      ptr[20] = '\0';
      if (n == selIndex)
      {
        lcd_lib_draw_string(4, drawOffset+12*n, CHINESE_POINT);
        lcd_lib_draw_string(16, drawOffset+12*n, ptr);
      }else{
        lcd_lib_draw_string(16, drawOffset+12*n, ptr);
      }
    }
    if (menuNameP != NULL) {
      lcd_lib_draw_hline(127-strlen_P(menuNameP)*6-6, 127, 10 + 3);
      lcd_lib_draw_stringP(127-strlen_P(menuNameP)*6, 1, menuNameP);
    }
    lcd_lib_draw_hline(0, 127, 53 - 3);

    break;
  case LANGUAGE_ENGLISH:
    drawOffset = (40-10*entryCount)/2+13;
    for(int8_t n=0; n<entryCount; n++)
    {

      char* ptr = entryNameCallback(n);
      ptr[20] = '\0';
      if (n == selIndex)
      {
        lcd_lib_draw_string(4, drawOffset+10*n, ENGLISH_POINT);
        lcd_lib_draw_string(10, drawOffset+10*n, ptr);
      }else{
        lcd_lib_draw_string(10, drawOffset+10*n, ptr);
      }
    }
    if (menuNameP != NULL) {
      lcd_lib_draw_hline(127-strlen_P(menuNameP)*6-6, 127, 10);
      lcd_lib_draw_stringP(127-strlen_P(menuNameP)*6, 1, menuNameP);
    }
    lcd_lib_draw_hline(0, 127, 53);
    break;
  default:

    break;
  }




  entryDetailsCallback(selIndex);
}



void lcd_menu_edit_setting()
{
  if (lcd_lib_encoder_pos < lcd_setting_min)
    lcd_lib_encoder_pos = lcd_setting_min;
  if (lcd_lib_encoder_pos > lcd_setting_max)
    lcd_lib_encoder_pos = lcd_setting_max;

  if (lcd_setting_type == 1)
    *(uint8_t*)lcd_setting_ptr = lcd_lib_encoder_pos;
  else if (lcd_setting_type == 2)
    *(uint16_t*)lcd_setting_ptr = lcd_lib_encoder_pos;
  else if (lcd_setting_type == 3)
    *(float*)lcd_setting_ptr = float(lcd_lib_encoder_pos) / 100.0;
  else if (lcd_setting_type == 4)
    *(int32_t*)lcd_setting_ptr = lcd_lib_encoder_pos;
  else if (lcd_setting_type == 5)
    *(uint8_t*)lcd_setting_ptr = lround(lcd_lib_encoder_pos * 255 / 100.0);
  else if (lcd_setting_type == 6)
    *(float*)lcd_setting_ptr = float(lcd_lib_encoder_pos) * 60;
  else if (lcd_setting_type == 7)
    *(float*)lcd_setting_ptr = float(lcd_lib_encoder_pos) * 100;
  else if (lcd_setting_type == 8)
    *(float*)lcd_setting_ptr = float(lcd_lib_encoder_pos);
  else if (lcd_setting_type == 9)
    *(uint8_t*)lcd_setting_ptr = lcd_lib_encoder_pos;

  lcd_lib_clear();
  char buffer[16];
  if (lcd_setting_type == 3)
    float_to_string(float(lcd_lib_encoder_pos) / 100.0, buffer, lcd_setting_postfix);
  else
    int_to_string(lcd_lib_encoder_pos, buffer, lcd_setting_postfix);

  
  lcd_lib_draw_string_centerP(LS(20, 11, 11) , lcd_setting_name);
  lcd_lib_draw_string_center(LS(30, 24, 24) , buffer);

  if (lcd_lib_button_pressed)
  {
    lcd_change_to_menu(previousMenu, previousEncoderPos, MenuBackward);
    lcd_lib_button_up_down_reversed=false;
    if (lcd_setting_type == 9) {
      Config_StoreSettings();
    }
  }

}
#endif//ENABLE_ULTILCD2

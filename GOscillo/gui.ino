void DrawText() {
  clear_text_area();

  switch (menu) {
  case 0:
    set_line_color(0);
    if (ch0_mode != MODE_OFF) {
      display_range(range0);
    } else {
      display.print(F("CH2")); display_ac(CH1DCSW);
    }
    set_line_color(1);
    if (ch1_mode != MODE_OFF && rate > 4 && rate < (RATE_MAX + 1)) {
      display_range(range1);
    } else {
      display.print(F("CH1")); display_ac(CH0DCSW);
    }
    set_line_color(2);
    display_rate();
    set_line_color(3);
    if (rate <= RATE_MAX) display.print(F("real"));
    else display.print(F("eqiv"));
    set_line_color(4);
    display_trig_mode();
    set_line_color(5);
    display.print(trig_ch == ad_ch0 ? F("TG1") : F("TG2")); 
#ifdef _ADAFRUIT_GFX_H
    display.print(trig_edge == TRIG_E_UP ? char(0x18) : char(0x19)); 
#else
    display.print(trig_edge == TRIG_E_UP ? '/' : '\\'); 
#endif
    set_line_color(6);
    display.print(F("Tlev")); 
    set_line_color(7);
    display.print(Start ? F("RUN") : F("HOLD")); 
    break;
  case 1:
    set_line_color(0);
    display.print(F("CH1")); display_ac(CH0DCSW);
    set_line_color(1);
    display_mode(ch0_mode);
    set_line_color(2);
    display_range(range0);
    set_line_color(3);
    display.print(F("OFS1")); 
    set_line_color(4);
    display.print(F("CH2")); display_ac(CH1DCSW);
    set_line_color(5);
    display_mode(ch1_mode);
    set_line_color(6);
    display_range(range1);
    set_line_color(7);
    display.print(F("OFS2"));
    break;
  case 2:
    set_line_color(0);
    display_range(range0);
    set_line_color(1);
    display_rate();
    set_line_color(2);
    if (!fft_mode) {
      display.print(F("FFT")); 
      set_line_color(3);
      display.print(F("FREQ")); 
      set_line_color(4);
      display.print(F("VOLT")); 
      set_line_color(5);
      display.print(F("PWM")); 
      set_line_color(6);
      display.print(F("DUTY")); 
      set_line_color(7);
      display.print(F("FREQ"));
      if (pulse_mode && (item > 20 && item < 24)) {
        display.setTextColor(TXTCOLOR, BGCOLOR);
        display.setCursor(DISPLNG - 28, txtLINE7);
        disp_pulse_frq();
      }
    }
    break;
  case 3:
    set_line_color(0);
    display_range(range0);
    set_line_color(1);
    display_rate();
    set_line_color(2);
    display.print(F("DDS"));
    set_line_color(3);
    disp_dds_wave();
    set_line_color(4);
    display.print(F("FREQ"));
    if (dds_mode) disp_dds_freq();
    set_line_color(5);
    display.print(F("FCNT"));
    fcount_disp();
    break;
  }
//  if (info_mode && Start) {
  if (info_mode & 3) {
    int ch = (info_mode & 4) ? 1 : 0;
    if (ch == 0) {
      display.setTextColor(CH1COLOR, BGCOLOR);
    } else {
      display.setTextColor(CH2COLOR, BGCOLOR);
    }
    dataAnalize(ch);
    if (info_mode & 1)
      measure_frequency(ch);
    if (info_mode & 2)
      measure_voltage(ch);
  }
  if (!full_screen && !fft_mode)
    display.drawFastHLine(DISPLNG, 60 - trig_lv, 3, TRGCOLOR);  // draw trig_lv tic
}

void TextBG(byte *y, int x, byte chrs) {
  int yinc, wid, hi;
  yinc = 10, wid = 6, hi = 8;
  display.fillRect(x, *y, wid * chrs - 1, hi, BGCOLOR);
  display.setCursor(x, *y);
  *y += yinc;
}

#define BTN_UP    0
#define BTN_DOWN  10
#define BTN_LEFT  7
#define BTN_RIGHT 3
#define BTN_FULL  12
#define BTN_RESET 11

byte lastsw = 255;
unsigned long vtime;

void CheckSW() {
  static unsigned long Millis = 0;
  unsigned long ms;
  byte sw;

  ms = millis();
  if ((ms - Millis)<200)
    return;
  Millis = ms;

/* SW10 Menu
 * SW9  CH1 range down
 * SW8  CH2 range down
 * SW7  TIME/DIV slow
 * SW6  TRIG_MODE down
 * SW5  Send
 * SW4  TRIG_MODE up
 * SW3  TIME/DIV fast
 * SW2  CH2 range up
 * SW1  CH1 range up
 * SW0  Start/Hold
 */
#ifdef BUTTON5DIR
  if (digitalRead(DOWNPIN) == LOW && digitalRead(LEFTPIN) == LOW) {
    sw = BTN_RESET; // both button press
  } else if (digitalRead(UPPIN) == LOW && digitalRead(RIGHTPIN) == LOW) {
    sw = BTN_FULL;  // both button press
#else
  if (digitalRead(RIGHTPIN) == LOW && digitalRead(LEFTPIN) == LOW) {
    sw = BTN_RESET; // both button press
  } else if (digitalRead(UPPIN) == LOW && digitalRead(DOWNPIN) == LOW) {
    sw = BTN_FULL;  // both button press
#endif
  } else if (digitalRead(DOWNPIN) == LOW) {
    sw = BTN_DOWN;  // down
  } else if (digitalRead(RIGHTPIN) == LOW) {
    sw = BTN_RIGHT; // right
  } else if (digitalRead(LEFTPIN) == LOW) {
    sw = BTN_LEFT;  // left
  } else if (digitalRead(UPPIN) == LOW) {
    sw = BTN_UP;    // up
  } else {
    lastsw = 255;
    return;
  }
  if (sw != lastsw)
    vtime = ms;
  saveTimer = 5000;     // set EEPROM save timer to 5 second
  if (sw == BTN_FULL) {
    full_screen = !full_screen;
    clear_text_area();
  } else {
    switch (menu) {
    case 0:
      menu0_sw(sw); 
      break;
    case 1:
      menu1_sw(sw); 
      break;
    case 2:
      menu2_sw(sw); 
      break;
    case 3:
      menu3_sw(sw); 
      break;
    default:
      break;
    }
    DrawText();
    display.display();
  }
  lastsw = sw;
}

void updown_ch0range(byte sw) {
  if (sw == BTN_RIGHT) {        // CH0 RANGE +
    if (range0 > 0)
      range0 --;
  } else if (sw == BTN_LEFT) {  // CH0 RANGE -
    if (range0 < RANGE_MAX)
      range0 ++;
  }
}

void updown_ch1range(byte sw) {
  if (sw == BTN_RIGHT) {        // CH1 RANGE +
    if (range1 > 0)
      range1 --;
  } else if (sw == BTN_LEFT) {  // CH1 RANGE -
    if (range1 < RANGE_MAX)
      range1 ++;
  }
}

void updown_rate(byte sw) {
  if (sw == BTN_RIGHT) {        // RATE FAST
    if (rate > 0 && rate != (RATE_MAX + 1)) rate --;
    else if (rate == 0) {
      rate = RATE_NUM - 1;
      dds_close();  // dds_mode = false;
    }
  } else if (sw == BTN_LEFT) {  // RATE SLOW
    if (rate > RATE_NUM - 2) rate = 0;
    else if (rate != RATE_MAX) rate ++;
  }
}

void menu0_sw(byte sw) {  
  switch (item) {
  case 0: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 1: // CH1 voltage range
    updown_ch1range(sw);
    break;
  case 2: // rate
    updown_rate(sw);
    break;
  case 3: // sampling mode
    break;
  case 4: // trigger mode
    if (sw == BTN_RIGHT) {        // TRIG MODE +
      if (trig_mode < TRIG_ONE)
        trig_mode ++;
      else
        trig_mode = 0;
    } else if (sw == BTN_LEFT) {  // TRIG MODE -
      if (trig_mode > 0)
        trig_mode --;
      else
        trig_mode = TRIG_ONE;
    }
    if (trig_mode != TRIG_ONE)
        Start = true;
    break;
  case 5: // trigger source and polarity
    if (sw == BTN_RIGHT) {        // trigger + edge
      if (trig_edge == TRIG_E_UP)
        trig_edge = TRIG_E_DN;
      else
        trig_edge = TRIG_E_UP;
    } else if (sw == BTN_LEFT) {  // trigger - channel
      if (trig_ch == ad_ch0)
        trig_ch = ad_ch1;
      else
        trig_ch = ad_ch0;
    }
    break;
  case 6: // trigger level
    if (sw == BTN_RIGHT) {        // trigger level +
      if (trig_lv < LCD_YMAX) {
        trig_lv ++;
      }
    } else if (sw == BTN_LEFT) {  // trigger level -
      if (trig_lv > 0) {
        trig_lv --;
      }
    }
    break;
  case 7: // run / hold
    if (sw == BTN_RIGHT || sw == BTN_LEFT) {
      Start = !Start;
    }
    break;
  }
  menu_updown(sw);
}

void menu1_sw(byte sw) {  
  switch (item - 8) {
  case 1: // CH0 mode
    if (sw == BTN_RIGHT) {        // CH0 + ON/INV
      if (ch0_mode == MODE_ON)
        ch0_mode = MODE_INV;
      else
        ch0_mode = MODE_ON;
    } else if (sw == BTN_LEFT) {  // CH0 - ON/OFF
      if (ch0_mode == MODE_OFF)
        ch0_mode = MODE_ON;
      else
        ch0_mode = MODE_OFF;
    }
    break;
  case 2: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 3: // CH0 offset
    if (sw == BTN_RIGHT) {        // offset +
      if (ch0_off < 1023)
        ch0_off += 1024/VREF[range0];
    } else if (sw == BTN_LEFT) {  // offset -
      if (ch0_off > -1023)
        ch0_off -= 1024/VREF[range0];
    } else if (sw == BTN_RESET) { // offset reset
      if (digitalRead(CH0DCSW) == LOW)    // DC/AC input
        ch0_off = pgm_read_dword(&ac_offset[range0]);
      else
        ch0_off = 0;
    }
    break;
  case 5: // CH1 mode
    if (sw == BTN_RIGHT) {        // CH1 + ON/INV
      if (ch1_mode == MODE_ON)
        ch1_mode = MODE_INV;
      else
        ch1_mode = MODE_ON;
    } else if (sw == BTN_LEFT) {  // CH1 - ON/OFF
      if (ch1_mode == MODE_OFF) {
        ch1_mode = MODE_ON;
      } else {
        ch1_mode = MODE_OFF;
      }
    }
    break;
  case 6: // CH1 voltage range
    updown_ch1range(sw);
    break;
  case 7: // CH1 offset
    if (sw == BTN_RIGHT) {        // offset +
      if (ch1_off < 1023)
        ch1_off += 1024/VREF[range1];
    } else if (sw == BTN_LEFT) {  // offset -
      if (ch1_off > -1023)
        ch1_off -= 1024/VREF[range1];
    } else if (sw == BTN_RESET) { // offset reset
      if (digitalRead(CH1DCSW) == LOW)    // DC/AC input
        ch1_off = pgm_read_dword(&ac_offset[range1]);
      else
        ch1_off = 0;
    }
    break;
  }
  menu_updown(sw);
}

void menu2_sw(byte sw) {
  char diff;
  switch (item - 16) {
  case 0: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 1: // rate
    updown_rate(sw);
    break;
  case 2: // FFT mode
    if (sw == BTN_RIGHT) {        // ON
      fft_mode = true;
    } else if (sw == BTN_LEFT) {  // OFF
      fft_mode = false;
    }
    break;
  case 3: // Frequency and Duty display
    if (sw == BTN_RIGHT) {        // ON
      info_mode |= 1;
    } else if (sw == BTN_LEFT) {  // OFF
      info_mode &= ~1;
    }
    break;
  case 4: // Voltage display
    if (sw == BTN_RIGHT) {        // ON
      info_mode |= 2;
    } else if (sw == BTN_LEFT) {  // OFF
      info_mode &= ~2;
    }
    break;
  case 5: // PWM
    if (sw == BTN_RIGHT) {        // +
      update_frq(0);
      pulse_start();
      pulse_mode = true;
    } else if (sw == BTN_LEFT) {  // -
      pulse_close();
      pulse_mode = false;
    }
    break;
  case 6: // PWM Duty ratio
    diff = 1;
    if (sw == lastsw) {
      if (millis() - vtime > 5000) diff = 8;
    }
    if (sw == BTN_RIGHT) {        // +
      if (pulse_mode) {
        if ((256 - duty) > diff) duty += diff;
      } else {
        pulse_start();
      }
      update_frq(0);
      pulse_mode = true;
    } else if (sw == BTN_LEFT) {  // -
      if (pulse_mode) {
        if (duty > diff) duty -= diff;
      } else {
        pulse_start();
      }
      update_frq(0);
      pulse_mode = true;
    }
    break;
  case 7: // PWM Frequency
    diff = sw_accel(sw);
    if (sw == BTN_RIGHT) {        // +
      if (pulse_mode)
        update_frq(-diff);
      else {
        update_frq(0);
        pulse_start();
      }
      pulse_mode = true;
    } else if (sw == BTN_LEFT) {  // -
      if (pulse_mode)
        update_frq(diff);
      else {
        update_frq(0);
        pulse_start();
      }
      pulse_mode = true;
    }
    break;
  }
  menu_updown(sw);
}

void menu3_sw(byte sw) {
  char diff;
  switch (item - 24) {
  case 0: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 1: // rate
    updown_rate(sw);
    break;
  case 2: // DDS
    if (sw == BTN_RIGHT) {        // +
      fcount_close();
      dds_setup();
      dds_mode = true;
    } else if (sw == BTN_LEFT) {  // -
      dds_close();  // dds_mode = false;
    }
    break;
  case 3: // WAVE
    if (sw == BTN_RIGHT) {        // +
      rotate_wave(true);
    } else if (sw == BTN_LEFT) {  // -
      rotate_wave(false);
    }
    break;
  case 4: // FREQ
    diff = sw_accel(sw);
    if (sw == BTN_RIGHT) {        // +
      update_ifrq(diff);
    } else if (sw == BTN_LEFT) { // -
      update_ifrq(-diff);
    }
    break;
  case 5: // Frequency Counter
    if (sw == BTN_RIGHT && rate <= RATE_MAX) {  // on
      dds_close();  // dds_mode = false;
      pulse_close();
      pulse_mode = false; fcount_mode = true;
      FreqCount.begin(1000);
    } else if (sw == BTN_LEFT) {                // off
      fcount_close();
    }
    break;
  }
  menu_updown(sw);
}

void menu_updown(byte sw) {
  if (sw == BTN_DOWN) {       // MENU down SW
    increment_item();
  } else if (sw == BTN_UP) {  // Menu up SW
    decrement_item();
  }
}

void increment_item() {
  ++item;
  if (item > ITEM_MAX) item = 0;
  if (item == 3) item = 4;  // skip real/DMA
  if (item < 16 || item > 18) fft_mode = false; // exit FFT mode
  menu = item >> 3;
}

void decrement_item() {
  if (item > 0) --item;
  else item = ITEM_MAX;
  if (item == 3) item = 2;  // skip real/DMA
  if (item < 16 || item > 18) fft_mode = false; // exit FFT mode
  menu = item >> 3;
}

byte sw_accel(byte sw) {
  char diff = 1;
  if (sw == lastsw) {
    unsigned long curtime = millis();
    if (curtime - vtime > 6000) diff = 4;
    else if (curtime - vtime > 4000) diff = 3;
    else if (curtime - vtime > 2000) diff = 2;
  }
  return (diff);
}

void clear_text_area(void) {
  display.fillRect(DISPLNG+1,0,LCD_WIDTH-DISPLNG-1,LCD_HEIGHT, BGCOLOR);  // clear text area that will be drawn below
}

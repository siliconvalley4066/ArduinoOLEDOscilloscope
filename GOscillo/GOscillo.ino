/*
 * Arduino Oscilloscope using a 128x64 OLED Version 1.30
 * The max realtime sampling rates are 17.2ksps with 2 channels and 307ksps with a channel.
 * The max equivalent time sampling rates is 16Msps with single channel.
 * + Pulse Generator
 * + PWM DDS Function Generator (8 waveforms)
 * + Frequency Counter
 * Copyright (c) 2022, Siliconvalley4066
 */
/*
 * Arduino Oscilloscope using a graphic LCD
 * The max sampling rates are 4.3ksps with 2 channels and 8.6ksps with a channel.
 * Copyright (c) 2009, Noriaki Mitsunaga
 */

#include <Adafruit_GFX.h>

//#define DISPLAY_IS_SSD1306
#define OLED_RESET     -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#ifdef DISPLAY_IS_SSD1306
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH   128              // OLED display width
#define SCREEN_HEIGHT   64              // OLED display height
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#else
#include <Adafruit_SH1106.h>
Adafruit_SH1106 display(OLED_RESET);
#endif
#include <EEPROM.h>
#include <fix_fft.h>
#include <FreqCount.h>
#define FFT_N 128

#define txtLINE0   0
#define txtLINE1   8
#define txtLINE2   16
#define txtLINE3   24
#define txtLINE4   32

float waveFreq;                // frequency (Hz)
float waveDuty;                // duty ratio (%)
int dataMin;                   // buffer minimum value (smallest=0)
int dataMax;                   //        maximum value (largest=1023)
int dataAve;                   // 10 x average value (use 10x value to keep accuracy. so, max=10230)
int saveTimer;                 // remaining time for saving EEPROM
int timeExec;                  // approx. execution time of current range setting (ms)
extern byte duty;
extern byte p_range;
extern unsigned int count;
extern long ifreq;
extern byte wave_id;

const int LCD_WIDTH = 128;
const int LCD_HEIGHT = 64;
const int SAMPLES = 128;
const int DISPLNG = 100;
const int ad_ch0 = 0;                   // Analog 0 pin for channel 0
const int ad_ch1 = 1;                   // Analog 1 pin for channel 1
const long VREF[] = {49, 98, 244, 488, 976}; // reference voltage 5.0V ->  50 :   1V/div range (100mV/dot)
                                        //                        -> 100 : 0.5V/div
                                        //                        -> 250 : 0.2V/div
                                        //                        -> 500 : 100mV/div
                                        //                       -> 1000 :  50mV/div
//const int MILLIVOL_per_dot[] = {100, 50, 20, 10, 5}; // mV/dot
#define CALPIN 10
#define CH0DCSW 2
#define CH1DCSW 4
const int ac_offset[] PROGMEM = {104, -204, -388, -450, -481};
const int MODE_ON = 0;
const int MODE_INV = 1;
const int MODE_OFF = 2;
const char ModesN[3][4] PROGMEM = {"ON", "INV", "OFF"};
const char * const Modes[3] PROGMEM = {ModesN[0], ModesN[1], ModesN[2]};
const int TRIG_AUTO = 0;
const int TRIG_NORM = 1;
const int TRIG_SCAN = 2;
const int TRIG_ONE  = 3;
const char TRIG_ModesN[4][5] PROGMEM = {"Auto", "Norm", "Scan", "One"};
const char * const TRIG_Modes[4] PROGMEM = {TRIG_ModesN[0], TRIG_ModesN[1], TRIG_ModesN[2], TRIG_ModesN[3]};
const int TRIG_E_UP = 0;
const int TRIG_E_DN = 1;
#define RATE_MIN 0
#define RATE_MAX 19
#define RATE_NUM 27
#define ITEM_MAX 29
const char RN[RATE_NUM][5] PROGMEM = {"33us", "49us", "100u", "200u", "500u", "640u", "800u", " 1ms", " 2ms", " 5ms", "10ms", "20ms", "50ms", "0.1s", "0.2s", "0.5s", " 1s ", " 2s ", " 5s ", " 10s",
                                      "0.6u", "1.3u", "3.1u", "6.3u", "13us", "31us", "63us"};
const char * const Rates[RATE_NUM] PROGMEM = {RN[0], RN[1], RN[2], RN[3], RN[4], RN[5], RN[6], RN[7], RN[8], RN[9], RN[10], RN[11], RN[12], RN[13], RN[14], RN[15], RN[16], RN[17], RN[18], RN[19],
                                              RN[20], RN[21], RN[22], RN[23], RN[24], RN[25], RN[26]};
const unsigned long HREF[] PROGMEM = {33, 49, 100, 200, 500, 640, 800, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000, 5000000, 10000000};
#define RANGE_MIN 0
#define RANGE_MAX 4
#define VRF 5
const char RangesN[5][5] PROGMEM = {" 1V ", "0.5V", "0.2V", "0.1V", "50mV"};
const char * const Ranges[5] PROGMEM = {RangesN[0], RangesN[1], RangesN[2], RangesN[3], RangesN[4]};
byte range0 = RANGE_MIN;
byte range1 = RANGE_MIN;
byte ch0_mode = MODE_ON, ch1_mode = MODE_ON, rate = 0;
byte trig_mode = TRIG_AUTO, trig_lv = 10, trig_edge = TRIG_E_UP, trig_ch = ad_ch0;
bool Start = true;  // Start sampling
byte item = 0;      // Default item
byte menu = 0;      // Default menu
short ch0_off = -200, ch1_off = 400;
byte data[2][SAMPLES];                  // keep the number of channels buffer
byte odat00, odat01, odat10, odat11;    // old data buffer for erase
byte sample=0;                          // index for double buffer
bool fft_mode = false, pulse_mode = false, dds_mode = false, fcount_mode = false;
bool full_screen = false;
byte info_mode = 3; // Text information display mode

void setup(){
  pinMode(2, INPUT_PULLUP);             // CH1 DC/AC
//  pinMode(3, OUTPUT);                   // Trigger level PWM out
  pinMode(4, INPUT_PULLUP);             // CH2 DC/AC
//  pinMode(5, INPUT);                    // Frequency Counter in
//  pinMode(6, INPUT);                    // Trigger level AC input
  pinMode(7, INPUT_PULLUP);             // up
  pinMode(8, INPUT_PULLUP);             // down
  pinMode(9, INPUT_PULLUP);             // right
  pinMode(10, OUTPUT);                  // PWM out
  pinMode(11, OUTPUT);                  // DDS out
  pinMode(12, INPUT_PULLUP);            // left
  DIDR0 = _BV(ADC1D) | _BV(ADC0D);      // disable digital input buffer of A0-A1
#ifdef DISPLAY_IS_SSD1306
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // select 3C or 3D (set your OLED I2C address)
#else
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);   // initialise the library
#endif

//  Serial.begin(115200);
  loadEEPROM();                         // read last settings from EEPROM
//  set_default();
  menu = item >> 3;
  display.clearDisplay();
//  DrawGrid();
//  DrawText();
//  display.display();
  (void) analogRead(ad_ch0);            // read and neglect to setup ADC
  if (pulse_mode)
    pulse_init();                       // calibration pulse output
  if (dds_mode)
    dds_setup();
}

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
  if (digitalRead(9) == LOW && digitalRead(12) == LOW) {
    sw = 11;    // both button press
  } else if (digitalRead(7) == LOW && digitalRead(8) == LOW) {
    sw = 12;    // both button press
  } else if (digitalRead(8) == LOW) {
    sw = 10;    // down
  } else if (digitalRead(9) == LOW) {
    sw = 3;     // right
  } else if (digitalRead(12) == LOW) {
    sw = 7;     // left
  } else if (digitalRead(7) == LOW) {
    sw = 0;     // up
  } else {
    lastsw = 255;
    return;
  }
  if (sw != lastsw)
    vtime = ms;
  saveTimer = 5000;     // set EEPROM save timer to 5 secnd
  if (sw == 12) {
    full_screen = !full_screen;
    display.fillRect(DISPLNG + 1,0,27,64, BLACK);  // clear text area that will be drawn below 
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
  if (sw == 3) {        // CH0 RANGE +
    if (range0 > 0)
      range0 --;
  } else if (sw == 7) { // CH0 RANGE -
    if (range0 < RANGE_MAX)
      range0 ++;
  }
}

void updown_ch1range(byte sw) {
  if (sw == 3) {        // CH1 RANGE +
    if (range1 > 0)
      range1 --;
  } else if (sw == 7) { // CH1 RANGE -
    if (range1 < RANGE_MAX)
      range1 ++;
  }
}

void updown_rate(byte sw) {
  if (sw == 3) {        // RATE FAST
    if (rate > 0 && rate != (RATE_MAX + 1)) rate --;
    else if (rate == 0) {
      rate = RATE_NUM - 1;
      dds_close();  // dds_mode = false;
    }
  } else if (sw == 7) { // RATE SLOW
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
    if (sw == 3) {        // TRIG MODE +
      if (trig_mode < TRIG_ONE)
        trig_mode ++;
      else
        trig_mode = 0;
    } else if (sw == 7) { // TRIG MODE -
      if (trig_mode > 0)
        trig_mode --;
      else
        trig_mode = TRIG_ONE;
    }
    if (trig_mode != TRIG_ONE)
        Start = true;
    break;
  case 5: // trigger source and polarity
    if (sw == 3) {        // trigger + edge
      if (trig_edge == TRIG_E_UP)
        trig_edge = TRIG_E_DN;
      else
        trig_edge = TRIG_E_UP;
    } else if (sw == 7) { // trigger - channel
      if (trig_ch == ad_ch0)
        trig_ch = ad_ch1;
      else
        trig_ch = ad_ch0;
    }
    break;
  case 6: // trigger level
    if (sw == 3) {        // trigger level +
      if (trig_lv < 60)
        trig_lv ++;
    } else if (sw == 7) { // trigger level -
      if (trig_lv > 0)
        trig_lv --;
    }
    break;
  case 7: // run / hold
    if (sw == 3 || sw == 7) {
      Start = !Start;
    }
    break;
  }
  menu_updown(sw);
}

void menu1_sw(byte sw) {  
  switch (item - 8) {
  case 1: // CH0 mode
    if (sw == 3) {        // CH0 + ON/INV
      if (ch0_mode == MODE_ON)
        ch0_mode = MODE_INV;
      else
        ch0_mode = MODE_ON;
    } else if (sw == 7) { // CH0 - ON/OFF
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
    if (sw == 3) {        // offset +
      if (ch0_off < 1023)
        ch0_off += 1024/VREF[range0];
    } else if (sw == 7) { // offset -
      if (ch0_off > -1023)
        ch0_off -= 1024/VREF[range0];
    } else if (sw == 11) { // offset reset
      if (digitalRead(CH0DCSW) == LOW)    // DC/AC input
        ch0_off = pgm_read_dword(&ac_offset[range0]);
      else
        ch0_off = 0;
    }
    break;
  case 5: // CH1 mode
    if (sw == 3) {        // CH1 + ON/INV
      if (ch1_mode == MODE_ON)
        ch1_mode = MODE_INV;
      else
        ch1_mode = MODE_ON;
    } else if (sw == 7) { // CH1 - ON/OFF
      if (ch1_mode == MODE_OFF)
        ch1_mode = MODE_ON;
      else
        ch1_mode = MODE_OFF;
    }
    break;
  case 6: // CH1 voltage range
    updown_ch1range(sw);
    break;
  case 7: // CH1 offset
    if (sw == 3) {        // offset +
      if (ch1_off < 1023)
        ch1_off += 1024/VREF[range1];
    } else if (sw == 7) { // offset -
      if (ch1_off > -1023)
        ch1_off -= 1024/VREF[range1];
    } else if (sw == 11) { // offset reset
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
    if (sw == 3) {        // ON
      fft_mode = true;
    } else if (sw == 7) { // OFF
      fft_mode = false;
    }
    break;
  case 3: // Frequency and Duty display
    if (sw == 3) {        // ON
      info_mode |= 1;
    } else if (sw == 7) { // OFF
      info_mode &= ~1;
    }
    break;
  case 4: // Voltage display
    if (sw == 3) {        // ON
      info_mode |= 2;
    } else if (sw == 7) { // OFF
      info_mode &= ~2;
    }
    break;
  case 5: // PWM
    if (sw == 3) {        // +
      update_frq(0);
      pulse_start();
      pulse_mode = true;
    } else if (sw == 7) { // -
      pulse_close();
      pulse_mode = false;
    }
    break;
  case 6: // PWM Duty ratio
    diff = 1;
    if (sw == lastsw) {
      if (millis() - vtime > 5000) diff = 8;
    }
    if (sw == 3) {        // +
      if (pulse_mode) {
        if ((256 - duty) > diff) duty += diff;
      } else {
        pulse_start();
      }
      update_frq(0);
      pulse_mode = true;
    } else if (sw == 7) { // -
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
    if (sw == 3) {        // +
      if (pulse_mode)
        update_frq(-diff);
      else {
        update_frq(0);
        pulse_start();
      }
      pulse_mode = true;
    } else if (sw == 7) { // -
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
    if (sw == 3) {        // +
      fcount_close();
      dds_setup();
      dds_mode = true;
    } else if (sw == 7) { // -
      dds_close();  // dds_mode = false;
    }
    break;
  case 3: // WAVE
    if (sw == 3) {        // +
      rotate_wave(true);
    } else if (sw == 7) { // -
      rotate_wave(false);
    }
    break;
  case 4: // FREQ
    diff = sw_accel(sw);
    if (sw == 3) {        // +
      update_ifrq(diff);
    } else if (sw == 7) { // -
      update_ifrq(-diff);
    }
    break;
  case 5: // Frequency Counter
    if (sw == 3 && rate <= RATE_MAX) {  // on
      dds_close();  // dds_mode = false;
      pulse_close();
      pulse_mode = false; fcount_mode = true;
      FreqCount.begin(1000);
    } else if (sw == 7) {               // off
      fcount_close();
    }
    break;
  }
  menu_updown(sw);
}

void menu_updown(byte sw) {
  if (sw == 10) {       // MENU down SW
    increment_item();
  } else if (sw == 0) { // Menu up SW
    decrement_item();
  }
}

void increment_item() {
  ++item;
  if (item > ITEM_MAX) item = 0;
  if (menu == 0 && item == 3) item = 4;
  if (item < 16 || item > 18) fft_mode = false; // exit FFT mode
  menu = item >> 3;
}

void decrement_item() {
  if (item > 0) --item;
  else item = ITEM_MAX;
  if (menu == 0 && item == 3) item = 2;
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

void DrawGrid() {
  int disp_leng;
  if (full_screen) disp_leng = SAMPLES;
  else disp_leng = DISPLNG;
  for (int x=0; x<=disp_leng; x += 2) { // Horizontal Line
    for (int y=0; y<=60; y += 10) {
      display.drawPixel(x, y, WHITE);
      CheckSW();
    }
  }
  for (int x=0; x<=disp_leng; x += 10 ) { // Vertical Line
    for (int y=0; y<=60; y += 2) {
      display.drawPixel(x, y, WHITE);
      CheckSW();
    }
  }
}

void DrawText() {
  display.fillRect(DISPLNG+1,0,27,64, BLACK);  // clear text area that will be drawn below 

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
    display.print(trig_edge == TRIG_E_UP ? char(0x18) : char(0x19)); 
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
      if (pulse_mode && (item > 20 && item < 24))
        disp_pulse_frq();
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
  if (info_mode) {
    dataAnalize();
    if (info_mode & 1)
      measure_frequency();
    if (info_mode & 2)
      measure_voltage();
  }
  if (!full_screen && !fft_mode)
    display.drawFastHLine(DISPLNG, 60 - trig_lv, 3, WHITE); // draw trig_lv tic
}

unsigned long fcount = 0;
const double freq_ratio = 20000.0 / 19987.0;

void fcount_disp() {
  if (!fcount_mode) return;
  if (FreqCount.available()) {
    fcount = FreqCount.read();
    fcount = fcount * freq_ratio; // compensate the ceramic osc
  }
  display.setTextColor(WHITE, BLACK); display.setCursor(74, 48);
  display.print(fcount); display.print(F("Hz"));
}

void fcount_close() {
  if (!fcount_mode) return;
  fcount_mode = false;
  FreqCount.end();
}

void display_range(byte rng) {
  char str[5];
  display.print(strcpy_P(str, (char*)pgm_read_word(&(Ranges[rng]))));
}

void display_rate(void) {
  char str[5];
  display.print(strcpy_P(str, (char*)pgm_read_word(&(Rates[rate]))));
}

void display_mode(byte chmode) {
  char str[5];
  display.print(strcpy_P(str, (char*)pgm_read_word(&(Modes[chmode])))); 
}

void display_trig_mode(void) {
  char str[5];
  display.print(strcpy_P(str, (char*)pgm_read_word(&(TRIG_Modes[trig_mode])))); 
}

void display_ac(byte pin) {
  if (digitalRead(pin) == LOW) display.print('~');
}

void set_line_color(byte line) {
  if ((item & 0x7) == line) display.setTextColor(BLACK, WHITE);  // highlight
  else display.setTextColor(WHITE, BLACK);            // normal
  display.setCursor(DISPLNG + 3, 8 * line); // locate curser for printing text
}

void DrawGrid(int x) {
  if ((x % 2) == 0)
    for (int y=0; y<=60; y += 10)
      display.drawPixel(x, y, WHITE);
  if ((x % 10) == 0)
    for (int y=0; y<=60; y += 2)
      display.drawPixel(x, y, WHITE);
}

void ClearAndDrawGraph() {
  int disp_leng;
  if (full_screen) disp_leng = SAMPLES-1;
  else disp_leng = DISPLNG-1;
  bool ch1_active = ch1_mode != MODE_OFF && rate > 4 && rate <= RATE_MAX;
#if 0
  for (int x=0; x<DISPLNG; x++) {
    display.drawPixel(x, 60-data[sample+0][x], WHITE);
    display.drawPixel(x, 60-data[sample+1][x], WHITE);
  }
#else
  for (int x=0; x<disp_leng; x++) {
    if (ch0_mode != MODE_OFF)
      display.drawLine(x, 60-data[sample+0][x], x+1, 60-data[sample+0][x+1], WHITE);
    if (ch1_active)
      display.drawLine(x, 60-data[sample+1][x], x+1, 60-data[sample+1][x+1], WHITE);
    CheckSW();
  }
#endif
}

void ClearAndDrawDot(int i) {
#if 0
  for (int x=0; x<DISPLNG; x++) {
    display.drawPixel(i, 60-odat01, BLACK);
    display.drawPixel(i, 60-odat11, BLACK);
    display.drawPixel(i, 60-data[sample+0][i], WHITE);
    display.drawPixel(i, 60-data[sample+1][i], WHITE);
  }
#else
  if (i < 1)
    return;
  if (ch0_mode != MODE_OFF) {
    display.drawLine(i-1, 60-odat00,   i, 60-odat01, BLACK);
    display.drawLine(i-1, 60-data[0][i-1], i, 60-data[0][i], WHITE);
  }
  if (ch1_mode != MODE_OFF) {
    display.drawLine(i-1, 60-odat10,   i, 60-odat11, BLACK);
    display.drawLine(i-1, 60-data[1][i-1], i, 60-data[1][i], WHITE);
  }
#endif
  DrawGrid(i);
}

//void scaleDataArray()
//{
//  byte *pdata;
//  int *idata;
//  long a;
//
//  idata = (int *) data[0];
//  pdata = data[0];
//  for (int i = 0; i < SAMPLES; i++) {
//    a = ((*idata++ + ch0_off) * VREF[range0] + 512) >> 10;
//    if (a > 60) a = 60;
//    else if (a < 0) a = 0;
//    if (ch0_mode == MODE_INV)
//      a = 60 - a;
//    *pdata++ = (byte) a;
//  }
//}

void scaleDataArray(byte ad_ch)
{
  byte *pdata, ch_mode, range;
  short ch_off;
  int *idata;
  long a;

  if (ad_ch == ad_ch1) {
    ch_off = ch1_off;
    ch_mode = ch1_mode;
    range = range1;
  } else {
    ch_off = ch0_off;
    ch_mode = ch0_mode;
    range = range0;
  }
  idata = (int *) data[0];
  pdata = data[0];
  for (int i = 0; i < SAMPLES; i++) {
    a = ((*idata++ + ch_off) * VREF[range] + 512) >> 10;
    if (a > 60) a = 60;
    else if (a < 0) a = 0;
    if (ch_mode == MODE_INV)
      a = 60 - a;
    *pdata++ = (byte) a;
  }
}

byte adRead(byte ch, byte mode, int off)
{
  long a = analogRead(ch);
  a = ((a+off)*VREF[ch == ad_ch0 ? range0 : range1]+512) >> 10;
  if (a > 60) a = 60;
  else if (a < 0) a = 0;
  if (mode == MODE_INV)
    return 60 - a;
  return a;
}

int advalue(int value, long vref, byte mode, int offs) {
  if (mode == MODE_INV)
    value = 60 - value;
//  return (((long)value << 10) - 512L) / vref - offs;
  return ((long)value << 10) / vref - offs;
}

void loop() {
  int oad, ad;
  unsigned long auto_time;
  int trigger_ad;

  timeExec = 100;
  if (rate <= RATE_MAX) {
    if (trig_ch == ad_ch0) {
      trigger_ad = advalue(trig_lv, VREF[range0], ch0_mode, ch0_off);
    } else {
      trigger_ad = advalue(trig_lv, VREF[range1], ch1_mode, ch1_off);
    }
    ADCSRA = (ADCSRA & 0x07)| _BV(ADEN) | _BV(ADIF);  // Auto Trigger disable
    ADCSRB &= 0xf8;   // Auto Trigger source free run
    auto_time = pow(10, rate / 3);
    if (rate < 7)
      auto_time *= 10;
    if (trig_mode != TRIG_SCAN) {
      unsigned long st = millis();
      oad = analogRead(trig_ch);
      for (;;) {
        ad = analogRead(trig_ch);

        if (trig_edge == TRIG_E_UP) {
          if (ad > trigger_ad && trigger_ad > oad)
            break;
        } else {
          if (ad < trigger_ad && trigger_ad < oad)
            break;
        }
        oad = ad;

        if (rate > 14)
          CheckSW();      // no need for fast sampling
        if (trig_mode == TRIG_SCAN)
          break;
        if (trig_mode == TRIG_AUTO && (millis() - st) > auto_time)
          break; 
      }
    }
  }
  
  // sample and draw depending on the sampling rate
  if (rate <= 14 && Start) {

    if (rate == 0) { // full speed, channel 0 only 3.25us sampling
      sample_33us();
    } else if (rate == 1) { // channel 0 only 5.1us sampling
      sample_51us();
    } else if (rate == 2) { // channel 0 only 10us sampling
      sample_100us();
    } else if (rate == 3) { // channel 0 only 20us sampling
      sample_200us(20, ad_ch0);
    } else if (rate == 4) { // channel 0 only 49us sampling
      sample_200us(50, ad_ch0);
    } else if (rate == 5) { // full speed, dual channel 64us sampling
      sample_dual_640us();
    } else if (rate >= 6 && rate <= 8) {  // dual channel 80us, 100us, 200us sampling
      sample_dual_us(pgm_read_dword(&HREF[rate]) / 10);
    } else {                // dual channel .5ms, 1ms, 2ms, 5ms, 10ms, 20ms sampling
      sample_dual_ms(pgm_read_dword(&HREF[rate]) / 10);
    }
    draw_screen();
  } else if (rate <= RATE_MAX && Start) { // 50ms - 1000ms sampling
    timeExec = 5000;
    ADCSRA = ADCSRA | 0x07;  // dividing ratio = arduino default
    static const unsigned long r_[] PROGMEM = {50000, 100000, 200000, 500000, 1000000};
    unsigned long r;
    int disp_leng;
    if (full_screen) disp_leng = SAMPLES;
    else disp_leng = DISPLNG;
//    unsigned long st0 = millis();
    unsigned long st = micros();
    for (int i=0; i<disp_leng; i ++) {
      r = pgm_read_dword(r_ + rate - 15);  // rate may be changed in loop
      while((st - micros())<r) {
        CheckSW();
        if (rate<15)
          break;
      }
      if (rate<15) { // sampling rate has been changed
        display.clearDisplay();
        break;
      }
      st += r;
      if (st - micros()>r)
          st = micros(); // sampling rate has been changed to shorter interval
      if (!Start) {
         i --;
         continue;
      }
      odat00 = odat01;      // save next previous data ch0
      odat10 = odat11;      // save next previous data ch1
      odat01 = data[0][i];  // save previous data ch0
      odat11 = data[1][i];  // save previous data ch1
      if (ch0_mode != MODE_OFF)
        data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
      if (ch1_mode != MODE_OFF)
        data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
      ClearAndDrawDot(i);     
      display.display();  // 42ms
    }
    // Serial.println(millis()-st0);
    DrawGrid();
    if (!full_screen) DrawText();
  } else if (Start) { // Equivalent Time sampling
    extern byte oscspeed;
    oscspeed = rate - RATE_MAX - 1; // 20...26 -> 0...6
    trigger_level(constrain(advalue(trig_lv, VREF[range0], ch0_mode, ch0_off)/4,0,255)); // PWM triger level for ET
    modeequiv();
    draw_screen();
  } else {
    DrawText();
  }
  if (trig_mode == TRIG_ONE)
    Start = false;
  CheckSW();
  saveEEPROM();                         // save settings to EEPROM if necessary
}

void draw_screen() {
  display.clearDisplay();
  if (fft_mode) {
    DrawText();
    plotFFT();
  } else {      // channel 0 only
    ClearAndDrawGraph();
    DrawGrid();
    if (!full_screen) DrawText();
  }
  display.display();
}

#define textINFO 54
void measure_frequency() {
  int x1, x2;
  freqDuty();
  display.setTextColor(WHITE, BLACK);
  display.setCursor(textINFO, txtLINE0);
  if (waveFreq < 999.5)
    display.print(waveFreq);
  else if (waveFreq < 999999.5)
    display.print(waveFreq, 0);
  else {
    display.print(waveFreq/1000.0, 0);
    display.print('k');
  }
  display.print(F("Hz"));
  if (fft_mode) return;
  display.setCursor(textINFO + 12, txtLINE1);
  display.print(waveDuty);  display.print('%');
}

void measure_voltage() {
  int x, dave, dmax, dmin;
  if (fft_mode) return;
  if (ch0_mode == MODE_INV) {
    dave = 60 * 10 - dataAve;
    dmax = dataMin;
    dmin = dataMax;
  } else {
    dave = dataAve;
    dmax = dataMax;
    dmin = dataMin;
  }
  float vavr = VRF * (((dave * 102.4) - 512.0) / VREF[range0] - ch0_off) / 1023.0;
  float vmax = VRF * advalue(dmax, VREF[range0], ch0_mode, ch0_off) / 1023.0;
  float vmin = VRF * advalue(dmin, VREF[range0], ch0_mode, ch0_off) / 1023.0;
  display.setCursor(textINFO, txtLINE2);
  display.print(F("max"));  display.print(vmax); if (vmax >= 0.0) display.print('V');
  display.setCursor(textINFO, txtLINE3);
  display.print(F("avr"));  display.print(vavr); if (vavr >= 0.0) display.print('V');
  display.setCursor(textINFO, txtLINE4);
  display.print(F("min"));  display.print(vmin); if (vmin >= 0.0) display.print('V');
}

void sample_dual_640us() { // dual channel full speed. 64us sampling (0x4)
  if (ch0_mode != MODE_OFF && ch1_mode == MODE_OFF) {
    sample_200us(64, ad_ch0);
  } else if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    sample_200us(64, ad_ch1);
    memcpy(data[1], data[0], SAMPLES);
    memset(data[0], 0, SAMPLES);
  } else {
    byte *p0, *p1;
    ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
    p0 = data[0]; p1 = data[1];
    for (int i=0; i<SAMPLES; i ++) {
      *p0++ = adRead(ad_ch0, ch0_mode, ch0_off);
      *p1++ = adRead(ad_ch1, ch1_mode, ch1_off);
    }
  }
}

void sample_dual_us(unsigned int r) { // dual channel. r > 67 (0x4)
  byte *p0, *p1;
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  p0 = data[0]; p1 = data[1];
  if (ch0_mode != MODE_OFF && ch1_mode == MODE_OFF) {
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      *p0++ = adRead(ad_ch0, ch0_mode, ch0_off);
      st += r;
    }
    memset(data[1], 0, SAMPLES);
  } else if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      *p1++ = adRead(ad_ch1, ch1_mode, ch1_off);
      st += r;
    }
    memset(data[0], 0, SAMPLES);
  } else {
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      *p0++ = adRead(ad_ch0, ch0_mode, ch0_off);
      *p1++ = adRead(ad_ch1, ch1_mode, ch1_off);
      st += r;
    }
  }
}

void sample_dual_ms(unsigned int r) { // dual channel. r > 500 (0x7)
// .5ms, 1ms or 2ms sampling
  ADCSRA = ADCSRA | 0x07;  // dividing ratio = arduino default
  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    while(micros() - st < r) ;
    st += r;
    if (ch0_mode != MODE_OFF)
      data[0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
    if (ch1_mode != MODE_OFF)
      data[1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
  }
  if (ch0_mode == MODE_OFF) memset(data[0], 0, SAMPLES);
//  if (ch1_mode == MODE_OFF) memset(data[1], 0, SAMPLES);
}

void sample_200us(unsigned int r, byte ad_ch) { // analogRead() with timing, channel 0 or 1. 200us/div 50ksps
  int *idata;
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  idata = (int *) data[0];
  unsigned long st = micros();
  for (byte i=0; i<SAMPLES; i ++) {
    while(micros() - st < r) ;
    *idata++ = analogRead(ad_ch);
    st += r;
  }
  scaleDataArray(ad_ch);
}

void sample_100us() { // register direct with timing, channel 0 only. 100us/div 100ksps
  byte *pdata;
  byte r = 10;
  ADMUX = (ADMUX & 0xf8) + ad_ch0;
  ADCSRA = (ADCSRA & 0xf8) | 0x02;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  pdata = data[0];
  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    while(micros() - st < r) ;
    ADCSRA |= 0x40;  // start the conversion(1 << ADSC)
    st += r;
    while (ADCSRA & 0x40); // ADSC is cleared when the conversion finishes
    *pdata++ = ADCL;            // must read adch low byte first
    *pdata++ = ADCH;            // read adch high byte
  }
  scaleDataArray(ad_ch0);
}

void sample_51us() {  // full speed, channel 0 only. 51us/div 189ksps
  byte *pdata;
  ADMUX = (ADMUX & 0xf8) + ad_ch0;
  ADCSRA = (ADCSRA & 0xf8) | 0x02;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  pdata = data[0];
//  unsigned long st0 = micros();
  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    ADCSRA |= 0x40;         // start the conversion(1 << ADSC)
    while (ADCSRA & 0x40);  // ADSC is cleared when the conversion finishes
    *pdata++ = ADCL;        // must read adch low byte first
    *pdata++ = ADCH;        // read adch high byte
  }
//  Serial.println(micros()-st0);
  scaleDataArray(ad_ch0);
}

void sample_33us() {  // full speed, channel 0 only ADC free run. 32.5us/div 308ksps
  byte *pdata;
  ADMUX = (ADMUX & 0xf8) + ad_ch0;
  ADCSRA = (ADCSRA & 0xf8)| 0x62; // Auto Trigger Enable. dividing ratio = 4(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  pdata = data[0];
  for (int i=0; i<SAMPLES; i ++) {
    while ((ADCSRA&0x10)==0) ;  // polling until adif==1
    ADCSRA |= 0x10;             // clear adif
    *pdata++ = ADCL;            // must read adch low byte first
    *pdata++ = ADCH;            // read adch high byte
  }
  ADCSRA = ADCSRA & 0x9f;       // stop ADC free run ADSC=0 ADATE=0
  scaleDataArray(ad_ch0);
}

void plotFFT() {
  char *im, *re;
  int ylim = 56;

  re = data[0];
  im = data[1];  // use ch1 buffer for imaginary data
  for (int i = 0; i < FFT_N; i++) {
    int d = *re << 2;   // 0 <= data <= 60 so multiply 4
    d = d - 120;        // center value should be 120
    *re++ = constrain(d, -128, 127);
    *im++ = 0;
  }
  re = data[0];
  im = data[1];  // use ch1 buffer for imaginary data
  fix_fft(re, im, 7, 0); // full scale 2^7=128, FFT mode
  for (int i = 1; i < FFT_N/2; i++) {
    int dat = sqrt(re[i] * re[i] + im[i] * im[i]);
    dat = constrain(dat, 0, ylim);
    display.drawFastVLine(i * 2, ylim - dat, dat, WHITE);
  }
  draw_scale();
}

void draw_scale() {
  int ylim = 56;
  float fhref, nyquist;
  display.setTextColor(WHITE);
  display.setCursor(0, ylim); display.print(F("0Hz")); 
  if (rate > RATE_MAX) {
    fhref = ethref();
  } else {
    fhref = (float)pgm_read_dword(&HREF[rate]);
  }
  nyquist = 5.0e6 / fhref; // Nyquist frequency
  if (nyquist > 999.0) {
    nyquist = nyquist / 1000.0;
    if (nyquist > 99.5) {
      display.setCursor(58, ylim); display.print(nyquist/2,0);display.print('k');
      display.setCursor(104, ylim); display.print(nyquist,0);
    } else if (nyquist > 9.95) {
      display.setCursor(58, ylim); display.print(nyquist/2,0);display.print('k');
      display.setCursor(110, ylim); display.print(nyquist,0);
    } else {
      display.setCursor(52, ylim); display.print(nyquist/2,1);display.print('k');
      display.setCursor(104, ylim); display.print(nyquist,1);
    }
    display.print('k');
  } else {
    display.setCursor(58, ylim); display.print(nyquist/2,0);
    display.setCursor(110, ylim); display.print(nyquist,0);
  }
}

#define EEPROM_START 64
void saveEEPROM() {                   // Save the setting value in EEPROM after waiting a while after the button operation.
  int p = EEPROM_START;
  if (saveTimer > 0) {                // If the timer value is positive
    saveTimer = saveTimer - timeExec; // Timer subtraction
    if (saveTimer <= 0) {             // if time up
      EEPROM.write(p++, range0);      // save current status to EEPROM
      EEPROM.write(p++, ch0_mode);
      EEPROM.write(p++, lowByte(ch0_off));  // save as Little endian
      EEPROM.write(p++, highByte(ch0_off));
      EEPROM.write(p++, range1);
      EEPROM.write(p++, ch1_mode);
      EEPROM.write(p++, lowByte(ch1_off));  // save as Little endian
      EEPROM.write(p++, highByte(ch1_off));
      EEPROM.write(p++, rate);
      EEPROM.write(p++, trig_mode);
      EEPROM.write(p++, trig_lv);
      EEPROM.write(p++, trig_edge);
      EEPROM.write(p++, trig_ch);
      EEPROM.write(p++, fft_mode);
      EEPROM.write(p++, info_mode);
      EEPROM.write(p++, item);
      EEPROM.write(p++, pulse_mode);
      EEPROM.write(p++, duty);
      EEPROM.write(p++, p_range);
      EEPROM.write(p++, lowByte(count));  // save as Little endian
      EEPROM.write(p++, highByte(count));
      EEPROM.write(p++, dds_mode);
      EEPROM.write(p++, wave_id);
      EEPROM.write(p++, ifreq & 0xff);
      EEPROM.write(p++, (ifreq >> 8) & 0xff);
      EEPROM.write(p++, (ifreq >> 16) & 0xff);
      EEPROM.write(p++, (ifreq >> 24) & 0xff);
    }
  }
}

void set_default() {
  range0 = RANGE_MIN;
  ch0_mode = MODE_ON;
  ch0_off = -200;
  range1 = RANGE_MIN;
  ch1_mode = MODE_ON;
  ch1_off = 400;
  rate = 5;
  trig_mode = TRIG_AUTO;
  trig_lv = 15;
  trig_edge = TRIG_E_UP;
  trig_ch = ad_ch0;
  fft_mode = false;
  info_mode = 1;  // display frequency and duty.  Voltage display is off
  item = 0;       // menu item
  pulse_mode = false;
  duty = 128;     // PWM 50%
  p_range = 0;    // PWM range
  count = 15999;  // PWM 1kHz
  dds_mode = false;
  wave_id = 0;    // sine wave
  ifreq = 12255;  // 122.55Hz
}

extern const byte wave_num;

void loadEEPROM() { // Read setting values from EEPROM (abnormal values will be corrected to default)
  int p = EEPROM_START, error = 0;

  range0 = EEPROM.read(p++);                // range0
  if ((range0 < RANGE_MIN) || (range0 > RANGE_MAX)) ++error;
  ch0_mode = EEPROM.read(p++);              // ch0_mode
  if (ch0_mode > 2) ++error;
  *((byte *)&ch0_off) = EEPROM.read(p++);     // ch0_off low
  *((byte *)&ch0_off + 1) = EEPROM.read(p++); // ch0_off high
  if ((ch0_off < -1024) || (ch0_off > 1023)) ++error;

  range1 = EEPROM.read(p++);                // range1
  if ((range1 < RANGE_MIN) || (range1 > RANGE_MAX)) ++error;
  ch1_mode = EEPROM.read(p++);              // ch1_mode
  if (ch1_mode > 2) ++error;
  *((byte *)&ch1_off) = EEPROM.read(p++);     // ch1_off low
  *((byte *)&ch1_off + 1) = EEPROM.read(p++); // ch1_off high
  if ((ch1_off < -1024) || (ch1_off > 1023)) ++error;

  rate = EEPROM.read(p++);                  // rate
  if ((rate < RATE_MIN) || (rate >= RATE_NUM)) ++error;
//  if (ch0_mode == MODE_OFF && rate < 5) ++error;  // correct ch0_mode
  trig_mode = EEPROM.read(p++);             // trig_mode
  if (trig_mode > TRIG_SCAN) ++error;
  trig_lv = EEPROM.read(p++);               // trig_lv
  if (trig_lv > 60) ++error;
  trig_edge = EEPROM.read(p++);             // trig_edge
  if (trig_edge > 1) ++error;
  trig_ch = EEPROM.read(p++);               // trig_ch
  if (trig_ch > 7) ++error;
  fft_mode = EEPROM.read(p++);              // fft_mode
  info_mode = EEPROM.read(p++);             // info_mode
  if (info_mode > 3) ++error;
  item = EEPROM.read(p++);                  // item
  if (item > ITEM_MAX) ++error;
  pulse_mode = EEPROM.read(p++);            // pulse_mode
  duty = EEPROM.read(p++);                  // duty
  p_range = EEPROM.read(p++);               // p_range
  if (p_range > 4) ++error;
  *((byte *)&count) = EEPROM.read(p++);     // count low
  *((byte *)&count + 1) = EEPROM.read(p++); // count high
  dds_mode = EEPROM.read(p++);              // DDS wave id
  wave_id = EEPROM.read(p++);               // DDS wave id
  if (wave_id >= wave_num) ++error;
  *((byte *)&ifreq) = EEPROM.read(p++);     // ifreq low
  *((byte *)&ifreq + 1) = EEPROM.read(p++); // ifreq
  *((byte *)&ifreq + 2) = EEPROM.read(p++); // ifreq
  *((byte *)&ifreq + 3) = EEPROM.read(p++); // ifreq high
  if (ifreq > 999999L) ++error;
  if (error > 0)
    set_default();
}

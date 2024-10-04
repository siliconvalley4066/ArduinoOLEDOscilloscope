/*
 * Arduino Oscilloscope using a 128x64 OLED Version 1.33
 * The max realtime sampling rates are 17.2ksps with 2 channels and 307ksps with a channel.
 * The max equivalent time sampling rates is 16Msps with single channel.
 * + Pulse Generator
 * + PWM DDS Function Generator (8 waveforms)
 * + Frequency Counter
 * Copyright (c) 2023, Siliconvalley4066
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
#define txtLINE5   40
#define txtLINE6   48
#define txtLINE7   56

float waveFreq;                // frequency (Hz)
float waveDuty;                // duty ratio (%)
int dataMin;                   // buffer minimum value (smallest=0)
int dataMax;                   //        maximum value (largest=1023)
int dataAve;                   // 10 x average value (use 10x value to keep accuracy. so, max=10230)
int saveTimer;                 // remaining time for saving EEPROM
int timeExec;                  // approx. execution time of current range setting (ms)
extern byte duty;
extern byte p_range;
extern unsigned short count;
extern long ifreq;
extern byte wave_id;

const int LCD_WIDTH = 128;
const int LCD_HEIGHT = 64;
const int LCD_YMAX = 60;
const int SAMPLES = 128;
const int DISPLNG = 100;
const int DOTS_DIV = 10;
const int ad_ch0 = 0;                   // Analog 0 pin for channel 0
const int ad_ch1 = 1;                   // Analog 1 pin for channel 1
const long VREF[] = {49, 98, 244, 488, 976}; // reference voltage 5.0V ->  50 :   1V/div range (100mV/dot)
                                        //                        -> 100 : 0.5V/div
                                        //                        -> 250 : 0.2V/div
                                        //                        -> 500 : 100mV/div
                                        //                       -> 1000 :  50mV/div
                                        // 5.0V / attn * DOTS_DIV / vdiv
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
#define RATE_DUAL 5
#define RATE_SLOW 14
#define RATE_ROLL 15
#define ITEM_MAX 29
const char RN[RATE_NUM][5] PROGMEM = {"33us", "49us", "100u", "200u", "500u", "600u", "800u", " 1ms", " 2ms", " 5ms", "10ms", "20ms", "50ms", "0.1s", "0.2s", "0.5s", " 1s ", " 2s ", " 5s ", " 10s",
                                      "0.6u", "1.3u", "3.1u", "6.3u", "13us", "31us", "63us"};
const char * const Rates[RATE_NUM] PROGMEM = {RN[0], RN[1], RN[2], RN[3], RN[4], RN[5], RN[6], RN[7], RN[8], RN[9], RN[10], RN[11], RN[12], RN[13], RN[14], RN[15], RN[16], RN[17], RN[18], RN[19],
                                              RN[20], RN[21], RN[22], RN[23], RN[24], RN[25], RN[26]};
const unsigned long HREF[] PROGMEM = {33, 49, 100, 200, 500, 600, 800, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000, 5000000, 10000000};
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
short ch0_off = 0, ch1_off = 400;
byte data[2][SAMPLES];                  // keep the number of channels buffer
byte odat00, odat01, odat10, odat11;    // old data buffer for erase
byte sample=0;                          // index for double buffer
bool fft_mode = false, pulse_mode = false, dds_mode = false, fcount_mode = false;
bool full_screen = false;
byte info_mode = 3; // Text information display mode
int trigger_ad;

#define LEFTPIN   12  // LEFT
#define RIGHTPIN  9   // RIGHT
#define UPPIN     7   // UP
#define DOWNPIN   8   // DOWN
#define CH0DCSW   2   // DC/AC switch ch0
#define CH1DCSW   4   // DC/AC switch ch1

#define BGCOLOR   BLACK
#define GRIDCOLOR WHITE
#define CH1COLOR  WHITE
#define CH2COLOR  WHITE
#define FRMCOLOR  WHITE
#define TXTCOLOR  WHITE
#define TRGCOLOR  WHITE
#define HIGHCOLOR WHITE

void setup(){
  pinMode(CH0DCSW, INPUT_PULLUP);   // CH1 DC/AC
//  pinMode(3, OUTPUT);               // Trigger level PWM out
  pinMode(CH1DCSW, INPUT_PULLUP);   // CH2 DC/AC
//  pinMode(5, INPUT);                // Frequency Counter in
//  pinMode(6, INPUT);                // Trigger level AC input
  pinMode(UPPIN, INPUT_PULLUP);     // up
  pinMode(DOWNPIN, INPUT_PULLUP);   // down
  pinMode(RIGHTPIN, INPUT_PULLUP);  // right
  pinMode(10, OUTPUT);                // PWM out
  pinMode(11, OUTPUT);                // DDS out
  pinMode(LEFTPIN, INPUT_PULLUP);   // left
  DIDR0 = _BV(ADC1D) | _BV(ADC0D);  // disable digital input buffer of A0-A1
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

void DrawGrid() {
  int disp_leng;
  if (full_screen) disp_leng = SAMPLES;
  else disp_leng = DISPLNG;
  for (int x=0; x<=disp_leng; x += 2) { // Horizontal Line
    for (int y=LCD_YMAX; y>=0; y -= DOTS_DIV) {
      display.drawPixel(x, y, GRIDCOLOR);
      CheckSW();
    }
  }
  for (int x=0; x<=disp_leng; x += DOTS_DIV ) { // Vertical Line
    for (int y=LCD_YMAX; y>=0; y -= 2) {
      display.drawPixel(x, y, GRIDCOLOR);
      CheckSW();
    }
  }
}

unsigned long fcount = 0;
const double freq_ratio = 20000.0 / 19987.0;

void fcount_disp() {
  if (!fcount_mode) return;
  if (FreqCount.available()) {
    fcount = FreqCount.read();
    fcount = fcount * freq_ratio; // compensate the ceramic osc
  }
  display.setTextColor(TXTCOLOR, BGCOLOR); display.setCursor(74, 48);
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
  if ((item & 0x7) == line) display.setTextColor(BGCOLOR, TXTCOLOR);  // highlight
  else display.setTextColor(TXTCOLOR, BGCOLOR);           // normal
  display.setCursor(DISPLNG + 3, 8 * line);  // locate curser for printing text
}

void DrawGrid(int x) {
  if ((x % 2) == 0)
    for (int y=0; y<=LCD_YMAX; y += 10)
      display.drawPixel(x, y, GRIDCOLOR);
  if ((x % 10) == 0)
    for (int y=0; y<=LCD_YMAX; y += 2)
      display.drawPixel(x, y, GRIDCOLOR);
}

void ClearAndDrawGraph() {
  int disp_leng;
  if (full_screen) disp_leng = SAMPLES-1;
  else disp_leng = DISPLNG-1;
  bool ch1_active = ch1_mode != MODE_OFF && rate > 4 && rate <= RATE_MAX;
#if 0
  for (int x=0; x<disp_leng; x++) {
    display.drawPixel(x, LCD_YMAX-data[sample+0][x], CH1COLOR);
    display.drawPixel(x, LCD_YMAX-data[sample+1][x], CH2COLOR);
  }
#else
  for (int x=0; x<disp_leng; x++) {
    if (ch0_mode != MODE_OFF) {
      display.drawLine(x, LCD_YMAX-data[sample+0][x], x+1, LCD_YMAX-data[sample+0][x+1], CH1COLOR);
    }
    if (ch1_active) {
      display.drawLine(x, LCD_YMAX-data[sample+1][x], x+1, LCD_YMAX-data[sample+1][x+1], CH2COLOR);
    }
    CheckSW();
  }
#endif
}

void ClearAndDrawDot(int i) {
  DrawGrid(i);
#if 0
  for (int x=0; x<DISPLNG; x++) {
    display.drawPixel(i, LCD_YMAX-odat01, BGCOLOR);
    display.drawPixel(i, LCD_YMAX-odat11, BGCOLOR);
    display.drawPixel(i, LCD_YMAX-data[sample+0][i], CH1COLOR);
    display.drawPixel(i, LCD_YMAX-data[sample+1][i], CH2COLOR);
  }
#else
  if (i < 1) {
    return;
  }
  if (ch0_mode != MODE_OFF) {
    display.drawLine(i-1, LCD_YMAX-odat00,   i, LCD_YMAX-odat01, BGCOLOR);
    display.drawLine(i-1, LCD_YMAX-data[0][i-1], i, LCD_YMAX-data[0][i], CH1COLOR);
  }
  if (ch1_mode != MODE_OFF) {
    display.drawLine(i-1, LCD_YMAX-odat10,   i, LCD_YMAX-odat11, BGCOLOR);
    display.drawLine(i-1, LCD_YMAX-data[1][i-1], i, LCD_YMAX-data[1][i], CH2COLOR);
  }
#endif
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
//    if (a > LCD_YMAX) a = LCD_YMAX;
//    else if (a < 0) a = 0;
//    if (ch0_mode == MODE_INV)
//      a = LCD_YMAX - a;
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
  idata = (int *) data[sample+ad_ch];
  pdata = data[sample+ad_ch];
  for (int i = 0; i < SAMPLES; i++) {
    a = ((*idata++ + ch_off) * VREF[range] + 512) >> 10;
    if (a > LCD_YMAX) a = LCD_YMAX;
    else if (a < 0) a = 0;
    if (ch_mode == MODE_INV)
      a = LCD_YMAX - a;
    *pdata++ = (byte) a;
  }
}

byte adRead(byte ch, byte mode, int off)
{
  long a = analogRead(ch);
  a = ((a+off)*VREF[ch == ad_ch0 ? range0 : range1]+512) >> 10;
  if (a > LCD_YMAX) a = LCD_YMAX;
  else if (a < 0) a = 0;
  if (mode == MODE_INV)
    return LCD_YMAX - a;
  return a;
}

int advalue(int value, long vref, byte mode, int offs) {
  if (mode == MODE_INV)
    value = LCD_YMAX - value;
  return ((long)value << 10) / vref - offs;
}

void set_trigger_ad() {
  if (trig_ch == ad_ch0) {
    trigger_ad = advalue(trig_lv, VREF[range0], ch0_mode, ch0_off);
  } else {
    trigger_ad = advalue(trig_lv, VREF[range1], ch1_mode, ch1_off);
  }
}

void loop() {
  int oad, ad;
  unsigned long auto_time;

  timeExec = 100;
  if (rate <= RATE_MAX) {
    set_trigger_ad();
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

        if (rate > RATE_SLOW)
          CheckSW();      // no need for fast sampling
        if (trig_mode == TRIG_SCAN)
          break;
        if (trig_mode == TRIG_AUTO && (millis() - st) > auto_time)
          break; 
      }
    }
  }

  // sample and draw depending on the sampling rate
  if (rate < RATE_ROLL && Start) {

    if (rate == 0) { // full speed, channel 0 only 3.25us sampling
      sample_33us();
    } else if (rate == 1) { // channel 0 only 4.9us sampling
      sample_49us();
    } else if (rate == 2) { // channel 0 only 10us sampling
      sample_100us(10, ad_ch0);
    } else if (rate == 3) { // channel 0 only 20us sampling
      sample_100us(20, ad_ch0);
    } else if (rate == 4) { // channel 0 only 50us sampling
      sample_100us(50, ad_ch0);
    } else if (rate == 5) { // full speed, dual channel 60us sampling
      sample_dual_600us();
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
      r = pgm_read_dword(r_ + rate - RATE_ROLL);  // rate may be changed in loop
      while((st - micros())<r) {
        CheckSW();
        if (rate < RATE_ROLL)
          break;
      }
      if (rate<RATE_ROLL) { // sampling rate has been changed
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

#define textINFO (DISPLNG-48)
void measure_frequency(int ch) {
  freqDuty(ch);
  display.setCursor(textINFO, txtLINE0);
  float freq = waveFreq;
  if (freq < 999.5)
    display.print(freq);
  else if (freq < 999999.5)
    display.print(freq, 0);
  else {
    display.print(freq/1000.0, 0);
    display.print('k');
  }
  display.print(F("Hz"));
  if (fft_mode) return;
  float duty = waveDuty;
  if (duty > 99.9499) duty = 99.9;
  display.setCursor(textINFO + 12, txtLINE1);
  display.print(duty, 1);  display.print('%');
}

void measure_voltage(int ch) {
  int x, dave, dmax, dmin;
  if (fft_mode) return;
  byte ch_mode = (ch == 0) ? ch0_mode : ch1_mode;
  if (ch_mode == MODE_INV) {
    dave = LCD_YMAX * 10 - dataAve;
    dmax = dataMin;
    dmin = dataMax;
  } else {
    dave = dataAve;
    dmax = dataMax;
    dmin = dataMin;
  }
  short ch_off = (ch == 0) ? ch0_off : ch1_off;
  long vref = (ch == 0) ? VREF[range0] : VREF[range1];
  float vavr = VRF * (((dave * 102.4) - 512.0) / vref - ch_off) / 1023.0;
  float vmax = VRF * advalue(dmax, vref, ch_mode, ch_off) / 1023.0;
  float vmin = VRF * advalue(dmin, vref, ch_mode, ch_off) / 1023.0;
  display.setCursor(textINFO, txtLINE2);
  display.print(F("max"));  display.print(vmax); if (vmax >= 0.0) display.print('V');
  display.setCursor(textINFO, txtLINE3);
  display.print(F("avr"));  display.print(vavr); if (vavr >= 0.0) display.print('V');
  display.setCursor(textINFO, txtLINE4);
  display.print(F("min"));  display.print(vmin); if (vmin >= 0.0) display.print('V');
}

void sample_dual_600us() { // dual channel full speed. 60us sampling (0x4)
  if (ch0_mode != MODE_OFF && ch1_mode == MODE_OFF) {
    sample_100us(60, ad_ch0);
  } else if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    sample_100us(60, ad_ch1);
  } else {
    byte *p0, *p1;
    ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
    p0 = data[sample+0]; p1 = data[sample+1];
    for (int i=0; i<SAMPLES; i ++) {
      *p0++ = adRead(ad_ch0, ch0_mode, ch0_off);
      *p1++ = adRead(ad_ch1, ch1_mode, ch1_off);
    }
  }
}

void sample_dual_us(unsigned int r) { // dual channel. r > 67 (0x4)
  byte *p0, *p1;
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  p0 = data[sample+0]; p1 = data[sample+1];
  if (ch0_mode != MODE_OFF && ch1_mode == MODE_OFF) {
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      *p0++ = adRead(ad_ch0, ch0_mode, ch0_off);
      st += r;
    }
  } else if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      *p1++ = adRead(ad_ch1, ch1_mode, ch1_off);
      st += r;
    }
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
      data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
    if (ch1_mode != MODE_OFF)
      data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
  }
}

void sample_100us(byte r, byte ad_ch) { // register direct with timing, channel 0 or 1. max 100us/div 100ksps
  byte *pdata, dr;
  if (ad_ch == ad_ch0)
    pdata = data[sample+0];
  else
    pdata = data[sample+1];
  ADMUX = (ADMUX & 0xf8) + ad_ch;
  if (r < 11)
    dr = 0x02;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  else
    dr = 0x04;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  ADCSRA = (ADCSRA & 0xf8) | dr;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    while(micros() - st < r) ;
    ADCSRA |= 0x40;  // start the conversion(1 << ADSC)
    st += r;
    while (ADCSRA & 0x40); // ADSC is cleared when the conversion finishes
    *pdata++ = ADCL;            // must read adch low byte first
    *pdata++ = ADCH;            // read adch high byte
  }
  scaleDataArray(ad_ch);
}

void sample_49us() {  // full speed, channel 0 only. 49us/div 204ksps
  byte *pdata;
  ADMUX = (ADMUX & 0xf8) + ad_ch0;
  ADCSRA = (ADCSRA & 0xf8) | 0x02;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  pdata = data[sample+0];
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
  pdata = data[sample+0];
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
  int ylim = LCD_HEIGHT - 8;

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
    display.drawFastVLine(i * 2, ylim - dat, dat, CH1COLOR);
  }
  draw_scale();
}

void draw_scale() {
  int ylim = LCD_HEIGHT - 8;
  float fhref, nyquist;
  display.setTextColor(TXTCOLOR);
  display.setCursor(0, ylim); display.print(F("0Hz")); 
  fhref = freqhref();
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

float freqhref() {
  float fhref;
  if (rate > RATE_MAX) {
    fhref = ethref();
  } else {
    fhref = (float)pgm_read_dword(&HREF[rate]);
  }
  return fhref;
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
  ch0_off = 0;
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
  if (trig_lv > LCD_YMAX) ++error;
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
  dds_mode = EEPROM.read(p++);              // DDS mode
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

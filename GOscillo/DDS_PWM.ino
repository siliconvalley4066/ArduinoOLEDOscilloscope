/*
 * DDS Sine Generator mit ATMEGS 168
 * Timer2 generates the  31250 KHz Clock Interrupt
 *
 * KHM 2009 /  Martin Nawrath
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne
 */

extern const unsigned char sine256[], saw256[], revsaw256[], triangle[], noise[];
extern const unsigned char sinc5[], sinc10[], sine2harmonic[], sine3harmonic[];
extern const unsigned char trapezoid[], chainsaw[];
unsigned char *wp;
const unsigned char * const wavetable[] PROGMEM = {sine256, saw256, revsaw256,
  triangle, noise, sinc5, trapezoid, chainsaw};
const char WnameN[][5] PROGMEM = {"Sine", "Saw", "RSaw", "Tri", "Nois", "Sinc", "Trpz", "Csaw"};
const char * const Wavename[] PROGMEM = {WnameN[0], WnameN[1], WnameN[2], WnameN[3],
  WnameN[4], WnameN[5], WnameN[6], WnameN[7]};
const byte wave_num = (sizeof(wavetable) / sizeof(&sine256));
long ifreq = 12255; // frequency * 100 for 0.01Hz resolution
byte wave_id = 0;

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define DDSPin 11

const double refclk=31372.549;  // =16MHz / 510

// variables used inside interrupt service declared as voilatile
volatile byte icnt;             // var inside interrupt
volatile unsigned long phaccu;  // pahse accumulator
volatile unsigned long tword_m; // dds tuning word m

void dds_setup() {
  pinMode(DDSPin, OUTPUT);      // pin11= PWM  output / frequency output
  Setup_timer2();
// disable interrupts to avoid timing distortion
//  cbi (TIMSK0,TOIE0);             // disable Timer0 !!! delay() is now not available
  sbi (TIMSK2,TOIE2);           // enable Timer2 Interrupt
  tword_m=pow(2,32)*ifreq*0.01/refclk; // calulate DDS new tuning word
  wp = (unsigned char *) pgm_read_word(&wavetable[wave_id]);
}

void dds_close() {
  pinMode(DDSPin, INPUT_PULLUP);  // pin11= PWM output / frequency output
  TCCR2A &= 0b00111100;           // disconnect DDSPin, Mode 0 idle
  TCCR2B &= 0b11111000;           // stop clock of DDS wave generator
//  sbi (TIMSK0,TOIE0);             // enable Timer0 !!! delay() is now available
  cbi (TIMSK2,TOIE2);             // disable Timer2 Interrupt
  dds_mode = false;
}

void dds_set_freq() {
  double dfreq;
  dfreq = (double)ifreq*0.01;     // adjust output frequency
  cbi (TIMSK2,TOIE2);             // disble Timer2 Interrupt
  tword_m=pow(2,32)*dfreq/refclk; // calulate DDS new tuning word
  sbi (TIMSK2,TOIE2);             // enable Timer2 Interrupt 
}

void rotate_wave(bool fwd) {
  if (fwd) {
    wave_id = (wave_id + 1) % wave_num;
  } else {
    if (wave_id > 0) --wave_id;
    else wave_id = wave_num - 1;
  }
  wp = (unsigned char *) pgm_read_word(&wavetable[wave_id]);
}
//******************************************************************
// timer2 setup
// set prscaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
void Setup_timer2() {

// Timer2 Clock Prescaler to : 1
  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);

  // Timer2 PWM Mode set to Phase Correct PWM
  cbi (TCCR2A, COM2A0);  // clear Compare Match
  sbi (TCCR2A, COM2A1);

  sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
}

//******************************************************************
// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : 8 microseconds ( inclusive push and pop)
ISR(TIMER2_OVF_vect) {
  phaccu=phaccu+tword_m;  // soft DDS, phase accu with 32 bits
  icnt=phaccu >> 24;      // use upper 8 bits for phase accu as frequency information
                          // read value fron ROM sine table and send to PWM DAC
  OCR2A = pgm_read_byte(wp + icnt);    
}

void update_ifrq(long diff) {
  long newFreq;
  int fast;
  if (diff != 0) {
    if (abs(diff) > 3) {
      fast = ifreq / 40;
    } else if (abs(diff) > 2) {
      fast = ifreq / 300;
    } else if (abs(diff) > 1) {
      fast = 25;
    } else {
      fast = 1;
    }
    if (fast < 1) fast = 1;
    newFreq = ifreq + fast * diff;
  } else {
    newFreq = ifreq;
  }
  newFreq = constrain(newFreq, 1, 999999);
  if (newFreq != ifreq) {
    ifreq = newFreq;
    dds_set_freq();
  }
}

void disp_dds_freq(void) {
  display.setTextColor(WHITE, BLACK);
  display.setCursor(72, 56);
  display.print((double)ifreq * 0.01, 2); display.print(F("Hz"));
}

void disp_dds_wave(void) {
  char str[5];
  display.print(strcpy_P(str, (char*)pgm_read_word(&(Wavename[wave_id])))); 
}

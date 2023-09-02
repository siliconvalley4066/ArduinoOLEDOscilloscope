#define PWMPin 10

byte duty = 128; // duty ratio
byte p_range = 0;
unsigned int count;
const int range_min[5] PROGMEM = {1, 8192, 8192, 16384, 16384};
const int range_div[5] PROGMEM = {1, 8, 64, 256, 1024};

void pulse_init() {
  int divide;
  p_range = constrain(p_range, 0, 4);
  divide = pgm_read_word(&range_div[p_range]);
//  count = (unsigned int)(16000000.0 / 1000.0 / divide - 1);

// TCCR1A: COM1A1, COM1A0, COM1B1, COM1B0, -, -, WGM11, 1GM10
// OC1A set on compare match, clear at BOTTOM (COM1A = 0b01)
// OC1B clear on compare match, set at BOTTOM (COM1B = 0b10)
// OCR1A Fast PWM Mode, TOP値=OCR1A (WGM11:10 = 0b11)
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); // Fast PWM mode - compare to OCR1A

// TCCR1B: ICNC1, ICES1, -, WGM13, WGM12, CS12, CS11, CS10
// OCR1A Fast PWM Mode (WGM13:12 = 0b11)
// CS12:10 001:ck/1, 010:ck/8, 011:ck/64, 100:ck/256, 101:ck/1024
//  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // 001:ck/1 for example
//  TCCR1B = 0b00011001;  // ck/1
//  TCCR1B = 0b00011010;  // ck/8
//  TCCR1B = 0b00011011;  // ck/64
//  TCCR1B = 0b00011100;  // ck/256
//  TCCR1B = 0b00011101;  // ck/1024
  setCounter(divide);           // set TCCR1B
  TCNT1 = 0x0000;               // initialize TCNT1
  OCR1A = count;                // set pulse period
  OCR1B = (unsigned int)(((long)count * duty) >> 8);
  pinMode(PWMPin, OUTPUT);
}

void update_frq(char diff) {
  int divide, fast;
  long newCount;

  if (abs(diff) > 3) {
    fast = 512;
  } else if (abs(diff) > 2) {
    fast = 128;
  } else if (abs(diff) > 1) {
    fast = 25;
  } else {
    fast = 1;
  }
  newCount = (long)count + fast * diff;

  if (newCount < pgm_read_word(&range_min[p_range])) {
    if (p_range < 1) {
      newCount = 1;
    } else {
      --p_range;
      newCount = 65535;
    }
  } else if (newCount > 65535) {
    if (p_range < 4) {
      ++p_range;
      newCount = pgm_read_word(&range_min[p_range]);
    } else {
      newCount = 65535;
    }
  }
  divide = pgm_read_word(&range_div[p_range]);
  setCounter(divide);
  count = newCount;
  // set TOP value
  OCR1A = count;
  // set Duty ratio
  OCR1B = (unsigned int)(((long)count * duty) >> 8);
}

void disp_pulse_frq(void) {
  float pulse_frq;        // 0.238Hz <= pulse_frq <= 8MHz
  int divide = pgm_read_word(&range_div[p_range]);
  pulse_frq = 16000000.0 / (((long)count + 1) * divide);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(72, 56);
  if (pulse_frq < 10.0) {
    display.print(pulse_frq, 5);
  } else if (pulse_frq < 100.0) {
    display.print(pulse_frq, 4);
  } else if (pulse_frq < 1000.0) {
    display.print(pulse_frq, 3);
  } else if (pulse_frq < 10000.0) {
    display.print(pulse_frq, 2);
  } else if (pulse_frq < 100000.0) {
    display.print(pulse_frq, 1);
  } else {
    display.print(pulse_frq, 0);
  }
  display.print(F("Hz"));
  display.setCursor(72, 48);
//  display.print(duty*100.0/256.0, 1); display.print('%');
  display.print(duty*0.390625, 1); display.print('%');
}

void setCounter(int divide) {
  if (divide == 1) {
    TCCR1B = 0b00011001;    // ck/1 > 244.1Hz
  } else if (divide == 8) {
    TCCR1B = 0b00011010;    // ck/8 > 30.5Hz
  } else if (divide == 64) {
    TCCR1B = 0b00011011;    // ck/64 > 3.81Hz
  } else if (divide == 256) {
    TCCR1B = 0b00011100;    // ck/256 > 0.954Hz
  } else if (divide == 1024) {
    TCCR1B = 0b00011101;    // ck/1024 > 0.238Hz
  } else {
    TCCR1B = 0b00011000;    // no clock
  }
}

void pulse_start() {
  setCounter(pgm_read_word(&range_div[p_range])); // start clock of pulse generator
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); // Fast PWM mode - compare to OCR1A
  pinMode(PWMPin, OUTPUT);
}

void pulse_close() {
  setCounter(0);              // stop clock of pulse generator
  TCCR1A = 0x00;              // disconnect PWMPin, Mode 0 idle
  pinMode(PWMPin, INPUT_PULLUP);
}

void trigger_level(byte tglv) {
  pinMode(3, OUTPUT);

// TCCR2A: COM2A1, COM2A0, COM2B1, COM2B0, -, -, WGM21, WGM20
// OC2A No connection, normal port            (COM2A = 0b00)
// OC2B clear on compare match, set at BOTTOM (COM2B = 0b10)
// OCR2A Fast PWM Mode, TOP=OCR2A (WGM21:20 = 0b11)
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Fast PWM mode - compare to OCR2A
// TCCR2B: FOC2A, FOC2B, -, -, WGM22, CS22, CS21, CS20
// OCR2A Fast PWM Mode (WGM22 = 0b0)
// CS22:20 001:ck/1, 010:ck/8, 011:ck/64, 100:ck/256, 101:ck/1024
//  TCNT2 = 0;            // initialize TCNT2
  TCCR2B = 0b00000001;  // ck/1, TOP=255
  // Duty ratio
  OCR2B = tglv;
}

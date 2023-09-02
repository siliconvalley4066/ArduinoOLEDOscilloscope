// Modified by Siliconvalley4066. July 23, 2021
//
// Kyutech Arduino Scope Prototype  v0.73                     Apr 10, 2019
//    (C) 2012-2019 M.Kurata Kyushu Institute of Technology
//    for Arduinos with a 5V-16MHz ATmega328.
//
//    Pin usage
//    
//    A0  oscilloscope probe ch1
//    A1  oscilloscope probe ch2
//    A2
//    A3
//    A4  I2C SDA
//    A5  I2C SCL
//    A6  reserved
//    A7  reserved
//    
//    D0  uart-rx
//    D1  uart-tx
//    D2  reserve
//    D3  PWM output for trigger level
//    D4  Up button
//    D5  reserve
//    D6  trigger level input
//    D7  reserve
//    D8  Down button
//    D9  Right button
//    D10 Pulse generator output
//    D11 PWM DDS output
//    D12 Left button
//    D13 LED output

byte  oscspeed   = 0;      // 0..6:equiv
byte  oscinput   = 0;      // input signal selection  0:CH1 1:CH2 2:DUAL
word  osctdly    = 800;    // time of delayed trigger  100..30000 usec
byte at;
//static const struct eqdic_s {
//  byte   tkn;
//  byte   tdif;
//} eqdic[] = {
//  {128,  1},   // 16Msample/s  , 0.625us/div
//  {128,  2},   // 8Msample/s   , 1.25us/div
//  {128,  5},   // 3.2Msample/s , 3.125us/div
//  { 64, 10},   // 1.6Msample/s , 6.25us/div
//  { 32, 20},   // 800ksample/s , 12.5us/div
//  { 16, 50},   // 320ksample/s , 31.25us/div
//  {  8,100},   // 160ksample/s , 62.5us/div
//};
const byte tkn[] PROGMEM = {128,128,128,64,32,16,8};
const byte tdif[] PROGMEM = {1,2,5,10,20,50,100};

void modeequiv() {
  byte   realnum, i, dp, admux;
  byte   tokadif, toka, tokanum;
  byte   ch, chnum, adch, adchT;
  word   ui1, waituntil, sinterval;

  int *buf0 = (int *) data[0];
  tokanum   = pgm_read_byte(tkn + oscspeed);
  waituntil = 64000;
  realnum   = SAMPLES / tokanum;
  tokadif   = pgm_read_byte(tdif + oscspeed);
  sinterval = tokanum * tokadif;  // 20us typical

  // ADMUX reg values
  admux = ADMUX & 0xf8;
  switch(oscinput) {
  default:
  case 0x00: adch = admux + 0; chnum = 1;  break; // CH1
  case 0x01: adch = admux + 1; chnum = 1;  break; // CH2
  case 0x02: adch = admux + 0; chnum = 2;         // CH1 Ch2 Dual
    break;
  }
  adchT = admux + 0;  // select CH1 for trigger

  sinterval--;
  at = 0;
  for(toka = 0; toka < tokanum; toka++) {
    dp = toka;
    for(ch = 0; ch < chnum; ch++) {     // for all ch (1 or 2)
      // reset and initialize timer1
      TCCR1B = 0x00; // stop, set normal mode
      TCCR1A = 0x00;
      TIMSK1 = 0x00; // no irq
      ICR1   = 0x0000;
      TCNT1  = 0x0000;
      TIFR1  = 0x27; // clear flags;

      // analog comparator setting
      // The BG is the positive input.
      // The negative input is A0 pin.
      ACSR   = 0x94;  // analog comparator off
      DIDR1  = 0x01;  // disable the digital input func of D6.
      ADMUX  = adchT; // select the negative input
      ADCSRA = 0x04;  // divide by 16
      ADCSRB = 0x40;  // AC multiplexer enable, ADC auto trigger source free run

      // start timer1 with pre=1/1 (16MHz)
      // input capture noise canceler ON
      TCCR1B = (trig_edge == TRIG_E_DN) ? 0xc1 : 0x81;  // edge selection
      ACSR   = 0x14;  // capture-on, aco to caputure timer1
      TIFR1  = 0x27; // clear flags again

      ui1 = (tokadif * toka) + (osctdly << 4);  // delay time

      // falling edge detection(rising edge for ICES1)
      // doesn't stabilize without a 20usec wait below.
      while(TCNT1 < 320)
          ;
      TIFR1 = 0x27;
      // wait until a trigger event happens
      while(true) {
        if (TIFR1 & 0x20) {
          // trigger event has happened.
          ui1 += ICR1;
          at = 0; // a trigger event has happened.
          break;
        }
        if (TCNT1 > waituntil) {
          ui1 += TCNT1;
          at = 1; // trigger failed.
          break;
        }
      }

      // at:0 -> trigger event has happened, 1 -> not happened
      ACSR   = 0x94; // disable analog comparator
      ADCSRB = 0x00; // AC multiplexer disable, ADC auto trigger source free run
      ADCSRA = 0x84; // adc enable, 1MHz, adate off

      TCCR1B = 0x19; // timer1 CTC-ICR1 mode pre1/1
      TCCR1A = 0x00; //             CTC mode;
      ICR1   = ui1;
      TIFR1  = 0x27; // clear flags

      ADMUX  = adch; // adc target is A0 pin to get trigger value;
      ADCSRB = 0x07; // timer1 capture event;
      ADCSRA = 0xf4; // adc auto trigger, force 1st conversion

      // wait until the 1st conversion finishes. 
      while((ADCSRA & 0x10) == 0x00)
        ;

      ADMUX = adch + ch;
      ADCSRA = 0xb4;   // clear flag, 1MHz, adate on

      if (toka == 0 && ch == 0) {   // needed only for the 1st loop
        if (at) {
          for(i = 0; i < SAMPLES; i++)
            buf0[i] = analogRead(ad_ch0);
          scaleDataArray(ad_ch0);
          return; // when trigger failed
        }
      }

      for(i = 0; i < realnum; i++) {
        while(true) {
          if (TIFR1 & 0x20) {
            ICR1 = sinterval;
            TIFR1 = 0x27; // clear timer1 flags;
          }
          if ((ADCSRA & 0x10) != 0x00)
            break;
        }
        byte *pdata = (byte *) &buf0[dp];
        *pdata++ = ADCL;
        *pdata = ADCH;
        dp += tokanum;
        ADCSRA = 0xb4;   // clear flag, 1MHz, adate on
      }
    }
  }
  scaleDataArray(ad_ch0);
}

#include <Arduino.h>
#include <avr/interrupt.h>
#include "lookup_tables.h"


#ifdef ATTINYx5
#define SD    PIN_B4 //TODO: support for shutdown
#define H1    PIN_B0
#define L1    PIN_B1
#define H2    PIN_B3
#define L2    PIN_B4

#elif ATTINYX61
#define POT1  A5 //debug pins
#define SEL1  A6 //debug pins
#define SD    PIN_B4 //TODO: support for shutdown
#define H1    PIN_B0
#define L1    PIN_B1
#define H2    PIN_B2
#define L2    PIN_B3
#endif


//#define DEADRISING 5
//#define DEADFALLING 5

volatile uint8_t pwmValue1 = 0;
volatile uint8_t pwmValue2 = 0;
volatile uint8_t index = 0;

#define MAX_COUNTER 1
uint16_t counter = 0;

/* UPDATE PWM
// f_wave := f_PCK / (prescaler * (OCR1C+1) * ARRAY_LEN * (MAX_COUNTER+1)) ~= 60.24Hz
// f_PCK := 32Mhz
// prescaler := 8
// OCR1C := 199
// ARRAY_LEN := 166
// MAX_COUNTER := 1
*/
ISR(TIMER1_OVF_vect) {
  if (counter++ >= MAX_COUNTER){
    counter = 0;
    index++;
    index %= ARRAY_LEN;

    OCR1A = pwmValue1;
    OCR1B = pwmValue2;
  }
}

void setup() {
  #ifdef ATTINYX61
  pinMode(POT1, INPUT);
  pinMode(SEL1, INPUT);
  #endif
  pinMode(SD, OUTPUT);
  pinMode(H1, OUTPUT); // OC1A
  pinMode(L1, OUTPUT); // not OC1A
  pinMode(H2, OUTPUT); // OC1B
  pinMode(L2, OUTPUT); // not OC1B

  digitalWrite(SD,LOW); // IR2112 active
  SREG = _BV(7); // Global interruption enabled

  /*timer1 conf*/
  PLLCSR = _BV(LSM) | _BV(PLLE); //activates Low Speed Mode in PCK and enables it.
  delay(1);
  while(!(PLLCSR & _BV(PLOCK))); //wait until PLL is in steady state.
  PLLCSR |= _BV(PCKE); //enable PCK source for timer1 (asynchronous mode).

  //PWM freq = 20khz, page 88 in datasheet (PCK = 32Mhz)
  TIMSK = _BV(TOIE1); // interrupt enable when TCNT1 overflows (TCNT1 == OCR1A).
  OCR1C = 199; // top value for TCNT1 for PWM freq = 20khz, R = 7.6
  OCR1A = 100; // initial value of comparation
  #ifdef ATTINYx5
  TCCR1 = _BV(PWM1A) | _BV(COM1A0) | _BV(CS12); //activates complementary mode in A and 1:8 prescaler
  GTCCR = _BV(PWM1B) | _BV(COM1B0);
  //DT1A = (0xF0 & (DEADRISING << 4)) | (0x0F & DEADFALLING);
  #elif ATTINYX61
  TCCR1A = _BV(PWM1A) | _BV(PWM1B) | _BV(COM1A0) | _BV(COM1B0); //activates PMW complementary mode in A and B
  TCCR1B = _BV(CS11) | _BV(CS10); //1:4 prescaler
  //DT1 = (0xF0 & (DEADRISING << 4)) | (0x0F & DEADFALLING);
  #endif
  //DTPS1 = _BV(DTPS10); //prescaler 1:1
}

uint16_t tmp = 0;

void loop() {
  #ifdef ATTINYX61
  tmp = analogRead(SEL1);
  if (tmp <= 512) {
    pwmValue1 = analogRead(POT1) >> 2;
    pwmValue2 = ~pwmValue1;
  } else {
  #endif
    pwmValue1 = pgm_read_byte(&(D[index]));
    //WARNING: if 0 > 255 use other index or index type
    pwmValue2 = pgm_read_byte(&(D[(index+PHASE180)%ARRAY_LEN]));
  #ifdef ATTINYX61
  }
  #endif
}

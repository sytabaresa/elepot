#include <Arduino.h>
#include <avr/interrupt.h>
#include "lookup_tables.h"

#ifdef ATTINYx5
#define POT1  A3
#define SEL1  A2
#define SD    2
#define H     1
#define L     0
#endif

volatile uint8_t pwmValue = 0;
volatile uint8_t index = 0;
uint8_t counter = 0;

//void updatePWM() {
ISR(TIMER1_OVF_vect) {
  if (counter++ >= 32){ //10khz
    counter = 0;
    index == 166 ? index=0 : index++;
    OCR1A = pwmValue;
  }
}

void setup() {
  pinMode(POT1, INPUT);
  pinMode(SEL1, INPUT);
  pinMode(SD, OUTPUT);
  pinMode(H, OUTPUT); // OC1A
  pinMode(L, OUTPUT); // not OC1A


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
  TCCR1 = _BV(PWM1A) | _BV(COM1A0) | _BV(CS12); //activates complementary mode in A and 1:8 prescaler

  /*timer0 conf*/
  //OCR0A = 100; // 100 us
  //TIMSK |= _BV(TOIE0); // enable interruption when TCNTR0 == OCR0A
  //TCCR0A = _BV(WGM01); // CTC mode
  //TCCR0B = _BV(CS00); // prescaler 1:1.
}

uint16_t tmp = 0;

void loop() {
  tmp = analogRead(SEL1);
  if (tmp <= 512) pwmValue = analogRead(POT1) >> 2;
  else if (tmp > 512 && tmp <= 853) pwmValue = D1[index];
  else pwmValue = D2[index];
}

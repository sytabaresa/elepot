// Based in AC Control v1.1 (http://playground.arduino.cc/Main/ACPhaseControl)
//
//
// AC Phase control is accomplished using the internal
// hardware timer1 in the Arduino
//
// Timing Sequence
// * timer is set up but disabled
// * zero crossing detected on pin DETECT
// * timer starts counting from zero
// * comparator set to "delay to on" value
// * counter reaches comparator value
// * comparator ISR turns on TRIAC gate
// * counter set to overflow - pulse width
// * counter reaches overflow
// * overflow ISR turns off TRIAC gate
// * TRIAC stops conducting at next zero cross

// For the Arduino Uno, Nano and Atmega328
// The hardware timer runs at 16MHz. Using a
// divide by 256 on the counter each count is
// 16 microseconds.  1/2 wave of a 60Hz AC signal
// is about 520 counts (8,333 microseconds).

// For the Attiny45,Attiny861
// The hardware timer runs at 1MHz, Using a
// divide by 32 on the counter each count is 
// 16 microseconds. 1/2 wave of a 60Hz AC signal
// is about 32 counts (8,333 microseconds).

#include <Arduino.h>
#include <EnableInterrupt.h>

#ifdef DEBUG
#include <avr_mcu_section.h>
AVR_MCU(F_CPU, "attiny45");
#endif

#define MODE RISING

//Pin Mapping
#ifdef ATTINY45
#define DETECT1 4  //zero cross detect
#define GATE1 3    //TRIAC gate
#define POTIN1  A1 // pin 4 Potentiometer input

#elif NANO
#define DETECT1 0  //zero cross detect
#define GATE1 2    //TRIAC gate
#define POTIN1  A1 // Potentiometer input
#endif

//constants
#define MINV 3
#define MAXV 240
#define PULSE 4   //trigger pulse width (counts)

//variables
int i=100;

//Interrupt Service Routines

void zeroCrossingInterrupt(){ //zero cross detect
  #ifdef NANO
  TCCR1B=0x04; //start timer with divide by 256 input
  #elif ATTINY45
  TCCR1=0x06;  //start timer with divide by 32 input
  #endif
  TCNT1 = 0;   //reset timer - count from zero
}

void setup(){
  // set up pins
  //Serial.begin(9600);
  pinMode(POTIN1, INPUT); // Analog read
  digitalWrite(POTIN1, HIGH); //enable pull-up resistor

  pinMode(GATE1, OUTPUT);      //TRIAC gate control
  pinMode(LED_BUILTIN, OUTPUT);      //TRIAC gate control
  pinMode(DETECT1, INPUT);     //zero cross detect
  digitalWrite(DETECT1, HIGH); //enable pull-up resistor

  // set up Timer1
  OCR1A = 255;      //initialize the comparator
  #ifdef NANO
  //(see ATMEGA 328 data sheet pg 134 for more details)
  TIMSK1 = 0x03;  //enable comparator A and overflow interrupts
  TCCR1A = 0x00;  //timer control registers set for
  TCCR1B = 0x00;  //normal operation, timer disabled
  #elif ATTINY45
  TIMSK = 0x44;    //enable comparator A and overflow interrupts
  TCCR1 = 0x00;    //timer control registers set for
                   //normal operation, timer disabled
  #endif

  // set up zero crossing interrupt
  //TODO: not use library?
  enableInterrupt(DETECT1, zeroCrossingInterrupt, MODE);
  #endif
}

ISR(TIMER1_COMPA_vect){ //comparator match
  digitalWrite(GATE1,HIGH);  //set TRIAC gate to high
  //OCR1A = 0x00;
  TCNT1 = 0xFF - PULSE;      //trigger pulse width
}

ISR(TIMER1_OVF_vect){ //timer1 overflow
  digitalWrite(GATE1,LOW); //turn off TRIAC gate
  #ifdef NANO
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
  #elif ATTINY45
  TCCR1 = 0x00;          //disable timer stopd unintended triggers
  #endif
}

void loop(){

  i = analogRead(POTIN1) >> 2;     //set the compare register brightness desired.
  if (i < MINV) i = MINV;
  if (i > MAXV) i = MAXV;
  //Serial.println(i);
  analogWrite(LED_BUILTIN,i);
  OCR1A = i;
}

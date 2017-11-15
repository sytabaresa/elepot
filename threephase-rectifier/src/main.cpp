// AC Phase control is accomplished using the internal
// hardware timer1 in the Arduino
//
// Timing Sequence
// * timer is set up and enabled and counts and overflow always
// * zero crossing detected on pin DETECTx (1,2 or 3 phase)
// * for the phase detected, pushes new phase id in the stack
// * comparator A or B set to timer1 value + "delay to on" value (mod 256 sum)
// * can be two values of comparation at the same time (two of three phase waves overlaps every time)
// * counter reaches comparator value
// * comparator ISR turns on TRIACx gate (reads phase id in the stack)
// * comparator sets to timer1 value + pulse width (mod 256 sum)
// * counter reaches comparator value
// * comparator ISR turns off TRIAC gate (pops phase id in the stack)
// * TRIACx stops conducting at next zero cross

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

// Definitions
// Three or mono phase control //TODO: add support for mono phase mode
#define THREEPHASE

// mode of operation:
// RISING: half-wave mode
// CHANGE: full-wave mode
#define MODE RISING
#define MODE CHANGE

// Comparation symbols
#define COMPA  0
#define COMPB  1


//Pin Mapping

#ifdef NANO
#define DETECT1 2  //zero cross detect pin 1
#define DETECT2 3  //zero cross detect pin 2
#define DETECT3 4  //zero cross detect pin 3
#define GATE1 5    //TRIAC1 gate
#define GATE2 6    //TRIAC2 gate
#define GATE3 7    //TRIAC3 gate
#define POTIN1  A2 // Potentiometer input

#elif ATTINYX61 //TODO: fix pins
#define DETECT1 0  //zero cross detect
#define DETECT2 1  //zero cross detect
#define DETECT3 2  //zero cross detect
#define GATE1 3    //TRIAC gate
#define GATE2 4    //TRIAC gate
#define GATE3 5    //TRIAC gate
#define POTIN1  A0 // Potentiometer input
#endif

//constants
#define MINV 3
#define MAXV 240
#define PULSE 4   //trigger pulse width (counts)

//variables

//Interrupt Service Routines
volatile int i=100;
volatile char phase[] = {0,0};
volatile bool comparator = COMPA;
volatile bool pulseA, pulseB = false;

void zeroCrossingInterrupt(){ //zero cross detect
  switch (comparator) {
    case COMPA: OCR1A = TCNT1 + i; break;
    case COMPB: OCR1B = TCNT1 + i; break;
  }
  comparator = !comparator;
}

inline void zero1() {
  phase[0] ? phase[1] = 1: phase[0] = 1;
  zeroCrossingInterrupt();
}

inline void zero2() {
  phase[0] ? phase[1] = 2: phase[0] = 2;
  zeroCrossingInterrupt();
}

inline void zero3() {
  phase[0] ? phase[1] = 3: phase[0] = 3;
  zeroCrossingInterrupt();
}

void setup(){
  // set up pins
  //Serial.begin(9600);
  pinMode(POTIN1, INPUT); // Analog read

  pinMode(LED_BUILTIN, OUTPUT);      //TRIAC gate control
  pinMode(GATE1, OUTPUT);      //TRIAC1 gate control
  pinMode(GATE2, OUTPUT);      //TRIAC2 gate control
  pinMode(GATE3, OUTPUT);      //TRIAC3 gate control

  pinMode(DETECT1, INPUT);     //zero cross detect 1
  pinMode(DETECT2, INPUT);     //zero cross detect 2
  pinMode(DETECT3, INPUT);     //zero cross detect 3

  digitalWrite(DETECT1, HIGH); //enable pull-up resistor
  digitalWrite(DETECT2, HIGH); //enable pull-up resistor
  digitalWrite(DETECT3, HIGH); //enable pull-up resistor

  // set up Timer1
  OCR1A = 255;      //initialize the comparator
  TCNT1 = 0; //reset timer1, begin in 0;
  #ifdef NANO
  //(see ATMEGA 328 data sheet pg 134 for more details)
  TIMSK1 = 0x03;  //enable comparator A and overflow interrupts
  TCCR1A = 0x00;  //timer control registers set for
  TCCR1B=0x04; //start timer with divide by 256 input

  #elif ATTINYX61 //TODO: fix timer1 conf
  TIMSK = 0xE0;  //enable comparator A and overflow interrupts
  TCCR1A = 0x00;  //timer control registers set for
  TCCR1B = 0x04; //start timer with divide by 256 input
                 // set up zero crossing interrupt
  #endif

  //TODO: no use library?
  attachInterrupt(digitalPinToInterrupt(DETECT1), zero1, MODE);
  attachInterrupt(digitalPinToInterrupt(DETECT2), zero2, MODE);
  attachInterrupt(digitalPinToInterrupt(DETECT3), zero3, MODE);

}

ISR(TIMER1_COMPA_vect){ //comparator match
  pulseA = !pulseA;
  switch (phase[0]) {
    case 1: digitalWrite(GATE1,pulseA); break; //set TRIAC1 gate to high
    case 2: digitalWrite(GATE2,pulseA); break; //set TRIAC2 gate to high
    case 3: digitalWrite(GATE3,pulseA); break; //set TRIAC3 gate to high
  }

  if(pulseA) OCR1A = TCNT1 + PULSE;  //trigger pulse width
  else {
    phase[0] = phase[1];
    phase[1] = 0;
  }
}

ISR(TIMER1_COMPB_vect){ //comparator match
  pulseB = !pulseB;
  switch (phase[0]) {
    case 1: digitalWrite(GATE1,pulseB); break; //set TRIAC1 gate to high
    case 2: digitalWrite(GATE2,pulseB); break; //set TRIAC2 gate to high
    case 3: digitalWrite(GATE3,pulseB); break; //set TRIAC3 gate to high
  }

  if(pulseB) OCR1B = TCNT1 + PULSE;  //trigger pulse width
  else {
    phase[0] = phase[1];
    phase[1] = 0;
  }
}

void loop(){ // sample code to exercise the circuit

  i = analogRead(POTIN1) >> 2;     //set the compare register brightness desired.
  if (i < MINV) i = MINV;
  if (i > MAXV) i = MAXV;
  //Serial.println(i);
  analogWrite(LED_BUILTIN,i);
  delay(15);
}

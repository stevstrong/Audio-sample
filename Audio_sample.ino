/// @dir RFM12B_OOK
/// Receive / decode OOK signals using a RFM12B as 868 MHz OOK receiver.
//
// Adapted from the experiments and code by JGJ Veken, as posted on the wiki at:
// http://jeelabs.net/projects/cafe/wiki/Receiving_OOKASK_with_a_modified_RFM12B
//
// The basic idea is to measure pulse widths between 0/1 and 1/0 transitions,
// and to keep track of pulse width sequences in a state machine.
//
// 2009-04-08 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2010-10-15 JGJ: changed RFM12 init OOK-mode

#include <avr/io.h>
#include <util/delay.h>

#define DEBUG 1 // set to 1 to also send out info on the serial port

#define RXDATA 14
//#define RELAIS_PIN 8
#define MAX_VAL 4 // number of consequtive increased value to detect transition
#ifdef DEBUG
  #define ADC_OUT_PIN 9
  //int ADC_OUT_PIN = 9;
  //#define DBG_PIN 8
  int DBG_PIN = 8;
  static byte dbg = 0;
  #define Set_Dbg_Pin() ()
#endif

static byte oldVal = 0;
static byte adcCnt = 0;
static byte adcStatus;
static byte adcTo0;
static byte adcTo1;
////////////////////////////////////////////////
static void ADC_Irq ()
{
  byte readVal = ADCH; // result is left adjusted, we need only the upper 8 bits
//  byte count;// = TCNT2; // timer 2 counter value

  if (readVal>oldVal) {
    if ( adcCnt<MAX_VAL && (++adcCnt)==MAX_VAL && adcStatus==LOW ) { // detect 0->1 transition
      adcStatus = HIGH;
      adcTo1 = true;
    }
  } else if (readVal<oldVal) {
    if ( adcCnt>0 && (--adcCnt)==0 && adcStatus==HIGH ) { // detect 1->0 transition
      adcStatus = LOW;
      adcTo0 = true;
    }
  } else { // readVal==oldVal
    if (adcStatus==HIGH) {
      if (adcCnt<MAX_VAL) adcCnt++;
    } else {
      if (adcCnt>0) adcCnt--;
    }
  }
  oldVal = readVal;

  if (adcTo0 || adcTo1) { // any transition detected?
#ifdef DEBUG
    if (adcTo1) digitalWrite(ADC_OUT_PIN, HIGH);
    else if (adcTo0) digitalWrite(ADC_OUT_PIN, 0);
#endif

    adcTo0 = false;
    adcTo1 = false;
 //   count = TCNT2;
    TCNT2 = 0;  // reset to avoid timeout
/*    if (bufCnt>0) { // take only counter values from the second edge onwards
      ookbuf[bufCnt-1] = count;
    }
    bufCnt++;*/
  }

#if 1
  if (dbg) {
    dbg = 0;
    digitalWrite(DBG_PIN,1);
  } else {
    dbg = 1;
    digitalWrite(DBG_PIN,0);
  }
#endif
}
////////////////////////////////////////////////
ISR(ADC_vect) {
  ADC_Irq();
}
///////////////////////////////////////////////
static void Audio_sample_reset (void)
{
  adcStatus = LOW;
  adcTo0 = false;
  adcTo1 = false;
#ifdef DEBUG
  digitalWrite(ADC_OUT_PIN,0);
#endif
}
////////////////////////////////////////////////
static void Audio_sample_init(void)
{
    pinMode(RXDATA, INPUT);
//    pinMode(RELAIS_PIN, OUTPUT);
//    digitalWrite(RXDATA, 1); // pull-up
#ifdef DEBUG
  pinMode(ADC_OUT_PIN,OUTPUT);
  pinMode(DBG_PIN,OUTPUT);
#endif
  // set ADC0
  ADMUX = _BV(REFS0) | _BV(ADLAR); // Vcc as ref, left adjust for 8 bit resolution, use ADC0
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | 0x04; // enable, start conversion, auto trigger, prescaler 128
  ADCSRB = 0;  // free running mode
  DIDR0 = ~(0xC0 | _BV(ADC0D)); // disable other ADC input buffers
/*
    // enable analog comparator with fixed voltage reference
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
    ADCSRA &= ~ _BV(ADEN);
    ADCSRB |= _BV(ACME);
    ADMUX = 0; // ADC0
*/
    // prescaler 64 -> 250 KHz = 4 usec/count, max 1.024 msec (16 MHz clock)
    TCNT2 = 0;
    TCCR2A = 0;
    TCCR2B = _BV(CS22);
    TIMSK2 = _BV(TOIE2);
    //
    Audio_sample_reset(); // initialise ADC transition detection
}
////////////////////////////////////////////////////////////
ISR(TIMER2_OVF_vect)
{
    Audio_sample_reset();
}
//////////////////////////////////////////////////////
void setup ()
{
#if DEBUG
    Serial.begin(57600);
    Serial.println("\n[Audio_sample]");
#endif
    Audio_sample_init();
}
//////////////////////////////////////////////////////
void loop ()
{
}


#include <avr/io.h>
#include <util/delay.h>

//#define DEBUG 1 // uncomment to send out info on the serial port

#define RXDATA 14
#define MAX_VAL 2 // number of consequtive increased value to detect transition
#define ADC_OUT_PIN 9
#define SET_ADC_OUT_PIN  asm("sbi 0x05, 1")  // set PB1 (D9)
#define CLEAR_ADC_OUT_PIN  asm("cbi 0x05, 1")  // set PB1 (D9)

#ifdef DEBUG
  //int ADC_OUT_PIN = 9;
  //#define DBG_PIN 8
  int DBG_PIN = 8;
  static byte dbg = 0;
  #define SET_DBG_PIN  asm("sbi 0x05, 0")  // set PB1 (D8)
  #define CLEAR_DBG_PIN  asm("cbi 0x05, 0")  // set PB1 (D8)
#endif

volatile byte oldVal;
////////////////////////////////////////////////
ISR(ADC_vect)
{
  byte readVal = ADCH; // result is left adjusted, we need only the upper 8 bits
//  byte count;// = TCNT2; // timer 2 counter value
  if ( readVal>5 && readVal>oldVal) {
//    if ( adcCnt<MAX_VAL && (++adcCnt)==MAX_VAL && adcStatus==LOW ) { // detect 0->1 transition
//      adcStatus = HIGH;
      SET_ADC_OUT_PIN;
//    }
  } else {
//  if (readVal<oldVal) {
//    if ( adcCnt>0 && (--adcCnt)==0 && adcStatus==HIGH ) { // detect 1->0 transition
//      adcStatus = LOW;
      CLEAR_ADC_OUT_PIN;
//    }
  }
  oldVal = readVal;

#if 0
  if (dbg) {
    dbg = 0;
    SET_DBG_PIN;
  } else {
    dbg = 1;
    CLEAR_DBG_PIN;
  }
#endif
}
////////////////////////////////////////////////
static void Audio_sample_init(void)
{
  oldVal = 0;
  pinMode(RXDATA, INPUT);
  pinMode(ADC_OUT_PIN,OUTPUT);
  digitalWrite(ADC_OUT_PIN,0);
#ifdef DEBUG
  pinMode(DBG_PIN,OUTPUT);
#endif
  // set ADC0
  ADMUX = _BV(REFS0) | _BV(ADLAR); // Vcc as ref, left adjust for 8 bit resolution, use ADC0
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS1); // enable, start conversion, auto trigger, prescaler 8
  ADCSRB = 0;  // free running mode
  DIDR0 = ~(0xC0 | _BV(ADC0D)); // disable other ADC input buffers
/*
    // enable analog comparator with fixed voltage reference
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
    ADCSRA &= ~ _BV(ADEN);
    ADCSRB |= _BV(ACME);
    ADMUX = 0; // ADC0
*//*
    // prescaler 64 -> 250 KHz = 4 usec/count, max 1.024 msec (16 MHz clock)
    TCNT2 = 0;
    TCCR2A = 0;
    TCCR2B = _BV(CS22);
    TIMSK2 = _BV(TOIE2);
 */
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
{}

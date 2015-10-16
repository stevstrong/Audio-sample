// Audio signal from ADC7...4 continuously samped, alternatively stored in 2 different buffers,
// copied into the SD card cache, and then stored on SD card.
// Based on RawWrite.ino example of SdFat library.
/////////////////////////////////////////////////////////////////////////////////////

#include <SdFat.h>
#include <SdFatUtil.h>
#include <util/delay.h>

/***********************************************************************************/
/* Configure the active board */
#define _BOARD_YUN_
/* Configure the ADC reference voltage */
#define _USE_AREF_
/***********************************************************************************/

/**/
#define DEBUG 1 // set to 1 to send out info on the serial port
/**/
#ifdef DEBUG
  #define ADC_OUT_PIN 9
  #define DBG_PIN 8
  static byte dbg = 0;
#endif
/************************************************************************************/
// number of blocks in the contiguous file
#define BLOCK_COUNT 100
/************************************************************************************/
#define BUF_SIZE  512
#define BUF_PTR_MASK  (BUF_SIZE-1)
static byte buf1[BUF_SIZE];  // buffer 1
static byte buf2[BUF_SIZE];  // buffer 2
static volatile int buf1_rdPtr;
static volatile int buf1_wrPtr;
static volatile int buf2_rdPtr;
static volatile int buf2_wrPtr;
static int buf_of;  // overflow
#define BUF1 0
#define BUF2 1
static volatile byte buf_rdSel;
static volatile byte buf_wrSel;
//------------------------------------------------------------------------------
// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))
//------------------------------------------------------------------------------
// file system
SdFat sd;
// test file
SdFile file;
// file extent
static uint8_t * pCache;
/************************************************************************************/
// ADC enable, start conversion, auto trigger, prescaler 128
#define ADC_START (ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | 0x04)
#define ADC_STOP (ADCSRA = 0)
//-----------------------------------------------
#define ADC_MUX_CHANNELS 4  // must be power of 2, ADC_MUX_INIT must be changed accordingly
#define ADC_MUX_CHANNELS_MASK (ADC_MUX_CHANNELS-1)
/**/
#ifdef _USE_AREF_
  // setup mux: external ref, left adjust for 8 bit resolution, use ADC4...7
  #define ADC_MUX_INIT  ( _BV(ADLAR) | _BV(MUX2) )
#else
  // setup mux: internal 2.56V (Yun) / 1.1 (Pro Mini) as ref, left adjust for 8 bit resolution, use ADC4...7
  #define ADC_MUX_INIT  ( _BV(REFS1) | _BV(REFS0) | _BV(ADLAR) | _BV(MUX2) )
#endif
/**/
/**/
static volatile byte adc_mux;
static volatile byte adc_0; // very first conversion result
/************************************************************************************/
static void adc_setup ()
{
  // set pin mode of ADC4...7 (Yun) or ADC0...3 (Pro Mini) to input
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  /**/
#ifdef DEBUG
  pinMode(ADC_OUT_PIN,OUTPUT);
  digitalWrite(ADC_OUT_PIN,0);
  pinMode(DBG_PIN,OUTPUT);
  digitalWrite(DBG_PIN,0);
#endif
  /**/
  adc_mux = 3;
  adc_0 = 0;
  // stop ADC
  ADC_STOP;
  ADMUX = ADC_MUX_INIT | adc_mux; // init mux to ADC7
  ADCSRB = 0;  // free running mode
#ifdef _BOARD_YUN_
  DIDR0 = ~(0xF0); // enable ADC 7...4 (Yun) input buffers
#else
  DIDR0 = ~(0xCF); // enable ADC 0...3 (Pro Mini) input buffers
#endif
}

/************************************************************************************/
static void buf_init (void)
{
  buf1_rdPtr = BUF_SIZE;
  buf1_wrPtr = 0;
  buf2_rdPtr = BUF_SIZE;
  buf2_wrPtr = 0;
  buf_rdSel = BUF1;
  buf_wrSel = BUF1;
  buf_of = 0;  // overflow
}
/************************************************************************************/
ISR(ADC_vect)
{
  if ( adc_0==0 ) {
    adc_0 = 1; // don't store very first conversion result
  } else {
    if ( buf_wrSel==BUF1 ) {
      // check overflow
      if ( buf1_rdPtr<BUF_SIZE ) {
        buf_of++; // overflow
        return;
      }
      buf1[buf1_wrPtr] = ADCH;
      if ( (++buf1_wrPtr)>=BUF_SIZE ) {
        buf2_wrPtr = 0; // update pointer for next writing
        buf_wrSel = BUF2; // switch to second buffer
        //digitalWrite(ADC_OUT_PIN,1);
      }
    } else {
      // check overflow
      if ( buf2_rdPtr<BUF_SIZE ) {
        buf_of++; // overflow
        return;
      }
      buf2[buf2_wrPtr] = ADCH;
      if ( (++buf2_wrPtr)>=BUF_SIZE ) {
        buf1_wrPtr = 0; // update pointer for next writing
        buf_wrSel = BUF1; // switch to first buffer
        //digitalWrite(ADC_OUT_PIN,0);
      }
    }
  }
  // update ADC_MUX
  ADMUX = ADC_MUX_INIT | ( adc_mux = (adc_mux-1) & ADC_MUX_CHANNELS_MASK);
}
/************************************************************************************/
void sd_init()
{
  // initialize the SD card at SPI_FULL_SPEED for best performance.
  // try SPI_HALF_SPEED if bus errors occur.
  if (!sd.begin(SS, SPI_FULL_SPEED)) sd.initErrorHalt();

  // delete possible existing file
  sd.remove("RAW.TXT");

  // create a contiguous file
  if (!file.createContiguous(sd.vwd(), "RAW.TXT", 512UL*BLOCK_COUNT)) {
    error("createContiguous failed");
  }
  uint32_t bgnBlock, endBlock;
  // get the location of the file's blocks
  if (!file.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }

  //*********************NOTE**************************************
  // NO SdFile calls are allowed while cache is used for raw writes
  //***************************************************************

  // clear the cache and use it as a 512 byte buffer
  pCache = (uint8_t*)sd.vol()->cacheClear();

  // tell card to setup for multiple block write with pre-erase
  if (!sd.card()->writeStart(bgnBlock, BLOCK_COUNT)) {
    error("writeStart failed");
  }
  Serial.println(F("SD card initialized."));
}
/************************************************************************************/
void sd_write()
{
  Serial.println(F("Recording started..."));
  // start ADC
  ADC_START;

  // init stats
  uint32_t maxWriteTime = 0;
  uint32_t maxBlockTime = 0;

  uint32_t t = millis();
  // acquire data to buffers
  for (uint16_t b = 0; b < BLOCK_COUNT; b++)
  {
    uint32_t tb = micros();
    /**/
    if ( buf_wrSel==BUF1 ) {
      //  ADC writes BUF1, wait till BUF1 is full
      while ( buf1_wrPtr<BUF_SIZE ) {};
      // ADC finished writing BUF1, so now is time to read BUF1
      buf_rdSel = BUF1;
      buf1_rdPtr = 0;
      memcpy(pCache,buf1,512);
      buf1_rdPtr = BUF_SIZE;
    } else {
      // ADC writes BUF2, wait till BUF2 is full
      while ( buf2_wrPtr<BUF_SIZE ) {};
      // ADC finished writing BUF2, so now is time to read BUF2
      buf_rdSel = BUF2;
      buf2_rdPtr = 0;
      memcpy(pCache,buf2,512);
      buf2_rdPtr = BUF_SIZE;
    }
/**/
    // write a 512 byte cache block to SD card
    uint32_t tw = micros();
        digitalWrite(DBG_PIN,1);
      if (!sd.card()->writeData(pCache)) error("writeData failed");
        digitalWrite(DBG_PIN,0);
    tw = micros() - tw;
    // update max write time
    if (tw > maxWriteTime)  maxWriteTime = tw;
    //Serial.println(b);
    tb = micros() - tb;
    // update max write time
    if (tb > maxBlockTime)  maxBlockTime = tb;
  }
  ADC_STOP;
  // total write time
  t = millis() - t;

  // end multiple block write mode
  if (!sd.card()->writeStop()) error("writeStop failed");
   // close file for next pass of loop
  file.close();
 
  Serial.println(F("Acquisition done."));
  Serial.print(F("Elapsed total time [millis]: ")); Serial.println(t);
  Serial.print(F("Max block rec time [micros]: ")); Serial.println(maxBlockTime);
  Serial.print(F("Max SD write time [micros]: ")); Serial.println( maxWriteTime );
  Serial.print(F("Data recording speed [kBps]:")); Serial.println(((uint32_t)BLOCK_COUNT*512)/t);
  Serial.print(F("Overruns: ")); Serial.println( buf_of );
}
/************************************************************************************/
void setup ()
{
#if DEBUG
    Serial.begin(57600);
    Serial.println(F("\n*** Audio sampling and recording ***"));
#endif

    adc_setup();
}
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
void loop ()
{
  while (Serial.read() >= 0) {};
  
  buf_init(); // initialise ADC transition detection
  sd_init();  // prepare SD card

  Serial.println(F("Press any key to start...\n"));
  while (Serial.read() <= 0) {};

  sd_write();
}

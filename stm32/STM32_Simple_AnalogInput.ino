/*
	=================================================================================
	ADC conversion of multiple channels in dual regular simultaneous conversion mode.
	=================================================================================
	Each conversion sequence is started by timer 3 update event (up-counting mode).
	The result of each dual sample (32bits) is stored into the ADC buffer via DMA transfer.
	The recording time and sampling frequency is configurable.
	---------------------------------------------------------------------------------
*/

#include <libmaple/adc.h>
#include <libmaple/dma.h>
//#include <SPI.h>
//#include <SdFat.h>
//#include <SdFatUtil.h>

//#define Serial Serial1

/********************************************************************/
// Configuration
/********************************************************************/

/*****************************************************************************/
/*****************************************************************************/

// use for debug, comment out when not needed
#define DEBUG_PIN		PA8
//#define DEBUG_PIN_SET	( GPIOA->regs->BSRR = (1U << 8) )
//#define DEBUG_PIN_CLEAR	( GPIOA->regs->BSRR = (1U << 8) << 16 )

/********************************************************************/
// used analog input pins corresponding to maple mini ports or pins
#define ADC_IN0 PA0	// 11
#define ADC_IN1 PA1
#define ADC_IN2 PA2
#define ADC_IN3 PA3
#define ADC_IN4 PA4
#define ADC_IN5 PA5
#define ADC_IN6 PA6
#define ADC_IN7 PA7
//#define ADC_IN8 3  // maple mini pin
//#define ADC_IN9 33  // maple mini pin - actually used as LED output

/********************************************************************/
// variables
/********************************************************************/

/********************************************************************/
extern void gpio_set_mode(gpio_dev *dev, uint8 pin, gpio_pin_mode mode);
extern void gpio_toggle_bit(gpio_dev *dev, uint8 pin);
/*****************************************************************************/
/*****************************************************************************/
void ADC_Setup(void)
{
	Serial.print(F("setting up the ADC..."));

	adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);

#if 1
	Serial.print(F("single sw start..."));
	adc_enable_single_swstart(ADC1);
#else
	adc_init(ADC1); //rcc_clk_enable(ADC1->clk_id), Must be the first adc command!
	adc_set_sample_rate(ADC1, ADC_SMPR_7_5);//ADC_SMPR_1_5);
	adc_set_reg_seqlen(ADC1, 1);
	ADC1->regs->CR1 = 0;
	adc_enable(ADC1);
	adc_calibrate(ADC1); //optional, but strongly recommended
#endif
	Serial.println(F("done."));
}
/*****************************************************************************/
/*****************************************************************************/
uint16_t ADC_Read(const adc_dev *dev, uint8_t channel)
{
	adc_reg_map *regs = dev->regs;
	regs->SQR3 = channel;
	regs->CR2 |= ADC_CR2_SWSTART;
	while (!(regs->SR & ADC_SR_EOC))
	;
	return (uint16_t)(regs->DR & ADC_DR_DATA);
}
/*****************************************************************************/
/*****************************************************************************/
void setup()
{
	Serial.begin(500000);
//	Serial1.begin(250000);
	while(!Serial) {}  // wait for Leonardo

	delay (5000);

	Serial.println("\n***** ADC single conversion mode *****\n");
	delay (100);

	// set port mode of used analog input pins
	// comment out lines of not used analog input pins
	pinMode(ADC_IN0, INPUT_ANALOG);
	pinMode(ADC_IN1, INPUT_ANALOG);
	pinMode(ADC_IN2, INPUT_ANALOG);
	pinMode(ADC_IN3, INPUT_ANALOG);
	pinMode(ADC_IN4, INPUT_ANALOG);
	pinMode(ADC_IN5, INPUT_ANALOG);
	pinMode(ADC_IN6, INPUT_ANALOG);
	pinMode(ADC_IN7, INPUT_ANALOG);
#if defined DEBUG_PIN
	pinMode(DEBUG_PIN, OUTPUT);
	//gpio_set_mode(GPIOA, 8,GPIO_OUTPUT_PP);	// PA8, pin 27
	Serial.println(F("Debug version!"));
#endif
	ADC_Setup();
}
/*****************************************************************************/
bool start = false;
/*****************************************************************************/
/*****************************************************************************/
void ParseParams(char * p)
{
	//Serial1.print(F("parsing: ")); Serial1.println(p);
	if ( strstr(p,"rec_time")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial.print(F(">> rec_time = ")); Serial.println(val);
			//recording_time = (uint16_t)val;
		}
	} else
	if ( strstr(p,"sampling_freq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1)*1000;
			Serial.print(F(">> sampling_freq = ")); Serial.println(val);
			//sampling_frequency = val;
		}
	} else
	if ( strstr(p,"samples_per_seq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial.print(F(">> samples_per_seq = ")); Serial.println(val);
			//samples_per_sequence = (uint8_t)val;
		}
	}
}
/*****************************************************************************/
uint16_t dat_buff[500] __attribute__ ((packed, aligned(2)));
char rec_buff[200] __attribute__ ((packed));
bool str_mode = true;
bool st = false;
/*****************************************************************************/
void ParseRxData()
{	// check for commands "go" or "set" in the Rx string
	if ( strncmp(rec_buff, "go", 2)==0 )			start = true;
	//else if ( strncmp(rec_buff, "get", 3)==0 )	TransmitBinaryData();
	else if ( strncmp(rec_buff, "st", 2)==0 )	st = (st? 0:1);
	else {	// divide string into tokens
		char * ptr = strtok(rec_buff, " ;");
		if ( ptr>0 && strcmp(ptr, "set")==0 ) {
			//Serial1.println(F("> set detected..."));
			while ( (ptr=strtok(NULL, " ;"))>0 ) // get parameters
				ParseParams(ptr);
		}
	}
	rec_buff[0] = 0;
}
/*****************************************************************************/
/*****************************************************************************/
byte SerialReadBytes()
{
#define SERIAL_RX_TIMEOUT	100	// millis
	// wait for serial data
	rec_buff[0] = 0;	// mark start of string
	//if ( Serial1.available()<=0 ) return 0;
	byte rec_index = 0;
#if 1
	uint32_t ts = millis();	// prepare reception time-out
	while ( (millis()-ts)<SERIAL_RX_TIMEOUT ) {	// no time-out occurred yet
		if ( Serial.available()<=0 ) continue;
		rec_buff[rec_index++] = Serial.read();
		ts = millis();	// reset time-out to receive next byte
	}
#else
	Serial.setTimeout(SERIAL_RX_TIMEOUT);
	rec_index = Serial.readBytesUntil('\n', rec_buff, BUFFER_LENGTH);
#endif
	if ( rec_index==0 ) return 0;	// nothing was received
	rec_buff[rec_index] = 0;	// mark end of string
	if ( rec_buff[rec_index-1]=='\n' ) rec_buff[rec_index-1] = 0;	// clear '\n'
	if ( str_mode ) {
		//Serial.print(F("> received: ")); Serial.println(rec_buff);// Serial.println(F(" chars."));
	}
	return rec_index;
}
/*****************************************************************************/
/*****************************************************************************/
void CheckSerial()
{
	if ( Serial.available()<=0 )	return;
	// read any existing Serial data
	rec_buff[0] = 0;
	if ( SerialReadBytes() )	// parse received serial data
		ParseRxData();
}
/*****************************************************************************/
int cnt = 0;
/*****************************************************************************/
void loop()
{
	str_mode = true;	// receive in string mode
	CheckSerial();	// try to receive setup parameters and/or commands

	uint16_t dat, i,j;
	dat = ADC_Read(ADC1, 0);	// read channel 0 value
	if (st) return;
	for (j = 0; j<500; j++) {
		dat_buff[j] = ADC_Read(ADC1, 0);	// read channel 0 value
	}
	cnt = 0;
	for (j = 0; j<500; ) {
		Serial.println();	Serial.print(cnt++);	Serial.print(": ");
		for (i = 0; i<10; i++) {
			dat = dat_buff[j++];	// read channel 0 value
			Serial.print(dat);
			Serial.print(',');
		}
	}
	st = true;
}

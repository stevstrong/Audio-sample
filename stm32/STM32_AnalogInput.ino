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
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>

#define Serial Serial1

/********************************************************************/
// Configuration
/********************************************************************/
// setup recording parameters
uint16_t recording_time = 1000;			// in milliseconds
uint32_t sampling_frequency = 26000;	// in Hz
// regular channel sequence length
uint8_t samples_per_sequence = 2;

#define SAMPLES_PER_SEQUENCE	4	// must be power of 2 !!!
// Set the ADC input channel sequences (left will be first)
// channel 17 = Vrefint, only available on ADC1, can be used for tests
uint8_t ADC1_Sequence[SAMPLES_PER_SEQUENCE] = {0,2,4,6};
uint8_t ADC2_Sequence[SAMPLES_PER_SEQUENCE] = {1,3,5,7};

/*****************************************************************************/
/*****************************************************************************/
// SD card chip select pin
//const uint8_t chipSelect = 31;
#define chipSelect PB12

// use for debug, comment out when not needed
#define DEBUG_PIN		PA8
//#define DEBUG_PIN_SET	( GPIOA->regs->BSRR = (1U << 8) )
//#define DEBUG_PIN_CLEAR	( GPIOA->regs->BSRR = (1U << 8) << 16 )

/********************************************************************/
// defines
/********************************************************************/
// DO NOT CHANGE !!!
/********************************************************************/
// size of data block which will be written in one shot to card
#define BLOCK_SIZE				512	// bytes
// size of data buffer where the ADC sampled data will be transferred over DMA
#define ADC_BUFFER_SIZE			(2*BLOCK_SIZE)
// some default values
#define RECORD_SIZE				2	// 16 bits result of an ADC conversion
#define RECORDS_PER_BLOCK		(BLOCK_SIZE/RECORD_SIZE)
#define RECORDS_PER_BUFFER		(ADC_BUFFER_SIZE/RECORD_SIZE)
#define SAMPLE_SIZE				4	// dual 16 bits, combined value of ADC2 (high word) and ADC1 (low word)
#define SAMPLES_PER_BLOCK		(BLOCK_SIZE/SAMPLE_SIZE)
#define SAMPLES_PER_BUFFER		(ADC_BUFFER_SIZE/SAMPLE_SIZE)
uint32_t sequences_per_block =	(SAMPLES_PER_BLOCK/samples_per_sequence);
//#define SEQUENCES_PER_BUFFER	(SAMPLES_PER_BUFFER/used_SAMPLES_PER_SEQUENCE)

// calculate some relevant values
uint32_t needed_sequences =	((sampling_frequency*recording_time)/1000);
uint32_t needed_blocks =		(needed_sequences/sequences_per_block);
// the SW can only record even number of blocks.
#if (NEEDED_BLOCKS%2)>0
 #define TOTAL_BLOCKS 			(NEEDED_BLOCKS+1)
#else
 #define TOTAL_BLOCKS 			(NEEDED_BLOCKS)
#endif
uint32_t total_blocks = 		(needed_blocks);
uint32_t total_sequences =		(total_blocks*sequences_per_block);
uint32_t total_samples =		(total_sequences*samples_per_sequence);

// timing related constants
#define TIMER_PRESCALER			4	// to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY			(72000000/TIMER_PRESCALER)
//#define TIMER_RELOAD_VALUE		((TIMER_FREQUENCY/SAMPLING_FREQUENCY)-1)
uint32_t timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1;

/********************************************************************/
/********************************************************************/
void SetupParameters() {
	// calculate some relevant values
	sequences_per_block =	(SAMPLES_PER_BLOCK/samples_per_sequence);
	needed_sequences =	((sampling_frequency*recording_time)/1000);
	needed_blocks =		(needed_sequences/sequences_per_block);
	// the SW can only count and record a complete number of blocks.
	// so we need to round up the number of sequences to match a complete block
	if ( (needed_blocks&1) )
		total_blocks =			(needed_blocks+1);
	else
		total_blocks = 		(needed_blocks);
	total_sequences =		(total_blocks*sequences_per_block);
	total_samples =		(total_sequences*samples_per_sequence);
	// timing related values
	timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1;
}
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
// the main buffer to store ADC sample values
uint32_t adc_buffer[ADC_BUFFER_SIZE] __attribute__ ((packed,aligned(4)));

// The interrupt handler, DMA_Rx_irq(), sets this to 1.
volatile uint8_t dma_irq_fired;	// not really needed, remove later
// set by HW when a complete DMA transfer was finished.
volatile uint8_t dma_irq_full_complete;
// set by HW when a DMA transfer is at its half.
volatile uint8_t dma_irq_half_complete;
// set by SW when overrun occurs
volatile uint8_t overrun;
volatile uint32_t dma_irq_counter;
volatile uint32_t dma_isr;	// must not be global, make it local later
// signalling for lower and upper buffer status
volatile uint8_t buff0_stored, buff1_stored, buff_index;

/********************************************************************/
// file system
SdFat sd;
// test file
SdFile file;
#define error(s) sd.errorHalt(F(s))

uint8_t * pCache;

/********************************************************************/
extern void gpio_set_mode(gpio_dev *dev, uint8 pin, gpio_pin_mode mode);
extern void gpio_toggle_bit(gpio_dev *dev, uint8 pin);
/*****************************************************************************/
/*****************************************************************************/
void TIMER_Setup(void)
{
	timer_init(TIMER3);
	// set timer 3 in up-counter mode with auto-reload.
	// as this mode is not supported by the core lib, we have to set up the registers manually.
	(TIMER3->regs).adv->CR1 = 0;
	(TIMER3->regs).adv->CR2 = ( TIMER_CR2_MMS_UPDATE );
	(TIMER3->regs).adv->SMCR = 0;
	(TIMER3->regs).adv->DIER = 0;
	(TIMER3->regs).adv->SR = 0;
	(TIMER3->regs).adv->EGR = 0;			// bit TIMER_EGR_UG can be set to generate event by SW !
	(TIMER3->regs).adv->CCMR1 = 0;
	(TIMER3->regs).adv->CCMR2 = 0;
	(TIMER3->regs).adv->CCER = 0;
	(TIMER3->regs).adv->CNT = 0;	// set it only in down-counting more
	(TIMER3->regs).adv->PSC = TIMER_PRESCALER-1;	
	(TIMER3->regs).adv->ARR = timer_reload_value;
	(TIMER3->regs).adv->CCR1 = 0;
	(TIMER3->regs).adv->CCR2 = 0;
	(TIMER3->regs).adv->CCR3 = 0;
	(TIMER3->regs).adv->CCR4 = 0;
	(TIMER3->regs).adv->DCR = 0;			// don't use DMA
	(TIMER3->regs).adv->DMAR = 0;
	// don't forget to set the ADC trigger source to TIMER3->TRG0. Do it before enabling the ADC !!!
}
/*****************************************************************************/
/*****************************************************************************/
void DMA_Init(void)
{
	dma_irq_full_complete = 0;
	dma_irq_half_complete = 0;
	overrun = 0;
	dma_irq_counter = 0;
	dma_isr = 0;
	buff0_stored = buff1_stored = 1;	// avoid overrun detection
	buff_index = 0;
	dma_clear_isr_bits(DMA1, DMA_CH1);
	// for test only: fill ADC buffer with dummy data
	for (int i = 0; i<SAMPLES_PER_BUFFER; ) adc_buffer[i++] = 0x11111111;
}
/*****************************************************************************/
// This is our DMA interrupt handler.
/*****************************************************************************/
void DMA_Rx_irq(void)
{
// Used to store DMA interrupt status register (ISR) bits. This helps explain what's going on
	dma_isr = dma_get_isr_bits(DMA1, DMA_CH1);
	if (dma_isr&DMA_ISR_HTIF1) {
		dma_irq_half_complete = 1;
		buff0_stored = 0;	// reset storage flag to detect overrun
		if ( buff1_stored==0 )	overrun++;	// upper buffer half being written before was stored
	}
	if (dma_isr&DMA_ISR_TCIF1) {
		dma_irq_full_complete = 1;
		buff1_stored = 0;	// reset storage flag to detect overrun
		if ( buff0_stored==0 )	overrun++;	// lower buffer half being written before was stored
	}
	dma_clear_isr_bits(DMA1, DMA_CH1);
}
/*****************************************************************************/
/*****************************************************************************/
void DMA_Setup(void)
{
	Serial.print(F("preparing the DMA..."));
	DMA_Init();
	// turn DMA on
	dma_init(DMA1);
// DMA tube configuration
dma_tube_config my_tube_cfg = {
	&ADC1->regs->DR,	// data source address
	DMA_SIZE_32BITS,	// source transfer size
	&adc_buffer,		// data destination address 
	DMA_SIZE_32BITS,	// destination transfer size
	//ADC_BUFFER_SIZE,	// nr. of data to transfer
	// nr. of data to transfer, e.g. 1024 bytes buffer size / 4 bytes per transfer = 256 samples to transfer
	SAMPLES_PER_BUFFER,
	// tube flags: auto increment dest addr, circular buffer, set half/full IRQ, very high prio:
	( DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_HALF_CMPLT_IE | DMA_CFG_CMPLT_IE | DMA_CCR_PL_VERY_HIGH ),
	0,	// unused
	DMA_REQ_SRC_ADC1,	// Hardware DMA request source
};
	// configure DMA channel
	int ret = dma_tube_cfg(DMA1, DMA_CH1, &my_tube_cfg);
	if ( ret>0 ) {
		Serial.print(F("DMA configuration error: ")); Serial.println(ret,HEX);
		Serial.print(F("It is not safe to continue!!!"));
		while ( Serial.read()<=0 );	// wait for a key stroke
	}
	// attach an interrupt handler.
	dma_attach_interrupt(DMA1, DMA_CH1, DMA_Rx_irq);
	// Turn on the DMA tube. It will now begin serving requests.
	dma_enable(DMA1, DMA_CH1);

//debug	Serial.print(F("DMA1->regs->CNDTR1: ")); Serial.println(DMA1->regs->CNDTR1);
	Serial.println(F("done."));
}
/*****************************************************************************/
/*
 * calc_adc_sequence(ADCx_Sequence) converts the SQR3 6 channels' list into
 * a valid 6 X 5=30 bits sequence format and returns that 30 bits number.
 * There are 6 available sequences in each SQR3 SQR2, and 4 in SQR1.
 * For more channels, repeat the same for SQR2, SQR1. (For SQR1 4 channels only!)
 */
/*****************************************************************************/
uint32_t ADC_sequence_build(uint8 adc_seq_array[])
{
	uint32_t adc_sequence = 0;
	for (int i = 0; i<SAMPLES_PER_SEQUENCE; i++) {
		adc_sequence |= adc_seq_array[i] << (i * 5);
	}
	return adc_sequence;
}
/*****************************************************************************/
/*****************************************************************************/
void ADC_Setup(void)
{
	Serial.print(F("setting up the ADC..."));

	adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);

	adc_init(ADC1); //rcc_clk_enable(ADC1->clk_id), Must be the first adc command!
	adc_init(ADC2);

	adc_set_sample_rate(ADC1, ADC_SMPR_7_5);//ADC_SMPR_1_5);
	adc_set_sample_rate(ADC2, ADC_SMPR_7_5);//ADC_SMPR_1_5);

	uint32_t cr2 = ( ADC_CR2_EXTTRIG | ADC_EXT_EV_TIM3_TRGO | ADC_CR2_DMA | ADC_CR2_TSVREFE ); // ADC_CR2_TSEREFE for test only
	ADC1->regs->CR2 = cr2;
	ADC2->regs->CR2 = cr2;

	adc_enable(ADC1);
	adc_enable(ADC2);

	adc_calibrate(ADC1); //optional, but strongly recommended
	adc_calibrate(ADC2);

	adc_set_reg_seqlen(ADC1, samples_per_sequence); //The number of channels to be converted.
	adc_set_reg_seqlen(ADC2, samples_per_sequence);
	// set the channel sequences
	ADC1->regs->SQR3 = ADC_sequence_build(ADC1_Sequence);
	ADC2->regs->SQR3 = ADC_sequence_build(ADC2_Sequence);

	ADC1->regs->CR1 = ( (0x06 << 16) | ADC_CR1_SCAN ); // b0110: Regular simultaneous mode only, enable SCAN
	ADC2->regs->CR1 = ( ADC_CR1_SCAN ); // enable SCAN, mode setting not possible for ADC2
	Serial.println(F("done."));
}
/*****************************************************************************/
void SD_Init(void)
{
	Serial.print(F("initializing the SD card..."));
	// initialize the SD card
	if ( !sd.begin(chipSelect, SPI_CLOCK_DIV2) ) {
		sd.initErrorHalt();
	}
	// delete possible existing file
	sd.remove("RawWrite.txt");
	// create a contiguous file
	if (!file.createContiguous(sd.vwd(), "RawWrite.txt", (total_blocks*BLOCK_SIZE))) {
		error("createContiguous failed");
	}
	uint32_t bgnBlock, endBlock;
	// get the location of the file's blocks
	if (!file.contiguousRange(&bgnBlock, &endBlock)) {
		error("contiguousRange failed");
	}
	// clear the cache and use it as a 512 byte buffer
	pCache = (uint8_t*)sd.vol()->cacheClear();
	/** Start a write multiple blocks sequence.
	 * \param[in] blockNumber Address of first block in sequence.
	 * \param[in] eraseCount The number of blocks to be pre-erased.
	*/
	if (!sd.card()->writeStart(bgnBlock, total_blocks)) {
		error("writeStart failed");
	}
	Serial.println(F("done."));
	delay(100);
}
/*****************************************************************************/
/*****************************************************************************/
void setup()
{
	Serial.begin(57600);
	Serial1.begin(250000);
	while(!Serial) {}  // wait for Leonardo

	delay (5000);

	Serial.println("\n***** ADC dual regular simultaneous mode acquisition *****\n");
	delay (100);

	// use SPI 2 because SPI 1 pins are used as analog input
	SPI.setModule(2);

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
//	pinMode(ADC_IN8, INPUT_ANALOG);
//	pinMode(ADC_IN9, INPUT_ANALOG);
#if defined DEBUG_PIN
	pinMode(DEBUG_PIN, OUTPUT);
	//gpio_set_mode(GPIOA, 8,GPIO_OUTPUT_PP);	// PA8, pin 27
	Serial.println(F("Debug version!"));
#endif
}
/*****************************************************************************/
bool start = false;
/*****************************************************************************/
void SetupModules()
{
//debug outputs
	Serial.println(F("Input parameters:"));
	Serial.print(F("recording_time [ms]: ")); Serial.println(recording_time);
	Serial.print(F("sampling_frequency [Hz]: ")); Serial.println(sampling_frequency);
	Serial.print(F("samples_per_sequence: ")); Serial.println(samples_per_sequence);

// set-up involved hardware modules
	TIMER_Setup();
	ADC_Setup();
	DMA_Setup();
	start = false;	// stop after first run
}
/*****************************************************************************/
#define BUFFER_LENGTH		200
char rec_buff[BUFFER_LENGTH];
/*****************************************************************************/
void ParseParams(char * p)
{
	//Serial1.print(F("parsing: ")); Serial1.println(p);
	if ( strstr(p,"rec_time")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial1.print(F(">> rec_time = ")); Serial1.println(val);
			recording_time = (uint16_t)val;
		}
	} else
	if ( strstr(p,"sampling_freq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1)*1000;
			Serial1.print(F(">> sampling_freq = ")); Serial1.println(val);
			sampling_frequency = val;
		}
	} else
	if ( strstr(p,"samples_per_seq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial1.print(F(">> samples_per_seq = ")); Serial1.println(val);
			samples_per_sequence = (uint8_t)val;
		}
	}
}
/*****************************************************************************/
/*****************************************************************************/
void ParseRxData()
{	// check for commands "go" or "set" in the Rx string
	if ( strncmp(rec_buff, "go", 2)==0 )			start = true;
	else if ( strncmp(rec_buff, "get", 3)==0 )	TransmitBinaryData();
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
bool str_mode = true;
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
		if ( Serial1.available()<=0 ) continue;
		rec_buff[rec_index++] = Serial1.read();
		ts = millis();	// reset time-out to receive next byte
	}
#else
	Serial1.setTimeout(SERIAL_RX_TIMEOUT);
	rec_index = Serial1.readBytesUntil('\n', rec_buff, BUFFER_LENGTH);
#endif
	if ( rec_index==0 ) return 0;	// nothing was received
	rec_buff[rec_index] = 0;	// mark end of string
	if ( rec_buff[rec_index-1]=='\n' ) rec_buff[rec_index-1] = 0;	// clear '\n'
	if ( str_mode ) {
		Serial1.print(F("> received: ")); Serial1.println(rec_buff);// Serial.println(F(" chars."));
	}
	return rec_index;
}
/*****************************************************************************/
/*****************************************************************************/
void CheckSerial()
{
	// read any existing Serial data
	//while (Serial1.read() >= 0) {}
	delay(100);
	Serial1.println(F("\ntype any character to start\n"));
	rec_buff[0] = 0;
	while ( !start ) {
		if ( SerialReadBytes() )	// parse received serial data
			ParseRxData();
	}
}
/*****************************************************************************/
/*****************************************************************************/
void SD_buffer_to_card(byte buf)
{
	buff_index = buf;
	//	copy data from buffer to cache
	uint32_t * sp = adc_buffer+(buf*SAMPLES_PER_BLOCK);
	uint32_t * dp = (uint32_t*)pCache;
	for ( int i=0; i<SAMPLES_PER_BLOCK; i++ )	{
		*dp++ = *sp++;	// optimized copy using destination and source pointers
	}
//	uint32_t tw = micros();
	// write a 512 byte block cache to card
	if (!sd.card()->writeData((const uint8_t*)pCache)) {
		error("writeData failed");
	}
//	tw = micros() - tw;
}
/*****************************************************************************/
/*****************************************************************************/
void loop()
{
	str_mode = true;	// receive in string mode
	CheckSerial();	// try to receive setup parameters and/or commands
	SetupParameters();
	SetupModules();
	// prepare card
	SD_Init();

	Serial.print(F("-> sampling started..."));

	uint32_t tout;	// time-out
	uint32_t dma_irq_full_complete_count = 0;
	uint32_t dma_irq_half_complete_count = 0;

	uint32_t tstart = micros();
	// let timer run - do this just before generating an update trigger by SW
	timer_resume(TIMER3);
	// from now on, the sampling of sequences should be automatically triggered by Timer3 update event.
	// sample until number of input blocks was filled with data
	uint32_t block;
	for(block=0; block<total_blocks; block++) {
		// time-out to avoid hangup if something goes wrong
		tout = 1000000;
		// each ADC sequence is sampled triggered by TIMER3 event.
		while ( (--tout)>0 ) {
			// check buffer status and push data to card if necessary
			if ( dma_irq_half_complete ) {
				dma_irq_half_complete = 0;
				SD_buffer_to_card(0);	// store lower block data to card
				buff0_stored = 1;
				dma_irq_half_complete_count++;
				break;
			}
			if ( dma_irq_full_complete ) {
				dma_irq_full_complete = 0;
				SD_buffer_to_card(1);	// store upper block data to card
				buff1_stored = 1;
				dma_irq_full_complete_count++;
				break;
			}
		}
		// only for debug purposes:
		if ( tout==0 ) {
			Serial.println(F("\n!!!! TIME_OUT !!!")); break;
		}
	}
	timer_pause(TIMER3);	// stop timer, to stop ADC triggering
	long t = micros()-tstart;	// measure time

	// end multiple block write mode
	if (!sd.card()->writeStop()) {
		Serial.println(F("\nERROR: sd.card->writeStop failed!"));
		error("writeStop failed");
	}
	// close file for next pass of loop
	file.close();

	Serial.println(F("done."));
	// print statistics
	Serial.print(F("Overruns: ")); Serial.println(overrun);
//	Serial.print(F("DMA1->regs->CNDTR1: ")); Serial.println(DMA1->regs->CNDTR1);
//	Serial.print(F("dma_isr: ")); Serial.println(dma_isr);
	Serial.print(F("Elapsed total time (useconds): ")); Serial.println(t);
	Serial.print(F("Recorded blocks: "));  Serial.println(block);	//TOTAL_BLOCKS
	Serial.print(F("Recording time for 1 block (useconds): ")); Serial.println(t/(double)block);
	Serial.print(F("Recorded sequences: "));  Serial.println(block*sequences_per_block);	//TOTAL_SEQUENCES
	Serial.print(F("Recording time for 1 sequence (useconds): ")); Serial.println(t/(double)(block*sequences_per_block));
/*debug
	Serial.print(F("dma_irq_counter: ")); Serial.println(dma_irq_counter);
	Serial.print(F("dma_irq_half_complete_count: ")); Serial.println(dma_irq_half_complete_count);
	Serial.print(F("dma_irq_full_complete_count: ")); Serial.println(dma_irq_full_complete_count);
*/
	// OPTIONAL: print last samples from ADC buffer to check validity
	//SendData();

return;

//debug
/*
	Serial.println(F("\nSave to card? (type 'y' for yes, any other for no\n"));
	int c;
	while ( (c= Serial.read())<=0 ) {}
	if ( c=='y' ) {
		//debug
		SD_Init();
				gpio_toggle_bit(GPIOA,8);	//debug - measure card transfer time
		SD_buffer_to_card(0);
				gpio_toggle_bit(GPIOA,8);	//debug - measure card transfer time
		SD_buffer_to_card(1);
				gpio_toggle_bit(GPIOA,8);	//debug - measure card transfer time
		// end multiple block write mode
		if (!sd.card()->writeStop()) {
			error("writeStop failed");
		}
		// close file for next pass of loop
		file.close();
	}*/
}
/***************************************************************************/
/*****************************************************************************/
void TransmitBinaryData(void)
{
	// load and send the binary recorded data to serial
	if ( file.open("RawWrite.txt", O_READ) ) {
		delay(100);
		if ( file.open("RawWrite.txt", O_READ) ) {
			Serial.println(F("ERROR: could not open file to read!"));
			sd.errorPrint();	return;
		}
	}
	uint32_t fsize = file.fileSize();
	Serial.print(F("binary_length: ")); Serial.println(fsize);	// send binary data length
	Serial.print(F(">>>"));	// send binary start marker
	for (uint32_t i=0; i<fsize; i++) {
		Serial.write((uint8_t)file.read());
	}
	int16_t chr;
#if 1
	uint8_t d_buff[256] __attribute__ ((packed));
	while ( (chr=file.read(d_buff, sizeof(d_buff)))>0 ) {
		Serial.write(d_buff, chr);
	}
#else
	while ( (chr=file.read())>=0 )	Serial.write((uint8_t)chr);
#endif
	file.close();
//return;
/*
	// send here some binary data
typedef struct {
	uint8_t byte0;			// 0x41
	uint16_t len;	// length of data of forthcoming data block
//	uint8_t data[512];	// max 512 bytes
	uint16_t crc;	// 16 bit CRC = sum of previous data bytes ^0xFF+1 so that the sum = 0.
} __attribute__ ((packed, aligned(1))) tx_pack_t;
tx_pack_t pack;
	Serial.print(F("sizeof pack: ")); Serial.println(sizeof(tx_pack_t));
*/
}
/***************************************************************************/
/*****************************************************************************/
void SendData()
{
	// print here the latest data block from buffer
	Serial.print(F("sampled data:"));
	uint8_t seq = 0;
	uint16_t * sp = (uint16_t*)adc_buffer + (buff_index*SAMPLES_PER_BLOCK*2);	// *2 because pointer is (uint16_t *) instead of (uint32_t *)
	for(int k=0; k<SAMPLES_PER_BLOCK; k++)
	{
		if( (k%samples_per_sequence)==0 ) {Serial.print('\n'); Serial.print(seq++); Serial.print(':');}
		Serial.print(" ");
		Serial.print(*sp++, HEX);	// there are 2 records in a sample, this is the first one
		Serial.print(" ");
		Serial.print(*sp++, HEX);	// there are 2 records in a sample, this is the second one
	}
	Serial.println();
//return;

	// send here some binary data
typedef struct {
	uint8_t byte0;			// 0x41
	uint16_t len;	// length of data of forthcoming data block
//	uint8_t data[512];	// max 512 bytes
	uint16_t crc;	// 16 bit CRC = sum of previous data bytes ^0xFF+1 so that the sum = 0.
} __attribute__ ((packed, aligned(1))) tx_pack_t;
tx_pack_t pack;
	Serial.print(F("sizeof pack: ")); Serial.println(sizeof(tx_pack_t));
return;

uint8_t reply;	// 0x06 = CRC+LEN OK, any other (0x55) for wrong length or wrong CRC, in which case the data block must be sent again.
#define ACK	(0x06)
	Serial.println(F("switch to binary trx"));
int8_t retry = 3;
	str_mode = false;	// don't reply received chars
	//while ( (retry--)>0 ) {	// switch to binary mode
		Serial.print("\t\t\t");
		delay(100);
/*		uint32_t t = millis();
		while ( (millis()-t)<1000 ) // wait max 1 second for reply
			if ( SerialReadBytes()>0 ) break;
		if ( rec_buff[0]==ACK ) break;	// done
	//}
	if ( rec_buff[0]==0 ) return;*/
	// switch back to string transmission
	//Serial.println(F("switch back to string trx"));
	retry = 3;
//	while ( (retry--)>0 ) {	// switch to binary mode
		pack.byte0 = 0x41;
		Serial.write(pack.byte0);	// send the pack with no data
		pack.len = 0;	// length of data of forthcoming data block
		Serial.write((byte*)&pack.len,2);	// send the pack with no data
		pack.crc = pack.byte0 + pack.len;
		pack.crc ^= 0xffff;
		Serial.write((byte*)&pack.crc,2);	// send the pack with no data
		//Serial.write((byte*)&pack, sizeof(tx_pack_t));	// send the pack with no data
		Serial.write((byte*)&pack, 5);	// send the pack with no data
		// wait for reply
		uint32_t t = millis();
		while ( (millis()-t)<1000 && SerialReadBytes()<=0 )	{}; // wait max 1 second for reply
//		if ( rec_buff[0]==ACK ) break; // done
//	}
	str_mode = true;	// reply received chars
return;

	retry = 3;
	int8_t buff_count = 2;
	while ( (retry--)>0 ) {
		pack.byte0 = 0x41;
		pack.len = 512;	// length of data of forthcoming data block
		Serial.write((byte*)&pack.byte0, 3);
		pack.crc = pack.byte0 + pack.len;
		// read next data block from SD card and send it
		//f.read(buff, 512);
		// send latest data block from buffer
		uint8_t * sp = (byte*)adc_buffer + (buff_index*SAMPLES_PER_BLOCK);
		for(int k=0; k<SAMPLES_PER_BLOCK; k++)
		{
			uint8_t dat = (byte)*sp++;
			Serial.write(dat);
			pack.crc += dat;
		}
		pack.crc = (0xffff^pack.crc);
		Serial.write(pack.crc&0xff);
		Serial.write(pack.crc/256);
		// wait for acq = 0x06
		uint32_t t = millis();
		while ( (millis()-t)<1000 ) // wait max 1 second for reply
			if ( SerialReadBytes()>0 ) break;
		if ( rec_buff[0]==0 || rec_buff[0]!=ACK ) continue;	// try to send the pack again
		buff_index = buff_index^0x01;	// switch buffer index to send second buffer
		if ( (--buff_count)==0 ) break;	// end of transmitting second buffer 
	}
	str_mode = true;	// reply received chars
}

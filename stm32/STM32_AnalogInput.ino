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

/********************************************************************/
// Configuration
/********************************************************************/
// setup recording parameters
#define RECORDING_TIME			1000	// in milliseconds
#define SAMPLING_FREQUENCY		26000	// in Hz

// regular channel sequence length
#define SAMPLES_PER_SEQUENCE	4	// must be power of 2 !!!
// Set the ADC input channel sequences (left will be first)
// channel 17 = Vrefint, only available on ADC1, can be used for tests
uint8_t ADC1_Sequence[SAMPLES_PER_SEQUENCE] = {17,17,0,1};
uint8_t ADC2_Sequence[SAMPLES_PER_SEQUENCE] = {1,0,1,0};

// SD card chip select pin
const uint8_t chipSelect = 31;

/*****************************************************************************/
/*****************************************************************************/
// use for debug, comment out when not needed
#define DEBUG_PIN		27	// PA8
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
#define SAMPLE_SIZE				4	// dual 16 bits, combined value of ADC2 (high word) and ADC1 (low word)
#define SAMPLES_PER_BLOCK		(BLOCK_SIZE/SAMPLE_SIZE)
#define SAMPLES_PER_BUFFER		(ADC_BUFFER_SIZE/SAMPLE_SIZE)
#define SEQUENCES_PER_BLOCK	(SAMPLES_PER_BLOCK/SAMPLES_PER_SEQUENCE)
#define SEQUENCES_PER_BUFFER	(SAMPLES_PER_BUFFER/SAMPLES_PER_SEQUENCE)

// calculate some relevant values
#define NEEDED_SEQUENCES		((SAMPLING_FREQUENCY*RECORDING_TIME)/1000)
#define NEEDED_BLOCKS			(NEEDED_SEQUENCES/SEQUENCES_PER_BLOCK)
// the SW can only count and record a complete number of blocks.
// so we need to round up the number of sequences to match a complete block
#if (NEEDED_SEQUENCES%SEQUENCES_PER_BLOCK)>0
 #define TOTAL_BLOCKS 			(NEEDED_BLOCKS+1)
#else
 #define TOTAL_BLOCKS 			(NEEDED_BLOCKS)
#endif
#define TOTAL_SEQUENCES			(TOTAL_BLOCKS*SEQUENCES_PER_BLOCK)
#define TOTAL_SAMPLES			(TOTAL_SEQUENCES*SAMPLES_PER_SEQUENCE)

//#define BLOCK_COUNT				(TOTAL_BLOCKS)

// timing related constants
#define TIMER_PRESCALER			4	// to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY			(72000000/TIMER_PRESCALER)
#define TIMER_RELOAD_VALUE		((TIMER_FREQUENCY/SAMPLING_FREQUENCY)-1)

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
uint32_t adc_buffer[ADC_BUFFER_SIZE];

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
volatile uint8_t buff0_stored, buff1_stored;

/********************************************************************/
// file system
SdFat sd;
// test file
SdFile file;
#define error(s) sd.errorHalt(F(s))

uint32_t * pCache;

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
	(TIMER3->regs).adv->ARR = TIMER_RELOAD_VALUE;
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
	dma_irq_fired = 0;
	dma_irq_full_complete = 0;
	dma_irq_half_complete = 0;
	overrun = 0;
	dma_irq_counter = 0;
	dma_isr = 0;
	buff0_stored = buff1_stored = 1;	// avoid overrun detection
	dma_clear_isr_bits(DMA1, DMA_CH1);
	// for test only: fill ADC buffer with dummy data
	for (int i = 0; i<SAMPLES_PER_BUFFER; i++) adc_buffer[i] = 0x11111111;
}
/*****************************************************************************/
// This is our DMA interrupt handler.
/*****************************************************************************/
void DMA_Rx_irq(void)
{
	dma_irq_fired = 1;
	dma_irq_counter++;
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
	Serial.println(F("preparing the DMA..."));
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
	for (int i = 0; i<SAMPLES_PER_SEQUENCE; i++)
	{
		adc_sequence |= adc_seq_array[i] << (i * 5);
	}
	return adc_sequence;
}
/*****************************************************************************/
/*****************************************************************************/
void ADC_Setup(void)
{
	Serial.println(F("setting up the ADC..."));

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

	adc_set_reg_seqlen(ADC1, SAMPLES_PER_SEQUENCE); //The number of channels to be converted.
	adc_set_reg_seqlen(ADC2, SAMPLES_PER_SEQUENCE);
	// set the channel sequences
	ADC1->regs->SQR3 = ADC_sequence_build(ADC1_Sequence);
	ADC2->regs->SQR3 = ADC_sequence_build(ADC2_Sequence);

	ADC1->regs->CR1 = ( (0x06 << 16) | ADC_CR1_SCAN ); // b0110: Regular simultaneous mode only, enable SCAN
	ADC2->regs->CR1 = ( ADC_CR1_SCAN ); // enable SCAN, mode setting not possible for ADC2
}
/*****************************************************************************/
void SD_Init(void)
{
	// initialize the SD card
	if ( !sd.begin(chipSelect, SPI_CLOCK_DIV2) ) {
		sd.initErrorHalt();
	}
	// delete possible existing file
	sd.remove("RawWrite.txt");
	// create a contiguous file
	if (!file.createContiguous(sd.vwd(), "RawWrite.txt", (TOTAL_BLOCKS*BLOCK_SIZE))) {
		error("createContiguous failed");
	}
	uint32_t bgnBlock, endBlock;
	// get the location of the file's blocks
	if (!file.contiguousRange(&bgnBlock, &endBlock)) {
		error("contiguousRange failed");
	}
	// clear the cache and use it as a 512 byte buffer
	pCache = (uint32_t*)sd.vol()->cacheClear();
	/** Start a write multiple blocks sequence.
	 * \param[in] blockNumber Address of first block in sequence.
	 * \param[in] eraseCount The number of blocks to be pre-erased.
	*/
	if (!sd.card()->writeStart(bgnBlock, TOTAL_BLOCKS)) {
		error("writeStart failed");
	}
	Serial.println(F("SD card initialized."));
}
/*****************************************************************************/
/*****************************************************************************/
void SD_buffer_to_card(int buff)
{
//return;	//debug
	if ( buff!=1 && buff!=0) { /*Serial.println(F("Invalid buffer, cannot copy!"));*/ return; }
	buff = buff*SAMPLES_PER_BLOCK;	// starting address of the current block where to copy from
	//	copy data from buffer to cache
	for ( int i=0; i<SAMPLES_PER_BLOCK; i++ )
	{
		pCache[i] = adc_buffer[i+buff];
	}
	// store cache to card, write a 512 byte block
//	uint32_t tw = micros();
	if (!sd.card()->writeData((const uint8_t*)pCache)) {
		error("writeData failed");
	}
//	tw = micros() - tw;
}
/*****************************************************************************/
/*****************************************************************************/
void setup()
{
	Serial.begin(57600);
	while(!Serial) {}  // wait for Leonardo

	delay (5000);

	Serial.println("\n***** ADC dual regular simultaneous mode acquisition *****\n");
	delay (100);
//debug outputs
	Serial.println(F("Input parameters:"));
	Serial.print(F("RECORDING_TIME [ms]: ")); Serial.println(RECORDING_TIME);
	Serial.print(F("SAMPLING_FREQUENCY [Hz]: ")); Serial.println(SAMPLING_FREQUENCY);
	Serial.print(F("TOTAL_BLOCKS: ")); Serial.println(TOTAL_BLOCKS);
	Serial.print(F("TOTAL_SEQUENCES: ")); Serial.println(TOTAL_SEQUENCES);
	Serial.print(F("TOTAL_SAMPLES: ")); Serial.println(TOTAL_SAMPLES);
/*	Serial.print(F("TIMER_RELOAD_VALUE: ")); Serial.println(TIMER_RELOAD_VALUE);
	Serial.print(F("SEQUENCES_PER_BLOCK: ")); Serial.println(SEQUENCES_PER_BLOCK);
	Serial.print(F("SEQUENCES_PER_BUFFER: ")); Serial.println(SEQUENCES_PER_BUFFER);
*/
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
// set-up involved hardware modules
	TIMER_Setup();
	ADC_Setup();
	DMA_Setup();
}
/*****************************************************************************/
/*****************************************************************************/
void loop()
{
	// read any existing Serial data
	while (Serial.read() >= 0) {}
	delay(1000);
	Serial.println(F("\ntype any character to start\n"));
	while (Serial.read() <= 0) {}

	// prepare card
	SD_Init();
	// initialize DMA variables
	DMA_Init();

	Serial.println(F("-> sampling started..."));

	uint32_t tout;	// time-out
	uint32_t dma_irq_full_complete_count = 0;
	uint32_t dma_irq_half_complete_count = 0;
//	uint32_t timer3_uif = 0;
//	int32_t data_cnt = (DMA1->regs->CNDTR1 - SAMPLES_PER_SEQUENCE);

	uint32_t tstart = micros();
	// let timer run - do this just before generating an update trigger by SW
	timer_resume(TIMER3);
	// from now on, the sampling of sequences should be automatically triggered by Timer3 update event.
	// sample until number of input blocks was filled with data
	uint32_t block;
	for(block=0; block<TOTAL_BLOCKS; block++) {
		// time-out to avoid hangup if something goes wrong
		tout = 1000000;
		// each ADC sequence is sampled triggered by TIMER3 event.
		// The DMA data counter should thereby decrease by SAMPLES_PER_SEQUENCE
		while ( (--tout)>0 ) {
/*debug
			// check timer 3 update flag and count pulses
			if ( (TIMER3->regs).adv->SR&TIMER_SR_UIF ) {
				(TIMER3->regs).adv->SR = 0;
				gpio_toggle_bit(GPIOA,8);	//debug - measure sequence sampling and transfer time
				timer3_uif++;
			}
			// check DMA data counter and count pulses
			if ( DMA1->regs->CNDTR1==data_cnt ) {
				gpio_toggle_bit(GPIOA,8);	//debug - measure sequence sampling and transfer time
				data_cnt -= SAMPLES_PER_SEQUENCE;
				if ( data_cnt<=0 ) data_cnt = SAMPLES_PER_BUFFER;
			}
*/
			// check buffer status and push data to card if necessary
			if ( dma_irq_half_complete ) {
				dma_irq_half_complete = 0;
				// store lower block data to card
				SD_buffer_to_card(0);
				buff0_stored = 1;
				dma_irq_half_complete_count++;
				break;
			}
			if ( dma_irq_full_complete ) {
				dma_irq_full_complete = 0;
				// store upper block data to card
				SD_buffer_to_card(1);
				buff1_stored = 1;
				dma_irq_full_complete_count++;
				break;
			}
			// clear buff0 flag if 
		}
		dma_irq_fired = 0; /*Serial.print(F("dma_isr: ")); Serial.println(dma_isr);*/
		// only for debug purposes:
		if ( tout==0 ) {
			Serial.println(F("\n!!!! TIME_OUT !!!"));
		}
	}
	timer_pause(TIMER3);	// stop timer, to stop ADC triggering
	long t = micros()-tstart;	// measure time

	// end multiple block write mode
	if (!sd.card()->writeStop()) {
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
	Serial.print(F("Recorded sequences: "));  Serial.println(block*SEQUENCES_PER_BLOCK);	//TOTAL_SEQUENCES
	Serial.print(F("Recording time for 1 sequence (useconds): ")); Serial.println(t/(double)(block*SEQUENCES_PER_BLOCK));
/*debug
	Serial.print(F("dma_irq_counter: ")); Serial.println(dma_irq_counter);
	Serial.print(F("dma_irq_half_complete_count: ")); Serial.println(dma_irq_half_complete_count);
	Serial.print(F("dma_irq_full_complete_count: ")); Serial.println(dma_irq_full_complete_count);
*/
	// OPTIONAL: print last samples from ADC buffer to check validity
	Serial.print(F("sampled data:"));
	uint8_t line = 1;
	for(int k=0; k<SAMPLES_PER_BUFFER; k++)
	{
		if( (k%SAMPLES_PER_SEQUENCE)==0 ) {Serial.println(); Serial.print(line++); Serial.print(':');}
		Serial.print(" ");
		uint32_t tmp = adc_buffer[k];
		Serial.print(tmp/0x10000, HEX);
		Serial.print(" ");
		Serial.print(tmp%0x10000, HEX);
	}
	Serial.println();
return;

//debug
	Serial.println(F("\nSave to card? (type 'y' for yes, any other for no\n"));
	int c;
	while ( (c= Serial.read())<=0 ) {}
	if ( c=='y' ) {
		/*debug*/
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
	}
}

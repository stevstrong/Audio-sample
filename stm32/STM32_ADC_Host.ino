/*
	=================================================================================
	Storage of multiple channels sampled in ADC dual regular simultaneous conversion mode.
	=================================================================================
	The samples of each conversion sequence is received over SPI1 and stored on the SD card over SPI2.
	The acquisition time, frequency and the channel sequences are configurable.
	These parameters are also forwarded to the slave. Reply from slave is directly forwarded to host.
	---------------------------------------------------------------------------------
*/

#include <libmaple/dma.h>
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>

//#define Serial Serial1

/********************************************************************/
// Configuration
/********************************************************************/
// setup recording parameters
uint16_t recording_time = 1000;			// in milliseconds
uint32_t sampling_frequency = 44100;	// in Hz
// Set the ADC input channel sequences (left will be first)

uint8_t samples_per_sequence = 2;

uint8_t use_diff_channels = 0;

/*****************************************************************************/
/*****************************************************************************/
// SD card chip select pin
#define CHIP_SELECT PB12
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
#define ADC_BUFFER_SIZE			(2*BLOCK_SIZE)	// bytes
#define RECORD_SIZE				2	// 16 bits result of an ADC channel conversion
#define RECORDS_PER_BLOCK		(BLOCK_SIZE/RECORD_SIZE)	// uint16_t
#define RECORDS_PER_BUFFER		(ADC_BUFFER_SIZE/RECORD_SIZE)	// uint16_t
#define SAMPLE_SIZE				4	// bytes
#define SAMPLES_PER_BLOCK		(BLOCK_SIZE/SAMPLE_SIZE)	// uint32_t
#define SAMPLES_PER_BUFFER		(ADC_BUFFER_SIZE/SAMPLE_SIZE)	// uint32_t
// the data buffer to store ADC sample values
uint16_t adc_buffer[ADC_BUFFER_SIZE] __attribute__ ((packed,aligned(2)));

uint32_t records_per_sample;
uint32_t total_blocks;
uint32_t total_sequences;
uint32_t total_samples;
uint32_t total_records;

// timing related constants and variables
#define TIMER_PRESCALER			4	// to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY			(72000000/TIMER_PRESCALER)
uint32_t timer_reload_value;
/********************************************************************/
// file system
SdFat sd;
// test file
SdFile file;
#define error(s) sd.errorHalt(F(s))

uint8_t * pCache;
/********************************************************************/
// variables
/********************************************************************/
// set by HW when a complete DMA transfer was finished.
volatile uint8_t dma_irq_full_complete;
// set by HW when a DMA transfer is at its half.
volatile uint8_t dma_irq_half_complete;
// set by SW when overrun occurs
volatile uint8_t overrun;
// signalling for lower and upper buffer status
volatile uint8_t buff0_stored, buff1_stored, buff_index;

/********************************************************************/
//extern void gpio_set_mode(gpio_dev *dev, uint8 pin, gpio_pin_mode mode);
//extern void gpio_toggle_bit(gpio_dev *dev, uint8 pin);

/********************************************************************/
/********************************************************************/
void SetupParameters(void)
{
	// calculate some relevant values
	total_sequences = sampling_frequency*recording_time/1000;
	records_per_sample = (2 - use_diff_channels);
	total_samples = (total_sequences*samples_per_sequence);
	total_records = (total_samples*records_per_sample);
	total_blocks = (total_records/RECORDS_PER_BLOCK);
	// the SW can only record an even number of blocks, so we need to adjust it
	if ( (total_blocks&1) )	total_blocks++;
	total_records = (total_blocks*RECORDS_PER_BLOCK);
	total_samples = total_records/records_per_sample;
	total_sequences = total_samples/samples_per_sequence;

	// timing related values
	timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1;
}
/*****************************************************************************/
/*****************************************************************************/
void DMA_Init(void)
{
	dma_irq_full_complete = 0;
	dma_irq_half_complete = 0;
	overrun = 0;
	buff0_stored = buff1_stored = 1;	// avoid overrun detection
	buff_index = 0;
	dma_clear_isr_bits(DMA1, DMA_CH2);
	// for test only: fill ADC buffer with dummy data
	for (int i = 0; i<RECORDS_PER_BUFFER; ) adc_buffer[i++] = 0x2222;
}
/*****************************************************************************/
// This is our DMA interrupt handler.
/*****************************************************************************/
void DMA_Rx_irq(void)
{
// Used to store DMA interrupt status register (ISR) bits. This helps explain what's going on
	uint32_t dma_isr = dma_get_isr_bits(DMA1, DMA_CH2);
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
	dma_clear_isr_bits(DMA1, DMA_CH2);
}
/*****************************************************************************/
/*****************************************************************************/
void DMA_Setup(void)
{
	Serial1.print(F("preparing the DMA..."));
	DMA_Init();

	dma_init(DMA1);	// turn DMA on
	dma_disable(DMA1, DMA_CH2);	// Enable the DMA tube. It will now begin serving requests.
// DMA tube configuration for SPI1 Rx - channel 2
dma_tube_config my_tube_cfg = {
	&SPI1->regs->DR,	// data source address
	DMA_SIZE_16BITS,	// source transfer size
	&adc_buffer,		// data destination address 
	DMA_SIZE_16BITS,	// destination transfer size
	RECORDS_PER_BUFFER,	// nr. of data to transfer
	// tube flags: auto increment dest addr, circular buffer, set tube full IRQ, very high prio:
	( DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_HALF_CMPLT_IE | DMA_CFG_CMPLT_IE | DMA_CCR_PL_VERY_HIGH ),
	0,	// unused
	DMA_REQ_SRC_SPI1_RX,	// Hardware DMA request source
};
	// configure DMA channel
	int ret = dma_tube_cfg(DMA1, DMA_CH2, &my_tube_cfg);	// SPI2 Rx channel is nr. 4
	if ( ret>0 ) {
		Serial1.print(F("DMA configuration error: ")); Serial1.println(ret,HEX);
		Serial1.print(F("Stopped, reset is needed!"));
		while ( 1 );	// halted
	}
	dma_attach_interrupt(DMA1, DMA_CH2, DMA_Rx_irq);	// attach an interrupt handler.
	dma_enable(DMA1, DMA_CH2);	// Enable the DMA tube. It will now begin serving requests.

	Serial1.println(F("done."));
}
/*****************************************************************************/
SPIClass SPI_2(2);
uint32_t app_cr1;
/*****************************************************************************/
void SPI_setup(void)
{
	Serial1.print(F("initializing SPI1..."));
	// use SPI 1 for recording, SPI 2 is used by SD card
	SPI.setModule(1);
	SPI.beginTransactionSlave(SPISettings(18000000, MSBFIRST, SPI_MODE0, DATA_SIZE_16BIT));
	SPI.setModule(2);
	Serial1.println(F("done."));
	//Serial1.print(F("cr1_config: ")); Serial1.println(app_cr1, HEX);
	Serial1.print(F("CR1: ")); Serial1.println(SPI1->regs->CR1, HEX);
	Serial1.print(F("CR2: ")); Serial1.println(SPI1->regs->CR2, HEX);
}
/*****************************************************************************/
void AppPrintCr1(uint32_t cr) { app_cr1 = cr;}
/*****************************************************************************/
/*****************************************************************************/
void SD_init(void)
{
	Serial1.print(F("deleting existing file..."));
	// delete possible existing file
	sd.remove("RawWrite.txt");
/*
	// open the file for write at end like the Native SD library
	if (!file.open("test.txt", O_WRITE | O_CREAT | O_AT_END)) {
		error("opening test.txt for write failed!");
	}
*/
	Serial1.print(F("done.\ncreate contiguous..."));
	// create a contiguous file
	if (!file.createContiguous(sd.vwd(), "RawWrite.txt", (total_blocks*BLOCK_SIZE))) {
		error("createContiguous failed");
	}
	Serial1.print(F("done.\ncontiguous range..."));
	uint32_t bgnBlock, endBlock;
	// get the location of the file's blocks
	if (!file.contiguousRange(&bgnBlock, &endBlock)) {
		error("contiguousRange failed");
	}
	Serial1.print(F("ok.\nstart write..."));
	// clear the cache and use it as a 512 byte buffer
	pCache = (uint8_t*)sd.vol()->cacheClear();
	if (!sd.card()->writeStart(bgnBlock, total_blocks)) {
		error("writeStart failed");
	}

	Serial1.println(F("done."));
	delay(100);
}
/*****************************************************************************/
void SD_end(void)
{
	file.close();
}
/*****************************************************************************/
/*****************************************************************************/
void setup()
{
	//Serial1.begin(57600);
	Serial1.begin(500000);
	while(!Serial1) {}  // wait for Leonardo

	Serial1.println(F("\n***** Multi-channel analog acquisition application *****\n"));
	Serial1.println(F("ADC data received over SPI1 in slave mode is stored to SD-card over SPI2."));
	delay (100);

#if defined DEBUG_PIN
	pinMode(DEBUG_PIN, OUTPUT);
	//gpio_set_mode(GPIOA, 8,GPIO_OUTPUT_PP);	// PA8, pin 27
	Serial1.println(F("Debug version!"));
#endif
	Serial1.print(F("initializing the SD card..."));
	// initialize the SD card
	if ( !sd.begin(CHIP_SELECT, SPI_CLOCK_DIV8) ) {
		sd.initErrorHalt("card begin failed!");
	}
}
/*****************************************************************************/
bool start = false;
/*****************************************************************************/
void SetupModules()
{	// set-up involved hardware modules
	SPI_setup();
	SD_init();
	DMA_Setup();
	start = false;	// stop after first run
}
/*****************************************************************************/
/*****************************************************************************/
void SD_buffer_to_card(byte buf)
{
	buff_index = buf;
	//	copy data from ADC buffer to SD cache
	uint16_t * sp = &adc_buffer[buf*RECORDS_PER_BLOCK];
	uint16_t * dp = (uint16_t*)pCache;
	if ( use_diff_channels ) {
		dp += (buf*RECORDS_PER_BLOCK/2);	// adjust destination pointer for second half cache
		uint16_t tmp = 0;
		//	copy data from source buffer to cache
		for ( uint16_t i=0; i<RECORDS_PER_BLOCK; i++ ) {
			uint16_t val = *sp++;
			if ( (i&0x0001) )
				*dp++ = (val - tmp)&0xffff;	// store differential value of current and previous reading
			else
				tmp = val;	// save the first record which is going to be subtracted from next record 
		}
		if ( buf==0 ) return;	// store only even blocks
	} else {
		for ( int i=0; i<RECORDS_PER_BLOCK; i++ )	{
			*dp++ = *sp++;	// optimized copy using destination and source pointers
		}
	}
//	uint32_t tw = micros();	// measure time needed for writing to card
	// write a 512 byte block cache to card
	if (!sd.card()->writeData((const uint8_t*)pCache)) {
		error("writeData failed");
	}
//	tw = micros() - tw;
}
/***************************************************************************/
/*****************************************************************************/
void TransmitBinaryData(void)
{
	// load and send the binary recorded data to serial
	if ( file.open("RawWrite.txt", O_READ) ) {
		delay(100);
		if ( file.open("RawWrite.txt", O_READ) ) {
			Serial1.println(F("ERROR: could not open file to read!"));
			sd.errorPrint();	return;
		}
	}
	uint32_t fsize = file.fileSize();
	Serial1.print(F("binary_length: ")); Serial1.println(fsize);	// send binary data length
	Serial1.print(F(">>>"));	// send binary start marker
	delay(100);
#if 1
	for (uint32_t i=0; i<fsize; i++) {
		Serial1.write((uint8_t)file.read());
	}
#else
	int16_t chr;
	uint8_t d_buff[256] __attribute__ ((packed));
	while ( (chr=file.read(d_buff, sizeof(d_buff)))>0 ) {
		Serial1.write(d_buff, chr);
	}
//	while ( (chr=file.read())>=0 )	Serial.write((uint8_t)chr);
#endif
	file.close();
}
/*****************************************************************************/
#define BUFFER_LENGTH		200
char rec_buff[BUFFER_LENGTH] __attribute__ ((packed));
/*****************************************************************************/
void ParseToken(char * p)
{
	//Serial1.print(F("parsing: ")); Serial1.println(p);
	if ( strstr(p,"time")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial1.print(F(">> rec_time = ")); Serial1.println(val);
			recording_time = (uint16_t)val;
		}
	} else
	if ( strstr(p,"freq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1)*1000;
			Serial1.print(F(">> sampling_freq = ")); Serial1.println(val);
			sampling_frequency = val;
		}
	} else
	if ( strstr(p,"seq_1")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			p++;
			Serial1.print(F(">> seq_1 = ")); Serial1.println(p);
			// store the channels of sequence 1
			uint16_t i;
			for (i = 0; i<8; i++) {
				(p++);
				if (*p++!=',')	break;
			}
			samples_per_sequence = i;
		}
	} else
	if ( strstr(p,"seq_2")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			p++;
			Serial1.print(F(">> seq_2 = ")); Serial1.println(p);
			// store the channels of sequence 2
			uint16_t i;
			for (i = 0; i<8; i++) {
				(p++);
				if (*p++!=',')	break;
			}
			if ( samples_per_sequence != i)	Serial1.print(F("__Error: invalid seq_2 length!__"));
		}
	} else
	if ( strstr(p,"go")>0 ) {
		start = true;
	} else
	if ( strstr(p, "get")>0 )	TransmitBinaryData();
}
/*****************************************************************************/
/*****************************************************************************/
void ParseRxData()
{	// check for command "go" or other parameters in the Rx string
	for (char * ptr = strtok(rec_buff, " ;");  ptr>0; ptr = strtok(NULL, " ;")) {
		ParseToken(ptr);
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
		Serial1.print(F("> received: ")); Serial1.println(rec_buff);// Serial1.println(F(" chars."));
	}
	return rec_index;
}
/*****************************************************************************/
/*****************************************************************************/
void CheckSerial()
{
	// read any existing Serial1 data
	//while (Serial1.read() >= 0) {}
	delay(100);
	Serial1.println(F("\n***** Accepted commands *****"));
	Serial1.print(F("\tSet up recording parameters: set rec_time=")); Serial1.print(recording_time);
		Serial1.print(F(";sampling_freq=")); Serial1.print(sampling_frequency/1000);
		Serial1.print(F(";samples_per_seq=")); Serial1.println(samples_per_sequence);
	Serial1.println(F("\tStart recording: go\\n"));
	Serial1.println(F("\tDump recorded data: get\\n\n*****"));
	rec_buff[0] = 0;
	while ( !start ) {
		if ( SerialReadBytes() )	// parse received serial data
			ParseRxData();
	}
}
/*****************************************************************************/
/*****************************************************************************/
void loop()
{
	str_mode = true;	// receive in string mode
	CheckSerial();	// try to receive setup parameters and/or commands
	SetupParameters();
	SetupModules();

	Serial1.print(F("-> total_blocks: ")); Serial1.print(total_blocks);

	uint32_t tout;	// time-out
	uint32_t dma_irq_count = 0;
//	uint32_t dma_irq_full_complete_count = 0;
//	uint32_t dma_irq_half_complete_count = 0;

	uint32_t tstart = micros();
	// from now on, the sampling of sequences should be automatically triggered by Timer3 update event.
	// sample until number of input blocks was filled with data
	uint32_t seq, val = 0;
	for(seq=0; seq<total_blocks; seq++) {
		// time-out to avoid hangup if something goes wrong
		tout = 1000000;
		// each ADC sequence is sampled triggered by TIMER3 event.
		while ( (--tout)>0 ) {
			// check ADC status and push data to SPI if sequence acquisition ended
			if ( dma_irq_half_complete ) {
				dma_irq_half_complete = 0;
				SD_buffer_to_card(0);	// store lower block data to card
				buff0_stored = 1;
				//dma_irq_half_complete_count++;
				break;
			}
			if ( dma_irq_full_complete ) {
				dma_irq_full_complete = 0;
				SD_buffer_to_card(1);	// store upper block data to card
				buff1_stored = 1;
				//dma_irq_full_complete_count++;
				break;
			}
			//if ( spi_is_rx_nonempty(SPI1) )	{ spi_rx_reg(SPI1); val++; }
		}
		// only for debug purposes:
		if ( tout==0 ) {
			Serial1.println(F("\n!!!! TIME_OUT !!!"));
			//Serial1.print(F("rx: ")); Serial1.println(val);
			break;
		}
	}
	long t = micros()-tstart;	// measure time
	
	SD_end();

	Serial1.println(F("done."));
	// print statistics
	Serial1.print(F("Overruns: ")); Serial1.println(overrun);
	Serial1.print(F("Elapsed total time (useconds): ")); Serial1.println(t);
	Serial1.print(F("Recorded sequences: "));  Serial1.println(seq);
	Serial1.print(F("Recording time for 1 sequence (useconds): ")); Serial1.println(t/(double)seq);
}

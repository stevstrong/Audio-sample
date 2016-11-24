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
//#include <SdFatUtil.h>

//#define Serial Serial1

//#define USE_FAT 1

/********************************************************************/
// Configuration
/********************************************************************/
// setup recording parameters
uint16_t recording_time = 1000;			// in milliseconds
uint32_t sampling_frequency = 44100;	// in Hz

// Set the ADC input channel sequences (left will be first)
// channel 17 = Vrefint, only available on ADC1, can be used for tests
uint8_t ADC1_Sequence[] __attribute__ ((aligned(1))) = {0,1};//,2,3,4,5,6,7};
uint8_t ADC2_Sequence[] __attribute__ ((aligned(1))) = {8,8};//,8,8,8,8,8,8};

uint16_t samples_per_sequence = sizeof(ADC1_Sequence);

uint8_t use_diff_channels = 0;

/*****************************************************************************/
/*****************************************************************************/
// SD card chip select pin
#define CHIP_SELECT PB12

#ifdef USE_FAT
  #define FILE_NAME	("RawWrite.txt")
#endif
// use for debug, comment out when not needed
//#define DEBUG_PIN	PB13 //PA8
//#define DEBUG_PIN_SET	( GPIOA->regs->BSRR = (1U << 8) )
//#define DEBUG_PIN_CLEAR	( GPIOA->regs->BSRR = (1U << 8) << 16 )
#define LED_PIN		PB1 // maple mini
//#define LED_PIN		PC13 // generic mini

/********************************************************************/
// defines
/********************************************************************/
// DO NOT CHANGE !!!
/********************************************************************/
// size of data block which will be written in one shot to card
#define BLOCK_SIZE				512	// bytes
#define SAMPLE_SIZE				4	// dual 16 bits, combined value of ADC2 (high word) and ADC1 (low word)
#define SAMPLES_PER_BLOCK		(BLOCK_SIZE/SAMPLE_SIZE)
#define BLOCKS_PER_BUFFER		16
#define BLOCKS_PER_BUFFER_MASK (BLOCKS_PER_BUFFER-1)
#define SAMPLES_PER_BUFFER		(BLOCKS_PER_BUFFER*SAMPLES_PER_BLOCK)	// the buffer size is twice the block size
// some default values
#define RECORD_SIZE				2	// 16 bits result of an ADC conversion
#define RECORDS_PER_BLOCK		(BLOCK_SIZE/RECORD_SIZE)

// size of data buffer where the ADC sampled data will be transferred over DMA
#define ADC_BUFFER_SIZE			(SAMPLES_PER_BUFFER)
// the main buffer to store ADC sample values
uint32_t adc_buffer[ADC_BUFFER_SIZE];
// some needed variables
uint32_t records_per_sample;
//uint32_t records_per_block;
uint32_t total_records;
uint32_t total_blocks;
uint32_t block_nr;

// timing related constants
#define TIMER_PRESCALER			4	// to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY			(72000000/TIMER_PRESCALER)
uint32_t timer_reload_value;

/********************************************************************/
/********************************************************************/
void SetupParameters() {
	records_per_sample = (2 - use_diff_channels);
	// calculate some relevant values
	total_records = ((samples_per_sequence*records_per_sample*sampling_frequency*recording_time)/1000);
	total_blocks = (total_records/RECORDS_PER_BLOCK);
	// the SW can only record an even number of blocks, so we need to adjust it
	if ( (total_blocks&1) )	total_blocks++;
	total_records = (total_blocks*RECORDS_PER_BLOCK);
	// timing related values
	timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1;
}
/********************************************************************/
// used analog input pins corresponding to maple mini ports
#define ADC_IN0 PA0
#define ADC_IN1 PA1
#define ADC_IN2 PA2
#define ADC_IN3 PA3
#define ADC_IN4 PA4
#define ADC_IN5 PA5
#define ADC_IN6 PA6
#define ADC_IN7 PA7
#define ADC_IN8 PA8  // reference voltage for differential channel recording

/********************************************************************/
// variables
/********************************************************************/
// set by HW when a complete DMA transfer was finished.
volatile uint8_t dma_irq_full_complete;
// set by HW when a DMA transfer is at its half.
volatile uint8_t dma_irq_half_complete;
// set by SW when overrun occurs
volatile uint8_t overrun;
//volatile uint32_t dma_irq_counter;
volatile uint32_t dma_isr;	// must not be global, make it local later
// signalling for lower and upper buffer status
volatile uint8_t buff0_stored, buff1_stored, buff_index;
/********************************************************************/
// file system
// use SPI 2 because SPI 1 pins are used as analog input
//SdFat sd(2); // use SPI port 2
SdFat sd;
// test file
#ifdef USE_FAT
SdFile file; // file class
#endif

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
	//dma_irq_counter = 0;
	dma_isr = 0;
	buff0_stored = buff1_stored = 1;	// avoid overrun detection
	buff_index = 0;
	dma_clear_isr_bits(DMA1, DMA_CH1);
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
		//if ( buff1_stored==0 )	overrun++;	// upper buffer half being written before was stored
	}
	if (dma_isr&DMA_ISR_TCIF1) {
		dma_irq_full_complete = 1;
		buff1_stored = 0;	// reset storage flag to detect overrun
		//if ( buff0_stored==0 )	overrun++;	// lower buffer half being written before was stored
	}
	dma_clear_isr_bits(DMA1, DMA_CH1);
}
/*****************************************************************************/
/*****************************************************************************/
void DMA_Setup(void)
{
	Serial.print("preparing the DMA...");
	DMA_Init();
	// turn DMA on
	dma_init(DMA1);
// DMA tube configuration
dma_tube_config my_tube_cfg = {
	&ADC1->regs->DR,	// data source address
	DMA_SIZE_32BITS,	// source transfer size
	&adc_buffer,		// data destination address 
	DMA_SIZE_32BITS,	// destination transfer size
	SAMPLES_PER_BUFFER,	// nr. of data to transfer
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

	Serial.println("done.");
}
/*****************************************************************************/
/*
 * calc_adc_sequence(ADCx_Sequence) converts the SQR3 6 channels' list into
 * a valid 6 X 5=30 bits sequence format and returns that 30 bits number.
 * There are 6 available sequences in each SQR3 SQR2, and 4 in SQR1.
 * For more channels, repeat the same for SQR2, SQR1. (For SQR1 4 channels only!)
 */
/*****************************************************************************/
void ADC_set_channel_sequences(void)
{
	uint32_t adc_sequence_low = 0;
	uint32_t adc_sequence_high = 0;
	for (int i = 0; i<samples_per_sequence; i++)	{
		if (i<6)	adc_sequence_low |= ADC1_Sequence[i] << (i * 5);
		else		adc_sequence_high |= ADC1_Sequence[i] << ((i-6) * 5);
	}
	ADC1->regs->SQR2 = adc_sequence_high;
	ADC1->regs->SQR3 = adc_sequence_low;
	adc_sequence_low = 0;
	adc_sequence_high = 0;
	for (int i = 0; i<samples_per_sequence; i++)	{
		if (i<6)	adc_sequence_low |= ADC2_Sequence[i] << (i * 5);
		else		adc_sequence_high |= ADC2_Sequence[i] << ((i-6) * 5);
	}
	ADC2->regs->SQR2 = adc_sequence_high;
	ADC2->regs->SQR3 = adc_sequence_low;
}
/*****************************************************************************/
/*****************************************************************************/
void ADC_Setup(void)
{
	Serial.print("setting up the ADC...");

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
	ADC_set_channel_sequences();

	ADC1->regs->CR1 = ( (0x06 << 16) | ADC_CR1_SCAN ); // b0110: Regular simultaneous mode only, enable SCAN
	ADC2->regs->CR1 = ( ADC_CR1_SCAN ); // enable SCAN, mode setting not possible for ADC2
	Serial.println("done.");
}
/*****************************************************************************/
int32_t bgnBlock, endBlock;
/*****************************************************************************/
// serial output steam
ArduinoOutStream cout(Serial);
// store error strings in flash
#define sdErrorMsg(msg) sd.errorPrint(F(msg));
/*****************************************************************************/
void SD_Init(void)
{
#ifdef USE_FAT
	// delete possible existing file
	if (sd.exists(FILE_NAME)) {
		Serial.print("deleting existing raw file...");
		if (!sd.remove(FILE_NAME)) {
			error("Can't remove raw file!");
		}
		Serial.println("done.");
	}
	file.close();
	// Create new file.
	Serial.print("creating new contiguous file...");
	sprintf(chr_buf, "total_blocks: %i, BLOCK_SIZE: %i...", total_blocks, BLOCK_SIZE); Serial.print(chr_buf);
	// create a contiguous file
	if (!file.createContiguous(sd.vwd(), FILE_NAME, (total_blocks*BLOCK_SIZE))) {
		error("createContiguous failed!");
	}
	// get the location of the file's blocks
	if (!file.contiguousRange(&bgnBlock, &endBlock)) {
		error("contiguousRange failed!");
	}
	// clear the cache and use it as a 512 byte buffer
	pCache = (uint8_t*)sd.vol()->cacheClear();
	// prepare writing contiguous blocks
	if (!sd.card()->writeStart(bgnBlock, endBlock)) {
		error("writeStart failed");
	}
#else
  #define START_BLOCK 1
	cout << ("starting raw write in blocks...");
	// clear the cache and use it as a 512 byte buffer
	pCache = (uint8_t*)sd.vol()->cacheClear();
	bgnBlock = 0;
	endBlock = total_blocks+START_BLOCK;
	if (!sd.card()->writeStart(bgnBlock, endBlock)) {
		error("writeStart failed");
	}
	cout << "bgnBlock: " << bgnBlock << ", endBlock: " << endBlock  << (", writing cache...");
	// write block 0 with recording parameters
	memset((uint8*)pCache,0,BLOCK_SIZE); // clear cache
	sprintf((char*)pCache, "sampling_frequency=%u;\nsamples_per_sequence=%u;\nrecords_per_sample=%u;\nendBlock=%u;\n",
							sampling_frequency, (samples_per_sequence), records_per_sample, endBlock);
	//cout << pCache; // show on serial what has been written
	if (!sd.card()->writeData(pCache) ) // write to card
		error("writing cache block 0 failed!");
#endif
	Serial.println("done.");
}

/*****************************************************************************/
// global for card size
uint32_t cardSize;
// global for card erase size
uint32_t eraseSize;
/*****************************************************************************/
uint8_t cidDmp() {
  cid_t cid;
  if (!sd.card()->readCID(&cid)) {
    sdErrorMsg("readCID failed");
    return false;
  }
  cout << F("\nManufacturer ID: ");
  cout << hex << int(cid.mid) << dec << endl;
  cout << F("OEM ID: ") << cid.oid[0] << cid.oid[1] << endl;
  cout << F("Product: ");
  for (uint8_t i = 0; i < 5; i++) {
    cout << cid.pnm[i];
  }
  cout << F("\nVersion: ");
  cout << int(cid.prv_n) << '.' << int(cid.prv_m) << endl;
  cout << F("Serial number: ") << hex << cid.psn << dec << endl;
  cout << F("Manufacturing date: ");
  cout << int(cid.mdt_month) << '/';
  cout << (2000 + cid.mdt_year_low + 10 * cid.mdt_year_high) << endl;
  cout << endl;
  return true;
}
//------------------------------------------------------------------------------
uint8_t csdDmp() {
  csd_t csd;
  uint8_t eraseSingleBlock;
  if (!sd.card()->readCSD(&csd)) {
    sdErrorMsg("readCSD failed");
    return false;
  }
  if (csd.v1.csd_ver == 0) {
    eraseSingleBlock = csd.v1.erase_blk_en;
    eraseSize = (csd.v1.sector_size_high << 1) | csd.v1.sector_size_low;
  } else if (csd.v2.csd_ver == 1) {
    eraseSingleBlock = csd.v2.erase_blk_en;
    eraseSize = (csd.v2.sector_size_high << 1) | csd.v2.sector_size_low;
  } else {
    cout << F("csd version error\n");
    return false;
  }
  eraseSize++;
  cout << F("cardSize: ") << 0.000512*cardSize;
  cout << F(" MB (MB = 1,000,000 bytes)\n");

  cout << F("flashEraseSize: ") << int(eraseSize) << F(" blocks\n");
  cout << F("eraseSingleBlock: ");
  if (eraseSingleBlock) {
    cout << F("true\n");
  } else {
    cout << F("false\n");
  }
  return true;
}
/*****************************************************************************/
void SD_Info(void)
{
	SPI.setModule(2); // remove for the new SD Fat beta

	uint32_t t = millis();
	cout << ("initializing the SD card...");
	// initialize the SD card
	//if ( !sd.begin(CHIP_SELECT, SPISettings(18000000)) ) { // SPI clock value
	if ( !sd.begin(CHIP_SELECT, SPI_CLOCK_DIV2) ) {
		//sd.initErrorHalt("card begin failed!"); // ignore FAT record error
	}
	cout << ("done.") << endl;
	t = millis() - t;
	
	cardSize = sd.card()->cardSize();
	if (cardSize == 0) {
		sd.errorPrint("cardSize failed");
	}
	cout << ("\ninit time: ") << t << " ms" << endl;
	cout << ("\nCard type: ");
	switch (sd.card()->type())
	{
		case SD_CARD_TYPE_SD1:			cout << F("SD1\n");	break;
		case SD_CARD_TYPE_SD2:			cout << F("SD2\n"); break;
		case SD_CARD_TYPE_SDHC:
			if (cardSize < 70000000)	cout << F("SDHC\n");
			else						cout << F("SDXC\n");
			break;
		default:						cout << F("Unknown\n"); break;
	}
	cidDmp();
	csdDmp();
#if 0
	// test sequence for the newly implemented compressed data storage
	uint32 * p = adc_buffer;
	Serial.print("ADC buffer lower half address: "); Serial.println((uint32)p, HEX);
	uint16 * p1 = (uint16*)adc_buffer+(ADC_BUFFER_SIZE/2);
	Serial.print("ADC buffer upper half  (16b*): "); Serial.println((uint32)p1, HEX);
	p = adc_buffer+(ADC_BUFFER_SIZE/2);
	Serial.print("ADC buffer upper half  (32b*): "); Serial.println((uint32)p, HEX);
//return;

	// fill the ADC buffer with test values
	p = adc_buffer;
	//int16 cnt[] = { 0x400, 0x500, 0x600, 0x700, 0x800, 0x900, 0xa00, 0xb00}; // start values
	int16 cnt[] = { 0x400, 0x600, 0x800, 0xa00}; // start values
	int16 dir[] = { 0, 1, 0, 1};//, 0, 1, 0, 1}; // count direction: 0=up, 1=down
	int16 diff[] = { 64, -64, 64, -64};//, 64, -64, 64, -64};  // difference values
	uint16 val;
	//for (uint16 i=0; i<(ADC_BUFFER_SIZE/4); i++) { // 8 channels
	for (uint16 i=0; i<(ADC_BUFFER_SIZE/2); i++) { // 4 channels
		// prepare test data
		//for (uint8 j = 0; j<8; j++) { // 8 channels
		for (uint8 j = 0; j<4; j++) { // 4 channels
			val = (cnt[j] + diff[j]);
			if ( (val)<0x200 ) {val=0x200; dir[j]=0; diff[j]=0;}
			if ( (val)>0xd00 ) {val=0xd00; dir[j]=1; diff[j]=0;}
			cnt[j] = val;
			if ( dir[j]==0 ) { // value up
				if ( (diff[j]+=10)>127 ) { diff[j] = 127; }
			} else {
				if ( (diff[j]-=10)<-128 ) { diff[j] = -128; }
			}
		}
		// write test data to adc_buffer
		*p++ = (uint32)(cnt[1]<<16) | cnt[0];
		*p++ = (uint32)(cnt[3]<<16) | cnt[2];
		//*p++ = (uint32)(cnt[5]<<16) | cnt[4];
		//*p++ = (uint32)(cnt[7]<<16) | cnt[6];
	}
	total_blocks = 4;
	SD_Init();
	p = adc_buffer;
	// write the first 512 byte block to card
	if (!sd.card()->writeData((const uint8_t*)p)) {
		error("writing lower ADC buffer failed");
	}
	p = adc_buffer+(ADC_BUFFER_SIZE/2);
	// write the second 512 byte block to card
	if (!sd.card()->writeData((const uint8_t*)p)) {
		error("writing upper ADC buffer failed");
	}
	// compress data from adc_buffer to cache and write cache to card 
	SD_buffer_to_card(0);
	SD_buffer_to_card(1);
	// end multiple block write mode
	if (!sd.card()->writeStop()) {
		Serial.println(F("\nERROR: sd.card->writeStop failed!"));
		error("writeStop failed");
	}
	Serial.println("Test data writen to card.");
#endif
}
/*****************************************************************************/
/*****************************************************************************/
void setup()
{
	//Serial.begin(57600);
	Serial.begin(500000);

#ifdef DEBUG_PIN
	pinMode(DEBUG_PIN, OUTPUT);
	digitalWrite(DEBUG_PIN, 0);
#endif
	pinMode(LED_PIN, OUTPUT);
	while ( !Serial.isConnected() ) Blink(500,500);
	
	cout << F("\n***** ADC dual regular simultaneous mode acquisition *****\n");
#ifdef DEBUG_PIN
	cout << F("Debug version!\n");
#endif

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
	pinMode(ADC_IN8, INPUT_ANALOG);
	
	SD_Info();

#ifdef USE_FAT
  #if 0
	// for test only. write the existing file with dummy data
	// this way one can download data even before making acquisition
	if ( sd.exists(FILE_NAME) ) {
		// init file with dummy data
		if ( !file.open(FILE_NAME, O_WRITE) && !file.open(FILE_NAME, O_WRITE) ) {
			cout << F("ERROR: could not open file to write!") << endl;
			sd.errorPrint();
			return;
		}
		file.rewind();
		uint32_t fsize = file.fileSize();
		cout << F("Writing ") << fsize << F(" dummy data...");
		for (uint32 i=0; i<fsize/2; i++) {
			file.write(i>>8);
			file.write((byte)i);
		}
		cout << F("done.");
		file.close();
		file.close();
	}
  #endif
#endif // USE_FAT
}
/*****************************************************************************/
bool start = false;
/*****************************************************************************/
void SetupModules()
{
//debug outputs
	cout << F("Input parameters:\n");
	cout << F("recording_time [ms]: ") << recording_time << endl;
	cout << F("sampling_frequency [Hz]: ") << sampling_frequency << endl;
	cout << F("samples_per_sequence: ") << (samples_per_sequence) << endl;

// set-up involved hardware modules
	TIMER_Setup();
	ADC_Setup();
	DMA_Setup();
	start = false;	// stop after first run
}
/*****************************************************************************/
#define BUFFER_LENGTH		512
char rec_buff[BUFFER_LENGTH];
/*****************************************************************************/
void ParseParams(char * p)
{
	//Serial.print(F("parsing: ")); Serial.println(p);
	if ( strstr(p,"rec_time")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial.print(F(">> rec_time = ")); Serial.println(val);
			recording_time = (uint16_t)val;
		}
	} else
	if ( strstr(p,"sampling_freq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial.print(F(">> sampling_freq = ")); Serial.println(val);
			sampling_frequency = val;
		}
	} else
	if ( strstr(p,"samples_per_seq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial.print(F(">> samples_per_seq = ")); Serial.println(val);
			samples_per_sequence = val;
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
			//Serial.println(F("> set detected..."));
			while ( (ptr=strtok(NULL, " ;"))>0 ) // get parameters
				ParseParams(ptr);
		}
	}
	rec_buff[0] = 0;
}
/*****************************************************************************/
void Blink(int ms_on, int ms_off)
{
	digitalWrite(LED_PIN,0);
	delay(ms_on);
	digitalWrite(LED_PIN,1);
	delay(ms_off);
}
/*****************************************************************************/
/*****************************************************************************/
void BlinkError()
{
	for (int i=0;i<5;i++) Blink(100,100);
}
/*****************************************************************************/
/*****************************************************************************/
int8 COM_RecAck(void)
{	// wait for 0x06 acknowledge
	byte rec;
	uint32_t tim = millis();
	while ( 1 ) {
		if ( Serial.available() ) {
			rec = Serial.read();
			//Blink(100,0);
			if ( rec==0x06 ) return 1;
			else return 0;
		}
		if ( (millis()-tim)>2000) {
			//Serial.println("ERROR: ACK reception timed out!");
			BlinkError();
			return 0;
		}
	}
}
/*****************************************************************************/
int rec_index = 0;
bool str_mode = true;
/*****************************************************************************/
byte SerialReadBytes()
{
#define SERIAL_RX_TIMEOUT	100	// millis
	// wait for serial data
	rec_buff[0] = 0;	// mark start of string
	//if ( Serial.available()<=0 ) return 0;
	char tmp;
	rec_index = 0;
#if 1
	uint32_t ts = millis();	// prepare reception time-out
	while ( (millis()-ts)<SERIAL_RX_TIMEOUT ) {	// no time-out occurred yet
		if ( Serial.available()<=0 ) continue;
		tmp = Serial.read();
		if (str_mode==true) {
			rec_buff[rec_index++] = tmp;
		} else {
			//COM_RecData(tmp);
		}
		ts = millis();	// reset time-out to receive next byte
	}
#else
	Serial.setTimeout(SERIAL_RX_TIMEOUT);
	rec_index = Serial.readBytesUntil('\n', rec_buff, BUFFER_LENGTH);
#endif
	if ( rec_index==0 ) return 0;	// nothing was received
	if ( str_mode ) {
		rec_buff[rec_index] = 0;	// mark end of string
		if ( rec_buff[rec_index-1]=='\n' ) rec_buff[rec_index-1] = 0;	// clear '\n'
		Serial.print(F("> received: ")); Serial.println(rec_buff);// Serial.println(F(" chars."));
	}
	return rec_index;
}
/*****************************************************************************/
/*****************************************************************************/
void CheckSerial()
{
	// read any existing Serial data
	while ( Serial.available()>0 ) Serial.read();
	//delay(10);
	cout << F("\n***** Accepted commands *****\n");
	cout << F("\tSet up recording parameters: set rec_time=") << recording_time << \
			F(";sampling_freq=") << sampling_frequency << \
			F(";samples_per_seq=") << (samples_per_sequence) << endl;
	cout << F("\tStart recording: go\\n\n");
	cout << F("\tDump recorded data: get\\n\n*****\n");
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
	//	copy data from ADC buffer to SD cache
	uint32_t * sp = adc_buffer+(buf*SAMPLES_PER_BLOCK);
#if 1
	// write a 512 byte block from ADC buffer directly to card
	if (!sd.card()->writeData((const uint8_t*)sp)) {
		error("writeData failed");
	}
#else
	uint32_t * dp = (uint32_t*)pCache;
	if ( use_diff_channels ) {
		dp += (buf*SAMPLES_PER_BLOCK/2);	// adjust destination pointer for second half cache
		uint32_t tmp = 0;
		//	copy data from source buffer to cache
		for ( uint16_t i=0; i<SAMPLES_PER_BLOCK; i++ ) {
			uint32_t val = *sp++;
			val = (val&0x0000FFFF) - (val/0x10000);	// ADC0 value (low word) - ADC1 value (high word)
			if ( (i&0x0001) )
				*dp++ = (uint32_t)(val<<16) + tmp;	// store value combined with that of previous reading
			else
				tmp = val&0x0000FFFF;	// save the lower word for the next writing
		}
		if ( buf==0 ) return;	// store only even blocks
	} else {
		for ( int i=0; i<SAMPLES_PER_BLOCK; i++ )	{
			*dp++ = *sp++;	// optimized copy using destination and source pointers
		}
	}
//	uint32_t tw = micros();	// measure time needed for writing to card
	// write a 512 byte block cache to card
	if (!sd.card()->writeData((const uint8_t*)pCache)) {
		error("writeData failed");
	}
//	tw = micros() - tw;
#endif
}
/*****************************************************************************/
/*****************************************************************************/
uint8_t DMA_CheckStatus(void)
{
	// check number of sampled and transferred data
	uint32_t cndtr1 = DMA1->regs->CNDTR1;	// remaining ADC samples to be written to ADC buffer
	if ( cndtr1==0 ) cndtr1++; // avoid head_bl to get the value of BLOCKS_PER_BUFFER as index
	cndtr1 = SAMPLES_PER_BUFFER-cndtr1;	// number of finished ADC samples
	uint16_t head_bl = cndtr1/SAMPLES_PER_BLOCK; // number of block which is currently being written by DMA
	uint16_t tail_bl = block_nr&BLOCKS_PER_BUFFER_MASK; // number of block which should be next written to SD card
	return (head_bl-tail_bl)&BLOCKS_PER_BUFFER_MASK; // blocks ready to be written to card
}
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

	uint16 dma_irq_full_complete_count = 0;
	uint16 dma_irq_half_complete_count = 0;

	uint32 trun, tout, tstart = micros();
	// let timer run - do this just before generating an update trigger by SW
	timer_resume(TIMER3);
	// from now on, the sampling of sequences should be automatically triggered by Timer3 update event.
	// sample until number of input blocks was filled with data
	for(block_nr=0; block_nr<total_blocks; ) {
		// time-out to avoid hangup if something goes wrong
		trun = millis();
		tout = 0;
		// each ADC sequence is sampled triggered by TIMER3 event.
		while ( tout==0 ) {
			/**/
			if ( dma_irq_half_complete ) {
				dma_irq_half_complete = 0;
				dma_irq_half_complete_count++;
			}
			if ( dma_irq_full_complete ) {
				dma_irq_full_complete = 0;
				dma_irq_full_complete_count++;
			}
			// check the end of filling a complete block: head_bl must be different to tail_bl
			if ( DMA_CheckStatus()>0 ) {
				SD_buffer_to_card(block_nr&BLOCKS_PER_BUFFER_MASK);	// store next block of data to card
				// check overflow
				if ( DMA_CheckStatus()==0 ) { overrun++; } // the block being saved to card is being overwritten by DMA
				block_nr ++;
				break;
			}
			if ( (millis()-trun)>1000 ) tout = 1;
		}
		// only for debug purposes:
		if ( tout>0 ) {
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
#ifdef USE_FILE
	// close file for next pass of loop
	file.close();
#endif
	Serial.println(F("done."));
	// print statistics
	Serial.print(F("Overruns: ")); Serial.println(overrun);
	Serial.print(F("Elapsed total time (useconds): ")); Serial.println(t);
	Serial.print(F("Recorded blocks: "));  Serial.println(block_nr);	//TOTAL_BLOCKS
	Serial.print(F("Recording time for 1 block (useconds): ")); Serial.println(t/(double)block_nr);
	Serial.print(F("Number of records: "));  Serial.println(total_records);	//TOTAL_SEQUENCES
	Serial.print(F("Time for 1 record (useconds): ")); Serial.println(t/(double)total_records);
/**/
	Serial.print(F("dma_irq_half_complete_count: ")); Serial.println(dma_irq_half_complete_count);
	Serial.print(F("dma_irq_full_complete_count: ")); Serial.println(dma_irq_full_complete_count);

	// OPTIONAL: print last samples from ADC buffer to check validity
	//SendData();
}
/***************************************************************************/
#define BUFF_SIZE	(2*BLOCK_SIZE) // 1024 bytes, experimental value for optimal upload speed
uint8 buf[BUFF_SIZE] __attribute__ ((aligned(1)));
/*****************************************************************************/
void TransmitBinaryData(void)
{
	uint16 c;
	// load and send the binary recorded data to serial
#ifdef USE_FAT
	// take data saved in a file 
	if ( !file.open(FILE_NAME, O_READ) && !file.open(FILE_NAME, O_READ) ) {
		Serial.println(F("ERROR: could not open file to read!"));
		sd.errorPrint();
		return;
	}
	uint32_t fsize = file.fileSize();
	
	Serial.print("binary_length: "); Serial.println(fsize);	// send binary data length

	Serial.print(">>>");	// send binary start marker
	if ( COM_RecAck()==0 ) {
		goto ret_1;	//Serial.write(0x05);	// send binary enquiry
	}

	int8 repeat;
	while ( (c=file.read(d_buff, BUFF_SIZE))>0 ) {
		repeat = 2;
		// {
			Serial.write(0x01);	// start header
			Serial.write(byte(c)); // low byte
			Serial.write(c>>8); // high byte
			Serial.write(d_buff, c);
		//} while ( COM_RecAck()==0 && (repeat--)>0);
		if (repeat<0) break;
	}
ret_1:
	file.close();
#else
	// Raw read: read block-wise from block 0 to endBlock
	sd.card()->readStart(bgnBlock); // start reading
	sd.card()->readData(buf); // read first block, which is block 0
	// detect endBlock value from block 0
	char * ptr;
	// search for the variable endBlock
	if ( (ptr=strstr((const char*)buf, "endBlock"))>0 ) {
		endBlock = atoi((const char*)ptr+9);
	} else {
		cout << F("Could not read endBlock.\n");
		return;
	}
	cout << ("binary_length: ") << ((endBlock*BLOCK_SIZE)) << endl;	// send binary data length
	cout << (">>>\n");	// send binary start marker
	if ( COM_RecAck()==0 ) {
		goto ret_2;	//Serial.write(0x05);	// send binary enquiry
	}
	// send here block 0
	Serial.write(0x01);	// start header
	c = BLOCK_SIZE;
	Serial.write(byte(c)); // low byte
	Serial.write(c>>8); // high byte
	Serial.write(buf, c);
	endBlock--;
	// send here the rest of data
	c = BUFF_SIZE;
	while ( endBlock>0 ) {
		// read two consecutive buffers
		sd.card()->readData(buf); // read first block
		sd.card()->readData(buf+BLOCK_SIZE); // read second block
		// transmitting to the serial
		Serial.write(0x01);	// start header
		Serial.write(byte(c)); // low byte
		Serial.write(c>>8); // high byte
		Serial.write(buf, c);
		endBlock -= 2;
	}
	sd.card()->readStop();
ret_2:
#endif

	Serial.write(0x17); // end of binary transmission
	str_mode = true;	// switch back to string mode
}

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

	str_mode = true;	// switch back to string mode
}

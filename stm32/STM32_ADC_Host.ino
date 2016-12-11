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
//#define LED_PIN		PB1 // maple mini
#define LED_PIN		PC13 // generic mini
/********************************************************************/
// defines
/********************************************************************/
// DO NOT CHANGE !!!
/********************************************************************/
// size of data block which will be written in one shot to card
#define BLOCK_SIZE				512	// bytes
#define SAMPLE_SIZE				2	// dual 16 bits, combined value of ADC2 (high word) and ADC1 (low word)
#define SAMPLES_PER_BLOCK		(BLOCK_SIZE/SAMPLE_SIZE)
#define BLOCKS_PER_BUFFER		16
#define BLOCKS_PER_BUFFER_MASK	(BLOCKS_PER_BUFFER-1)
#define SAMPLES_PER_BUFFER		(BLOCKS_PER_BUFFER*SAMPLES_PER_BLOCK)	// the buffer size is twice the block size
// some default values
#define RECORD_SIZE				2	// 16 bits result of an ADC conversion
#define RECORDS_PER_BLOCK		(BLOCK_SIZE/RECORD_SIZE)

// size of data buffer where the ADC sampled data will be transferred over DMA
#define ADC_BUFFER_SIZE			(SAMPLES_PER_BUFFER)
// the data buffer to store ADC sample values
uint16_t adc_buffer[ADC_BUFFER_SIZE] __attribute__ ((aligned(2)));

uint32_t records_per_sample;
uint32_t total_blocks;
uint32_t total_sequences;
uint32_t total_samples;
uint32_t total_records;
uint32_t block_nr;

// timing related constants and variables
#define TIMER_PRESCALER			4	// to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY			(72000000/TIMER_PRESCALER)
uint32_t timer_reload_value;
/********************************************************************/
// file system
SdFat sd(2); // use SPI 2 to access SD card
#define error(s) sd.errorHalt(F(s))

uint8_t * pCache;
#define SPI1_CS_PIN PA4
/********************************************************************/
// variables
/********************************************************************/
// set by SW when overrun occurs
volatile uint8_t overrun;

char sBuf[250];
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
void DMA_Setup(void)
{
	Serial.print("preparing the DMA...");

	overrun = 0;
	dma_init(DMA1);	// turn DMA on
// DMA tube configuration for SPI1 Rx - channel 2
dma_tube_config my_tube_cfg = {
	&SPI1->regs->DR,	// data source address
	DMA_SIZE_16BITS,	// source transfer size
	&adc_buffer,		// data destination address 
	DMA_SIZE_16BITS,	// destination transfer size
	SAMPLES_PER_BUFFER,	// nr. of data to transfer
	// tube flags: auto increment dest addr, circular buffer, very high prio:
	( DMA_CFG_DST_INC | DMA_CFG_CIRC |  DMA_CCR_PL_VERY_HIGH ), // interrupt is not used
	0,	// unused
	DMA_REQ_SRC_SPI1_RX,	// Hardware DMA request source
};
	// configure DMA channel
	int ret = dma_tube_cfg(DMA1, DMA_CH2, &my_tube_cfg);	// SPI2 Rx channel is nr. 4
	if ( ret<0 ) {
		Serial.print("FATAL ERROR: halt due to DMA configuration error: "); Serial.println(ret,HEX);
		while ( 1 );	// halt
	}
	// Enable the DMA tube. It will begin serving requests only after SPI_DMA enable.
	dma_enable(DMA1, DMA_CH2);

	Serial.println("done.");
}
/*****************************************************************************/
void SPI_setup(void)
{
	Serial.print(("initializing SPI1..."));
	// use SPI 1 for recording, SPI 2 is used by SD card
	SPI.beginTransactionSlave(SPISettings(18000000, MSBFIRST, SPI_MODE0, DATA_SIZE_16BIT));
	Serial.println(("done."));
}
/*****************************************************************************/
void REGS_info(void)
{
	Serial.print("DMA1.ISR: "); Serial.println(DMA1->regs->ISR, HEX);
	Serial.print("DMA1.CCR2: "); Serial.println(DMA1->regs->CCR2, HEX);
	Serial.print("DMA1.CNDTR2: " ); Serial.println(DMA1->regs->CNDTR2, HEX);
	Serial.print("SPI1.CR1: "); Serial.println(SPI1->regs->CR1, HEX);
	Serial.print("SPI1.CR2: "); Serial.println(SPI1->regs->CR2, HEX);
	Serial.print("SPI1.SR: "); Serial.println(SPI1->regs->SR, HEX);
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
	cout << ("starting raw write in blocks...");
	// clear the cache and use it as a 512 byte buffer
	pCache = (uint8_t*)sd.vol()->cacheClear();
	bgnBlock = 0;
	endBlock = total_blocks+1; // data is stored starting with block 1
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
	uint32_t t = millis();
	cout << ("initializing the SD card...");
	// initialize the SD card
	if ( !sd.begin(CHIP_SELECT, SPISettings(18000000)) ) { // SPI clock value
		//sd.initErrorHalt("card begin failed!"); // ignore error in case of RAW read/write
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
}
/*****************************************************************************/
void SD_End(void)
{
	spi_rx_dma_disable(SPI1);
	// end multiple block write mode
	if (!sd.card()->writeStop()) {
		Serial.println("\nERROR: sd.card->writeStop failed!");
		error("writeStop failed");
	}
}
/*****************************************************************************/
void setup()
{
	Serial.begin(); // PC
	Serial1.begin(1000000); // ADC slave port
	while( !Serial.isConnected() ); // wait for USB serial connection 

	Serial.println(("\n***** Multi-channel analog acquisition application *****\n"));
	Serial.println(("ADC data received via SPI1 in slave mode is stored to SD-card via SPI2."));

#if defined DEBUG_PIN
	pinMode(DEBUG_PIN, OUTPUT);
	Serial.println(("Debug version!"));
#endif
	SD_Info();
}
/*****************************************************************************/
bool start = false;
/*****************************************************************************/
void SetupModules()
{	// set-up involved hardware modules
//	REGS_info();
	SD_Init();
	DMA_Setup();
	SPI_setup();
//	REGS_info();
	start = false;	// stop after first run
}
/*****************************************************************************/
void SD_buffer_to_card(byte buf)
{
	// set current read pointer
	uint16_t * sp = adc_buffer+(buf*SAMPLES_PER_BLOCK);
	// write a 512 byte raw data block from ADC buffer directly to card
	if (!sd.card()->writeData((const uint8_t*)sp)) {
		error("writeData failed");
	}
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
int8 COM_RecAck(void)
{	// wait for 0x06 acknowledge
	byte rec;
	uint32_t tim = millis();
	while ( (millis()-tim)<2000 ) {
		if ( Serial.available() ) {
			rec = Serial.read();
			//Blink(100,0);
			if ( rec==0x06 ) { return 1; }
			else {
				sprintf(sBuf, "ERROR: wrong ACK byte received: 0x%x", rec);
				goto ret_fail;
			}
		}
	}
	sprintf(sBuf, "ERROR: ACK reception timed out!");
ret_fail:
	BlinkError();
	return 0;
}
/*****************************************************************************/
#define BUFF_SIZE	(2*BLOCK_SIZE) // 1024 bytes, experimental value for optimal upload speed
uint8 buf[BUFF_SIZE] __attribute__ ((aligned(1)));
bool str_mode = true;
/*****************************************************************************/
void TransmitBinaryData(void)
{
	sBuf[0] = 0;
	// load and send the binary recorded data to serial
	// Raw data reading block-wise from block 0 to endBlock
	sd.card()->readStart(bgnBlock); // start reading
	sd.card()->readData(buf); // read first block, which is block 0 with recording parameters
	// detect endBlock value from block 0
	char * ptr;
	// search for the variable endBlock
	if ( (ptr=strstr((const char*)buf, "endBlock"))>0 ) {
		endBlock = atoi((const char*)ptr+9);
	} else {
		cout << ("Could not read endBlock.\n");
		return;
	}
	cout << ("binary_length: ") << (endBlock*BLOCK_SIZE) << endl;	// send binary data length
	cout << (">>>\n");	// send binary start marker
	if ( COM_RecAck()!=0 ) {
		// send here block 0
		Serial.write(0x01);	// start header
		uint16 c = BLOCK_SIZE;
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
	}

	Serial.write(0x17); // end of binary transmission
	str_mode = true;	// switch back to string mode
	if ( sBuf[0]>0 ) { delay(500); Serial.println(sBuf); }
}
/*****************************************************************************/
#define BUFFER_LENGTH		200
char rec_buff[BUFFER_LENGTH] __attribute__ ((aligned(1)));
/*****************************************************************************/
void ParseToken(char * p)
{
	//Serial.print("parsing: "); Serial.println(p);
	if ( strstr(p,"time")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial.print(">> rec_time = "); Serial.println(val);
			recording_time = (uint16_t)val;
		}
	} else
	if ( strstr(p,"freq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1)*1000;
			Serial.print(">> sampling_freq = "); Serial.println(val);
			sampling_frequency = val;
		}
	} else
	if ( strstr(p,"seq_1")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			p++;
			Serial.print(">> seq_1 = "); Serial.println(p);
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
			Serial.print(">> seq_2 = "); Serial.println(p);
			// store the channels of sequence 2
			uint16_t i;
			for (i = 0; i<8; i++) {
				(p++);
				if (*p++!=',')	break;
			}
			if ( samples_per_sequence != i)	Serial.print("__Error: invalid seq_2 length!__");
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
/*****************************************************************************/
byte SerialReadBytes()
{
#define SERIAL_RX_TIMEOUT	100	// millis
	// wait for serial data
	rec_buff[0] = 0;	// mark start of string
	//if ( Serial1.available()<=0 ) return 0;
	byte rec_index = 0;
	// read bytes
	uint32_t ts = millis();	// prepare reception time-out
	while ( (millis()-ts)<SERIAL_RX_TIMEOUT ) {	// no time-out occurred yet
		if ( Serial.available()<=0 ) continue;
		rec_buff[rec_index++] = Serial.read();
		ts = millis();	// reset time-out to receive next byte
	}
	if ( rec_index==0 ) return 0;	// nothing was received
	rec_buff[rec_index] = 0;	// mark end of string
	if ( rec_buff[rec_index-1]=='\n' ) rec_buff[rec_index-1] = 0;	// clear '\n'
	if ( str_mode ) {
		Serial.print("> received: "); Serial.println(rec_buff);// Serial.println(" chars.");
	}
	return rec_index;
}
/*****************************************************************************/
/*****************************************************************************/
void CheckSerial()
{
	Serial.println("\n***** Accepted commands *****");
	Serial.print("\tSet up recording parameters: set rec_time="); Serial.print(recording_time);
		Serial.print(";sampling_freq="); Serial.print(sampling_frequency/1000);
		Serial.print(";samples_per_seq="); Serial.println(samples_per_sequence);
	Serial.println("\tStart recording: go\\n");
	Serial.println("\tDump recorded data: get\\n\n*****");
	rec_buff[0] = 0;
	while ( !start ) {
		if ( SerialReadBytes() )	// parse received serial data
			ParseRxData();
	}
}
/*****************************************************************************/
/*****************************************************************************/
uint8_t DMA_CheckStatus(void)
{
	//REGS_info(); // debug
	// check number of sampled and transferred data
	uint32_t cndtr2 = DMA1->regs->CNDTR2;	// remaining ADC samples to be written to ADC buffer
	if ( cndtr2==0 ) cndtr2++; // avoid head_bl to get the value of BLOCKS_PER_BUFFER as index
	cndtr2 = SAMPLES_PER_BUFFER-cndtr2;	// number of finished ADC samples
	uint16_t head_bl = cndtr2/SAMPLES_PER_BLOCK; // number of block which is currently being written by DMA
	uint16_t tail_bl = block_nr&BLOCKS_PER_BUFFER_MASK; // number of block which should be next written to SD card
	return (head_bl-tail_bl)&BLOCKS_PER_BUFFER_MASK; // blocks ready to be written to card
}
/*****************************************************************************/
void loop()
{
	str_mode = true;	// receive in string mode
	CheckSerial();	// try to receive setup parameters and/or commands
	SetupParameters();
	// prepare modules, including card
	SetupModules();
	Serial.print("-> sampling started...");
	// wait for any current transmission to end 
	while ( !digitalRead(SPI1_CS_PIN) ) ; // wait till high
	// enable data storage here
	spi_rx_dma_enable(SPI1);

	uint32 trun, tout, tstart = micros();

	// from now on, the samples received over SPI1 are stored in the ADC_BUFFER via DMA1 channel 2.
	// sample until number of input blocks was filled with data
	for(block_nr=0; block_nr<total_blocks; block_nr ++) {
		// time-out to avoid hangup if something goes wrong
		trun = millis();
		tout = 0; // time-out
		while ( tout==0 ) {
			// check the end of filling a complete block: currently receiving block must differ from next block to be stored
			if ( DMA_CheckStatus()>0 ) {
				SD_buffer_to_card(block_nr&BLOCKS_PER_BUFFER_MASK);	// store next block of data to card
				// check overflow
				if ( DMA_CheckStatus()==0 ) { overrun++; } // the block being saved to card is being overwritten by DMA
				break;
			}
			if ( (millis()-trun)>1000 ) tout = 1;
		}
		// only for debug purposes:
		if ( tout>0 ) {
			Serial.println("\n!!!! TIME_OUT !!!"); break;
		}
	}
	long t = micros()-tstart;	// measure time

	SD_End(); // stop data storage
	
	Serial.println("done.");
	// print statistics
	Serial.print("Overruns: "); Serial.println(overrun);
	Serial.print("Elapsed total time (useconds): "); Serial.println(t);
	Serial.print("Recorded blocks: ");  Serial.println(block_nr);	//TOTAL_BLOCKS
	Serial.print("Recording time for 1 block (useconds): "); Serial.println(t/(double)block_nr);
	Serial.print("Number of records: ");  Serial.println(total_records);	//TOTAL_SEQUENCES
	Serial.print("Time for 1 record (useconds): "); Serial.println(t/(double)total_records);
/**/
}

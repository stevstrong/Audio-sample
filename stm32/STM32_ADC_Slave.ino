/*
	=================================================================================
	ADC conversion of multiple channels in dual regular simultaneous conversion mode.
	=================================================================================
	Each conversion sequence is started by timer 3 update event (up-counting mode).
	The result of each dual sample (32bits) is stored into the ADC buffer via DMA transfer.
	The ADC buffer is containing up to 8 samples of one complete sequence.
	After the buffer is filled, the samples are sent to SPI.
	The acquisition time, frequency and the samples of the sequences is configurable.
	---------------------------------------------------------------------------------
*/


#include <libmaple/adc.h>
#include <libmaple/dma.h>
#include <libmaple/gpio.h>
#include <SPI.h>

//#define Serial1 Serial1

/********************************************************************/
// Configuration
/********************************************************************/
// setup recording parameters
uint16_t recording_time = 1000;			// in milliseconds
uint32_t sampling_frequency = 20000;	// in Hz
// Set the ADC input channel sequences (left will be first)
#define SAMPLES_PER_SEQUENCE 8
#define RECORDS_PER_SEQUENCE (SAMPLES_PER_SEQUENCE*2)
// the main buffer to store ADC sample values
uint32_t adc_buffer[SAMPLES_PER_SEQUENCE] __attribute__ ((packed,aligned(4)));
uint32_t total_sequences;
uint32_t timer_reload_value;
// channel 17 = Vrefint, only available on ADC1, can be used for tests
uint8_t sequence_1[SAMPLES_PER_SEQUENCE] __attribute__ ((packed)) = {0,1,2,3,4,5,6,7};
uint8_t sequence_2[SAMPLES_PER_SEQUENCE] __attribute__ ((packed)) = {8,8,8,8,8,8,8,8};
uint8_t samples_per_sequence = SAMPLES_PER_SEQUENCE;
uint8_t records_per_sequence = RECORDS_PER_SEQUENCE;
/********************************************************************/
// variables
/********************************************************************/
volatile uint8_t dma_irq_full_complete;	// set by HW when a complete DMA transfer was finished.
volatile uint8_t overrun;	// set by SW when overrun occurs
volatile uint8_t buff_sent;	// overrun management

/*****************************************************************************/
/*****************************************************************************/
// use for debug, comment out when not needed
#define DEBUG_PIN		PA8
//#define DEBUG_PIN_SET	( GPIOA->regs->BSRR = (1U << 8) )
//#define DEBUG_PIN_CLEAR	( GPIOA->regs->BSRR = (1U << 8) << 16 )
#define NSS_PIN	PB12
#define NSS_SET	( GPIOB->regs->BSRR = (1U << 12) )
#define NSS_CLEAR	( GPIOB->regs->BRR = (1U << 12) )
/********************************************************************/
// defines
/********************************************************************/
// DO NOT CHANGE !!!
/********************************************************************/
// size of data block which will be written in one shot to card
//#define SAMPLE_SIZE				4	// bytes
#define sample_t uint32_t
// timing related constants
#define TIMER_PRESCALER			4	// to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY			(72000000/TIMER_PRESCALER)
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
/********************************************************************/
void SetupParameters() {
	samples_per_sequence = 2;
	records_per_sequence = 2*samples_per_sequence;
	//total_sequences = sampling_frequency*recording_time/1000;
	// timing related values
	timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1;
}
/********************************************************************/
//extern void gpio_set_mode(gpio_dev *dev, uint8 pin, gpio_pin_mode mode);
//extern void gpio_toggle_bit(gpio_dev *dev, uint8 pin);
/*****************************************************************************/
/*****************************************************************************/
void TIMER_Setup(void)
{
	timer_pause(TIMER3);	// stop timer, to stop ADC triggering
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
	overrun = 0;
	buff_sent = 1;	// avoid overrun detection
	dma_clear_isr_bits(DMA1, DMA_CH1);
	// for test only: fill ADC buffer with dummy data
	for (int i = 0; i<SAMPLES_PER_SEQUENCE; ) adc_buffer[i++] = 0x11111111;
}
/*****************************************************************************/
/*****************************************************************************/
void DMA_RX_ISR(void)
{
// Used to store DMA interrupt status register (ISR) bits. This helps explain what's going on
	uint32_t dma_isr = dma_get_isr_bits(DMA1, DMA_CH1);
	if (dma_isr&DMA_ISR_TCIF1) {
		dma_irq_full_complete = 1;
		if ( buff_sent==0 )	overrun++;	// lower buffer half being written before was stored
		buff_sent = 0;
	}
	dma_clear_isr_bits(DMA1, DMA_CH1);
}
/*****************************************************************************/
dma_tube_config my_tube_cfg = {
	&ADC1->regs->DR,	// data source address
	DMA_SIZE_32BITS,	// source transfer size
	&adc_buffer,		// data destination address 
	DMA_SIZE_32BITS,	// destination transfer size
	SAMPLES_PER_SEQUENCE,	// nr. of data to transfer
	// tube flags: auto increment dest addr, circular buffer, set tube full IRQ, very high prio:
	( DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_CMPLT_IE | DMA_CCR_PL_VERY_HIGH ),
	0,	// unused
	DMA_REQ_SRC_ADC1,	// Hardware DMA request source
};
/*****************************************************************************/
void DMA_Setup(void)
{
// DMA tube configuration

	Serial1.print(F("preparing the DMA..."));
	DMA_Init();
	dma_disable(DMA1, DMA_CH1);	// disable the DMA tube
	dma_init(DMA1);	// turn DMA on
	my_tube_cfg.tube_nr_xfers = samples_per_sequence;	// configure DMA nr of transfers
	int ret = dma_tube_cfg(DMA1, DMA_CH1, &my_tube_cfg);
	if ( ret>0 ) {
		Serial1.print(F("DMA configuration error: ")); Serial1.println(ret,HEX);
		Serial1.print(F("Stopped, reset is needed!"));
		while ( 1 );	// halted
	}
	dma_attach_interrupt(DMA1, DMA_CH1, DMA_RX_ISR);	// attach an interrupt handler.
	dma_enable(DMA1, DMA_CH1);	// Enable the DMA tube. It will now begin serving requests.

	Serial1.println(F("done."));
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
		if (i<6)	adc_sequence_low |= sequence_1[i] << (i * 5);
		else		adc_sequence_high |= sequence_1[i] << ((i-6) * 5);
	}
	ADC1->regs->SQR2 = adc_sequence_high;
	ADC1->regs->SQR3 = adc_sequence_low;
	adc_sequence_low = 0;
	adc_sequence_high = 0;
	for (int i = 0; i<samples_per_sequence; i++)	{
		if (i<6)	adc_sequence_low |= sequence_2[i] << (i * 5);
		else		adc_sequence_high |= sequence_2[i] << ((i-6) * 5);
	}
	ADC2->regs->SQR2 = adc_sequence_high;
	ADC2->regs->SQR3 = adc_sequence_low;
}
/*****************************************************************************/
// write data from ADC buffer to SPI interface in 16 bit mode
/*****************************************************************************/
void ADC_buffer_to_SPI(void)
{
	uint32_t tmp;
	NSS_CLEAR;	// activate SS
	for (uint16_t i = 0; i<samples_per_sequence; i++) {
		tmp = adc_buffer[i];
		SPI2->regs->DR = (uint16_t)tmp;	// ADC1 value
		while ( (SPI2->regs->SR & SPI_SR_TXE) == 0);
		SPI2->regs->DR = tmp>>16;	// ADC2 value
		while ( (SPI2->regs->SR & SPI_SR_TXE) == 0);
	}
	while ( (SPI2->regs->SR & SPI_SR_BSY) != 0);
	NSS_SET;	// release SS
}
/*****************************************************************************/
/*****************************************************************************/
void ADC_Setup(void)
{
	Serial1.print(F("setting up the ADC..."));

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
	Serial1.println(F("done."));
}
/*****************************************************************************/
SPIClass SPI_2(2);
/*****************************************************************************/
void SPI_setup(void)
{
	Serial1.print(F("setting up the SPI..."));
	pinMode(NSS_PIN, OUTPUT);	// configure NSS pin
	NSS_SET;	// set to inactive
	// use SPI 2 because SPI 1 pins are used as analog input
	SPI_2.beginTransaction(SPISettings(18000000, MSBFIRST, SPI_MODE0, DATA_SIZE_16BIT));
	Serial1.println(F("done."));
}
/*****************************************************************************/
#define BUFFER_LENGTH		200
char rec_buff[BUFFER_LENGTH] __attribute__ ((packed));
bool reinit = false;
bool start = false;
/*****************************************************************************/
void setup()
{
	//Serial1.begin(57600);
	Serial1.begin(500000);
	while(!Serial1) {}  // wait for Leonardo
	//delay (5000);	// wait 5 seconds due to PC serial driver recognition delay

	Serial1.println(F("\n***** ADC dual regular simultaneous mode acquisition *****\n"));
	Serial1.println(F("Acquired data on selected analog input pins is sent over SPI2."));
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
	pinMode(ADC_IN8, INPUT_ANALOG);
#if defined DEBUG_PIN
	pinMode(DEBUG_PIN, OUTPUT);
	//gpio_set_mode(GPIOA, 8,GPIO_OUTPUT_PP);	// PA8, pin 27
	Serial1.println(F("Debug version!"));
#endif
	SetupParameters();
	SetupModules();
	Serial1.print(F("-> sampling started..."));
	rec_buff[0] = 0;
}
/*****************************************************************************/
/*****************************************************************************/
void SetupModules()
{
	Serial1.println(F("\n***** Accepted commands *****"));
	Serial1.print(F("\tSet up recording parameters: set rec_time=")); Serial1.print(recording_time);
		Serial1.print(F(";sampling_freq=")); Serial1.print(sampling_frequency/1000);
		Serial1.print(F(";samples_per_seq=")); Serial1.println(samples_per_sequence);
	Serial1.println(F("\tStart recording: go\\n"));
	Serial1.println(F("\tDump recorded data: get\\n\n*****"));
	delay(100);
/*
//debug outputs
	Serial1.println(F("Input parameters:"));
	Serial1.print(F("recording_time [ms]: ")); Serial1.println(recording_time);
	Serial1.print(F("sampling_frequency [Hz]: ")); Serial1.println(sampling_frequency);
	Serial1.print(F("samples_per_sequence: ")); Serial1.println(samples_per_sequence);
*/
// set-up involved hardware modules
	TIMER_Setup();
	ADC_Setup();
	DMA_Setup();
	SPI_setup();
	start = false;	// stop after first run
	// let timer run - do this just before generating an update trigger by SW
	timer_resume(TIMER3);
}
/*****************************************************************************/
/*****************************************************************************/
void ParseToken(char * p)
{
	if ( strstr(p,"time")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1);
			Serial1.print(F(">> rec_time = ")); Serial1.println(val);
			recording_time = (uint16_t)val;
			reinit = true;
		}
	} else if ( strstr(p,"freq")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			uint32_t val = atol(p+1)*1000;
			Serial1.print(F(">> sampling_freq = ")); Serial1.println(val);
			sampling_frequency = val;
			reinit = true;
		}
	} else if ( strstr(p,"seq_1")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			p++;
			Serial1.print(F(">> seq_1 = ")); Serial1.println(p);
			// store the channels of sequence 1
			uint16_t i;
			for (i = 0; i<8; i++) {
				sequence_1[i] = atoi(p++);
				if (*p++!=',')	break;
			}
			samples_per_sequence = i;
			reinit = true;
		}
	} else if ( strstr(p,"seq_2")>0 ) {
		if ( (p=strchr(p,'='))>0 ) {
			p++;
			Serial1.print(F(">> seq_2 = ")); Serial1.println(p);
			// store the channels of sequence 2
			uint16_t i;
			for (i = 0; i<8; i++) {
				sequence_2[i] = atoi(p++);
				if (*p++!=',')	break;
			}
			if ( samples_per_sequence != i)	Serial1.print(F("__Error: invalid seq_2 length!__"));
			reinit = true;
		}
	} else if ( strstr(p,"go")>0 ) start = true;
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
uint16_t SerialReadBytes()
{
	if ( Serial1.available()<=0 ) return 0;
#define SERIAL_RX_TIMEOUT	100	// millis
	rec_buff[0] = 0;	// mark start of string
	//if ( Serial1.available()<=0 ) return 0;
	uint16_t rec_index = 0;
	uint32_t ts = millis();	// prepare reception time-out
	// wait for more serial data
	while ( (millis()-ts)<SERIAL_RX_TIMEOUT ) {	// no time-out occurred yet
		if ( Serial1.available()<=0 ) continue;
		rec_buff[rec_index++] = Serial1.read();
		ts = millis();	// reset time-out to receive next byte
	}
	if ( rec_index==0 ) return 0;	// nothing was received
	rec_buff[rec_index] = 0;	// mark end of string
	if ( rec_buff[rec_index-1]=='\n' ) rec_buff[rec_index-1] = 0;	// clear '\n'
	//if ( str_mode ) { Serial1.print(F("> received: ")); Serial1.println(rec_buff); }
	return rec_index;
}
/*****************************************************************************/
/*****************************************************************************/
void loop()
{
	// check and parse received serial data, if any
	if ( SerialReadBytes()>0 )
		ParseRxData();
	// re-init if parameters have changed 
	if ( reinit ) {
		reinit = false;
		SetupParameters();
		SetupModules();
	}		
	// check ADC status and push data to SPI if sequence acquisition ended
	if ( dma_irq_full_complete ) {
		dma_irq_full_complete = 0;
		ADC_buffer_to_SPI();	// send data to SPI
		buff_sent = 1;	// overrun management
	}
}

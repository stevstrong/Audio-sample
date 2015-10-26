/*
	ADC conversion of multiple channels in dual regular simultaneous conversion mode.
	=================================================================================
	Each conversion sequence is started by SW.
	The result of each dual sample (32bits) is stored into the ADC buffer via DMA transfer.
	After a sequence is completely sampled, a new sequence is automatically started.
	The number of total sequences to sample is configurable.
	---------------------------------------------------------------------------------
	Todo:
	- The start of each sequence sampling should be timed (timer IRQ?)
	- When the number of sampled bytes reaches 512 bytes, these shall be stored onto SD card.
*/


#include <libmaple/adc.h>
#include <libmaple/dma.h>

/********************************************************************/
// Configuration
/********************************************************************/
// number of samples to take
#define SAMPLE_COUNT	10
// time between samples in microseconds - not used yet
#define SAMPLE_TIME_GAP 20
// Set the ADC input channel sequences (left will be first)
// channel 17 = Vrefint, only available on ADC1, can be used for tests
uint8 ADC1_Sequence[] = {17,17,0};
uint8 ADC2_Sequence[] = {1,0,1};

/********************************************************************/
// defines and variables
/********************************************************************/
// regular channel sequence length
#define ADC_SEQ_LEN	( sizeof(ADC1_Sequence) )
// size of buffer to store the ADC conversion values
#define ADC_BUFFER_SIZE ( SAMPLE_COUNT*ADC_SEQ_LEN )

// the main buffer to store ADC sample values
uint32_t adc_buffer[ADC_BUFFER_SIZE];
// index to the buffer
int buf_index = 0;

// analog input pins corresponding to maple mini ports or pins
#define ADC_IN0 PA0
#define ADC_IN1 PA1
#define ADC_IN2 PA2
#define ADC_IN3 PA3
#define ADC_IN4 PA4
#define ADC_IN5 PA5
#define ADC_IN6 PA6
#define ADC_IN7 PA7
#define ADC_IN8 3  // maple mini pin
//#define ADC_IN9 33  // maple mini pin - actually used as LED output

// The interrupt handler, rx_dma_irq(), sets this to 1.
volatile uint32 dma_irq_fired = 0;
// Used to store DMA interrupt status register (ISR) bits inside rx_dma_irq().
// This helps explain what's going on inside loop(); see comments below.
volatile uint32 dma_isr = 0;

/*****************************************************************************/
/*
* calc_adc_sequence(ADCx_Sequence) converts the SQR3 6 channels' (each ADC1 and ADC2) list into
 * a valid 6 X 5=30 bits sequence format and returns that 30 bits number.
 * There are 6 available sequences in each SQR3 SQR2, and 4 in SQR1.
 * For more channels, repeat the same for SQR2, SQR1. (For SQR1 4 channels only!)
 */
/*****************************************************************************/
uint32_t adc_sequence_build(uint8 adc_seq_array[])
{
	uint32 adc_sequence = 0;
	for (int i = 0; i<ADC_SEQ_LEN; i++)
	{
		adc_sequence |= adc_seq_array[i] << (i * 5);
	}
	return adc_sequence;
}
/*****************************************************************************/
// This is our DMA interrupt handler.
/*****************************************************************************/
void rx_dma_irq(void)
{
	dma_irq_fired = 1;
	dma_isr = dma_get_isr_bits(DMA1, DMA_CH1);
}
/*****************************************************************************/
/*****************************************************************************/
void DMA_Setup(void)
{
// DMA tube configuration
dma_tube_config my_tube_cfg = {
	&ADC1->regs->DR,	// data source address
	DMA_SIZE_32BITS,	// source transfer size
	&adc_buffer,		// data destination address 
	DMA_SIZE_32BITS,	// destination transfer size
	ADC_BUFFER_SIZE,	// nr. of data to transfer
	// tube flags: increment dest addr, circular buffer, set IRQ, very high prio:
	( DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_CMPLT_IE | DMA_CCR_PL_VERY_HIGH ),
	0,	// unused
	DMA_REQ_SRC_ADC1,	// Hardware DMA request source
};
	Serial.println(F("preparing the DMA..."));
	// turn DMA on
	dma_init(DMA1);
	// configure DMA channel
	int ret = dma_tube_cfg(DMA1, DMA_CH1, &my_tube_cfg);
	if ( ret>0 ) {
		Serial.print(F("DMA configuration error: ")); Serial.println(ret,HEX);
		Serial.print(F("It is not safe to continue!!!"));
		while ( Serial.read()<=0 );	// wait for a key stroke
	}
	// attach an interrupt handler.
	dma_attach_interrupt(DMA1, DMA_CH1, rx_dma_irq);
	// Turn on the DMA tube. It will now begin serving requests.
	dma_enable(DMA1, DMA_CH1);
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

	uint32_t cr2 = ( ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL | ADC_CR2_DMA | ADC_CR2_TSEREFE); // ADC_CR2_TSEREFE for test only
	ADC1->regs->CR2 = cr2;
	ADC2->regs->CR2 = cr2;

	adc_enable(ADC1);
	adc_enable(ADC2);

	adc_calibrate(ADC1); //optional, but strongly recommended
	adc_calibrate(ADC2);

	adc_set_reg_seqlen(ADC1, ADC_SEQ_LEN); //The number of channels to be converted.
	adc_set_reg_seqlen(ADC2, ADC_SEQ_LEN);
	// set the channel sequences
	ADC1->regs->SQR3 = adc_sequence_build(ADC1_Sequence);
	ADC2->regs->SQR3 = adc_sequence_build(ADC2_Sequence);

	ADC1->regs->CR1 = ( (0x06 << 16) | ADC_CR1_SCAN ); // 0b0110: Regular simultaneous mode only, enable SCAN
	ADC2->regs->CR1 = ( ADC_CR1_SCAN ); // enable SCAN, mode setting not possible for ADC2
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
//	pinMode(ADC_IN9, INPUT_ANALOG);
	ADC_Setup();
	DMA_Setup();
}
/*****************************************************************************/
/*****************************************************************************/
void printSamples(int chan)
{
	Serial.print(F("CHAN_")); Serial.print(chan); Serial.print(" =");
  int i;
	for(int k=0; k<SAMPLE_COUNT; k++)
	{
		if( (k%20)==0 ) Serial.println();
		Serial.print(" ");
		// needs rework !!!
		i = (k+chan);
		uint32_t tmp = adc_buffer[k+chan];
		//if (chan%1) tmp = 
		Serial.print(tmp, HEX);
	}
	Serial.println();
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

	Serial.println(F("-> sampling started..."));

	// test: fill ADC buffer with dummy data
	for (int i = 0; i<ADC_BUFFER_SIZE; i++) adc_buffer[i] = 0x11111111;
	// clear DMA transfer complete flag
	DMA1->regs->IFCR |= DMA_IFCR_CTCIF1;
	uint32_t data_cnt = DMA1->regs->CNDTR1;
	dma_irq_fired = 0;
	int32_t tout;

	long tstart = micros();
	for(int m = 0; m<SAMPLE_COUNT; m++) {
		// update data counter
		data_cnt -= ADC_SEQ_LEN;
		// start ADC conversion by SW
		ADC1->regs->CR2 |= ADC_CR2_SWSTART;
		// time-out to avoid hangup if something goes wrong
		tout = 100000;
		// wait for conversion end of the ADC sequence. The DMA data counter should decrease by ADC_SEQ_LEN
		while ( !(dma_irq_fired) && (DMA1->regs->CNDTR1>data_cnt) && (--tout)>0 );
		// only for debug purposes:
		if ( tout<=0 ) {
			Serial.println(F("\nDMA data_cnt TIME_OUT !!!"));
			Serial.print(F("irq_fired: ")); Serial.print(dma_irq_fired);
			Serial.print(F(", dma_isr: ")); Serial.println(dma_isr);
			Serial.print(F("sample counter: ")); Serial.print(m);
			Serial.print(F(", data counter: ")); Serial.print(data_cnt);
			Serial.print(F(", CNDTR1: ")); Serial.println(DMA1->regs->CNDTR1);
		}
	}
	long tstop = micros();

	Serial.print("done.\nElapsed Time: ");
	Serial.print(tstop - tstart);
	Serial.print(" us (for ");
	Serial.print(SAMPLE_COUNT*ADC_SEQ_LEN);
	Serial.println(" samples)");
	Serial.print((tstop-tstart)/(double)(SAMPLE_COUNT*ADC_SEQ_LEN));
	Serial.print(" us (for 1 sample) ");
	Serial.print((tstop-tstart)/(double)(SAMPLE_COUNT*ADC_SEQ_LEN*2));
	Serial.println(" us (for 1 channel)");
	Serial.print(F("sampled data:")); /*Serial.print(chan); Serial.print(" =");*/
	for(int k=0; k<ADC_BUFFER_SIZE; k++)
	{
		if( (k%ADC_SEQ_LEN)==0 ) Serial.println();
		Serial.print(" ");
		uint32_t tmp = adc_buffer[k];
		Serial.print(tmp/0x10000, HEX);
		Serial.print(" ");
		Serial.print(tmp%0x10000, HEX);
	}
	Serial.println();
}

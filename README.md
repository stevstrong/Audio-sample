Audio sampling with STM32 (Maple Mini clone)
--------------------------------------------
	
The ADC conversion of multiple channels is performed in dual regular simultaneous conversion mode.

The result of each dual sample (32bits) is stored into the ADC buffer via DMA transfer.
After a sequence is completely sampled, a new sequence is automatically started by timer 3 update event.
The sampled data is stored on SD card, each half buffer is pushed to card after being filled by DMA.

The number of sequences, the recording time and the sampling frequency are configurable.

Current performance: 8 channel data get sampled with 28kHz.

----------------------------------------------------------------------------------------------------------
	
Audio sampling with Arduino Yun / Pro Mini.
------------------------------------------

Analog inputs 0 to 3 are continuously and sequentially sampled in the ADC interrupt service routine.
The sampled data is sequentially stored in a double buffer area. Once the first buffer is full, the second buffer is used to store the ADC data in the ISR, while the first buffer will be written onto SD card on main level.
For writing to SD card the contiguous writing is used, see RawWrite example of the SdFat lib.

Current performance: 4 audio channel data get sampled with a frequency of ~19kHz.

Todo: use sample-and-hold buffers, so that no deviation in time will occur between the audio channels.

// 2015.10.09.Fri:

- first version with ARef, which eliminates the DC offset, as Ucc of the preams is fed to the ADCs.
- for playback, use Audacity > Import > Raw > unsigned 8bit, 4channels, 19000Hz (77kHz /4)


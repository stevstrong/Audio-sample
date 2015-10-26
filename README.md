Audio-sample
============

Audio sampling with Arduino Yun / Pro Mini.
Analog inputs 0 to 3 are continuously and sequentially sampled in the ADC interrupt service routine.
The sampled data is sequentially stored in a double buffer area. Once the first buffer is full, the second buffer is used to store the ADC data in the ISR, while the first buffer will be written onto SD card on main level.
For writing to SD card the contiguous writing is used, see RawWrite example of the SdFat lib.

Current performance: 4 audio channel data get sampled with a frequency of ~19kHz.

Todo: use sample-and-hold buffers, so that no deviation in time will occur between the audio channels.

// 2015.10.09.Fri:

- first version with ARef, which eliminates the DC offset, as Ucc of the preams is fed to the ADCs.
- for playback, use Audacity > Import > Raw > unsigned 8bit, 4channels, 19000Hz (77kHz /4)


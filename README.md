# Audio driver for Raspberrypi 1 (BCM2835)

This driver allows ALSA to output audio using the PWM peripheral
using a high frequency output of 200 kHz.
It configures the PWM peripheral with these parameters:
 - MSEN = 0 (for highest frequency output above 400 kHz in practice depending on signal level)
 - 25 MHz peripheral clock
 - range of PWM values from 0 to 124
 - Each PWM sample is 32 bits wide
 - Data is transfered from memory to PWM peripheral using DMA
 - PWM outputs alternatively on left then right channel

This driver is used with this ALSA format:
 - Sample rate: 200 KHz
 - Format: `U32_BE` (with values ranging from 0 to 124 (included))
 - Channels: 2, interleaved

The userland software should:
 - Take the normal audio stream and upsample it to 200 Khz
 - Convert each sample to 0-124 32 bits unsigned values
 - Integrate the truncation error to these values into the next values
 - Use `snd_pcm_writei` to write the conveted audio data to the driver via ALSA

This driver allows very low latencies, lower than the default driver which use a message box
to the VideoCore via VCHIQ, but the upsampling to 200 KHz eat a non negligible amount of CPU time
depending on algorithms used. This can be 40% of CPU time.
In case of receiving audio from the LAN network, the conversion to 200 KHz can be offloaded
to a more capable computer.

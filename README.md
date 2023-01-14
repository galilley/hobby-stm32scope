Use stm32f103 (Blue Pill) or stm32g431 (NUCLEO-G431KB) as a hobby oscilloscope.

This has two channels, measuring on PA0 and PA1,
this can be easily increased in the code though.
Be careful to not fry your microcontroller,
don't give more than 3.3V (VDD) to them.

It uses USART1  (Pins A9 and A10) to communicate with your computer for F1 family or STLINK VCP (USART2) for G4.

### Functional overview:

* `TIM1` triggers `ADC1` to sample
* `DMA1_CH1` transfers `ADC1->DR` to memory
* `DMA1_CH1` interrupt handler starts `DMA1_CH4` (on half and complete)
* `DMA1_CH4` transfers memory to `USART`

### Protocol

Commands are sent from computer over usart:

* `x01 STOP` stops current sampling (stops TIM1)
* `x02 START` starts sampling (starts TIM1)
* `x03 GET_SAMPLERATE` Returns the samplerate, as ascii, newline terminated


Samples will be flowing out of USART, 2 bytes per channel.
Note that the ADC is only 12bit so the highest 4bits are unused in each sample.
 

### Noise

If your stm32f103 package doesn't have a VREF pin
(most don't, for example the bluepill doesn't)
VDD will be the reference point, and any noise there
will affect your measurements, but if you worry about
+-100mV this is not the tool for you anyway.

As the ADC input impedence is quite low you may want to put a voltage buffer in front of each channel,
e.g. an opamp with the output connected to negative input,
and then put your signal on the positive input of the opamp.

G4 family is more suitable for real use as it has internals opamps.

### Build

Use cmake to build the project and openocd to flash it:

```
mkdir ./build/
cd ./build/
cmake ..
make
make ocd
```

You also may need to reset the target manually for successful flashing.

### Sigrok

To use with sigrok/pulseview, apply [libsigrok.patch](./libsigrok.patch),
the device scanning is just hardcoded, so make sure to change protocol.h
to point to the serial device your stm32 shows up as if you want to use it with
pulseview:

```
#define STM32SCOPE_DEFAULT_CONN "/dev/ttyUSB0";
```

With sigrok you can specify the device directly:

```
LD_LIBRARY_PATH=../lib ./sigrok-cli --driver=stm32scope:conn=/dev/ttyUSB0 --samples=200  --channels=A0,A1
```


License:
libigrok.patch: GPLv3
The rest: MIT

# Blog #

## Update 2012-07-06 ##
I managed to figure out the hardware flow control on the STM32-P103 boards. To be able to use the hardware flow control (RTS/CTS), the following conditions must be met:
  * The CTS\_E solder bridge must be closed.
  * The RTS\_E solder bridge must be closed.
  * USART2 CTS pin (PA0) must be configured as floating input.
  * USART2 RTS pin (PA1) must be configured as alternate push-pull output.
  * USART2 TX pin (PA2) must be configured as alternate push-pull output.
  * USART2 RX pin (PA3) must be configured as floating input.
Due to the components in the signal path of the CTS input pin, the board cannot send anything if the CTS input pin is not actively pulled low. If left floating, the input is detected as high.

This essentially means that it is not possible to send anything if there is no cable connected. To work around this, I had to use a loopback connector to be able to test the code. The loopback connector I made is a simple male DB-9 connector with a bridge soldered between pins 2 and 3 and a second bridge between pins 7 and 8.

## Update 2012-07-05 ##
Today, I committed a work in progress patch to implement the protocol for talking to the DMX controller via the serial UART.

Communication will use hardware flow control to preserve the accurate timings of the DMX packets that have to be sent out.

**Please note**, in order to use the RTS/CTS flow control on the development board, 2 solder bridges must be closed to connect the lines. Please refer to the schematics to find out which ones I mean.

I have ordered an STM32-P407 development board from Olimex via eBay. The plan is to have this P407 board talk to 2 STM32-P103 boards via the UARTS, and expose an interface for controlling the 2 DMX universes via Ethernet.

It is currently undecided if I will develop a protocol for the Ethernet of the P407, or if I will use a web server to control the DMX channels.

## Update 2012-05-05 ##
Modified the project for development on Ubuntu 12.04 LTS. The openocd configurations have been changed to use the built-in scripts. I am now also using the ARM GCC from the launchpad project. https://launchpad.net/gcc-arm-embedded

I swapped compilers because this compiler does not require the user to supply his email to be able to download it.

My version of the compiler was built from the sources, compiled for 64 bit machines. The project should work without issues on the prebuilt 32bit binary version, although I have not tested this.

## Update 2011-12-19 (2) ##
Committed first drop of the code. The DMX 512 communication is implemented. The application runs an RGB color wheel on channels 1-3 of the DMX512 bus.

You can find the .hex file [here](http://users.telenet.be/rsq/stm32-dmx512-r6_debug.hex).

## Update 2011-12-19 ##
I have received the boards from Olimex, and I am coding the DMX controller.

The DMX protocol is implemented, and I can send colors to my RGB dimmer and led strip:

![http://users.telenet.be/rsq/IMG_20111219_135042.jpg](http://users.telenet.be/rsq/IMG_20111219_135042.jpg)
![http://users.telenet.be/rsq/IMG_20111219_135052.jpg](http://users.telenet.be/rsq/IMG_20111219_135052.jpg)

The RGB dimmer used is one of these:
http://www.ebay.com/itm/DMX512-PX-Decoder-Driver-9A-Amplifier-12V-24V-RGB-LED-/170725894859?pt=UK_ConElec_LightingLEDsStrobes_RL&hash=item27c00e6acb#ht_3070wt_1165

It drives a standard RGB LED strip.

I am currently working on code cleanup and documenting the build system. A commit should happen later today.

## Update 2011-12-10 ##
The project has been started on Google Code. Parts of the wiki structure have been put in place.

Work on the build system has begun.
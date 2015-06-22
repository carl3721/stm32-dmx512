# Introduction #

This wiki describes the various aspects of the DMX controller software for the Olimex STM32-P103.

Use this page as a starting point if you are looking for information.

# News/Updates/Blog #
See Stm32DmxBlog.

# Details #

## What is DMX? ##
DMX is a communication protocol used to remotely control dimmers. More information can be found [here](http://en.wikipedia.org/wiki/DMX512).

## About the project ##
This project is developed as part of the Open source initiative from Olimex. More information can be found [here](http://www.olimex.com/dev/projects.html).

The project will use an STM32-P103 development board and the MOD-RS485 module from Olimex to provide a DMX controller.

The DMX controller can be used via the serial port on the board. A simple communication protocol is foreseen to let any RS232 capable device use the DMX controller as a component. This project is developed with the goal of allowing a normal computer to speak DMX512 with minimal CPU load, but there is no reason why this could not be used from a PLC or even another microcontroller.

All source code will be written in C99 with the GNU99 extensions. The compiler will be [Sourcery Codebench Lite Edition from Mentor Graphics](http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition) which is basically GCC. A Make build script will be foreseen.

The project is developed and used on Linux, but it will be possible to port to windows with minimal effort.

## License ##
The project is licensed under the LGPL V3. The LGPL was chosen over the GPL because the GPL forces all derived works to inherit the GPL license. Choosing LGPL lets the developer wishing to use my code as a base for his/her project choose the license of the derived work.

# More information #

## Documentation ##
Stm32DmxDocumentation
## Toolchain setup ##
Stm32DmxToolchainSetup
## Serial protocol specification ##
Stm32DmxSerialProtocolSpecification
ADC SPI
=======

Microcontroller code for sending ADC values over SPI, developed for
Atmel ATtiny25.

Wiring
------

PB3: ADC input in range 0V - 2.56V (do not exceed Vcc)

PB2: SCK

PB1: MISO

PB0: SS (active low)

(no MOSI channel needed)

SPI mode 2 (CPOL=1, CPHA=0: one-based clock, negative edge)

License
-------

The program is licensed under the terms of the
[Simplified (2-cluase) BSD License][license] as approved by the Open Source
Initiative.

  [license]: http://opensource.org/licenses/BSD-2-Clause "2-clause BSD License"

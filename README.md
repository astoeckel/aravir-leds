# Aravir LEDs ― Linux compatible I²C LED controller

[![Build Status](https://travis-ci.org/astoeckel/aravir-leds.svg?branch=master)](https://travis-ci.org/astoeckel/aravir-leds)

[📙 Click here to read the documentation as PDF](https://github.com/astoeckel/aravir/blob/master/doc/tex/aravir_leds.pdf)


**The *Aravir LEDs* C++14 library implements a software-defined I²C LED
controller that can be used on 8-bit microcontrollers.** The LED controller
supports programmable blinking and ramp patterns for up to eight LEDs.
The library provides a Linux LED subsystem compatible driver with support for
hardware accelerated blinking.
## FAQ

#### Q: I can't communicate with the I²C slave device when I use a Raspberry Pi as I²C master. What am I doing wrong?

**A:** Nothing, most likely. The Raspberry Pi I²C hardware is just broken. See
the appendix below on how to properly setup a Raspberry Pi as I²C master.

#### Q: Can I contribute?

**A:** Sure! Feel free to report bugs or pull requests in the corresponding
*Aravir* GitHub repositories. However, note that I mainly developed these libraries
for use in my own hobby projects, so I may be a little nitpicky regarding the
changes that I accept.

#### Q: What's with the name?

**A:** According to [Wikipedia](https://en.wikipedia.org/wiki/Rangers_of_the_North):

> *Aravir* (2156–2319) succeeded his father Aranuir in 2247. During his
> chieftainship the *Watchful Peace* slowly came to an end as Sauron returned to
> the north-west of Middle-earth, although his presence remained unknown.

In all of Norse, Celtic, Roman and Greek mythology, as well as J.&nbsp;R.&nbsp;R.&nbsp;Tolkien's legendarium, Wikipedia's lists of flowering plants and cute marsupials, this was the only somewhat pronouncable name I came across that had zero hits on GitHub.

#### Q: I'm using your library in a safety critical system...

**A:** Don't do that.

#### Q: Are there other licensing options?

**A:** Short answer: please contact me. I do not like to publish my work
under a non-copyleft license, but I may be persuaded.
## Raspberry Pi as I²C master

There is a hardware-bug in the I²C hardware module of the BCM283x SoC used in
the Raspberry Pi (see
[this writeup](http://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html)
for more information). The bug affects slave devices that rely on
a standards-compliant implementation of clock-stretching. While this isn't a
problem with most hardware I²C slaves (as they do not rely on clock-stretching),
this incompatibility causes issues with software-defined devices that have
longer response times, such as an ATmega running *Aravir*.

Unfortunately, there are only two rather suboptimal solutions to this problem: using
a software I²C implementation on the Raspberry Pi via the `i2c-gpio` driver or
dramatically reducing the I²C clock speed.

The instructions below explain how to use the Raspberry Pi with the `i2c-gpio`
driver (recommended), or the hardware I²C bus with reduced speed.

### Using software I²C via `i2c-gpio`

Add the following lines (without the linebreak between the second and third line) to
the `/boot/config.txt` on your Raspberry Pi:
```
dtparam=i2c=on
dtoverlay=i2c-gpio,
          i2c_gpio_sda=23,i2c_gpio_scl=24
```
after rebooting you should see a device `/dev/i2c-3`. Per default the bus will
run at 100&nbsp;kHz. The bus lines are set to [23 (SDA)](https://pinout.xyz/pinout/pin16_gpio23#) and [24 (SCL)](https://pinout.xyz/pinout/pin16_gpio24#) in the above
example.

*Note:* In contrast to the below method, you *will need* additional pull-up
resistors. A recommended value is about 4.7&nbsp;kΩ.

### Using hardware I²C with reduced clock-speed

Add the following lines to the `/boot/config.txt` on your Raspberry Pi (or
uncomment the corresponding, already existing lines)
```
dtparam=i2c_arm=on,i2c_arm_baudrate=10000
```
after rebooting you should see a device `/dev/i2c-1`. This I²C device will
operate at a 10&nbsp;kHz baud-rate. At this bus speed a microcontroller running
at 8 MHz has about 200 clock cycles to respond to an I²C request without
running into the above bug. This should be sufficient for most of the operations
implemented by the *Aravir* libraries; however, this assumes that the slave
I²C interrupt handler can immediately react to I²C bus requests. Depending on
the presence of other interrupt handlers on the slave device this may not be the
case.

⚠ *Note:* Because of the above caveat, this method is not recommended.

⚠ *Note:* The [GPIO pins](https://pinout.xyz/pinout/i2c#) on the Raspberry Pi
reserved for hardware I²C (GPIO 2, SDA; GPIO 3, SCL) already have a adequate
pull-up resistors. You should not add additional pull-up resistors to the bus-lines.
## Legal

The *Aravir* libraries are licensed under the GNU GPLv3. All documentation is
made available under the GNU FDLv1.3.

### Software License

Copyright (c) 2019  Andreas Stöckel

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

### Documentation License

Copyright (c)  2019  Andreas Stöckel

Permission is granted to copy, distribute and/or modify this document
under the terms of the GNU Free Documentation License, Version 1.3
or any later version published by the Free Software Foundation;
with no Invariant Sections, no Front-Cover Texts, and no Back-Cover Texts.

You should have received a copy of the GNU Free Documentation License
along with this document.  If not, see <https://www.gnu.org/licenses/>.


### Trademark disclaimer

All product and company names (including, but not limited to Arduino,
Dallas Semiconductor, Maxim Integrated, Microchip, Linux, Raspberry Pi, AVR,
I²C) are trademarks or registered trademarks of their respective holders. Use
of them does not imply any affiliation with or endorsement by them.


----------------

 *This file was automatically generated. Do not edit. Please edit the files in the `doc/` folder instead.*

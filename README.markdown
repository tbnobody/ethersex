Changes in this fork
====================

## Description

Control RGB leds connected to a PCA9685 PWM controller.

## Pre-Requirements

```bash
 apt install gcc gcc-avr avr-libc binutils-avr m4 gawk libncurses5-dev make dialog git-core avrdude
```

## Download sources

```bash
git clone http://github.com/tbnobody/ethersex.git
```

## Compile and upload

Type

```bash
make menuconfig
```

select `Load a Default Configuration` and select between `EtherLed (Debug)` or `EtherLed (No Debug)`. Save and exit.

Run

```bash
make
```

to compile and 

```bash
make program
```

to flash.

About Ethersex
==============
Ethersex, originally developed to provide an alternative firmware for the [etherrape hardware](http://www.lochraster.org/etherrape),
evolved into a full-featured still light-weight firmware for the Atmel megaAVR processors.  
For more information and a comprehensive documentation  consult [http://www.ethersex.de](http://www.ethersex.de)!

How to configure the firmware
=============================
Make sure that you meet the requirements.  
Use `make menuconfig` to configure and `make` to compile the firmware.
The final hex file is named `ethersex.hex`.

[See the Quick Start Guide in the wiki for more information](http://ethersex.de/index.php/Quick_Start_Guide)

How to add a new hardware pinning
=================================
Use the script at `scripts/add-hardware` to add a new pinning.


Used 3rd party software 
=======================
This program contains software by other authors:

* [the uIP tcp/ip stack](https://github.com/adamdunkels/uip) in the directory `/protocols/uip/`, written by Adam Dunkels
* [usb-software stack from obdev](https://www.obdev.at/products/vusb/index.html) in `/protocols/usb/usbdrv/`, written by Objective Development
* [sd card reader](http://www.roland-riegel.de/sd-reader/) in `/hardware/storage/sd_reader`, written by Roland Riegel
* [IRMP - Infrared Multi Protocol Decoder](https://www.mikrocontroller.net/articles/IRMP) in `/hardware/ir/irmp/lib`, written by Frank Meyer
* [ucglib](https://github.com/olikraus/ucglib) in `/libs/ucglib`, written by Oliver Kraus
* [u8g2](https://github.com/olikraus/u8g2) in `/libs/u8g2lib`, written by Oliver Kraus

License
=======
All ethersex related code is licensed under GPLv3, unless otherwise noted. See COPYING in the main
directory, but in doubt check the file header. Usually every file contains a
header, stating all contributing authors and the specific license used.

Various make targets
====================

* `make show-config` -- Shows the activated modules

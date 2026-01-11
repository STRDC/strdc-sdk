![STRDC Logo](STRDC.png)

# About
strdc-sdk aims to be your go to development kit for all sensor fusion and navigation applications. Built with hardware developers and makers in mind, get from prototype to finished product faster with easy to use HALs, Drivers, and Modules for your application.

# Module Status

## BNO08x
Turntable Interactive Self-Calibration, Tap Detection, Pickup Detection, Sleep Detection, Pocket Detection, and Heart Rate Monitor are experimental (functions are commented out) at the moment.

## GNSS-Ublox
Support for firmware used in the SAM-M10Q and NEO-M9N, though the module will also serve most basic functions with other Ublox GNSS Receiver products. Note that not all functions (configurations) are supported by every chip/firmware. Be sure to review all Ublox product documentation (including not just the datasheet, but the integration and interface manuals, as well as the firmware release note for the version used on the IC). Configurations not supported by the chip firmware will return a NACK message.

## LIS2MDL
Support for I2C. SPI (3-wire and 4-wire) and interrupts are experimental at the moment.

## BMP390
Support for I2C. SPI (3-wire and 4-wire) and interrupts are experimental at the moment.
# About the BMP390
The Bosch BMP390 is an ultra-low power consuming absolute pressure and temperature sensor intended for mobile phones, GPS modules, and watches. The sensor includes an IIR Filter for ignoring short-term pressure disturbances (e.g. slamming a door), oversampling ability for high resolution, and a FIFO for both reducing host power and preventing data loss in non-real-time systems. Intended applications:

- GPS/GNSS Enhancement
- Weather forcasting
- Wearables
- Altitude and vertical velocity measurement

# About the STRDC BMP390 Library
The STRDC BMP390 library delivers the functionality of this low-power absolute pressure and temperature sensor, abstracting the minutia of lower level tasks so the user can spend more time prototyping, less time setting up. Main features:

- Initialization and configuration of the sensor settings
- FIFO configuration, management, and reporting
- Interrupt configuration
- Measurement output compensation
- Sensor mode configuration
    - Abstracts switching method to reduce user lines of code (no need to switch to sleep first)
- Intuitive examples to make getting started easy

# Module Status
Support for I2C. SPI (3-wire and 4-wire) and interrupts are experimental at the moment.
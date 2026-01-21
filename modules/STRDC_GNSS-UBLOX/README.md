# About u-blox GNSS Receivers
Offering meter-level accuracy, u-blox standard precision GNSS receivers offer an excellent mix of accuracy, low-power, small size, integration, and easy setup. The onboard firmwares host many useful features for numerous tracking and navigation applications such as geofencing, power save modes, static hold, and more. For those new to GNSS or experienced veterans, u-blox GNSS receivers allow users to avoid compromising between performance and practicality, extending technologies:

- Asset Tracking
- Telematics and fleet management
- Navigation
- Wearables

# About the STRDC GNSS-UBLOX Library
Intended for STRDC GNSS Receivers, the STRDC GNSS-UBLOX library offers access to many of the shared features across the u-blox GNSS Receiver portfolio. Not all u-blox firmware support every function (e.g. Geofencing is not supported by the M10 firmware as of this writing). Main features:

- Communication and parsing with both NMEA 0183 and UBX protocols
- Configuration through UBX-CFG messages
- Periodic and one time messaging
- Implementation of power save modes
- Batch messaging
- Geofencing
- Time pulse output configuration
- Simplified TXREADY configuration
    - Automatically disables primary function of selected PIO
- Simplified GNSS system selection
    - Detects for invalid configuration
- Simplified UART speed selection handover
    - Automatically re-initializes peripheral
- Intuitive examples to make getting started easy
    - Data record to SD card

# Module Status
Support for firmware used in the SAM-M10Q and NEO-M9N, though the module will also serve most basic functions with other Ublox GNSS Receiver products. Note that not all functions (configurations) are supported by every chip/firmware. Be sure to review all u-blox product documentation (including not just the datasheet, but the integration and interface manuals, as well as the firmware release note for the version used on the IC). Configurations not supported by the chip firmware will return a NACK message.
No support for USB communication at this time.

# Store Pages
[SAM-M10Q GPS Receiver Breakout Board](https://shop.strdc.com/products/sam-m10q-bob/)
[NEO-M9N GPS Receiver Breakout Board](https://shop.strdc.com/products/neo-m9n-bob/)
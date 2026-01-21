![STRDC Logo](STRDC.png)

# About
strdc-sdk aims to be your go to development kit for all sensor fusion and navigation applications. Built with hardware developers and makers in mind, get from prototype to finished product faster with easy to use HALs, Drivers, and Modules for your application.

# Installation and Usage
The middleware and modules require a HAL implementation to run properly. This is done to make applications platform agnostic.

## Arduino IDE
Arduino IDE compatibile libraries can be found in each module (as well as HAL and Middleware) as a .zip. Libraries can be added from the IDE from Sketch > Include Library > Add .ZIP Library...

Each library includes examples that can be run from the IDE (e.g. File > Examples).

## PlatformIO
Example projects have been included in each module (as well as the drivers). The required module(s), HAL, and drivers must be manually transferred to the lib folder within the PlatformIO project to properly build.
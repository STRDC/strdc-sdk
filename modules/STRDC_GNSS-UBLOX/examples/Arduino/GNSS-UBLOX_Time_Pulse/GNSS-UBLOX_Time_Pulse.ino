/*
 * Ublox GNSS Time Pulse Example Sketch.
 *
 * @file        GNSS-UBLOX_Time_Pulse.ino
 * @author      Alex Zundel
 * @copyright   Copyright (c) 2025 Stardust Orbital
 *
 * MIT License
 * 
 * Copyright (c) 2025 Stardust Orbital
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*************************************
 * INCLUDES
 *************************************/

#include "interrupt.h"
#include "gnss-ublox.h"

/*************************************
 * DEFINITIONS
 *************************************/

gnss_t myGNSS;

// Select desired peripheral, initialization, pinout, interrupt will be configured automatically
#define USE_I2C
//#define USE_SPI
//#define USE_UART

// Select Ublox product used for correct cfg changes needed for some functions
#define SAM_M10Q
//#define NEO_M9N
//#define NEO_M9V
//#define DAN_F10N

#ifdef SAM_M10Q
#define CHIP GNSS_SAM_M10Q
#endif

#ifdef NEO_M9N
#define CHIP GNSS_NEO_M9N
#endif

#ifdef NEO_M9V
#define CHIP GNSS_NEO_M9V
#endif

#ifdef DAN_F10N
#define CHIP GNSS_DAN_F10N
#endif

// Create interrupt handler for easier enable/disable
isr_handle_t* gnss_int;

// Poll variable changed in INT
volatile uint8_t gnss_msg;

// Messages we will be using
gnss_ubx_nav_pvt_t pvt_msg; // UBX-NAV-PVT
gnss_ubx_nav_clock_t clock_msg; // UBX-NAV-CLOCK
gnss_info_t info_msg; // Generic Info messages from both UBX & NMEA

uint8_t peripheral;

// I2C Address Info
#define GNSS_ADDRESS 0x42;

// Host Pin definitions
#if defined(USE_I2C) || defined(USE_SPI)
#define TX_RDY 8
#endif

#ifdef USE_SPI
#define GNSS_SS 0
#endif

#define EXTINT 4
#define RESET 9

// Peripheral definitions
#ifdef USE_I2C
#define ENABLE_PERIPH 0
#endif

#ifdef USE_SPI
#define ENABLE_PERIPH 1
#endif

// ISR to notify program to read messages
void gnss_ISR() {
  gnss_msg = 1;

  // Disable interrupt until we read the message
  interrupt_disable(gnss_int);
}

void setup()
{
    
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital Ublox GNSS Time Pulse Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  myGNSS.rcvr = CHIP;
  myGNSS.pinRst = RESET;
  
  #ifdef USE_I2C
  // I2C Configuration
  myGNSS.bus = &i2c1;
  myGNSS.busType = GNSS_I2C;
  myGNSS.busAddr = GNSS_ADDRESS;

  peripheral = 0;

  i2c_open((i2c_handle_t*)myGNSS.bus, 400000); // Max speed is 400 kHz

  while(gnss_init(&myGNSS))
  {
    Serial.println("Failed to initialize GNSS I2C");
    gnss_reset_hw(&myGNSS);
    delay(1000);
  }
  #endif

  #ifdef USE_SPI
  // SPI Configuration
  myGNSS.bus = &SPI_0;
  myGNSS.busType = GNSS_SPI;
  myGNSS.busAddr = GNSS_SS;

  peripheral = 2;

  spi_open((spi_handle_t*)myGNSS.bus, 3000000, SPI_MODE_0, SPI_BIT_ORDER_MSB); // Max speed is 5.5 MHz

  while(gnss_init(&myGNSS))
  {
    Serial.println("Failed to initialize GNSS SPI");
    gnss_reset_hw(&myGNSS);
    delay(1000);
  }
  #endif
  
  #ifdef USE_UART
  // UART Configuration
  myGNSS.bus = &uart2; // UART Bus 2
  myGNSS.busType = GNSS_UART;

  uint32_t uart_speed = 9600;

  #ifdef DAN_F10N
  uart_speed = 38400;
  #endif

  peripheral = 1;

  if(serial_open((serial_handle_t*)myGNSS.bus, uart_speed, UART_TYPE_BASIC))
  {
    Serial.println("Failed to open UART");
    while (1)
      ;
  }

  while(gnss_init(&myGNSS))
  {
    Serial.println("Failed to initialize GNSS Serial");
    gnss_reset_hw(&myGNSS);
    delay(1000);
  }
  #endif

  /************************************
  * Initiate Startup Reset
  ************************************/

  // Not entirely necessary, but sometimes it clears up configuration issues when attempting to re-upload
  gnss_reset_hw(&myGNSS);

  // Clear initial INFO messages
  timer_handle_t startup_timer;
  timer_init(&startup_timer, 1000000); // Wait 1s to receive all startup messages

  timer_start(&startup_timer);

  while (!timer_check_exp(&startup_timer))
    gnss_rec_and_parse(&myGNSS);

  /*************************************
  * Interrupt Setup
  *************************************/

  #if defined(USE_I2C) || defined(USE_SPI)
  // TXREADY functionality is not available for UART per Ublox datasheet
  gnss_int = interrupt_init(TX_RDY, GPIO_HIGH, gnss_ISR); // Interrupt polarity depends on gnss_enable_rdy() function

  if (gnss_int == NULL)
    Serial.println("GNSS Failed to init interrupt");

  
  // Enable interrupt
  interrupt_set(gnss_int);
  
  // Set TXREADY pin
  uint8_t enable_pio = 0; // PIO No. of chip, see function comments
  uint8_t enable_polarity = 0; // 0: active high, 1: active low
  uint8_t threshold = 2; // Threshold of # x 8 bytes to trigger TXREADY

  if (gnss_enable_rdy(&myGNSS, enable_pio, enable_polarity, threshold, ENABLE_PERIPH))
  {
    Serial.println("Failed to enable TXREADY");
    while (1)
      ;
  }

  #endif

  /*************************************
  * Setup Periodic Messaging
  *************************************/

  // Disable Default Messages

  if (gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_RMC, 0, peripheral))
  {
    Serial.println("Failed to update NMEA-Standard-RMC Message Rate");
    while (1)
      ;
  }
  
  if (gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GSA, 0, peripheral))
  {
    Serial.println("Failed to update NMEA-Standard-GSA Message Rate");
    while (1)
      ;
  }

  if (gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GLL, 0, peripheral))
  {
    Serial.println("Failed to update NMEA-Standard-GLL Message Rate");
    while (1)
      ;
  }

  if (gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GGA, 0, peripheral))
  {
    Serial.println("Failed to update NMEA-Standard-GGA Message Rate");
    while (1)
      ;
  }

  if (gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GSV, 0, peripheral))
  {
    Serial.println("Failed to update NMEA-Standard-GSV Message Rate");
    while (1)
      ;
  }

  if (gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_VTG, 0, peripheral))
  {
    Serial.println("Failed to update NMEA-Standard-VTG Message Rate");
    while (1)
      ;
  }

  // Enable Desired Messages

  if (gnss_set_msg_auto(&myGNSS, GNSS_UBX_NAV_PVT, 1, peripheral))
  {
    Serial.println("Failed to update UBX-NAV-PVT Message Rate");
    while (1)
      ;
  }

  if (gnss_set_msg_auto(&myGNSS, GNSS_UBX_NAV_CLOCK, 30, peripheral))
  {
    Serial.println("Failed to update UBX-NAV-CLOCK Message Rate");
    while (1)
      ;
  }

  /*************************************
  * Setup Time Pulse Configuration
  *************************************/

  gnss_pulse_cfg_t pulseCfg;

  // Determines whether time pulse is interpreted as a frequency or period
  pulseCfg.pulseDef = GNSS_PULSE_PERIOD;
  // Determines whether time pulse length is interpreted in us or %
  pulseCfg.lengthDef = GNSS_PULSE_LENGTH;
  // Antenna Cable Delay in (ns)
  pulseCfg.antDelay = 0;
  // Time Pulse Period (us) (only if pulseDef is period)
  pulseCfg.period = 1000000; // 1s period
  // Time Pulse Period when locked to GNSS time (us) (only if pulseDef is period and useLocked set)
  pulseCfg.periodLock = 0;
  // Time Pulse Frequency (Hz) (only if pulseDef is freq)
  pulseCfg.freq = 100;
  // Time Pulse Frequency when locked to GNSS time (Hz) (only if pulseDef is freq and useLocked set)
  pulseCfg.freqLock = 0;
  // Time Pulse Length (us) (only if lengthDef is length)
  pulseCfg.pulseLength = 100000; // 100ms pulse
  // Time Pulse length when locked to GNSS time (us) (only if lengthDef is length and useLocked set)
  pulseCfg.pulseLengthLock = 0;
  // Time Pulse Duty Cycle (%) (only if lengthDef is ratio)
  pulseCfg.duty = 50.0;
  // Time Pulse Duty Cycle when locked to GNSS time (%) (only if lengthDef is ratio and useLocked set)
  pulseCfg.dutyLock = 0.0;
  // User-configurable Time Pulse Delay (ns)
  pulseCfg.userDelay = 0;
  // Enable Time Pulse
  pulseCfg.enable = 1;
  // Sync time pulse to GNSS time (1) or local clock (0)
  pulseCfg.syncGnss = 1;
  // Use locked parameters when possible
  pulseCfg.useLocked = 0;
  // Align pulse to top of second (only if syncGnss set)
  pulseCfg.alignTOW = 0;
  // Falling edge at top of second (0) or rising edge at top of second (1)
  pulseCfg.polarity = 1;
  // Time Grid to use (only if syncGnss set)
  pulseCfg.timeGrid = GNSS_PULSE_UTC;

  gnss_set_pulse(&myGNSS, &pulseCfg);

  /*************************************
  * Change UART Baudrate (Comment out to use Default Speed)
  *************************************/
  /*
  #ifdef USE_UART

  timer_handle_t changeover_timer;
  timer_init(&changeover_timer, 1000000); // Wait 1s to receive any pending messages so we don't miss anything on the speed handover

  timer_start(&changeover_timer);

  while (!timer_check_exp(&changeover_timer))
    gnss_rec_and_parse(&myGNSS);

  Serial.println("Attempting to switch baudrate...");

  if(gnss_set_uart_baud(&myGNSS, GNSS_UART_BAUD_230400))
  {
    Serial.println("Failed to reset Serial, help!");
    while(1)
      ;
  }

  Serial.println("Successfully switched baudrate!");

  #endif
  */
}

void loop() {

  #if defined(USE_I2C) || defined(USE_SPI)
  if (gnss_msg == 1){
    gnss_msg = 0;
  #endif

  gnss_rec_and_parse(&myGNSS);

  #if defined(USE_I2C) || defined(USE_SPI)
  interrupt_enable(gnss_int);
  }
  #endif

  /*************************************
  * Output any Info Messages
  *************************************/
 
  if(!gnss_get_msg_info(&myGNSS, &info_msg))
  {
    Serial.print("New Info Message from Receiver: ");
    for (uint8_t i = 0; i < info_msg.length; i++)
    {
      Serial.print((char)info_msg.payload[i]);
    }
    Serial.println();
  }
  
  /*************************************
  * Output PVT and CLOCK Messages (See header for structs to access full messages)
  *************************************/

  // This function polls and receives UBX-NAV-PVT if not enabled for periodic messaging, or it checks if it received a new message if it is enabled for periodic messaging
  if(!gnss_get_nav_pvt(&myGNSS, &pvt_msg))
  {

    if (pvt_msg.validDate)
    {
      Serial.print("Date: "); Serial.print(pvt_msg.dayUTC); Serial.print("-"); Serial.print(pvt_msg.monthUTC); Serial.print("-"); Serial.println(pvt_msg.yearUTC);
    }
    else
    {
      Serial.println("Waiting for Valid Date...");
    }
    if(pvt_msg.validTime)
    {
      Serial.print("UTC Time: "); Serial.print(pvt_msg.hoursUTC); Serial.print(":"); Serial.print(pvt_msg.minutesUTC); Serial.print(":"); Serial.println(pvt_msg.secondsUTC);;
    }
    else
    {
      Serial.println("Waiting for Valid Time...");
    }

    if(pvt_msg.fixOK)
    {
      Serial.print("Latitude: "); Serial.println(pvt_msg.latitude, 7);
      Serial.print("Longitude: "); Serial.println(pvt_msg.longitude, 7);
      Serial.print("Height above Mean Sea Level (mm): "); Serial.println(pvt_msg.hMSL);
      Serial.print("Number of Satellites in View: "); Serial.println(pvt_msg.numSV);
      Serial.print("Speed over Ground (mm/s): "); Serial.println(pvt_msg.gSpeed);
    }
    else
    {
      Serial.println("Waiting for fix...");
    }

    Serial.println();

  }

  if(!gnss_get_nav_clock(&myGNSS, &clock_msg))
  {

    Serial.print("Clock Bias (ns): "); Serial.println(clock_msg.bias);
    Serial.print("Clock Drift (ns/s): "); Serial.println(clock_msg.drift);

    Serial.println();

  }

}

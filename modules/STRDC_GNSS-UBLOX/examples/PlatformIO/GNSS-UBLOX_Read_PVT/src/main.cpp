/*
 * Ublox GNSS Read PVT Example C++ File.
 *
 * @file        main.cpp
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

#ifdef SAM_M10Q
#define CHIP GNSS_SAM_M10Q
#endif

#ifdef NEO_M9N
#define CHIP GNSS_NEO_M9N
#endif

#ifdef NEO_M9V
#define CHIP GNSS_NEO_M9V
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
  Serial.println(" Stardust Orbital Ublox GNSS Read PVT Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  myGNSS.pinRst = RESET;

  
  #ifdef USE_I2C
  // I2C Configuration
  myGNSS.bus = &i2c1;
  myGNSS.busType = GNSS_I2C;
  myGNSS.busAddr = GNSS_ADDRESS;

  peripheral = 0;

  while(gnss_init(&myGNSS, 400000))
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

  while(gnss_init(&myGNSS, 3000000))
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

  peripheral = 1;

  while(gnss_init(&myGNSS, 9600))
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

  if (gnss_enable_rdy(&myGNSS, enable_pio, enable_polarity, threshold, ENABLE_PERIPH, CHIP))
  {
    Serial.println("Failed to enable TXREADY");
    while (1)
      ;
  }

  #endif

  /*************************************
  * Disable Signals (if desired) (Comment out to use Default)
  *************************************/
  /*
  // The receiver will prevent any invalid signal configurations
  // See gnss_update_signals() definition for more information

  gnss_signal_cfg_t signals;

  signals.gps_ena = 1;
  signals.gps_l1ca_ena = 1;
  signals.sbas_ena = 1;
  signals.sbas_l1ca_ena = 1;
  signals.gal_ena = 1;
  signals.gal_e1_ena = 1;
  signals.bds_ena = 1;
  signals.bds_b1_ena = 0;
  signals.bds_b1c_ena = 1;
  signals.qzss_ena = 1;
  signals.qzss_l1ca_ena = 1;
  signals.qzss_l1s_ena = 1;
  signals.glo_ena = 1;
  signals.glo_l1_ena = 1;

  
  if (gnss_update_signals(&myGNSS, &signals))
  {
    Serial.println("Failed to update signals");
    while (1)
      ;
  }
  */
  /*************************************
  * Setup Navigation Rates (Comment out to use Default)
  *************************************/
  /*
  // 250ms between measurements, 1 message per measurement
  if (gnss_set_nav_rate(&myGNSS, 250, 1, 0))
  {
    Serial.println("Failed to update navigation rate");
    while (1)
      ;
  }
  */
  /*************************************
  * Setup Static Hold (Comment out to use Default/Disable)
  *************************************/
  /*
  if (gnss_set_static_hold(&myGNSS, 100, 5))
  {
    Serial.println("Failed to set static hold");
    while (1)
      ;
  }
  */
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
  * Change UART Baudrate (Comment out to use Default 9600)
  *************************************/

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

}

void loop() {

  /*************************************
  * Read Messages
  *************************************/

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
  if(!gnss_get_pvt(&myGNSS, &pvt_msg))
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
      Serial.print("UTC Time: "); Serial.print(pvt_msg.hoursUTC); Serial.print(":"); Serial.print(pvt_msg.minutesUTC); Serial.print(":"); Serial.println(pvt_msg.secondsUTC);
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

  if(!gnss_get_clock(&myGNSS, &clock_msg))
  {

    Serial.print("Clock Bias (ns): "); Serial.println(clock_msg.bias);
    Serial.print("Clock Drift (ns/s): "); Serial.println(clock_msg.drift);

    Serial.println();

  }

}

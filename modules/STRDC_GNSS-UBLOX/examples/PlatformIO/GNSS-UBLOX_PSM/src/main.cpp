/*
 * Ublox GNSS Power Save Mode C++ Example.
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

#ifdef SAM_M10Q
#define CHIP GNSS_SAM_M10Q
#endif

#ifdef NEO_M9N
#define CHIP GNSS_NEO_M9N
#endif

// Create interrupt handler for easier enable/disable
isr_handle_t* gnss_int;

// PSM Configuration
gnss_psm_cfg_t psmCfg;

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
  Serial.println(" Stardust Orbital Ublox GNSS PSM Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  gpio_mode(EXTINT, GPIO_MODE_OUTPUT);
  gpio_write(EXTINT, GPIO_HIGH);

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

  if (gnss_set_msg_auto(&myGNSS, GNSS_UBX_NAV_CLOCK, 10, peripheral))
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

  /*************************************
  * Set PSM Configuration
  *************************************/

  gnss_rec_and_parse(&myGNSS);

  psmCfg.mode = GNSS_PM_PSMOO;
  // Position Update Period for PSMOO - Time between successive position fixes (s). Must be >= 5 but less than seconds in a week. If 0, receiver will never retry a fix and wait for external events
  psmCfg.posUpdatePeriod = 600;
  // Acquisition Period if previously failed to achieve a fix - Time before retry after failed position fix (s)
  psmCfg.acqPeriod = 60;
  // Position Update Period Grid Offset Relative to GPS start of week (s). Not used in PSMCT
  psmCfg.gridOffset = 0;
  // Time to stay in Tracking State (s) - If set to 0, receiver will only briefly enter tracking state after acquisition. How long the receiver stays in Tracking before POT (PSMCT) or Inactive for Update (PSMOO)
  psmCfg.onTime = 15;
  // Minimum time to spend in Acquisition State (s) - Minimum time to spend in acquisition even if signals are insufficient
  psmCfg.minAcqTime = 20;
  // Maximum time to spend in Acquisition State (s)
  psmCfg.maxAcqTime = 30;
  // Enable to prevent receiver from entering Inactive State after failing to achieve a fix
  psmCfg.doNotEnterOff = 0;
  // Disable to wait for normal fix OK before starting ONTIME, Enable for time fix
  psmCfg.waitTimeFix = 0;
  // Update ephemeris regularly (wakeup)
  psmCfg.updateEph = 0;

  // EXTINT pin select (if multiple exist on package), otherwise leave blank
  //psmCfg.extIntSel = 1;
  // EXTINT Pin Control (Wake) - Awake as long as EXTINT is HIGH
  psmCfg.extIntWake = 1;
  // EXTINT Pin Control (Backup) - Force BACKUP mode when EXTINT is LOW
  psmCfg.extIntBackup = 0;
  // EXTINT Pin Control (Inactive) - Force backup if EXTINT is inactive for longer than extIntInactivity
  psmCfg.extIntInactive = 0;
  // Inactivity timeout on ESTINT pin if enabled (ms)
  psmCfg.extIntInactivity = 0;
  // Limit Peak Current
  psmCfg.limitPeakCurr = 0;

  gnss_set_psm(&myGNSS, &psmCfg);

  gpio_write(EXTINT, GPIO_HIGH); // Wait for first Fix, might take longer than intermittent

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
  * Output PVT, CLOCK, and PSM Messages (See header for structs to access full messages)
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
      gpio_write(EXTINT, GPIO_LOW); // After first fix, allow normal operation to resume
    }
    else
    {
      Serial.println("Waiting for fix...");
    }

    Serial.print("PSM State: ");
    switch(pvt_msg.psmState)
    {
      case 0:
        Serial.println("PSM is not active");
        break;
      case 1:
        Serial.println("Enabled");
        break;
      case 2:
        Serial.println("Acquisition");
        break;
      case 3:
        Serial.println("Tracking");
        break;
      case 4:
        Serial.println("Power Optimized Tracking");
        break;
      case 5:
        Serial.println("Inactive");
        break;
      default:
        Serial.println("Unknown");
        break;
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

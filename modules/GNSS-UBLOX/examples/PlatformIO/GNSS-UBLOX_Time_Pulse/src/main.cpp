/*
 * Ublox GNSS Time Pulse C++ Example.
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

#define USE_I2C
//#define USE_UART

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

// Pin definitions
#ifdef USE_I2C
#define TX_RDY 8
#endif
#define EXTINT 4
#define RESET 9

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
  
  #ifdef USE_UART
  // UART Configuration
  myGNSS.bus = &uart2;
  myGNSS.busType = GNSS_UART;

  peripheral = 1;

  while(gnss_init(&myGNSS, 9600))
  {
    Serial.println("Failed to initialize GNSS Serial");
    gnss_reset_hw(&myGNSS);
    delay(1000);
  }
  #endif

  /*************************************
  * Interrupt Setup
  *************************************/

  #ifdef USE_I2C
  // TXREADY functionality is not available for UART per Ublox datasheet
  gnss_int = interrupt_init(TX_RDY, GPIO_HIGH, gnss_ISR); // Interrupt polarity depends on gnss_enable_rdy() function

  if (gnss_int == NULL)
    Serial.println("GNSS Failed to init interrupt");

  
  // Enable interrupt
  interrupt_set(gnss_int);

  gnss_enable_rdy(&myGNSS, 0x00, 0x00, 0x02, 0x00);
  #endif

  /*************************************
  * Setup Periodic Messaging
  *************************************/

  gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_RMC, 0, peripheral);
  gnss_set_msg_auto(&myGNSS, GNSS_UBX_NAV_PVT, 1, peripheral);
  gnss_set_msg_auto(&myGNSS, GNSS_UBX_NAV_CLOCK, 30, peripheral);
  gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GSA, 0, peripheral);
  gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GSV, 0, peripheral);
  gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GLL, 0, peripheral);
  gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_GGA, 0, peripheral);
  gnss_set_msg_auto(&myGNSS, GNSS_NMEA_STANDARD_VTG, 0, peripheral);

  /*************************************
  * Setup Time Pulse Configuration
  *************************************/

  gnss_pulse_cfg_t pulseCfg;

  pulseCfg.pulseDef = GNSS_PULSE_PERIOD;
  pulseCfg.lengthDef = GNSS_PULSE_LENGTH;
  pulseCfg.antDelay = 0;
  pulseCfg.period = 1000000; // 1s period
  pulseCfg.periodLock = 0;
  pulseCfg.freq = 100;
  pulseCfg.freqLock = 0;
  pulseCfg.pulseLength = 100000; // 100ms pulse
  pulseCfg.pulseLengthLock = 0;
  pulseCfg.duty = 50.0;
  pulseCfg.dutyLock = 0.0;
  pulseCfg.userDelay = 0;
  pulseCfg.enable = 1;
  pulseCfg.syncGnss = 1;
  pulseCfg.useLocked = 0;
  pulseCfg.alignTOW = 0;
  pulseCfg.polarity = 1;
  pulseCfg.timeGrid = GNSS_PULSE_UTC;

  gnss_update_pulse(&myGNSS, &pulseCfg);

  /*************************************
  * Change UART Baudrate
  *************************************/
  #ifdef USE_UART
  timer_handle_t startup_timer;
  timer_init(&startup_timer, 1000000); // Wait 1s to receive all startup messages so we don't miss anything on the speed handover

  timer_start(&startup_timer);

  while (!timer_check_exp(&startup_timer))
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

  #ifdef USE_I2C
  if (gnss_msg == 1){
    gnss_msg = 0;
  #endif

  gnss_rec_and_parse(&myGNSS);

  #ifdef USE_I2C
  interrupt_enable(gnss_int);
  }
  #endif

  if(!gnss_get_msg_info(&myGNSS, &info_msg))
  {
    Serial.print("New Info Message from Receiver: ");
    for (uint8_t i = 0; i < info_msg.length; i++)
    {
      Serial.print((char)info_msg.payload[i]);
    }
    Serial.println();
  }
  
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

  if(!gnss_get_clock(&myGNSS, &clock_msg))
  {

    Serial.print("Clock Bias (ns): "); Serial.println(clock_msg.bias);
    Serial.print("Clock Drift (ns/s): "); Serial.println(clock_msg.drift);

    Serial.println();

  }

}

/*
 * Ublox GNSS Record PVT Example Sketch.
 *
 * @file        GNSS-UBLOX_Record_PVT.ino
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
#include <SD.h>

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

// SD Card Info
#define SD_CS_PIN BUILTIN_SDCARD  // Teensy 4.1 built-in SD card
File dataFile;
char csvFilename[100];
bool sdCardAvailable = false;
unsigned long lastLogTime = 0;
#define LOG_INTERVAL 100  // Log every 100ms (10 Hz)

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
  Serial.println(" Stardust Orbital Ublox GNSS Record PVT Example ");
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
  * Disable Signals (if desired) (Comment out to use Default)
  *************************************/
  /*
  // The receiver will prevent any invalid signal configurations
  // See gnss_update_signals() definition for more information
  // Signals not supported by receiver will be ignored in configuration (and need not be set)

  gnss_signal_cfg_t signals;

  signals.gps_ena = 1; // GPS Enable
  signals.gps_l1ca_ena = 1; // GPS L1 C/A
  signals.gps_l5_ena = 1; // GPS L5
  signals.sbas_ena = 1; // SBAS Enable
  signals.sbas_l1ca_ena = 1; // SBAS L1 C/A
  signals.gal_ena = 1; // Galileo Enable
  signals.gal_e1_ena = 1; // Galileo E1
  signals.gal_e5a_ena = 1; // Galileo E5a
  signals.bds_ena = 1; // BeiDou Enable
  signals.bds_b1_ena = 0; // BeiDou B1I
  signals.bds_b1c_ena = 1; // BeiDou B1C
  signals.bds_b2a_ena = 1; // BeiDou B2a
  signals.qzss_ena = 1; // QZSS Enable
  signals.qzss_l1ca_ena = 1; // QZSS L1 C/A
  signals.qzss_l1s_ena = 1; // QZSS L1S
  signals.qzss_l5_ena = 1; // QZSS L5
  signals.glo_ena = 1; // GLONASS Enable
  signals.glo_l1_ena = 1; // GLONASS L1
  signals.navic_ena = 1; // NavIC Enable
  signals.navic_l5_ena = 1; // NavIC L5

  Serial.println("Setting signals (this may take a few seconds)...");
  
  if (gnss_set_signals(&myGNSS, &signals))
  {
    Serial.println("Failed to update signals");
    while (1)
      ;
  }
  */
  /*************************************
  * Override GPS L5 Unhealthy, Preoperational Status
  *************************************/
  /*
  // At this time, GPS L5 Signals are preoperational and are set unhealthy until sufficient monitoring capability is established.
  // GPS L5 signals may be evaluated before they become fully operational by overriding the GPS L5 health status with the respective GPS L1 C/A signal status.
  // DO NOT USE UNHEALTHY, PRE-OPERATIONAL GPS L5 SIGNALS FOR SAFETY-OF-LIFE OR OTHER CRITICAL PURPOSES.
  // See DAN-F10N Integration Manual 2.1.3 for more information.
  
  #ifdef DAN_F10N

  bool l5Override = 1;

  if (gnss_gps_l5_override(&myGNSS, l5Override))
  {
    Serial.println("Failed to update GPS L5 Override");
    while (1)
      ;
  }

  #endif
  */
  /*************************************
  * Setup Navigation Rates (Comment out to use Default)
  *************************************/
  
  // 250ms between measurements, 1 message per measurement
  if (gnss_set_nav_rate(&myGNSS, 250, 1, 0))
  {
    Serial.println("Failed to update navigation rate");
    while (1)
      ;
  }
  
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
  if(!gnss_get_nav_pvt(&myGNSS, &pvt_msg))
  {

    if (pvt_msg.validDate)
    {
      Serial.print("Date: "); Serial.print(pvt_msg.dayUTC); Serial.print("-"); Serial.print(pvt_msg.monthUTC); Serial.print("-"); Serial.println(pvt_msg.yearUTC);

      if(pvt_msg.validTime)
      {
        Serial.print("UTC Time: "); Serial.print(pvt_msg.hoursUTC); Serial.print(":"); Serial.print(pvt_msg.minutesUTC); Serial.print(":"); Serial.println(pvt_msg.secondsUTC);

        if (!sdCardAvailable)
        {
          /*************************************
          * SD Card Init
          *************************************/

          if (!SD.begin(SD_CS_PIN))
            sdCardAvailable = false;
          else
          {
            sdCardAvailable = true;

            // Create file with current datetime
            snprintf(csvFilename, sizeof(csvFilename), "tracker_%d%d%d_%d%d%d.csv", pvt_msg.yearUTC, pvt_msg.monthUTC, pvt_msg.dayUTC, pvt_msg.hoursUTC, pvt_msg.minutesUTC, pvt_msg.secondsUTC);
            dataFile = SD.open(csvFilename, FILE_WRITE);

            if (dataFile) {
              // Write header in file
              dataFile.println("timestamp_ms,gps_datetime,latitude,longitude,altitude_mm,velocity_mps,siv");
              dataFile.close();
            }
            else
              sdCardAvailable = false;
          }
        }
      }
      else
      {
        Serial.println("Waiting for Valid Time...");
      }
    }
    else
    {
      Serial.println("Waiting for Valid Date...");
    }

    if(pvt_msg.fixOK)
    {
      // Record data once fix is achieved
      unsigned long now = millis();

      char buf[35];
      snprintf(buf, sizeof(buf), "%04u-%02u-%02uT%02u:%02u:%02uZ", pvt_msg.yearUTC, pvt_msg.monthUTC, pvt_msg.dayUTC, pvt_msg.hoursUTC, pvt_msg.minutesUTC, pvt_msg.secondsUTC);
      String gpsDateTime = buf;
      
      if (sdCardAvailable && (now - lastLogTime > LOG_INTERVAL)) {
        dataFile = SD.open(csvFilename, FILE_WRITE);
        if (dataFile) {
          dataFile.print(now);
          dataFile.print(",");
          dataFile.print(gpsDateTime);
          dataFile.print(",");
          dataFile.print(pvt_msg.latitude);
          dataFile.print(",");
          dataFile.print(pvt_msg.longitude);
          dataFile.print(",");
          dataFile.print(pvt_msg.hMSL);
          dataFile.print(",");
          dataFile.print(pvt_msg.gSpeed / 1000.0, 3);
          dataFile.print(",");
          dataFile.print(pvt_msg.numSV);
          dataFile.println();
          dataFile.close();
          lastLogTime = now;
        }
      }

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

}

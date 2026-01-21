/*
 * Ublox GNSS Batch Example Sketch.
 *
 * @file        GNSS-UBLOX_Batch_Messages.ino
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

#ifdef SAM_M10Q
#define CHIP GNSS_SAM_M10Q
#endif

// Create interrupt handler for easier enable/disable
isr_handle_t* gnss_int;

// Poll variable changed in INT
volatile uint8_t gnss_msg;

// Messages we will be using
gnss_info_t info_msg; // Generic Info messages from both UBX & NMEA

gnss_ubx_mon_batch_t monMsg; // UBX-MON-BATCH
gnss_ubx_log_batch_t** batches; // Pointer of Batch message Array

gnss_ubx_log_batch_t batchMsg; // Individual Batch Messages

uint8_t peripheral;

// Timer for Batch
timer_handle_t batch_timer;
#define BATCH_TIME 10000000 // 10s (in us)

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
  Serial.println(" Stardust Orbital Ublox GNSS Batch Message Example ");
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

  if (gnss_set_msg_auto(&myGNSS, GNSS_UBX_NAV_PVT, 0, peripheral))
  {
    Serial.println("Failed to update UBX-NAV-PVT Message Rate");
    while (1)
      ;
  }

  if (gnss_set_msg_auto(&myGNSS, GNSS_UBX_NAV_CLOCK, 0, peripheral))
  {
    Serial.println("Failed to update UBX-NAV-CLOCK Message Rate");
    while (1)
      ;
  }

  /*************************************
  * Configure Batch Messaging
  *************************************/

  gnss_batch_cfg_t batch_cfg;

  // Enable Data Batching - Requires maxEntries to be set
  batch_cfg.enable = 1;
  // Enable PIO Notification when buffer fill level exceeds warnThresh
  batch_cfg.pioEnable = 0;
  // Maximum Entries in Buffer (num epochs) - Will be rejected if exceeds available memory
  batch_cfg.maxEntries = 20;
  // Buffer fill level that triggers PIO notification
  batch_cfg.warnThresh = 5;
  // Polarity for PIO, set for active low, otherwise active high
  batch_cfg.pioActiveLow = 0;
  // ID of PIO for buffer fill level notification
  batch_cfg.pioID = 5;
  // Include additional PVT information in batch messages (see interface description)
  batch_cfg.extraPVT = 1;
  // Include additional ODO information in batch messages (see interface description)
  batch_cfg.extraODO = 1;

  if(gnss_set_batch(&myGNSS, &batch_cfg, CHIP))
  {
    Serial.println("Failed to update batch configuration");
    while(1)
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

  timer_init(&batch_timer, BATCH_TIME);
  timer_start(&batch_timer);

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
  
  if (timer_check_exp(&batch_timer))
  {

    #if defined(USE_I2C) || defined(USE_SPI)
    interrupt_disable(gnss_int); // Disable interrupt for Batch operation
    #endif

    if (!gnss_retrieve_batch(&myGNSS, &monMsg, &batches))
    {

      Serial.print("Batch Messages Retrieved: "); Serial.println(monMsg.fillLevel);
      
      for (uint16_t i = 0; i < monMsg.fillLevel; i++)
      {
        
        batchMsg = *batches[i]; // Grab individual batch message
        Serial.print("iTOW: "); Serial.println(batchMsg.iTOW);
        Serial.print("msgCnt: "); Serial.println(batchMsg.msgCnt);
        Serial.print("numSV: "); Serial.println(batchMsg.numSV);
        if (batchMsg.extraPVT)
        {
          if (batchMsg.gnssFixOK)
          {
            Serial.print("Longitude (deg): "); Serial.println(batchMsg.longitude, 7);
            Serial.print("Latitude (deg): "); Serial.println(batchMsg.latitude, 7);
            Serial.print("hMSL (mm): "); Serial.println(batchMsg.hMSL);
          }
          else
            Serial.println("Waiting for Fix...");
        }
        
        Serial.println();
      }
    }
    else
      Serial.println("No Batch Messages to Retrieve");

    #ifdef USE_I2C
    interrupt_enable(gnss_int); // Enable interrupt after Batch operation
    #endif

    timer_reset(&batch_timer); // Reset timer
  }


}

/*
 * BMP390 FIFO Example Sketch.
 *
 * @file        BMP390_FIFO.ino
 * @author      Alex Zundel
 * @copyright   Copyright (c) 2026 Stardust Orbital
 *
 * MIT License
 * 
 * Copyright (c) 2026 Stardust Orbital
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
#include "bmp390.h"

/*************************************
 * DEFINITIONS
 *************************************/

bmp390_t myBmp; // Our BMP390 object

// Create interrupt handler for easier enable/disable
isr_handle_t* bmp390_int;

// Poll variable changed in INT
volatile uint8_t bmp390_msg;

// Select desired peripheral, initialization, pinout, interrupt will be configured automatically (if USE_INT is defined)
#define USE_I2C
//#define USE_SPI_4_WIRE
//#define USE_SPI_3_WIRE

// Uncomment USE_INT in order to setup and use the interrupt
//#define USE_INT
#define USE_POLL

//#define BMP390_ADDRESS 0x76 // I2C Address if SDO is connected to GND
#define BMP390_ADDRESS 0x77 // I2C Address if SDO is connected to VDDIO

#ifdef USE_SPI_4_WIRE
#define BMP390_SS 0 // Use CS_0 pin on bus
#endif

#define INT_PIN 8

// Timer to trigger a FIFO Reading
timer_handle_t poll_timer;
uint32_t poll_interval = 2000000; // Interval in us

// ISR to notify program to read messages
void bmp390_ISR() {
  bmp390_msg = 1;

  // Disable interrupt until we read the message
  interrupt_disable(bmp390_int);
}

void setup() {

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital BMP390 FIFO ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/
  
  #ifdef USE_I2C
  // I2C Configuration
  myBmp.bus = &i2c1; // Pointer to I2C bus (object). See HAL for more details
  myBmp.busType = BMP390_I2C;
  myBmp.busAddr = BMP390_ADDRESS;

  i2c_open((i2c_handle_t*)myBmp.bus, 400000); // Max Speed 3.4 MHz

  while(bmp390_init(&myBmp))
  {
    Serial.println("Failed to initialize BMP390 I2C");
    delay(1000);
  }
  #endif

  #ifdef USE_SPI_4_WIRE
  // SPI Configuration
  myBmp.bus = &SPI_0; // Pointer to SPI bus (object). See HAL for more details
  myBmp.busType = BMP390_SPI_4_WIRE;
  myBmp.busAddr = BMP390_SS;

  // Max speed is 10 MHz
  spi_open((spi_handle_t*)myBmp.bus, 5000000, SPI_MODE_3, SPI_BIT_ORDER_MSB); // Can operate on both Mode 0 and Mode 3, it switches automatically on CSB assert

  while(bmp390_init(&myBmp))
  {
    Serial.println("Failed to initialize BMP390 SPI");
    delay(1000);
  }
  #endif

  #ifdef USE_SPI_3_WIRE
  // SPI Configuration
  myBmp.bus = &SPI_0; // Pointer to SPI bus (object). See HAL for more details
  myBmp.busType = BMP390_SPI_3_WIRE;
  myBmp.busAddr = BMP390_SS;

  // Max speed is 10 MHz
  spi_open((spi_handle_t*)myBmp.bus, 5000000, SPI_MODE_3, SPI_BIT_ORDER_MSB); // Can operate on both Mode 0 and Mode 3, it switches automatically on CSB assert

  while(bmp390_init(&myBmp))
  {
    Serial.println("Failed to initialize BMP390 SPI 3-Wire");
    delay(1000);
  }
  #endif

  /*************************************
  * Output Data Rate and Sampling
  *************************************/

  // ODR settings
  if (bmp390_set_odr(&myBmp, BMP390_ODR_25))
  {
    Serial.println("Failed to set ODR");
    while (1)
      ;
  }

  // Oversampling
  // Note: If using oversampling (> x1), you must set the ODR prescaler to the same level or higher as the oversampling
  // See header file and datasheet for more details
  if (bmp390_set_osr(&myBmp, BMP390_OSR_4, BMP390_OSR_8))
  {
    Serial.println("Failed to set oversampling");
    while (1)
      ;
  }

  /*************************************
  * Filtering
  *************************************/
  
  // IIR Filter
  if (bmp390_set_iir_filter(&myBmp, BMP390_IIR_COEFF_3))
  {
    Serial.println("Failed to set IIR Filter");
    while (1)
      ;
  }

  /*************************************
  * Sensor Selection
  *************************************/

  // Enable/Disable Sensors
  bool pressEn = true;
  bool tempEn = true;

  if (bmp390_enable_sensors(&myBmp, pressEn, tempEn))
  {
    Serial.println("Failed to set sensor enable configuration");
    while (1)
      ;
  }

  /*************************************
  * I2C Watchdog
  *************************************/

  #ifdef USE_I2C

  // Enable/disable watchdog
  bool watchdogEn = true;
  bool period = false; // Enable for long period

  if (bmp390_set_watchdog(&myBmp, watchdogEn, period))
  {
    Serial.println("Failed to set I2C Watchdog");
    while (1)
      ;
  }

  #endif

  /*************************************
  * Mode Selection
  *************************************/

  // Set power mode
  // Note: init() will default to Normal Mode
  if (bmp390_set_power_mode(&myBmp, BMP390_MODE_NORMAL))
  {
    Serial.println("Failed to set power mode");
    while (1)
      ;
  }

  /*************************************
  * FIFO Setup
  *************************************/

  fifoCfg_t fifoCfg;

  fifoCfg.en = true; // Enable FIFO
  fifoCfg.stopOnFull = true; // Enable to stop writing to FIFO when full
  fifoCfg.timeEn = true; // Enable to return sensortime frame
  fifoCfg.pressEn = true; // Enable to store pressure data
  fifoCfg.tempEn = true; // Enable to store temperature data
  fifoCfg.subsampling = BMP390_ODR_50; // FIFO downsampling for pressure and temperature - 2 ^ subsampling. This is on-top of the general ODR setting
  fifoCfg.dataSelect = BMP390_FIFO_SOURCE_FILTERED; // Data source for pressure and temperature data
  //fifoCfg.dataSelect = BMP390_FIFO_SOURCE_UNFILTERED;
  fifoCfg.watermark = 256; // Up to 512 bytes

  if (bmp390_set_fifo(&myBmp, &fifoCfg))
  {
    Serial.println("Failed to set FIFO configuration");
    while (1)
      ;
  }

  /*************************************
  * Interrupt Setup
  *************************************/

  #ifdef USE_INT
  bmp390_int = interrupt_init(INT_PIN, GPIO_HIGH, bmp390_ISR); // Interrupt polarity depends on IEA bit in INT_CTRL_REG

  if (bmp390_int == NULL)
  {
    Serial.println("BMP390 Failed to init interrupt");
    while (1)
      ;
  }
  
  // Enable interrupt
  interrupt_set(bmp390_int);
  

  bool output = false; // Type of output on INT pin. 0: Push-pull, 1: Open-drain
  bool polarity = false; // 0: Active Low, 1: Active High
  bool latch = true; // Enable/disable latching for INT pin and INT_STATUS register.
  bool watermark = false; // Enable/disable FIFO watermark interrupt for INT pin and INT_STATUS register.
  bool full = true; // Enable/disable FIFO full interrupt for INT pin and INT_STATUS register.
  bool drdy = true; // Enable/disable temperature & pressure data ready interrupt for INT pin and INT_STATUS register.

  if (bmp390_set_interrupt(&myBmp, output, polarity, latch, watermark, full, drdy))
  {
    Serial.println("Failed to set BMP390 interrupt");
    while (1)
      ;
  }

  #endif

  /*************************************
  * Timer Setup
  *************************************/

  timer_init(&poll_timer, poll_interval); // Initialize timer
  timer_start(&poll_timer); // Start timer

}

// Values for use in loop()
double meas_press = 0;
double meas_temp = 0;
uint32_t meas_time = 0;
uint8_t int_status = 0x00;
uint16_t count = 0;

void loop() {

  /*************************************
  * Read Sensor
  *************************************/

  #ifdef USE_INT
  if (bmp390_msg == 1){
    bmp390_msg = 0;

    if (bmp390_get_int_status(&myBmp, &int_status))
      Serial.println("Failed to get INT status");

    if (int_status & BMP390_INT_FIFO_WATERMARK) // Check if interrupted for FIFO Watermark
    {
  #endif

  #ifdef USE_POLL
  if (timer_check_exp(&poll_timer) != 0){

    timer_reset(&poll_timer);

  #endif

  Serial.println("=====================");
  Serial.println("Checking FIFO...");

  if (bmp390_read_fifo(&myBmp))
    Serial.println("Failed to read FIFO buffer");
  
  count = 0;

  // Output FIFO values stored in handle
  while (!bmp390_retrieve_fifo_value(&myBmp, &meas_temp, &meas_press, &meas_time)) // Each function call retrieves a single value stored in the handle's buffer
  {
    Serial.println("=====================");
    Serial.print("Time: "); Serial.print(meas_time); Serial.print(" Frame: "); Serial.println(count); // Time output by sensor is time at which last FIFO data frame is read
    Serial.print("Temperature: "); Serial.print(meas_temp); Serial.println(" C");
    Serial.print("Pressure: "); Serial.print(meas_press); Serial.println(" Pa");
    count++;
  }

  #ifdef USE_POLL
  }
  #endif

  #ifdef USE_INT
    }
    interrupt_enable(bmp390_int); // Re-enable interrupt
  }
  #endif

}

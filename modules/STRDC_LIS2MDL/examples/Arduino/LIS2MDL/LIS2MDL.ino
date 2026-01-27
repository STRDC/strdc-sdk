/*
 * LIS2MDL Magnetometer Example Sketch.
 *
 * @file        LIS2MDL_magnetometer.ino
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
#include "lis2mdl.h"

/*************************************
 * DEFINITIONS
 *************************************/

lis2mdl_t myLis; // Our LIS2MDL object

// Create interrupt handler for easier enable/disable
isr_handle_t* lis2mdl_int;

// Poll variable changed in INT
volatile uint8_t lis2mdl_msg;

// Select desired peripheral, initialization, pinout, interrupt will be configured automatically (if USE_INT is defined)
#define USE_I2C
//#define USE_SPI_4_WIRE
//#define USE_SPI_3_WIRE

// Uncomment USE_INT in order to setup and use the interrupt
//#define USE_INT

#define LIS2MDL_ADDRESS 0x1E // I2C Address

#ifdef USE_SPI_4_WIRE
#define LIS2MDL_SS 0 // Use CS_0 pin on bus
#endif

#define INT_PIN 8

// ISR to notify program to read messages
void lis2mdl_ISR() {
  lis2mdl_msg = 1;

  // Disable interrupt until we read the message
  interrupt_disable(lis2mdl_int);
}

void setup() {

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital LIS2MDL Magnetometer ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/
  
  #ifdef USE_I2C
  // I2C Configuration
  myLis.bus = &i2c1; // Pointer to I2C bus (object). See HAL for more details
  myLis.busType = LIS2MDL_I2C;
  myLis.busAddr = LIS2MDL_ADDRESS;

  i2c_open((i2c_handle_t*)myLis.bus, 400000); // Max speed is 3.4 MHz

  while(lis2mdl_init(&myLis))
  {
    Serial.println("Failed to initialize LIS2MDL I2C");
    delay(1000);
  }
  #endif

  #ifdef USE_SPI_4_WIRE
  // SPI Configuration
  myLis.bus = &SPI_0; // Pointer to SPI bus (object). See HAL for more details
  myLis.busType = LIS2MDL_SPI_4_WIRE;
  myLis.busAddr = LIS2MDL_SS;

  spi_open((spi_handle_t*)handle->bus, 5000000, SPI_MODE_3, SPI_BIT_ORDER_MSB); // Max speed is 10 MHz

  while(lis2mdl_init(&myLis))
  {
    Serial.println("Failed to initialize LIS2MDL SPI");
    delay(1000);
  }
  #endif

  #ifdef USE_SPI_3_WIRE
  // SPI Configuration
  myLis.bus = &SPI_0; // Pointer to SPI bus (object). See HAL for more details
  myLis.busType = LIS2MDL_SPI_3_WIRE;
  myLis.busAddr = LIS2MDL_SS;

  spi_open((spi_handle_t*)handle->bus, 5000000, SPI_MODE_3, SPI_BIT_ORDER_MSB); // Max speed is 10 MHz

  while(lis2mdl_init(&myLis))
  {
    Serial.println("Failed to initialize LIS2MDL SPI 3-Wire");
    delay(1000);
  }
  #endif

  /*************************************
  * Update Sensor Rate (Uncomment to use)
  *************************************/
  /*
  if (lis2mdl_set_rate(&myLis, LIS2MDL_20HZ))
  {
    Serial.println("Failed to update sensor rate");
    while (1)
      ;
  }
  */
  /*************************************
  * Set Low Pass Filter (Uncomment to use)
  *************************************/
  /*
  lis2mdl_set_low_pass(&myLis, true)
  {
    Serial.println("Failed to set low pass filter");
    while (1)
      ;
  }
  */
  /*************************************
  * Set Sensor Offsets (Uncomment to use)
  *************************************/
  /*
  float xOffset = 100.0;
  float yOffset = 100.0;
  float zOffset = 100.0;

  if (lis2mdl_set_offsets(&myLis, xOffset, yOffset, zOffset))
  {
    Serial.println("Failed to set sensor offsets");
    while (1)
      ;
  }
  */
  /*************************************
  * Set Operation Mode (Uncomment to use)
  *************************************/
  /*
  if (lis2mdl_set_op_mode(&myLis, LIS2MDL_MODE_CONTINUOUS)) // Default: Continuous. Other options are single and idle (see datasheet for more information).
  {
    Serial.println("Failed to set operation mode");
    while (1)
      ;
  }
  */
  /*************************************
  * Set Low Power Mode (Uncomment to use)
  *************************************/
  /*
  if (lis2mdl_set_low_power(&myLis, true))
  {
    Serial.println("Failed to set low power mode");
    while (1)
      ;
  }
  */
  /*************************************
  * Interrupt Setup
  *************************************/

  #ifdef USE_INT
  lis2mdl_int = interrupt_init(INT_PIN, GPIO_HIGH, lis2mdl_ISR); // Interrupt polarity depends on IEA bit in INT_CTRL_REG

  if (lis2mdl_int == NULL)
    Serial.println("LIS2MDL Failed to init interrupt");
  
  // Enable interrupt
  interrupt_set(lis2mdl_int);

  bool x_int = true; // Enable x-field direction interrupt
  bool y_int = true; // Enable y-field direction interrupt
  bool z_int = true; // Enable z-field direction interrupt
  bool polarity = true; // Set INT bit HIGH upon interrupt
  bool latch = true; // Set for INT to latch until status message is read
  bool enabled = true; // Enable interrupt

  if(lis2mdl_set_int_cfg(&myLis, x_int, y_int, z_int, polarity, latch, enabled))
  {
    Serial.println("Failed to set interrupt configuration");
    while (1)
      ;
  }

  /************* For Setting Threshold Interrupt ****************/
  /*
  float threshold_int = 200.0;

  if (lis2mdl_set_threshold(&myLis, threshold_int))
  {
    Serial.println("Failed to set threshold for interrupt");
    while (1)
      ;
  }
  */

  bool threshold = false; // Set for threshold interrupt
  bool newData = true; // Set for new data interrupt

  if (lis2mdl_enable_int(&myLis, threshold, newData))
  {
    Serial.println("Failed to enable interrupt for new data");
    while (1)
      ;
  }

  #endif

}

// Initialize Values
float magX = 0;
float magY = 0;
float magZ = 0;
float temp = 0;

uint8_t status;

void loop() {

  /*************************************
  * Read Sensor
  *************************************/

  #ifdef USE_INT
  if (lis2mdl_msg == 1){
    lis2mdl_msg = 0;

    lis2mdl_get_int_source(&myLis, &status); // Read status to release latch

  #endif

  // Note, if not using Zyxda bit from STATUS_REG to synchronize reads, must set BDU bit (CFG_REG_C) to guarentee LSB & MSB registers are sampled at the same time
  lis2mdl_get_status(&myLis, &status); // Read status 

  if (status | LIS2MDL_STATUS_NEW_ANY) // If status shows there's new data
  {
    lis2mdl_get_mags(&myLis, &magX, &magY, &magZ); // Read sensor data

    lis2mdl_get_internal_temp(&myLis, &temp); // Read internal temperature

    Serial.print("Incoming reading: ");
    Serial.print(magX); Serial.print(" mgauss, ");
    Serial.print(magY); Serial.print(" mgauss, ");
    Serial.print(magZ); Serial.print(" mgauss, ");
    Serial.print(temp); Serial.println(" C");
  }

  #ifdef USE_INT
  interrupt_enable(lis2mdl_int);
  }
  #endif

}

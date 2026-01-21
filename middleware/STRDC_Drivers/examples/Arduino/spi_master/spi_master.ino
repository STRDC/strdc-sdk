/*
 * SPI Master Example Sketch.
 *
 * @file        spi_master.ino
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

#include "spi_master.h"

/*************************************
 * DEFINITIONS
 *************************************/

#define SS_0 0

void setup()
{
    
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital Drivers SPI Master Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  // Specify SS pins on hardware (these can be set by default within the HAL at hal_spi_master.cpp)
  SPI_0.SS0 = 10;
  SPI_0.SS1 = 14;
  SPI_0.SS2 = 15;

  // SPI peripherals are defined in HAL (hal_spi_master.cpp and header)
  spi_open(&SPI_0, 5000000, SPI_MODE_0, SPI_BIT_ORDER_MSB); // Open SPI peripheral at 5MHz

  /*************************************
  * SPI Write Read
  *************************************/
  /*
  * In this write, we send a single data byte to a 2-byte register and read during the full-duplex communication
  */

  uint8_t reg[2]; // 2-byte register to address
  reg[0] = 0x00;
  reg[1] = 0x01;

  uint8_t tx[3]; // Total Data Bytes
  tx[0] = reg[0];
  tx[1] = reg[1];
  tx[2] = 0x02; // Single data byte

  uint8_t recBuffer[3]; // Buffer to receive dtaa

  memset(recBuffer, 0x00, 3); // Reset recBuffer to 0x00s

  uint16_t length = 3; // Length of register + data

  spi_write_read(&SPI_0, SS_0, tx, recBuffer, length); // Full-Duplex write to the SS0. Any data transmitted during the write from the peripheral will be stored in recBuffer

  Serial.print("SPI Read during Write: ");
  for (uint16_t i = 0; i < length; i++)
  {
    Serial.print(recBuffer[i]); Serial.print(" ");
  }
  Serial.println();

  /*************************************
  * SPI Write Only
  *************************************/
  /*
  * In this write, we send a single data byte to a 2-byte register and discard any read during the full-duplex communication
  */

  uint8_t addr[2]; // 2-byte register to address
  addr[0] = 0x00;
  addr[1] = 0x01;

  uint8_t data[3]; // Total Data Bytes
  data[0] = addr[0];
  data[1] = addr[1];
  data[2] = 0x02; // Single data byte

  uint16_t leng = 3; // Length of register + data

  spi_write(&SPI_0, SS_0, data, leng); // Full-Duplex write to the SS0. Any data transmitted during the write from the peripheral will be discarded

  /*************************************
  * SPI Read Only
  *************************************/
  /*
  * In this read, we receive 6 bytes. During full-duplex communication, the SPI peripheral will write empty bytes (0x00) to the peripheral
  */

  uint8_t buffer[6];
  memset(buffer, 0x00, 6); // Reset data to empty (0x00)

  uint16_t readLength = 6;

  spi_read(&SPI_0, SS_0, buffer, readLength); // Full-Duplex read from the SS0. Any data transmitted during the read from the SPI peripheral will be empty bytes

  Serial.print("SPI Read: ");
  for (uint16_t i = 0; i < readLength; i++)
  {
    Serial.print(buffer[i]); Serial.print(" ");
  }
  Serial.println();

}

void loop() {


}

/*
 * I2C Master Example Sketch.
 *
 * @file        i2c_master.ino
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

#include "i2c_master.h"

/*************************************
 * DEFINITIONS
 *************************************/

// I2C Address Info
#define I2C_ADDRESS 0x77

void setup()
{
    
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital Drivers I2C Master Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  // I2C peripherals are defined in HAL (hal_i2c_master.cpp and header)
  i2c_open(&i2c1, 400000); // Open I2C peripheral at 400kHz
        
  if(i2c_find(&i2c1, I2C_ADDRESS)) // Search for I2C address, confirm it's on the bus
  {
    i2c_close(&i2c1); // Close peripheral if device wasn't found

    while (1)
        Serial.println("Failed to find device");

  }
    
  i2c_set_addr(&i2c1, I2C_ADDRESS); // Once set, future messages will be sent to this address

  /*************************************
  * I2C Write
  *************************************/
  /*
  * In this write, we send a single data byte to a 2-byte register
  */

  uint8_t reg[2]; // 2-byte register to address
  reg[0] = 0x00;
  reg[1] = 0x01;

  uint8_t data[3]; // Total Data Bytes
  data[0] = reg[0];
  data[1] = reg[1];
  data[2] = 0x02; // Single data byte

  uint16_t length = 3; // Length of register + data

  if (i2c_write(&i2c1, data, length)) // Return 0 if successful
    Serial.println("Failed to write I2C message");

  /*************************************
  * I2C Read from Specific Register
  *************************************/
  /*
  * In this read, we read 4 bytes from a 2-byte register
  */

  uint8_t regAddr[1]; // 2-byte register to address
  regAddr[0] = 0x00;
  regAddr[1] = 0x01;

  uint8_t regLength = 1; // Length of the register to read from in bytes

  uint8_t readBuffer[1]; // Buffer to store data

  uint16_t readLength = 1; // Length of data to read

  if (i2c_read_reg(&i2c1, regAddr, regLength, readBuffer, readLength)) // Return 0 if successful
    Serial.println("Failed to read I2C message from register");
  else
  {
    Serial.print("I2C Read Register: ");
    for (uint8_t i = 0; i < readLength; i++)
    {
      Serial.print(readBuffer[i]); Serial.print(" ");
    }
    Serial.println();
  }

  /*************************************
  * I2C Read from Current Register
  *************************************/
  /*
  * In this read, we read a single byte from the current register
  */

  uint8_t buffer;
  uint16_t leng = 1;

  if (i2c_read(&i2c1, &buffer, leng)) // Return 0 if successful
    Serial.println("Failed to read I2C message");
  else
  {
    Serial.print("I2C Read Current Register: ");
    Serial.println(buffer);
  }

}

void loop() {


}

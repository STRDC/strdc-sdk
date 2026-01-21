/*
 * Serial Master Example C++ File.
 *
 * @file        main.cpp
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

#include "serial_master.h"

/*************************************
 * DEFINITIONS
 *************************************/

#define READ_EXTRA 1024
#define WRITE_EXTRA 512

uint8_t extraRBuffer[READ_EXTRA];
uint8_t extraWBuffer[WRITE_EXTRA];

void setup()
{
    
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital Drivers Serial Master Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

   if(serial_open(&uart2, 115200, UART_TYPE_BASIC))
    Serial.println("Failed to open UART");

  // Increase buffers (if desired)
  serial_buffer_read_add(&uart2, extraRBuffer, 1024);
  serial_buffer_write_add(&uart2, extraWBuffer, 512);

  /*************************************
  * UART Write
  *************************************/
  /*
  * In this write, we send 4 data bytes
  */

  uint8_t tx[4]; // Total Data Bytes
  tx[0] = 0x00;
  tx[1] = 0x01;
  tx[2] = 0x02;
  tx[3] = 0x03;

  serial_write(&uart2, tx, 4); // Send data

  /*************************************
  * UART Read
  *************************************/
  /*
  * In this read, we check how many bytes are available in the UART peripheral buffer and then read those
  */

  uint8_t buffer[128]; // Storage for read data

  uint16_t length = serial_read_available(&uart2); // Check data length in buffer

  if (length >= 64 + READ_EXTRA)
    Serial.println("Serial FIFO may have overflowed");

  if (length != 0)
  {
    if (serial_read(&uart2, buffer, length))
      Serial.println("Failed to read serial data, exiting...");
    else
    {
      Serial.print("UART Read: ");
      for (uint16_t i = 0; i < length; i++)
      {
        Serial.print(buffer[i]); Serial.print(" ");
      }
      Serial.println();
    }

  }

  /*************************************
  * UART Peek
  *************************************/
  /*
  * The Peek function outputs the next byte in the UART peripheral read buffer, but does not iterate the buffer (and lose the value)
  */

  uint8_t peekedVal = serial_peek(&uart2);

  Serial.print("Peeked Value: ");
  Serial.println(peekedVal);

  /*************************************
  * UART Buffer Clear
  *************************************/
  /*
  * Discard all unread data in the UART peripheral read buffer
  */

  serial_buffer_clear(&uart2);

  /*************************************
  * UART Close Peripheral
  *************************************/

  serial_close(&uart2);
  
}

void loop() {


}

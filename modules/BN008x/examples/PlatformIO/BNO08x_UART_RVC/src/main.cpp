/*
 * BNO08x UART RVC Example C++ File.
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

/*
 * Many thanks to JP Schramel and his BNO080 Rock Bottom Code for Arduino Atmega 328P 16Mhz (Arduino Nano etc) and Cortex M0 (Simblee)
 * It provided a solid foundation and reference for this effort. Find their work here: https://github.com/jps2000/BNO080/tree/master
 *
 * Additional thanks to Adafruit and Bryan Siepert for their implementation of CEVA's BN008x HAL which also provided a great reference.
 * Find that work here: https://github.com/adafruit/Adafruit_BNO08x/tree/master?tab=readme-ov-file
 */

#include "serial_master.h"
#include "timer.h"

// Rx on SDA Pin
// Tx on SCL Pin

void setup() {

  Serial.begin(115200);                  // 115200 baud
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital BNO08x UART_RVC Example ");
  Serial.println("***************************************");

  Serial3.begin(115200);

}

void loop() {

  uint8_t buff;
  uint8_t bytes[17];
  float yaw;
  float pitch;
  float roll;
  float x_accel;
  float y_accel;
  float z_accel;
  uint8_t chksum = 0;

  Serial.print("Received Data: ");
  while (1)
  {
    
    if(!serial_read(&uart3, &buff, 1))
    {
      if (buff == 0xAA) // First byte of header
      {
        if(!serial_read(&uart3, &buff, 1))
        {
          if (buff == 0xAA) // Second byte of header
          {
            memset(bytes, 0x00, 17);
            if (!serial_read(&uart3, bytes, 17))
            {
              for (uint8_t j = 0; j < 17; j++) // Header matches! 
              {
                if (j != 16) // Don't include checksum in checksum tabulation
                  chksum += bytes[j];
              }
              break;
            }
          }
        }
      }
    }
  }

  if(chksum == bytes[16]) // Confirm checksum matches and we have a valid message
  {

  yaw = (float)((int16_t)(bytes[1] | (bytes[2] << 8))) * 0.01;
  pitch = (float)((int16_t)(bytes[3] | ((int16_t)bytes[4] << 8))) * 0.01;
  roll = (float)((int16_t)(bytes[5] | ((int16_t)bytes[6] << 8))) * 0.01;

  x_accel = (float)((int16_t)(bytes[7] | ((int16_t)bytes[8] << 8))) * 0.001;
  y_accel = (float)((int16_t)(bytes[9] | ((int16_t)bytes[10] << 8))) * 0.001;
  z_accel = (float)((int16_t)(bytes[11] | ((int16_t)bytes[12] << 8))) * 0.001;

  Serial.print("yaw: "); Serial.print(yaw);
  Serial.print("; pitch: "); Serial.print(pitch);
  Serial.print("; roll: "); Serial.print(roll);

  Serial.print("; x_accel: "); Serial.print(x_accel);
  Serial.print("; y_accel: "); Serial.print(y_accel);
  Serial.print("; z_accel: "); Serial.println(z_accel);

  }
  else // Checksum doesn't match - Something went wrong
  {
    Serial.print("Checksum = "); Serial.print(chksum,HEX); Serial.print("; When it should be equal to: "); Serial.println(bytes[16]);
  }

}

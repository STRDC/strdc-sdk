/*
 * BNO08x Shake Detector Example Sketch.
 *
 * @file        BNO08x_Shake_Detector.ino
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

/*************************************
 * INCLUDES
 *************************************/

#include "interrupt.h"
#include "BNO08X.h"

/*************************************
 * DEFINITIONS
 *************************************/

bno08x_t bno;

// Select desired peripheral, initialization, pinout, interrupt will be configured automatically
//#define USE_I2C
#define USE_SPI
//#define USE_UART

// Select desired data gathering behavior
#define USE_INT
//#define USE_POLL

// Pin Definition
#define LED 3

#ifdef USE_SPI
// Pin definitions for SPI
#define BNO_PIN_INT 14
#define BNO_PIN_RST 20
#define BNO_PIN_WAKE 7
#endif

#ifdef USE_I2C
// Pin definitions for I2C
#define BNO_PIN_INT 17
#define BNO_PIN_RST 16
#define BNO_PIN_WAKE 15
#endif

#ifdef USE_UART
// Pin definitions for UART
#define BNO_PIN_INT 17
#define BNO_PIN_RST 16
#define BNO_PIN_WAKE 6
#define UART_BUFFER_EXTRA // Unnecessary for board configurations with excess of 300 byte read buffers and other communication configurations
#endif

// BNO08x Info
#define BNO_ADDRESS 0x4A
#define BNO_SS 0

// Sensor to enable
const uint8_t report = BNO08X_SENSOR_SHAKE_DETECT;

// Create struct to keep data organized
typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t z;
  uint8_t status;
  uint32_t timestamp;
} bno_data_t;

bno_data_t bno_data;

// Create interrupt handler for easier enable/disable
isr_handle_t* bno_int;

// Poll variable changed in INT
volatile uint8_t bno_msg;

int print_interval = 10000; // Print interval in us
int poll_interval = 200000; // In us

const int reporting_frequency = 400; // Reporting frequency in Hz

uint32_t rate = 1000000 / reporting_frequency; 

// Utilities
#define QP(n) (1.0f / (1 << n)) // 1 << n ==  2^-n
#define radtodeg (180.0f / PI)

// Helpers
timer_handle_t loop_timer; // Create timer to print data at a steady rate

// ISR to notify program to read messages
void bno_ISR() {
  bno_msg = 1;

  // Disable interrupt until we read the message
  interrupt_disable(bno_int);
}

// Format data in handler to printable version
void get_data(bno08x_t * ic, bno_data_t * bno_data)
{
  uint8_t x, y, z;

  bno_data->timestamp = bno08x_get_shake_detect(ic, &(bno_data->status), &x, &y, &z); // Timestamp in 100s us since interrupt

  bno_data->x = x; bno_data->y = y; bno_data->z = z; // Assign to data structure

}

void setup() {

  #ifdef USE_SPI
  // SPI Configuration
  bno.bus = &SPI_0;
  bno.busType = BNO08X_SPI;
  bno.busAddr = BNO_SS;
  #endif
  
  #ifdef USE_I2C
  // I2C Configuration
  bno.bus = &i2c1;
  bno.busType = BNO08X_I2C;
  bno.busAddr = BNO_ADDRESS;
  #endif
  
  #ifdef USE_UART
  // UART Configuration
  bno.bus = &uart2;
  bno.busType = BNO08X_UART;
  #endif
  
  // Shared Pin Configuration
  bno.wakePin = BNO_PIN_WAKE;
  bno.pinInt = BNO_PIN_INT;
  bno.pinRst = BNO_PIN_RST;
  
  // Initialize GPIO not handled in BNO handler
  gpio_mode(LED, OUTPUT);
  gpio_write(LED, HIGH);

  timer_init(&loop_timer, print_interval); // Initialize timer

  Serial.begin(115200); // 115200 baud
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital BNO08x Shake Detector Example ");
  Serial.println("***************************************");

  // Initialize BNO chip
  uint8_t init = 1;

  while (init)
  {
    init = bno08x_init(&bno, 1000000); // UART must be 3Mbaud (3000000)
    switch(init)
    {
      case 0:
        Serial.println("BNO Initialized Successfully");
        break;
      case 1:
        Serial.println("Failed waiting for interrupt");
        break;
      case 2:
        Serial.println("Failed to Find I2C Device");
        delay(1000);
        bno08x_hw_reset(&bno);
        break;
      case 3:
        Serial.println("Failed to Receive Startup Advertisement");
        delay(1000);
        bno08x_hw_reset(&bno);
        break;
      case 4:
        Serial.println("Failed to Receive Hub Startup Message");
        delay(1000);
        bno08x_hw_reset(&bno);
        break;
      case 5:
        Serial.println("Failed to Get Product ID");
        delay(1000);
        bno08x_hw_reset(&bno);
        break;
      case 6:
        Serial.println("Failed to Receive Reset Complete");
        delay(1000);
        bno08x_hw_reset(&bno);
        break;
      default:
        break;
    }
  }

  // Set Sensor Configuration

  uint8_t shake_config[20];
  shake_config[0] = 0x50; // 1 Word (4 bytes) for minimum time
  shake_config[1] = 0xC3;
  shake_config[2] = 0x00;
  shake_config[3] = 0x00;
  shake_config[4] = 0x80; // 1 Word (4 bytes) for maximum time
  shake_config[5] = 0x1A;
  shake_config[6] = 0x61;
  shake_config[7] = 0x00;
  shake_config[8] = 0x00; // 1 Word (4 bytes) for threshold (acceleration)
  shake_config[9] = 0x00;
  shake_config[10] = 0x00;
  shake_config[11] = 0x03;
  shake_config[12] = 0x03; // 1 Word (4 bytes) for shake count
  shake_config[13] = 0x00;
  shake_config[14] = 0x00;
  shake_config[15] = 0x00;
  shake_config[16] = 0x07; // 1 Word (4 bytes) for enable flags
  shake_config[17] = 0x00;
  shake_config[18] = 0x00;
  shake_config[19] = 0x00;

  while (bno08x_FRS_write(&bno, BNO08X_FRS_REC_SHAKE_DETECT, shake_config, 0x0000, 5)) // No offset, 5 words
    Serial.println("Failed to update Shake Detector Configuration");

  // Reboot chip (required for changes to take effect)
  while (bno08x_init(&bno, 400000))
    Serial.println("Attempting to reboot after FRS write");

  uint16_t sensitivity = 0;
  uint32_t interval = rate;
  uint32_t configWord = 0;
  
  Serial.println("BNO Begin Feature Set");

  // Enable Sensor
  while(bno08x_feature_set(&bno, report, 2, sensitivity, interval, 0, configWord))
    Serial.println("BNO Failed to set feature report");
  
  Serial.println("BNO Begin MECal Config");

  // Enable/disable calibration routines
  if (bno08x_mecal_config(&bno, 1, 1, 1, 1, 0))
    Serial.println("BNO Cal failed");

  Serial.println("BNO Finished MECal Config");

  timer_start(&loop_timer); // Start loop timer
  
  #ifdef USE_INT
  // Generate interrupt handler
  bno_int = interrupt_init(BNO_PIN_INT, GPIO_LOW, bno_ISR); // Interrupt is active low
    if (bno_int == NULL)
      Serial.println("BNO Failed to init interrupt");
  
  // Enable interrupt
  interrupt_set(bno_int);
  #endif
  
  Serial.println("Begin loop()");

}

void loop() {
  
  #ifdef USE_INT
  // Read messages if received interrupt
  if (bno_msg == 1){
  #endif

    bno_msg = 0;

    // Read, parse, and store messages on the handler
    bno08x_get_messages(&bno);

    // Process report data and add to struct
    get_data(&bno, &bno_data);

    Serial.println("Shake Detected!");

    Serial.print("Accuracy: ");

    switch (bno_data.status){
      case 0:
        Serial.print("Unreliable");
        break;
      case 1:
        Serial.print("Low");
        break;
      case 2:
        Serial.print("Medium");
        break;
      case 3:
        Serial.print("High");
        break;
      default:
        break;
    }

    Serial.print(", ");        
    Serial.print("x: "); Serial.print(bno_data.x); Serial.print(", ");
    Serial.print("y: "); Serial.print (bno_data.y); Serial.print(", ");
    Serial.print("z: "); Serial.print (bno_data.z); Serial.println();

    #ifdef USE_INT
    // Re-enable after reading the message
    interrupt_enable(bno_int);
    #endif
  }
  
  
  if (bno_data.status == 3)  gpio_toggle(LED);
  else gpio_write(LED, LOW);


}

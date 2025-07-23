/*
 * BNO08x Rotation Vector Example Sketch.
 *
 * @file        STRDC_BNO08x_Rotation_Vector_Example.ino
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

// Pin Definition
#define LED 3
/*
// Pin definitions for SPI
#define BNO_PIN_INT 14
#define BNO_PIN_RST 20
#define BNO_PIN_WAKE 7
*/
// Pin definitions for I2C
#define BNO_PIN_INT 17
#define BNO_PIN_RST 16
#define BNO_PIN_WAKE 15


// BNO08x Info
#define BNO_ADDRESS 0x4A
#define BNO_SS 0

// Sensor to enable
const uint8_t report = BNO08X_SENSOR_ROT_VECTOR;

// Create struct to keep data organized
typedef struct {
  float Q0;
  float Q1;
  float Q2;
  float Q3;
  float ACC;
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
timer_handle_t poll_timer; // Create timer to read messages regularly

// ISR to notify program to read messages
void bno_ISR() {
  bno_msg = 1;

  // Disable interrupt until we read the message
  interrupt_disable(bno_int);
}

// Format data in handler to printable version
void get_data(bno08x_t * ic, bno_data_t * bno_data)
{
  int16_t q0, q1, q2, q3, acc_est;
  float a, b, c, d;

  bno_data->timestamp = bno08x_get_rot_vector(ic, &(bno_data->status), &q1, &q2, &q3, &q0, &acc_est); // Timestamp in 100s us since interrupt

  a = q0 * QP(14); b = q1 * QP(14); c = q2 * QP(14); d = q3 * QP(14); // Convert from Q-point

  bno_data->ACC = QP(12) * radtodeg; // Calculate accurace estimate
  
  bno_data->Q0 = a; bno_data->Q1 = b; bno_data->Q2 = c; bno_data->Q3 = d; // Assign to data structure

}

void setup() {
  /*
  // SPI Configuration
  bno.bus = &SPI_0;
  bno.busType = BNO08X_SPI;
  bno.busAddr = BNO_SS;
  */
  // I2C Configuration
  bno.bus = &i2c1;
  bno.busType = BNO08X_I2C;
  bno.busAddr = BNO_ADDRESS;
  
  // Shared Pin Configuration
  bno.wakePin = BNO_PIN_WAKE;
  bno.pinInt = BNO_PIN_INT;
  bno.pinRst = BNO_PIN_RST;
  
  // Initialize GPIO not handled in BNO handler
  gpio_mode(LED, OUTPUT);
  gpio_write(LED, HIGH);

  timer_init(&loop_timer, print_interval); // Initialize timer
  timer_init(&poll_timer, poll_interval); // Initialize timer

  Serial.begin(115200); // 115200 baud
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital BNO08x Rotation Vector Example ");
  Serial.println("***************************************");

  // Initialize BNO chip
  uint8_t init = 1;

  while (init)
  {
    init = bno08x_init(&bno, 400000);
    switch(init)
    {
      case 0:
        Serial.println("BNO Initialized Successfully");
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

  uint8_t sensitivity = 0;
  uint32_t interval = rate;
  uint32_t configWord = 0;
  
  Serial.println("BNO Begin Feature Set");

  // Enable Sensor
  while(bno08x_feature_set(&bno, report, 0, sensitivity, interval, 0, configWord))
    Serial.println("BNO Failed to set feature report");
  
  Serial.println("BNO Begin MECal Config");

  // Enable/disable calibration routines
  if (bno08x_mecal_config(&bno, 1, 1, 1, 1, 0))
    Serial.println("BNO Cal failed");

  Serial.println("BNO Finished MECal Config");

  timer_start(&loop_timer); // Start loop timer
  //timer_start(&poll_timer); // Start poll timer
  
  // Generate interrupt handler
  bno_int = interrupt_init(BNO_PIN_INT, GPIO_LOW, bno_ISR); // Interrupt is active low
    if (bno_int == NULL)
      Serial.println("BNO Failed to init interrupt");
  
  // Enable interrupt
  interrupt_set(bno_int);
  
  Serial.println("Begin loop()");

}

void loop() {
  
  // Read messages if received interrupt or poll timer expires
  if (bno_msg == 1){ // timer_check_exp(&poll_timer) for polling

    bno_msg = 0;

    // Read, parse, and store messages on the handler
    bno08x_get_messages(&bno);

    // Process report data and add to struct
    get_data(&bno, &bno_data);

    //timer_reset(&poll_timer); for polling

    // Re-enable after reading the message
    interrupt_enable(bno_int);
  }
  
  if (timer_check_exp(&loop_timer) != 0){
    interrupt_disable(bno_int);
    
    timer_reset(&loop_timer);

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
    Serial.print("q0: "); Serial.print(bno_data.Q0 + 0.0005f,4); Serial.print(", ");
    Serial.print("q1: "); Serial.print (bno_data.Q1 + 0.0005f,4); Serial.print(", ");
    Serial.print("q2: "); Serial.print (bno_data.Q2 + 0.0005f,4); Serial.print(", ");
    Serial.print("q3: "); Serial.print (bno_data.Q3 + 0.0005f,4); Serial.println();
    
    interrupt_enable(bno_int);
 }

  if (bno_data.status == 3)  gpio_toggle(LED);
  else gpio_write(LED, LOW);


}

/*
 * BNO08x Multiple Example C++ File.
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

/*************************************
 * INCLUDES
 *************************************/

#include "interrupt.h"
#include "BNO08X.h"

/*************************************
 * DEFINITIONS
 *************************************/

bno08x_t bno1;
bno08x_t bno2;

// Select desired peripheral, initialization, pinout, interrupt will be configured automatically
#define USE_I2C
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

bno_data_t bno_data1;
bno_data_t bno_data2 ;

// Create interrupt handler for easier enable/disable
isr_handle_t* bno_int1;
isr_handle_t* bno_int2;

// Poll variable changed in INT
volatile uint8_t bno_msg1;
volatile uint8_t bno_msg2;

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
void bno_ISR1() {
  bno_msg1 = 1;

  // Disable interrupt until we read the message
  interrupt_disable(bno_int1);
}

void bno_ISR2() {
  bno_msg2 = 1;

  // Disable interrupt until we read the message
  interrupt_disable(bno_int2);
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
  
  #ifdef USE_SPI
  // SPI Configuration
  bno1.bus = &SPI_0;
  bno1.busType = BNO08X_SPI;
  bno1.busAddr = BNO_SS;

  spi_open((spi_handle_t*)bno1.bus, 1000000, SPI_MODE_3, SPI_BIT_ORDER_MSB);

  #endif
  
  #ifdef USE_I2C
  // I2C Configuration
  bno2.bus = &i2c1;
  bno2.busType = BNO08X_I2C;
  bno2.busAddr = BNO_ADDRESS;

  i2c_open((i2c_handle_t*)bno2.bus, 400000);

  #endif
  
  #ifdef USE_UART
  // UART Configuration
  bno1.bus = &uart2;
  bno1.busType = BNO08X_UART;

  if(serial_open((serial_handle_t*)bno1.bus, 3000000, UART_TYPE_BASIC)) // UART must be 3Mbaud (3000000)
  {
    Serial.println("Failed to open Serial connection")
    while (1)
      ;
  }

  #endif
  
  // Shared Pin Configuration
  bno1.wakePin = BNO_PIN_WAKE_1;
  bno1.pinInt = BNO_PIN_INT_1;
  bno1.pinRst = BNO_PIN_RST_1;

  bno2.wakePin = BNO_PIN_WAKE_2;
  bno2.pinInt = BNO_PIN_INT_2;
  bno2.pinRst = BNO_PIN_RST_2;
  
  // Initialize GPIO not handled in BNO handler
  gpio_mode(LED, OUTPUT);
  gpio_write(LED, HIGH);

  timer_init(&loop_timer, print_interval); // Initialize timer
  timer_init(&poll_timer, poll_interval); // Initialize timer

  Serial.begin(115200); // 115200 baud
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital BNO08x Multiple Example ");
  Serial.println("***************************************");

  // Initialize BNO chips
  uint8_t init1 = 1;

  while (init1)
  {
    init1 = bno08x_init(&bno1);
    switch(init1)
    {
      case 0:
        Serial.println("BNO1 Initialized Successfully");
        break;
      case 1:
        Serial.println("Failed waiting for interrupt");
        break;
      case 2:
        Serial.println("Failed to Find I2C Device");
        delay(1000);
        bno08x_hw_reset(&bno1);
        break;
      case 3:
        Serial.println("Failed to Receive Startup Advertisement");
        delay(1000);
        bno08x_hw_reset(&bno1);
        break;
      case 4:
        Serial.println("Failed to Receive Hub Startup Message");
        delay(1000);
        bno08x_hw_reset(&bno1);
        break;
      case 5:
        Serial.println("Failed to Get Product ID");
        delay(1000);
        bno08x_hw_reset(&bno1);
        break;
      case 6:
        Serial.println("Failed to Receive Reset Complete");
        delay(1000);
        bno08x_hw_reset(&bno1);
        break;
      default:
        break;
    }
  }

  uint8_t init2 = 1;

  while (init2)
  {
    init2 = bno08x_init(&bno2);
    switch(init2)
    {
      case 0:
        Serial.println("BNO2 Initialized Successfully");
        break;
      case 2:
        Serial.println("Failed to Find I2C Device");
        delay(1000);
        bno08x_hw_reset(&bno2);
        break;
      case 3:
        Serial.println("Failed to Receive Startup Advertisement");
        delay(1000);
        bno08x_hw_reset(&bno2);
        break;
      case 4:
        Serial.println("Failed to Receive Hub Startup Message");
        delay(1000);
        bno08x_hw_reset(&bno2);
        break;
      case 5:
        Serial.println("Failed to Get Product ID");
        delay(1000);
        bno08x_hw_reset(&bno2);
        break;
      case 6:
        Serial.println("Failed to Receive Reset Complete");
        delay(1000);
        bno08x_hw_reset(&bno2);
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
  while(bno08x_feature_set(&bno1, report, 0, sensitivity, interval, 0, configWord))
    Serial.println("BNO1 Failed to set feature report");

  while(bno08x_feature_set(&bno2, report, 0, sensitivity, interval, 0, configWord))
    Serial.println("BNO2 Failed to set feature report");
  
  Serial.println("BNO Begin MECal Config");

  // Enable/disable calibration routines
  if (bno08x_mecal_config(&bno1, 1, 1, 1, 1, 0))
    Serial.println("BNO1 Cal failed");

  if (bno08x_mecal_config(&bno2, 1, 1, 1, 1, 0))
    Serial.println("BNO2 Cal failed");

  Serial.println("BNO Finished MECal Config");

  timer_start(&loop_timer); // Start loop timer

  #ifdef USE_POLL
  timer_start(&poll_timer); // Start poll timer
  #endif
  
  #ifdef USE_INT
  // Generate interrupt handler
  bno_int1 = interrupt_init(BNO_PIN_INT_1, GPIO_LOW, bno_ISR1); // Interrupt is active low
    if (bno_int1 == NULL)
      Serial.println("BNO1 Failed to init interrupt");
  
  bno_int2 = interrupt_init(BNO_PIN_INT_2, GPIO_LOW, bno_ISR2); // Interrupt is active low
    if (bno_int2 == NULL)
      Serial.println("BNO2 Failed to init interrupt");
  
  // Enable interrupt
  interrupt_set(bno_int1);
  interrupt_set(bno_int2);
  #endif
  
  Serial.println("Begin loop()");

}

void loop() {
  
  // Read messages if received interrupt or poll timer expires
  #ifdef USE_INT
  if (bno_msg1 == 1){
  #endif

  #ifdef USE_POLL
  if (timer_check_exp(&poll_timer)){
  #endif


    bno_msg1 = 0;

    // Read, parse, and store messages on the handler
    bno08x_get_messages(&bno1);

    // Process report data and add to struct
    get_data(&bno1, &bno_data1);

    #ifdef USE_POLL
    timer_reset(&poll_timer);
    #endif

    #ifdef USE_INT
    // Re-enable after reading the message
    interrupt_enable(bno_int1);
    #endif
  }

  #ifdef USE_INT
  if (bno_msg2 == 1){
  #endif

  #ifdef USE_POLL
  if (timer_check_exp(&poll_timer)){
  #endif

    bno_msg2 = 0;

    // Read, parse, and store messages on the handler
    bno08x_get_messages(&bno2);

    // Process report data and add to struct
    get_data(&bno2, &bno_data2);

    #ifdef USE_POLL
    timer_reset(&poll_timer);
    #endif

    #ifdef USE_INT
    // Re-enable after reading the message
    interrupt_enable(bno_int2);
    #endif
  }
  
  if (timer_check_exp(&loop_timer) != 0){
    #ifdef USE_INT
    interrupt_disable(bno_int1);
    interrupt_disable(bno_int2);
    #endif
    
    timer_reset(&loop_timer);

    Serial.print("BNO1 - Accuracy: ");

    switch (bno_data1.status){
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
    Serial.print("q0: "); Serial.print(bno_data1.Q0 + 0.0005f,4); Serial.print(", ");
    Serial.print("q1: "); Serial.print (bno_data1.Q1 + 0.0005f,4); Serial.print(", ");
    Serial.print("q2: "); Serial.print (bno_data1.Q2 + 0.0005f,4); Serial.print(", ");
    Serial.print("q3: "); Serial.print (bno_data1.Q3 + 0.0005f,4); Serial.println();

    Serial.print("BNO2 - Accuracy: ");

    switch (bno_data2.status){
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
    Serial.print("q0: "); Serial.print(bno_data2.Q0 + 0.0005f,4); Serial.print(", ");
    Serial.print("q1: "); Serial.print (bno_data2.Q1 + 0.0005f,4); Serial.print(", ");
    Serial.print("q2: "); Serial.print (bno_data2.Q2 + 0.0005f,4); Serial.print(", ");
    Serial.print("q3: "); Serial.print (bno_data2.Q3 + 0.0005f,4); Serial.println();
    
    #ifdef USE_INT
    interrupt_enable(bno_int1);
    interrupt_enable(bno_int2);
    #endif
 }

  if (bno_data1.status == 3)  gpio_toggle(LED);
  else gpio_write(LED, LOW);


}

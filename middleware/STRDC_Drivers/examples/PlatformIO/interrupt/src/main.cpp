/*
 * Interrupt Example C++ File.
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

#include "gpio.h"
#include "timer.h"
#include "interrupt.h"

/*************************************
 * DEFINITIONS
 *************************************/

#define LED_PIN 13

#define INT_PIN 8

isr_handle_t* example_int; // Create interrupt handler

volatile bool int_set = 0;

// ISR to notify program to read messages
void ex_ISR() {
  int_set = true;

  // Disable interrupt
  interrupt_disable(example_int);
}

void setup()
{
    
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital Interrupt Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  example_int = interrupt_init(LED_PIN, GPIO_HIGH, ex_ISR); // Initiate interrupt with polarity and ISR

  if (example_int == NULL) // Confirm init worked properly
    Serial.println("Failed to init interrupt");
  
  // Enable interrupt
  interrupt_set(example_int);

  gpio_mode(LED_PIN, GPIO_MODE_OUTPUT);

  gpio_write(LED_PIN, GPIO_LOW);

}

void loop() {

  if (int_set)
  {
    Serial.println("Interrupt triggered!");
    timer_blocking_delay(200000); // Set blocking delay for 200ms
    gpio_write(LED_PIN, GPIO_LOW);
    interrupt_enable(example_int); // Re-enable interrupt
  }

  timer_blocking_delay(1000000); // Set blocking delay for 1000ms

  gpio_write(LED_PIN, GPIO_HIGH);
    

}

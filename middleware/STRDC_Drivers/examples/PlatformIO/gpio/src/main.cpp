/*
 * GPIO Example C++ File.
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

/*************************************
 * DEFINITIONS
 *************************************/

#define LED_PIN 13
#define READ_PIN 7

volatile uint8_t output = GPIO_HIGH;

timer_handle_t led_timer; // Create timer handler

void setup()
{
    
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital Drivers GPIO Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  gpio_mode(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_mode(READ_PIN, GPIO_MODE_INPUT_PULLUP);

  gpio_write(LED_PIN, output);

  // Use non-blocking timer so we can instantly react to changes on READ_PIN
  timer_init(&led_timer, 1000000); // Initiate timer with 1000ms expiration

  timer_start(&led_timer); // Begin timer 

}

void loop() {

  if (timer_check_exp(&led_timer)) // Check if timer has expired
  {
    if (output == GPIO_HIGH)
      output = GPIO_LOW;
    else
      output = GPIO_HIGH;
    
    timer_reset(&led_timer); // Reset timer and expiration
  }

  gpio_write(LED_PIN, output);
  
  if (gpio_read(READ_PIN) == GPIO_LOW)
    Serial.println("READ_PIN is low");
  else
    Serial.println("READ_PIN is high");

}

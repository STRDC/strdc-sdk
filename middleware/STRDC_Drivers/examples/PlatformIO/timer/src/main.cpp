/*
 * Timer Example C++ File.
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

#include "timer.h"

/*************************************
 * DEFINITIONS
 *************************************/

timer_handle_t gen_timer; // Create timer handler

void setup()
{
    
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("***************************************");
  Serial.println(" Stardust Orbital Drivers Timer Example ");
  Serial.println("***************************************");

  /*************************************
  * Initialization
  *************************************/

  timer_init(&gen_timer, 1000000); // Initiate timer with 1000ms expiration

  timer_blocking_delay(1500000); // Pause program for 1500ms

  /*************************************
  * Start Timer
  *************************************/

  timer_start(&gen_timer); // Begin timer 

}

void loop() {

  if (timer_check_exp(&gen_timer)) // Check if timer has expired
  {
    Serial.println("Timer has expired! Resetting timer...");
    timer_reset(&gen_timer); // Reset timer and expiration
  } 

}

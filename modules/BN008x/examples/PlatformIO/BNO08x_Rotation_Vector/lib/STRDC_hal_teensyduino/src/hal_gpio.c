/*
 * Teensyduino GPIO HAL C File.
 *
 * @file        hal_gpio.c
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

#include "hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * @brief Configure mode of GPIO pin.
 * @param pin Pin number as listed in the Teensy Information.
 * @param mode Mode to configure GPIO.
 ****************************************************************************/
void hal_gpio_mode(uint8_t pin, uint8_t mode)
{
    pinMode(pin, mode);
}

/****************************************************************************
 * @brief Write value to output digital pin.
 * @param pin Pin number as listed in the Teensy Information.
 * @param value Value of output (GPIO_LOW or GPIO_HIGH).
 ****************************************************************************/
void hal_gpio_write(uint8_t pin, uint8_t value)
{
    digitalWrite(pin, value);
}

/****************************************************************************
 * @brief Read value of input digital pin.
 * @param pin Pin number as listed in the Teensy Information.
 * @return Value of input (GPIO_LOW or GPIO_HIGH).
 ****************************************************************************/
uint8_t hal_gpio_read(uint8_t pin)
{
    return digitalRead(pin);
}

/****************************************************************************
 * @brief Toggle value of output digital pin. Sets the pin high when low, low when high.
 * @param pin Pin number as listed in the Teensy Information.
 ****************************************************************************/
void hal_gpio_toggle(uint8_t pin)
{
    digitalToggle(pin);
}


#ifdef __cplusplus
}
#endif
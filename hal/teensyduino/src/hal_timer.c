/*
 * Teensyduino Timer HAL C File.
 *
 * @file        hal_timer.c
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

#include "hal_timer.h"

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************
 * @brief Configure and initialize a timer handler.
 * @param newTimer Pointer to timer handler.
 * @param period Timer duration in microseconds.
 ****************************************************************************/
void hal_timer_init(timer_handle_t *newTimer, uint32_t period)
{

    newTimer->isExp = 0;
    newTimer->set = period; // Time in microseconds

}

/****************************************************************************
 * @brief Set timer expiration from duration and start timer.
 * @param newTimer Pointer to timer handler.
 ****************************************************************************/
void hal_timer_start(timer_handle_t *timer)
{
    timer->start = micros();
    timer->curr = timer->start;
    timer->exp = timer->start + timer->set;

    timer->lapsed = 0;

    if (timer->exp < timer->start) // Check for rollover
        timer->expLapsed = 1;
    else
        timer->expLapsed = 0;
}

/****************************************************************************
 * @brief Reset timer expiration from duration and restart timer.
 * @param newTimer Pointer to timer handler.
 ****************************************************************************/
void hal_timer_reset(timer_handle_t *timer)
{

    hal_timer_start(timer);
    timer->isExp = 0;

}

/****************************************************************************
 * @brief Check if timer has expired.
 * @param newTimer Pointer to timer handler.
 * @return 0: Timer has not expired
 *  1: Timer has expired
 ****************************************************************************/
uint8_t hal_timer_check_exp(timer_handle_t *timer)
{

    uint32_t split = micros();

    if (split < timer->curr) // Check if rollover occurred
        timer->lapsed++;

    timer->curr = split;

    if ((timer->curr > timer->exp) && (timer->lapsed >= timer->expLapsed))
        timer->isExp = 1;
    
    return timer->isExp;

}

/****************************************************************************
 * @brief Pause program for requested duration.
 * @param period Duration of time to pause program in microseconds.
 ****************************************************************************/
void hal_timer_blocking_delay(uint32_t period)
{

    /* Note:
     * delay() uses an interrupt for counting
     * If interrupts are disabled, delay() will sit forever (because the count never increments)
     * Disabling individual interrupts is preferable or using a non-ISR delay function
     */
    
    delayMicroseconds(period); // in us

}

#ifdef __cplusplus
}
#endif
/*
 * Teensyduino Interrupt HAL H File.
 *
 * @file        hal_interrupt.h
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

 /* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2018 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef HAL_INTERRUPT_H
#define HAL_INTERRUPT_H

#include <Arduino.h>


#ifdef __cplusplus
extern "C" {
#endif

/* From Teensyduino

CHANGE:  0
RISING:  2
FALLING: 3
LOW:     0
HIGH:    1

*/

// Interrupt Definitions
#define HAL_IRQ_PIT IRQ_PIT
#define HAL_IRQ_GPT1 IRQ_GPT1
#define HAL_IRQ_GPT2 IRQ_GPT1
#define HAL_IRQ_QTIMER1 IRQ_QTIMER1
#define HAL_IRQ_GPIO6_9 IRQ_GPIO6789

typedef struct {

    uint8_t pin;
    volatile uint32_t * gpio;
    uint32_t mask;
    uint32_t irq;
    uint8_t mode;
    void (*callback)(void);

} isr_handle_t;

void hal_interrupt_init(isr_handle_t *);
void hal_interrupt_disable(isr_handle_t *);
void hal_interrupt_enable(isr_handle_t *);
void hal_interrupt_disable_all(void);
void hal_interrupt_enable_all(void);
void hal_interrupt_set(isr_handle_t *);
void hal_interrupt_priority(isr_handle_t *, uint8_t);
void hal_interrupt_clear(isr_handle_t *);

// Existing Interrupts
extern isr_handle_t interrupt_periodic_timer;
extern isr_handle_t interrupt_GPT1;
extern isr_handle_t interrupt_GPT2;
extern isr_handle_t interrupt_QTimer;

#ifdef __cplusplus
}
#endif

#endif
/*
 * Teensyduino Interrupt HAL C File.
 *
 * @file        hal_interrupt.c
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

#include "hal_interrupt.h"

// Define handlers
isr_handle_t interrupt_periodic_timer = {.gpio = NULL, .irq = HAL_IRQ_PIT};
isr_handle_t interrupt_GPT1 = {.gpio = NULL, .irq = HAL_IRQ_GPT1};
isr_handle_t interrupt_GPT2 = {.gpio = NULL, .irq = HAL_IRQ_GPT2};
isr_handle_t interrupt_QTimer = {.gpio = NULL, .irq = HAL_IRQ_QTIMER1};

/****************************************************************************
 * @brief Initialize interrupt handler for a GPIO interrupt. Grabs registers for enable/disable.
 * @param handle Pointer to interrupt handler. Handler should have a defined pin.
 ****************************************************************************/
void hal_interrupt_init(isr_handle_t * handle)
{
    handle->gpio = portOutputRegister(digitalPinToInterrupt(handle->pin)); // GPIO Register
    handle->mask = digitalPinToBitMask(digitalPinToInterrupt(handle->pin)); // Bit Mask
    handle->irq = HAL_IRQ_GPIO6_9;
}

/****************************************************************************
 * @brief Disable an initialized and previously set interrupt.
 * @param handle Pointer to interrupt handler. Handler should be initialized and previously set.
 ****************************************************************************/
void hal_interrupt_disable(isr_handle_t * handle)
{
    if (handle->gpio != NULL)
    {
        handle->gpio[5] &= ~handle->mask;	// disable interrupt
    }
    else
    {
        NVIC_DISABLE_IRQ(handle->irq); // Disable IRQ, don't use with GPIO because then you disable interrupts on all pins
    }
}

/****************************************************************************
 * @brief Enable an initialized and previously set interrupt.
 * @param handle Pointer to interrupt handler. Handler should be initialized and previously set.
 ****************************************************************************/
void hal_interrupt_enable(isr_handle_t * handle)
{
    if (handle->gpio != NULL)
    {
        handle->gpio[6] = handle->mask;  // clear any prior pending interrupt
	    handle->gpio[5] |= handle->mask; // enable interrupt
    }
    else
    {
        NVIC_ENABLE_IRQ(handle->irq); // Enable IRQ
    }
}

/****************************************************************************
 * @brief Blocks all regular interrupts, leaving only NMI and HardFault.
 ****************************************************************************/
void hal_interrupt_disable_all(void)
{
    noInterrupts();
}

/****************************************************************************
 * @brief Allows all regular interrupts to function again.
 ****************************************************************************/
void hal_interrupt_enable_all(void)
{
    interrupts();
}

/****************************************************************************
 * @brief Register and enable a previously initialized interrupt.
 * @param handle Pointer to interrupt handler. Handler should be initialized.
 ****************************************************************************/
void hal_interrupt_set(isr_handle_t * handle)
{
    attachInterrupt(digitalPinToInterrupt(handle->pin), handle->callback, handle->mode);
}

/****************************************************************************
 * @brief Update the priority of a previously initialized interrupt.
 * @param handle Pointer to interrupt handler. Handler should be initialized.
 * @param priority Interrupt priority 0-255.
 ****************************************************************************/
void hal_interrupt_priority(isr_handle_t * handle, uint8_t priority)
{
    NVIC_SET_PRIORITY(handle->irq, priority);
}

/****************************************************************************
 * @brief Disable and free interrupt. Calling this function will render the interrupt handler useless.
 * @param handle Pointer to interrupt handler. Handler should be initialized and previously set.
 ****************************************************************************/
void hal_interrupt_clear(isr_handle_t * handle)
{
    detachInterrupt(digitalPinToInterrupt(handle->pin));
}

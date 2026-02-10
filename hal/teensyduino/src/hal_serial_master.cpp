/*
 * Teensyduino Serial Master HAL C++ File.
 *
 * @file        hal_serial_master.cpp
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

#include "hal_serial_master.h"

// Define Handlers
serial_handle_t uart1 = {.bus = &Serial1, .format = 0, .rxPin = 255, .txPin = 255}; // Initialize to arbitrary value for change by user
serial_handle_t uart2 = {.bus = &Serial2, .format = 0, .rxPin = 255, .txPin = 255};
serial_handle_t uart3 = {.bus = &Serial3, .format = 0, .rxPin = 255, .txPin = 255};
serial_handle_t uart4 = {.bus = &Serial4, .format = 0, .rxPin = 255, .txPin = 255};
serial_handle_t uart5 = {.bus = &Serial5, .format = 0, .rxPin = 255, .txPin = 255};
serial_handle_t uart6 = {.bus = &Serial6, .format = 0, .rxPin = 255, .txPin = 255};
serial_handle_t uart7 = {.bus = &Serial7, .format = 0, .rxPin = 255, .txPin = 255};

/****************************************************************************
 * @brief Initialize and open a serial (UART) peripheral.
 * @param handle Pointer to serial handler. Handler should have configuration defined before calling this function.
 * @param speed Speed of operation of peripheral
 * @return 0: Success
 *  1: Timed out
 ****************************************************************************/
uint8_t hal_serial_open(serial_handle_t *handle, uint32_t speed, uint8_t busType)
{
    handle->type = busType;

    if (handle->rxPin != 255) // Check if using alternate pin
        handle->bus->setRX(handle->rxPin);
    if (handle->txPin != 255) // Check if using alternate pin
        handle->bus->setTX(handle->txPin);

    if (handle->type != UART_TYPE_BASIC)
    {
        if (handle->type == UART_TYPE_RS485)
            handle->bus->transmitterEnable(handle->rs4Pin);
        else
        {
            if ((handle->type & 0x01) == UART_TYPE_FLOW_RTS)
                handle->bus->attachRts(handle->rtsPin);
            if ((handle->type & 0x02) == UART_TYPE_FLOW_CTS)
                handle->bus->attachCts(handle->ctsPin);
        }
    }

    handle->bus->begin(speed, handle->format);

    if (!handle->bus)
        delay(250); // Give delay and try again

    if (!handle->bus)
        return 1; // Timeout

    return 0;

}

/****************************************************************************
 * @brief Read serial data from peripheral. Will increment buffer.
 * @param handle Pointer to serial handler.
 * @return Output data byte.
 ****************************************************************************/
uint8_t hal_serial_read(serial_handle_t *handle)
{
    return handle->bus->read();
}

/****************************************************************************
 * @brief Write serial data on peripheral.
 * @param handle Pointer to serial handler.
 * @param data Data byte to send.
 ****************************************************************************/
void hal_serial_write(serial_handle_t *handle, uint8_t data)
{
    handle->bus->write(data);
}

/****************************************************************************
 * @brief Check if serial peripheral has data to be read in buffer.
 * @param handle Pointer to serial handler.
 * @return Number of bytes available for read.
 ****************************************************************************/
uint16_t hal_serial_read_available(serial_handle_t *handle)
{
    return handle->bus->available();
}

/****************************************************************************
 * @brief Read next byte in buffer without removing it.
 * @param handle Pointer to serial handler.
 * @return Next data byte in buffer for read. Returns -1 if no data.
 ****************************************************************************/
uint8_t hal_serial_peek(serial_handle_t *handle)
{
    return handle->bus->peek();
}

/****************************************************************************
 * @brief Add additional memory to serial read buffer
 * @param buffer Pointer to array of bytes for additional buffer location.
 * @param bytes Length of additional buffer array (buffer).
 ****************************************************************************/
void hal_serial_buffer_read_add(serial_handle_t *handle, uint8_t *buffer, size_t length)
{
    
    handle->bus->addMemoryForRead(buffer, length);

}

/****************************************************************************
 * @brief Add additional memory to serial write buffer
 * @param buffer Pointer to array of bytes for additional buffer location.
 * @param bytes Length of additional buffer array (buffer).
 ****************************************************************************/
void hal_serial_buffer_write_add(serial_handle_t *handle, uint8_t *buffer, size_t length)
{
    
    handle->bus->addMemoryForWrite(buffer, length);

}

/****************************************************************************
 * @brief Clear buffer, losing all unread data.
 * @param handle Pointer to serial handler.
 ****************************************************************************/
void hal_serial_clear(serial_handle_t *handle)
{
    handle->bus->clear();
}

/****************************************************************************
 * @brief Check number of bytes available for immediate write in buffer.
 * @param handle Pointer to serial handler.
 * @return Number of bytes that may be written to the Serial buffer for immediate transmission.
 ****************************************************************************/
size_t hal_serial_write_available(serial_handle_t *handle)
{
    return handle->bus->availableForWrite();
}

/****************************************************************************
 * @brief Close serial peripheral.
 * @param handle Pointer to serial handler.
 ****************************************************************************/
void hal_serial_close(serial_handle_t *handle)
{
    handle->bus->end();
}

/****************************************************************************
 * @brief Wait for all pending write data to send.
 * @param handle Pointer to serial handler.
 ****************************************************************************/
void hal_serial_flush(serial_handle_t *handle)
{
    handle->bus->flush();
}
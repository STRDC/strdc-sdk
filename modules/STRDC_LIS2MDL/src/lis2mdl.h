/*
 * LIS2MDL Module H File.
 *
 * @file        lis2mdl.h
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

#ifndef LIS2MDL_H
#define LIS2MDL_H

#include "i2c_master.h"
#include "spi_master.h"
#include "timer.h"
#include "gpio.h"

// Bus Types
#define LIS2MDL_I2C 0
#define LIS2MDL_SPI_4_WIRE 1
#define LIS2MDL_SPI_3_WIRE 2

// Output Data Rate Configuration
#define LIS2MDL_10HZ 0
#define LIS2MDL_20HZ 1
#define LIS2MDL_50HZ 2
#define LIS2MDL_100HZ 3

// Mode of Operation
#define LIS2MDL_MODE_CONTINUOUS 0
#define LIS2MDL_MODE_SINGLE 1
#define LIS2MDL_MODE_IDLE 3

// Status Register Mask
#define LIS2MDL_STATUS_NEW_X 0x01
#define LIS2MDL_STATUS_NEW_Y 0x02
#define LIS2MDL_STATUS_NEW_Z 0x04
#define LIS2MDL_STATUS_NEW_ANY 0x08
#define LIS2MDL_STATUS_OVERRUN_X 0x10
#define LIS2MDL_STATUS_OVERRUN_Y 0x20
#define LIS2MDL_STATUS_OVERRUN_Z 0x40
#define LIS2MDL_STATUS_OVERRUN_ANY 0x80

// Interrupt Source Register Mask
#define LIS2MDL_INT_SOURCE_INT 0x01
#define LIS2MDL_INT_SOURCE_MROI 0x02
#define LIS2MDL_INT_SOURCE_Z_N 0x04 // Exceeded Negative threshold
#define LIS2MDL_INT_SOURCE_Y_N 0x08
#define LIS2MDL_INT_SOURCE_X_N 0x10
#define LIS2MDL_INT_SOURCE_Z_P 0x20 // Exceeded Positive threshold
#define LIS2MDL_INT_SOURCE_Y_P 0x40
#define LIS2MDL_INT_SOURCE_X_P 0x80

// Register Mapping
#define LIS2MDL_OFFSET_X_REG_L          0x45
#define LIS2MDL_OFFSET_X_REG_H          0x46
#define LIS2MDL_OFFSET_Y_REG_L          0x47
#define LIS2MDL_OFFSET_Y_REG_H          0x48
#define LIS2MDL_OFFSET_Z_REG_L          0x49
#define LIS2MDL_OFFSET_Z_REG_H          0x4A

#define LIS2MDL_CFG_REG_A               0x60
#define LIS2MDL_CFG_REG_B               0x61
#define LIS2MDL_CFG_REG_C               0x62

#define LIS2MDL_INT_CTRL_REG            0x63

#define LIS2MDL_INT_THS_L_REG           0x65
#define LIS2MDL_INT_THS_H_REG           0x66

// Read Only
#define LIS2MDL_WHO_AM_I                0x4F

#define LIS2MDL_INT_SOURCE_REG          0x64

#define LIS2MDL_STATUS_REG              0x67

#define LIS2MDL_OUTX_L_REG              0x68
#define LIS2MDL_OUTX_H_REG              0x69
#define LIS2MDL_OUTY_L_REG              0x6A
#define LIS2MDL_OUTY_H_REG              0x6B
#define LIS2MDL_OUTZ_L_REG              0x6C
#define LIS2MDL_OUTZ_H_REG              0x6D

#define LIS2MDL_TEMP_OUT_L_REG          0x6E
#define LIS2MDL_TEMP_OUT_H_REG          0x6F

typedef struct {

    void *bus; // Pointer to communication peripheral bus
    uint8_t busType; // Bus type (e.g. LIS2MDL_I2C, LIS2MDL_UART)
    uint8_t busAddr; // Address for I2C, SS for SPI, Not used for UART

} lis2mdl_t; // Handler for LIS2MDL

// Functions

uint8_t lis2mdl_init(lis2mdl_t *, uint32_t);
uint8_t lis2mdl_sw_reset(lis2mdl_t *);
uint8_t lis2mdl_reboot_memory(lis2mdl_t *);
uint8_t lis2mdl_set_rate(lis2mdl_t *, uint8_t);
uint8_t lis2mdl_set_low_power(lis2mdl_t *, bool);
uint8_t lis2mdl_set_op_mode(lis2mdl_t *, uint8_t);
uint8_t lis2mdl_set_low_pass(lis2mdl_t *, bool);
uint8_t lis2mdl_get_status(lis2mdl_t *, uint8_t *);
uint8_t lis2mdl_get_int_source(lis2mdl_t *, uint8_t *);
uint8_t lis2mdl_set_int_cfg(lis2mdl_t *, bool, bool, bool, bool, bool, bool);
uint8_t lis2mdl_enable_int(lis2mdl_t *, bool, bool);
uint8_t lis2mdl_set_threshold(lis2mdl_t *, float);
uint8_t lis2mdl_write(lis2mdl_t *, uint8_t, uint8_t *, uint8_t);
uint8_t lis2mdl_read(lis2mdl_t *, uint8_t, uint8_t *, uint8_t);
uint8_t lis2mdl_set_offsets(lis2mdl_t *, float, float, float);
uint8_t lis2mdl_get_mags(lis2mdl_t *, float *, float *, float *);
uint8_t lis2mdl_get_internal_temp(lis2mdl_t *, float *);

#endif
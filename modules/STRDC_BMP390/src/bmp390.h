/*
 * BMP390 Module H File.
 *
 * @file        bmp390.h
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

#ifndef BMP390_H
#define BMP390_H

#include "i2c_master.h"
#include "spi_master.h"
#include "timer.h"
#include "gpio.h"

#define FIFO_BUFFER_SIZE 1028 // 2 Max buffers (512) + 4 bytes for sensor time frame

#define I2C_MAX_BUFFER_SIZE 32 // 32 byte is most common but depends on MCU and HAL setup

// Bus Types
#define BMP390_I2C 0
#define BMP390_SPI_4_WIRE 1
#define BMP390_SPI_3_WIRE 2

// Power Modes via PWR_CTRL
#define BMP390_MODE_SLEEP 0
#define BMP390_MODE_FORCED 1
#define BMP390_MODE_FORCED2 2 // No different than the original, just a different number in the register per the datasheet
#define BMP390_MODE_NORMAL 3

// Error Conditions via ERR_REG
#define BMP390_ERR_FATAL 1
#define BMP390_ERR_CMD 2 // Command execution failed. Cleared on read
#define BMP390_ERR_CONFIG 4 // Sensor configuration error detected (only working in normal mode). Cleared on read

// Status Flags via STATUS
#define BMP390_STATUS_CMD_RDY 0x10 // CMD decoder status - 0: Command in Progress, 1: Ready for new command (this status will be | 0x10 when Ready for command)
#define BMP390_STATUS_DRDY_PRESS 0x20 // Data ready for pressure, reset when pressure is read from DATA register
#define BMP390_STATUS_DRDY_TEMP 0x40 // Data ready for temperature, reset when temperature is read from DATA register

// Events via EVENT
#define BMP390_EVENT_POR 1 // Power up or softreset has occured. Cleared on read
#define BMP390_EVENT_ITF 2 // Serial interface transaction occurred during a pressure or temperature conversion. Cleared on read

// Interrupt Status via INT_STATUS
#define BMP390_INT_FIFO_WATERMARK 1 // FIFO Watermark Interrupt
#define BMP390_INT_FIFO_FULL 2 // FIFO Full Interrupt
#define BMP390_INT_DRDY 8 // Data Ready

// I2C Watchdog Timer Period
#define BMP390_WATCHDOG_SHORT 0
#define BMP390_WATCHDOG_LONG 1

// FIFO Data Select/Source Parameters
#define BMP390_FIFO_SOURCE_UNFILTERED 0
#define BMP390_FIFO_SOURCE_FILTERED 23

// IIR Filter Coefficient Settings
#define BMP390_IIR_COEFF_0 0
#define BMP390_IIR_COEFF_1 1
#define BMP390_IIR_COEFF_3 2
#define BMP390_IIR_COEFF_7 3
#define BMP390_IIR_COEFF_15 4
#define BMP390_IIR_COEFF_31 5
#define BMP390_IIR_COEFF_63 6
#define BMP390_IIR_COEFF_127 7

// ODR Subdivision Settings
#define BMP390_ODR_200      0x00 // 200Hz, prescaler 1, 5ms Sampling
#define BMP390_ODR_100      0x01 // 100Hz, prescaler 2, 10ms Sampling
#define BMP390_ODR_50       0x02 // 50Hz, prescaler 4, 20ms Sampling
#define BMP390_ODR_25       0x03 // 25Hz, prescaler 8, 40ms Sampling
#define BMP390_ODR_12p5     0x04 // 12.5Hz (25/2), prescaler 16, 80ms Sampling
#define BMP390_ODR_6p25     0x05 // 6.25Hz (25/4), prescaler 32, 160ms Sampling
#define BMP390_ODR_3p1      0x06 // 3.1Hz (25/8), prescaler 64, 320ms Sampling
#define BMP390_ODR_1p5      0x07 // 1.5Hz (25/16), prescaler 127, 640ms Sampling
#define BMP390_ODR_0p78     0x08 // 0.78Hz (25/32), prescaler 256, 1.280s Sampling
#define BMP390_ODR_0p39     0x09 // 0.39Hz (25/64), prescaler 512, 2.560s Sampling
#define BMP390_ODR_0p2      0x0A // 0.2Hz (25/128), prescaler 1024, 5.120s Sampling
#define BMP390_ODR_0p1      0x0B // 0.1Hz (25/256), prescaler 2048, 10.24s Sampling
#define BMP390_ODR_0p05     0x0C // 0.05Hz (25/512), prescaler 4096, 20.48s Sampling
#define BMP390_ODR_0p02     0x0D // 0.02Hz (25/1024), prescaler 8192, 40.96s Sampling
#define BMP390_ODR_0p01     0x0E // 0.01Hz (25/2048), prescaler 16384, 81.92s Sampling
#define BMP390_ODR_0p006    0x0F // 0.006Hz (25/4096), prescaler 32768, 163.84s Sampling
#define BMP390_ODR_0p003    0x10 // 0.003Hz (25/8192), prescaler 65536, 327.68s Sampling
#define BMP390_ODR_0p0015   0x11 // 0.0015Hz (25/16384), prescaler 131072, 655.36s Sampling

// Oversampling Settings
#define BMP390_OSR_NO       0x00
#define BMP390_OSR_2        0x01
#define BMP390_OSR_4        0x02
#define BMP390_OSR_8        0x03
#define BMP390_OSR_16       0x04
#define BMP390_OSR_32       0x05

// Register Mapping

// Read Only
#define BMP390_REG_CHIP_ID          0x00
#define BMP390_REG_REV_ID           0x01
#define BMP390_REG_ERR_REG          0x02
#define BMP390_REG_STATUS           0x03
#define BMP390_REG_DATA_0           0x04
#define BMP390_REG_DATA_1           0x05
#define BMP390_REG_DATA_2           0x06
#define BMP390_REG_DATA_3           0x07
#define BMP390_REG_DATA_4           0x08
#define BMP390_REG_DATA_5           0x09
#define BMP390_REG_SENSORTIME_0     0x0C
#define BMP390_REG_SENSORTIME_1     0x0D
#define BMP390_REG_SENSORTIME_2     0x0E
#define BMP390_REG_EVENT            0x10
#define BMP390_REG_INT_STATUS       0x11
#define BMP390_REG_FIFO_LENGTH_0    0x12
#define BMP390_REG_FIFO_LENGTH_1    0x13
#define BMP390_REG_FIFO_DATA        0x14

// Read/Write
#define BMP390_REG_FIFO_WTM_0       0x15
#define BMP390_REG_FIFO_WTM_1       0x16
#define BMP390_REG_FIFO_CONFIG_1    0x17
#define BMP390_REG_FIFO_CONFIG_2    0x18
#define BMP390_REG_INT_CTRL         0x19
#define BMP390_REG_IF_CONF          0x1A
#define BMP390_REG_PWR_CTRL         0x1B
#define BMP390_REG_OSR              0x1C
#define BMP390_REG_ODR              0x1D
#define BMP390_REG_CONFIG           0x1F
#define BMP390_REG_CMD              0x7E

// NVM Trimming Coefficient Registers

// Pressure Coefficients
#define BMP390_NVM_PAR_P11     0x45
#define BMP390_NVM_PAR_P10     0x44
#define BMP390_NVM_PAR_P9_MSB  0x43
#define BMP390_NVM_PAR_P9_LSB  0x42
#define BMP390_NVM_PAR_P8      0x41
#define BMP390_NVM_PAR_P7      0x40
#define BMP390_NVM_PAR_P6_MSB  0x3F
#define BMP390_NVM_PAR_P6_LSB  0x3E
#define BMP390_NVM_PAR_P5_MSB  0x3D
#define BMP390_NVM_PAR_P5_LSB  0x3C
#define BMP390_NVM_PAR_P4      0x3B
#define BMP390_NVM_PAR_P3      0x3A
#define BMP390_NVM_PAR_P2_MSB  0x39
#define BMP390_NVM_PAR_P2_LSB  0x38
#define BMP390_NVM_PAR_P1_MSB  0x37
#define BMP390_NVM_PAR_P1_LSB  0x36

// Temperature Coefficients
#define BMP390_NVM_PAR_T3      0x35
#define BMP390_NVM_PAR_T2_MSB  0x34
#define BMP390_NVM_PAR_T2_LSB  0x33
#define BMP390_NVM_PAR_T1_MSB  0x32
#define BMP390_NVM_PAR_T1_LSB  0x31


#include <stdint.h>

// Struct for NVM Trimming Coefficients
typedef struct {

    // Pressure Coefficients
    double par_p11;
    double par_p10;
    double par_p9;
    double par_p8;
    double par_p7;
    double par_p6;
    double par_p5;
    double par_p4;
    double par_p3;
    double par_p2;
    double par_p1;

    // Temperature Coefficients
    double par_t3;
    double par_t2;
    double par_t1;

    double t_lin; // Temperature carryover for pressure compensation

} trimCoefficients_t;

typedef struct fifoFrame_t {

    uint32_t time;
    uint32_t pressure;
    uint32_t temperature;

    struct fifoFrame_t *next;

} fifoFrame_t;

typedef struct {

    bool en; // Enable FIFO
    bool stopOnFull; // Enable to stop writing to FIFO when full
    bool timeEn; // Enable to return sensortime frame
    bool pressEn; // Enable to store pressure data
    bool tempEn; // Enable to store temperature data
    uint8_t subsampling; // FIFO downsampling for pressure and temperature - 2 ^ subsampling. This is on-top of the general ODR setting
    uint8_t dataSelect; // Data source for pressure and temperature data

    uint16_t watermark; // Up to 512 bytes

} fifoCfg_t;


typedef struct {

    void *bus; // Pointer to communication peripheral bus
    uint8_t busType; // Bus type (e.g. BMP390_I2C, BMP390_UART)
    uint8_t busAddr; // Address for I2C, SS for SPI, Not used for UART

    trimCoefficients_t calib_data; // Trimming coefficients for BMP390

    uint8_t fifoBuffer[FIFO_BUFFER_SIZE];

    fifoFrame_t *fifoData;

    fifoCfg_t *fifoCfg;

} bmp390_t; // Handler for BMP390

// Functions

uint8_t bmp390_init(bmp390_t *);
uint8_t bmp390_soft_reset(bmp390_t *);

uint8_t bmp390_write(bmp390_t *, uint8_t, uint8_t *, uint8_t);
uint8_t bmp390_read(bmp390_t *, uint8_t, uint8_t *, uint16_t);

uint8_t bmp390_get_err_status(bmp390_t *, uint8_t *);
uint8_t bmp390_get_status(bmp390_t *, uint8_t *);
uint8_t bmp390_get_event(bmp390_t *, uint8_t *);
uint8_t bmp390_get_int_status(bmp390_t *, uint8_t *);

uint8_t bmp390_set_iir_filter(bmp390_t *, uint8_t);
uint8_t bmp390_set_odr(bmp390_t *, uint8_t);
uint8_t bmp390_set_osr(bmp390_t *, uint8_t, uint8_t);
uint8_t bmp390_set_power_mode(bmp390_t *, uint8_t);
uint8_t bmp390_set_watchdog(bmp390_t *, bool, bool);
uint8_t bmp390_set_interrupt(bmp390_t *, bool, bool, bool, bool, bool, bool);
uint8_t bmp390_enable_sensors(bmp390_t *, bool, bool);

uint8_t bmp390_get_temperature_pressure(bmp390_t *, double *, double *);
uint8_t bmp390_get_sensor_time(bmp390_t *, uint32_t *);

uint8_t bmp390_set_fifo(bmp390_t *, fifoCfg_t*);
uint8_t bmp390_read_fifo(bmp390_t *);
uint8_t bmp390_retrieve_fifo_value(bmp390_t *, double *, double *, uint32_t *);
uint8_t bmp390_flush_fifo(bmp390_t *);

#endif
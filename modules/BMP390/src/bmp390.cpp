/*
 * BMP390 Module C++ File.
 *
 * @file        bmp390.cpp
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

#include "bmp390.h"

#include <math.h>


//#define DEBUG

#ifdef DEBUG
#include <Arduino.h>
#endif





/****************************************************************************
 * Static Function Declarations
 ****************************************************************************/


static uint8_t bmp390_load_trimmings(bmp390_t *);
static double bmp390_compensate_temperature(bmp390_t *, uint32_t);
static double bmp390_compensate_pressure(bmp390_t *, uint32_t);
static uint8_t bmp390_parse_fifo(bmp390_t *, uint16_t);




/****************************************************************************
 * Executive Functions
 ****************************************************************************/





 /****************************************************************************
 * @brief Initialize blank configuration for BMP390 IC:
 *  Initialize Attributes and Buffer,
 *  Configure associated GPIO,
 *  Initialize comm peripheral (I2C, SPI, UART).
 * @param handle Handle for bmp390 module.
 * @param speed Speed of peripheral (Hz).
 * @return 0: Initialization was successful
 * 1: Failed to Find I2C Device
 * 2: Failed to change configuration to SPI 3 Wire
 * 3: Failed to receive chip identifier as part of initialization
 * 4: Received incorrect chip identifier as part of initialization
 * 5: Failed to enable pressure and temperature measurement
 * 6: Failed to load trimming coefficients as part of initialization
 * 7: Failed to initiate soft reset
 ****************************************************************************/
uint8_t bmp390_init(bmp390_t *handle, uint32_t speed)
{

    memset(handle->fifoBuffer, 0x00, FIFO_BUFFER_SIZE); // Reset FIFO Buffer

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 2000); // Wait 2ms to confirm IC initialization has occurred

    timer_start(&gen_timer);

    while (!timer_check_exp(&gen_timer))
        ;

    if(handle->busType == BMP390_I2C)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Init I2C");
        #endif

        if (speed > 3400000)
            speed = 3400000; // Max speed is 3.4 MHz

        i2c_open((i2c_handle_t*)handle->bus, speed);
        
        if(i2c_find((i2c_handle_t*)handle->bus, handle->busAddr))
        {
            i2c_close((i2c_handle_t*)handle->bus);

            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to init I2C");
            #endif

            return 1; // Didn't receive response from I2C Address
        }
        
        i2c_set_addr((i2c_handle_t*)handle->bus, handle->busAddr);

        if (bmp390_soft_reset(handle))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to initiate soft reset");
            #endif

            return 7;
        }

    }
    else if((handle->busType == BMP390_SPI_4_WIRE) || (handle->busType == BMP390_SPI_3_WIRE))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Init SPI");
        #endif

        if (speed > 10000000)
            speed = 10000000; // Max speed is 10 MHz

        spi_open((spi_handle_t*)handle->bus, speed, SPI_MODE_3, SPI_BIT_ORDER_MSB); // Can operate on both Mode 0 and Mode 3, it switches automatically on CSB assert

        if (bmp390_soft_reset(handle))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to initiate soft reset");
            #endif

            return 7;
        }

        if (handle->busType == BMP390_SPI_3_WIRE)
        {
            uint8_t data = 0x01; // Set SPI 3-wire mode

            handle->busType = BMP390_SPI_4_WIRE; // Change to 4 wire temporarily

            if (bmp390_write(handle, BMP390_REG_IF_CONF, &data, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to switch to SPI 3 Wire on Initialization");
                #endif

                handle->busType = BMP390_SPI_3_WIRE; // Change back

                return 2;
            }

            handle->busType = BMP390_SPI_3_WIRE; // Change back

        }

    }

    uint8_t data;

    // Verify communication and IC by checking the CHIP_ID register
    if (bmp390_read(handle, BMP390_REG_CHIP_ID, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to receive chip identifier as part of initialization");
        #endif

        return 3;
    }

    if (data != 0x60)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Received incorrect chip identifier as part of initialization");
        #endif

        return 4;

    }

    if (bmp390_load_trimmings(handle))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to load trimming coefficients as part of initialization");
        #endif

        return 6;
    }

    data = 0x33;

    // Enable Pressure and Temperature Measurement, normal mode by default
    if (bmp390_write(handle, BMP390_REG_PWR_CTRL, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable pressure and temperature measurement");
        #endif

        return 5;
    }


    return 0;

}

/****************************************************************************
 * @brief Submit softreset command via the CMD register. All configurations will be reset to default.
 * @param handle Handle for bmp390 module.
 * @return 0: Success
 * 1: Failed to write CMD register
 ****************************************************************************/
uint8_t bmp390_soft_reset(bmp390_t *handle)
{

    uint8_t data = 0xB6;

    if (bmp390_write(handle, BMP390_REG_CMD, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CMD register");
        #endif

        return 1;
    }

    return 0;

}





/****************************************************************************
 * Messaging Framework
 ****************************************************************************/





/****************************************************************************
 * @brief Send message to BMP390 module. Abstracts communication peripheral.
 * @param handle Handle for bmp390 module.
 * @param address Register to write to (1 byte).
 * @param data Data to send to bmp390 module (array of bytes).
 * @param bytes Length of data to send (max 255).
 * @return 0: Success
 * 1: Failed to write I2C message
 ****************************************************************************/
uint8_t bmp390_write(bmp390_t *handle, uint8_t address, uint8_t *data, uint8_t bytes)
{

    uint8_t txData[32];

    txData[0] = address;

    memcpy(txData + 1, data, bytes);

    if(handle->busType == BMP390_I2C)
    {

        if (i2c_write((i2c_handle_t*)handle->bus, txData, bytes + 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write I2C message");
            #endif
            return 1;
        }
    }
    if((handle->busType == BMP390_SPI_4_WIRE) || (handle->busType == BMP390_SPI_3_WIRE))
    {
        // SPI write uses 0 for MSb RW bit of address
        spi_write((spi_handle_t*)handle->bus, handle->busAddr, txData, (size_t)bytes + 1);

    }

    #ifdef DEBUG
    Serial.print("[DEBUG] Sent data: ");
    

    for (uint8_t j = 0; j < bytes + 1; j++)
    {
        Serial.print(txData[j]); Serial.print(" ");
    }

    Serial.println();

    #endif
    
    return 0;
}

/****************************************************************************
 * @brief Receive messages from BMP390 module.
 * @param handle Handle for bmp390 module.
 * @param address Register to write to (1 byte).
 * @param data Pointer to array of bytes to store data.
 * @param length Amount of data in bytes to read.
 * @return 0: Success
 * 1: I2C Failed to read register
 ****************************************************************************/
uint8_t bmp390_read(bmp390_t *handle, uint8_t address, uint8_t *data, uint16_t length)
{

    if(handle->busType == BMP390_I2C)
    {
        
        if (i2c_read_reg((i2c_handle_t*)handle->bus, &address, 0x01, data, length))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] I2C Failed to read register");
            #endif
            return 1;
        }

        #ifdef DEBUG
        Serial.print("[DEBUG] Incoming Rx: ");
        for (uint16_t i = 0; i < length; i++)
        {
            Serial.print(data[i]);
            Serial.print(" ");
        }
        Serial.println();
        #endif

    }
    else if((handle->busType == BMP390_SPI_4_WIRE) || (handle->busType == BMP390_SPI_3_WIRE))
    {

        uint8_t addr_data[32]; // Allow for 
        addr_data[0] = address | 0x80; // Set RW bit 1 (MSb)
        memset(addr_data + 1, 0x00, 1);

        uint8_t tempBuff[32]; // Create Temp Buffer

        spi_write_read((spi_handle_t*)handle->bus, handle->busAddr, addr_data, tempBuff, (size_t)length + 1); // Add 1 byte to length for dummy byte

        memcpy(data, tempBuff + 2, length); // Remove initial empty byte and dummy byte

        #ifdef DEBUG
        Serial.print("[DEBUG] Incoming Rx: ");
        for (uint16_t i = 0; i < length; i++)
        {
            Serial.print(data[i]);
            Serial.print(" ");
        }
        Serial.println();
        #endif
        
    }

    return 0;

}




/****************************************************************************
 * Status Messages
 ****************************************************************************/




/****************************************************************************
 * @brief Read sensor error status via ERR_REG register.
 * @param handle Handle for bmp390 module.
 * @param err_status Pointer to store error status. See header file and datasheet for details.
 * @return 0: Success
 * 1: Failed to retrieve reported sensor error conditions
 ****************************************************************************/
uint8_t bmp390_get_err_status(bmp390_t *handle, uint8_t *err_status)
{

    if (bmp390_read(handle, BMP390_REG_ERR_REG, err_status, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to retrieve reported sensor error conditions");
        #endif

        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Read sensor status via STATUS register.
 * @param handle Handle for bmp390 module.
 * @param status Pointer to store status. See header file and datasheet for details.
 * @return 0: Success
 * 1: Failed to retrieve reported sensor status
 ****************************************************************************/
uint8_t bmp390_get_status(bmp390_t *handle, uint8_t *status)
{

    if (bmp390_read(handle, BMP390_REG_STATUS, status, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to retrieve reported sensor status");
        #endif

        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Read event status via EVENT register.
 * @param handle Handle for bmp390 module.
 * @param event Pointer to store event. See header file and datasheet for details.
 * @return 0: Success
 * 1: Failed to retrieve reported event
 ****************************************************************************/
uint8_t bmp390_get_event(bmp390_t *handle, uint8_t *event)
{

    if (bmp390_read(handle, BMP390_REG_EVENT, event, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to retrieve reported event");
        #endif

        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Read interrupt status via INT_STATUS register.
 * @param handle Handle for bmp390 module.
 * @param status Pointer to store interrupt status. See header file and datasheet for details.
 * @return 0: Success
 * 1: Failed to retrieve interrupt status
 ****************************************************************************/
uint8_t bmp390_get_int_status(bmp390_t *handle, uint8_t *status)
{

    if (bmp390_read(handle, BMP390_REG_INT_STATUS, status, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to retrieve interrupt status");
        #endif

        return 1;
    }

    return 0;

}






/****************************************************************************
 * Configuration
 ****************************************************************************/





/****************************************************************************
 * @brief Update IIR Filter coefficient via CONFIG register.
 * @param handle Handle for bmp390 module.
 * @param setting Filter coefficient setting. See header file and datasheet for details.
 * @return 0: Success
 * 1: Failed to write CONFIG register
 ****************************************************************************/
uint8_t bmp390_set_iir_filter(bmp390_t *handle, uint8_t setting)
{

    if (setting > BMP390_IIR_COEFF_127)
        setting = BMP390_IIR_COEFF_127;

    uint8_t data = 0x00 | ((setting & 0x07) << 1);

    if (bmp390_write(handle, BMP390_REG_CONFIG, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CONFIG register");
        #endif

        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Update ODR setting via ODR register.
 * @param handle Handle for bmp390 module.
 * @param odr_sel Subdivision factor pressure and temperature measurements. See header file and datasheet for details.
 * @return 0: Success
 * 1: Failed to check current mode
 * 2: Failed to write ODR register
 * 3: Failed to check OSR settings
 * 4: Error, incorrect ODR/OSR configuration
 ****************************************************************************/
uint8_t bmp390_set_odr(bmp390_t *handle, uint8_t odr_sel)
{

    if (odr_sel > BMP390_ODR_0p0015)
        odr_sel = BMP390_ODR_0p0015;

    uint8_t curr_mode = 0;

    if (bmp390_read(handle, BMP390_REG_PWR_CTRL, &curr_mode, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to check current mode");
        #endif

        return 1;
    }

    curr_mode = (curr_mode & 0x30) >> 4;

    if (curr_mode == BMP390_MODE_NORMAL)
    {
        // If attempting to update settings while in normal mode - we have to confirm the configuration is correct
        uint8_t osr = 0;

        if (bmp390_read(handle, BMP390_REG_OSR, &osr, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to check OSR settings");
            #endif

            return 3;
        }

        uint8_t osr_p = osr & 0x07;
        uint8_t osr_t = (osr >> 3) & 0x07;

        if ((odr_sel < osr_p) || (odr_sel < osr_t))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Error, incorrect ODR/OSR configuration");
            #endif

            return 4;
        }

    }

    uint8_t data = 0x00 | (odr_sel & 0x1F);

    if (bmp390_write(handle, BMP390_REG_ODR, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write ODR register");
        #endif

        return 2;
    }

    return 0;

}

/****************************************************************************
 * @brief Update oversampling setting via OSR register.
 * @param handle Handle for bmp390 module.
 * @param osr_p Oversampling setting for pressure measurement. See header file and datasheet for details.
 * @param osr_t Oversampling setting for temperature measurement. See header file and datasheet for details.
 * @return 0: Success
 * 1: Failed to check current mode
 * 2: Failed to write OSR register
 * 3: Failed to check ODR settings
 * 4: Error, incorrect ODR/OSR configuration
 ****************************************************************************/
uint8_t bmp390_set_osr(bmp390_t *handle, uint8_t osr_p, uint8_t osr_t)
{

    if (osr_p > BMP390_OSR_32)
        osr_p = BMP390_OSR_32;

    if (osr_t > BMP390_OSR_32)
        osr_t = BMP390_OSR_32;

    uint8_t curr_mode = 0;

    if (bmp390_read(handle, BMP390_REG_PWR_CTRL, &curr_mode, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to check current mode");
        #endif

        return 1;
    }

    curr_mode = (curr_mode & 0x30) >> 4;

    if (curr_mode == BMP390_MODE_NORMAL)
    {
        // If attempting to update settings while in normal mode - we have to confirm the configuration is correct
        uint8_t odr = 0;

        if (bmp390_read(handle, BMP390_REG_ODR, &odr, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to check ODR settings");
            #endif

            return 3;
        }

        odr = odr & 0x1F;

        if ((odr < osr_p) || (odr < osr_t))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Error, incorrect ODR/OSR configuration");
            #endif

            return 4;
        }

    }

    uint8_t data = 0x00 | (osr_p & 0x07) | ((osr_t & 0x07) << 3);

    if (bmp390_write(handle, BMP390_REG_OSR, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write OSR register");
        #endif

        return 2;
    }

    return 0;

}

/****************************************************************************
 * @brief Update power mode via the PWR_CTRL register. Will maintain rest of PWR_CTRL configuration.
 * @param handle Handle for bmp390 module.
 * @param mode Power mode. See header file and datasheet for options.
 * @return 0: Success
 * 1: Failed to obtain PWR_CTRL configuration
 * 2: Failed to update power mode configuration to intermediate Sleep Mode
 * 3: Failed to update power mode configuration to new mode
 * 4: Failed to read ODR register for sampling period
 * 5: Timeout occurred waiting for Forced Mode to return to Sleep
 * 6: Failed to read power mode configuration while waiting for Forced Mode
 * 7: Failed to check ODR settings
 * 8: Failed to check OSR settings
 * 9: Error, incorrect ODR/OSR configuration
 ****************************************************************************/
uint8_t bmp390_set_power_mode(bmp390_t *handle, uint8_t mode)
{

    if (mode > BMP390_MODE_NORMAL)
        mode = BMP390_MODE_NORMAL;

    uint8_t data;

    // Get current configuration
    if (bmp390_read(handle, BMP390_REG_PWR_CTRL, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to obtain PWR_CTRL configuration");
        #endif

        return 1;
    }

    uint8_t curr_mode = (data >> 4) & 0x03;

    if (curr_mode == mode) // Nothing to change here
        return 0;

    if ((curr_mode == BMP390_MODE_NORMAL) && (mode == BMP390_MODE_FORCED))
    {
        // Not in Sleep mode and attempting to switch from Normal to Forced

        data = (data & 0x03) | ((BMP390_MODE_SLEEP & 0x03) << 4);

        if (bmp390_write(handle, BMP390_REG_PWR_CTRL, &data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update power mode configuration to intermediate Sleep Mode");
            #endif

            return 2;
        }

    }
    else if ((curr_mode == BMP390_MODE_FORCED) || (curr_mode == BMP390_MODE_FORCED2))
    {
        // If mode is forced, we have to wait until sample has completed and it returns to normal

        uint8_t odr_sel = 0;
        if (bmp390_read(handle, BMP390_REG_ODR, &odr_sel, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to read ODR register for sampling period");
            #endif

            return 4;
        }

        odr_sel = odr_sel & 0x1F;

        uint32_t samp_period = (0x00000001 << odr_sel) * 5000; // Calculate sampling period (max time till forced mode returns to sleep)

        Serial.print("samp_period = "); Serial.println(samp_period);

        // Set a timeout so we stop after the sampling period
        timer_handle_t timeout_timer;
        timer_init(&timeout_timer, samp_period);

        timer_start(&timeout_timer);

        while ((curr_mode == BMP390_MODE_FORCED) || (curr_mode == BMP390_MODE_FORCED2))
        {
            timer_blocking_delay(500); // Wait some time for a measurement to occur

            if (timer_check_exp(&timeout_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Timeout occurred waiting for Forced Mode to return to Sleep");
                #endif

                return 5;
            }

            if (bmp390_read(handle, BMP390_REG_PWR_CTRL, &data, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to read power mode configuration while waiting for Forced Mode");
                #endif

                return 6;
            }

            curr_mode = (data >> 4) & 0x03;
        }
        
    }
    else if (mode == BMP390_MODE_NORMAL)
    {
        // If attempting to switch to normal mode - we have to confirm the configuration is correct
        uint8_t odr = 0;

        if (bmp390_read(handle, BMP390_REG_ODR, &odr, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to check ODR settings");
            #endif

            return 7;
        }

        odr = odr & 0x1F;

        uint8_t osr = 0;

        if (bmp390_read(handle, BMP390_REG_OSR, &osr, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to check OSR settings");
            #endif

            return 8;
        }

        uint8_t osr_p = osr & 0x07;
        uint8_t osr_t = (osr >> 3) & 0x07;

        if ((odr < osr_p) || (odr < osr_t))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Error, incorrect ODR/OSR configuration");
            #endif

            return 9;
        }
    }

    data = (data & 0x03) | ((mode & 0x03) << 4);

    if (bmp390_write(handle, BMP390_REG_PWR_CTRL, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update power mode configuration to new mode");
        #endif

        return 3;
    }

    return 0;

}

/****************************************************************************
 * @brief Enable/disable I2C watchdog timer via the IF_CONF register. Will maintain rest of IF_CONF configuration.
 * @param handle Handle for bmp390 module.
 * @param enable Enable/disable of I2C Watchdog Timer.
 * @param period Timer period for I2C Watchdog Timer. 0: Short (1.25ms), 1: Long (40ms)
 * @return 0: Success
 * 1: Failed to obtain IF_CONF configuration
 * 2: Failed to update I2C Watchdog Timer Configuration
 ****************************************************************************/
uint8_t bmp390_set_watchdog(bmp390_t *handle, bool enable, bool period)
{

    uint8_t data = 0x00;

    // Get current configuration
    if (bmp390_read(handle, BMP390_REG_IF_CONF, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to obtain IF_CONF configuration");
        #endif

        return 1;
    }

    data = (data & 0x01) | (enable << 1) | (period << 2);

    if (bmp390_write(handle, BMP390_REG_IF_CONF, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update I2C Watchdog Timer Configuration");
        #endif

        return 2;
    }

    return 0;

}

/****************************************************************************
 * @brief Set interrupt configuration via the INT_CTRL register.
 * Note: Datasheet specifies an "int_ds" flag but doesn't elaborate what it's used for.
 * Treating it like a reserved value (defaults to false).
 * @param handle Handle for bmp390 module.
 * @param output 0: Push-pull, 1: Open-drain
 * @param polarity 0: Active Low, 1: Active High
 * @param latch Enable/disable latching for INT pin and INT_STATUS register.
 * @param watermark Enable/disable FIFO watermark interrupt for INT pin and INT_STATUS register.
 * @param full Enable/disable FIFO full interrupt for INT pin and INT_STATUS register.
 * @param drdy Enable/disable temperature & pressure data ready interrupt for INT pin and INT_STATUS register.
 * @return 0: Success
 * 1: Failed to update INT_CTRL register
 ****************************************************************************/
uint8_t bmp390_set_interrupt(bmp390_t *handle, bool output, bool polarity, bool latch, bool watermark, bool full, bool drdy)
{

    uint8_t data = 0x00 | output | ((uint8_t)polarity << 1) | ((uint8_t)latch << 2) | ((uint8_t)watermark << 3) | ((uint8_t)full << 4) | ((uint8_t)drdy << 6);

    if (bmp390_write(handle, BMP390_REG_INT_CTRL, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update INT_CTRL register");
        #endif

        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Enable/Disable Pressure and Temperature sensors.
 * @param handle Handle for bmp390 module.
 * @param press Enable/Disable pressure sensor.
 * @param temp Enable/Disable temperature sensor.
 * @return 0: Success
 * 1: Failed to obtain PWR_CTRL configuration
 * 2: Failed to update PWR_CTRL configuration
 ****************************************************************************/
uint8_t bmp390_enable_sensors(bmp390_t *handle, bool press, bool temp)
{

    uint8_t data = 0x00;

    // Get current configuration
    if (bmp390_read(handle, BMP390_REG_PWR_CTRL, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to obtain PWR_CTRL configuration");
        #endif

        return 1;
    }

    data = (data & 0xFC) | ((uint8_t)press & 0x01) | (((uint8_t)temp & 0x01) << 1);

    // Get current configuration
    if (bmp390_write(handle, BMP390_REG_PWR_CTRL, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update PWR_CTRL configuration");
        #endif

        return 2;
    }

    return 0;

}




/****************************************************************************
 * Measurements and Calibration
 ****************************************************************************/





/****************************************************************************
 * @brief Read trimming data from BMP390 memory and store parameters for compensation.
 * @param handle Handle for bmp390 module.
 * @return 0: Success
 * 1: Failed to read trimming coefficient registers
 ****************************************************************************/
static uint8_t bmp390_load_trimmings(bmp390_t *handle)
{
    uint8_t tempBuff[21];

    // Read coefficients
    if (bmp390_read(handle, BMP390_NVM_PAR_T1_LSB, tempBuff, 21))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read trimming coefficient registers");
        #endif

        return 1;
    }

    // Store coefficients in struct
    handle->calib_data.par_t1 = (double)((uint16_t)tempBuff[0] | ((uint16_t)tempBuff[1] << 8)) * 256.0; // * 2 ^ 8
    handle->calib_data.par_t2 = (double)((uint16_t)tempBuff[2] | ((uint16_t)tempBuff[3] << 8)) / 1073741824.0; // / 2 ^ 30
    handle->calib_data.par_t3 = (double)((int8_t)tempBuff[4]) / 281474976710656.0; // / 2 ^ 48

    handle->calib_data.par_p1 = ((double)((int16_t)tempBuff[5] | ((int16_t)tempBuff[6] << 8)) - 16384.0) / 1048576.0; // (() - 2 ^ 14) / 2 ^ 20
    handle->calib_data.par_p2 = ((double)((int16_t)tempBuff[7] | ((int16_t)tempBuff[8] << 8)) - 16384.0) / 536870912.0; // (() - 2 ^ 14) / 2 ^ 29
    handle->calib_data.par_p3 = (double)((int8_t)tempBuff[9]) / 4294967296.0; // / 2 ^ 32
    handle->calib_data.par_p4 = (double)((int8_t)tempBuff[10]) / 137438953472.0; // / 2 ^ 37
    handle->calib_data.par_p5 = (double)((uint16_t)tempBuff[11] | ((uint16_t)tempBuff[12] << 8)) * 8.0; // * 2 ^ 3
    handle->calib_data.par_p6 = (double)((uint16_t)tempBuff[13] | ((uint16_t)tempBuff[14] << 8)) / 64.0; // / 2 ^ 6
    handle->calib_data.par_p7 = (double)((int8_t)tempBuff[15]) / 256.0; // / 2 ^ 8
    handle->calib_data.par_p8 = (double)((int8_t)tempBuff[16]) / 32768.0; // / 2 ^ 15
    handle->calib_data.par_p9 = (double)((uint16_t)tempBuff[17] | ((uint16_t)tempBuff[18] << 8)) / 281474976710656.0; // / 2 ^ 48
    handle->calib_data.par_p10 = (double)((int8_t)tempBuff[19]) / 281474976710656.0; // / 2 ^ 48
    handle->calib_data.par_p11 = (double)((int8_t)tempBuff[20]) / 36893488147419103232.0; // / 2 ^ 65

    #ifdef DEBUG
    Serial.print("[DEBUG] T1: "); Serial.println(handle->calib_data.par_t1);
    Serial.print("[DEBUG] T2: "); Serial.println(handle->calib_data.par_t2);
    Serial.print("[DEBUG] T3: "); Serial.println(handle->calib_data.par_t3);

    Serial.print("[DEBUG] P1: "); Serial.println(handle->calib_data.par_p1);
    Serial.print("[DEBUG] P2: "); Serial.println(handle->calib_data.par_p2);
    Serial.print("[DEBUG] P3: "); Serial.println(handle->calib_data.par_p3);
    Serial.print("[DEBUG] P4: "); Serial.println(handle->calib_data.par_p4);
    Serial.print("[DEBUG] P5: "); Serial.println(handle->calib_data.par_p5);
    Serial.print("[DEBUG] P6: "); Serial.println(handle->calib_data.par_p6);
    Serial.print("[DEBUG] P7: "); Serial.println(handle->calib_data.par_p7);
    Serial.print("[DEBUG] P8: "); Serial.println(handle->calib_data.par_p8);
    Serial.print("[DEBUG] P9: "); Serial.println(handle->calib_data.par_p9);
    Serial.print("[DEBUG] P10: "); Serial.println(handle->calib_data.par_p10);
    Serial.print("[DEBUG] P11: "); Serial.println(handle->calib_data.par_p11);
    #endif

    return 0;

}

/****************************************************************************
 * @brief Compensate raw temperature data - use trimming coefficients to obtain true measurement from raw data.
 * Must be executed before attempting to compensate pressure data, as compensated temperature is needed to accurately determine pressure.
 * @param handle Handle for bmp390 module.
 * @param raw_temp Raw temperature data (32-bit)
 * @return Compensated temperature reading (double)
 ****************************************************************************/
static double bmp390_compensate_temperature(bmp390_t *handle, uint32_t raw_temp)
{

    double partial_data1;
    double partial_data2;

    partial_data1 = (double)(raw_temp - handle->calib_data.par_t1);
    partial_data2 = (double)(partial_data1 * handle->calib_data.par_t2);

    // Calculate compensated temperature and store it for use with pressure
    handle->calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * handle->calib_data.par_t3;

    // Return compensated temperature
    return handle->calib_data.t_lin;

}

/****************************************************************************
 * @brief Compensate raw pressure data - use trimming coefficients to obtain true measurement from raw data.
 * Must compensate for temperature first.
 * @param handle Handle for bmp390 module.
 * @param raw_temp Raw pressure data (32-bit)
 * @return Compensated pressure reading (double)
 ****************************************************************************/
static double bmp390_compensate_pressure(bmp390_t *handle, uint32_t raw_pressure)
{

    // Variable to store the compensated pressure
    double comp_press;

    // Temporary variables used for compensation
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    // Calibration data
    partial_data1 = handle->calib_data.par_p6 * handle->calib_data.t_lin;
    partial_data2 = handle->calib_data.par_p7 * (handle->calib_data.t_lin * handle->calib_data.t_lin);
    partial_data3 = handle->calib_data.par_p8 * (handle->calib_data.t_lin * handle->calib_data.t_lin * handle->calib_data.t_lin);

    partial_out1 = handle->calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = handle->calib_data.par_p2 * handle->calib_data.t_lin;
    partial_data2 = handle->calib_data.par_p3 * (handle->calib_data.t_lin * handle->calib_data.t_lin);
    partial_data3 = handle->calib_data.par_p4 * (handle->calib_data.t_lin * handle->calib_data.t_lin * handle->calib_data.t_lin);

    partial_out2 = (double)raw_pressure * (handle->calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (double)raw_pressure * (double)raw_pressure;
    partial_data2 = handle->calib_data.par_p9 + handle->calib_data.par_p10 * handle->calib_data.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((double)raw_pressure * (double)raw_pressure * (double)raw_pressure) * handle->calib_data.par_p11;

    // Calculate compensated pressure
    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;

}

/****************************************************************************
 * @brief Perform single read of temperature and pressure measurements and compensate to actuals.
 * @param handle Handle for bmp390 module.
 * @param temperature Pointer to store temperature reading (double)
 * @param pressure Pointer to store pressure reading (double)
 * @return 0: Success
 * 1: Failed to read registers DATA_0 through DATA_5
 ****************************************************************************/
uint8_t bmp390_get_temperature_pressure(bmp390_t *handle, double *temperature, double *pressure)
{

    uint8_t tempBuff[6];

    uint32_t raw_temp = 0x00;
    uint32_t raw_press = 0x00;

    memset(tempBuff, 0x00, 6);

    if (bmp390_read(handle, BMP390_REG_DATA_0, tempBuff, 6))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read registers DATA_0 through DATA_5");
        #endif

        return 1;
    }

    raw_press = (uint32_t)tempBuff[0] | ((uint32_t)tempBuff[1] << 8) | ((uint32_t)tempBuff[2] << 16);
    raw_temp = (uint32_t)tempBuff[3] | ((uint32_t)tempBuff[4] << 8) | ((uint32_t)tempBuff[5] << 16);

    // Temperature has to be compensated first in order to be used to compensate pressure
    if (raw_temp != 0)
        *temperature = bmp390_compensate_temperature(handle, raw_temp);
    else
        *temperature = 0; // If temperature isn't enabled, don't compensate to avoid aliasing pressure compensation

    if (raw_press != 0)
        *pressure = bmp390_compensate_pressure(handle, raw_press);
    else
        *pressure = 0;

    return 0;

}

/****************************************************************************
 * @brief Perform single read of sensor time measurement.
 * @param handle Handle for bmp390 module.
 * @param time Pointer to store sensor time reading (uint32_t)
 * @return 0: Success
 * 1: Failed to read registers SENSORTIME_0 through SENSORTIME_2
 ****************************************************************************/
uint8_t bmp390_get_sensor_time(bmp390_t *handle, uint32_t *time)
{

    uint8_t tempBuff[3];

    memset(tempBuff, 0x00, 3);

    if (bmp390_read(handle, BMP390_REG_SENSORTIME_0, tempBuff, 3))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read registers SENSORTIME_0 through SENSORTIME_2");
        #endif

        return 1;
    }

    *time = (uint32_t)tempBuff[0] | ((uint32_t)tempBuff[1] << 8) | ((uint32_t)tempBuff[2] << 16);

    return 0;

}





/****************************************************************************
 * FIFO
 ****************************************************************************/



/****************************************************************************
 * @brief Set FIFO configuration via FIFO_CONFIG registers.
 * @param handle Handle for bmp390 module.
 * @param fifoCfg FIFO Configuration structure to store the desired FIFO configuration.
 * @return 0: Success
 * 1: Failed to update FIFO_CONFIG_1 register
 * 2: Failed to update FIFO_CONFIG_2 register
 * 3: Failed to update FIFO Watermark 0 register
 * 4: Failed to update FIFO Watermark 1 register
 * 5: Failed to allocate memory for handle fifoCfg
 ****************************************************************************/
uint8_t bmp390_set_fifo(bmp390_t *handle, fifoCfg_t *fifoCfg)
{
    
    handle->fifoCfg = (fifoCfg_t *)malloc(sizeof(fifoCfg_t));
    if (handle->fifoCfg == NULL)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to allocate memory for handle fifoCfg");
        #endif

        return 5;
    }
    
    uint8_t fifo_config_1 = 0x00 | (uint8_t)fifoCfg->en | ((uint8_t)fifoCfg->stopOnFull << 1) | ((uint8_t)fifoCfg->timeEn << 2) | ((uint8_t)fifoCfg->pressEn << 3) | ((uint8_t)fifoCfg->tempEn << 4);
    uint8_t fifo_config_2 = 0x00 | (fifoCfg->subsampling & 0x07) | ((fifoCfg->dataSelect & 0x03) << 3);

    if (bmp390_write(handle, BMP390_REG_FIFO_CONFIG_2, &fifo_config_2, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update FIFO_CONFIG_2 register");
        #endif

        return 2;
    }

    if (bmp390_write(handle, BMP390_REG_FIFO_CONFIG_1, &fifo_config_1, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update FIFO_CONFIG_1 register");
        #endif

        return 1;
    }

    uint8_t wm0 = (uint8_t)(fifoCfg->watermark & 0xFF);
    uint8_t wm1 = (uint8_t)((fifoCfg->watermark >> 8) & 0x01);

    if (bmp390_write(handle, BMP390_REG_FIFO_WTM_0, &wm0, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update FIFO Watermark 0 register");
        #endif

        return 3;
    }

    if (bmp390_write(handle, BMP390_REG_FIFO_WTM_1, &wm1, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update FIFO Watermark 1 register");
        #endif

        return 4;
    }

    // Store parameters to handle
    
    *(handle->fifoCfg) = *fifoCfg;

    return 0;

}

/****************************************************************************
 * @brief Read FIFO (must be enabled first) and then store values to handle->fifoData.
 * Note: Currently does not work properly in I2C - working HAL cannot read more than 32 bytes at a time which causes partial frame reads.
 * @param handle Handle for bmp390 module.
 * @return 0: Success or FIFO is empty
 * 1: Failed to disable FIFO for reading
 * 2: Failed to read FIFO length
 * 3: Failed to read data from FIFO_DATA register
 * 4: Failed to re-enable FIFO after reading
 ****************************************************************************/
uint8_t bmp390_read_fifo(bmp390_t *handle)
{

    uint8_t lengthData[2] = {0x00, 0x00};

    if (bmp390_read(handle, BMP390_REG_FIFO_LENGTH_0, lengthData, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read FIFO length");
        #endif

        return 2;
    }
    
    volatile uint16_t fifoLength = (uint16_t)lengthData[0] | (((uint16_t)lengthData[1] & 0x01) << 8); // Max 512 bytes per datasheet

    if (fifoLength == 0)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] FIFO is empty");
        #endif

        return 0;
    }

    if (handle->fifoCfg->timeEn)
        fifoLength += 4; // + 4 for sensor time frame

    // Check if I2C and if so, check if we have mroe data than max buffer size
    if ((handle->busType == BMP390_I2C) && (fifoLength > I2C_MAX_BUFFER_SIZE))
    {
        uint16_t bytesRead = 0;
        uint16_t bytesLeft = fifoLength;
        uint16_t last_good_byte = 0;
        uint8_t frame_end = 0;
        uint8_t bytes_to_read = 0;
        uint8_t good_bytes = 0;

        while (bytesLeft > 0)
        {

            if (bytesLeft > I2C_MAX_BUFFER_SIZE)
                bytes_to_read = I2C_MAX_BUFFER_SIZE;
            else
                bytes_to_read = bytesLeft;

            if (bmp390_read(handle, BMP390_REG_FIFO_DATA, handle->fifoBuffer + bytesRead, bytes_to_read))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to read data from FIFO_DATA register");
                #endif

                return 3;
            }

            uint16_t u = bytesRead;

            while (u < (bytesRead + bytes_to_read))
            {
                
                switch (handle->fifoBuffer[u])
                {
                    case 0xA0: // Sensor Time Frame
                        u += 3;
                        frame_end = 4;
                        break;
                    case 0x94: // Pressure/Temp Frame
                        u += 6;
                        frame_end = 7;
                        break;
                    case 0x84: // Pressure Only Frame
                        u += 3;
                        frame_end = 4;
                        break;
                    case 0x90: // Temp Only Frame
                        u += 3;
                        frame_end = 4;
                        break;
                    case 0x48: // FIFO Input Config Frame
                        u += 1;
                        frame_end = 2;
                    case 0x44: // Config Error Frame
                        u += 1;
                        frame_end = 2;
                    case 0x80: // FIFO Empty Frame
                        u += 1;
                    default: // Everything Else
                        break;
                }

                if (u < (bytesRead + bytes_to_read))
                    last_good_byte = u;
                
                u++;
            }

            good_bytes = last_good_byte - bytesRead + 1;

            bytesRead = last_good_byte + 1; // Convert to bytes read

            bytesLeft -= good_bytes;

            if ((bytesLeft > 0) && (bytesLeft < frame_end))
                bytesLeft = frame_end;
            else if ((bytesLeft == 0) && (handle->fifoCfg->timeEn) && (handle->fifoBuffer[last_good_byte - 3] != 0xA0))
                bytesLeft = 4;

        }

        fifoLength = bytesRead; // Update fifoLength for parsing

    }
    else
    {
        if (bmp390_read(handle, BMP390_REG_FIFO_DATA, handle->fifoBuffer, fifoLength))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to read data from FIFO_DATA register");
            #endif

            return 3;
        }
    }
    
    bmp390_parse_fifo(handle, fifoLength);

    return 0;

}

/****************************************************************************
 * @brief Parse sensor frame data read from FIFO into handle->fifoData.
 * @param handle Handle for bmp390 module.
 * @return 0: Success
 * 1: Failed to allocate memory for fifoData
 ****************************************************************************/
static uint8_t bmp390_parse_fifo(bmp390_t *handle, uint16_t fifoLength)
{

    fifoFrame_t *workingFrame = NULL;
    
    uint32_t time = 0;

    if (handle->fifoCfg->timeEn)
        // Check if final frame is sensor time frame
        if (handle->fifoBuffer[fifoLength - 4] == 0xA0)
            time = (uint32_t)handle->fifoBuffer[fifoLength - 3] | ((uint32_t)handle->fifoBuffer[fifoLength - 2] << 8) | ((uint32_t)handle->fifoBuffer[fifoLength - 1] << 16);


    uint8_t mode;
    uint8_t params;

    uint16_t i = 0;
    
    while (i < fifoLength - 4)
    {

        // Check frame header
        mode = (handle->fifoBuffer[i] >> 6) & 0x03;
        params = (handle->fifoBuffer[i] >> 2) & 0x0F;

        if (mode == 0x02) // Sensor Frame
        {   
            // Verify received a valid frame
            if (((handle->fifoBuffer[i] & 0x0B) != 0) || ((handle->fifoBuffer[i] & 0x20) && ((handle->fifoBuffer[i] & 0x04) || (handle->fifoBuffer[i] & 0x10))))
            {
                i++;

                #ifdef DEBUG
                Serial.println("[DEBUG] Received Invalid Frame");
                #endif
                
                continue; // Not a valid frame, skip
            }
            else if (handle->fifoBuffer[i] == 0x80)
            {
                i++;

                #ifdef DEBUG
                Serial.println("[DEBUG] Received Empty Frame");
                #endif

                continue; // Empty frame, skip
            }
            
            
            if (workingFrame == NULL) // First valid sensor frame
            {
                fifoFrame_t *ptr = (fifoFrame_t *)malloc(sizeof(fifoFrame_t));

                if (ptr == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate memory for fifoData");
                    #endif

                    return 1;
                }

                if (handle->fifoData == NULL) // At head, no data left
                {
                    handle->fifoData = ptr;
                    workingFrame = handle->fifoData;
                    workingFrame->next = NULL;
                }
                else // Create new node if not at head
                {

                    workingFrame = handle->fifoData;

                    while (workingFrame->next != NULL)
                        workingFrame = workingFrame->next;

                    workingFrame->next = ptr; // Add new node

                    workingFrame = workingFrame->next; // Update working frame

                    workingFrame->next = NULL; // Ensure new node->next is NULL

                }
            }
            else
            {
                workingFrame->next = (fifoFrame_t *)malloc(sizeof(fifoFrame_t));

                if (workingFrame->next == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate memory for fifoData next");
                    #endif

                    return 1;
                }

                workingFrame = workingFrame->next;

                workingFrame->next = NULL;
            }
            
            i++;

            #ifdef DEBUG
            Serial.println("[DEBUG] Received Sensor Frame");
            #endif

            if (params == 0x05)
            {

                workingFrame->temperature = 0 | (uint32_t)handle->fifoBuffer[i] | ((uint32_t)handle->fifoBuffer[i+1] << 8) | ((uint32_t)handle->fifoBuffer[i+2] << 16);
                workingFrame->pressure = 0 | (uint32_t)handle->fifoBuffer[i+3] | ((uint32_t)handle->fifoBuffer[i+4] << 8) | ((uint32_t)handle->fifoBuffer[i+5] << 16);
                workingFrame->time = time;
                i += 6;
                
            }
            else if (params == 0x04)
            {

                workingFrame->temperature = 0 | (uint32_t)handle->fifoBuffer[i] | ((uint32_t)handle->fifoBuffer[i+1] << 8) | ((uint32_t)handle->fifoBuffer[i+2] << 16);
                workingFrame->pressure = 0;
                workingFrame->time = time;
                i += 3;
                
            }
            else if (params == 0x01)
            {

                workingFrame->pressure = 0 | (uint32_t)handle->fifoBuffer[i] | ((uint32_t)handle->fifoBuffer[i+1] << 8) | ((uint32_t)handle->fifoBuffer[i+2] << 16);
                workingFrame->temperature = 0;
                workingFrame->time = time;
                i += 3;
                
            }

        }
        else
        {
            i++; // Make sure we don't get stuck on control frames

            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid Mode");
            #endif
        }

    }
    
    return 0;

}

/****************************************************************************
 * @brief Retrieve single pressure and temperature measurement from handle->fifoData and compensate to actuals.
 * Use to retrieve data from FIFO after performing bmp390_read_fifo().
 * @param handle Handle for bmp390 module.
 * @param temp Pointer to store temperature reading (double)
 * @param press Pointer to store pressure reading (double)
 * @param time Pointer to store time reading (uint32_t)
 * @return 0: Success
 * 1: No data to retrieve from FIFO
 ****************************************************************************/
uint8_t bmp390_retrieve_fifo_value(bmp390_t *handle, double *temp, double *press, uint32_t *time)
{

    fifoFrame_t *workingFrame = handle->fifoData;

    if (workingFrame == NULL)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] No data to retrieve from FIFO");
        #endif

        return 1;
    }
    
    if (workingFrame->temperature != 0)
        *temp = bmp390_compensate_temperature(handle, workingFrame->temperature);
    else
        *temp = 0;

    if (workingFrame->pressure != 0)
        *press = bmp390_compensate_pressure(handle, workingFrame->pressure);
    else
        *press = 0;

    *time = workingFrame->time;
    
    handle->fifoData = workingFrame->next;
    
    free(workingFrame);
    
    return 0;

}

/****************************************************************************
 * @brief Submit fifo_flush command via the CMD register. Does not change FIFO config.
 * @param handle Handle for bmp390 module.
 * @return 0: Success
 * 1: Failed to write CMD register
 ****************************************************************************/
uint8_t bmp390_flush_fifo(bmp390_t *handle)
{

    uint8_t data = 0xB0;

    if (bmp390_write(handle, BMP390_REG_CMD, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CMD register");
        #endif

        return 1;
    }

    return 0;

}

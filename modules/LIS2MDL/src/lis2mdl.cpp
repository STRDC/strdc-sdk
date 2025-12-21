/*
 * LIS2MDL Module C++ File.
 *
 * @file        lis2mdl.cpp
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

#include "lis2mdl.h"

#include <math.h>


//#define DEBUG

#ifdef DEBUG
#include <Arduino.h>
#endif





/****************************************************************************
 * Executive Functions
 ****************************************************************************/






/****************************************************************************
 * @brief Initialize blank configuration for LIS2MDL IC:
 *  Initialize Attributes and Buffer,
 *  Configure associated GPIO,
 *  Initialize comm peripheral (I2C, SPI, UART).
 * @param handle Handle for lis2mdl module.
 * @param speed Speed of peripheral (Hz).
 * @return 0: Initialization was successful
 * 1: Failed to Find I2C Device
 * 2: Failed to change configuration to SPI 3 Wire
 * 3: Failed to enable temperature compensation as part of initialization
 * 4: Failed to enable mag data ready interrupt as part of initialization
 * 5: Failed to receive chip identifier as part of initialization
 * 6: Received incorrect chip identifier as part of initialization
 * 7: Failed to reboot memory for loading trimming parameters
 ****************************************************************************/
uint8_t lis2mdl_init(lis2mdl_t *handle, uint32_t speed)
{

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 20000); // Wait 20ms to confirm IC initialization has occurred

    timer_start(&gen_timer);

    while (!timer_check_exp(&gen_timer))
        ;

    if(handle->busType == LIS2MDL_I2C)
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

    }
    else if(handle->busType == LIS2MDL_SPI_4_WIRE)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Init SPI");
        #endif

        if (speed > 10000000)
            speed = 10000000; // Max speed is 10 MHz

        spi_open((spi_handle_t*)handle->bus, speed, SPI_MODE_3, SPI_BIT_ORDER_MSB);

        uint8_t data = 0x04; // Change to 4-Wire Interface

        handle->busType = LIS2MDL_SPI_3_WIRE; // Change to 3 Wire temporarily to write the register

        if (lis2mdl_write(handle, LIS2MDL_CFG_REG_C, &data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to switch to SPI 3 Wire on Initialization");
            #endif

            handle->busType = LIS2MDL_SPI_4_WIRE; // Change back

            return 2;
        }

        handle->busType = LIS2MDL_SPI_4_WIRE; // Change back

    }
    else if(handle->busType == LIS2MDL_SPI_3_WIRE)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Init SPI 3 Wire");
        #endif

        if (speed > 10000000)
            speed = 10000000; // Max speed is 10 MHz

        spi_open((spi_handle_t*)handle->bus, speed, SPI_MODE_3, SPI_BIT_ORDER_MSB);
    }

    uint8_t data;

    // Verify communication and IC by checking the WHO_AM_I register
    if (lis2mdl_read(handle, LIS2MDL_WHO_AM_I, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to receive chip identifier as part of initialization");
        #endif

        return 5;
    }

    if (data != 0x40)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Received incorrect chip identifier as part of initialization");
        #endif

        return 6;

    }

    data = 0x80; // Enable Temperature Compensation and Continuous Mode

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_A, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable temperature compensation as part of initialization");
        #endif

        return 3;
    }

    if(handle->busType == LIS2MDL_SPI_4_WIRE)
        data = 0x05; // Enable INT pin to trigger based on data-ready Zyxda bit in STATUS_REG (and maintain 4-wire SPI)
    else
        data = 0x01; // Enable INT pin to trigger based on data-ready Zyxda bit in STATUS_REG

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_C, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable mag data ready interrupt as part of initialization");
        #endif

        return 4;
    }

    // Load trimming parameters
    if (lis2mdl_reboot_memory(handle))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to reboot memory for loading trimming parameters");
        #endif

        return 7;
    }

    return 0;

}

/****************************************************************************
 * @brief Issue SW Reset via CFG_REG_A. This functions much like a POR.
 * Will automatically enable temperature compensation and set mode to continuous (as well as update SPI comms if that was the busType)
 * @param handle Handle for lis2mdl module.
 * @return 0: Success
 * 1: Failed to write CFG_REG_A with SW Reset
 * 2: Failed to enable temperature compensation as part of initialization
 * 3: Failed to enable mag data ready interrupt as part of initialization
 ****************************************************************************/
uint8_t lis2mdl_sw_reset(lis2mdl_t *handle)
{

    uint8_t data = 0x20;

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_A, &data, 1)) // Resets all CFG registers
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG_REG_A with SW Reset");
        #endif
        return 1;
    }

    // Re-enable default startup configuration
    
    data = 0x80; // Enable Temperature Compensation and Continuous Mode

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_A, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable temperature compensation as part of initialization");
        #endif

        return 2;
    }
    
    if(handle->busType == LIS2MDL_SPI_4_WIRE)
    {
        data = 0x04; // Change to 4-Wire Interface

        handle->busType = LIS2MDL_SPI_3_WIRE; // Change to 3 Wire temporarily to write the register

        if (lis2mdl_write(handle, LIS2MDL_CFG_REG_C, &data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to switch to SPI 3 Wire after reset");
            #endif

            handle->busType = LIS2MDL_SPI_4_WIRE; // Change back

            return 2;
        }

        handle->busType = LIS2MDL_SPI_4_WIRE; // Change back

        data = 0x05; // Mag data ready interrupt
    }
    else
        data = 0x01; // Mag data ready interrupt

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_C, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable mag data ready interrupt as part of initialization");
        #endif

        return 3;
    }

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 10); // Wait 10us for reboot to take effect (buffer from App Note 5us)

    timer_start(&gen_timer);

    while (!timer_check_exp(&gen_timer))
        ;
    
    return 0;

}

/****************************************************************************
 * @brief Issue magnetometer memory reboot via CFG_REG_A. Reloads the trimming parameters.
 * Normally done after POR or SW Reset. This will reset operating mode to idle!
 * Side note, this can also be used when one accidentally writes over a reserved register.
 * @param handle Handle for lis2mdl module.
 * @return 0: Success
 * 1: Failed to read CFG_REG_A
 * 2: Failed to write CFG_REG_A with REBOOT
 * 3: Failed to enter continuous operating mode after reboot
 ****************************************************************************/
uint8_t lis2mdl_reboot_memory(lis2mdl_t *handle)
{

    uint8_t data = 0;

    if (lis2mdl_read(handle, LIS2MDL_CFG_REG_A, &data, 1)) // Read current register to maintain configuration
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read CFG_REG_A");
        #endif
        return 1;
    }

    data = (data & 0xBF) | 0x40; // Mask REBOOT flag and set reboot

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_A, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG_REG_A with REBOOT");
        #endif
        return 2;
    }

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 30000); // Wait 30ms for reboot to take effect (buffer from 20ms in App Note)

    timer_start(&gen_timer);

    while (!timer_check_exp(&gen_timer))
        ;

    // Set continuous mode - Reboot places module into idle mode
    if (lis2mdl_set_op_mode(handle, LIS2MDL_MODE_CONTINUOUS))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enter continuous operating mode after reboot");
        #endif

        return 3;
    }

    return 0;

}






/****************************************************************************
 * Messaging Framework
 ****************************************************************************/





/****************************************************************************
 * @brief Send message to LIS2MDL module. Abstracts communication peripheral.
 * @param handle Handle for lis2mdl module.
 * @param data Data to send to lis2mdl module (array of bytes).
 * @param bytes Length of data to send (max 255).
 * @return 0: Success
 * 1: Invalid number of bytes to write (must be > 2)
 * 2: Failed to write I2C
 * 3: Parse buffer after SPI read/write failed
 ****************************************************************************/
uint8_t lis2mdl_write(lis2mdl_t *handle, uint8_t address, uint8_t *data, uint8_t bytes)
{

    uint8_t txData[32];

    txData[0] = address;

    memcpy(txData + 1, data, bytes);

    if(handle->busType == LIS2MDL_I2C)
    {

        if (i2c_write((i2c_handle_t*)handle->bus, txData, bytes + 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write I2C message");
            #endif
            return 2;
        }
    }
    if((handle->busType == LIS2MDL_SPI_4_WIRE) || (handle->busType == LIS2MDL_SPI_3_WIRE))
    {

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
 * @brief Receive messages from LIS2MDL module.
 * For I2C: This function performs the Random Read Access - checking addresses 0xFD and 0xFE before reading the amount of data from 0xFF.
 * The data for all cases is stored in a singular buffer, then it can separated using gnss_parse_buffer().
 * @param handle Handle for lis2mdl module.
 * @return 0: Success
 * 1: I2C Failed to read register
 ****************************************************************************/
uint8_t lis2mdl_read(lis2mdl_t *handle, uint8_t address, uint8_t *data, uint8_t length)
{

    if(handle->busType == LIS2MDL_I2C)
    {
        
        if (i2c_read_reg((i2c_handle_t*)handle->bus, &address, 0x01, data, length))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] I2C Failed to read data");
            #endif
            return 1;
        }

        #ifdef DEBUG
        Serial.print("[DEBUG] Incoming Rx: ");
        for (uint8_t i = 0; i < length; i++)
        {
            Serial.print(data[i]);
            Serial.print(" ");
        }
        Serial.println();
        #endif

    }
    if(handle->busType == LIS2MDL_SPI_4_WIRE)
    {

        uint8_t addr_data[32]; // Max 2 bytes data + address
        addr_data[0] = address;
        memset(addr_data + 1, 0x00, 2);

        uint8_t tempBuff[32]; // Create Temp Buffer

        spi_write_read((spi_handle_t*)handle->bus, handle->busAddr, addr_data, tempBuff, (size_t)length);

        memcpy(data, tempBuff + 1, length); // Remove initial empty byte

        #ifdef DEBUG
        Serial.print("[DEBUG] Incoming Rx: ");
        for (uint8_t i = 0; i < length; i++)
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
 * Configuration
 ****************************************************************************/





/****************************************************************************
 * @brief Update output data rate via CFG_REG_A.
 * @param handle Handle for lis2mdl module.
 * @param rate Output data rate (See definitions in header file).
 * @return 0: Success
 * 1: Incorrect output data rate
 * 2: Failed to read CFG_REG_A
 * 3: Failed to write CFG_REG_A with ODR flags
 ****************************************************************************/
uint8_t lis2mdl_set_rate(lis2mdl_t *handle, uint8_t rate)
{

    if (rate > LIS2MDL_100HZ)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Incorrect output data rate");
        #endif
        return 1;
    }

    uint8_t data = 0;

    if (lis2mdl_read(handle, LIS2MDL_CFG_REG_A, &data, 1)) // Read current register to maintain configuration
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read CFG_REG_A");
        #endif
        return 2;
    }

    data = (data & 0xF3) | (rate << 2); // Mask ODR1:ODR0 flags and set output data rate

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_A, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG_REG_A with ODR flags");
        #endif
        return 3;
    }

    return 0;

}

/****************************************************************************
 * @brief Update low-power mode via CFG_REG_A.
 * @param handle Handle for lis2mdl module.
 * @param mode 0: High-Resolution mode. 1: Enable Low-Power mode.
 * @return 0: Success
 * 1: Failed to read CFG_REG_A
 * 2: Failed to write CFG_REG_A with LP flag
 ****************************************************************************/
uint8_t lis2mdl_set_low_power(lis2mdl_t *handle, bool mode)
{

    uint8_t data = 0;

    if (lis2mdl_read(handle, LIS2MDL_CFG_REG_A, &data, 1)) // Read current register to maintain configuration
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read CFG_REG_A");
        #endif
        return 1;
    }

    data = (data & 0xEF) | (mode << 4); // Mask LP flag and set mode

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_A, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG_REG_A with LP flag");
        #endif
        return 2;
    }

    return 0;

}

/****************************************************************************
 * @brief Update operating mode via CFG_REG_A.
 * @param handle Handle for lis2mdl module.
 * @param mode Operating mode (See definitions in header file).
 * @return 0: Success
 * 1: Incorrect operating mode
 * 2: Failed to read CFG_REG_A
 * 3: Failed to write CFG_REG_A with MD flags
 ****************************************************************************/
uint8_t lis2mdl_set_op_mode(lis2mdl_t *handle, uint8_t mode)
{

    if (mode > LIS2MDL_MODE_IDLE)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Incorrect operating mode");
        #endif
        return 1;
    }

    uint8_t data = 0;

    if (lis2mdl_read(handle, LIS2MDL_CFG_REG_A, &data, 1)) // Read current register to maintain configuration
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read CFG_REG_A");
        #endif
        return 2;
    }

    data = (data & 0xFC) | mode; // Mask MD1:MD0 flags and set output data rate

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_A, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG_REG_A with MD flags");
        #endif
        return 3;
    }

    return 0;

}

/****************************************************************************
 * @brief Set low-pass filter via CFG_REG_B.
 * @param handle Handle for lis2mdl module.
 * @param mode 0: Disable low-pass filter. 1: Enable low-pass filter
 * @return 0: Success
 * 1: Failed to read CFG_REG_B
 * 2: Failed to write CFG_REG_B with LPF flag
 ****************************************************************************/
uint8_t lis2mdl_set_low_pass(lis2mdl_t *handle, bool filter)
{

    uint8_t data = 0;

    if (lis2mdl_read(handle, LIS2MDL_CFG_REG_B, &data, 1)) // Read current register to maintain configuration
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read CFG_REG_B");
        #endif
        return 1;
    }

    data = (data & 0xFE) | 0x01; // Mask LPF flag and set LPF

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_B, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG_REG_B with LPF");
        #endif
        return 2;
    }

    return 0;

}

/****************************************************************************
 * @brief Set interrupt threshold for all axes on LIS2MDL module.
 * @param handle Handle for lis2mdl module.
 * @param threshold Threshold value in mgauss.
 * @return 0: Success
 * 1: Threshold must be an absolute number (no negatives allowed)
 * 2: Failed to write interrupt threshold value
 ****************************************************************************/
uint8_t lis2mdl_set_threshold(lis2mdl_t *handle, float threshold)
{

    if (threshold < 0)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Threshold must be an absolute number (no negatives allowed)");
        #endif
        return 1;
    }

    uint8_t data[2];

    data[0] = (uint16_t)(threshold / 1.5) & 0xFF;
    data[1] = ((uint16_t)(threshold / 1.5) >> 8) & 0xFF;

    if (lis2mdl_write(handle, LIS2MDL_INT_THS_L_REG, data, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write interrupt threshold value");
        #endif
        return 2;
    }

    return 0;

}

/****************************************************************************
 * @brief Set sensor report offsets on LIS2MDL module.
 * @param handle Handle for lis2mdl module.
 * @param x X direction offset.
 * @param y Y direction offset.
 * @param z Z direction offset.
 * @return 0: Success
 * 1: Failed to write hard-iron offset registers
 ****************************************************************************/
uint8_t lis2mdl_set_offsets(lis2mdl_t *handle, float x, float y, float z)
{

    uint8_t data[6];

    data[0] = ((int16_t)(x / 1.5)) & 0xFF;
    data[1] = ((int16_t)(x / 1.5) >> 8) & 0xFF;
    data[2] = ((int16_t)(y / 1.5)) & 0xFF;
    data[3] = ((int16_t)(y / 1.5) >> 8) & 0xFF;
    data[4] = ((int16_t)(z / 1.5)) & 0xFF;
    data[5] = ((int16_t)(z / 1.5) >> 8) & 0xFF;

    if (lis2mdl_write(handle, LIS2MDL_OFFSET_X_REG_L, data, 6))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write hard-iron offset registers");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Set interrupt configuration of LIS2MDL module via INT_CTRL_REG.
 * Will control the behavior of the INT_SOURCE_REG register.
 * @param handle Handle for lis2mdl module.
 * @param x Set to enable interrupt on X-axis.
 * @param y Set to enable interrupt on Y-axis.
 * @param z Set to enable interrupt on Z-axis.
 * @param polarity 1: INT_SOURCE_REG INT bit is HIGH when interrupt is detected. 0: Same but LOW. Also controls signal polarity on INT pin.
 * @param latch 1: INT_SOURCE_REG bits are latched and cleared by reading INT_SOURCE_REG. 0: Same but pulsed.
 * @param enabled Set to enable generation of INT bit on INT_SOURCE_REG.
 * @return 0: Success
 * 1: Failed to read INT_SOURCE_REG
 ****************************************************************************/
uint8_t lis2mdl_set_int_cfg(lis2mdl_t *handle, bool x, bool y, bool z, bool polarity, bool latch, bool enabled)
{

    uint8_t data = 0x00 | ((uint8_t)x << 7) | ((uint8_t)y << 6) | ((uint8_t)z << 5) | ((uint8_t)polarity << 2) | ((uint8_t)latch << 1) | (uint8_t)enabled;

    if (lis2mdl_write(handle, LIS2MDL_INT_CTRL_REG, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write INT_CTRL_REG");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Enable (or Disable) interrupt on INT/DRDY pin of LIS2MDL module via CFG_REG_C.
 * @param handle Handle for lis2mdl module.
 * @param intOnPin Set to trigger INT/DRDY pin on INT from INT_SOURCE_REG. Sets pin to push-pull mode.
 * @param drdyOnPin Set to trigger INT/DRDY pin on Zyxda (New data available) from STATUS_REG. Sets pin to push-pull mode.
 * @return 0: Success
 * 1: Failed to read INT_SOURCE_REG
 * 2: Failed to write CFG_REG_C
 ****************************************************************************/
uint8_t lis2mdl_enable_int(lis2mdl_t *handle, bool intOnPin, bool drdyOnPin)
{

    uint8_t data = 0;

    if (lis2mdl_read(handle, LIS2MDL_CFG_REG_C, &data, 1)) // Read current register to maintain configuration
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read CFG_REG_C");
        #endif
        return 1;
    }

    data = (data & 0xBE) | ((uint8_t)intOnPin << 6) | (uint8_t)drdyOnPin; // Mask flags and set

    if (lis2mdl_write(handle, LIS2MDL_CFG_REG_C, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG_REG_C");
        #endif
        return 2;
    }

    return 0;

}







/****************************************************************************
 * Register Retrieval and Sensor Reading
 ****************************************************************************/






/****************************************************************************
 * @brief Get status of LIS2MDL module via STATUS_REG.
 * @param handle Handle for lis2mdl module.
 * @param status Pointer to 8-bit integer for status flags to copy data to. (See datasheet and header file for interpretation)
 * @return 0: Success
 * 1: Failed to read STATUS_REG
 ****************************************************************************/
uint8_t lis2mdl_get_status(lis2mdl_t *handle, uint8_t *status)
{

    if (lis2mdl_read(handle, LIS2MDL_STATUS_REG, status, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read STATUS_REG");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Get interrupt source status of LIS2MDL module via INT_SOURCE_REG.
 * @param handle Handle for lis2mdl module.
 * @param status Pointer to 8-bit integer for status flags to copy data to. (See datasheet and header file for interpretation)
 * @return 0: Success
 * 1: Failed to read INT_SOURCE_REG
 ****************************************************************************/
uint8_t lis2mdl_get_int_source(lis2mdl_t *handle, uint8_t *status)
{

    if (lis2mdl_read(handle, LIS2MDL_INT_SOURCE_REG, status, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read INT_SOURCE_REG");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Get magnetic field sensor report on LIS2MDL module.
 * @param handle Handle for lis2mdl module.
 * @param x Pointer to float for x-axis magnetic field to copy data to.
 * @param y Pointer to float for y-axis magnetic field to copy data to.
 * @param z Pointer to float for z-axis magnetic field to copy data to.
 * @return 0: Success
 * 1: Failed to read magnetic field data
 ****************************************************************************/
uint8_t lis2mdl_get_mags(lis2mdl_t *handle, float *x, float *y, float *z)
{

    uint8_t data[6];

    if (lis2mdl_read(handle, LIS2MDL_OUTX_L_REG, data, 6)) // Read multiple registers
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read magnetic field data");
        #endif
        return 1;
    }

    *x = (float)((int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8))) * 1.5; // convert to mgauss
    *y = (float)((int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8))) * 1.5; // convert to mgauss
    *z = (float)((int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8))) * 1.5; // convert to mgauss
    
    return 0;

}

/****************************************************************************
 * @brief Get internal temperature reading on LIS2MDL module.
 * @param handle Handle for lis2mdl module.
 * @param temp Pointer to float for temperature to copy data to.
 * @return 0: Success
 * 1: Failed to read internal temperature data
 ****************************************************************************/
uint8_t lis2mdl_get_internal_temp(lis2mdl_t *handle, float *temp)
{

    uint8_t data[2];

    if (lis2mdl_read(handle, LIS2MDL_TEMP_OUT_L_REG, data, 2)) // Read multiple registers
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to read internal temperature data");
        #endif
        return 1;
    }

    // Convert to degC
    *temp = (float)((int16_t)((uint16_t)data[0] | (((uint16_t)data[1]) << 8))) / 0xFF + 25.0; // 8 LSB/C means 255(?), then add 25C per AN5069
    
    return 0;

}


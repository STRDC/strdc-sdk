/*
 * GNSS-UBLOX Module C++ File.
 *
 * @file        gnss-ublox.cpp
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

#include "gnss-ublox.h"

#include <string.h>
#include <math.h>


//#define DEBUG

#ifdef DEBUG
#include <Arduino.h>
#endif

#define ADD_MEMORY

#ifdef ADD_MEMORY
#define SERIAL_RX_BUFFER_SIZE 1024
uint8_t uart_buff[SERIAL_RX_BUFFER_SIZE]; // Additional buffer for UART (not as much as we need but it's better than nothing)
#endif

/****************************************************************************
 * Static Function Declarations
 ****************************************************************************/

static uint8_t gnss_tx(gnss_t *, uint8_t *, uint8_t);
static uint8_t gnss_ubx_msg(gnss_t *, uint8_t, uint8_t, uint16_t, uint8_t *, bool);
static uint8_t gnss_nmea_msg(gnss_t *, uint8_t, uint16_t);
static uint8_t gnss_pubx_msg(gnss_t *, uint8_t *, uint8_t);
static uint8_t gnss_parse_buffer(gnss_t *);
static uint16_t gnss_find_msg_start(uint8_t *, uint16_t, uint16_t);
static uint16_t gnss_find_msg_end(gnss_t *, uint16_t, uint16_t);
static uint8_t gnss_ascii_data(uint8_t);
static double parse_decimal_string(uint8_t *, uint8_t, uint8_t);
static uint8_t gnss_find_next_group(uint8_t *, uint16_t, uint16_t);
static uint32_t gnss_parse_time(uint8_t *, uint8_t);
static uint16_t gnss_nmea_checksum(uint8_t *, uint16_t);
static uint16_t gnss_ubx_checksum(gnss_msg_t *);
static uint16_t getUByte16_LEnd(uint8_t *, uint16_t);
static int16_t getIByte16_LEnd(uint8_t *, uint16_t);
static uint32_t getUByte32_LEnd(uint8_t *, uint16_t);
static int32_t getIByte32_LEnd(uint8_t *, uint16_t);
static float getFloat(uint8_t *, uint16_t);
static uint8_t gnss_create_info_node(gnss_t *, gnss_msg_t *, uint16_t);


/****************************************************************************
 * Executive Functions
 ****************************************************************************/




/****************************************************************************
 * @brief Initialize blank configuration for GNSS-UBLOX IC:
 *  Initialize Attributes and Buffer,
 *  Configure associated GPIO,
 *  Initialize comm peripheral (I2C, SPI, UART).
 * @param handle Handle for ublox gnss module.
 * @param mainTalker Default TalkerID for sending NMEA Tx messages to the gnss module.
 * @return 0: Initialization was successfull
 *  1: Failed to Find I2C Device
 ****************************************************************************/
uint8_t gnss_init(gnss_t *handle, uint8_t mainTalker)
{

    // Initialize Attributes and Buffer

    handle->write_message = 0;
    handle->pending_messages = 0;
    handle->next_message = 0;

    handle->mainTalker = mainTalker;

    handle->awake = true;

    for (uint8_t i = 0; i < MSG_BUFFER_COUNT; i++)
        memset(handle->messages[i].buffer, 0x00, MSG_BUFFER_SIZE);

    // Setup GPIO

    gpio_mode(handle->pinRst, GPIO_MODE_OUTPUT);
    gpio_write(handle->pinRst, GPIO_HIGH);

    if(handle->busType == GNSS_I2C)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Init I2C");
        #endif
        
        if(i2c_find((i2c_handle_t*)handle->bus, handle->busAddr))
        {

            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to init I2C");
            #endif

            return 1; // Didn't receive response from I2C Address
        }
        
        i2c_set_addr((i2c_handle_t*)handle->bus, handle->busAddr);

    }
    else if(handle->busType == GNSS_SPI)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Init SPI");
        #endif
    }
    else if(handle->busType == GNSS_UART)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Init UART");
        #endif

        #ifdef ADD_MEMORY
        // Need to increase buffer because native 64 bytes on Teensy 4.1 is too low
        serial_buffer_read_add((serial_handle_t*)handle->bus, uart_buff, SERIAL_RX_BUFFER_SIZE);
        #endif
    }

    handle->txMsg = NULL;
    handle->info = NULL;
    handle->cfgMsg = NULL;
    handle->nmeaStdDtm = NULL;
    handle->nmeaStdGbs = NULL;
    handle->nmeaStdGga = NULL;
    handle->nmeaStdGll = NULL;
    handle->nmeaStdGns = NULL;
    handle->nmeaStdGrs = NULL;
    handle->nmeaStdGsa = NULL;
    handle->nmeaStdGst = NULL;
    handle->nmeaStdGsv = NULL;
    handle->nmeaStdRlm = NULL;
    handle->nmeaStdRmc = NULL;
    handle->nmeaStdVlw = NULL;
    handle->nmeaStdVtg = NULL;
    handle->nmeaStdZda = NULL;
    handle->nmeaPubxPos = NULL;
    handle->nmeaPubxSvstatus = NULL;
    handle->nmeaPubxTime = NULL;
    handle->ubxNavAopstatus = NULL;
    handle->ubxNavClock = NULL;
    handle->ubxNavCov = NULL;
    handle->ubxNavDop = NULL;
    handle->ubxNavEoe = NULL;
    handle->ubxNavOdo = NULL;
    handle->ubxNavOrb = NULL;
    handle->ubxNavPl = NULL;
    handle->ubxNavPosecef = NULL;
    handle->ubxNavPosllh = NULL;
    handle->ubxNavPvt = NULL;
    handle->ubxNavSat = NULL;
    handle->ubxNavSbas = NULL;
    handle->ubxNavSig = NULL;
    handle->ubxNavSlas = NULL;
    handle->ubxNavStatus = NULL;
    handle->ubxNavTimebds = NULL;
    handle->ubxNavTimegal = NULL;
    handle->ubxNavTimeglo = NULL;
    handle->ubxNavTimegps = NULL;
    handle->ubxNavTimels = NULL;
    handle->ubxNavTimenavic = NULL;
    handle->ubxNavTimeqzss = NULL;
    handle->ubxNavTimeutc = NULL;
    handle->ubxNavVelecef = NULL;
    handle->ubxNavVelned = NULL;
    handle->ubxRxmMeas20 = NULL;
    handle->ubxRxmMeas50 = NULL;
    handle->ubxRxmMeasc12 = NULL;
    handle->ubxRxmMeasd12 = NULL;
    handle->ubxRxmMeasx = NULL;
    handle->ubxRxmRlm = NULL;
    handle->ubxRxmSfrbx = NULL;
    handle->ubxSecSig = NULL;
    handle->ubxSecSiglog = NULL;
    handle->ubxSecUniqid = NULL;
    handle->ubxTimTm2 = NULL;
    handle->ubxTimTp = NULL;
    handle->ubxTimVrfy = NULL;
    handle->ubxLogBatch = NULL;
    handle->ubxLogFindtime = NULL;
    handle->ubxLogInfo = NULL;
    handle->ubxMonBatch = NULL;
    handle->ubxMonComms = NULL;
    handle->ubxMonGnss = NULL;
    handle->ubxMonHw3 = NULL;
    handle->ubxMonPatch = NULL;
    handle->ubxMonRcvrstat = NULL;
    handle->ubxMonRf = NULL;
    handle->ubxMonSpan = NULL;
    handle->ubxMonVer = NULL;
    handle->ubxSecSig = NULL;
    handle->ubxSecSiglog = NULL;

    return 0;

}

/****************************************************************************
 * @brief Reset GNSS chip via Reset pin.
 * @param handle Handle for ublox gnss module.
 ****************************************************************************/
void gnss_reset_hw(gnss_t *handle)
{

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 1500); // 1.5ms, says 1ms per datasheet but just to be safe :)

    gpio_write(handle->pinRst, GPIO_LOW);

    timer_start(&gen_timer);

    while (!timer_check_exp(&gen_timer))
        ;

    gpio_write(handle->pinRst, GPIO_HIGH);

}

/****************************************************************************
 * @brief Reset ublox gnss module via UBX-CFG-RST.
 * @param handle Handle for ublox gnss module.
 * @param bbrMask BBR Sections to clear. See UBX-CFG-RST for details (0x0000: Hot Start, 0x0001: Warm Start, 0xFFFF: Cold Start)
 * @param mode Reset Type. See UBX-CFG-RST for details
 * @return 0: Success
 *  1: Failed to send message
 ****************************************************************************/
uint8_t gnss_reset_sw(gnss_t *handle, uint16_t bbrMask, uint8_t mode)
{


    // Send UBX-CFG-RST

    uint8_t data[4];

    data[0] = bbrMask & 0xFF; // Change to little endian
    data[1] = (bbrMask >> 8) & 0xFF;
    data[2] = mode;
    data[3] = 0x00; // Reserved

    if(gnss_ubx_msg(handle, (GNSS_UBX_CFG_RST >> 8) & 0xFF, GNSS_UBX_CFG_RST & 0xFF, 0x04, data, 0))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send SW reset");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Generic function for allocating memory. This is intended to be used to avoid allocating memory past initialization (should runtime allocation be unacceptable).
 * Otherwise, the normal program functionality will dynamically allocate memory as necessary.
 * ex. handle->ubxTimTp = (gnss_ubx_tim_tp_t*)allocate_msg((void**)&handle->ubxTimTp, sizeof(gnss_ubx_tim_tp_t));
 * @param ptr Pointer to assign memory allocation to.
 * @param message Size of structure to use.
 * @return void* ptr or NULL if failed to allocate memory
 ****************************************************************************/
void* allocate_msg(void **ptr, size_t struct_size)
{
    *ptr = malloc(struct_size);

    if (*ptr == NULL)
        return NULL;

    return *ptr;
}

/****************************************************************************
 * @brief Generic function for freeing all allocated memory for messages.
 * @param handle Handle for ublox gnss module.
 ****************************************************************************/
void free_all_msg(gnss_t *handle)
{

    gnss_tx_msg_node_t *currentTx = handle->txMsg;
    gnss_tx_msg_node_t *nextTx = NULL;

    if (currentTx != NULL)
    {
        currentTx = currentTx->next;

        free(handle->txMsg->data);
        free(handle->txMsg);
        handle->txMsg = NULL;

        while (currentTx != NULL)
        {
            nextTx = currentTx->next;

            free(currentTx->data);
            free(currentTx);

            currentTx = nextTx;
        }
    }

    gnss_info_node_t *currentInfo = handle->info;
    gnss_info_node_t *nextInfo = NULL;

    if (currentInfo != NULL)
    {

        currentInfo = currentInfo->next;

        free(handle->info->data);
        free(handle->info);
        handle->info = NULL;

        while (currentInfo != NULL)
        {
            nextInfo = currentInfo->next;

            free(currentInfo->data);
            free(currentInfo);

            currentInfo = nextInfo;
        }
    }

    if (handle->cfgMsg != NULL)
    {
        free(handle->cfgMsg);
        handle->cfgMsg = NULL;
    }
    if (handle->nmeaStdDtm != NULL)
    {
        free(handle->nmeaStdDtm);
        handle->nmeaStdDtm = NULL;
    }
    if (handle->nmeaStdGbs != NULL)
    {
        free(handle->nmeaStdGbs);
        handle->nmeaStdGbs = NULL;
    }
    if (handle->nmeaStdGga != NULL)
    {
        free(handle->nmeaStdGga);
        handle->nmeaStdGga = NULL;
    }
    if (handle->nmeaStdGll != NULL)
    {
        free(handle->nmeaStdGll);
        handle->nmeaStdGll = NULL;
    }
    if (handle->nmeaStdGns != NULL)
    {
        free(handle->nmeaStdGns);
        handle->nmeaStdGns = NULL;
    }
    if (handle->nmeaStdGrs != NULL)
    {
        free(handle->nmeaStdGrs);
        handle->nmeaStdGrs = NULL;
    }
    if (handle->nmeaStdGsa != NULL)
    {
        free(handle->nmeaStdGsa);
        handle->nmeaStdGsa = NULL;
    }
    if (handle->nmeaStdGst != NULL)
    {
        free(handle->nmeaStdGst);
        handle->nmeaStdGst = NULL;
    }
    if (handle->nmeaStdGsv != NULL)
    {
        free(handle->nmeaStdGsv);
        handle->nmeaStdGsv = NULL;
    }
    if (handle->nmeaStdRlm != NULL)
    {
        free(handle->nmeaStdRlm);
        handle->nmeaStdRlm = NULL;
    }
    if (handle->nmeaStdRmc != NULL)
    {
        free(handle->nmeaStdRmc);
        handle->nmeaStdRmc = NULL;
    }
    if (handle->nmeaStdVlw != NULL)
    {
        free(handle->nmeaStdVlw);
        handle->nmeaStdVlw = NULL;
    }
    if (handle->nmeaStdVtg != NULL)
    {
        free(handle->nmeaStdVtg);
        handle->nmeaStdVtg = NULL;
    }
    if (handle->nmeaStdZda != NULL)
    {
        free(handle->nmeaStdZda);
        handle->nmeaStdZda = NULL;
    }
    if (handle->nmeaPubxPos != NULL)
    {
        free(handle->nmeaPubxPos);
        handle->nmeaPubxPos = NULL;
    }
    if (handle->nmeaPubxSvstatus != NULL)
    {
        free(handle->nmeaPubxSvstatus);
        handle->nmeaPubxSvstatus = NULL;
    }
    if (handle->nmeaPubxTime != NULL)
    {
        free(handle->nmeaPubxTime);
        handle->nmeaPubxTime = NULL;
    }
    if (handle->ubxNavAopstatus != NULL)
    {
        free(handle->ubxNavAopstatus);
        handle->ubxNavAopstatus = NULL;
    }
    if (handle->ubxNavClock != NULL)
    {
        free(handle->ubxNavClock);
        handle->ubxNavClock = NULL;
    }
    if (handle->ubxNavCov != NULL)
    {
        free(handle->ubxNavCov);
        handle->ubxNavCov = NULL;
    }
    if (handle->ubxNavDop != NULL)
    {
        free(handle->ubxNavDop);
        handle->ubxNavDop = NULL;
    }
    if (handle->ubxNavEoe != NULL)
    {
        free(handle->ubxNavEoe);
        handle->ubxNavEoe = NULL;
    }
    if (handle->ubxNavOdo != NULL)
    {
        free(handle->ubxNavOdo);
        handle->ubxNavOdo = NULL;
    }
    if (handle->ubxNavOrb != NULL)
    {
        free(handle->ubxNavOrb);
        handle->ubxNavOrb = NULL;
    }
    if (handle->ubxNavPl != NULL)
    {
        free(handle->ubxNavPl);
        handle->ubxNavPl = NULL;
    }
    if (handle->ubxNavPosecef != NULL)
    {
        free(handle->ubxNavPosecef);
        handle->ubxNavPosecef = NULL;
    }
    if (handle->ubxNavPosllh != NULL)
    {
        free(handle->ubxNavPosllh);
        handle->ubxNavPosllh = NULL;
    }
    if (handle->ubxNavPvt != NULL)
    {
        free(handle->ubxNavPvt);
        handle->ubxNavPvt = NULL;
    }
    if (handle->ubxNavSat != NULL)
    {
        free(handle->ubxNavSat);
        handle->ubxNavSat = NULL;
    }
    if (handle->ubxNavSbas != NULL)
    {
        free(handle->ubxNavSbas);
        handle->ubxNavSbas = NULL;
    }
    if (handle->ubxNavSig != NULL)
    {
        free(handle->ubxNavSig);
        handle->ubxNavSig = NULL;
    }
    if (handle->ubxNavSlas != NULL)
    {
        free(handle->ubxNavSlas);
        handle->ubxNavSlas = NULL;
    }
    if (handle->ubxNavStatus != NULL)
    {
        free(handle->ubxNavStatus);
        handle->ubxNavStatus = NULL;
    }
    if (handle->ubxNavTimebds != NULL)
    {
        free(handle->ubxNavTimebds);
        handle->ubxNavTimebds = NULL;
    }
    if (handle->ubxNavTimegal != NULL)
    {
        free(handle->ubxNavTimegal);
        handle->ubxNavTimegal = NULL;
    }
    if (handle->ubxNavTimeglo != NULL)
    {
        free(handle->ubxNavTimeglo);
        handle->ubxNavTimeglo = NULL;
    }
    if (handle->ubxNavTimegps != NULL)
    {
        free(handle->ubxNavTimegps);
        handle->ubxNavTimegps = NULL;
    }
    if (handle->ubxNavTimels != NULL)
    {
        free(handle->ubxNavTimels);
        handle->ubxNavTimels = NULL;
    }
    if (handle->ubxNavTimeqzss != NULL)
    {
        free(handle->ubxNavTimeqzss);
        handle->ubxNavTimeqzss = NULL;
    }
    if (handle->ubxNavTimeutc != NULL)
    {
        free(handle->ubxNavTimeutc);
        handle->ubxNavTimeutc = NULL;
    }
    if (handle->ubxNavVelecef != NULL)
    {
        free(handle->ubxNavVelecef);
        handle->ubxNavVelecef = NULL;
    }
    if (handle->ubxNavVelned != NULL)
    {
        free(handle->ubxNavVelned);
        handle->ubxNavVelned = NULL;
    }
    if (handle->ubxRxmMeas20 != NULL)
    {
        free(handle->ubxRxmMeas20);
        handle->ubxRxmMeas20 = NULL;
    }
    if (handle->ubxRxmMeas50 != NULL)
    {
        free(handle->ubxRxmMeas50);
        handle->ubxRxmMeas50 = NULL;
    }
    if (handle->ubxRxmMeasc12 != NULL)
    {
        free(handle->ubxRxmMeasc12);
        handle->ubxRxmMeasc12 = NULL;
    }
    if (handle->ubxRxmMeasd12 != NULL)
    {
        free(handle->ubxRxmMeasd12);
        handle->ubxRxmMeasd12 = NULL;
    }
    if (handle->ubxRxmMeasx != NULL)
    {
        free(handle->ubxRxmMeasx);
        handle->ubxRxmMeasx = NULL;
    }
    if (handle->ubxRxmRlm != NULL)
    {
        free(handle->ubxRxmRlm);
        handle->ubxRxmRlm = NULL;
    }
    if (handle->ubxRxmSfrbx != NULL)
    {
        free(handle->ubxRxmSfrbx);
        handle->ubxRxmSfrbx = NULL;
    }
    if (handle->ubxSecUniqid != NULL)
    {
        free(handle->ubxSecUniqid);
        handle->ubxSecUniqid = NULL;
    }
    if (handle->ubxTimTm2 != NULL)
    {
        free(handle->ubxTimTm2);
        handle->ubxTimTm2 = NULL;
    }
    if (handle->ubxTimTp != NULL)
    {
        free(handle->ubxTimTp);
        handle->ubxTimTp = NULL;
    }
    if (handle->ubxTimVrfy != NULL)
    {
        free(handle->ubxTimVrfy);
        handle->ubxTimVrfy = NULL;
    }
    if (handle->ubxLogBatch != NULL)
    {
        for (uint16_t i = 0; i < 600; i++)
        {
            free(handle->ubxLogBatch[i]);
            handle->ubxLogBatch[i] = NULL;
        }

        free(handle->ubxLogBatch);
        handle->ubxLogBatch = NULL;
    }
    if (handle->ubxMonBatch != NULL)
    {
        free(handle->ubxMonBatch);
        handle->ubxMonBatch = NULL;
    }
    if (handle->ubxMonComms != NULL)
    {
        free(handle->ubxMonComms);
        handle->ubxMonComms = NULL;
    }
    if (handle->ubxMonGnss != NULL)
    {
        free(handle->ubxMonGnss);
        handle->ubxMonGnss = NULL;
    }
    if (handle->ubxMonHw3 != NULL)
    {
        free(handle->ubxMonHw3);
        handle->ubxMonHw3 = NULL;
    }
    if (handle->ubxMonPatch != NULL)
    {
        free(handle->ubxMonPatch);
        handle->ubxMonPatch = NULL;
    }
    if (handle->ubxMonRf != NULL)
    {
        free(handle->ubxMonRf);
        handle->ubxMonRf = NULL;
    }
    if (handle->ubxMonSpan != NULL)
    {
        free(handle->ubxMonSpan);
        handle->ubxMonSpan = NULL;
    }
    if (handle->ubxMonVer != NULL)
    {
        free(handle->ubxMonVer);
        handle->ubxMonVer = NULL;
    }

}







/****************************************************************************
 * Configuration Modifications
 ****************************************************************************/





/****************************************************************************
 * @brief Clear configuration through UBX-CFG-CFG message.
 * @param handle Handle for ublox gnss module.
 * @param mask Memory location to clear (0: BBR & Flash, 1: BBR, 2: Flash, 4: EEPROM, 8: SPI Flash).
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_ubx_cfg_clear(gnss_t *handle, uint8_t mask)
{

    uint8_t payload[13]; // Max set value
    uint8_t length = 12;

    memset(payload, 0x00, 13);

    payload[0] = 0x01; // Clear Mask

    if (mask > 0)
    {
        payload[12] = mask;
        length = 13;
    }
        
    if (gnss_ubx_msg(handle, (GNSS_UBX_CFG_CFG >> 8) & 0xFF, GNSS_UBX_CFG_CFG & 0xFF, length, payload, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send configuration clear via UBX-CFG-CFG");
        #endif
        return 1;
    }    

    return 0;
}

/****************************************************************************
 * @brief Save current configuration to non-volatile memory through UBX-CFG-CFG message.
 * @param handle Handle for ublox gnss module.
 * @param mask Memory location to save (0: BBR & Flash, 1: BBR, 2: Flash, 4: EEPROM, 8: SPI Flash).
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_ubx_cfg_save(gnss_t *handle, uint8_t mask)
{

    uint8_t payload[13]; // Max set value
    uint8_t length = 12;

    memset(payload, 0x00, 13);

    payload[4] = 0x01; // Save Mask

    if (mask > 0)
    {
        payload[12] = mask;
        length = 13;
    }
        
    if (gnss_ubx_msg(handle, (GNSS_UBX_CFG_CFG >> 8) & 0xFF, GNSS_UBX_CFG_CFG & 0xFF, length, payload, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send configuration save via UBX-CFG-CFG");
        #endif
        return 1;
    }    

    return 0;
}

/****************************************************************************
 * @brief Load configuration from non-volatile memory through UBX-CFG-CFG message.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_ubx_cfg_load(gnss_t *handle)
{

    uint8_t payload[12]; // Max set value

    memset(payload, 0x00, 12);

    payload[8] = 0x01; // Load Mask
        
    if (gnss_ubx_msg(handle, (GNSS_UBX_CFG_CFG >> 8) & 0xFF, GNSS_UBX_CFG_CFG & 0xFF, 12, payload, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send configuration load via UBX-CFG-CFG");
        #endif
        return 1;
    }    

    return 0;
}

/****************************************************************************
 * @brief Delete configuration key value pair via UBX-CFG-VALDEL.
 * @param handle Handle for ublox gnss module.
 * @param layer Memory location to delete configuration from (1: RAM, 2: BBR, 4: Flash).
 * @param key KeyID for CFG message.
 * @return 0: Success
 *  1: Failed to send message
 ****************************************************************************/
uint8_t gnss_cfg_del(gnss_t *handle, uint8_t layer, uint32_t key)
{

    uint8_t payload[32]; // Max set value

    payload[0] = 0x00; // Version
    payload[1] = layer; // Layers: 0x01 - RAM, 0x02 - BBR, 0x04 - Flash
    payload[2] = 0x00; // Reserved
    payload[3] = 0x00; // Reserved
    payload[4] = key & 0xFF; // Begin Configuration Data
    payload[5] = (key >> 8) & 0xFF;
    payload[6] = (key >> 16) & 0xFF;
    payload[7] = (key >> 24) & 0xFF;
        
    if (gnss_ubx_msg(handle, (GNSS_UBX_CFG_VALDEL >> 8) & 0xFF, GNSS_UBX_CFG_VALDEL & 0xFF, 8, payload, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] CFG Delete failed to write UBX Msg");
        #endif
        return 1;
    }    

    return 0;
}

/****************************************************************************
 * @brief Set configuration key value pair via UBX-CFG-VALSET.
 * @param handle Handle for ublox gnss module.
 * @param layer Memory location to set configuration (1: RAM, 2: BBR, 4: Flash).
 * @param key KeyID for CFG message.
 * @param value Value for CFG options (array of bytes, must be little-endian).
 * @param length Length of value (max 24).
 * @param verify If set, confirm that configuration was set correctly (default true)
 * @return 0: Success
 *  1: Failed to send message
 *  2: Failed to send VALGET message
 *  3: Received incorrect KeyID from VALGET
 *  4: Return from VALGET didn't match VALSET
 ****************************************************************************/
uint8_t gnss_cfg_set(gnss_t *handle, uint8_t layer, uint32_t key, uint8_t *value, uint8_t length, bool verify)
{

    uint8_t payload[32]; // Max set value

    payload[0] = 0x00; // Version
    payload[1] = layer; // Layers: 0x01 - RAM, 0x02 - BBR, 0x04 - Flash
    payload[2] = 0x00; // Reserved
    payload[3] = 0x00; // Reserved
    payload[4] = key & 0xFF; // Begin Configuration Data
    payload[5] = (key >> 8) & 0xFF;
    payload[6] = (key >> 16) & 0xFF;
    payload[7] = (key >> 24) & 0xFF;

    memcpy(payload + 8, value, length);
        
    if (gnss_ubx_msg(handle, (GNSS_UBX_CFG_VALSET >> 8) & 0xFF, GNSS_UBX_CFG_VALSET & 0xFF, length + 8, payload, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] CFG Set failed to write UBX Msg");
        #endif
        return 1;
    }

    if (verify)
    {
        gnss_cfg_msg_t message;

        // Verify each applied layer is correct
        if (layer & GNSS_RAM)
        {

            if (gnss_cfg_get(handle, &message, key, 0))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] CFG Set failed to send VALGET check for RAM");
                #endif
                return 2;
            }

            if (key != (((uint32_t)message.data[0] & 0xFF) | (((uint32_t)message.data[1] & 0xFF) << 8) | (((uint32_t)message.data[2] & 0xFF) << 16) | (((uint32_t)message.data[3] & 0xFF) << 24)))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Received incorrect keyID for VALGET check for RAM");
                #endif
                return 3;
            }

            for (uint8_t i = 0; i < length; i++)
            {
                if (value[i] != message.data[4 + i])
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] VALGET return did not match VALSET CFG for RAM");
                    #endif
                    return 4;
                }
            }

        }

        if (layer & GNSS_BBR)
        {

            if (gnss_cfg_get(handle, &message, key, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] CFG Set failed to send VALGET check for BBR");
                #endif
                return 2;
            }

            if (key != (((uint32_t)message.data[0] & 0xFF) | (((uint32_t)message.data[1] & 0xFF) << 8) | (((uint32_t)message.data[2] & 0xFF) << 16) | (((uint32_t)message.data[3] & 0xFF) << 24)))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Received incorrect keyID for VALGET check for BBR");
                #endif
                return 3;
            }

            for (uint8_t i = 0; i < length; i++)
            {
                if (value[i] != message.data[4 + i])
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] VALGET return did not match VALSET CFG for BBR");
                    #endif
                    return 4;
                }
            }

        }

        if (layer & GNSS_FLASH)
        {

            if (gnss_cfg_get(handle, &message, key, 2))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] CFG Set failed to send VALGET check for FLASH");
                #endif
                return 2;
            }

            if (key != (((uint32_t)message.data[0] & 0xFF) | (((uint32_t)message.data[1] & 0xFF) << 8) | (((uint32_t)message.data[2] & 0xFF) << 16) | (((uint32_t)message.data[3] & 0xFF) << 24)))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Received incorrect keyID for VALGET check for FLASH");
                #endif
                return 3;
            }

            for (uint8_t i = 0; i < length; i++)
            {
                if (value[i] != message.data[4 + i])
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] VALGET return did not match VALSET CFG for FLASH");
                    #endif
                    return 4;
                }
            }

        }
    }
    
    #ifdef DEBUG
    Serial.println("[DEBUG] VALSET Success!");
    #endif

    return 0;
}

/****************************************************************************
 * @brief Request CFG Value.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @param keyID Key of CFG register to retrieve.
 * @param layer Data location to retrieve from. 0: RAM, 1: BBR, 2: Flash, 7: Default.
 * @return 0: Success
 * 1: Unable to send poll request
 * 2: Failed to allocate memory for configuration message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_cfg_get(gnss_t *handle, gnss_cfg_msg_t *message, uint32_t keyID, uint8_t layer)
{

    uint8_t data[8];
    data[0] = 0x00; // Version
    data[1] = layer; // Layers: 0x01 - RAM, 0x02 - BBR, 0x04 - Flash
    data[2] = 0x00; // Position (leave blank)
    data[3] = 0x00; // Position (leave blank)
    data[4] = keyID & 0xFF;
    data[5] = (keyID >> 8) & 0xFF;
    data[6] = (keyID >> 16) & 0xFF;
    data[7] = (keyID >> 24) & 0xFF;


    if (handle->cfgMsg == NULL)
    {
        gnss_cfg_msg_t *ptr = (gnss_cfg_msg_t *)malloc(sizeof(gnss_cfg_msg_t));
        
        if (ptr == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-CFG-VALGET message");
            #endif
            return 2;
        }

        handle->cfgMsg = ptr;
    }

    handle->cfgMsg->version = 0x00;
    
    timer_handle_t gen_timer;
    timer_init(&gen_timer, 1000000); // 1000ms
    
    if(gnss_ubx_msg(handle, (GNSS_UBX_CFG_VALGET >> 8) & 0xFF, GNSS_UBX_CFG_VALGET & 0xFF, 8, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-CFG-VALGET Request");
        #endif
        return 1;
    }
    
    timer_start(&gen_timer);

    // Wait till received response
    while (handle->cfgMsg->version == 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] UBX-CFG-VALGET poll request timed out");
            #endif
            return 3;
        }
        
        gnss_rec_and_parse(handle);

    }

    *message = *(handle->cfgMsg);
    
    return 0;

}

/****************************************************************************
 * @brief Send NMEA-PUBX-CONFIG message.
 * @param handle Handle for ublox gnss module.
 * @param portID ID of communication port.
 * @param inputMask Input protocol bit mask.
 * @param outputMask Output protocol bit mask.
 * @param baud Baud Rate.
 * @return 0: Success
 *  1: Failed to send message
 * @note Untested!
 ****************************************************************************/
uint8_t gnss_pubx_set_config(gnss_t *handle, uint8_t portID, uint16_t inputMask, uint16_t outputMask, uint32_t baud)
{

    uint8_t data[32];

    data[0] = '4'; // Message Identifier
    data[1] = '1';
    data[2] = ',';

    data[3] = portID + '0';

    data[4] = ',';

    uint8_t workingData;

    for (uint8_t i = 0; i < 4; i++)
    {
        workingData = (inputMask >> (12 - (i * 4))) & 0x0F;

        // Convert to ASCII
        if (workingData > 9)
            data[i + 5] = workingData - 10 + 'A';
        else
            data[i + 5] = workingData + '0';
    }

    data[9] = ',';

    for (uint8_t i = 0; i < 4; i++)
    {
        workingData = (outputMask >> (12 - (i * 4))) & 0x0F;

        // Convert to ASCII
        if (workingData > 9)
            data[i + 10] = workingData - 10 + 'A';
        else
            data[i + 10] = workingData + '0';
    }

    data[14] = ',';

    uint8_t indexAdd = 0;

    if ((baud == 0) || (baud > 10000000)) // Constrain minimum and maximum baud rates
        data[15] = '0';
    else
    {
        for (uint8_t i = 0; i < 7; i++)
        {
            workingData = (baud / 1000000 * (uint32_t)pow(10, i)) % (uint32_t)pow(10, i);

            if ((workingData > 0) || (indexAdd > 0))
            {
                data[15 + indexAdd] = workingData + '0';
                indexAdd++;
            }
        }
    }

    data[16 + indexAdd] = ',';

    data[17 + indexAdd] = '0'; // Default disable autobauding

    // Send out poll request
    if (gnss_pubx_msg(handle, data, 18 + indexAdd))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send PUBX-CONFIG message.");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Set NMEA-Standard message rates. This is an alternative method of updating the messaging rate of NMEA messages (as opposed to CFG messages).
 * This method only allows output every epoch or disabled.
 * @note Untested!
 * @param handle Handle for ublox gnss module.
 * @param msg NMEA message to adjust rate for.
 * @param ddc Output rate on DDC (I2C).
 * @param uart1 Output rate on UART1.
 * @param uart2 Output rate on UART2.
 * @param usb Output rate on USB.
 * @param spi Output rate on SPI
 * @return 0: Success
 * 1: Failed to send message
 * 2: Failed to recognize NMEA message requested
 ****************************************************************************/
uint8_t gnss_pubx_set_msg_rate(gnss_t *handle, uint16_t msg, bool ddc, bool uart1, bool uart2, bool usb, bool spi)
{

    uint8_t data[18];

    data[0] = '4'; // Message Identifier
    data[1] = '0';
    data[2] = ',';

    switch (msg)
    {
        case GNSS_NMEA_STANDARD_DTM:
            data[3] = 'D';
            data[4] = 'T';
            data[5] = 'M';
            break;
        case GNSS_NMEA_STANDARD_GBS:
            data[3] = 'G';
            data[4] = 'B';
            data[5] = 'S';
            break;
        case GNSS_NMEA_STANDARD_GGA:
            data[3] = 'G';
            data[4] = 'G';
            data[5] = 'A';
            break;
        case GNSS_NMEA_STANDARD_GLL:
            data[3] = 'G';
            data[4] = 'L';
            data[5] = 'L';
            break;
        case GNSS_NMEA_STANDARD_GNS:
            data[3] = 'G';
            data[4] = 'N';
            data[5] = 'S';
            break;
        case GNSS_NMEA_STANDARD_GRS:
            data[3] = 'G';
            data[4] = 'R';
            data[5] = 'S';
            break;
        case GNSS_NMEA_STANDARD_GSA:
            data[3] = 'G';
            data[4] = 'S';
            data[5] = 'A';
            break;
        case GNSS_NMEA_STANDARD_GST:
            data[3] = 'G';
            data[4] = 'S';
            data[5] = 'T';
            break;
        case GNSS_NMEA_STANDARD_GSV:
            data[3] = 'G';
            data[4] = 'S';
            data[5] = 'V';
            break;
        case GNSS_NMEA_STANDARD_RLM:
            data[3] = 'R';
            data[4] = 'L';
            data[5] = 'M';
            break;
        case GNSS_NMEA_STANDARD_RMC:
            data[3] = 'R';
            data[4] = 'M';
            data[5] = 'C';
            break;
        case GNSS_NMEA_STANDARD_VLW:
            data[3] = 'V';
            data[4] = 'L';
            data[5] = 'W';
            break;
        case GNSS_NMEA_STANDARD_VTG:
            data[3] = 'V';
            data[4] = 'T';
            data[5] = 'G';
            break;
        case GNSS_NMEA_STANDARD_ZDA:
            data[3] = 'Z';
            data[4] = 'D';
            data[5] = 'A';
            break;
        default:
            #ifdef DEBUG
            Serial.println("[DEBUG] PUBX-RATE Message not recognized.");
            #endif
            return 2;
            break;
    }

    data[6] = ',';

    if (ddc)
        data[7] = '1';
    else
        data[7] = '0';

    data[8] = ',';

    if (uart1)
        data[9] = '1';
    else
        data[9] = '0';

    data[10] = ',';

    if (uart2)
        data[11] = '1';
    else
        data[11] = '0';

    data[12] = ',';

    if (usb)
        data[13] = '1';
    else
        data[13] = '0';

    data[14] = ',';

    if (spi)
        data[15] = '1';
    else
        data[15] = '0';

    data[16] = ',';
    data[17] = '0';

    // Send out poll request
    if (gnss_pubx_msg(handle, data, 18))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send PUBX-RATE message.");
        #endif
        return 1;
    }

    return 0;

}





/****************************************************************************
 * Configuration Specifics
 ****************************************************************************/





/****************************************************************************
 * @brief Set message to report periodically.
 * @param handle Handle for ublox gnss module.
 * @param msg 16bit combined Message ((Class << 8) | ID) to be called periodically.
 * @param period Epochs per message (e.g. 1 = every epoch, 2 = every other epoch).
 * @param peripheral Channel to output message, 0: I2C, 1: UART, 2: SPI.
 * @return 0: Success
 * 1: Failed to set periodic messaging
 * 2: Message not able to be set to periodic (or does not exist)
 * 3: Failed to allocate memory for temp buffer
 * 4: Unrecognized peripheral
 ****************************************************************************/
uint8_t gnss_set_msg_auto(gnss_t *handle, uint16_t msg, uint8_t period, uint8_t peripheral)
{

    /* Peripheral
    0: I2C
    1: UART
    2: SPI
    */

    uint32_t key;

    switch (msg)
    {
        case GNSS_NMEA_STANDARD_DTM:
            // Check if memory already allocated
            if ((handle->nmeaStdDtm == NULL) && (period != 0))
            {
                handle->nmeaStdDtm = (gnss_nmea_std_dtm_t *)malloc(sizeof(gnss_nmea_std_dtm_t));
                
                if (handle->nmeaStdDtm == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-DTM message");
                    #endif
                    return 3;
                }

                 handle->nmeaStdDtm->periodic = true;

            }
            else if ((handle->nmeaStdDtm != NULL) && (period == 0))
                handle->nmeaStdDtm->periodic = false;
            else if ((handle->nmeaStdDtm != NULL) && (period != 0))
                handle->nmeaStdDtm->periodic = true;

            // Select correct CFG Key based on peripheral
            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_DTM_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_DTM_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_DTM_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GBS:
            if ((handle->nmeaStdGbs == NULL) && (period != 0))
            {
                handle->nmeaStdGbs = (gnss_nmea_std_gbs_t *)malloc(sizeof(gnss_nmea_std_gbs_t));
                
                if (handle->nmeaStdGbs == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GBS message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGbs->periodic = true;

            }
            else if ((handle->nmeaStdGbs != NULL) && (period == 0))
                handle->nmeaStdGbs->periodic = false;
            else if ((handle->nmeaStdGbs != NULL) && (period != 0))
                handle->nmeaStdGbs->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GBS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GBS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GBS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GGA:
            if ((handle->nmeaStdGga == NULL) && (period != 0))
            {
                handle->nmeaStdGga = (gnss_nmea_std_gga_t *)malloc(sizeof(gnss_nmea_std_gga_t));
                
                if (handle->nmeaStdGga == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GGA message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGga->periodic = true;

            }
            else if ((handle->nmeaStdGga != NULL) && (period == 0))
                handle->nmeaStdGga->periodic = false;
            else if ((handle->nmeaStdGga != NULL) && (period != 0))
                handle->nmeaStdGga->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GGA_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GGA_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GGA_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GLL:
            if ((handle->nmeaStdGll == NULL) && (period != 0))
            {
                handle->nmeaStdGll = (gnss_nmea_std_gll_t *)malloc(sizeof(gnss_nmea_std_gll_t));
                
                if (handle->nmeaStdGll == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GLL message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGll->periodic = true;

            }
            else if ((handle->nmeaStdGll != NULL) && (period == 0))
                handle->nmeaStdGll->periodic = false;
            else if ((handle->nmeaStdGll != NULL) && (period != 0))
                handle->nmeaStdGll->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GLL_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GLL_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GLL_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GNS:
            if ((handle->nmeaStdGns == NULL) && (period != 0))
            {
                handle->nmeaStdGns = (gnss_nmea_std_gns_t *)malloc(sizeof(gnss_nmea_std_gns_t));
                
                if (handle->nmeaStdGns == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GNS message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGns->periodic = true;

            }
            else if ((handle->nmeaStdGns != NULL) && (period == 0))
                handle->nmeaStdGns->periodic = false;
            else if ((handle->nmeaStdGns != NULL) && (period != 0))
                handle->nmeaStdGns->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GNS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GNS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GNS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GRS:
            if ((handle->nmeaStdGrs == NULL) && (period != 0))
            {
                handle->nmeaStdGrs = (gnss_nmea_std_grs_t *)malloc(sizeof(gnss_nmea_std_grs_t));
                
                if (handle->nmeaStdGrs == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GRS message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGrs->periodic = true;

                // Must initialize array of pointers
                for (uint8_t i = 0; i < 8; i++)
                    handle->nmeaStdGrs->grs[i] = NULL;

            }
            else if ((handle->nmeaStdGrs != NULL) && (period == 0))
                handle->nmeaStdGrs->periodic = false;
            else if ((handle->nmeaStdGrs != NULL) && (period != 0))
                handle->nmeaStdGrs->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GRS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GRS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GRS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GSA:
            if ((handle->nmeaStdGsa == NULL) && (period != 0))
            {
                handle->nmeaStdGsa = (gnss_nmea_std_gsa_t *)malloc(sizeof(gnss_nmea_std_gsa_t));
                
                if (handle->nmeaStdGsa == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GSA message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGsa->periodic = true;

                // Must initialize array of pointers
                for (uint8_t i = 0; i < 8; i++)
                    handle->nmeaStdGsa->gsa[i] = NULL;

            }
            else if ((handle->nmeaStdGsa != NULL) && (period == 0))
                handle->nmeaStdGsa->periodic = false;
            else if ((handle->nmeaStdGsa != NULL) && (period != 0))
                handle->nmeaStdGsa->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GSA_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GSA_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GSA_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GST:
            if ((handle->nmeaStdGst == NULL) && (period != 0))
            {
                handle->nmeaStdGst = (gnss_nmea_std_gst_t *)malloc(sizeof(gnss_nmea_std_gst_t));
                
                if (handle->nmeaStdGst == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GST message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGst->periodic = true;

            }
            else if ((handle->nmeaStdGst != NULL) && (period == 0))
                handle->nmeaStdGst->periodic = false;
            else if ((handle->nmeaStdGst != NULL) && (period != 0))
                handle->nmeaStdGst->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GST_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GST_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GST_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_GSV:
            if ((handle->nmeaStdGsv == NULL) && (period != 0))
            {
                handle->nmeaStdGsv = (gnss_nmea_std_gsv_t *)malloc(sizeof(gnss_nmea_std_gsv_t));
                
                if (handle->nmeaStdGsv == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-GSV message");
                    #endif
                    return 3;
                }

                handle->nmeaStdGsv->periodic = true;

            }
            else if ((handle->nmeaStdGsv != NULL) && (period == 0))
                handle->nmeaStdGsv->periodic = false;
            else if ((handle->nmeaStdGsv != NULL) && (period != 0))
                handle->nmeaStdGsv->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GSV_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GSV_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_GSV_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_RMC:
            if ((handle->nmeaStdRmc == NULL) && (period != 0))
            {
                handle->nmeaStdRmc = (gnss_nmea_std_rmc_t *)malloc(sizeof(gnss_nmea_std_rmc_t));
                
                if (handle->nmeaStdRmc == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-RMC message");
                    #endif
                    return 3;
                }

                handle->nmeaStdRmc->periodic = true;

            }
            else if ((handle->nmeaStdRmc != NULL) && (period == 0))
                handle->nmeaStdRmc->periodic = false;
            else if ((handle->nmeaStdRmc != NULL) && (period != 0))
                handle->nmeaStdRmc->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_RMC_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_RMC_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_RMC_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_VLW:
            if ((handle->nmeaStdVlw == NULL) && (period != 0))
            {
                handle->nmeaStdVlw = (gnss_nmea_std_vlw_t *)malloc(sizeof(gnss_nmea_std_vlw_t));
                
                if (handle->nmeaStdVlw == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-VLW message");
                    #endif
                    return 3;
                }

                handle->nmeaStdVlw->periodic = true;

            }
            else if ((handle->nmeaStdVlw != NULL) && (period == 0))
                handle->nmeaStdVlw->periodic = false;
            else if ((handle->nmeaStdVlw != NULL) && (period != 0))
                handle->nmeaStdVlw->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_VLW_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_VLW_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_VLW_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_VTG:
            if ((handle->nmeaStdVtg == NULL) && (period != 0))
            {
                handle->nmeaStdVtg = (gnss_nmea_std_vtg_t *)malloc(sizeof(gnss_nmea_std_vtg_t));
                
                if (handle->nmeaStdVtg == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-VTG message");
                    #endif
                    return 3;
                }

                handle->nmeaStdVtg->periodic = true;

            }
            else if ((handle->nmeaStdVtg != NULL) && (period == 0))
                handle->nmeaStdVtg->periodic = false;
            else if ((handle->nmeaStdVtg != NULL) && (period != 0))
                handle->nmeaStdVtg->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_VTG_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_VTG_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_VTG_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_STANDARD_ZDA:
            if ((handle->nmeaStdZda == NULL) && (period != 0))
            {
                handle->nmeaStdZda = (gnss_nmea_std_zda_t *)malloc(sizeof(gnss_nmea_std_zda_t));
                
                if (handle->nmeaStdZda == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-STANDARD-ZDA message");
                    #endif
                    return 3;
                }

                handle->nmeaStdZda->periodic = true;

            }
            else if ((handle->nmeaStdZda != NULL) && (period == 0))
                handle->nmeaStdZda->periodic = false;
            else if ((handle->nmeaStdZda != NULL) && (period != 0))
                handle->nmeaStdZda->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_NMEA_ID_ZDA_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_NMEA_ID_ZDA_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_NMEA_ID_ZDA_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_PUBX_POSITION:
            if ((handle->nmeaPubxPos == NULL) && (period != 0))
            {
                handle->nmeaPubxPos = (gnss_nmea_pubx_pos_t *)malloc(sizeof(gnss_nmea_pubx_pos_t));
                
                if (handle->nmeaPubxPos == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-POSITION message");
                    #endif
                    return 3;
                }

                handle->nmeaPubxPos->periodic = true;

            }
            else if ((handle->nmeaPubxPos != NULL) && (period == 0))
                handle->nmeaPubxPos->periodic = false;
            else if ((handle->nmeaPubxPos != NULL) && (period != 0))
                handle->nmeaPubxPos->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYP_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYP_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYP_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_PUBX_SVSTATUS:
            if ((handle->nmeaPubxSvstatus == NULL) && (period != 0))
            {
                handle->nmeaPubxSvstatus = (gnss_nmea_pubx_svstatus_t *)malloc(sizeof(gnss_nmea_pubx_svstatus_t));
                
                if (handle->nmeaPubxSvstatus == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-SVSTATUS message");
                    #endif
                    return 3;
                }

                handle->nmeaPubxSvstatus->periodic = true;

            }
            else if ((handle->nmeaPubxSvstatus != NULL) && (period == 0))
                handle->nmeaPubxSvstatus->periodic = false;
            else if ((handle->nmeaPubxSvstatus != NULL) && (period != 0))
                handle->nmeaPubxSvstatus->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_NMEA_PUBX_TIME:
            if ((handle->nmeaPubxTime == NULL) && (period != 0))
            {
                handle->nmeaPubxTime = (gnss_nmea_pubx_time_t *)malloc(sizeof(gnss_nmea_pubx_time_t));
                
                if (handle->nmeaPubxTime == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-TIME message");
                    #endif
                    return 3;
                }

                handle->nmeaPubxTime->periodic = true;

            }
            else if ((handle->nmeaPubxTime != NULL) && (period == 0))
                handle->nmeaPubxTime->periodic = false;
            else if ((handle->nmeaPubxTime != NULL) && (period != 0))
                handle->nmeaPubxTime->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYT_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYT_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_PUBX_ID_POLYT_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_AOPSTATUS:
            if ((handle->ubxNavAopstatus == NULL) && (period != 0))
            {
                handle->ubxNavAopstatus = (gnss_ubx_nav_aopstatus_t *)malloc(sizeof(gnss_ubx_nav_aopstatus_t));
                
                if (handle->ubxNavAopstatus == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-AOPSTATUS message");
                    #endif
                    return 3;
                }

                handle->ubxNavAopstatus->periodic = true;

            }
            else if ((handle->ubxNavAopstatus != NULL) && (period == 0))
                handle->ubxNavAopstatus->periodic = false;
            else if ((handle->ubxNavAopstatus != NULL) && (period != 0))
                handle->ubxNavAopstatus->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_AOPSTATUS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_AOPSTATUS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_AOPSTATUS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_CLOCK:
            if ((handle->ubxNavClock == NULL) && (period != 0))
            {
                handle->ubxNavClock = (gnss_ubx_nav_clock_t *)malloc(sizeof(gnss_ubx_nav_clock_t));
                
                if (handle->ubxNavClock == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-CLOCK message");
                    #endif
                    return 3;
                }

                handle->ubxNavClock->periodic = true;

            }
            else if ((handle->ubxNavClock != NULL) && (period == 0))
                handle->ubxNavClock->periodic = false;
            else if ((handle->ubxNavClock != NULL) && (period != 0))
                handle->ubxNavClock->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_CLOCK_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_CLOCK_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_CLOCK_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_COV:
            if ((handle->ubxNavCov == NULL) && (period != 0))
            {
                handle->ubxNavCov = (gnss_ubx_nav_cov_t *)malloc(sizeof(gnss_ubx_nav_cov_t));
                
                if (handle->ubxNavCov == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-COV message");
                    #endif
                    return 3;
                }

                handle->ubxNavCov->periodic = true;

            }
            else if ((handle->ubxNavCov != NULL) && (period == 0))
                handle->ubxNavCov->periodic = false;
            else if ((handle->ubxNavCov != NULL) && (period != 0))
                handle->ubxNavCov->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_COV_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_COV_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_COV_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_DOP:
            if ((handle->ubxNavDop == NULL) && (period != 0))
            {
                handle->ubxNavDop = (gnss_ubx_nav_dop_t *)malloc(sizeof(gnss_ubx_nav_dop_t));
                
                if (handle->ubxNavDop == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-DOP message");
                    #endif
                    return 3;
                }

                handle->ubxNavDop->periodic = true;

            }
            else if ((handle->ubxNavDop != NULL) && (period == 0))
                handle->ubxNavDop->periodic = false;
            else if ((handle->ubxNavDop != NULL) && (period != 0))
                handle->ubxNavDop->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_DOP_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_DOP_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_DOP_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_EOE:
            if ((handle->ubxNavEoe == NULL) && (period != 0))
            {
                handle->ubxNavEoe = (gnss_ubx_nav_eoe_t *)malloc(sizeof(gnss_ubx_nav_eoe_t));
                
                if (handle->ubxNavEoe == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-EOE message");
                    #endif
                    return 3;
                }

                handle->ubxNavEoe->periodic = true;

            }
            else if ((handle->ubxNavEoe != NULL) && (period == 0))
                handle->ubxNavEoe->periodic = false;
            else if ((handle->ubxNavEoe != NULL) && (period != 0))
                handle->ubxNavEoe->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_EOE_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_EOE_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_EOE_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_ODO:
            if ((handle->ubxNavOdo == NULL) && (period != 0))
            {
                handle->ubxNavOdo = (gnss_ubx_nav_odo_t *)malloc(sizeof(gnss_ubx_nav_odo_t));
                
                if (handle->ubxNavOdo == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-ODO message");
                    #endif
                    return 3;
                }

                handle->ubxNavOdo->periodic = true;

            }
            else if ((handle->ubxNavOdo != NULL) && (period == 0))
                handle->ubxNavOdo->periodic = false;
            else if ((handle->ubxNavOdo != NULL) && (period != 0))
                handle->ubxNavOdo->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_ODO_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_ODO_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_ODO_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_ORB:
            if ((handle->ubxNavOrb == NULL) && (period != 0))
            {
                handle->ubxNavOrb = (gnss_ubx_nav_orb_t *)malloc(sizeof(gnss_ubx_nav_orb_t));
                
                if (handle->ubxNavOrb == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-ORB message");
                    #endif
                    return 3;
                }

                handle->ubxNavOrb->periodic = true;

            }
            else if ((handle->ubxNavOrb != NULL) && (period == 0))
                handle->ubxNavOrb->periodic = false;
            else if ((handle->ubxNavOrb != NULL) && (period != 0))
                handle->ubxNavOrb->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_ORB_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_ORB_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_ORB_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_PL:
            if ((handle->ubxNavPl == NULL) && (period != 0))
            {
                handle->ubxNavPl = (gnss_ubx_nav_pl_t *)malloc(sizeof(gnss_ubx_nav_pl_t));
                
                if (handle->ubxNavPl == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-PL message");
                    #endif
                    return 3;
                }

                handle->ubxNavPl->periodic = true;

            }
            else if ((handle->ubxNavPl != NULL) && (period == 0))
                handle->ubxNavPl->periodic = false;
            else if ((handle->ubxNavPl != NULL) && (period != 0))
                handle->ubxNavPl->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_PL_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_PL_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_PL_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_POSECEF:
            if ((handle->ubxNavPosecef == NULL) && (period != 0))
            {
                handle->ubxNavPosecef = (gnss_ubx_nav_posecef_t *)malloc(sizeof(gnss_ubx_nav_posecef_t));
                
                if (handle->ubxNavPosecef == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-POSECEF message");
                    #endif
                    return 3;
                }

                handle->ubxNavPosecef->periodic = true;

            }
            else if ((handle->ubxNavPosecef != NULL) && (period == 0))
                handle->ubxNavPosecef->periodic = false;
            else if ((handle->ubxNavPosecef != NULL) && (period != 0))
                handle->ubxNavPosecef->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_POSECEF_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_POSECEF_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_POSECEF_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_POSLLH:
            if ((handle->ubxNavPosllh == NULL) && (period != 0))
            {
                handle->ubxNavPosllh = (gnss_ubx_nav_posllh_t *)malloc(sizeof(gnss_ubx_nav_posllh_t));
                
                if (handle->ubxNavPosllh == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-POSLLH message");
                    #endif
                    return 3;
                }

                handle->ubxNavPosllh->periodic = true;

            }
            else if ((handle->ubxNavPosllh != NULL) && (period == 0))
                handle->ubxNavPosllh->periodic = false;
            else if ((handle->ubxNavPosllh != NULL) && (period != 0))
                handle->ubxNavPosllh->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_POSLLH_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_POSLLH_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_POSLLH_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_PVT:
            if ((handle->ubxNavPvt == NULL) && (period != 0))
            {
                handle->ubxNavPvt = (gnss_ubx_nav_pvt_t *)malloc(sizeof(gnss_ubx_nav_pvt_t));
                
                if (handle->ubxNavPvt == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-PVT message");
                    #endif
                    return 3;
                }

                 handle->ubxNavPvt->periodic = true;

            }
            else if ((handle->ubxNavPvt != NULL) && (period == 0))
                handle->ubxNavPvt->periodic = false;
            else if ((handle->ubxNavPvt != NULL) && (period != 0))
                handle->ubxNavPvt->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_PVT_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_PVT_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_PVT_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_SAT:
            if ((handle->ubxNavSat == NULL) && (period != 0))
            {
                handle->ubxNavSat = (gnss_ubx_nav_sat_t *)malloc(sizeof(gnss_ubx_nav_sat_t));
                
                if (handle->ubxNavSat == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SAT message");
                    #endif
                    return 3;
                }

                handle->ubxNavSat->periodic = true;

            }
            else if ((handle->ubxNavSat != NULL) && (period == 0))
                handle->ubxNavSat->periodic = false;
            else if ((handle->ubxNavSat != NULL) && (period != 0))
                handle->ubxNavSat->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SAT_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SAT_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SAT_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_SBAS:
            if ((handle->ubxNavSbas == NULL) && (period != 0))
            {
                handle->ubxNavSbas = (gnss_ubx_nav_sbas_t *)malloc(sizeof(gnss_ubx_nav_sbas_t));
                
                if (handle->ubxNavSbas == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SBAS message");
                    #endif
                    return 3;
                }

                handle->ubxNavSbas->periodic = true;

            }
            else if ((handle->ubxNavSbas != NULL) && (period == 0))
                handle->ubxNavSbas->periodic = false;
            else if ((handle->ubxNavSbas != NULL) && (period != 0))
                handle->ubxNavSbas->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SBAS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SBAS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SBAS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_SIG:
            if ((handle->ubxNavSig == NULL) && (period != 0))
            {
                handle->ubxNavSig = (gnss_ubx_nav_sig_t *)malloc(sizeof(gnss_ubx_nav_sig_t));
                
                if (handle->ubxNavSig == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SIG message");
                    #endif
                    return 3;
                }

                handle->ubxNavSig->periodic = true;

            }
            else if ((handle->ubxNavSig != NULL) && (period == 0))
                handle->ubxNavSig->periodic = false;
            else if ((handle->ubxNavSig != NULL) && (period != 0))
                handle->ubxNavSig->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SIG_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SIG_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SIG_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_SLAS:
            if ((handle->ubxNavSlas == NULL) && (period != 0))
            {
                handle->ubxNavSlas = (gnss_ubx_nav_slas_t *)malloc(sizeof(gnss_ubx_nav_slas_t));
                
                if (handle->ubxNavSlas == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SLAS message");
                    #endif
                    return 3;
                }

                handle->ubxNavSlas->periodic = true;

            }
            else if ((handle->ubxNavSlas != NULL) && (period == 0))
                handle->ubxNavSlas->periodic = false;
            else if ((handle->ubxNavSlas != NULL) && (period != 0))
                handle->ubxNavSlas->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SLAS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SLAS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_SLAS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_STATUS:
            if ((handle->ubxNavStatus == NULL) && (period != 0))
            {
                handle->ubxNavStatus = (gnss_ubx_nav_status_t *)malloc(sizeof(gnss_ubx_nav_status_t));
                
                if (handle->ubxNavStatus == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-STATUS message");
                    #endif
                    return 3;
                }

                handle->ubxNavStatus->periodic = true;

            }
            else if ((handle->ubxNavStatus != NULL) && (period == 0))
                handle->ubxNavStatus->periodic = false;
            else if ((handle->ubxNavStatus != NULL) && (period != 0))
                handle->ubxNavStatus->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_STATUS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_STATUS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_STATUS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMEBDS:
            if ((handle->ubxNavTimebds == NULL) && (period != 0))
            {
                handle->ubxNavTimebds = (gnss_ubx_nav_timebds_t *)malloc(sizeof(gnss_ubx_nav_timebds_t));
                
                if (handle->ubxNavTimebds == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEBDS message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimebds->periodic = true;

            }
            else if ((handle->ubxNavTimebds != NULL) && (period == 0))
                handle->ubxNavTimebds->periodic = false;
            else if ((handle->ubxNavTimebds != NULL) && (period != 0))
                handle->ubxNavTimebds->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEBDS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEBDS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEBDS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMEGAL:
            if ((handle->ubxNavTimegal == NULL) && (period != 0))
            {
                handle->ubxNavTimegal = (gnss_ubx_nav_timegal_t *)malloc(sizeof(gnss_ubx_nav_timegal_t));
                
                if (handle->ubxNavTimegal == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEGAL message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimegal->periodic = true;

            }
            else if ((handle->ubxNavTimegal != NULL) && (period == 0))
                handle->ubxNavTimegal->periodic = false;
            else if ((handle->ubxNavTimegal != NULL) && (period != 0))
                handle->ubxNavTimegal->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGAL_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGAL_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGAL_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMEGLO:
            if ((handle->ubxNavTimeglo == NULL) && (period != 0))
            {
                handle->ubxNavTimeglo = (gnss_ubx_nav_timeglo_t *)malloc(sizeof(gnss_ubx_nav_timeglo_t));
                
                if (handle->ubxNavTimeglo == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEGLO message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimeglo->periodic = true;

            }
            else if ((handle->ubxNavTimeglo != NULL) && (period == 0))
                handle->ubxNavTimeglo->periodic = false;
            else if ((handle->ubxNavTimeglo != NULL) && (period != 0))
                handle->ubxNavTimeglo->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGLO_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGLO_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGLO_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMEGPS:
            if ((handle->ubxNavTimegps == NULL) && (period != 0))
            {
                handle->ubxNavTimegps = (gnss_ubx_nav_timegps_t *)malloc(sizeof(gnss_ubx_nav_timegps_t));
                
                if (handle->ubxNavTimegps == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEGPS message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimegps->periodic = true;

            }
            else if ((handle->ubxNavTimegps != NULL) && (period == 0))
                handle->ubxNavTimegps->periodic = false;
            else if ((handle->ubxNavTimegps != NULL) && (period != 0))
                handle->ubxNavTimegps->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGPS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEGPS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMELS:
            if ((handle->ubxNavTimels == NULL) && (period != 0))
            {
                handle->ubxNavTimels = (gnss_ubx_nav_timels_t *)malloc(sizeof(gnss_ubx_nav_timels_t));
                
                if (handle->ubxNavTimels == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMELS message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimels->periodic = true;

            }
            else if ((handle->ubxNavTimels != NULL) && (period == 0))
                handle->ubxNavTimels->periodic = false;
            else if ((handle->ubxNavTimels != NULL) && (period != 0))
                handle->ubxNavTimels->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMELS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMELS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMELS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMENAVIC:
            if ((handle->ubxNavTimenavic == NULL) && (period != 0))
            {
                handle->ubxNavTimenavic = (gnss_ubx_nav_timenavic_t *)malloc(sizeof(gnss_ubx_nav_timenavic_t));
                
                if (handle->ubxNavTimenavic == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMENAVIC message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimenavic->periodic = true;

            }
            else if ((handle->ubxNavTimenavic != NULL) && (period == 0))
                handle->ubxNavTimenavic->periodic = false;
            else if ((handle->ubxNavTimenavic != NULL) && (period != 0))
                handle->ubxNavTimenavic->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMENAVIC_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMENAVIC_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMENAVIC_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMEQZSS:
            if ((handle->ubxNavTimeqzss == NULL) && (period != 0))
            {
                handle->ubxNavTimeqzss = (gnss_ubx_nav_timeqzss_t *)malloc(sizeof(gnss_ubx_nav_timeqzss_t));
                
                if (handle->ubxNavTimeqzss == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEQZSS message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimeqzss->periodic = true;

            }
            else if ((handle->ubxNavTimeqzss != NULL) && (period == 0))
                handle->ubxNavTimeqzss->periodic = false;
            else if ((handle->ubxNavTimeqzss != NULL) && (period != 0))
                handle->ubxNavTimeqzss->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEQZSS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEQZSS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_TIMEUTC:
            if ((handle->ubxNavTimeutc == NULL) && (period != 0))
            {
                handle->ubxNavTimeutc = (gnss_ubx_nav_timeutc_t *)malloc(sizeof(gnss_ubx_nav_timeutc_t));
                
                if (handle->ubxNavTimeutc == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEUTC message");
                    #endif
                    return 3;
                }

                handle->ubxNavTimeutc->periodic = true;

            }
            else if ((handle->ubxNavTimeutc != NULL) && (period == 0))
                handle->ubxNavTimeutc->periodic = false;
            else if ((handle->ubxNavTimeutc != NULL) && (period != 0))
                handle->ubxNavTimeutc->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_TIMEUTC_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_VELECEF:
            if ((handle->ubxNavVelecef == NULL) && (period != 0))
            {
                handle->ubxNavVelecef = (gnss_ubx_nav_velecef_t *)malloc(sizeof(gnss_ubx_nav_velecef_t));
                
                if (handle->ubxNavVelecef == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-VELECEF message");
                    #endif
                    return 3;
                }

                handle->ubxNavVelecef->periodic = true;

            }
            else if ((handle->ubxNavVelecef != NULL) && (period == 0))
                handle->ubxNavVelecef->periodic = false;
            else if ((handle->ubxNavVelecef != NULL) && (period != 0))
                handle->ubxNavVelecef->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_VELECEF_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_VELECEF_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_VELECEF_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_NAV_VELNED:
            if ((handle->ubxNavVelned == NULL) && (period != 0))
            {
                handle->ubxNavVelned = (gnss_ubx_nav_velned_t *)malloc(sizeof(gnss_ubx_nav_velned_t));
                
                if (handle->ubxNavVelned == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-VELNED message");
                    #endif
                    return 3;
                }

                handle->ubxNavVelned->periodic = true;

            }
            else if ((handle->ubxNavVelned != NULL) && (period == 0))
                handle->ubxNavVelned->periodic = false;
            else if ((handle->ubxNavVelned != NULL) && (period != 0))
                handle->ubxNavVelned->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_NAV_VELNED_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_NAV_VELNED_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_NAV_VELNED_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_RXM_MEAS20:
            if ((handle->ubxRxmMeas20 == NULL) && (period != 0))
            {
                handle->ubxRxmMeas20 = (gnss_ubx_rxm_meas20_t *)malloc(sizeof(gnss_ubx_rxm_meas20_t));
                
                if (handle->ubxRxmMeas20 == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEAS20 message");
                    #endif
                    return 3;
                }

                handle->ubxRxmMeas20->periodic = true;

            }
            else if ((handle->ubxRxmMeas20 != NULL) && (period == 0))
                handle->ubxRxmMeas20->periodic = false;
            else if ((handle->ubxRxmMeas20 != NULL) && (period != 0))
                handle->ubxRxmMeas20->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEAS20_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEAS20_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEAS20_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_RXM_MEAS50:
            if ((handle->ubxRxmMeas50 == NULL) && (period != 0))
            {
                handle->ubxRxmMeas50 = (gnss_ubx_rxm_meas50_t *)malloc(sizeof(gnss_ubx_rxm_meas50_t));
                
                if (handle->ubxRxmMeas50 == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEAS50 message");
                    #endif
                    return 3;
                }

                handle->ubxRxmMeas50->periodic = true;

            }
            else if ((handle->ubxRxmMeas50 != NULL) && (period == 0))
                handle->ubxRxmMeas50->periodic = false;
            else if ((handle->ubxRxmMeas50 != NULL) && (period != 0))
                handle->ubxRxmMeas50->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEAS50_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEAS50_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEAS50_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_RXM_MEASC12:
            if ((handle->ubxRxmMeasc12 == NULL) && (period != 0))
            {
                handle->ubxRxmMeasc12 = (gnss_ubx_rxm_measc12_t *)malloc(sizeof(gnss_ubx_rxm_measc12_t));
                
                if (handle->ubxRxmMeasc12 == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEASC12 message");
                    #endif
                    return 3;
                }

                handle->ubxRxmMeasc12->periodic = true;

            }
            else if ((handle->ubxRxmMeasc12 != NULL) && (period == 0))
                handle->ubxRxmMeasc12->periodic = false;
            else if ((handle->ubxRxmMeasc12 != NULL) && (period != 0))
                handle->ubxRxmMeasc12->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASC12_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASC12_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASC12_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_RXM_MEASD12:
            if ((handle->ubxRxmMeasd12 == NULL) && (period != 0))
            {
                handle->ubxRxmMeasd12 = (gnss_ubx_rxm_measd12_t *)malloc(sizeof(gnss_ubx_rxm_measd12_t));
                
                if (handle->ubxRxmMeasd12 == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEASD12 message");
                    #endif
                    return 3;
                }

                handle->ubxRxmMeasd12->periodic = true;

            }
            else if ((handle->ubxRxmMeasd12 != NULL) && (period == 0))
                handle->ubxRxmMeasd12->periodic = false;
            else if ((handle->ubxRxmMeasd12 != NULL) && (period != 0))
                handle->ubxRxmMeasd12->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASD12_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASD12_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASD12_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_RXM_MEASX:
            if ((handle->ubxRxmMeasx == NULL) && (period != 0))
            {
                handle->ubxRxmMeasx = (gnss_ubx_rxm_measx_t *)malloc(sizeof(gnss_ubx_rxm_measx_t));
                
                if (handle->ubxRxmMeasx == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEASX message");
                    #endif
                    return 3;
                }

                handle->ubxRxmMeasx->periodic = true;

            }
            else if ((handle->ubxRxmMeasx != NULL) && (period == 0))
                handle->ubxRxmMeasx->periodic = false;
            else if ((handle->ubxRxmMeasx != NULL) && (period != 0))
                handle->ubxRxmMeasx->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASX_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASX_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_RXM_MEASX_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_SEC_SIG:
            if ((handle->ubxSecSig == NULL) && (period != 0))
            {
                handle->ubxSecSig = (gnss_ubx_sec_sig_t *)malloc(sizeof(gnss_ubx_sec_sig_t));
                
                if (handle->ubxSecSig == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-SEC-SIG message");
                    #endif
                    return 3;
                }

                handle->ubxSecSig->periodic = true;

            }
            else if ((handle->ubxSecSig != NULL) && (period == 0))
                handle->ubxSecSig->periodic = false;
            else if ((handle->ubxSecSig != NULL) && (period != 0))
                handle->ubxSecSig->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_SEC_SIG_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_SEC_SIG_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_SEC_SIG_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_SEC_SIGLOG:
            if ((handle->ubxSecSiglog == NULL) && (period != 0))
            {
                handle->ubxSecSiglog = (gnss_ubx_sec_siglog_t *)malloc(sizeof(gnss_ubx_sec_siglog_t));
                
                if (handle->ubxSecSiglog == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-SEC-SIGLOG message");
                    #endif
                    return 3;
                }

                handle->ubxSecSiglog->periodic = true;

            }
            else if ((handle->ubxSecSiglog != NULL) && (period == 0))
                handle->ubxSecSiglog->periodic = false;
            else if ((handle->ubxSecSiglog != NULL) && (period != 0))
                handle->ubxSecSiglog->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_SEC_SIGLOG_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_SEC_SIGLOG_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_SEC_SIGLOG_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_TIM_TM2:
            if ((handle->ubxTimTm2 == NULL) && (period != 0))
            {
                handle->ubxTimTm2 = (gnss_ubx_tim_tm2_t *)malloc(sizeof(gnss_ubx_tim_tm2_t));
                
                if (handle->ubxTimTm2 == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-TIM-TM2 message");
                    #endif
                    return 3;
                }

                handle->ubxTimTm2->periodic = true;

            }
            else if ((handle->ubxTimTm2 != NULL) && (period == 0))
                handle->ubxTimTm2->periodic = false;
            else if ((handle->ubxTimTm2 != NULL) && (period != 0))
                handle->ubxTimTm2->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_TIM_TM2_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_TIM_TM2_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_TIM_TM2_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_TIM_TP:
            if ((handle->ubxTimTp == NULL) && (period != 0))
            {
                handle->ubxTimTp = (gnss_ubx_tim_tp_t *)malloc(sizeof(gnss_ubx_tim_tp_t));
                
                if (handle->ubxTimTp == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-TIM-TP message");
                    #endif
                    return 3;
                }

                handle->ubxTimTp->periodic = true;

            }
            else if ((handle->ubxTimTp != NULL) && (period == 0))
                handle->ubxTimTp->periodic = false;
            else if ((handle->ubxTimTp != NULL) && (period != 0))
                handle->ubxTimTp->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_TIM_TP_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_TIM_TP_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_TIM_TP_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_TIM_VRFY:
            if ((handle->ubxTimVrfy == NULL) && (period != 0))
            {
                handle->ubxTimVrfy = (gnss_ubx_tim_vrfy_t *)malloc(sizeof(gnss_ubx_tim_vrfy_t));
                
                if (handle->ubxTimVrfy == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-TIM-VRFY message");
                    #endif
                    return 3;
                }

                handle->ubxTimVrfy->periodic = true;

            }
            else if ((handle->ubxTimVrfy != NULL) && (period == 0))
                handle->ubxTimVrfy->periodic = false;
            else if ((handle->ubxTimVrfy != NULL) && (period != 0))
                handle->ubxTimVrfy->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_TIM_VRFY_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_TIM_VRFY_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_TIM_VRFY_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_MON_COMMS:
            if ((handle->ubxMonComms == NULL) && (period != 0))
            {
                handle->ubxMonComms = (gnss_ubx_mon_comms_t *)malloc(sizeof(gnss_ubx_mon_comms_t));
                
                if (handle->ubxMonComms == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-COMMS message");
                    #endif
                    return 3;
                }

                handle->ubxMonComms->periodic = true;

            }
            else if ((handle->ubxMonComms != NULL) && (period == 0))
                handle->ubxMonComms->periodic = false;
            else if ((handle->ubxMonComms != NULL) && (period != 0))
                handle->ubxMonComms->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_MON_COMMS_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_MON_COMMS_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_MON_COMMS_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_MON_HW3:
            if ((handle->ubxMonHw3 == NULL) && (period != 0))
            {
                handle->ubxMonHw3 = (gnss_ubx_mon_hw3_t *)malloc(sizeof(gnss_ubx_mon_hw3_t));
                
                if (handle->ubxMonHw3 == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-HW3 message");
                    #endif
                    return 3;
                }

                handle->ubxMonHw3->periodic = true;

            }
            else if ((handle->ubxMonHw3 != NULL) && (period == 0))
                handle->ubxMonHw3->periodic = false;
            else if ((handle->ubxMonHw3 != NULL) && (period != 0))
                handle->ubxMonHw3->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_MON_HW3_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_MON_HW3_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_MON_HW3_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_MON_RF:
            if ((handle->ubxMonRf == NULL) && (period != 0))
            {
                handle->ubxMonRf = (gnss_ubx_mon_rf_t *)malloc(sizeof(gnss_ubx_mon_rf_t));
                
                if (handle->ubxMonRf == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-RF message");
                    #endif
                    return 3;
                }

                handle->ubxMonRf->periodic = true;

            }
            else if ((handle->ubxMonRf != NULL) && (period == 0))
                handle->ubxMonRf->periodic = false;
            else if ((handle->ubxMonRf != NULL) && (period != 0))
                handle->ubxMonRf->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_MON_RF_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_MON_RF_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_MON_RF_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        case GNSS_UBX_MON_SPAN:
            if ((handle->ubxMonSpan == NULL) && (period != 0))
            {
                handle->ubxMonSpan = (gnss_ubx_mon_span_t *)malloc(sizeof(gnss_ubx_mon_span_t));
                
                if (handle->ubxMonSpan == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-SPAN message");
                    #endif
                    return 3;
                }

                handle->ubxMonSpan->periodic = true;

            }
            else if ((handle->ubxMonSpan != NULL) && (period == 0))
                handle->ubxMonSpan->periodic = false;
            else if ((handle->ubxMonSpan != NULL) && (period != 0))
                handle->ubxMonSpan->periodic = true;

            switch (peripheral)
            {
                case 0: // I2C
                    key = GNSS_CFG_MSGOUT_UBX_MON_SPAN_I2C;
                    break;
                case 1: // UART
                    key = GNSS_CFG_MSGOUT_UBX_MON_SPAN_UART1;
                    break;
                case 2: //SPI
                    key = GNSS_CFG_MSGOUT_UBX_MON_SPAN_SPI;
                    break;
                default:
                    return 4;
                    break;
            }
            break;
        default:
            #ifdef DEBUG
            Serial.print("[DEBUG] Failed to find message for setting periodic: "); Serial.println(msg);
            #endif
            return 2;
            break;
    }

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, key, &period, 1))
    {
        #ifdef DEBUG
        Serial.print("[DEBUG] Failed to set periodic message cfg for "); Serial.println(msg, HEX);
        #endif
        return 1;
    }

    return 0;
}

/****************************************************************************
 * @brief Set CFG-RATE, parameters on navigation measurements and solutions.
 * Note that soln only indicates which measurement the solution is calculated from.
 * Setting soln higher than 1 requires more resources and should only be done when raw measurement data is needed at a higher rate than the nav data.
 * See Message Output Configuration of proper datasheet.
 * @param handle Handle for ublox gnss module.
 * @param meas Nominal time between GNSS measurements (ms) (minimum of 25).
 * @param soln Ratio of number of measurements to number of navigation solutions.
 * @param timeref Time system to align measurements to.
 * @return 0: Success
 *  1: Failed to set configurations
 ****************************************************************************/
uint8_t gnss_set_nav_rate(gnss_t *handle, uint16_t meas, uint16_t soln, uint8_t timeref)
{

    /*
     * Timeref configuration:
     * 0: UTC
     * 1: GPS
     * 2: GLONASS
     * 3: BeiDou
     * 4: Galileo
     * 5: NavIC
     */

    if (meas < 25)
        meas = 25; // Minimum value per datasheet

    uint8_t data[2];

    data[0] = meas & 0xFF;
    data[1] = (meas >> 8) & 0xFF;
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_RATE_MEAS, data, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set time between GNSS measurements");
        #endif
        return 1;
    }

    data[0] = soln & 0xFF;
    data[1] = (soln >> 8) & 0xFF;
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_RATE_NAV, data, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set measurements per solution");
        #endif
        return 1;
    }

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_RATE_TIMEREF, &timeref, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set time reference");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Enable TXREADY functionality on ublox gnss module. Will automatically disable primary function of peripheral for selected pin.
 * @param handle Handle for ublox gnss module.
 * @param pin PIO number of pin to use for TXREADY.
 * @param polarity 0: active high, 1: active low.
 * @param threshold Threshold of 8 bytes to trigger TXREADY. Number of 8 bytes, so a value of 5 = 40 byte threshold. Should not be set above 256 8-byte chunks
 * @param interface 0: I2C, 1: SPI
 * @return 0: Success
 * 1: Failed to set configuration data
 * @note Does not reenable previously disabled primary function upon making a new selection.
 ****************************************************************************/
uint8_t gnss_enable_rdy(gnss_t *handle, uint8_t pin, bool polarity, uint8_t threshold, uint8_t interface)
{

    /*
        Pin number as specified by UBX-MON-HW3 as PIO no. (SAM-M10Q):
        0: RXD
        1: TXD
        2: SDA
        3: SCL
        4: TIMEPULSE
        5: EXTINT

        Pin number as specified by UBX-MON-HW3 as PIO no. (NEO-M9N):
        0: D_SEL
        1: RXD
        2: TXD
        3: SDA
        4: SCL
        5: TIMEPULSE
        6: SAFEBOOT_N
        7: EXTINT
        8: Reserved (Pin 15)
        15: Reserved (Pin 16)
        16: LNA_EN

        Pin number as specified in UBX-MON-HW3 as PIO no. (NEO-M9V):
        0: D_SEL
        1: RXD
        2: TXD
        3: SDA
        4: SCL
        6: TIMEPULSE
        8: EXTINT/Wheel Tick
        16: LNA_EN

        Pin number as specified in UBX-MON-HW3 as PIO no. (DAN-F10N):
        0: RXD
        1: TXD
        4: TIMEPULSE
        5: EXTINT
    */

    bool uart = false;
    bool i2c = false;
    bool spi = false;
    bool timepulse = false;
    bool extint = false;

    if (handle->rcvr == GNSS_SAM_M10Q)
    {
        if ((pin == 0) || (pin == 1))
            uart = true;
        else if ((pin == 2) || (pin == 3))
            i2c = true;
        else if (pin == 4)
            timepulse = true;
        else if (pin == 5)
            extint = true;
    }
    else if (handle->rcvr == GNSS_NEO_M9N)
    {
        if ((pin == 1) || (pin == 2))
        {
            uart = true;
            spi = true;
        }
        else if ((pin == 3) || (pin == 4))
        {
            i2c = true;
            spi = true;
        }
        else if (pin == 5)
            timepulse = true;
        else if (pin == 7)
            extint = true;
    }
    else if (handle->rcvr == GNSS_NEO_M9V)
    {
        if ((pin == 1) || (pin == 2))
            {
            uart = true;
            spi = true;
        }
        else if ((pin == 3) || (pin == 4))
        {
            i2c = true;
            spi = true;
        }
        else if (pin == 6)
            timepulse = true;
        else if (pin == 8)
            extint = true;
    }
    else if (handle->rcvr == GNSS_DAN_F10N)
    {
        if ((pin == 0) || (pin == 1))
            uart = true;
        else if (pin == 4)
            timepulse = true;
        else if (pin == 5)
            extint = true;
    }

    uint8_t dat;

    if (uart)
    {
        // If pin is part of UART, disable UART pins per datasheet

        dat = 0x00;
        // CFG-UART1-ENABLED
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_UART1_ENABLED, &dat, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to disable UART peripheral");
            #endif
            return 1;
        }

    }

    else if (i2c)
    {
        // If pin is part of I2C, disable I2C pins per datasheet

        dat = 0x00;
        // CFG-I2C-ENABLED
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_I2C_ENABLED, &dat, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to disable I2C peripheral");
            #endif
            return 1;
        }

    }

    else if (spi)
    {
        // If pin is part of SPI, disable SPI pins per datasheet

        dat = 0x00;
        // CFG-SPI-ENABLED
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SPI_ENABLED, &dat, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to disable SPI peripheral");
            #endif
            return 1;
        }

    }
    
    else if (timepulse)
    {
        // If pin is part of TIMEPULSE, disable TIMEPULSE pin per datasheet

        dat = 0x00;

        if (handle->rcvr == GNSS_NEO_M9V)
        {
            // CFG-TP-TP2-ENA
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP2_ENA, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable TIMEPULSE Pin");
                #endif
                return 1;
            }
        }
        else
        {
            // CFG-TP-TP1-ENA
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP1_ENA, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable TIMEPULSE Pin");
                #endif
                return 1;
            }
        }
        
    }
    
    else if (extint)
    {
        // If pin is part of EXTINT, disable EXTINT pin per datasheet

        if (handle->rcvr == GNSS_NEO_M9V)
        {
            dat = 0x00;

            // CFG-SFODO-USE_WT_PIN
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SFODO_USE_WT_PIN, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable Wheel Tick functionality");
                #endif
                return 1;
            }
        }
        else
        {
            dat = 0x00;
            // CFG-PM-EXTINTWAKE
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTWAKE, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable EXTINT Wake functionality");
                #endif
                return 1;
            }
            // CFG-PM-BACKUP
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTBACKUP, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable EXTINT Backup functionality");
                #endif
                return 1;
            }
            // CFG-PM-INACTIVE
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTINACTIVE, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable EXTINT Inactive functionality");
                #endif
                return 1;
            }
        }
        
    }
    
    dat = 0x01;
    // CFG-TXREADY-ENABLED
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TXREADY_ENABLED, &dat, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable TXREADY");
        #endif
        return 1;
    }

    dat = polarity;
    // CFG-TXREADY-POLARITY
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TXREADY_POLARITY, &dat, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set TXREADY Polarity");
        #endif
        return 1;
    }

    dat = pin;
    // CFG-TXREADY-PIN
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TXREADY_PIN, &dat, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set TXREADY Pin");
        #endif
        return 1;
    }

    uint8_t dat2[] = {threshold, 0x00};
    // CFG-TXREADY-THRESHOLD
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TXREADY_THRESHOLD, dat2, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set TXREADY Threshold");
        #endif
        return 1;
    }

    dat = interface; // 0: I2C, 1: SPI
    // CFG-TXREADY-INTERFACE
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TXREADY_INTERFACE, &dat, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set TXREADY Interface");
        #endif
        return 1;
    }


    return 0;
}

/****************************************************************************
 * @brief Set CFG-BATCH for batch messaging functionality
 * @param handle Handle for ublox gnss module.
 * @param batchCfg Pointer to Batch Configuration Object
 * @return 0: Success
 * 1: Failed to set configuration data
 * 2: Failed to allocate memory for messages
 * @note Does not reenable previously disabled primary function upon making a new selection.
 ****************************************************************************/
uint8_t gnss_set_batch(gnss_t *handle, gnss_batch_cfg_t *batchCfg)
{

    /*
        Pin number as specified by UBX-MON-HW3 as PIO no. (SAM-M10Q):
        0: RXD
        1: TXD
        2: SDA
        3: SCL
        4: TIMEPULSE
        5: EXTINT

        Pin number as specified by UBX-MON-HW3 as PIO no. (NEO-M9N):
        0: D_SEL
        1: RXD
        2: TXD
        3: SDA
        4: SCL
        5: TIMEPULSE
        6: SAFEBOOT_N
        7: EXTINT
        8: Reserved (Pin 15)
        15: Reserved (Pin 16)
        16: LNA_EN

        Pin number as specified in UBX-MON-HW3 as PIO no. (NEO-M9V):
        0: D_SEL
        1: RXD
        2: TXD
        3: SDA
        4: SCL
        6: TIMEPULSE
        8: EXTINT/Wheel Tick
        16: LNA_EN

        Pin number as specified in UBX-MON-HW3 as PIO no. (DAN-F10N):
        0: RXD
        1: TXD
        4: TIMEPULSE
        5: EXTINT
    */

    uint8_t data[2];
    if (batchCfg->pioEnable)
    {

        bool uart = false;
        bool i2c = false;
        bool spi = false;
        bool timepulse = false;
        bool extint = false;

        if (handle->rcvr == GNSS_SAM_M10Q)
        {
            if ((batchCfg->pioID == 0) || (batchCfg->pioID == 1))
                uart = true;
            else if ((batchCfg->pioID == 2) || (batchCfg->pioID == 3))
                i2c = true;
            else if (batchCfg->pioID == 4)
                timepulse = true;
            else if (batchCfg->pioID == 5)
                extint = true;
        }
        else if (handle->rcvr == GNSS_NEO_M9N)
        {
            if ((batchCfg->pioID == 1) || (batchCfg->pioID == 2))
            {
                uart = true;
                spi = true;
            }
            else if ((batchCfg->pioID == 3) || (batchCfg->pioID == 4))
            {
                i2c = true;
                spi = true;
            }
            else if (batchCfg->pioID == 5)
                timepulse = true;
            else if (batchCfg->pioID == 7)
                extint = true;
        }
        else if (handle->rcvr == GNSS_NEO_M9V)
        {
            if ((batchCfg->pioID == 1) || (batchCfg->pioID == 2))
                {
                uart = true;
                spi = true;
            }
            else if ((batchCfg->pioID == 3) || (batchCfg->pioID == 4))
            {
                i2c = true;
                spi = true;
            }
            else if (batchCfg->pioID == 6)
                timepulse = true;
            else if (batchCfg->pioID == 8)
                extint = true;
        }
        else if (handle->rcvr == GNSS_DAN_F10N)
        {
            if ((batchCfg->pioID == 0) || (batchCfg->pioID == 1))
                uart = true;
            else if (batchCfg->pioID == 4)
                timepulse = true;
            else if (batchCfg->pioID == 5)
                extint = true;
        }

        if (uart)
        {
            // If pin is part of UART, disable UART pins per datasheet

            data[0] = 0x00;
            // CFG-UART1-ENABLED
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_UART1_ENABLED, data, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable UART peripheral");
                #endif
                return 1;
            }

        }

        else if (i2c)
        {
            // If pin is part of I2C, disable I2C pins per datasheet

            data[0] = 0x00;
            // CFG-I2C-ENABLED
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_I2C_ENABLED, data, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable I2C peripheral");
                #endif
                return 1;
            }

        }

        else if (spi)
        {
            // If pin is part of SPI, disable SPI pins per datasheet

            data[0] = 0x00;
            // CFG-SPI-ENABLED
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SPI_ENABLED, data, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable SPI peripheral");
                #endif
                return 1;
            }

        }
        
        else if (timepulse)
        {
            // If pin is part of TIMEPULSE, disable TIMEPULSE pin per datasheet

            data[0] = 0x00;
            if (handle->rcvr == GNSS_NEO_M9V)
            {
                // CFG-TP-TP2-ENA
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP2_ENA, data, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable TIMEPULSE Pin");
                    #endif
                    return 1;
                }
            }
            else
            {
                // CFG-TP-TP1-ENA
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP1_ENA, data, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable TIMEPULSE Pin");
                    #endif
                    return 1;
                }
            }
            
        }
        
        else if (extint)
        {
            // If pin is part of EXTINT, disable EXTINT pin per datasheet
            
            if (handle->rcvr == GNSS_NEO_M9V)
            {
                data[0] = 0x00;

                // CFG-SFODO-USE_WT_PIN
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SFODO_USE_WT_PIN, data, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable Wheel Tick functionality");
                    #endif
                    return 1;
                }
            }
            else
            {
                data[0] = 0x00;
                // CFG-PM-EXTINTWAKE
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTWAKE, data, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable EXTINT Wake functionality");
                    #endif
                    return 1;
                }
                // CFG-PM-BACKUP
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTBACKUP, data, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable EXTINT Backup functionality");
                    #endif
                    return 1;
                }
                // CFG-PM-INACTIVE
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTINACTIVE, data, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable EXTINT Inactive functionality");
                    #endif
                    return 1;
                }
            }

        }

    }

    data[0] = batchCfg->pioEnable;
    // CFG-BATCH-PIOENABLE
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_PIOENABLE, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set BATCH PIO Enable");
        #endif
        return 1;
    }

    data[0] = batchCfg->maxEntries & 0xFF;
    data[1] = (batchCfg->maxEntries >> 8) & 0xFF;
    // CFG-BATCH-MAXENTRIES
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_MAXENTRIES, data, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set BATCH Max Entries");
        #endif
        return 1;
    }

    data[0] = batchCfg->warnThresh & 0xFF;
    data[1] = (batchCfg->warnThresh >> 8) & 0xFF;
    // CFG-BATCH-WARNTHRS
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_WARNTHRS, data, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set BATCH Warning Threshold");
        #endif
        return 1;
    }

    data[0] = batchCfg->pioActiveLow;
    // CFG-BATCH-PIOACTIVELOW
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_PIOACTIVELOW, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set BATCH PIO Polarity");
        #endif
        return 1;
    }

    data[0] = batchCfg->pioID;
    // CFG-BATCH-PIOID
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_PIOID, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set BATCH PIO ID");
        #endif
        return 1;
    }

    data[0] = batchCfg->extraPVT;
    // CFG-BATCH-EXTRAPVT
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_EXTRAPVT, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set BATCH Extra PVT");
        #endif
        return 1;
    }

    data[0] = batchCfg->extraODO;
    // CFG-BATCH-EXTRAODO
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_EXTRAODO, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set BATCH Extra ODO");
        #endif
        return 1;
    }

    data[0] = batchCfg->enable;
    // CFG-BATCH-ENABLE
    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_BATCH_ENABLE, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable BATCH");
        #endif
        return 1;
    }
    
    // Allocate memory for UBX-MON-BATCH
    if (handle->ubxMonBatch == NULL)
    {
        gnss_ubx_mon_batch_t *ptr = (gnss_ubx_mon_batch_t *)malloc(sizeof(gnss_ubx_mon_batch_t));
        
        if (ptr == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Memory allocation failed for UBX-MON-BATCH");
            #endif
            return 2;
        }

        handle->ubxMonBatch = ptr;
    }

    // Allocate memory for UBX-LOG-BATCH
    if (handle->ubxLogBatch == NULL)
    {
        gnss_ubx_log_batch_t **ptr = (gnss_ubx_log_batch_t **)malloc(600 * sizeof(gnss_ubx_log_batch_t *)); // 600 is the max per the datasheet
        
        if (ptr == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Memory allocation failed for UBX-LOG-BATCH");
            #endif
            return 2;
        }

        handle->ubxLogBatch = ptr;
        
        for (uint16_t i = 0; i < 600; i++)
        {
            handle->ubxLogBatch[i] = (gnss_ubx_log_batch_t *)malloc(sizeof(gnss_ubx_log_batch_t));
            if (handle->ubxLogBatch[i] == NULL)
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Memory allocation failed for individual UBX-LOG-BATCH");
                #endif
                return 2;
            }
        }
        
    }

    return 0;
}

/****************************************************************************
 * @brief Set Time Pulse Configuration
 * @param handle Handle for ublox gnss module.
 * @param pulseCfg Pointer to Time Pulse Configuration Object
 * @return 0: Success
 * 1: Failed to set configuration data
 ****************************************************************************/
uint8_t gnss_set_pulse(gnss_t *handle, gnss_pulse_cfg_t *pulseCfg)
{

    // Write Values

    uint8_t data[8];

    data[0] = pulseCfg->pulseDef;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_PULSE_DEF, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-TP-PULSE_DEF");
        #endif
        return 1;
    }

    data[0] = pulseCfg->lengthDef;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_PULSE_LENGTH_DEF, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-TP-PULSE_LENGTH_DEF");
        #endif
        return 1;
    }

    data[0] = pulseCfg->antDelay & 0xFF;
    data[1] = (pulseCfg->antDelay >> 8) & 0xFF;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_ANT_CABLEDELAY, data, 0x02))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-TP-ANT_CABLEDELAY");
        #endif
        return 1;
    }

    if (handle->rcvr == GNSS_NEO_M9V)
    {

            data[0] = pulseCfg->period & 0xFF;
        data[1] = (pulseCfg->period >> 8) & 0xFF;
        data[2] = (pulseCfg->period >> 16) & 0xFF;
        data[3] = (pulseCfg->period >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_PERIOD_TP2, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-PERIOD_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->periodLock & 0xFF;
        data[1] = (pulseCfg->periodLock >> 8) & 0xFF;
        data[2] = (pulseCfg->periodLock >> 16) & 0xFF;
        data[3] = (pulseCfg->periodLock >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_PERIOD_LOCK_TP2, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-PERIOD_LOCK_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->freq & 0xFF;
        data[1] = (pulseCfg->freq >> 8) & 0xFF;
        data[2] = (pulseCfg->freq >> 16) & 0xFF;
        data[3] = (pulseCfg->freq >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_FREQ_TP2, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-FREQ_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->freqLock & 0xFF;
        data[1] = (pulseCfg->freqLock >> 8) & 0xFF;
        data[2] = (pulseCfg->freqLock >> 16) & 0xFF;
        data[3] = (pulseCfg->freqLock >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_FREQ_LOCK_TP2, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-FREQ_LOCK_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->pulseLength & 0xFF;
        data[1] = (pulseCfg->pulseLength >> 8) & 0xFF;
        data[2] = (pulseCfg->pulseLength >> 16) & 0xFF;
        data[3] = (pulseCfg->pulseLength >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_LEN_TP2, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-LEN_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->pulseLengthLock & 0xFF;
        data[1] = (pulseCfg->pulseLengthLock >> 8) & 0xFF;
        data[2] = (pulseCfg->pulseLengthLock >> 16) & 0xFF;
        data[3] = (pulseCfg->pulseLengthLock >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_LEN_LOCK_TP2, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-LEN_LOCK_TP2");
            #endif
            return 1;
        }

        data[0] = ((uint64_t)pulseCfg->duty >> 56) & 0xFF;
        data[1] = ((uint64_t)pulseCfg->duty >> 48) & 0xFF;
        data[2] = ((uint64_t)pulseCfg->duty >> 40) & 0xFF;
        data[3] = ((uint64_t)pulseCfg->duty >> 32) & 0xFF;
        data[4] = ((uint64_t)pulseCfg->duty >> 24) & 0xFF;
        data[5] = ((uint64_t)pulseCfg->duty >> 16) & 0xFF;
        data[6] = ((uint64_t)pulseCfg->duty >> 8) & 0xFF;
        data[7] = (uint64_t)pulseCfg->duty & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_DUTY_TP2, data, 0x08))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-DUTY_TP2");
            #endif
            return 1;
        }

        data[0] = ((uint64_t)pulseCfg->dutyLock >> 56) & 0xFF;
        data[1] = ((uint64_t)pulseCfg->dutyLock >> 48) & 0xFF;
        data[2] = ((uint64_t)pulseCfg->dutyLock >> 40) & 0xFF;
        data[3] = ((uint64_t)pulseCfg->dutyLock >> 32) & 0xFF;
        data[4] = ((uint64_t)pulseCfg->dutyLock >> 24) & 0xFF;
        data[5] = ((uint64_t)pulseCfg->dutyLock >> 16) & 0xFF;
        data[6] = ((uint64_t)pulseCfg->dutyLock >> 8) & 0xFF;
        data[7] = (uint64_t)pulseCfg->dutyLock & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_DUTY_LOCK_TP2, data, 0x08))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-DUTY_LOCK_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->userDelay & 0xFF;
        data[1] = (pulseCfg->userDelay >> 8) & 0xFF;
        data[2] = (pulseCfg->userDelay >> 16) & 0xFF;
        data[3] = (pulseCfg->userDelay >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_USER_DELAY_TP2, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-USER_DELAY_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->enable;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP2_ENA, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-TP2_ENA");
            #endif
            return 1;
        }

        data[0] = pulseCfg->syncGnss;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_SYNC_GNSS_TP2, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-SYNC_GNSS_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->useLocked;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_USE_LOCKED_TP2, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-USE_LOCKED_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->alignTOW;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_ALIGN_TO_TOW_TP2, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-ALIGN_TO_TOW_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->polarity;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_POL_TP2, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-POL_TP2");
            #endif
            return 1;
        }

        data[0] = pulseCfg->timeGrid;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TIMEGRID_TP2, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-TIMEGRID_TP2");
            #endif
            return 1;
        }

    }
    else
    {

        data[0] = pulseCfg->period & 0xFF;
        data[1] = (pulseCfg->period >> 8) & 0xFF;
        data[2] = (pulseCfg->period >> 16) & 0xFF;
        data[3] = (pulseCfg->period >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_PERIOD_TP1, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-PERIOD_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->periodLock & 0xFF;
        data[1] = (pulseCfg->periodLock >> 8) & 0xFF;
        data[2] = (pulseCfg->periodLock >> 16) & 0xFF;
        data[3] = (pulseCfg->periodLock >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_PERIOD_LOCK_TP1, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-PERIOD_LOCK_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->freq & 0xFF;
        data[1] = (pulseCfg->freq >> 8) & 0xFF;
        data[2] = (pulseCfg->freq >> 16) & 0xFF;
        data[3] = (pulseCfg->freq >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_FREQ_TP1, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-FREQ_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->freqLock & 0xFF;
        data[1] = (pulseCfg->freqLock >> 8) & 0xFF;
        data[2] = (pulseCfg->freqLock >> 16) & 0xFF;
        data[3] = (pulseCfg->freqLock >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_FREQ_LOCK_TP1, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-FREQ_LOCK_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->pulseLength & 0xFF;
        data[1] = (pulseCfg->pulseLength >> 8) & 0xFF;
        data[2] = (pulseCfg->pulseLength >> 16) & 0xFF;
        data[3] = (pulseCfg->pulseLength >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_LEN_TP1, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-LEN_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->pulseLengthLock & 0xFF;
        data[1] = (pulseCfg->pulseLengthLock >> 8) & 0xFF;
        data[2] = (pulseCfg->pulseLengthLock >> 16) & 0xFF;
        data[3] = (pulseCfg->pulseLengthLock >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_LEN_LOCK_TP1, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-LEN_LOCK_TP1");
            #endif
            return 1;
        }

        data[0] = ((uint64_t)pulseCfg->duty >> 56) & 0xFF;
        data[1] = ((uint64_t)pulseCfg->duty >> 48) & 0xFF;
        data[2] = ((uint64_t)pulseCfg->duty >> 40) & 0xFF;
        data[3] = ((uint64_t)pulseCfg->duty >> 32) & 0xFF;
        data[4] = ((uint64_t)pulseCfg->duty >> 24) & 0xFF;
        data[5] = ((uint64_t)pulseCfg->duty >> 16) & 0xFF;
        data[6] = ((uint64_t)pulseCfg->duty >> 8) & 0xFF;
        data[7] = (uint64_t)pulseCfg->duty & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_DUTY_TP1, data, 0x08))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-DUTY_TP1");
            #endif
            return 1;
        }

        data[0] = ((uint64_t)pulseCfg->dutyLock >> 56) & 0xFF;
        data[1] = ((uint64_t)pulseCfg->dutyLock >> 48) & 0xFF;
        data[2] = ((uint64_t)pulseCfg->dutyLock >> 40) & 0xFF;
        data[3] = ((uint64_t)pulseCfg->dutyLock >> 32) & 0xFF;
        data[4] = ((uint64_t)pulseCfg->dutyLock >> 24) & 0xFF;
        data[5] = ((uint64_t)pulseCfg->dutyLock >> 16) & 0xFF;
        data[6] = ((uint64_t)pulseCfg->dutyLock >> 8) & 0xFF;
        data[7] = (uint64_t)pulseCfg->dutyLock & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_DUTY_LOCK_TP1, data, 0x08))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-DUTY_LOCK_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->userDelay & 0xFF;
        data[1] = (pulseCfg->userDelay >> 8) & 0xFF;
        data[2] = (pulseCfg->userDelay >> 16) & 0xFF;
        data[3] = (pulseCfg->userDelay >> 24) & 0xFF;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_USER_DELAY_TP1, data, 0x04))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-USER_DELAY_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->enable;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP1_ENA, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-TP1_ENA");
            #endif
            return 1;
        }

        data[0] = pulseCfg->syncGnss;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_SYNC_GNSS_TP1, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-SYNC_GNSS_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->useLocked;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_USE_LOCKED_TP1, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-USE_LOCKED_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->alignTOW;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_ALIGN_TO_TOW_TP1, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-ALIGN_TO_TOW_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->polarity;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_POL_TP1, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-POL_TP1");
            #endif
            return 1;
        }

        data[0] = pulseCfg->timeGrid;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TIMEGRID_TP1, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-TP-TIMEGRID_TP1");
            #endif
            return 1;
        }
    }

    return 0;

}

/****************************************************************************
 * @brief Set Power Save Mode Configuration
 * @param handle Handle for ublox gnss module.
 * @param psmCfg Pointer to Power Save Mode Configuration Object
 * @return 0: Success
 * 1: Failed to set configuration data
 ****************************************************************************/
uint8_t gnss_set_psm(gnss_t *handle, gnss_psm_cfg_t *psmCfg)
{

    // Write Values

    uint8_t data[4];

    if (psmCfg != GNSS_PM_FULL)
    {
        // BeiDou B1C Signal isn't supported in PSM so it must be disabled
        data[0] = 0;
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_BDS_B1C_ENA, data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-SIGNAL-BDS-B1C-ENA");
            #endif
            return 1;
        }

        // SBAS recommended to be shutoff
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_SBAS_ENA, data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-SIGNAL-SBAS-ENA");
            #endif
            return 1;
        }
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SBAS_USE_TESTMODE, data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-SBAS-USE-TESTMODE");
            #endif
            return 1;
        }
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SBAS_USE_RANGING, data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-SBAS-USE-RANGING");
            #endif
            return 1;
        }
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SBAS_USE_DIFFCORR, data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-SBAS-USE-DIFFCORR");
            #endif
            return 1;
        }
        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SBAS_USE_INTEGRITY, data, 1))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-SBAS-USE-INTEGRITY");
            #endif
            return 1;
        }
    }

    // Continue with PSM CFG
    data[0] = psmCfg->posUpdatePeriod & 0xFF;
    data[1] = (psmCfg->posUpdatePeriod >> 8) & 0xFF;
    data[2] = (psmCfg->posUpdatePeriod >> 16) & 0xFF;
    data[3] = (psmCfg->posUpdatePeriod >> 24) & 0xFF;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_POSUPDATEPERIOD, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-POSUPDATEPERIOD");
        #endif
        return 1;
    }

    data[0] = psmCfg->acqPeriod & 0xFF;
    data[1] = (psmCfg->acqPeriod >> 8) & 0xFF;
    data[2] = (psmCfg->acqPeriod >> 16) & 0xFF;
    data[3] = (psmCfg->acqPeriod >> 24) & 0xFF;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_ACQPERIOD, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-ACQPERIOD");
        #endif
        return 1;
    }

    data[0] = psmCfg->gridOffset & 0xFF;
    data[1] = (psmCfg->gridOffset >> 8) & 0xFF;
    data[2] = (psmCfg->gridOffset >> 16) & 0xFF;
    data[3] = (psmCfg->gridOffset >> 24) & 0xFF;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_GRIDOFFSET, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-GRIDOFFSET");
        #endif
        return 1;
    }

    data[0] = psmCfg->onTime & 0xFF;
    data[1] = (psmCfg->onTime >> 8) & 0xFF;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_ONTIME, data, 0x02))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-ONTIME");
        #endif
        return 1;
    }

    data[0] = psmCfg->minAcqTime;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_MINACQTIME, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-MINACQTIME");
        #endif
        return 1;
    }

    data[0] = psmCfg->maxAcqTime;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_MAXACQTIME, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-MAXACQTIME");
        #endif
        return 1;
    }

    data[0] = psmCfg->doNotEnterOff;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_DONOTENTEROFF, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-DONOTENTEROFF");
        #endif
        return 1;
    }

    data[0] = psmCfg->waitTimeFix;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_WAITTIMEFIX, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-WAITTIMEFIX");
        #endif
        return 1;
    }

    data[0] = psmCfg->updateEph;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_UPDATEEPH, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-UPDATEEPH");
        #endif
        return 1;
    }
    
    if ((handle->rcvr == GNSS_NEO_M9N) || (handle->rcvr == GNSS_NEO_M9V))
    {
        data[0] = psmCfg->extIntSel;
        if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTSEL, data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write CFG-PM-EXTINTSEL");
            #endif
            return 1;
        }
    }
    
    data[0] = psmCfg->extIntWake;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTWAKE, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-EXTINTWAKE");
        #endif
        return 1;
    }

    data[0] = psmCfg->extIntBackup;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTBACKUP, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-EXTINTBACKUP");
        #endif
        return 1;
    }

    data[0] = psmCfg->extIntInactive;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTINACTIVE, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-EXTINTINACTIVE");
        #endif
        return 1;
    }

    data[0] = psmCfg->extIntInactivity & 0xFF;
    data[1] = (psmCfg->extIntInactivity >> 8) & 0xFF;
    data[2] = (psmCfg->extIntInactivity >> 16) & 0xFF;
    data[3] = (psmCfg->extIntInactivity >> 24) & 0xFF;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTINACTIVITY, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-EXTINTINACTIVITY");
        #endif
        return 1;
    }

    data[0] = psmCfg->limitPeakCurr;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_LIMITPEAKCURR, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-LIMITPEAKCURR");
        #endif
        return 1;
    }

    data[0] = psmCfg->mode;
    if(gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_OPERATEMODE, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-PM-OPERATEMODE");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Update Signals Used for GNSS
 * @param handle Handle for ublox gnss module.
 * @param signalCfg Pointer to Signal Configuration Object
 * @return 0: Success
 * 1: Failed to set configuration data
 * 2: Invalid signal configuration data
 ****************************************************************************/
uint8_t gnss_set_signals(gnss_t *handle, gnss_signal_cfg_t *signalCfg)
{

    /*
    * Invalid Signal Configurations
    * Disabling GPS_ENA or GPS_L1CA_ENA without disabling QZSS_L1CA_ENA and QZSS_L1S_ENA
    * SBAS is only functional with GPS
    *
    * Enabling BDS_ENA while BDS_B1_ENA is enabled
    * Enabling BDS_B1_ENA while BDS_ENA is enabled
    * No enabled signals (both the GNSS system and one of its signals)
    * Enabling QZSS_L1CA_ENA without GPS_ENA and GPS_L1CA_ENA enabled (Including QZSS_ENA)
    * Disabling QZSS_L1CA_ENA while QZSS_L1S_ENA is enabled
    * Enabling QZSS_L1S_ENA without GPS_ENA and GPS_L1CA_ENA enabled (Including QZSS_ENA)
    * 
    * Requires at least one GNSS enabled with at least one child signal
    * 
    * DAN-F10N:
    * Dual-band must be enabled - cannot enable single band on individual constellations (per datasheet 2.1.2)
    * - QZSS_L1S_ENA can be off any configuration
    * - NAVIC_ENA can be enabled while NAVIC_L5_ENA is disabled
    * - SBAS_ENA can be enabled while SBAS_L1CA_ENA is disabled
    * Cannot turn-off bands while constellation is enabled (e.g. Cannot disable GAL_E1_ENA while GAL_ENA is still enabled)
    * - QZSS_L1S_ENA can be turned off anytime
    * - NAVIC_L5_ENA can be turned off at anytime
    * - SBAS_L1CA_ENA can be turned off at anytime
    * - GPS bands cannot be turned off at anytime (but GPS_ENA can be)
    */

    // Check for Invalid Signal Configurations

    if (handle->rcvr == GNSS_DAN_F10N)
    {

        if (!signalCfg->gps_ena && (signalCfg->qzss_ena || (signalCfg->navic_ena && signalCfg->navic_l5_ena))) // Technically the receiver will let you do it, but it's non-functional
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (signalCfg->gps_ena && !(signalCfg->gps_l1ca_ena && signalCfg->gps_l5_ena))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (!signalCfg->gps_l1ca_ena && !signalCfg->gps_l5_ena)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (signalCfg->gal_ena && (!(signalCfg->gal_e1_ena && signalCfg->gal_e5a_ena) && !(!signalCfg->gal_e1_ena && !signalCfg->gal_e5a_ena)))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (signalCfg->qzss_ena && !(signalCfg->qzss_l1ca_ena && signalCfg->qzss_l5_ena))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (signalCfg->bds_ena && signalCfg->bds_b1_ena)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (signalCfg->bds_ena && (!(signalCfg->bds_b1c_ena && signalCfg->bds_b2a_ena) && !(!signalCfg->bds_b1c_ena && !signalCfg->bds_b2a_ena)))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (!signalCfg->gps_ena && (!signalCfg->gal_ena || (signalCfg->gal_ena && !(signalCfg->gal_e1_ena && signalCfg->gal_e5a_ena))) && (!signalCfg->bds_ena || (signalCfg->bds_ena && !(signalCfg->bds_b1c_ena && signalCfg->bds_b2a_ena)))) // No constellations enabled
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
    }
    else
    {
        if ((!signalCfg->gps_ena || !signalCfg->gps_l1ca_ena) && ((signalCfg->qzss_l1ca_ena || signalCfg->qzss_l1s_ena || signalCfg->qzss_ena) || (signalCfg->sbas_ena || signalCfg->sbas_l1ca_ena)))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (signalCfg->bds_ena && signalCfg->bds_b1_ena)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (!signalCfg->qzss_l1ca_ena && signalCfg->qzss_l1s_ena)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
        else if (!(signalCfg->gps_ena && signalCfg->gps_l1ca_ena) && !(signalCfg->gal_ena && signalCfg->gal_e1_ena) && !(signalCfg->bds_ena || signalCfg->bds_b1c_ena || signalCfg->bds_b1_ena)
        && !(signalCfg->qzss_ena && signalCfg->qzss_l1ca_ena) && !(signalCfg->glo_ena && signalCfg->glo_l1_ena))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Invalid signal configuration");
            #endif
            return 2;
        }
    }

    uint8_t data = 0x01;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 500000); // 0.5s per datasheet

    // Process: Turn-on GPS to prevent invalid configuration (due to no constellations or QZSS/NAVIC), assume all constellations have to be updated, lastly update GPS

    data = 0x01;

    // Turn on GPS so there's at least one constellation active to prevent invalid configuraiton

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GPS_ENA, &data, 0x01)) // Individual GPS signals cannot be disabled on DAN-F10N
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to enable GNSS_CFG_SIGNAL_GPS_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    /* SBAS */

    if (signalCfg->sbas_ena)
        data = 0x01;
    else
        data = 0x00;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_SBAS_ENA, &data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_SBAS_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    if (signalCfg->sbas_l1ca_ena)
        data = 0x01;
    else
        data = 0x00;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_SBAS_L1CA_ENA, &data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_SBAS_L1CA_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    /* GAL */

    data = 0x00;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GAL_ENA, &data, 0x01)) // Disable gal_ena first
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to disable GNSS_CFG_SIGNAL_GAL_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    if (signalCfg->gal_e1_ena)
        data = 0x01;
    else
        data = 0x00;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GAL_E1_ENA, &data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_GAL_E1_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    if (handle->rcvr == GNSS_DAN_F10N)
    {

        if (signalCfg->gal_e5a_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GAL_E5A_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_GAL_E5A_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }

    if (signalCfg->gal_ena)
    {

        data = 0x01;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GAL_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to enable GNSS_CFG_SIGNAL_GAL_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }
    // Else leave disabled

    /* BDS */

    data = 0x00;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_BDS_ENA, &data, 0x01)) // Disable bds_ena first
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to disable GNSS_CFG_SIGNAL_BDS_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_BDS_B1_ENA, &data, 0x01)) // Disable bds_b1 for later
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to disable GNSS_CFG_SIGNAL_BDS_B1_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    if ((handle->rcvr != GNSS_NEO_M9N) || (handle->rcvr != GNSS_NEO_M9V))
    {

        if (signalCfg->bds_b1c_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_BDS_B1C_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_BDS_B1C_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }

    if (handle->rcvr == GNSS_DAN_F10N)
    {

        if (signalCfg->bds_b2a_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_BDS_B2A_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_BDS_B2A_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }

    if (signalCfg->bds_ena)
    {

        data = 0x01;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_BDS_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to enable GNSS_CFG_SIGNAL_BDS_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }
    // Else leave disabled

    if (signalCfg->bds_b1_ena)
    {

        data = 0x01;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_BDS_B1_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to enable GNSS_CFG_SIGNAL_BDS_B1_ENA");
            #endif
            return 1;
        }
        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }
    // Else leave disabled

    /* QZSS */

    data = 0x00;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_QZSS_ENA, &data, 0x01)) // Disable gal_ena first
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to disable GNSS_CFG_SIGNAL_QZSS_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    if (signalCfg->qzss_l1ca_ena)
        data = 0x01;
    else
        data = 0x00;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_QZSS_L1CA_ENA, &data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_QZSS_L1CA_ENA");
        #endif
        return 1;
    }

    timer_reset(&gen_timer);
    while (!timer_check_exp(&gen_timer));
        ;

    if (handle->rcvr == GNSS_DAN_F10N)
    {

        if (signalCfg->qzss_l5_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_QZSS_L5_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_QZSS_L5_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }

    if (signalCfg->qzss_ena)
    {

        data = 0x01;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_QZSS_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to enable GNSS_CFG_SIGNAL_QZSS_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }
    // Else leave disabled

    if (handle->rcvr != GNSS_NEO_M9N)
    {

        if (signalCfg->qzss_l1s_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_QZSS_L1S_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_QZSS_L1S_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }

    /* GLONASS */

    if (handle->rcvr != GNSS_DAN_F10N)
    {

        if (signalCfg->glo_l1_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GLO_L1_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_GLO_L1_ENA");
            #endif
            return 1;
        }
        
        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer))
            ;
                
        if (signalCfg->glo_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GLO_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_GLO_ENA");
            #endif
            return 1;
        }
        
        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer))
            ;
    }

    /* NAVIC */
    if (handle->rcvr == GNSS_DAN_F10N)
    {

        if (signalCfg->navic_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_NAVIC_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_NAVIC_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;

        if (signalCfg->navic_l5_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_NAVIC_L5_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_NAVIC_L5_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;
    }

    /* GPS */

    if (handle->rcvr != GNSS_DAN_F10N)
    {

        data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GPS_ENA, &data, 0x01)) // Disable gps_ena first
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to disable GNSS_CFG_SIGNAL_GPS_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;

        if (signalCfg->gps_l1ca_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GPS_L1CA_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_GPS_L1CA_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;

        if (signalCfg->gps_ena)
            data = 0x01;
        else
            data = 0x00;

        if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GPS_ENA, &data, 0x01))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to update GNSS_CFG_SIGNAL_GPS_ENA");
            #endif
            return 1;
        }

        timer_reset(&gen_timer);
        while (!timer_check_exp(&gen_timer));
            ;

    }
    else
    {
        if (!signalCfg->gps_ena)
        {
            data = 0x00;

            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SIGNAL_GPS_ENA, &data, 0x01)) // Individual GPS signals cannot be disabled on DAN-F10N
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable GNSS_CFG_SIGNAL_GPS_ENA");
                #endif
                return 1;
            }

            timer_reset(&gen_timer);
            while (!timer_check_exp(&gen_timer));
                ;
        }
        // Else leave enabled
    }

    return 0;

}

/****************************************************************************
 * @brief Set CFG-NAV-SPG-DYNMODEL to one of the presets, configures max velocities, altitude, position deviation, and sanity check type.
 * @param handle Handle for ublox gnss module.
 * @param model Dynamic Model to set in the configuration.
 * @return 0: Success
 * 1: Failed to set configuration data
 ****************************************************************************/
uint8_t gnss_set_dynamic_model(gnss_t *handle, uint8_t model)
{

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_NAVSPG_DYNMODEL, &model, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set DYNMODEL");
        #endif
        return 1;
    }

    return 0;
}

/****************************************************************************
 * @brief Set CFG-ODO-PROFILE to one of the presets.
 * @param handle Handle for ublox gnss module.
 * @param model Odometer Model to set in the configuration.
 * @return 0: Success
 * 1: Failed to set configuration data
 ****************************************************************************/
uint8_t gnss_set_odo_model(gnss_t *handle, uint8_t model)
{

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_ODO_PROFILE, &model, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set Odometer profile");
        #endif
        return 1;
    }

    return 0;
}

/****************************************************************************
 * @brief Set Static Hold configuration. Set both to 0 to disable.
 * @param handle Handle for ublox gnss module.
 * @param speed Speed threshold (0.01 m/s)
 * @param distance Distance threshold (m?)
 * @return 0: Success
 * 1: Failed to set configuration data
 ****************************************************************************/
uint8_t gnss_set_static_hold(gnss_t *handle, uint8_t speed, uint16_t distance)
{

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_MOT_GNSSSPEED_THRS, &speed, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set GNSSSPEED-THRS");
        #endif
        return 1;
    }

    uint8_t data[2];

    data[0] = distance & 0xFF;
    data[1] = (distance >> 8) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_MOT_GNSSDIST_THRS, data, 2))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set GNSSDIST-THRS");
        #endif
        return 1;
    }

    return 0;
}

/****************************************************************************
 * @brief Set UART Baudrate. Must wait 100ms before attempting to send data after changing. Will re-open serial bus if busType = GNSS_UART
 * @param handle Handle for ublox gnss module.
 * @param speed Baudrate (only rates specified in datasheet)
 * @return 0: Success
 * 1: Failed to set configuration data
 * 2: Failed to restart serial
 * 3: Invalid Baudrate
 ****************************************************************************/
uint8_t gnss_set_uart_baud(gnss_t *handle, uint32_t baud)
{

    if ((baud != GNSS_UART_BAUD_4800) && (baud != GNSS_UART_BAUD_9600) && (baud != GNSS_UART_BAUD_19200) && (baud != GNSS_UART_BAUD_38400) && 
    (baud != GNSS_UART_BAUD_57600) && (baud != GNSS_UART_BAUD_115200) && (baud != GNSS_UART_BAUD_230400) && (baud != GNSS_UART_BAUD_460800) && (baud != GNSS_UART_BAUD_921600))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Invalid Baudrate");
        #endif
        return 3;
    }

    uint8_t data[4];
    data[0] = baud & 0xFF;
    data[1] = (baud >> 8) & 0xFF;
    data[2] = (baud >> 16) & 0xFF;
    data[3] = (baud >> 24) & 0xFF;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 150000); // Datasheet says 100ms, do 150ms to be safe

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_UART1_BAUDRATE, data, 4, 0))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to set CFG-UART1-BAUDRATE");
        #endif
        return 1;
    }
    
    if (handle->busType == GNSS_UART)
    {

        serial_close((serial_handle_t*)handle->bus);

        if (serial_open((serial_handle_t*)handle->bus, baud, UART_TYPE_BASIC))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to re-open serial bus");
            #endif
            return 2;
        }

        timer_start(&gen_timer);

        while(!timer_check_exp(&gen_timer))
            ;

    }
    else
    {
        timer_start(&gen_timer);

        while(!timer_check_exp(&gen_timer))
            ;
    }

    return 0;

}

/****************************************************************************
 * @brief Set Logging Configuration
 * @param handle Handle for ublox gnss module.
 * @param psmCfg Pointer to Logging Configuration Object
 * @return 0: Success
 * 1: Failed to set configuration data
 ****************************************************************************/
uint8_t gnss_set_logging(gnss_t *handle, gnss_log_cfg_t *logCfg)
{

    uint8_t filters = 0;
    uint8_t data[4];

    if (logCfg->psmPerWake != 0)
        filters = 1;

    data[0] = logCfg->psmPerWake;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_LOGFILTER_ONCE_PER_WAKE_UP_ENA, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-LOGFILTER-ONCE_PER_WAKE_UP_ENA");
        #endif
        return 1;
    }

    if (logCfg->minInterval != 0)
        filters = 1;

    data[0] = logCfg->minInterval & 0xFF;
    data[1] = (logCfg->minInterval >> 8) & 0xFF;
    data[2] = (logCfg->minInterval >> 16) & 0xFF;
    data[3] = (logCfg->minInterval >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_LOGFILTER_MIN_INTERVAL, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-LOGFILTER_MIN_INTERVAL");
        #endif
        return 1;
    }

    if (logCfg->timeThreshold != 0)
        filters = 1;

    data[0] = logCfg->timeThreshold & 0xFF;
    data[1] = (logCfg->timeThreshold >> 8) & 0xFF;
    data[2] = (logCfg->timeThreshold >> 16) & 0xFF;
    data[3] = (logCfg->timeThreshold >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_LOGFILTER_TIME_THRS, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-LOGFILTER_TIME_THRS");
        #endif
        return 1;
    }

    if (logCfg->speedThreshold != 0)
        filters = 1;

    data[0] = logCfg->speedThreshold & 0xFF;
    data[1] = (logCfg->speedThreshold >> 8) & 0xFF;
    data[2] = (logCfg->speedThreshold >> 16) & 0xFF;
    data[3] = (logCfg->speedThreshold >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_LOGFILTER_SPEED_THRS, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-LOGFILTER_SPEED_THRS");
        #endif
        return 1;
    }

    if (logCfg->posThreshold != 0)
        filters = 1;

    data[0] = logCfg->posThreshold & 0xFF;
    data[1] = (logCfg->posThreshold >> 8) & 0xFF;
    data[2] = (logCfg->posThreshold >> 16) & 0xFF;
    data[3] = (logCfg->posThreshold >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_LOGFILTER_POSITION_THRS, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-LOGFILTER_POSITION_THRS");
        #endif
        return 1;
    }

    // Set Apply All Filters if necessary

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_LOGFILTER_APPLY_ALL_FILTERS, &filters, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-LOGFILTER-APPLY_ALL_FILTERS");
        #endif
        return 1;
    }

    data[0] = logCfg->enable;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_LOGFILTER_RECORD_ENA, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-LOGFILTER_RECORD_ENA");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Set Geofencing Configuration
 * @param handle Handle for ublox gnss module.
 * @param geoCfg Pointer to Geofencing Configuration Object
 * @param chip Ublox product used - necessary to auto-disable the correct function
 * @return 0: Success
 * 1: Failed to set configuration data
 * 2: Invalid Geofence Confidence Level
 ****************************************************************************/
uint8_t gnss_set_geofencing(gnss_t *handle, gnss_geofence_cfg_t *geoCfg, uint8_t chip)
{

    /*
        Pin number as specified by UBX-MON-HW3 as PIO no. (SAM-M10Q):
        0: RXD
        1: TXD
        2: SDA
        3: SCL
        4: TIMEPULSE
        5: EXTINT

        Pin number as specified by UBX-MON-HW3 as PIO no. (NEO-M9N):
        0: D_SEL
        1: RXD
        2: TXD
        3: SDA
        4: SCL
        5: TIMEPULSE
        6: SAFEBOOT_N
        7: EXTINT
        8: Reserved (Pin 15)
        15: Reserved (Pin 16)
        16: LNA_EN

        Pin number as specified in UBX-MON-HW3 as PIO no. (NEO-M9V):
        0: D_SEL
        1: RXD
        2: TXD
        3: SDA
        4: SCL
        6: TIMEPULSE
        8: EXTINT/Wheel Tick
        16: LNA_EN

        Pin number as specified in UBX-MON-HW3 as PIO no. (DAN-F10N):
        0: RXD
        1: TXD
        4: TIMEPULSE
        5: EXTINT
    */

    bool uart = false;
    bool i2c = false;
    bool spi = false;
    bool timepulse = false;
    bool extint = false;

    if (chip == GNSS_SAM_M10Q)
    {
        if ((geoCfg->pin == 0) || (geoCfg->pin == 1))
            uart = true;
        else if ((geoCfg->pin == 2) || (geoCfg->pin == 3))
            i2c = true;
        else if (geoCfg->pin == 4)
            timepulse = true;
        else if (geoCfg->pin == 5)
            extint = true;
    }
    else if (chip == GNSS_NEO_M9N)
    {
        if ((geoCfg->pin == 1) || (geoCfg->pin == 2))
        {
            uart = true;
            spi = true;
        }
        else if ((geoCfg->pin == 3) || (geoCfg->pin == 4))
        {
            i2c = true;
            spi = true;
        }
        else if (geoCfg->pin == 5)
            timepulse = true;
        else if (geoCfg->pin == 7)
            extint = true;
    }
    else if (chip == GNSS_NEO_M9V)
    {
        if ((geoCfg->pin == 1) || (geoCfg->pin == 2))
            {
            uart = true;
            spi = true;
        }
        else if ((geoCfg->pin == 3) || (geoCfg->pin == 4))
        {
            i2c = true;
            spi = true;
        }
        else if (geoCfg->pin == 6)
            timepulse = true;
        else if (geoCfg->pin == 8)
            extint = true;
    }
    else if (chip == GNSS_DAN_F10N)
    {
        if ((geoCfg->pin == 0) || (geoCfg->pin == 1))
            uart = true;
        else if (geoCfg->pin == 4)
            timepulse = true;
        else if (geoCfg->pin == 5)
            extint = true;
    }

    uint8_t dat;

    if (geoCfg->usePio)
    {
        if (uart)
        {
            // If pin is part of UART, disable UART pins per datasheet

            dat = 0x00;
            // CFG-UART1-ENABLED
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_UART1_ENABLED, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable UART peripheral");
                #endif
                return 1;
            }

        }

        else if (i2c)
        {
            // If pin is part of I2C, disable I2C pins per datasheet

            dat = 0x00;
            // CFG-I2C-ENABLED
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_I2C_ENABLED, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable I2C peripheral");
                #endif
                return 1;
            }

        }

        else if (spi)
        {
            // If pin is part of SPI, disable SPI pins per datasheet

            dat = 0x00;
            // CFG-SPI-ENABLED
            if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SPI_ENABLED, &dat, 1))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Failed to disable SPI peripheral");
                #endif
                return 1;
            }

        }
        
        else if (timepulse)
        {
            // If pin is part of TIMEPULSE, disable TIMEPULSE pin per datasheet

            dat = 0x00;

            if (chip == GNSS_NEO_M9V)
            {
                // CFG-TP-TP2-ENA
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP2_ENA, &dat, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable TIMEPULSE Pin");
                    #endif
                    return 1;
                }
            }
            else
            {
                // CFG-TP-TP1-ENA
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_TP_TP1_ENA, &dat, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable TIMEPULSE Pin");
                    #endif
                    return 1;
                }
            }
            
        }
        
        else if (extint)
        {
            // If pin is part of EXTINT, disable EXTINT pin per datasheet

            if (chip == GNSS_NEO_M9V)
            {
                dat = 0x00;
                /*
                // CFG-SFODO-USE_WT_PIN
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_SFODO_USE_WT_PIN, &dat, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable Wheel Tick functionality");
                    #endif
                    return 1;
                }

                */
            }
            else
            {
                dat = 0x00;
                // CFG-PM-EXTINTWAKE
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTWAKE, &dat, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable EXTINT Wake functionality");
                    #endif
                    return 1;
                }
                // CFG-PM-BACKUP
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTBACKUP, &dat, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable EXTINT Backup functionality");
                    #endif
                    return 1;
                }
                // CFG-PM-INACTIVE
                if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_PM_EXTINTINACTIVE, &dat, 1))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Failed to disable EXTINT Inactive functionality");
                    #endif
                    return 1;
                }
            }
            
        }
    }

    if (geoCfg->confLvl > 5)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Invalid GEOFENCE Confidence Level");
        #endif
        return 2;
    }

    uint8_t data[4];

    data[0] = geoCfg->confLvl;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_CONFLVL, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-CONFLVL");
        #endif
        return 1;
    }

    data[0] = geoCfg->usePio;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_USE_PIO, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-USE_PIO");
        #endif
        return 1;
    }

    data[0] = geoCfg->pinPol;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_PINPOL, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-PINPOL");
        #endif
        return 1;
    }

    data[0] = geoCfg->pin;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_PIN, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-PIN");
        #endif
        return 1;
    }

    data[0] = geoCfg->latFence1 & 0xFF;
    data[1] = (geoCfg->latFence1 >> 8) & 0xFF;
    data[2] = (geoCfg->latFence1 >> 16) & 0xFF;
    data[3] = (geoCfg->latFence1 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE1_LAT, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE1_LAT");
        #endif
        return 1;
    }

    data[0] = geoCfg->lonFence1 & 0xFF;
    data[1] = (geoCfg->lonFence1 >> 8) & 0xFF;
    data[2] = (geoCfg->lonFence1 >> 16) & 0xFF;
    data[3] = (geoCfg->lonFence1 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE1_LON, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE1_LON");
        #endif
        return 1;
    }

    data[0] = geoCfg->radFence1 & 0xFF;
    data[1] = (geoCfg->radFence1 >> 8) & 0xFF;
    data[2] = (geoCfg->radFence1 >> 16) & 0xFF;
    data[3] = (geoCfg->radFence1 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE1_RAD, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE1_RAD");
        #endif
        return 1;
    }

    data[0] = geoCfg->useFence1;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_USE_FENCE1, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-USE_FENCE1");
        #endif
        return 1;
    }

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE2_LAT, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE2_LAT");
        #endif
        return 1;
    }

    data[0] = geoCfg->lonFence2 & 0xFF;
    data[1] = (geoCfg->lonFence2 >> 8) & 0xFF;
    data[2] = (geoCfg->lonFence2 >> 16) & 0xFF;
    data[3] = (geoCfg->lonFence2 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE2_LON, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE2_LON");
        #endif
        return 1;
    }

    data[0] = geoCfg->radFence2 & 0xFF;
    data[1] = (geoCfg->radFence2 >> 8) & 0xFF;
    data[2] = (geoCfg->radFence2 >> 16) & 0xFF;
    data[3] = (geoCfg->radFence2 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE2_RAD, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE2_RAD");
        #endif
        return 1;
    }

    data[0] = geoCfg->useFence2;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_USE_FENCE2, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-USE_FENCE2");
        #endif
        return 1;
    }

    data[0] = geoCfg->latFence2 & 0xFF;
    data[1] = (geoCfg->latFence2 >> 8) & 0xFF;
    data[2] = (geoCfg->latFence2 >> 16) & 0xFF;
    data[3] = (geoCfg->latFence2 >> 24) & 0xFF;

    data[0] = geoCfg->latFence3 & 0xFF;
    data[1] = (geoCfg->latFence3 >> 8) & 0xFF;
    data[2] = (geoCfg->latFence3 >> 16) & 0xFF;
    data[3] = (geoCfg->latFence3 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE3_LAT, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE3_LAT");
        #endif
        return 1;
    }

    data[0] = geoCfg->lonFence3 & 0xFF;
    data[1] = (geoCfg->lonFence3 >> 8) & 0xFF;
    data[2] = (geoCfg->lonFence3 >> 16) & 0xFF;
    data[3] = (geoCfg->lonFence3 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE3_LON, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE3_LON");
        #endif
        return 1;
    }

    data[0] = geoCfg->radFence3 & 0xFF;
    data[1] = (geoCfg->radFence3 >> 8) & 0xFF;
    data[2] = (geoCfg->radFence3 >> 16) & 0xFF;
    data[3] = (geoCfg->radFence3 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE3_RAD, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE3_RAD");
        #endif
        return 1;
    }

    data[0] = geoCfg->useFence3;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_USE_FENCE3, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-USE_FENCE3");
        #endif
        return 1;
    }

    data[0] = geoCfg->latFence4 & 0xFF;
    data[1] = (geoCfg->latFence4 >> 8) & 0xFF;
    data[2] = (geoCfg->latFence4 >> 16) & 0xFF;
    data[3] = (geoCfg->latFence4 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE4_LAT, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE4_LAT");
        #endif
        return 1;
    }

    data[0] = geoCfg->lonFence4 & 0xFF;
    data[1] = (geoCfg->lonFence4 >> 8) & 0xFF;
    data[2] = (geoCfg->lonFence4 >> 16) & 0xFF;
    data[3] = (geoCfg->lonFence4 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE4_LON, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE4_LON");
        #endif
        return 1;
    }

    data[0] = geoCfg->radFence4 & 0xFF;
    data[1] = (geoCfg->radFence4 >> 8) & 0xFF;
    data[2] = (geoCfg->radFence4 >> 16) & 0xFF;
    data[3] = (geoCfg->radFence4 >> 24) & 0xFF;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_FENCE4_RAD, data, 0x04))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-FENCE4_RAD");
        #endif
        return 1;
    }

    data[0] = geoCfg->useFence4;

    if (gnss_cfg_set(handle, GNSS_BBR_RAM, GNSS_CFG_GEOFENCE_USE_FENCE4, data, 0x01))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write CFG-GEOFENCE-USE_FENCE4");
        #endif
        return 1;
    }

    return 0;

}








/****************************************************************************
 * Messaging
 ****************************************************************************/







/****************************************************************************
 * @brief Send message to gnss module. Abstracts communication peripheral.
 * @param handle Handle for ublox gnss module.
 * @param data Data to send to ublox gnss module (array of bytes).
 * @param bytes Length of data to send (max 255).
 * @return 0: Success
 * 1: Invalid number of bytes to write (must be > 2)
 * 2: Failed to write I2C
 * 3: Parse buffer after SPI read/write failed
 ****************************************************************************/
static uint8_t gnss_tx(gnss_t *handle, uint8_t *data, uint8_t bytes)
{
    if(handle->busType == GNSS_I2C)
    {

        if (bytes < 2) // Required by Integration manual to distinguish from setting address counter to actual writes
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] I2C Writes must be 2 bytes in length minimum");
            #endif
            return 1;
        }

        i2c_set_addr((i2c_handle_t*)handle->bus, handle->busAddr); // Address this IC in case we're using this bus for multiple devices

        if (i2c_write((i2c_handle_t*)handle->bus, data, bytes))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to write I2C message");
            #endif
            return 2;
        }
    }
    if(handle->busType == GNSS_SPI)
    {

        handle->buffLength = bytes;

        memset(handle->buffer, 0x00, bytes);

        spi_write_read((spi_handle_t*)handle->bus, handle->busAddr, data, handle->buffer,(size_t)bytes);

        if(gnss_parse_buffer(handle))
            return 3;

        if (handle->buffer[0] != 0xFF) // Otherwise our output will be clogged with NULL data
        {
            Serial.print("[DEBUG] Incoming Rx from SPI Tx: ");
            for (uint16_t i = 0; i < bytes; i++)
            {
                Serial.print(handle->buffer[i]);
                Serial.print(" ");
            }
            Serial.println();
        }

    }
    else if(handle->busType == GNSS_UART)
    {

        for (uint8_t j = 0; j < bytes; j++)
        {
            serial_write((serial_handle_t*)handle->bus, &data[j], 1); // Send data byte
        }
        

    }

    #ifdef DEBUG
    Serial.print("[DEBUG] Sent data: ");
    

    for (uint8_t j = 0; j < bytes; j++)
    {
        Serial.print(data[j]); Serial.print(" ");
    }

    Serial.println();

    #endif
    
    return 0;
}

/****************************************************************************
 * @brief Send message with UBX protocol to ublox gnss module.
 * Additionally, creates a pointer for checking ACK during message parse if desired.
 * @param handle Handle for ublox gnss module.
 * @param cls Class for UBX protocol.
 * @param id ID for UBX protocol.
 * @param length Length of payload.
 * @param payload Pointer to payload (array of bytes).
 * @param ack Whether to expect an ACK or not (will create pointer node to be freed later).
 * @return 0: Success
 *  1: Failed to send tx message.
 *  2: Failed to allocate memory for ACK data.
 *  3: Failed to allocate memory for ACK node.
 * @note Data in payload often has to be coded for little endian, ensure this is correct before calling this function.
 ****************************************************************************/
static uint8_t gnss_ubx_msg(gnss_t *handle, uint8_t cls, uint8_t id, uint16_t length, uint8_t *payload, bool ack)
{

    uint8_t msg[65535]; // Max message

    uint16_t msg_length = 6 + length + 2;

    msg[0] = 0xB5; // UBX protocol
    msg[1] = 0x62;
    msg[2] = cls; // Class
    msg[3] = id; // ID
    msg[4] = length & 0xFF; // Length (LSB)
    msg[5] = (length >> 8) & 0xFF; // Length (MSB)

    memcpy(msg + 6, payload, length);
    
    // Calculate checksum
    uint8_t chkA = 0;
    uint8_t chkB = 0;
    
    for (uint16_t i = 2; i < (msg_length - 2); i++)
    {
        chkA += msg[i];
        chkB += chkA;
    }
    
    msg[msg_length - 2] = chkA;
    msg[msg_length - 1] = chkB;
    
    if (gnss_tx(handle, msg, msg_length))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] UBX Message failed to write TX");
        #endif
        return 1;
    }

    if (ack)
    {
        gnss_tx_msg_t *msgData = (gnss_tx_msg_t *)malloc(sizeof(gnss_tx_msg_t));
        if (msgData == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate memory for tx msg");
            #endif
            return 2;
        }
        msgData->msgClass = cls;
        msgData->msgID = id;
        msgData->length = length;
        memcpy(msgData->payload, payload, length);
        msgData->retry = 0;

        gnss_tx_msg_node_t *newMsg = (gnss_tx_msg_node_t *)malloc(sizeof(gnss_tx_msg_node_t));
        if (newMsg == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate memory for tx msg node");
            #endif
            return 3;
        }

        newMsg->data = msgData;
        newMsg->next = NULL;

        gnss_tx_msg_node_t *current = handle->txMsg;

        if (current == NULL)
        {
            handle->txMsg = newMsg;
        }
        else
        {
            gnss_tx_msg_node_t *next = current->next;
            while (next != NULL)
            {
                current = next;
                next = current->next;
            }

            current->next = newMsg;
        }

    }    

    return 0;
}

/****************************************************************************
 * @brief Receive messages from gnss module.
 * For I2C: This function performs the Random Read Access - checking addresses 0xFD and 0xFE before reading the amount of data from 0xFF.
 * For Serial: This function reads the data stored in the Serial buffer, checks if the final message is complete, and continues reading until the final message is complete if necessary.
 * The data for all cases is stored in a singular buffer, then it can separated using gnss_parse_buffer().
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: I2C Failed to read 0xFD and 0xFE message length registers.
 * 2: I2C Invalid Length (Likely unresponsive)
 * 3: I2C Not enough bytes available to be sent (minimum 2)
 * 4: I2C Failed to read message
 * 5: UART No bytes available in Serial buffer
 * 6: UART Buffer contains more than 500 bytes, likely overflow
 * 7: UART Failed to read buffer
 ****************************************************************************/
uint8_t gnss_rx(gnss_t *handle)
{

    uint16_t length = 0;

    handle->buffLength = 0;

    memset(handle->buffer, 0x00, GNSS_BUFFER_SIZE); // Clear buffer

    if(handle->busType == GNSS_I2C)
    {

        uint8_t regAddr = 0xFD; // Register of first byte of message length

        uint8_t len[2];

        i2c_set_addr((i2c_handle_t*)handle->bus, handle->busAddr); // Address this IC in case we're using this bus for multiple devices
        
        if (i2c_read_reg((i2c_handle_t*)handle->bus, &regAddr, 0x01, len, 0x02))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to read message length registers");
            #endif
            return 1;
        }

        length = (len[0] << 8) | len[1];

        if (length == 0xFFFF) // If invalid length
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Bytes to read is 0xFFFF");
            #endif
            return 2;
        }
        else if (length == 0x0000)
        {
            #ifdef DEBUG
            //Serial.println("[DEBUG] Message length = 0");
            #endif
            return 3;
        }
        #ifdef DEBUG
        Serial.print("[DEBUG] Data to read: ");
        Serial.println(length);
        #endif

        // Read data of size == length
        if (i2c_read((i2c_handle_t*)handle->bus, handle->buffer, length))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to read message");
            #endif
            return 4;
        }
        #ifdef DEBUG
        Serial.print("[DEBUG] Incoming Rx: ");
        for (uint16_t i = 0; i < length; i++)
        {
            Serial.print(handle->buffer[i]);
            Serial.print(" ");
        }
        #endif
    }
    if(handle->busType == GNSS_SPI)
    {

        length = 256;

        uint8_t tempData[length];
        memset(tempData, 0xFF, length); // Reads are conducted by keeping SDI high (per datasheet)

        // Receiver will parse incoming data until 50 consecutive 0xFF bytes, so if we attempt to read like Serial, we will lock the receiver attempting to read fake data.
        spi_write_read((spi_handle_t*)handle->bus, handle->busAddr, tempData, handle->buffer, length);
        
        // If last byte read indicates it's idle, then there should be no more data to read
        while ((handle->buffer[length - 1] != 0xFF) && (length < GNSS_BUFFER_SIZE))
        {
            spi_write_read((spi_handle_t*)handle->bus, handle->busAddr, tempData, handle->buffer + length, 100);
            length += 100; // Read 100 at a time to avoid locking the receiver
        }

        #ifdef DEBUG
        if (handle->buffer[0] != 0xFF) // Otherwise our output will be clogged with NULL data
        {
            Serial.print("[DEBUG] Incoming Rx: ");
            for (uint16_t i = 0; i < length; i++)
            {
                Serial.print(handle->buffer[i]);
                Serial.print(" ");
            }
            Serial.println();
        }
        #endif
        
    }
    else if(handle->busType == GNSS_UART)
    {

        length = serial_read_available((serial_handle_t*)handle->bus); // Check data length in buffer

        if (length == 0)
            return 5;
        else if (length >= 64 + SERIAL_RX_BUFFER_SIZE)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Serial FIFO may have overflowed, exiting...");
            #endif
            return 6;
        }

        if (serial_read((serial_handle_t*)handle->bus, handle->buffer, length))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to read serial data, exiting...");
            #endif
            return 7;
        }
        
        #ifdef DEBUG
        Serial.print("[DEBUG] Incoming Rx: ");
        for (uint16_t i = 0; i < length; i++)
        {
            Serial.print(handle->buffer[i]);
            Serial.print(" ");
        }
        Serial.println();
        #endif
    }

    handle->buffLength = length; // For use with parsing

    return 0;

}

/****************************************************************************
 * @brief Parse messages from buffer into individual messages.
 * The data is separated by searching for beginning and ending message portions and stored in individual message buffers.
 * The individual messages can then be interpreted using the parse_message function.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 8: Failed to find start of new message (Received data but no start of new message and no pending message in buffer)
 ****************************************************************************/
static uint8_t gnss_parse_buffer(gnss_t *handle)
{

    // Identify messages from buffer and store as individual messages for later parsing

    /*
    Strategy:
    Identify next message to write to -> parse message for complete message -> store in message buffer -> check if next message exists
    */
    uint16_t j = 0; // Used to store position of beginning of each message
    uint16_t i = 0; // Essentially the current cursor position on the buffer
    
    #ifdef DEBUG
    if(handle->busType == GNSS_I2C) // Other data will clog our output
    {
        Serial.println();
    }
    #endif

    while (i < handle->buffLength)
    {

        // Check if we've filled up the message buffer, erase and overwrite if so
        if (handle->pending_messages >= MSG_BUFFER_COUNT)
        {
            handle->messages[handle->write_message].length = 0;
            memset(handle->messages[handle->write_message].buffer, 0x00, MSG_BUFFER_SIZE);

            // Because we just erased this message, we no longer have a full message buffer. If pending_messages isn't decremented, a partial message may be parsed prematurely
            handle->pending_messages--;

            #ifdef DEBUG
            Serial.println();
            Serial.println("[DEBUG] Message buffer full, overwriting...");
            #endif

            // If we don't increment next_message, we'll be out of sync because write_message gets incremented with an overwrite
            if (handle->next_message == MSG_BUFFER_COUNT - 1)
                handle->next_message = 0;
            else
                handle->next_message += 1;

            #ifdef DEBUG
            Serial.print("[DEBUG] next_message: "); Serial.println(handle->next_message);
            Serial.print("[DEBUG] pending_messages: "); Serial.println(handle->pending_messages);
            #endif
        }
        
        // Check if a pending message of length 1 is truly the start of a UBX message
        if ((handle->messages[handle->write_message].length == 1) && (handle->messages[handle->write_message].buffer[0] == 0xB5))
        {
            if (handle->buffer[i] != 0x62)
            {
                // Pending message was a false positive, reset it
                handle->messages[handle->write_message].length = 0;
                memset(handle->messages[handle->write_message].buffer, 0x00, 1);
            }
        }
        
        // Begin identifying messages

        if (handle->messages[handle->write_message].length == 0) // No existing partial message, must find start of message
        {
            
            i = gnss_find_msg_start(handle->buffer, i, handle->buffLength); // Search for first character of message protocols

            if (i >= handle->buffLength) // We reached end of buffer without finding a start of message
            {
                // Account for potential that we only have 1 byte and couldn't verify UBX message header
                if ((i - j == 1) && (handle->buffer[j] == 0xB5))
                {
                    handle->messages[handle->write_message].length = 1;
                    memcpy(handle->messages[handle->write_message].buffer, handle->buffer + j, 1);
                    
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Potential initial UBX message");
                    #endif

                    return 0;
                }

                #ifdef DEBUG
                if(handle->busType == GNSS_I2C) // Other data will clog our output
                {
                    Serial.println("[DEBUG] Couldn't find start of message for new message");
                }
                #endif

                return 0;
            }

            j = i; // Found start of message, record position
            
            if (handle->buffer[i] == 0xB5) // UBX protocol
            {

                if (handle->buffLength - j < 6)
                {
                    // Not enough in buffer to determine length of message
                    handle->messages[handle->write_message].length = handle->buffLength - j;
                    memcpy(handle->messages[handle->write_message].buffer, handle->buffer + j, handle->buffLength - j);

                    #ifdef DEBUG
                    Serial.println("[DEBUG] Reached end without message ending 1");
                    #endif
                    return 0;
                }

                i += 4; // Go to length field

                i += ((uint16_t)handle->buffer[i] | (uint16_t)handle->buffer[i + 1] << 8) + 4 - 1; // 2 for length, 2 for checksum

                if (i >= handle->buffLength) // Length of message exceeds data read
                {
                    // Store partial message
                    handle->messages[handle->write_message].length = handle->buffLength - j;
                    memcpy(handle->messages[handle->write_message].buffer, handle->buffer + j, handle->buffLength - j);

                    #ifdef DEBUG
                    Serial.println("[DEBUG] Reached end without message ending 5");
                    #endif
                    return 0;
                }

                if (i - j + 1 > MSG_BUFFER_SIZE)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message size is too large for buffer, skipping");
                    #endif
                    i++;
                    continue;

                }
                // Store full message
                handle->messages[handle->write_message].length = i - j + 1;
                memcpy(handle->messages[handle->write_message].buffer, handle->buffer + j, i - j + 1);

                #ifdef DEBUG
                Serial.print("[DEBUG] write_message = "); Serial.println(handle->write_message);
                Serial.print("[DEBUG] New Message: ");
                for (uint16_t t = 0; t < handle->messages[handle->write_message].length; t++)
                {
                    Serial.print(handle->messages[handle->write_message].buffer[t]);
                    Serial.print(" ");
                }

                Serial.println();
                #endif

                // Increment message buffer trackers
                if (handle->pending_messages < MSG_BUFFER_COUNT)
                    handle->pending_messages++;

                if (handle->write_message >= MSG_BUFFER_COUNT - 1)
                    handle->write_message = 0;
                else
                    handle->write_message++;

                #ifdef DEBUG
                Serial.print("[DEBUG] pending_messages = "); Serial.println(handle->pending_messages);
                #endif

            }
            else if (handle->buffer[i] == '$') // Message is NMEA
            {

                i = gnss_find_msg_end(handle, i, handle->buffLength); // Find end of message

                if (i >= handle->buffLength) // Reached end of buffer before message was fully read
                {
                    i--;
                    // Store partial message
                    handle->messages[handle->write_message].length = i - j + 1;
                    memcpy(handle->messages[handle->write_message].buffer, handle->buffer + j, i - j + 1);

                    #ifdef DEBUG
                    Serial.println("[DEBUG] Reached end without message ending 2");
                    #endif
                    return 0;
                }

                if (i - j + 1 > MSG_BUFFER_SIZE)
                {

                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message size is too large for buffer, skipping");
                    #endif
                    i++;
                    continue;

                }
                // Store full message
                handle->messages[handle->write_message].length = i - j + 1;
                memcpy(handle->messages[handle->write_message].buffer, handle->buffer + j, i - j + 1);

                #ifdef DEBUG
                Serial.print("[DEBUG] write_message = "); Serial.println(handle->write_message);
                Serial.print("[DEBUG] New Message: ");
                for (uint16_t t = 0; t < handle->messages[handle->write_message].length; t++)
                {
                    Serial.print((char)handle->messages[handle->write_message].buffer[t]);
                }
                Serial.println();
                #endif

                // Increment message trackers
                if (handle->pending_messages < MSG_BUFFER_COUNT)
                    handle->pending_messages++;

                if (handle->write_message >= MSG_BUFFER_COUNT - 1)
                    handle->write_message = 0;
                else
                    handle->write_message++;

                #ifdef DEBUG
                Serial.print("[DEBUG] pending_messages = "); Serial.println(handle->pending_messages);
                #endif

            }
            else
            {
                #ifdef DEBUG
                Serial.println();
                Serial.println("[DEBUG] Start of new message does not match NMEA or UBX preamble");
                #endif
            }
        }
        else // Partial message exists already
        {

            j = i; // Record position of continuation of message

            if (handle->messages[handle->write_message].buffer[0] == 0xB5) // Stored partial message is UBX
            {

                uint16_t len;

                // Check if data already in buffer has length
                if (handle->messages[handle->write_message].length >= 5)
                {
                    // Check if we only have the first byte of the length in the previous message
                    if (handle->messages[handle->write_message].length == 5)
                        len = (uint16_t)handle->messages[handle->write_message].buffer[4] | ((uint16_t)handle->buffer[i] << 8);
                    else
                        len = (uint16_t)handle->messages[handle->write_message].buffer[4] | ((uint16_t)handle->messages[handle->write_message].buffer[5] << 8);
                       
                    i += len + 6 + 2 - handle->messages[handle->write_message].length - 1; // payload + length + everything before length + checksum - already gathered

                    if (i >= handle->buffLength)
                    {
                        // Not enough in buffer to determine length of message
                        memcpy(handle->messages[handle->write_message].buffer + handle->messages[handle->write_message].length, handle->buffer + j, handle->buffLength - j);
                        handle->messages[handle->write_message].length += handle->buffLength - j;

                        #ifdef DEBUG
                        Serial.println("[DEBUG] Reached end without message ending 6");
                        #endif
                        return 0;
                    }

                }
                else
                {
                    // We haven't captured length of payload yet
                    i += 4 - handle->messages[handle->write_message].length;

                    if (i >= handle->buffLength)
                    {
                        // Not enough in buffer to determine length of message
                        memcpy(handle->messages[handle->write_message].buffer + handle->messages[handle->write_message].length, handle->buffer + j, handle->buffLength - j);
                        handle->messages[handle->write_message].length += handle->buffLength - j;

                        #ifdef DEBUG
                        Serial.println("[DEBUG] Reached end without message ending 7");
                        #endif
                        return 0;
                    }

                    i += ((uint16_t)handle->buffer[i] | (uint16_t)handle->buffer[i + 1] << 8) + 4 - 1; // 2 for length, 2 for checksum

                    if (i >= handle->buffLength)
                    {
                        // Length of message exceeds data read
                        memcpy(handle->messages[handle->write_message].buffer + handle->messages[handle->write_message].length, handle->buffer + j, handle->buffLength - j);
                        handle->messages[handle->write_message].length += handle->buffLength - j;

                        #ifdef DEBUG
                        Serial.println("[DEBUG] Reached end without message ending 8");
                        #endif
                        return 0;
                    }

                }

                if (i - j + 1 + handle->messages[handle->write_message].length > MSG_BUFFER_SIZE)
                {

                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message size is too large for buffer, skipping");
                    #endif

                    // Erase existing partial message because we can't complete it
                    handle->messages[handle->write_message].length = 0;
                    memset(handle->messages[handle->write_message].buffer, 0x00, MSG_BUFFER_SIZE);

                    i++;
                    continue;

                }

                // Store full message
                memcpy(handle->messages[handle->write_message].buffer + handle->messages[handle->write_message].length, handle->buffer + j, i - j + 1);
                handle->messages[handle->write_message].length += i - j + 1;

                #ifdef DEBUG
                Serial.print("[DEBUG] write_message = "); Serial.println(handle->write_message);
                Serial.print("[DEBUG] New Message: ");
                for (uint16_t t = 0; t < handle->messages[handle->write_message].length; t++)
                {
                    Serial.print(handle->messages[handle->write_message].buffer[t]);
                    Serial.print(" ");
                }
                Serial.println();
                #endif

                // Increment message trackers
                if (handle->pending_messages < MSG_BUFFER_COUNT)
                    handle->pending_messages++;

                if (handle->write_message >= MSG_BUFFER_COUNT - 1)
                    handle->write_message = 0;
                else
                    handle->write_message++;

                #ifdef DEBUG
                Serial.print("[DEBUG] pending_messages = "); Serial.println(handle->pending_messages);
                #endif
            }
            else if (handle->messages[handle->write_message].buffer[0] == '$') // Stored partial message is NMEA
            {

                i = gnss_find_msg_end(handle, i, handle->buffLength); // Find end of message

                // Re-search if previous message wasn't missing final end byte but the just found end byte was the next... byte
                if (handle->messages[handle->write_message].buffer[handle->messages[handle->write_message].length - 1] != 0x13 && (i - j) == 0)
                    i = gnss_find_msg_end(handle, i, handle->buffLength);

                if (i >= handle->buffLength) // Reached end of buffer without finding end of message
                {
                    i--;
                    // Store partial message
                    memcpy(handle->messages[handle->write_message].buffer + handle->messages[handle->write_message].length, handle->buffer + j, i - j + 1);
                    handle->messages[handle->write_message].length += i - j + 1;

                    #ifdef DEBUG
                    Serial.println("[DEBUG] Reached end without message ending 4");
                    #endif
                    return 0;
                }

                if (i - j + 1 + handle->messages[handle->write_message].length > MSG_BUFFER_SIZE)
                {

                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message size is too large for buffer, skipping");
                    #endif

                    // Erase existing partial message because we can't complete it
                    handle->messages[handle->write_message].length = 0;
                    memset(handle->messages[handle->write_message].buffer, 0x00, MSG_BUFFER_SIZE);

                    i++;
                    continue;

                }
                
                // Store full message
                memcpy(handle->messages[handle->write_message].buffer + handle->messages[handle->write_message].length, handle->buffer + j, i - j + 1);
                handle->messages[handle->write_message].length += i - j + 1;

                #ifdef DEBUG
                Serial.print("[DEBUG] write_message = "); Serial.println(handle->write_message);
                Serial.print("[DEBUG] New Message: ");
                for (uint16_t t = 0; t < handle->messages[handle->write_message].length; t++)
                {
                    Serial.print((char)handle->messages[handle->write_message].buffer[t]);
                }
                #endif

                // Increment message trackers
                if (handle->pending_messages < MSG_BUFFER_COUNT)
                    handle->pending_messages++;

                if (handle->write_message >= MSG_BUFFER_COUNT - 1)
                    handle->write_message = 0;
                else
                    handle->write_message++;

                #ifdef DEBUG
                Serial.print("[DEBUG] pending_messages = "); Serial.println(handle->pending_messages);
                #endif

            }
            else
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] Start of existing message does not match NMEA or UBX preamble");
                #endif
            }
            
        }
        i++;
    }
    
    return 0;

}

/****************************************************************************
 * @brief Send NMEA message to ublox gnss module.
 * @param handle Handle for ublox gnss module.
 * @param talkerID TalkerID to use for sending message - must correspond to the Main Talker ID set on the receiver.
 * @param msg Message to send to the receiver ((class << 8) | id) e.g. GNSS_NMEA_STANDARD_DTM.
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
static uint8_t gnss_nmea_msg(gnss_t *handle, uint8_t talkerID, uint16_t msg)
{

    // The message used must correspond to the main talker ID (if auto, must use GNQ)

    /* Talker IDs:
        1: GPQ (GP, GPS, SBAS)
        2: GLQ (GL, GLONASS)
        3: GNQ (GN, Any combination of GNSS)
        4: GAQ (GA, Galileo)
        5: GBQ (GB, BeiDou)
        7: GQQ (GQ, QZSS)
        */

    uint8_t data[15];
    data[0] = '$';
    data[1] = 'E';
    data[2] = 'I';

    switch (talkerID)
    {
        case GNSS_TALKER_GPS:
            data[3] = 'G';
            data[4] = 'P';
            data[5] = 'Q';
            break;
        case GNSS_TALKER_GLONASS:
            data[3] = 'G';
            data[4] = 'L';
            data[5] = 'Q';
            break;
        case GNSS_TALKER_GNSS:
            data[3] = 'G';
            data[4] = 'N';
            data[5] = 'Q';
            break;
        case GNSS_TALKER_GALILEO:
            data[3] = 'G';
            data[4] = 'A';
            data[5] = 'Q';
            break;
        case GNSS_TALKER_BEIDOU:
            data[3] = 'G';
            data[4] = 'B';
            data[5] = 'Q';
            break;
        case GNSS_TALKER_QZSS:
            data[3] = 'G';
            data[4] = 'Q';
            data[5] = 'Q';
            break;
        default: // Default to Auto (GNSS)
            data[3] = 'G';
            data[4] = 'N';
            data[5] = 'Q';
            break; 
    }

    data[6] = ',';

    switch (msg)
    {
        case GNSS_NMEA_STANDARD_DTM:
            data[7] = 'D';
            data[8] = 'T';
            data[9] = 'M';
            break;
        case GNSS_NMEA_STANDARD_GBS:
            data[7] = 'G';
            data[8] = 'B';
            data[9] = 'S';
            break;
        case GNSS_NMEA_STANDARD_GGA:
            data[7] = 'G';
            data[8] = 'G';
            data[9] = 'A';
            break;
        case GNSS_NMEA_STANDARD_GLL:
            data[7] = 'G';
            data[8] = 'L';
            data[9] = 'L';
            break;
        case GNSS_NMEA_STANDARD_GNS:
            data[7] = 'G';
            data[8] = 'N';
            data[9] = 'S';
            break;
        case GNSS_NMEA_STANDARD_GRS:
            data[7] = 'G';
            data[8] = 'R';
            data[9] = 'S';
            break;
        case GNSS_NMEA_STANDARD_GSA:
            data[7] = 'G';
            data[8] = 'S';
            data[9] = 'A';
            break;
        case GNSS_NMEA_STANDARD_GST:
            data[7] = 'G';
            data[8] = 'S';
            data[9] = 'T';
            break;
        case GNSS_NMEA_STANDARD_GSV:
            data[7] = 'G';
            data[8] = 'S';
            data[9] = 'V';
            break;
        case GNSS_NMEA_STANDARD_RMC:
            data[7] = 'R';
            data[8] = 'M';
            data[9] = 'C';
            break;
        case GNSS_NMEA_STANDARD_TXT:
            data[7] = 'T';
            data[8] = 'X';
            data[9] = 'T';
            break;
        case GNSS_NMEA_STANDARD_VLW:
            data[7] = 'V';
            data[8] = 'L';
            data[9] = 'W';
            break;
        case GNSS_NMEA_STANDARD_VTG:
            data[7] = 'V';
            data[8] = 'T';
            data[9] = 'G';
            break;
        case GNSS_NMEA_STANDARD_ZDA:
            data[7] = 'Z';
            data[8] = 'D';
            data[9] = 'A';
            break;
        default:
            return 2;
            break;
    }

    uint16_t chksum = gnss_nmea_checksum(data, 10);

    data[10] = '*';
    data[11] = (chksum >> 8) & 0xFF;
    data[12] = chksum & 0xFF;
    data[13] = 0x0D;
    data[14] = 0x0A;

    if (gnss_tx(handle, data, 15))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write NMEA message.");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Send PUBX message to ublox gnss module.
 * @param handle Handle for ublox gnss module.
 * @param data Data to send to ublox gnss module (array of bytes).
 * @param length Length of data to send.
 * @return 0: Success
 * 1: Requested message to send is larger than buffer size
 * 2: Failed to send message
 ****************************************************************************/
static uint8_t gnss_pubx_msg(gnss_t *handle, uint8_t *data, uint8_t length)
{

    uint8_t msgData[64];
    msgData[0] = '$';
    msgData[1] = 'P';
    msgData[2] = 'U';
    msgData[3] = 'B';
    msgData[4] = 'X';
    msgData[5] = ',';

    if (length > 64 - 11)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Requested message exceeds buffer size");
        #endif
        return 1;
    }

    memcpy(msgData + 6, data, length);

    uint16_t chksum = gnss_nmea_checksum(msgData, 6 + length);

    msgData[6 + length] = '*';
    msgData[6 + length + 1] = (chksum >> 8) & 0xFF;
    msgData[6 + length + 2] = chksum & 0xFF;
    msgData[6 + length + 3] = 0x0D;
    msgData[6 + length + 4] = 0x0A;

    if (gnss_tx(handle, msgData, 6 + length + 5))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to write pubx message.");
        #endif
        return 2;
    }

    return 0;

}






/****************************************************************************
 * Messaging Operations
 ****************************************************************************/





/****************************************************************************
 * @brief Search buffer for start of message (both NMEA and UBX protocols).
 * @param handle Handle for ublox gnss module.
 * @param start Index to start search.
 * @param length Length of buffer.
 * @return If found: Index of message start
 * Failed to find: Input length
 ****************************************************************************/
static uint16_t gnss_find_msg_start(uint8_t *buffer, uint16_t start, uint16_t length)
{

    if (start >= length)
        return length;
    
    while (buffer[start] != 0x62)
    {
        
        while ((buffer[start] != 0xB5) && (buffer[start] != 36))
        {
            start++;
            
            if (start >= length)
                return length;
            
        }
        
        if (buffer[start] == 36)
            return start;

        start++;
        if (start >= length)
            return length;

    }

    start--;

    return start;
}

/****************************************************************************
 * @brief Search buffer for end of message (only NMEA, UBX is found through checksum).
 * @param handle Handle for ublox gnss module.
 * @param start Index to start search.
 * @param length Length of buffer.
 * @return If found: Index of message start
 * Failed to find: Input length
 ****************************************************************************/
static uint16_t gnss_find_msg_end(gnss_t *handle, uint16_t start, uint16_t length)
{

    if (start >= length)
        return length;
    
    // As odd as the NMEA checksum is, because the checksum is written as ASCII characters to represent values, we don't have to worry about confusing them for CR/LF

    while (handle->buffer[start] != 0x0A)
    {
        
        while (handle->buffer[start] != 0x0D)
        {
            start++;
            if (start == length)
                return start;
            
        }

        start++;
        if (start == length)
            return start;

    }
    
    return start;
}

/****************************************************************************
 * @brief Return single digit from ascii byte.
 * @param ascii Single ASCII digit/byte
 * @return 0 - 9: Actual Data
 * 10: ASCII input is a period (e.g. 0x2E)
 * 11: ASCII input is outside the bounds of 0 - 9
 ****************************************************************************/
static uint8_t gnss_ascii_data(uint8_t ascii)
{

    uint8_t data;

    // ASCII numbers correspond to 0x30 = 0, 0x31 = 1, etc...
    if (ascii == 0x2E) // Period
        data = 10;
    else
        data = ascii - '0';

    if (data > 9)
        data = 11;
    
    return data;

}

/****************************************************************************
 * @brief Return double from a string of ASCII characters.
 * @param data ASCII data (array of bytes).
 * @param start Index of array to start from.
 * @param end Index of array to end before (e.g. end = 5 means the for loop ends as i < 5).
 * @return Converted data from input
 ****************************************************************************/
static double parse_decimal_string(uint8_t *data, uint8_t start, uint8_t end)
{
    double result = 0.0;
    int integer_part = 0;
    double decimal_part = 0.0;
    int decimal_place = 0;
    int is_decimal = 0;
    bool is_negative = 0;

    if (data[start] == '-') // Check once and then don't check anymore
        is_negative = 1;

    for (uint8_t i = start; i < end; i++) {
        if (data[i] == '.') {
            is_decimal = 1;
            decimal_place = 0;
            continue;
        }

        if (data[i] >= '0' && data[i] <= '9') {
            if (!is_decimal) {
                // Build integer part
                integer_part = integer_part * 10 + (data[i] - '0');
            } else {
                // Build decimal part
                decimal_part = decimal_part * 10 + (data[i] - '0');
                decimal_place++;
            }
        }

    }

    // Combine integer and decimal parts
    result = integer_part + (decimal_part / pow(10, decimal_place));

    if (is_negative)
        result = result * -1;

    return result;
}

/****************************************************************************
 * @brief Within a NMEA message, search for terminator of current group. Start search on the first character of current group.
 * @param buffer Pointer to buffer to search (array of bytes).
 * @param start Start index in buffer.
 * @param length Length of buffer.
 * @return Index of terminator of current group
 ****************************************************************************/
static uint8_t gnss_find_next_group(uint8_t *buffer, uint16_t start, uint16_t length)
{
    
    while ((buffer[start] != ',') && (buffer[start] != '*'))
    {
        
        start++;

        if (start >= length)
            return length;

    }

    return start;
}

/****************************************************************************
 * @brief Convert ASCII data of time format hhmmss.ss to milliseconds
 * @param buffer Pointer to buffer to search (array of bytes).
 * @param index Index of the first digit in the data.
 * @return Millisecond representation of time.
 ****************************************************************************/
static uint32_t gnss_parse_time(uint8_t *buffer, uint8_t index)
{

    uint8_t hours = gnss_ascii_data(buffer[index]) * 10 + gnss_ascii_data(buffer[index + 1]);
    uint8_t minutes = gnss_ascii_data(buffer[index + 2]) * 10 + gnss_ascii_data(buffer[index + 3]);
    uint8_t seconds = gnss_ascii_data(buffer[index + 4]) * 10 + gnss_ascii_data(buffer[index + 5]);
    uint8_t mils = gnss_ascii_data(buffer[index + 7]) * 100 + gnss_ascii_data(buffer[index + 8]) * 10;

    return mils + (seconds * 1000) + (minutes * 1000 * 60) + (hours * 1000 * 3600);

}

/****************************************************************************
 * @brief Calculate NMEA Checksum. 
 * @param message Pointer to message to calculate checksum (array of bytes). Should be the start of the full message.
 * @param length Length of message not including the checksum or terminating bytes.
 * @return (CheckA << 8) | CheckB
 ****************************************************************************/
static uint16_t gnss_nmea_checksum(uint8_t *message, uint16_t length)
{

    uint8_t checkByte = 0;
    char checkA;
    char checkB;

    for (uint16_t i = 1; i < length; i++)
    {
        checkByte = checkByte ^ message[i];
    }

    checkA = (checkByte & 0xF0) >> 4;
    checkB = checkByte & 0x0F;

    if (checkA > 9)
        checkA = checkA - 10 + 'A';
    else
        checkA += '0';

    if (checkB > 9)
        checkB = checkB - 10 + 'A';
    else
        checkB += '0';

    #ifdef DEBUG
    Serial.print("[DEBUG] checksum: "); Serial.print((char)checkA); Serial.println((char)checkB);
    #endif

    return (checkA << 8) | checkB;

}

/****************************************************************************
 * @brief Calculate UBX Checksum. 
 * @param message Pointer to message object to calculate checksum.
 * @return (CheckA << 8) | CheckB
 ****************************************************************************/
static uint16_t gnss_ubx_checksum(gnss_msg_t *message)
{

    uint8_t chkA = 0;
    uint8_t chkB = 0;
    
    for (uint16_t i = 2; i < (message->length - 2); i++)
    {
        chkA += message->buffer[i];
        chkB += chkA;
    }

    return (chkA << 8) | chkB;

}

/****************************************************************************
 * @brief Return 16-bit unsigned integer from 2 byte little endian encoded data.
 * @param data Data to convert from from (array of bytes).
 * @param index Index of data to pull data from (index start).
 * @return 16-bit unsigned integer
 ****************************************************************************/
static uint16_t getUByte16_LEnd(uint8_t *data, uint16_t index)
{
    return (uint16_t)((data[index + 1] << 8) | data[index]);
}

/****************************************************************************
 * @brief Return 16-bit signed integer from 2 byte little endian encoded data.
 * @param data Data to convert from from (array of bytes).
 * @param index Index of data to pull data from (index start).
 * @return 16-bit signed integer
 ****************************************************************************/
static int16_t getIByte16_LEnd(uint8_t *data, uint16_t index)
{
    return (int16_t)((data[index + 1] << 8) | data[index]);
}

/****************************************************************************
 * @brief Return 32-bit unsigned integer from 4 byte little endian encoded data.
 * @param data Data to convert from from (array of bytes).
 * @param index Index of data to pull data from (index start).
 * @return 32-bit unsigned integer
 ****************************************************************************/
static uint32_t getUByte32_LEnd(uint8_t *data, uint16_t index)
{
    return (uint32_t)((data[index + 3] << 24) | (data[index + 2] << 16) | (data[index + 1] << 8) | data[index]);
}

/****************************************************************************
 * @brief Return 32-bit signed integer from 4 byte little endian encoded data.
 * @param data Data to convert from from (array of bytes).
 * @param index Index of data to pull data from (index start).
 * @return 32-bit signed integer
 ****************************************************************************/
static int32_t getIByte32_LEnd(uint8_t *data, uint16_t index)
{
    return (int32_t)((data[index + 3] << 24) | (data[index + 2] << 16) | (data[index + 1] << 8) | data[index]);
}

/****************************************************************************
 * @brief Return float from 4 bytes data (big endian).
 * @param data Data to convert from from (array of bytes).
 * @param index Index of data to pull data from (index start).
 * @return Float number
 ****************************************************************************/
static float getFloat(uint8_t *data, uint16_t index)
{
    return (float)((data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3]);
}

/****************************************************************************
 * @brief Create and store info message to be read by the user.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer to message object.
 * @return 0: Success
 * 1: Failed to allocate memory for the info message
 * 2: Failed to allocate memory for the info node
 ****************************************************************************/
static uint8_t gnss_create_info_node(gnss_t *handle, gnss_msg_t *message, uint16_t msgType)
{

    uint8_t offset;
    uint8_t truncate;

    if (message->buffer[0] == '$') // NMEA message
    {
        offset = 1;
        truncate = 5;
    }
    else // UBX message
    {
        offset = 6;
        truncate = 2;
    }

    // Allocate memory for new node
    gnss_info_t *infoData = (gnss_info_t *)malloc(sizeof(gnss_info_t));
    if (infoData == NULL)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to allocate memory for info msg");
        #endif
        return 1;
    }
    
    infoData->msgClass = (msgType >> 8) & 0xFF;
    infoData->msgID = msgType & 0xFF;
    infoData->length = message->length - offset - truncate;
    memcpy(infoData->payload, message->buffer + offset, message->length - offset - truncate);

    gnss_info_node_t *newInfo = (gnss_info_node_t *)malloc(sizeof(gnss_info_node_t));
    if (newInfo == NULL)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to allocate memory for info node");
        #endif
        return 2;
    }
    
    newInfo->data = infoData;
    newInfo->next = NULL;

    gnss_info_node_t *current = handle->info;
    
    if (current == NULL)
    {
        handle->info = newInfo;
    }
    else
    {
        gnss_info_node_t *next = current->next;
        while (next != NULL)
        {
            current = next;
            next = current->next;
        }

        current->next = newInfo;
    }
    
    return 0;

}

/****************************************************************************
 * @brief Parse pending messages (from individual message objects) and add them to gnss object message pointers.
 * Will dynamically allocate memory for pointers (each message type only has one) if not already.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 *  1: No pending messages
 ****************************************************************************/
uint8_t gnss_parse_messages(gnss_t *handle)
{
    
    if (handle->pending_messages == 0)
        return 1; // No pending messages
    
    gnss_msg_t *message;

    // Loop through pending messages
    for (uint8_t i = 0; i < handle->pending_messages; i++)
    {

        // Initialize values
        uint8_t msgID = 0xFF;
        uint8_t msgClass = 0xFF;
        uint8_t talkerID = 0;

        // Get next message
        message = &(handle->messages[handle->next_message]);

        if (message->buffer[0] == '$') // NMEA Message
        {

            char identifier[2]; // Talker identifier
            uint8_t formatter[3]; // Sentence formatter

            memcpy(identifier, message->buffer + 1, 0x02);

            if (identifier[0] != 'G') // All valid talkers start with 'G'
                talkerID = 0;
            else
            {

                switch (identifier[1])
                {
                    case 'P':
                        talkerID = GNSS_TALKER_GPS; // GPS
                        #ifdef DEBUG
                        Serial.println("[DEBUG] GPS Message: ");
                        #endif
                        break;
                    
                    case 'A':
                        talkerID = GNSS_TALKER_GALILEO; // Galileo
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Galileo Message: ");
                        #endif
                        break;

                    case 'L':
                        talkerID = GNSS_TALKER_GLONASS; // GLONASS
                        #ifdef DEBUG
                        Serial.println("[DEBUG] GLONASS Message: ");
                        #endif
                        break;

                    case 'B':
                        talkerID = GNSS_TALKER_BEIDOU; // BeiDou
                        #ifdef DEBUG
                        Serial.println("[DEBUG] BeiDou Message: ");
                        #endif
                        break;

                    case 'I':
                        talkerID = GNSS_TALKER_NAVIC; // NavIC
                        #ifdef DEBUG
                        Serial.println("[DEBUG] NavIC Message: ");
                        #endif
                        break;

                    case 'Q':
                        talkerID = GNSS_TALKER_QZSS; // QZSS
                        #ifdef DEBUG
                        Serial.println("[DEBUG] QZSS Message: ");
                        #endif
                        break;

                    case 'N':
                        talkerID = GNSS_TALKER_GNSS; // GNSS (Combined)
                        #ifdef DEBUG
                        Serial.println("[DEBUG] GNSS Message: ");
                        #endif
                        break;
                }
            }

            // Check for PUBX message
            if ((message->buffer[1] == 'P') && (message->buffer[2] == 'U') && (message->buffer[3] == 'B') && (message->buffer[4] == 'X'))
            {
                
                if (gnss_find_next_group(message->buffer, 6, message->length) != 0) // Check if msgClass is empty
                {
                    msgClass = 0xF1;
                    msgID = gnss_ascii_data(message->buffer[6]) * 10 + gnss_ascii_data(message->buffer[7]);
                }
                else // Class is empty
                {
                    msgClass = 0x00;
                    msgID = 0x00;
                }
            }
            else
            {

                memcpy(formatter, message->buffer + 3, 0x03);
                // Generally, you pick the message with the minimum info you need and disable the rest
                switch (formatter[0])
                {
                    case 'D':
                        if ((formatter[1] == 'T') && (formatter[2] == 'M'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_DTM >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_DTM & 0xFF;
                        }
                        break;
                    case 'G':
                        if ((formatter[1] == 'B') && (formatter[2] == 'S'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GBS >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GBS & 0xFF;
                        }
                        else if ((formatter[1] == 'G') && (formatter[2] == 'A'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GGA >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GGA & 0xFF;
                        }
                        else if ((formatter[1] == 'L') && (formatter[2] == 'L'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GLL >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GLL & 0xFF;
                        }
                        else if ((formatter[1] == 'N') && (formatter[2] == 'S'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GNS >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GNS & 0xFF;
                        }
                        else if ((formatter[1] == 'R') && (formatter[2] == 'S'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GRS >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GRS & 0xFF;
                        }
                        else if ((formatter[1] == 'S') && (formatter[2] == 'A'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GSA >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GSA & 0xFF;
                        }
                        else if ((formatter[1] == 'S') && (formatter[2] == 'T'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GST >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GST & 0xFF;
                        }
                        else if ((formatter[1] == 'S') && (formatter[2] == 'V'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_GSV >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_GSV & 0xFF;
                        }
                        break;
                    case 'R':
                        if ((formatter[1] == 'L') && (formatter[2] == 'M'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_RLM >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_RLM & 0xFF;
                        }
                        else if ((formatter[1] == 'M') && (formatter[2] == 'C'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_RMC >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_RMC & 0xFF;
                        }
                        break;
                    case 'T':
                        if ((formatter[1] == 'X') && (formatter[2] == 'T'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_TXT >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_TXT & 0xFF;
                        }
                        break;
                    case 'V':
                        if ((formatter[1] == 'L') && (formatter[2] == 'W'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_VLW >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_VLW & 0xFF;
                        }
                        else if ((formatter[1] == 'T') && (formatter[2] == 'G'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_VTG >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_VTG & 0xFF;
                        }
                        break;
                    case 'Z':
                        if ((formatter[1] == 'D') && (formatter[2] == 'A'))
                        {
                            msgClass = (GNSS_NMEA_STANDARD_ZDA >> 8) & 0xFF;
                            msgID = GNSS_NMEA_STANDARD_ZDA & 0xFF;
                        }
                        break;
                    default:
                        msgID = 0xFF;
                        break;
                }
            }

        }
        else if ((message->buffer[0] == 0xB5) && (message->buffer[1] == 0x62)) // UBX Message
        {

            msgClass = message->buffer[2];
            msgID = message->buffer[3];

        }
        else
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to get msgClass & msgID for the following: ");
            for (uint8_t i = 0; i < message->length; i++)
            {
                Serial.print(message->buffer[i]);
            }
            Serial.println();
            #endif
        }

        uint32_t timestamp;

        uint8_t k = 0; // Start of next message
        uint8_t j = 0; // Current cursor

        uint8_t offset = 0; // Preamble + everything before payload

        switch (msgClass << 8 | msgID)
        {
            case 0xFFFF:
                #ifdef DEBUG
                Serial.println("[DEBUG] Unknown message type");
                #endif
                break;
            case GNSS_NMEA_STANDARD_DTM:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] DTM");
                #endif
                j = 7;
                k = 7;
                // Allocate memory if not already done so
                if (handle->nmeaStdDtm == NULL)
                {
                    gnss_nmea_std_dtm_t *ptr = (gnss_nmea_std_dtm_t *)malloc(sizeof(gnss_nmea_std_dtm_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-DTM message");
                        #endif
                        break;
                    }

                    handle->nmeaStdDtm = ptr;
                }

                gnss_nmea_std_dtm_t *msg = handle->nmeaStdDtm; // Simplify call

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                // Loop through message groups
                for (uint8_t group = 1; group < 9; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get local datum code
                                memcpy(msg->datum, message->buffer + j, 3);
                                break;
                            case 2: // Get subDatum (not used above protocol 14.00)
                                break;
                            case 3: // Get Latitude offset (minutes)
                                msg->latOffset = parse_decimal_string(message->buffer, j, k) / 60.0;
                                break;
                            case 4: // Get North/South notation for Latitude offset
                                if (message->buffer[j] == 'S')
                                    msg->latOffset *= -1.0;
                                break;
                            case 5: // Get Longitude offset (minutes)
                                msg->lonOffset = parse_decimal_string(message->buffer, j, k) / 60.0;
                                break;
                            case 6: // Get East/West notation for Longitude offset
                                if (message->buffer[j] == 'W')
                                    msg->lonOffset *= -1.0;
                                break;
                            case 7: // Get Altitude offset
                                msg->altOffset = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Get Reference Datum
                                memcpy(msg->refDatum, message->buffer + j, 3);
                                break;

                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get local datum code
                                memset(msg->datum, 0, 3);
                                break;
                            case 2: // Get subDatum (not used above protocol 14.00)
                                break;
                            case 3: // Get Latitude offset (minutes)
                                msg->latOffset = 0;
                                break;
                            case 4: // Get North/South notation for Latitude offset
                                break;
                            case 5: // Get Longitude offset (minutes)
                                msg->lonOffset = 0;
                                break;
                            case 6: // Get East/West notation for Longitude offset
                                break;
                            case 7: // Get Altitude offset
                                msg->altOffset = 0;
                                break;
                            case 8: // Get Reference Datum
                                memset(msg->refDatum, 0, 3);
                                break;

                        }
                    }
                    
                    j = k + 1; // Move to checksum

                }
                #ifdef DEBUG
                Serial.print("[DEBUG] Datum: "); Serial.print(msg->datum[0]); Serial.print(msg->datum[1]); Serial.println(msg->datum[2]);
                Serial.print("[DEBUG] Latitude Offset: "); Serial.println(msg->latOffset, 8);
                Serial.print("[DEBUG] Longitude Offset: "); Serial.println(msg->lonOffset, 8);
                Serial.print("[DEBUG] Altitude Offset: "); Serial.println(msg->altOffset);
                Serial.print("[DEBUG] Reference Datum: "); Serial.print(msg->refDatum[0]); Serial.print(msg->refDatum[1]); Serial.println(msg->refDatum[2]);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_GBS:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GBS");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGbs == NULL)
                {
                    gnss_nmea_std_gbs_t *ptr = (gnss_nmea_std_gbs_t *)malloc(sizeof(gnss_nmea_std_gbs_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GBS message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGbs = ptr;
                }

                gnss_nmea_std_gbs_t *msg = handle->nmeaStdGbs;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 11; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 2: // Get Latitude error
                                msg->errLat = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 3: // Get Longitude error
                                msg->errLon = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 4: // Get Altitude error
                                msg->errAlt = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 5: // Get most likely failed satellite
                                msg->svID = gnss_ascii_data(message->buffer[j]) * 10.0 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 6: // Probability of missed detection, not supported
                                break;
                            case 7: // Get bias of most likely failed satellite
                                msg->bias = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Get std dev of estimated bias
                                msg->stddev = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 9: // Get systemID
                                msg->systemID = gnss_ascii_data(message->buffer[j]);
                                break;
                            case 10: // Get signalID
                                msg->signalID = gnss_ascii_data(message->buffer[j]);
                                break;

                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 2: // Get Latitude error
                                msg->errLat = 0;
                                break;
                            case 3: // Get Longitude error
                                msg->errLon = 0;
                                break;
                            case 4: // Get Altitude error
                                msg->errAlt = 0;
                                break;
                            case 5: // Get most likely failed satellite
                                msg->svID = 0;
                                break;
                            case 6: // Probability of missed detection, not supported
                                break;
                            case 7: // Get bias of most likely failed satellite
                                msg->bias = 0;
                                break;
                            case 8: // Get std dev of estimated bias
                                msg->stddev = 0;
                                break;
                            case 9: // Get systemID
                                msg->systemID = 0;
                                break;
                            case 10: // Get signalID
                                msg->signalID = 0;
                                break;

                        }
                    }
                    
                    j = k + 1;

                }
                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Latitude Error: "); Serial.println(msg->errLat, 8);
                Serial.print("[DEBUG] Longitude Error: "); Serial.println(msg->errLon, 8);
                Serial.print("[DEBUG] Altitude Error: "); Serial.println(msg->errAlt, 8);
                Serial.print("[DEBUG] SvID: "); Serial.println(msg->svID);
                Serial.print("[DEBUG] Bias: "); Serial.println(msg->bias);
                Serial.print("[DEBUG] Stddev: "); Serial.println(msg->stddev);
                Serial.print("[DEBUG] SystemID: "); Serial.println(msg->systemID);
                Serial.print("[DEBUG] SignalID: "); Serial.println(msg->signalID);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;

            }
            case GNSS_NMEA_STANDARD_GGA:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GGA");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGga == NULL)
                {
                    gnss_nmea_std_gga_t *ptr = (gnss_nmea_std_gga_t *)malloc(sizeof(gnss_nmea_std_gga_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-DTM message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGga = ptr;
                }

                gnss_nmea_std_gga_t *msg = handle->nmeaStdGga;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 15; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 2: // Get Latitude
                                msg->latitude = gnss_ascii_data(message->buffer[j]) * 10.0 + gnss_ascii_data(message->buffer[j + 1]) + (parse_decimal_string(message->buffer, j + 2, k) / 60.0);
                                break;
                            case 3: // Get North/South notation for Latitude
                                if (message->buffer[j] == 'S')
                                    msg->latitude *= -1.0;
                                break;
                            case 4: // Get Longitude
                                msg->longitude = gnss_ascii_data(message->buffer[j]) * 100.0 + gnss_ascii_data(message->buffer[j + 1]) * 10.0 + gnss_ascii_data(message->buffer[j + 2]) + (parse_decimal_string(message->buffer, j + 3, k) / 60.0);
                                break;
                            case 5: // Get East/West notation for Longitude
                                if (message->buffer[j] == 'W')
                                    msg->longitude *= -1.0;
                                break;
                            case 6: // Get Quality
                                msg->quality = gnss_ascii_data(message->buffer[j]);
                                break;
                            case 7: // Get Number of Satellites in View
                                msg->numSV = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 8: // Get HDOP
                                msg->hdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 9: // Get altitude
                                msg->altitude = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 10: // Get altitude Unit
                                break;
                            case 11: // Get geoid separation
                                msg->sep = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 12: // Get geoid separation unit
                                break;
                            case 13: // Get age of differential corrections
                                msg->diffAge = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 14: // Get ID of station providing differential corrections
                                memcpy(msg->diffStation, message->buffer + j, k - j);
                                break;

                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 2: // Get Latitude
                                msg->latitude = 0;
                                break;
                            case 3: // Get North/South notation for Latitude
                                break;
                            case 4: // Get Longitude
                                msg->longitude = 0;
                                break;
                            case 5: // Get East/West notation for Longitude
                                break;
                            case 6: // Get Quality
                                msg->quality = 0;
                                break;
                            case 7: // Get Number of Satellites in View
                                msg->numSV = 0;
                                break;
                            case 8: // Get HDOP
                                msg->hdop = 0;
                                break;
                            case 9: // Get altitude
                                msg->altitude = 0;
                                break;
                            case 10: // Get altitude Unit
                                break;
                            case 11: // Get geoid separation
                                msg->sep = 0;
                                break;
                            case 12: // Get geoid separation unit
                                break;
                            case 13: // Get age of differential corrections
                                msg->diffAge = 0;
                                break;
                            case 14: // Get ID of station providing differential corrections
                                memset(msg->diffStation, 0, 32);
                                break;

                        }
                    }
                    
                    j = k + 1;

                }
                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Latitude: "); Serial.println(msg->latitude, 8);
                Serial.print("[DEBUG] Longitude: "); Serial.println(msg->longitude, 8);
                Serial.print("[DEBUG] Quality: "); Serial.println(msg->quality);
                Serial.print("[DEBUG] HDOP: "); Serial.println(msg->hdop);
                Serial.print("[DEBUG] Altitude: "); Serial.println(msg->altitude);
                Serial.print("[DEBUG] Geoid Separation: "); Serial.println(msg->sep);
                Serial.print("[DEBUG] Differential Age: "); Serial.println(msg->diffAge);
                Serial.print("[DEBUG] Differential Station: "); for (uint8_t i = 0; i < 32; i++) {Serial.print((char)msg->diffStation[i]);} Serial.println();
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_GLL:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GLL");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGll == NULL)
                {
                    gnss_nmea_std_gll_t *ptr = (gnss_nmea_std_gll_t *)malloc(sizeof(gnss_nmea_std_gll_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GLL message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGll = ptr;
                }

                gnss_nmea_std_gll_t *msg = handle->nmeaStdGll;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 8; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get Latitude
                                msg->latitude = gnss_ascii_data(message->buffer[j]) * 10.0 + gnss_ascii_data(message->buffer[j + 1]) + (parse_decimal_string(message->buffer, j + 2, k) / 60.0);
                                break;
                            case 2: // Get North/South notation for Latitude
                                if (message->buffer[j] == 'S')
                                    msg->latitude *= -1.0;
                                break;
                            case 3: // Get Longitude
                                msg->longitude = gnss_ascii_data(message->buffer[j]) * 100.0 + gnss_ascii_data(message->buffer[j + 1]) * 10.0 + gnss_ascii_data(message->buffer[j + 2]) + (parse_decimal_string(message->buffer, j + 3, k) / 60.0);
                                break;
                            case 4: // Get East/West notation for Longitude
                                if (message->buffer[j] == 'W')
                                    msg->longitude *= -1.0;
                                break;
                            case 5: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 6:// Get data validity status
                                msg->status = message->buffer[j];
                                break;
                            case 7: // Get Positioning Mode
                                msg->posMode = message->buffer[j];
                                break;

                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get Latitude
                                msg->latitude = 0;
                                break;
                            case 2: // Get North/South notation for Latitude
                                break;
                            case 3: // Get Longitude
                                msg->longitude = 0;
                                break;
                            case 4: // Get East/West notation for Longitude
                                break;
                            case 5: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 6:// Get data validity status
                                msg->status = 0;
                                break;
                            case 7: // Get Positioning Mode
                                msg->posMode = 0;
                                break;

                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Status: "); Serial.println(msg->status);
                Serial.print("[DEBUG] PosMode: "); Serial.println(msg->posMode);
                Serial.print("[DEBUG] Latitude: "); Serial.println(msg->latitude, 8);
                Serial.print("[DEBUG] Longitude: "); Serial.println(msg->longitude, 8);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_GNS:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GNS");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGns == NULL)
                {
                    gnss_nmea_std_gns_t *ptr = (gnss_nmea_std_gns_t *)malloc(sizeof(gnss_nmea_std_gns_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GNS message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGns = ptr;
                }

                gnss_nmea_std_gns_t *msg = handle->nmeaStdGns;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 14; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 2: // Get Latitude
                                msg->latitude = gnss_ascii_data(message->buffer[j]) * 10.0 + gnss_ascii_data(message->buffer[j + 1]) + (parse_decimal_string(message->buffer, j + 2, k) / 60.0);
                                break;
                            case 3: // Get North/South notation for Latitude
                                if (message->buffer[j] == 'S')
                                    msg->latitude *= -1.0;
                                break;
                            case 4: // Get Longitude
                                msg->longitude = gnss_ascii_data(message->buffer[j]) * 100.0 + gnss_ascii_data(message->buffer[j + 1]) * 10.0 + gnss_ascii_data(message->buffer[j + 2]) + (parse_decimal_string(message->buffer, j + 3, k) / 60.0);
                                break;
                            case 5: // Get East/West notation for Longitude
                                if (message->buffer[j] == 'W')
                                    msg->longitude *= -1.0;
                                break;
                            case 6: // Get Positioning Mode
                                msg->posMode = message->buffer[j];
                                break;
                            case 7: // Get Number of Satellites in View
                                msg->numSV = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 8: // Get HDOP
                                msg->hdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 9: // Get altitude
                                msg->altitude = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 10: // Get geoid separation
                                msg->sep = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 11: // Get age of differential corrections
                                msg->diffAge = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 12: // Get ID of station providing differential corrections
                                memcpy(msg->diffStation, message->buffer + j, k - j);
                                break;
                            case 13: // Get navStatus
                                msg->navStatus = message->buffer[j];
                                break;

                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 2: // Get Latitude
                                msg->latitude = 0;
                                break;
                            case 3: // Get North/South notation for Latitude
                                break;
                            case 4: // Get Longitude
                                msg->longitude = 0;
                                break;
                            case 5: // Get East/West notation for Longitude
                                break;
                            case 6: // Get Positioning Mode
                                msg->posMode = 0;
                                break;
                            case 7: // Get Number of Satellites in View
                                msg->numSV = 0;
                                break;
                            case 8: // Get HDOP
                                msg->hdop = 0;
                                break;
                            case 9: // Get altitude
                                msg->altitude = 0;
                                break;
                            case 10: // Get geoid separation
                                msg->sep = 0;
                                break;
                            case 11: // Get age of differential corrections
                                msg->diffAge = 0;
                                break;
                            case 12: // Get ID of station providing differential corrections
                                memset(msg->diffStation, 0, 32);
                                break;
                            case 13: // Get navStatus
                                msg->navStatus = 0;
                                break;

                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Latitude: "); Serial.println(msg->latitude, 8);
                Serial.print("[DEBUG] Longitude: "); Serial.println(msg->longitude, 8);
                Serial.print("[DEBUG] posMode: "); Serial.println(msg->posMode);
                Serial.print("[DEBUG] numSV: "); Serial.println(msg->numSV);
                Serial.print("[DEBUG] HDOP: "); Serial.println(msg->hdop);
                Serial.print("[DEBUG] Altitude: "); Serial.print(msg->altitude);
                Serial.print("[DEBUG] Geoid Separation: "); Serial.print(msg->sep);
                Serial.print("[DEBUG] Differential Age: "); Serial.println(msg->diffAge);
                Serial.print("[DEBUG] Differential Station: "); for (uint8_t i = 0; i < 32; i++) {Serial.print((char)msg->diffStation[i]);} Serial.println();
                Serial.print("[DEBUG] navStatus: "); Serial.println(msg->navStatus);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif

                break;
            }
            case GNSS_NMEA_STANDARD_GRS:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GRS");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGrs == NULL)
                {
                    gnss_nmea_std_grs_t *ptr = (gnss_nmea_std_grs_t *)malloc(sizeof(gnss_nmea_std_grs_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GRS message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGrs = ptr;

                    // Must initialize array of pointers
                    for (uint8_t i = 0; i < 8; i++)
                        handle->nmeaStdGrs->grs[i] = NULL;

                }

                if (handle->nmeaStdGrs->grs[talkerID] == NULL)
                {
                    gnss_grs_t *ptr = (gnss_grs_t *)malloc(sizeof(gnss_grs_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GRS message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGrs->grs[talkerID] = ptr;
                }

                gnss_grs_t *msg = handle->nmeaStdGrs->grs[talkerID];

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 17; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 2: // Get Mode (always 1)
                                break;
                            case 3: // Get Range Residual
                            case 4: // Get Range Residual
                            case 5: // Get Range Residual
                            case 6: // Get Range Residual
                            case 7: // Get Range Residual
                            case 8: // Get Range Residual
                            case 9: // Get Range Residual
                            case 10: // Get Range Residual
                            case 11: // Get Range Residual
                            case 12: // Get Range Residual
                            case 13: // Get Range Residual
                            case 14: // Get Range Residual
                                msg->residual[group - 3] = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 15: // Get System ID
                                msg->systemID = gnss_ascii_data(message->buffer[j]);
                                break;
                            case 16: // Get Signal ID
                                msg->signalID = gnss_ascii_data(message->buffer[j]);
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 2: // Get Mode (always 1)
                                break;
                            case 3: // Get Range Residual
                            case 4: // Get Range Residual
                            case 5: // Get Range Residual
                            case 6: // Get Range Residual
                            case 7: // Get Range Residual
                            case 8: // Get Range Residual
                            case 9: // Get Range Residual
                            case 10: // Get Range Residual
                            case 11: // Get Range Residual
                            case 12: // Get Range Residual
                            case 13: // Get Range Residual
                            case 14: // Get Range Residual
                                msg->residual[group - 3] = 0;
                                break;
                            case 15: // Get System ID
                                msg->systemID = 0;
                                break;
                            case 16: // Get Signal ID
                                msg->signalID = 0;
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Residual: "); for (uint8_t i = 0; i < 12; i++) {Serial.print(msg->residual[i]);} Serial.println();
                Serial.print("[DEBUG] SystemID: "); Serial.println(msg->systemID);
                Serial.print("[DEBUG] SignalID: "); Serial.println(msg->signalID);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif

                break;
            }
            case GNSS_NMEA_STANDARD_GSA:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GSA");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGsa == NULL)
                {
                    gnss_nmea_std_gsa_t *ptr = (gnss_nmea_std_gsa_t *)malloc(sizeof(gnss_nmea_std_gsa_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GSA message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGsa = ptr;
                    
                    // Must initialize array of pointers
                    for (uint8_t i = 0; i < 8; i++)
                        handle->nmeaStdGsa->gsa[i] = NULL;
                    
                }
                
                if (handle->nmeaStdGsa->gsa[talkerID] == NULL)
                {
                    gnss_gsa_t *ptr = (gnss_gsa_t *)malloc(sizeof(gnss_gsa_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GSA ptr");
                        #endif
                        break;
                    }

                    handle->nmeaStdGsa->gsa[talkerID] = ptr;
                }

                gnss_gsa_t *msg = handle->nmeaStdGsa->gsa[talkerID];
                
                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;
                
                for (uint8_t group = 1; group < 19; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get Operation Mode
                                msg->opMode = message->buffer[j];
                                break;
                            case 2: // Get NavMode
                                msg->navMode = gnss_ascii_data(message->buffer[j]);
                                break;
                            case 3: // Satellite Number
                            case 4: // Satellite Number
                            case 5: // Satellite Number
                            case 6: // Satellite Number
                            case 7: // Satellite Number
                            case 8: // Satellite Number
                            case 9: // Satellite Number
                            case 10: // Satellite Number
                            case 11: // Satellite Number
                            case 12: // Satellite Number
                            case 13: // Satellite Number
                            case 14: // Satellite Number
                                msg->svID[group - 3] = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 15: // Get PDOP
                                msg->pdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 16: // Get HDOP
                                msg->hdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 17: // Get VDOP
                                msg->vdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 18: // Get systemId
                                msg->systemID = gnss_ascii_data(message->buffer[j]);
                                break;

                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get Operation Mode
                                msg->opMode = 0;
                                break;
                            case 2: // Get NavMode
                                msg->navMode = 0;
                                break;
                            case 3: // Satellite Number
                            case 4: // Satellite Number
                            case 5: // Satellite Number
                            case 6: // Satellite Number
                            case 7: // Satellite Number
                            case 8: // Satellite Number
                            case 9: // Satellite Number
                            case 10: // Satellite Number
                            case 11: // Satellite Number
                            case 12: // Satellite Number
                            case 13: // Satellite Number
                            case 14: // Satellite Number
                                msg->svID[group - 3] = 0;
                                break;
                            case 15: // Get PDOP
                                msg->pdop = 0;
                                break;
                            case 16: // Get HDOP
                                msg->hdop = 0;
                                break;
                            case 17: // Get VDOP
                                msg->vdop = 0;
                                break;
                            case 18: // Get systemId
                                msg->systemID = 0;
                                break;

                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] OpMode: "); Serial.println(msg->opMode);
                Serial.print("[DEBUG] NavMode: "); Serial.println(msg->navMode);
                Serial.print("[DEBUG] SvID: "); for (uint8_t i = 0; i < 12; i++) {Serial.print(msg->svID[i]); Serial.print(" ");} Serial.println();
                Serial.print("[DEBUG] PDOP: "); Serial.println(msg->pdop);
                Serial.print("[DEBUG] HDOP: "); Serial.println(msg->hdop);
                Serial.print("[DEBUG] VDOP: "); Serial.println(msg->vdop);
                Serial.print("[DEBUG] SystemID: "); Serial.println(msg->systemID);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_GST:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GST");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGst == NULL)
                {
                    gnss_nmea_std_gst_t *ptr = (gnss_nmea_std_gst_t *)malloc(sizeof(gnss_nmea_std_gst_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GST message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGst = ptr;
                }

                gnss_nmea_std_gst_t *msg = handle->nmeaStdGst;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 9; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 2: // Get RangeRMS
                                msg->rangeRms = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 3: // Get stdMajor
                                msg->stdMajor = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 4: // Get stdMinor
                                msg->stdMinor = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 5: // Get orient
                                msg->orient = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 6: // Get stdLat
                                msg->stdLat = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 7: // Get stdLong
                                msg->stdLong = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Get stdAlt
                                msg->stdAlt = parse_decimal_string(message->buffer, j, k);
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 2: // Get RangeRMS
                                msg->rangeRms = 0;
                                break;
                            case 3: // Get stdMajor
                                msg->stdMajor = 0;
                                break;
                            case 4: // Get stdMinor
                                msg->stdMinor = 0;
                                break;
                            case 5: // Get orient
                                msg->orient = 0;
                                break;
                            case 6: // Get stdLat
                                msg->stdLat = 0;
                                break;
                            case 7: // Get stdLong
                                msg->stdLong = 0;
                                break;
                            case 8: // Get stdAlt
                                msg->stdAlt = 0;
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] RangeRMS: "); Serial.println(msg->rangeRms);
                Serial.print("[DEBUG] StdMajor: "); Serial.println(msg->stdMajor);
                Serial.print("[DEBUG] StdMinor: "); Serial.println(msg->stdMinor);
                Serial.print("[DEBUG] Orient: "); Serial.println(msg->orient);
                Serial.print("[DEBUG] StdLat: "); Serial.println(msg->stdLat);
                Serial.print("[DEBUG] StdLong: "); Serial.println(msg->stdLong);
                Serial.print("[DEBUG] StdAlt: "); Serial.println(msg->stdAlt);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_GSV:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GSV");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdGsv == NULL)
                {
                    gnss_nmea_std_gsv_t *ptr = (gnss_nmea_std_gsv_t *)malloc(sizeof(gnss_nmea_std_gsv_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GSV message");
                        #endif
                        break;
                    }

                    handle->nmeaStdGsv = ptr;
                }

                gnss_nmea_std_gsv_t *msg = handle->nmeaStdGsv;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 21; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get Number of messages
                                msg->numMsg = gnss_ascii_data(message->buffer[j]);
                                break;
                            case 2: // Get Message Number
                                msg->msgNum = gnss_ascii_data(message->buffer[j]);
                                break;
                            case 3: // Get Number of known satellites in view
                                msg->numSV = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                if (msg->numSV == 0)
                                    group = 19;
                                break;
                            case 4:
                                msg->sv[0].svID = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 5:
                                msg->sv[0].elv = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 6:
                                msg->sv[0].az = gnss_ascii_data(message->buffer[j]) * 100 + gnss_ascii_data(message->buffer[j + 1]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 7:
                                msg->sv[0].cno = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                if (msg->numSV < 2)
                                    group = 19;
                                break;
                            case 8:
                                msg->sv[1].svID = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 9:
                                msg->sv[1].elv = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 10:
                                msg->sv[1].az = gnss_ascii_data(message->buffer[j]) * 100 + gnss_ascii_data(message->buffer[j + 1]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 11:
                                msg->sv[1].cno = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                if (msg->numSV < 3)
                                    group = 19;
                                break;
                            case 12:
                                msg->sv[2].svID = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 13:
                                msg->sv[2].elv = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 14:
                                msg->sv[2].az = gnss_ascii_data(message->buffer[j]) * 100 + gnss_ascii_data(message->buffer[j + 1]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 15:
                                msg->sv[2].cno = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                if (msg->numSV < 4)
                                    group = 19;
                                break;
                            case 16:
                                msg->sv[3].svID = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 17:
                                msg->sv[3].elv = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 18:
                                msg->sv[3].az = gnss_ascii_data(message->buffer[j]) * 100 + gnss_ascii_data(message->buffer[j + 1]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 19:
                                msg->sv[3].cno = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 20:
                                msg->signalID = gnss_ascii_data(message->buffer[j]);
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get Number of messages
                                msg->numMsg = 0;
                                break;
                            case 2: // Get Message Number
                                msg->msgNum = 0;
                                break;
                            case 3: // Get Number of known satellites in view
                                msg->numSV = 0;
                                break;
                            case 4:
                                msg->sv[0].svID = 0;
                                break;
                            case 5:
                                msg->sv[0].elv = 0;
                                break;
                            case 6:
                                msg->sv[0].az = 0;
                                break;
                            case 7:
                                msg->sv[0].cno = 0;
                                if (msg->numSV < 2) // Should be at least one message
                                    group = 19;
                                break;
                            case 8:
                                msg->sv[1].svID = 0;
                                break;
                            case 9:
                                msg->sv[1].elv = 0;
                                break;
                            case 10:
                                msg->sv[1].az = 0;
                                break;
                            case 11:
                                msg->sv[1].cno = 0;
                                if (msg->numSV < 3)
                                    group = 19;
                                break;
                            case 12:
                                msg->sv[2].svID = 0;
                                break;
                            case 13:
                                msg->sv[2].elv = 0;
                                break;
                            case 14:
                                msg->sv[2].az = 0;
                                break;
                            case 15:
                                msg->sv[2].cno = 0;
                                if (msg->numSV < 4)
                                    group = 19;
                                break;
                            case 16:
                                msg->sv[3].svID = 0;
                                break;
                            case 17:
                                msg->sv[3].elv = 0;
                                break;
                            case 18:
                                msg->sv[3].az = 0;
                                break;
                            case 19:
                                msg->sv[3].cno = 0;
                                break;
                            case 20:
                                msg->signalID = 0;
                                break;
                        }
                    }
                    
                    j = k + 1;

                }
                #ifdef DEBUG
                Serial.print("[DEBUG] NumMsg: "); Serial.println(msg->numMsg);
                Serial.print("[DEBUG] MsgNum: "); Serial.println(msg->msgNum);
                Serial.print("[DEBUG] NumSV: "); Serial.println(msg->numSV);
                for (uint8_t i = 0; i < msg->numSV; i++)
                {
                    Serial.print("[DEBUG] SvID: "); Serial.println(msg->sv[i].svID);
                    Serial.print("[DEBUG] Elevation: "); Serial.println(msg->sv[i].elv);
                    Serial.print("[DEBUG] Azimuth: "); Serial.println(msg->sv[i].az);
                    Serial.print("[DEBUG] CNO: "); Serial.println(msg->sv[i].cno);
                }
                Serial.print("[DEBUG] SignalID: "); Serial.println(msg->signalID);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_RLM:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] RLM");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdRlm == NULL)
                {
                    gnss_nmea_std_rlm_t *ptr = (gnss_nmea_std_rlm_t *)malloc(sizeof(gnss_nmea_std_rlm_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-RLM message");
                        #endif
                        break;
                    }

                    handle->nmeaStdRlm = ptr;
                }

                gnss_nmea_std_rlm_t *msg = handle->nmeaStdRlm;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 5; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get Beacon ID
                                memcpy(msg->beacon, message->buffer + j, 15);
                                break;
                            case 2: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 3: // Get Code
                                msg->code = message->buffer[j];
                                break;
                            case 4: // Get Body
                                memcpy(msg->body, message->buffer + j, 4);
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get Beacon ID
                                memset(msg->beacon, 0, 15);
                                break;
                            case 2: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 3: // Get Code
                                msg->code = 0;
                                break;
                            case 4: // Get Body
                                memset(msg->body, 0, 4);
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Beacon: "); for (uint8_t i = 0; i < 15; i++) {Serial.print((char)msg->beacon[i]);} Serial.println();
                Serial.print("[DEBUG] Code: "); Serial.println((char)msg->code);
                Serial.print("[DEBUG] Body: "); for (uint8_t i = 0; i < 4; i++) {Serial.print((char)msg->body[i]);} Serial.println();
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_RMC:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] RMC");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdRmc == NULL)
                {
                    gnss_nmea_std_rmc_t *ptr = (gnss_nmea_std_rmc_t *)malloc(sizeof(gnss_nmea_std_rmc_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-RMC message");
                        #endif
                        break;
                    }

                    handle->nmeaStdRmc = ptr;
                }

                gnss_nmea_std_rmc_t *msg = handle->nmeaStdRmc;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 14; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 2: // Get data validity status
                                msg->status = message->buffer[j];
                                break;
                            case 3: // Get Latitude
                                msg->latitude = gnss_ascii_data(message->buffer[j]) * 10.0 + gnss_ascii_data(message->buffer[j + 1]) + (parse_decimal_string(message->buffer, j + 2, k) / 60.0);
                                break;
                            case 4: // Get North/South notation for Latitude
                                if (message->buffer[j] == 'S')
                                    msg->latitude *= -1.0;
                                break;
                            case 5: // Get Longitude
                                msg->longitude = gnss_ascii_data(message->buffer[j]) * 100.0 + gnss_ascii_data(message->buffer[j + 1]) * 10.0 + gnss_ascii_data(message->buffer[j + 2]) + (parse_decimal_string(message->buffer, j + 3, k) / 60.0);
                                break;
                            case 6: // Get East/West notation for Longitude
                                if (message->buffer[j] == 'W')
                                    msg->longitude *= -1.0;
                                break;
                            case 7: // Speed over Ground
                                msg->spd = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Course over Ground
                                msg->cog = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 9: // Get Date
                                msg->dayUTC = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                msg->monthUTC = gnss_ascii_data(message->buffer[j + 2]) * 10 + gnss_ascii_data(message->buffer[j + 3]);
                                msg->yearUTC = gnss_ascii_data(message->buffer[j + 4]) * 10 + gnss_ascii_data(message->buffer[j + 5]);
                                break;
                            case 10: // Get MV
                                msg->mv = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 11: // Get MV E/W
                                if (message->buffer[j] == 'W')
                                    msg->mv *= -1.0;
                                break;
                            case 12: // Get posMode
                                msg->posMode = message->buffer[j];
                                break;
                            case 13: // Get navStatus
                                msg->navStatus = message->buffer[j];
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 2: // Get data validity status
                                msg->status = 0;
                                break;
                            case 3: // Get Latitude
                                msg->latitude = 0;
                                break;
                            case 4: // Get North/South notation for Latitude
                                break;
                            case 5: // Get Longitude
                                msg->longitude = 0;
                                break;
                            case 6: // Get East/West notation for Longitude
                                break;
                            case 7: // Speed over Ground
                                msg->spd = 0;
                                break;
                            case 8: // Course over Ground
                                msg->cog = 0;
                                break;
                            case 9: // Get Date
                                msg->dayUTC = 0;
                                msg->monthUTC = 0;
                                msg->yearUTC = 0;
                                break;
                            case 10: // Get MV
                                msg->mv = 0;
                                break;
                            case 11: // Get MV E/W
                                break;
                            case 12: // Get posMode
                                msg->posMode = 0;
                                break;
                            case 13: // Get navStatus
                                msg->navStatus = 0;
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Status: "); Serial.println(msg->status);
                Serial.print("[DEBUG] Latitude: "); Serial.println(msg->latitude, 8);
                Serial.print("[DEBUG] Longitude: "); Serial.println(msg->longitude, 8);
                Serial.print("[DEBUG] Speed over Ground: "); Serial.println(msg->spd, 4);
                Serial.print("[DEBUG] Course over Ground: "); Serial.println(msg->cog, 4);
                Serial.print("[DEBUG] Date: "); Serial.print(msg->dayUTC); Serial.print("-"); Serial.print(msg->monthUTC); Serial.print("-"); Serial.println(msg->yearUTC);
                Serial.print("[DEBUG] Mag Variation: "); Serial.println(msg->mv, 4);
                Serial.print("[DEBUG] posMode: "); Serial.println(msg->posMode);
                Serial.print("[DEBUG] navStatus: "); Serial.println(msg->navStatus);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_TXT:
                #ifdef DEBUG
                Serial.println("[DEBUG] TXT");
                #endif
                j = 7;
                k = 7;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_create_info_node(handle, message, GNSS_NMEA_STANDARD_TXT);

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[message->length - 4]); Serial.println((char)message->buffer[message->length - 3]);
                #endif
                break;
            case GNSS_NMEA_STANDARD_VLW:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] VLW");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdVlw == NULL)
                {
                    gnss_nmea_std_vlw_t *ptr = (gnss_nmea_std_vlw_t *)malloc(sizeof(gnss_nmea_std_vlw_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-VLW message");
                        #endif
                        break;
                    }

                    handle->nmeaStdVlw = ptr;
                }

                gnss_nmea_std_vlw_t *msg = handle->nmeaStdVlw;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 9; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get total cumulative water distance (fixed null)
                                break;
                            case 2: // Get water distance units (fixed N)
                                break;
                            case 3: // Get water distance since reset (fixed null)
                                break;
                            case 4: // Get water distance units (fixed N)
                                break;
                            case 5: // Get Total Cumulative Ground Distance
                                msg->tgd = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 6: // Get Ground Distance Units (fixed N)
                                break;
                            case 7: // Get Ground Distance since reset
                                msg->gd = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Get Ground distance units (fixed N)
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get total cumulative water distance (fixed null)
                                break;
                            case 2: // Get water distance units (fixed N)
                                break;
                            case 3: // Get water distance since reset (fixed null)
                                break;
                            case 4: // Get water distance units (fixed N)
                                break;
                            case 5: // Get Total Cumulative Ground Distance
                                msg->tgd = 0;
                                break;
                            case 6: // Get Ground Distance Units (fixed N)
                                break;
                            case 7: // Get Ground Distance since reset
                                msg->gd = 0;
                                break;
                            case 8: // Get Ground distance units (fixed N)
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Total Cumulative Ground Distance: "); Serial.println(msg->tgd);
                Serial.print("[DEBUG] Ground Distance Since Reset: "); Serial.println(msg->gd);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_VTG:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] VTG");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdVtg == NULL)
                {
                    gnss_nmea_std_vtg_t *ptr = (gnss_nmea_std_vtg_t *)malloc(sizeof(gnss_nmea_std_vtg_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-VTG message");
                        #endif
                        break;
                    }

                    handle->nmeaStdVtg = ptr;
                }

                gnss_nmea_std_vtg_t *msg = handle->nmeaStdVtg;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 10; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get Course Over Ground (true)
                                msg->cogt = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 2: // Get Course Over Ground Unit (fixed T)
                                break;
                            case 3: // Get Course Over Ground (magnetic)
                                msg->cogm = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 4: // Get Course Over Ground Unit (fixed M)
                                break;
                            case 5: // Get Speed over Ground (knots)
                                msg->sogn = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 6: // Get Speed over Ground Unit (fixed N)
                                break;
                            case 7: // Get Speed over Ground (kph)
                                msg->sogk = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Get Speed over Ground Unit (fixed K)
                                break;
                            case 9: // Get Position Mode
                                msg->posMode = message->buffer[j];
                                break;

                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get Course Over Ground (true)
                                msg->cogt = 0;
                                break;
                            case 2: // Get Course Over Ground Unit (fixed T)
                                break;
                            case 3: // Get Course Over Ground (magnetic)
                                msg->cogm = 0;
                                break;
                            case 4: // Get Course Over Ground Unit (fixed M)
                                break;
                            case 5: // Get Speed over Ground (knots)
                                msg->sogn = 0;
                                break;
                            case 6: // Get Speed over Ground Unit (fixed N)
                                break;
                            case 7: // Get Speed over Ground (kph)
                                msg->sogk = 0;
                                break;
                            case 8: // Get Speed over Ground Unit (fixed K)
                                break;
                            case 9: // Get Position Mode
                                msg->posMode = 0;
                                break;

                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Course over Ground (True): "); Serial.println(msg->cogt, 4);
                Serial.print("[DEBUG] Course over Ground (Mag): "); Serial.println(msg->cogm, 4);
                Serial.print("[DEBUG] Speed over Ground (Knots): "); Serial.println(msg->sogn, 4);
                Serial.print("[DEBUG] Speed over Ground (km/h): "); Serial.println(msg->sogk, 4);
                Serial.print("[DEBUG] posMode: "); Serial.println((char)msg->posMode);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_STANDARD_ZDA:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] ZDA");
                #endif
                j = 7;
                k = 7;

                if (handle->nmeaStdZda == NULL)
                {
                    gnss_nmea_std_zda_t *ptr = (gnss_nmea_std_zda_t *)malloc(sizeof(gnss_nmea_std_zda_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-ZDA message");
                        #endif
                        break;
                    }

                    handle->nmeaStdZda = ptr;
                }

                gnss_nmea_std_zda_t *msg = handle->nmeaStdZda;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->talkerID = talkerID;
                msg->stale = false;

                for (uint8_t group = 1; group < 7; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 2: // Get Day UTC
                                msg->dayUTC = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 3: // Get Month UTC
                                msg->monthUTC = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 4: // Get Year UTC
                                msg->yearUTC = gnss_ascii_data(message->buffer[j]) * 1000 + gnss_ascii_data(message->buffer[j + 1]) * 100 + gnss_ascii_data(message->buffer[j + 2]) * 10 + gnss_ascii_data(message->buffer[j + 3]);
                                break;
                            case 5: // Get Local time zone hours (fixed 00)
                                break;
                            case 6: // Get Local time zone minutes (fixed 00)
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            case 2: // Get Day UTC
                                msg->dayUTC = 0;
                                break;
                            case 3: // Get Month UTC
                                msg->monthUTC = 0;
                                break;
                            case 4: // Get Year UTC
                                msg->yearUTC = 0;
                                break;
                            case 5: // Get Local time zone hours (fixed 00)
                                break;
                            case 6: // Get Local time zone minutes (fixed 00)
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Date: "); Serial.print(msg->dayUTC); Serial.print("-"); Serial.print(msg->monthUTC); Serial.print("-"); Serial.println(msg->yearUTC);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_PUBX_POSITION:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] POSITION");
                #endif
                j = 6;
                k = 6;

                if (handle->nmeaPubxPos == NULL)
                {
                    gnss_nmea_pubx_pos_t *ptr = (gnss_nmea_pubx_pos_t *)malloc(sizeof(gnss_nmea_pubx_pos_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-POSITION message");
                        #endif
                        break;
                    }

                    handle->nmeaPubxPos = ptr;
                }

                gnss_nmea_pubx_pos_t *msg = handle->nmeaPubxPos;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                for (uint8_t group = 1; group < 21; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->msgID = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                            case 2: // Get Day UTC
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 3: // Get Latitude
                                msg->latitude = gnss_ascii_data(message->buffer[j]) * 10.0 + gnss_ascii_data(message->buffer[j + 1]) + (parse_decimal_string(message->buffer, j + 2, k) / 60.0);
                                break;
                            case 4: // Get North/South notation for Latitude
                                if (message->buffer[j] == 'S')
                                    msg->latitude *= -1.0;
                                break;
                            case 5: // Get Longitude
                                msg->longitude = gnss_ascii_data(message->buffer[j]) * 100.0 + gnss_ascii_data(message->buffer[j + 1]) * 10.0 + gnss_ascii_data(message->buffer[j + 2]) + (parse_decimal_string(message->buffer, j + 3, k) / 60.0);
                                break;
                            case 6: // Get East/West notation for Longitude
                                if (message->buffer[j] == 'W')
                                    msg->longitude *= -1.0;
                                break;
                            case 7: // Altitude above user datum ellipsoid
                                msg->altRef = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Navigation Status
                                msg->navStat[0] = message->buffer[j];
                                msg->navStat[1] = message->buffer[j + 1];
                                break;
                            case 9: // Horizontal Accuracy Estimate
                                msg->hAcc = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 10: // Vertical Accuracy Estimate
                                msg->vAcc = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 11: // Speed over Ground
                                msg->sog = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 12: // Course over Ground
                                msg->cog = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 13: // Vertical Velocity
                                msg->vVel = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 14: // Age of Differential Corrections
                                msg->diffAge = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 15: // Horizontal Dilution of Precision
                                msg->hdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 16: // Vertical Dilution of Precision
                                msg->vdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 17: // Time Dilution of Precision
                                msg->tdop = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 18: // Number of Satellites
                                msg->numSvs = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 19: // Reserved
                                break;
                            case 20: // DR
                                msg->dr = gnss_ascii_data(message->buffer[j]);
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->msgID = 0;
                            case 2: // Get Day UTC
                            {
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            }
                            case 3: // Get Latitude
                                msg->latitude = 0;
                                break;
                            case 4: // Get North/South notation for Latitude
                                break;
                            case 5: // Get Longitude
                                msg->longitude = 0;
                                break;
                            case 6: // Get East/West notation for Longitude
                                break;
                            case 7: // Altitude above user datum ellipsoid
                                msg->altRef = 0;
                                break;
                            case 8: // Navigation Status
                                msg->navStat[0] = 0;
                                msg->navStat[1] = 0;
                                break;
                            case 9: // Horizontal Accuracy Estimate
                                msg->hAcc = 0;
                                break;
                            case 10: // Vertical Accuracy Estimate
                                msg->vAcc = 0;
                                break;
                            case 11: // Speed over Ground
                                msg->sog = 0;
                                break;
                            case 12: // Course over Ground
                                msg->cog = 0;
                                break;
                            case 13: // Vertical Velocity
                                msg->vVel = 0;
                                break;
                            case 14: // Age of Differential Corrections
                                msg->diffAge = 0;
                                break;
                            case 15: // Horizontal Dilution of Precision
                                msg->hdop = 0;
                                break;
                            case 16: // Vertical Dilution of Precision
                                msg->vdop = 0;
                                break;
                            case 17: // Time Dilution of Precision
                                msg->tdop = 0;
                                break;
                            case 18: // Number of Satellites
                                msg->numSvs = 0;
                                break;
                            case 19: // Reserved
                                break;
                            case 20: // DR
                                msg->dr = 0;
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] MsgID: "); Serial.println(msg->msgID);
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] Latitude: "); Serial.println(msg->latitude);
                Serial.print("[DEBUG] Longitude: "); Serial.println(msg->longitude);
                Serial.print("[DEBUG] AltRef: "); Serial.println(msg->altRef);
                Serial.print("[DEBUG] NavStat: "); Serial.print((char)msg->navStat[0]); Serial.println((char)msg->navStat[1]);
                Serial.print("[DEBUG] hAcc: "); Serial.println(msg->hAcc);
                Serial.print("[DEBUG] vAcc: "); Serial.println(msg->vAcc);
                Serial.print("[DEBUG] sog: "); Serial.println(msg->sog);
                Serial.print("[DEBUG] cog: "); Serial.println(msg->cog);
                Serial.print("[DEBUG] vVel: "); Serial.println(msg->vVel);
                Serial.print("[DEBUG] diffAge: "); Serial.println(msg->diffAge);
                Serial.print("[DEBUG] hdop: "); Serial.println(msg->hdop);
                Serial.print("[DEBUG] vdop: "); Serial.println(msg->vdop);
                Serial.print("[DEBUG] tdop: "); Serial.println(msg->tdop);
                Serial.print("[DEBUG] numSvs: "); Serial.println(msg->numSvs);
                Serial.print("[DEBUG] DR: "); Serial.println(msg->dr);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_PUBX_SVSTATUS:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SVSTATUS");
                #endif
                j = 6;
                k = 6;

                if (handle->nmeaPubxSvstatus == NULL)
                {
                    gnss_nmea_pubx_svstatus_t *ptr = (gnss_nmea_pubx_svstatus_t *)malloc(sizeof(gnss_nmea_pubx_svstatus_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-SVSTATUS message");
                        #endif
                        break;
                    }

                    handle->nmeaPubxSvstatus = ptr;
                }

                gnss_nmea_pubx_svstatus_t *msg = handle->nmeaPubxSvstatus;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                for (uint8_t group = 1; group < 3; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get msgID
                                msg->msgID = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 2: // Get Number of Satellites
                                msg->n = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);

                                for (uint8_t i = 0; i < msg->n; i++)
                                {
                                    j = k + 1;
                                    k = gnss_find_next_group(message->buffer, j, message->length);
                                    if (k != j)
                                        msg->sv[i].sv = parse_decimal_string(message->buffer, j, k); // Variable amount of digits
                                    else
                                        msg->sv[i].sv = 0;

                                    j = k + 1;
                                    k = gnss_find_next_group(message->buffer, j, message->length);
                                    if (k != j)
                                        msg->sv[i].status = message->buffer[j];
                                    else
                                        msg->sv[i].status = 0;

                                    j = k + 1;
                                    k = gnss_find_next_group(message->buffer, j, message->length);
                                    if (k != j)
                                        msg->sv[i].az = gnss_ascii_data(message->buffer[j]) * 100 + gnss_ascii_data(message->buffer[j + 1]) * 10 + gnss_ascii_data(message->buffer[j + 2]);
                                    else
                                        msg->sv[i].az = 0;

                                    j = k + 1;
                                    k = gnss_find_next_group(message->buffer, j, message->length);
                                    if (k != j)
                                        msg->sv[i].el = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                    else
                                        msg->sv[i].el = 0;

                                    j = k + 1;
                                    k = gnss_find_next_group(message->buffer, j, message->length);
                                    if (k != j)
                                        msg->sv[i].cno = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                    else
                                        msg->sv[i].cno = 0;

                                    j = k + 1;
                                    k = gnss_find_next_group(message->buffer, j, message->length);
                                    if (k != j)
                                        msg->sv[i].lck = gnss_ascii_data(message->buffer[j]) * 100 + gnss_ascii_data(message->buffer[j + 1]) * 10 + gnss_ascii_data(message->buffer[j + 2]);
                                    else
                                        msg->sv[i].lck = 0;
                                }
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get msgID
                                msg->msgID = 0;
                            case 2: // Get Number of Satellites
                                msg->n = 0;
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] MsgID: "); Serial.println(msg->msgID);
                Serial.print("[DEBUG] n: "); Serial.println(msg->n);
                for (uint8_t i = 0; i < msg->n; i++)
                {
                    Serial.print("[DEBUG] SV: "); Serial.println(msg->sv[i].sv);
                    Serial.print("[DEBUG] s: "); Serial.println((char)msg->sv[i].status);
                    Serial.print("[DEBUG] az: "); Serial.println(msg->sv[i].az);
                    Serial.print("[DEBUG] el: "); Serial.println(msg->sv[i].el);
                    Serial.print("[DEBUG] cno: "); Serial.println(msg->sv[i].cno);
                    Serial.print("[DEBUG] lck: "); Serial.println(msg->sv[i].lck);
                    Serial.println();
                }
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_NMEA_PUBX_TIME:
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIME");
                #endif
                j = 6;
                k = 6;

                if (handle->nmeaPubxTime == NULL)
                {
                    gnss_nmea_pubx_time_t *ptr = (gnss_nmea_pubx_time_t *)malloc(sizeof(gnss_nmea_pubx_time_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-TIME message");
                        #endif
                        break;
                    }

                    handle->nmeaPubxTime = ptr;
                }

                gnss_nmea_pubx_time_t *msg = handle->nmeaPubxTime;

                // Check checksum first
                if (gnss_nmea_checksum(message->buffer, message->length - 5) != ((message->buffer[message->length - 4] << 8) | message->buffer[message->length - 3]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                for (uint8_t group = 1; group < 10; group++)
                {
                    k = gnss_find_next_group(message->buffer, j, message->length);

                    if (k != j)
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->msgID = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                            case 2: // Get Day UTC
                            {
                                uint32_t mils = gnss_parse_time(message->buffer, j);
                                msg->milsUTC = mils % 1000;
                                msg->secondsUTC = (mils / 1000) % 60;
                                msg->minutesUTC = (mils / 60000) % 60;
                                msg->hoursUTC = mils / 3600000;
                                break;
                            }
                            case 3: // Get Date
                                msg->dayUTC = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                msg->monthUTC = gnss_ascii_data(message->buffer[j + 2]) * 10 + gnss_ascii_data(message->buffer[j + 3]);
                                msg->yearUTC = gnss_ascii_data(message->buffer[j + 4]) * 10 + gnss_ascii_data(message->buffer[j + 5]);
                                break;
                            case 4: // Get UTC Time of Week
                                msg->utcTow = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 5: // Get UTC Number of Weeks
                                msg->utcWk = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 6: // Get Leap Seconds
                                msg->leapSec = gnss_ascii_data(message->buffer[j]) * 10 + gnss_ascii_data(message->buffer[j + 1]);
                                break;
                            case 7: // Get Clock Bias
                                msg->clkBias = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 8: // Get Clock Drift
                                msg->clkDrift = parse_decimal_string(message->buffer, j, k);
                                break;
                            case 9: // Get Timepulse Granularity
                                msg->tpGran = parse_decimal_string(message->buffer, j, k);
                                break;
                        }
                    }
                    else
                    {
                        switch (group)
                        {
                            case 1: // Get time
                                msg->msgID = 0;
                            case 2: // Get Day UTC
                            {
                                msg->milsUTC = 0;
                                msg->secondsUTC = 0;
                                msg->minutesUTC = 0;
                                msg->hoursUTC = 0;
                                break;
                            }
                            case 3: // Get Date
                                msg->dayUTC = 0;
                                msg->monthUTC = 0;
                                msg->yearUTC = 0;
                                break;
                            case 4: // Get UTC Time of Week
                                msg->utcTow = 0;
                                break;
                            case 5: // Get UTC Number of Weeks
                                msg->utcWk = 0;
                                break;
                            case 6: // Get Leap Seconds
                                msg->leapSec = 0;
                                break;
                            case 7: // Get Clock Bias
                                msg->clkBias = 0;
                                break;
                            case 8: // Get Clock Drift
                                msg->clkDrift = 0;
                                break;
                            case 9: // Get Timepulse Granularity
                                msg->tpGran = 0;
                                break;
                        }
                    }
                    
                    j = k + 1;

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] MsgID: "); Serial.println(msg->msgID);
                Serial.print("[DEBUG] UTC Time: "); Serial.print(msg->hoursUTC); Serial.print(":"); Serial.print(msg->minutesUTC); Serial.print(":"); Serial.print(msg->secondsUTC); Serial.print(":"); Serial.println(msg->milsUTC);
                Serial.print("[DEBUG] UTC Date: "); Serial.print(msg->dayUTC); Serial.print("-"); Serial.print(msg->monthUTC); Serial.print("-"); Serial.println(msg->yearUTC);
                Serial.print("[DEBUG] utcTow: "); Serial.println(msg->utcTow);
                Serial.print("[DEBUG] utcWk: "); Serial.println(msg->utcWk);
                Serial.print("[DEBUG] leapSec: "); Serial.println(msg->leapSec);
                Serial.print("[DEBUG] clkBias: "); Serial.println(msg->clkBias);
                Serial.print("[DEBUG] clkDrift: "); Serial.println(msg->clkDrift);
                Serial.print("[DEBUG] tpGran: "); Serial.println(msg->tpGran);
                Serial.print("[DEBUG] Checksum: "); Serial.print((char)message->buffer[j]); Serial.println((char)message->buffer[j + 1]);
                #endif
                break;
            }
            case GNSS_UBX_ACK_ACK: //UBX-ACK-ACK
            {
                j = 6;
                offset = 6;
                // Unfortunately, the ACK message only provides the class and ID.
                // So if multiple messages have been sent with the same class and ID, there's no way of knowing which one was confirmed.

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_tx_msg_node_t *current = handle->txMsg;
                gnss_tx_msg_node_t *prev = NULL;
                
                if (current == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Received ACK for no existing message");
                    #endif
                }
                else
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Checking for existing message for ACK...");
                    #endif
                    
                    // Loop through pending nodes until we find the one we're looking for
                    while ((current->data->msgClass != message->buffer[offset]) && (current->data->msgID != message->buffer[offset + 1]))
                    {
                        prev = current;
                        current = current->next;
                        
                        if (current == NULL)
                        {
                            #ifdef DEBUG
                            Serial.println("[DEBUG] Received ACK for no existing message in queue");
                            #endif
                            break;
                        }
                    }

                    if (current != NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Received ACK for existing message");
                        #endif

                        // Free nodes
                        if (prev == NULL) // Head
                        {
                            if (current->next == NULL) // No more messages
                            {
                                free(handle->txMsg->data);
                                free(handle->txMsg);
                                handle->txMsg = NULL;
                            }
                            else
                            {
                                handle->txMsg = current->next;
                                free(current->data);
                                free(current);
                            }

                        }
                        else
                        {
                            prev->next = current->next;
                            free(current->data);
                            free(current);
                        }

                    }

                }
                #ifdef DEBUG
                Serial.print("[DEBUG] Class of ACK Message: "); Serial.println(message->buffer[offset]);
                j = offset + 1;
                Serial.print("[DEBUG] ID of ACK Message: "); Serial.println(message->buffer[offset + 1]);
                j += 1;
                #endif
                break;
            }
            case GNSS_UBX_ACK_NACK: //UBX-ACK-NACK
            {
                j = 6;
                offset = 6;
                // Unfortunately, the NACK message only provides the class and ID.
                // So if multiple messages have been sent with the same class and ID, there's no way of knowing which one was NACK.

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_tx_msg_node_t *current = handle->txMsg;
                gnss_tx_msg_node_t *prev = NULL;

                if (current == NULL)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Received NACK for no existing message");
                    #endif
                }
                else
                {
                    while ((current->data->msgClass != message->buffer[offset]) && (current->data->msgID != message->buffer[offset + 1]))
                    {
                        prev = current;
                        current = current->next;
                        
                        if (current == NULL)
                        {
                            #ifdef DEBUG
                            Serial.println("[DEBUG] Received NACK for no existing message in queue");
                            #endif
                            break;
                        }
                    }

                    if (current != NULL)
                    {
                        
                        if (current->data->retry < 5)
                        {

                            #ifdef DEBUG
                            Serial.println("[DEBUG] Received NACK for existing message. Resending message");
                            #endif

                            gnss_ubx_msg(handle, current->data->msgClass, current->data->msgID, current->data->length, current->data->payload, 0);
                            current->data->retry++;
                        }
                        else // Failed too many attempts
                        {

                            #ifdef DEBUG
                            Serial.println("[DEBUG] Received NACK for existing message. Failed on too many attempts");
                            #endif

                            if (prev == NULL) // Head
                            {

                                free(handle->txMsg->data);
                                free(handle->txMsg);
                                handle->txMsg = NULL;

                            }

                            else
                            {
                                prev->next = current->next;
                                free(current->data);
                                free(current);
                            }
                        }

                    }

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Class of NACK Message: "); Serial.println(message->buffer[offset]);
                j = offset + 1;
                Serial.print("[DEBUG] ID of NACK Message: "); Serial.println(message->buffer[offset + 1]);
                j += 1;
                #endif
                break;
            }
            case GNSS_UBX_CFG_VALGET: // UBX-CFG-VALGET
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-CFG-VALGET");
                #endif

                if (handle->cfgMsg == NULL)
                {
                    gnss_cfg_msg_t *ptr = (gnss_cfg_msg_t *)malloc(sizeof(gnss_cfg_msg_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-CFG-VALGET message");
                        #endif
                        break;
                    }

                    handle->cfgMsg = ptr;
                }

                gnss_cfg_msg_t *msg = handle->cfgMsg;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset];
                msg->layer = message->buffer[offset + 1];
                msg->position = getUByte16_LEnd(message->buffer, offset + 2);

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] layer: "); Serial.println(msg->layer);
                Serial.print("[DEBUG] position: "); Serial.println(msg->position);
                Serial.print("[DEBUG] data: ");
                #endif

                msg->dataLength = message->length - 2 - offset - 4;

                for (uint16_t i = 0; i < msg->dataLength; i++)
                {
                    msg->data[i] = message->buffer[offset + 4 + i];

                    #ifdef DEBUG
                    Serial.print(msg->data[i]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.println();
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_AOPSTATUS: // UBX-NAV-AOPSTATUS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-AOPSTATUS");
                #endif

                if (handle->ubxNavAopstatus == NULL)
                {
                    gnss_ubx_nav_aopstatus_t *ptr = (gnss_ubx_nav_aopstatus_t *)malloc(sizeof(gnss_ubx_nav_aopstatus_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-AOPSTATUS message");
                        #endif
                        break;
                    }

                    handle->ubxNavAopstatus = ptr;
                }

                gnss_ubx_nav_aopstatus_t *msg = handle->ubxNavAopstatus;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->useAOP = message->buffer[offset + 4] & 0x01;
                msg->status = message->buffer[offset + 5];

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] useAOP: "); Serial.println(msg->useAOP);
                Serial.print("[DEBUG] Status: "); Serial.println(msg->status);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_CLOCK: // UBX-NAV-CLOCK
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-CLOCK");
                #endif

                if (handle->ubxNavClock == NULL)
                {
                    gnss_ubx_nav_clock_t *ptr = (gnss_ubx_nav_clock_t *)malloc(sizeof(gnss_ubx_nav_clock_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-CLOCK message");
                        #endif
                        break;
                    }

                    handle->ubxNavClock = ptr;
                }

                gnss_ubx_nav_clock_t *msg = handle->ubxNavClock;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }
                
                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->bias = getIByte32_LEnd(message->buffer, offset + 4);
                msg->drift = getIByte32_LEnd(message->buffer, offset + 8);
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 12);
                msg->fAcc = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] clkB: "); Serial.println(msg->bias);
                Serial.print("[DEBUG] clkD: "); Serial.println(msg->drift);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] fAcc: "); Serial.println(msg->fAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_COV: // UBX-NAV-COV
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-COV");
                #endif

                if (handle->ubxNavCov == NULL)
                {
                    gnss_ubx_nav_cov_t *ptr = (gnss_ubx_nav_cov_t *)malloc(sizeof(gnss_ubx_nav_cov_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-COV message");
                        #endif
                        break;
                    }

                    handle->ubxNavCov = ptr;
                }

                gnss_ubx_nav_cov_t *msg = handle->ubxNavCov;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->version = message->buffer[offset + 4];
                msg->posCovValid = message->buffer[offset + 5];
                msg->velCovValid = message->buffer[offset + 6];
                msg->posCovNN = getFloat(message->buffer, offset + 16);
                msg->posCovNE = getFloat(message->buffer, offset + 20);
                msg->posCovND = getFloat(message->buffer, offset + 24);
                msg->posCovEE = getFloat(message->buffer, offset + 28);
                msg->posCovED = getFloat(message->buffer, offset + 32);
                msg->posCovDD = getFloat(message->buffer, offset + 36);
                msg->velCovNN = getFloat(message->buffer, offset + 40);
                msg->velCovNE = getFloat(message->buffer, offset + 44);
                msg->velCovND = getFloat(message->buffer, offset + 48);
                msg->velCovEE = getFloat(message->buffer, offset + 52);
                msg->velCovED = getFloat(message->buffer, offset + 56);
                msg->velCovDD = getFloat(message->buffer, offset + 60);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] posCovValid: "); Serial.println(msg->posCovValid);
                Serial.print("[DEBUG] velCovValid: "); Serial.println(msg->velCovValid);
                Serial.print("[DEBUG] posCovNN: "); Serial.println(msg->posCovNN);
                Serial.print("[DEBUG] posCovNE: "); Serial.println(msg->posCovNE);
                Serial.print("[DEBUG] posCovND: "); Serial.println(msg->posCovND);
                Serial.print("[DEBUG] posCovEE: "); Serial.println(msg->posCovEE);
                Serial.print("[DEBUG] posCovED: "); Serial.println(msg->posCovED);
                Serial.print("[DEBUG] posCovDD: "); Serial.println(msg->posCovDD);
                Serial.print("[DEBUG] velCovNN: "); Serial.println(msg->velCovNN);
                Serial.print("[DEBUG] velCovNE: "); Serial.println(msg->velCovNE);
                Serial.print("[DEBUG] velCovND: "); Serial.println(msg->velCovND);
                Serial.print("[DEBUG] velCovEE: "); Serial.println(msg->velCovEE);
                Serial.print("[DEBUG] velCovED: "); Serial.println(msg->velCovED);
                Serial.print("[DEBUG] velCovDD: "); Serial.println(msg->velCovDD);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_DOP: // UBX-NAV-DOP
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-DOP");
                #endif

                if (handle->ubxNavDop == NULL)
                {
                    gnss_ubx_nav_dop_t *ptr = (gnss_ubx_nav_dop_t *)malloc(sizeof(gnss_ubx_nav_dop_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-DOP message");
                        #endif
                        break;
                    }

                    handle->ubxNavDop = ptr;
                }

                gnss_ubx_nav_dop_t *msg = handle->ubxNavDop;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->gdop = getUByte16_LEnd(message->buffer, offset + 4) / 100.0;
                msg->pdop = getUByte16_LEnd(message->buffer, offset + 6) / 100.0;
                msg->tdop = getUByte16_LEnd(message->buffer, offset + 8) / 100.0;
                msg->vdop = getUByte16_LEnd(message->buffer, offset + 10) / 100.0;
                msg->hdop = getUByte16_LEnd(message->buffer, offset + 12) / 100.0;
                msg->ndop = getUByte16_LEnd(message->buffer, offset + 14) / 100.0;
                msg->edop = getUByte16_LEnd(message->buffer, offset + 16) / 100.0;

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] gDOP: "); Serial.println(msg->gdop);
                Serial.print("[DEBUG] pDOP: "); Serial.println(msg->pdop);
                Serial.print("[DEBUG] tDOP: "); Serial.println(msg->tdop);
                Serial.print("[DEBUG] vDOP: "); Serial.println(msg->vdop);
                Serial.print("[DEBUG] hDOP: "); Serial.println(msg->hdop);
                Serial.print("[DEBUG] nDOP: "); Serial.println(msg->ndop);
                Serial.print("[DEBUG] eDOP: "); Serial.println(msg->edop);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_EOE: // UBX-NAV-EOE
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-EOE");
                #endif

                if (handle->ubxNavEoe == NULL)
                {
                    gnss_ubx_nav_eoe_t *ptr = (gnss_ubx_nav_eoe_t *)malloc(sizeof(gnss_ubx_nav_eoe_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-EOE message");
                        #endif
                        break;
                    }

                    handle->ubxNavEoe = ptr;
                }

                gnss_ubx_nav_eoe_t *msg = handle->ubxNavEoe;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_ODO: // UBX-NAV-ODO
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-ODO");
                #endif

                if (handle->ubxNavOdo == NULL)
                {
                    gnss_ubx_nav_odo_t *ptr = (gnss_ubx_nav_odo_t *)malloc(sizeof(gnss_ubx_nav_odo_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-ODO message");
                        #endif
                        break;
                    }

                    handle->ubxNavOdo = ptr;
                }

                gnss_ubx_nav_odo_t *msg = handle->ubxNavOdo;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                Serial.print("[DEBUG] version: "); Serial.println(message->buffer[offset]);

                timestamp = getUByte32_LEnd(message->buffer, offset + 4);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->distance = getUByte32_LEnd(message->buffer, offset + 8);
                msg->totalDistance = getUByte32_LEnd(message->buffer, offset + 12);
                msg->distanceStd = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] distance: "); Serial.println(msg->distance);
                Serial.print("[DEBUG] totalDistance: "); Serial.println(msg->totalDistance);
                Serial.print("[DEBUG] distanceStd: "); Serial.println(msg->distanceStd);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_ORB: // UBX-NAV-ORB
            {
                
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-ORB");
                #endif

                if (handle->ubxNavOrb == NULL)
                {
                    gnss_ubx_nav_orb_t *ptr = (gnss_ubx_nav_orb_t *)malloc(sizeof(gnss_ubx_nav_orb_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-ORB message");
                        #endif
                        break;
                    }

                    handle->ubxNavOrb = ptr;
                }

                gnss_ubx_nav_orb_t *msg = handle->ubxNavOrb;

                j = 6;
                offset = 6;
                
                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->version = message->buffer[offset + 4];
                msg->numSv = message->buffer[offset + 5];

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] numSv: "); Serial.println(msg->numSv);
                #endif

                for (uint8_t i = 0; i < msg->numSv; i++)
                {
                    #ifdef DEBUG
                    Serial.print("[DEBUG] for loop iteration "); Serial.println(i);
                    #endif
                    msg->sv[i].gnssID = message->buffer[offset + 8 + i * 6];
                    msg->sv[i].svID = message->buffer[offset + 9 + i * 6];
                    msg->sv[i].health = message->buffer[offset + 10 + i * 6] & 0x03;
                    msg->sv[i].visibility = message->buffer[offset + 10 + i * 6] >> 2;
                    msg->sv[i].ephUsability = message->buffer[offset + 11 + i * 6] & 0x1F;
                    msg->sv[i].ephSource = message->buffer[offset + 11 + i * 6] >> 5;
                    msg->sv[i].almUsability = message->buffer[offset + 12 + i * 6] & 0x1F;
                    msg->sv[i].almSource = message->buffer[offset + 12 + i * 6] >> 5;
                    msg->sv[i].anoAopUsability = message->buffer[offset + 13 + i * 6] & 0x1F;
                    msg->sv[i].type = message->buffer[offset + 13 + i * 6] >> 5;

                    #ifdef DEBUG
                    Serial.print("[DEBUG] gnssId: "); Serial.println(msg->sv[i].gnssID);
                    Serial.print("[DEBUG] svId: "); Serial.println(msg->sv[i].svID);
                    Serial.print("[DEBUG] svHealth: "); Serial.println(msg->sv[i].health);
                    Serial.print("[DEBUG] svVisibility: "); Serial.println(msg->sv[i].visibility);
                    Serial.print("[DEBUG] ephUsability: "); Serial.println(msg->sv[i].ephUsability);
                    Serial.print("[DEBUG] ephSource: "); Serial.println(msg->sv[i].ephSource);
                    Serial.print("[DEBUG] almUsability: "); Serial.println(msg->sv[i].almUsability);
                    Serial.print("[DEBUG] almSource: "); Serial.println(msg->sv[i].almSource);
                    Serial.print("[DEBUG] anoAopUsability: "); Serial.println(msg->sv[i].anoAopUsability);
                    Serial.print("[DEBUG] type: "); Serial.println(msg->sv[i].type);
                    #endif
                }
                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                
                break;
            }
            case GNSS_UBX_NAV_PL: // UBX-NAV-PL
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-PL");
                #endif

                if (handle->ubxNavPl == NULL)
                {
                    gnss_ubx_nav_pl_t *ptr = (gnss_ubx_nav_pl_t *)malloc(sizeof(gnss_ubx_nav_pl_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-PL message");
                        #endif
                        break;
                    }

                    handle->ubxNavPl = ptr;
                }

                gnss_ubx_nav_pl_t *msg = handle->ubxNavPl;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset + 12);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->msgVersion = message->buffer[offset];
                msg->tmirCoeff = message->buffer[offset + 1];
                msg->tmirExp = message->buffer[offset + 2];
                msg->plPosValid = message->buffer[offset + 3];
                msg->plPosFrame = message->buffer[offset + 4];
                msg->plVelValid = message->buffer[offset + 5];
                msg->plVelFrame = message->buffer[offset + 6];
                msg->plTimeValid = message->buffer[offset + 7];
                msg->plPosInvReason = message->buffer[offset + 8];
                msg->plVelInvReason = message->buffer[offset + 9];
                msg->plTimeInvReason = message->buffer[offset + 10];
                msg->plPos1 = getUByte32_LEnd(message->buffer, offset + 16);
                msg->plPos2 = getUByte32_LEnd(message->buffer, offset + 20);
                msg->plPos3 = getUByte32_LEnd(message->buffer, offset + 24);
                msg->plVel1 = getUByte32_LEnd(message->buffer, offset + 28);
                msg->plVel2 = getUByte32_LEnd(message->buffer, offset + 32);
                msg->plVel3 = getUByte32_LEnd(message->buffer, offset + 36);
                msg->plPosHorizOrient = getUByte16_LEnd(message->buffer, offset + 40) / 100.0;
                msg->plVelHorizOrient = getUByte16_LEnd(message->buffer, offset + 42) / 100.0;
                msg->plTime = getUByte32_LEnd(message->buffer, offset + 44);

                #ifdef DEBUG
                Serial.print("[DEBUG] msgVersion: "); Serial.println(msg->msgVersion);
                Serial.print("[DEBUG] tmirCoeff: "); Serial.println(msg->tmirCoeff);
                Serial.print("[DEBUG] tmirExp: "); Serial.println(msg->tmirExp);
                Serial.print("[DEBUG] plPosValid: "); Serial.println(msg->plPosValid);
                Serial.print("[DEBUG] plPosFrame: "); Serial.println(msg->plPosFrame);
                Serial.print("[DEBUG] plVelValid: "); Serial.println(msg->plVelValid);
                Serial.print("[DEBUG] plVelFrame: "); Serial.println(msg->plVelFrame);
                Serial.print("[DEBUG] plTimeValid: "); Serial.println(msg->plTimeValid);
                Serial.print("[DEBUG] plPosInvalidityReason: "); Serial.println(msg->plPosInvReason);
                Serial.print("[DEBUG] plVelInvalidityReason: "); Serial.println(msg->plVelInvReason);
                Serial.print("[DEBUG] plTimeInvalidityReason: "); Serial.println(msg->plTimeInvReason);
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] plPos1: "); Serial.println(msg->plPos1);
                Serial.print("[DEBUG] plPos2: "); Serial.println(msg->plPos2);
                Serial.print("[DEBUG] plPos3: "); Serial.println(msg->plPos3);
                Serial.print("[DEBUG] plVel1: "); Serial.println(msg->plVel1);
                Serial.print("[DEBUG] plVel2: "); Serial.println(msg->plVel2);
                Serial.print("[DEBUG] plVel3: "); Serial.println(msg->plVel3);
                Serial.print("[DEBUG] plPosHorizOrient: "); Serial.println();
                Serial.print("[DEBUG] plVelHorizOrient: "); Serial.println();
                Serial.print("[DEBUG] plTime: "); Serial.println();
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_POSECEF: // UBX-NAV-POSECEF
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-POSECEF");
                #endif

                if (handle->ubxNavPosecef == NULL)
                {
                    gnss_ubx_nav_posecef_t *ptr = (gnss_ubx_nav_posecef_t *)malloc(sizeof(gnss_ubx_nav_posecef_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-POSECEF message");
                        #endif
                        break;
                    }

                    handle->ubxNavPosecef = ptr;
                }

                gnss_ubx_nav_posecef_t *msg = handle->ubxNavPosecef;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->ecefX = getIByte32_LEnd(message->buffer, offset + 4);
                msg->ecefY = getIByte32_LEnd(message->buffer, offset + 8);
                msg->ecefZ = getIByte32_LEnd(message->buffer, offset + 12);
                msg->pAcc = getUByte32_LEnd(message->buffer, offset + 16);
                
                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] ecefX: "); Serial.println(msg->ecefX);
                Serial.print("[DEBUG] ecefY: "); Serial.println(msg->ecefY);
                Serial.print("[DEBUG] ecefZ: "); Serial.println(msg->ecefZ);
                Serial.print("[DEBUG] pAcc: "); Serial.println(msg->pAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_POSLLH: // UBX-NAV-POSLLH
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-POSLLH");
                #endif

                if (handle->ubxNavPosllh == NULL)
                {
                    gnss_ubx_nav_posllh_t *ptr = (gnss_ubx_nav_posllh_t *)malloc(sizeof(gnss_ubx_nav_posllh_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-POSLLH message");
                        #endif
                        break;
                    }

                    handle->ubxNavPosllh = ptr;
                }

                gnss_ubx_nav_posllh_t *msg = handle->ubxNavPosllh;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->longitude = (getIByte32_LEnd(message->buffer, offset + 4)) / 10000000.0;
                msg->latitude = (getIByte32_LEnd(message->buffer, offset + 8)) / 10000000.0;
                msg->heightEllip = getIByte32_LEnd(message->buffer, offset + 12);
                msg->hMSL = getIByte32_LEnd(message->buffer, offset + 16);
                msg->hAcc = getUByte32_LEnd(message->buffer, offset + 20);
                msg->vAcc = getUByte32_LEnd(message->buffer, offset + 24);
                
                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] lon: "); Serial.println(msg->longitude, 6);
                Serial.print("[DEBUG] lat: "); Serial.println(msg->latitude, 6);
                Serial.print("[DEBUG] height: "); Serial.println(msg->heightEllip);
                Serial.print("[DEBUG] hMSL: "); Serial.println(msg->hMSL);
                Serial.print("[DEBUG] hAcc: "); Serial.println(msg->hAcc);
                Serial.print("[DEBUG] vAcc: "); Serial.println(msg->vAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_PVT: // UBX-NAV-PVT
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-PVT");
                #endif

                if (handle->ubxNavPvt == NULL)
                {
                    gnss_ubx_nav_pvt_t *ptr = (gnss_ubx_nav_pvt_t *)malloc(sizeof(gnss_ubx_nav_pvt_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-PVT message");
                        #endif
                        break;
                    }

                    handle->ubxNavPvt = ptr;
                }

                gnss_ubx_nav_pvt_t *msg = handle->ubxNavPvt;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->validDate = message->buffer[offset + 11] & 0x01;
                msg->validTime = (message->buffer[offset + 11] >> 1) & 0x01;
                msg->fullyResolved = (message->buffer[offset + 11] >> 2) & 0x01;
                msg->validMag = (message->buffer[offset + 11] >> 3) & 0x01;

                if (msg->validDate)
                {
                    msg->yearUTC = getUByte16_LEnd(message->buffer, offset + 4) % 2000;
                    msg->monthUTC = message->buffer[offset + 6];
                    msg->dayUTC = message->buffer[offset + 7];
                }
                else
                {
                    msg->yearUTC = 0;
                    msg->monthUTC = 0;
                    msg->dayUTC = 0;
                }

                if (msg->validTime)
                {
                    msg->hoursUTC = message->buffer[offset + 8];
                    msg->minutesUTC = message->buffer[offset + 9];
                    msg->secondsUTC = message->buffer[offset + 10];
                }
                else
                {
                    msg->hoursUTC = 0;
                    msg->minutesUTC = 0;
                    msg->secondsUTC = 0;
                }

                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 12);
                msg->nanosUTC = getIByte32_LEnd(message->buffer, offset + 16);
                msg->fixType = message->buffer[offset + 20];
                msg->fixOK = message->buffer[offset + 21] & 0x01;
                msg->diffSoln = (message->buffer[offset + 21] >> 1) & 0x01;
                msg->psmState = (message->buffer[offset + 21] >> 2) & 0x07;
                msg->headVehValid = (message->buffer[offset + 21] >> 5) & 0x01;
                msg->carrSoln = message->buffer[offset + 21] >> 6;
                msg->confirmedAvai = (message->buffer[offset + 22] >> 5) & 0x01;
                msg->confirmedDate = (message->buffer[offset + 22] >> 6) & 0x01;
                msg->confirmedTime = (message->buffer[offset + 22] >> 7) & 0x01;
                msg->numSV = message->buffer[offset + 23];
                
                msg->invalidLlh = message->buffer[offset + 78] & 0x01;
                if (!msg->invalidLlh)
                {
                    msg->longitude = (getIByte32_LEnd(message->buffer, offset + 24)) / 10000000.0;
                    msg->latitude = (getIByte32_LEnd(message->buffer, offset + 28)) / 10000000.0;
                }
                else
                {
                    msg->longitude = 0;
                    msg->latitude = 0;
                }

                msg->heightEllip = getIByte32_LEnd(message->buffer, offset + 32);
                msg->hMSL = getIByte32_LEnd(message->buffer, offset + 36);
                msg->hAcc = getUByte32_LEnd(message->buffer, offset + 40);
                msg->vAcc = getUByte32_LEnd(message->buffer, offset + 44);
                msg->velN = getIByte32_LEnd(message->buffer, offset + 48);
                msg->velE = getIByte32_LEnd(message->buffer, offset + 52);
                msg->velD = getIByte32_LEnd(message->buffer, offset + 56);
                msg->gSpeed = getIByte32_LEnd(message->buffer, offset + 60);
                msg->course = getIByte32_LEnd(message->buffer, offset + 64) / 100000.0;
                msg->sAcc = getUByte32_LEnd(message->buffer, offset + 68);
                msg->headAcc = getUByte32_LEnd(message->buffer, offset + 72) / 100000.0;
                msg->pdop = getUByte16_LEnd(message->buffer, offset + 76) / 100.0;
                msg->lastCorrAge = (message->buffer[offset + 78] >> 1) & 0x0F;
                msg->authTime = (message->buffer[offset + 79] >> 5) & 0x01;
                
                if (msg->headVehValid)
                    msg->heading = getIByte32_LEnd(message->buffer, offset + 84) / 100000.0;
                else
                    msg->heading = 0;

                msg->magDec = getIByte16_LEnd(message->buffer, offset + 88) / 100.0;
                msg->magAcc = getUByte16_LEnd(message->buffer, offset + 90) / 100.0;
                
                #ifdef DEBUG
                Serial.print("[DEBUG] year: "); Serial.println(msg->yearUTC);
                Serial.print("[DEBUG] month: "); Serial.println(msg->monthUTC);
                Serial.print("[DEBUG] day: "); Serial.println(msg->dayUTC);
                Serial.print("[DEBUG] hour: "); Serial.println(msg->hoursUTC);
                Serial.print("[DEBUG] min: "); Serial.println(msg->minutesUTC);
                Serial.print("[DEBUG] sec: "); Serial.println(msg->secondsUTC);
                Serial.print("[DEBUG] validDate: "); Serial.println(msg->validDate);
                Serial.print("[DEBUG] validTime: "); Serial.println(msg->validTime);
                Serial.print("[DEBUG] fullyResolved: "); Serial.println(msg->fullyResolved);
                Serial.print("[DEBUG] validMag: "); Serial.println(msg->validMag);
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] nano: "); Serial.println(msg->nanosUTC);
                Serial.print("[DEBUG] fixType: "); Serial.println(msg->fixType);
                Serial.print("[DEBUG] gnssFixOK: "); Serial.println(msg->fixOK);
                Serial.print("[DEBUG] diffSoln: "); Serial.println(msg->diffSoln);
                Serial.print("[DEBUG] psmState: "); Serial.println(msg->psmState);
                Serial.print("[DEBUG] headVehValid: "); Serial.println(msg->headVehValid);
                Serial.print("[DEBUG] carrSoln: "); Serial.println(msg->carrSoln);
                Serial.print("[DEBUG] confirmedAvai: "); Serial.println(msg->confirmedAvai);
                Serial.print("[DEBUG] confirmedDate: "); Serial.println(msg->confirmedDate);
                Serial.print("[DEBUG] confirmedTime: "); Serial.println(msg->confirmedTime);
                Serial.print("[DEBUG] numSV: "); Serial.println(msg->numSV);
                Serial.print("[DEBUG] lon: "); Serial.println(msg->longitude, 6);
                Serial.print("[DEBUG] lat: "); Serial.println(msg->latitude, 6);
                Serial.print("[DEBUG] height: "); Serial.println(msg->heightEllip);
                Serial.print("[DEBUG] hMSL: "); Serial.println(msg->hMSL);
                Serial.print("[DEBUG] hAcc: "); Serial.println(msg->hAcc);
                Serial.print("[DEBUG] vAcc: "); Serial.println(msg->vAcc);
                Serial.print("[DEBUG] velN: "); Serial.println(msg->velN);
                Serial.print("[DEBUG] velE: "); Serial.println(msg->velE);
                Serial.print("[DEBUG] velD: "); Serial.println(msg->velD);
                Serial.print("[DEBUG] gSpeed: "); Serial.println(msg->gSpeed);
                Serial.print("[DEBUG] headMot: "); Serial.println(msg->course);
                Serial.print("[DEBUG] sAcc: "); Serial.println(msg->sAcc);
                Serial.print("[DEBUG] headAcc: "); Serial.println(msg->headAcc);
                Serial.print("[DEBUG] pDOP: "); Serial.println(msg->pdop);
                Serial.print("[DEBUG] invalidLlh: "); Serial.println(msg->invalidLlh);
                Serial.print("[DEBUG] lastCorrectionAge: "); Serial.println(msg->lastCorrAge);
                Serial.print("[DEBUG] authTime: "); Serial.println(msg->authTime);
                Serial.print("[DEBUG] headVeh: "); Serial.println(msg->heading);
                Serial.print("[DEBUG] magDec: "); Serial.println(msg->magDec);
                Serial.print("[DEBUG] magAcc: "); Serial.println(msg->magAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_SAT: // UBX-NAV-SAT
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-SAT");
                #endif

                if (handle->ubxNavSat == NULL)
                {
                    gnss_ubx_nav_sat_t *ptr = (gnss_ubx_nav_sat_t *)malloc(sizeof(gnss_ubx_nav_sat_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SAT message");
                        #endif
                        break;
                    }

                    handle->ubxNavSat = ptr;
                }

                gnss_ubx_nav_sat_t *msg = handle->ubxNavSat;

                offset = 6;
                
                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }
                
                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->version = message->buffer[offset + 4];
                msg->numSv = message->buffer[offset + 5];

                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] numSv: "); Serial.println(msg->numSv);

                for (uint8_t i = 0; i < msg->numSv; i++)
                {
                    msg->sv[i].gnssID = message->buffer[offset + 8 + i * 12];
                    msg->sv[i].svID = message->buffer[offset + 9 + i * 12];
                    msg->sv[i].cno = message->buffer[offset + 10 + i * 12];
                    msg->sv[i].elev = message->buffer[offset + 11 + i * 12];
                    msg->sv[i].azim = getIByte16_LEnd(message->buffer, offset + 12 + i * 12);
                    msg->sv[i].prRes = getIByte16_LEnd(message->buffer, offset + 14 + i * 12) / 10.0;
                    msg->sv[i].qualityInd = message->buffer[offset + 16 + i * 12] & 0x07;
                    msg->sv[i].svUsed = (message->buffer[offset + 16 + i * 12] >> 3) & 0x01;
                    msg->sv[i].health = (message->buffer[offset + 16 + i * 12] >> 4) & 0x03;
                    msg->sv[i].diffCorr = (message->buffer[offset + 16 + i * 12] >> 6) & 0x01;
                    msg->sv[i].smoothed = (message->buffer[offset + 16 + i * 12] >> 7) & 0x01;
                    msg->sv[i].orbitSource = message->buffer[offset + 17 + i * 12] & 0x07;
                    msg->sv[i].ephAvail = (message->buffer[offset + 17 + i * 12] >> 3) & 0x01;
                    msg->sv[i].almAvail = (message->buffer[offset + 17 + i * 12] >> 4) & 0x01;
                    msg->sv[i].anoAvail = (message->buffer[offset + 17 + i * 12] >> 5) & 0x01;
                    msg->sv[i].aopAvail = (message->buffer[offset + 17 + i * 12] >> 6) & 0x01;
                    msg->sv[i].sbasCorrUsed = message->buffer[offset + 18 + i * 12] & 0x01;
                    msg->sv[i].rtcmCorrUsed = (message->buffer[offset + 18 + i * 12] >> 1) & 0x01;
                    msg->sv[i].slasCorrUsed = (message->buffer[offset + 18 + i * 12] >> 2) & 0x01;
                    msg->sv[i].spartnCorrUsed = (message->buffer[offset + 18 + i * 12] >> 3) & 0x01;
                    msg->sv[i].prCorrUsed = (message->buffer[offset + 18 + i * 12] >> 4) & 0x01;
                    msg->sv[i].crCorrUsed = (message->buffer[offset + 18 + i * 12] >> 5) & 0x01;
                    msg->sv[i].doCorrUsed = (message->buffer[offset + 18 + i * 12] >> 6) & 0x01;
                    msg->sv[i].clasCorrUsed = (message->buffer[offset + 18 + i * 12] >> 7) & 0x01;

                    #ifdef DEBUG
                    Serial.print("[DEBUG] gnssId: "); Serial.println(msg->sv[i].gnssID);
                    Serial.print("[DEBUG] svId: "); Serial.println(msg->sv[i].svID);
                    Serial.print("[DEBUG] cno: "); Serial.println(msg->sv[i].cno);
                    Serial.print("[DEBUG] elev: "); Serial.println(msg->sv[i].elev);
                    Serial.print("[DEBUG] azim: "); Serial.println(msg->sv[i].azim);
                    Serial.print("[DEBUG] prRes: "); Serial.println(msg->sv[i].prRes);
                    Serial.print("[DEBUG] qualityInd: "); Serial.println(msg->sv[i].qualityInd);
                    Serial.print("[DEBUG] svUsed: "); Serial.println(msg->sv[i].svUsed);
                    Serial.print("[DEBUG] health: "); Serial.println(msg->sv[i].health);
                    Serial.print("[DEBUG] diffCorr: "); Serial.println(msg->sv[i].diffCorr);
                    Serial.print("[DEBUG] smoothed: "); Serial.println(msg->sv[i].smoothed);
                    Serial.print("[DEBUG] orbitSource: "); Serial.println(msg->sv[i].orbitSource);
                    Serial.print("[DEBUG] ephAvail: "); Serial.println(msg->sv[i].ephAvail);
                    Serial.print("[DEBUG] almAvail: "); Serial.println(msg->sv[i].almAvail);
                    Serial.print("[DEBUG] anoAvail: "); Serial.println(msg->sv[i].anoAvail);
                    Serial.print("[DEBUG] aopAvail: "); Serial.println(msg->sv[i].aopAvail);
                    Serial.print("[DEBUG] sbasCorrUsed: "); Serial.println(msg->sv[i].sbasCorrUsed);
                    Serial.print("[DEBUG] rtcmCorrUsed: "); Serial.println(msg->sv[i].rtcmCorrUsed);
                    Serial.print("[DEBUG] slasCorrUsed: "); Serial.println(msg->sv[i].slasCorrUsed);
                    Serial.print("[DEBUG] spartnCorrUsed: "); Serial.println(msg->sv[i].spartnCorrUsed);
                    Serial.print("[DEBUG] prCorrUsed: "); Serial.println(msg->sv[i].prCorrUsed);
                    Serial.print("[DEBUG] crCorrUsed: "); Serial.println(msg->sv[i].crCorrUsed);
                    Serial.print("[DEBUG] doCorrUsed: "); Serial.println(msg->sv[i].doCorrUsed);
                    Serial.print("[DEBUG] clasCorrUsed: "); Serial.println(msg->sv[i].clasCorrUsed);
                    #endif
                }
                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_SBAS: // UBX-NAV-SBAS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-SBAS");
                #endif

                if (handle->ubxNavSbas == NULL)
                {
                    gnss_ubx_nav_sbas_t *ptr = (gnss_ubx_nav_sbas_t *)malloc(sizeof(gnss_ubx_nav_sbas_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SAT message");
                        #endif
                        break;
                    }

                    handle->ubxNavSbas = ptr;
                }

                gnss_ubx_nav_sbas_t *msg = handle->ubxNavSbas;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->geo = message->buffer[offset + 4];
                msg->mode = message->buffer[offset + 5];
                msg->sys = message->buffer[offset + 6];
                msg->ranging = message->buffer[offset + 7] & 0x01;
                msg->corrections = (message->buffer[offset + 7] >> 1) & 0x01;
                msg->integrity = (message->buffer[offset + 7] >> 2) & 0x01;
                msg->testMode = (message->buffer[offset + 7] >> 3) & 0x01;
                msg->bad = (message->buffer[offset + 7] >> 4) & 0x01;
                msg->cnt = message->buffer[offset + 8];
                msg->integrityUsed = message->buffer[offset + 9] & 0x03;

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] geo: "); Serial.println(msg->geo);
                Serial.print("[DEBUG] mode: "); Serial.println(msg->mode);
                Serial.print("[DEBUG] sys: "); Serial.println(msg->sys);
                Serial.print("[DEBUG] Ranging: "); Serial.println(msg->ranging);
                Serial.print("[DEBUG] Corrections: "); Serial.println(msg->corrections);
                Serial.print("[DEBUG] Integrity: "); Serial.println(msg->integrity);
                Serial.print("[DEBUG] Testmode: "); Serial.println(msg->testMode);
                Serial.print("[DEBUG] Bad: "); Serial.println(msg->bad);
                Serial.print("[DEBUG] cnt: "); Serial.println(msg->cnt);
                Serial.print("[DEBUG] integrityUsed: "); Serial.println(msg->integrityUsed);
                #endif

                for (uint8_t i = 0; i < msg->cnt; i++)
                {
                    msg->sv[i].svID = message->buffer[offset + 12 + i * 12];
                    msg->sv[i].udre = message->buffer[offset + 14 + i * 12];
                    msg->sv[i].svSys = message->buffer[offset + 15 + i * 12];
                    msg->sv[i].svService = message->buffer[offset + 16 + i * 12];
                    msg->sv[i].prc = getIByte16_LEnd(message->buffer, offset + 18 + i * 12);
                    msg->sv[i].ic = getIByte16_LEnd(message->buffer, offset + 22 + i * 12);

                    #ifdef DEBUG
                    Serial.print("[DEBUG] svid: "); Serial.println(msg->sv[i].svID);
                    Serial.print("[DEBUG] udre: "); Serial.println(msg->sv[i].udre);
                    Serial.print("[DEBUG] svSys: "); Serial.println(msg->sv[i].svSys);
                    Serial.print("[DEBUG] svService: "); Serial.println(msg->sv[i].svService);
                    Serial.print("[DEBUG] prc: "); Serial.println(msg->sv[i].prc);
                    Serial.print("[DEBUG] ic: "); Serial.println(msg->sv[i].ic);
                    #endif
                }
                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_SIG: // UBX-NAV-SIG
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-SIG");
                #endif

                if (handle->ubxNavSig == NULL)
                {
                    gnss_ubx_nav_sig_t *ptr = (gnss_ubx_nav_sig_t *)malloc(sizeof(gnss_ubx_nav_sig_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SIG message");
                        #endif
                        break;
                    }

                    handle->ubxNavSig = ptr;
                }

                gnss_ubx_nav_sig_t *msg = handle->ubxNavSig;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->version = message->buffer[offset + 4];
                msg->numSv = message->buffer[offset + 5];

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] numSigs: "); Serial.println(msg->numSv);
                #endif

                for (uint8_t i = 0; i < message->buffer[offset + 5]; i++)
                {
                    msg->sv[i].gnssID = message->buffer[offset + 8 + i * 16];
                    msg->sv[i].svID = message->buffer[offset + 9 + i * 16];
                    msg->sv[i].sigID = message->buffer[offset + 10 + i * 16];
                    msg->sv[i].freqID = message->buffer[offset + 11 + i * 16];
                    msg->sv[i].prRes = getIByte16_LEnd(message->buffer, offset + 12 + i * 16) / 10.0;
                    msg->sv[i].cno = message->buffer[offset + 14 + i * 16];
                    msg->sv[i].qualityInd = message->buffer[offset + 15 + i * 16];
                    msg->sv[i].corrSource = message->buffer[offset + 16 + i * 16];
                    msg->sv[i].ionoModel = message->buffer[offset + 17 + i * 16];
                    msg->sv[i].health = message->buffer[offset + 18 + i * 16] & 0x03;
                    msg->sv[i].prSmoothed = (message->buffer[offset + 18 + i * 16] >> 2) & 0x01;
                    msg->sv[i].prUsed = (message->buffer[offset + 18 + i * 16] >> 3) & 0x01;
                    msg->sv[i].crUsed = (message->buffer[offset + 18 + i * 16] >> 4) & 0x01;
                    msg->sv[i].doUsed = (message->buffer[offset + 18 + i * 16] >> 5) & 0x01;
                    msg->sv[i].prCorrUsed = (message->buffer[offset + 18 + i * 16] >> 6) & 0x01;
                    msg->sv[i].crCorrUsed = (message->buffer[offset + 18 + i * 16] >> 7) & 0x01;
                    msg->sv[i].doCorrUsed = message->buffer[offset + 19 + i * 16] & 0x01;
                    msg->sv[i].authStatus = (message->buffer[offset + 19 + i * 16] >> 1) & 0x01;

                    #ifdef DEBUG
                    Serial.print("[DEBUG] gnssId: "); Serial.println(msg->sv[i].gnssID);
                    Serial.print("[DEBUG] svId: "); Serial.println(msg->sv[i].svID);
                    Serial.print("[DEBUG] sigId: "); Serial.println(msg->sv[i].sigID);
                    Serial.print("[DEBUG] freqId: "); Serial.println(msg->sv[i].freqID);
                    Serial.print("[DEBUG] prRes: "); Serial.println(msg->sv[i].prRes);
                    Serial.print("[DEBUG] cno: "); Serial.println(msg->sv[i].cno);
                    Serial.print("[DEBUG] qualityInd: "); Serial.println(msg->sv[i].qualityInd);
                    Serial.print("[DEBUG] corrSource: "); Serial.println(msg->sv[i].corrSource);
                    Serial.print("[DEBUG] ionoModel: "); Serial.println(msg->sv[i].ionoModel);
                    Serial.print("[DEBUG] health: "); Serial.println(msg->sv[i].health);
                    Serial.print("[DEBUG] prSmoothed: "); Serial.println(msg->sv[i].prSmoothed);
                    Serial.print("[DEBUG] prUsed: "); Serial.println(msg->sv[i].prUsed);
                    Serial.print("[DEBUG] crUsed: "); Serial.println(msg->sv[i].crUsed);
                    Serial.print("[DEBUG] doUsed: "); Serial.println(msg->sv[i].doUsed);
                    Serial.print("[DEBUG] prCorrUsed: "); Serial.println(msg->sv[i].prCorrUsed);
                    Serial.print("[DEBUG] crCorrUsed: "); Serial.println(msg->sv[i].crCorrUsed);
                    Serial.print("[DEBUG] doCorrUsed: "); Serial.println(msg->sv[i].doCorrUsed);
                    Serial.print("[DEBUG] authStatus: "); Serial.println(msg->sv[i].authStatus);
                    #endif
                }
                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_SLAS: // UBX-NAV-SLAS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-SLAS");
                #endif

                if (handle->ubxNavSlas == NULL)
                {
                    gnss_ubx_nav_slas_t *ptr = (gnss_ubx_nav_slas_t *)malloc(sizeof(gnss_ubx_nav_slas_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SLAS message");
                        #endif
                        break;
                    }

                    handle->ubxNavSlas = ptr;
                }

                gnss_ubx_nav_slas_t *msg = handle->ubxNavSlas;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->version = message->buffer[offset + 4];
                msg->gmsLon = getIByte32_LEnd(message->buffer, offset + 8) / 1000.0;
                msg->gmsLat = getIByte32_LEnd(message->buffer, offset + 12) / 1000.0;
                msg->gmsCode = message->buffer[offset + 16];
                msg->qzssSvID = message->buffer[offset + 17];
                msg->gmsAvailable = message->buffer[offset + 18] & 0x01;
                msg->qzssSvAvailable = (message->buffer[offset + 18] >> 1) & 0x01;
                msg->testMode = (message->buffer[offset + 18] >> 2) & 0x01;
                msg->cnt = message->buffer[offset + 19];
                
                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] gmsLon: "); Serial.println(msg->gmsLon);
                Serial.print("[DEBUG] gmsLat: "); Serial.println(msg->gmsLat);
                Serial.print("[DEBUG] gmsCode: "); Serial.println(msg->gmsCode);
                Serial.print("[DEBUG] qzssSvId: "); Serial.println(msg->qzssSvID);
                Serial.print("[DEBUG] gmsAvailable: "); Serial.println(msg->gmsAvailable);
                Serial.print("[DEBUG] qzssSvAvailable: "); Serial.println(msg->qzssSvAvailable);
                Serial.print("[DEBUG] testMode: "); Serial.println(msg->testMode);
                Serial.print("[DEBUG] cnt: "); Serial.println(msg->cnt);
                #endif

                for (uint8_t i = 0; i < msg->cnt; i++)
                {
                    msg->sv[i].gnssID = message->buffer[offset + 20 + i * 8];
                    msg->sv[i].svID = message->buffer[offset + 21 + i * 8];
                    msg->sv[i].prc = getIByte16_LEnd(message->buffer, offset + 26 + i * 8);

                    #ifdef DEBUG
                    Serial.print("[DEBUG] gnssId: "); Serial.println(msg->sv[i].gnssID);
                    Serial.print("[DEBUG] svId: "); Serial.println(msg->sv[i].svID);
                    Serial.print("[DEBUG] prc: "); Serial.println(msg->sv[i].prc);
                    #endif

                }
                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_STATUS: // UBX-NAV-STATUS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-STATUS");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavStatus == NULL)
                {
                    gnss_ubx_nav_status_t *ptr = (gnss_ubx_nav_status_t *)malloc(sizeof(gnss_ubx_nav_status_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-STATUS");
                        #endif
                        break;
                    }

                    handle->ubxNavStatus = ptr;
                }

                gnss_ubx_nav_status_t *msg = handle->ubxNavStatus;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->gpsFix = message->buffer[offset + 4];
                msg->gpsFixOK = message->buffer[offset + 5] & 0x01;
                msg->diffSoln = (message->buffer[offset + 5] >> 1) & 0x01;
                msg->wknSet = (message->buffer[offset + 5] >> 2) & 0x01;
                msg->towSet = (message->buffer[offset + 5] >> 3) & 0x01;
                msg->diffCorr = message->buffer[offset + 6] & 0x01;
                msg->carrSolnValid = (message->buffer[offset + 6] >> 1) & 0x01;
                msg->mapMatching = (message->buffer[offset + 6] >> 6) & 0x03;
                msg->psmState = message->buffer[offset + 7] & 0x03;
                msg->spoofDetState = (message->buffer[offset + 7] >> 3) & 0x03;
                msg->carrSoln = (message->buffer[offset + 7] >> 6) & 0x03;
                msg->ttff = getUByte32_LEnd(message->buffer, offset + 8);
                msg->msss = getUByte32_LEnd(message->buffer, offset + 12);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] gpsFix: "); Serial.println(msg->gpsFix);
                Serial.print("[DEBUG] gpsFixOK: "); Serial.println(msg->gpsFixOK);
                Serial.print("[DEBUG] diffSoln: "); Serial.println(msg->diffSoln);
                Serial.print("[DEBUG] wknSet: "); Serial.println(msg->wknSet);
                Serial.print("[DEBUG] towSet: "); Serial.println(msg->towSet);
                Serial.print("[DEBUG] diffCorr: "); Serial.println(msg->diffCorr);
                Serial.print("[DEBUG] carrSolnValid: "); Serial.println(msg->carrSolnValid);
                Serial.print("[DEBUG] mapMatching: "); Serial.println(msg->mapMatching);
                Serial.print("[DEBUG] psmState: "); Serial.println(msg->psmState);
                Serial.print("[DEBUG] spoofDetState: "); Serial.println(msg->spoofDetState);
                Serial.print("[DEBUG] carrSoln: "); Serial.println(msg->carrSoln);
                Serial.print("[DEBUG] ttff: "); Serial.println(msg->ttff);
                Serial.print("[DEBUG] msss: "); Serial.println(msg->msss);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMEBDS: // UBX-NAV-TIMEBDS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMEBDS");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimebds == NULL)
                {
                    gnss_ubx_nav_timebds_t *ptr = (gnss_ubx_nav_timebds_t *)malloc(sizeof(gnss_ubx_nav_timebds_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMEBDS");
                        #endif
                        break;
                    }

                    handle->ubxNavTimebds = ptr;
                }

                gnss_ubx_nav_timebds_t *msg = handle->ubxNavTimebds;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->SOW = getUByte32_LEnd(message->buffer, offset + 4);
                msg->fSOW = getIByte32_LEnd(message->buffer, offset + 8);
                msg->week = getIByte16_LEnd(message->buffer, offset + 12);
                msg->leapS = message->buffer[offset + 14];
                msg->sowValid = message->buffer[offset + 15] & 0x01;
                msg->weekValid = (message->buffer[offset + 15] >> 1) & 0x01;
                msg->leapSValid = (message->buffer[offset + 15] >> 2) & 0x01;
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] SOW: "); Serial.println(msg->SOW);
                Serial.print("[DEBUG] fSOW: "); Serial.println(msg->fSOW);
                Serial.print("[DEBUG] week: "); Serial.println(msg->week);
                Serial.print("[DEBUG] leapS: "); Serial.println(msg->leapS);
                Serial.print("[DEBUG] sowValid: "); Serial.println(msg->sowValid);
                Serial.print("[DEBUG] weekValid: "); Serial.println(msg->weekValid);
                Serial.print("[DEBUG] leapSValid: "); Serial.println(msg->leapSValid);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMEGAL: // UBX-NAV-TIMEGAL
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMEGAL");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimegal == NULL)
                {
                    gnss_ubx_nav_timegal_t *ptr = (gnss_ubx_nav_timegal_t *)malloc(sizeof(gnss_ubx_nav_timegal_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMEGAL");
                        #endif
                        break;
                    }

                    handle->ubxNavTimegal = ptr;
                }

                gnss_ubx_nav_timegal_t *msg = handle->ubxNavTimegal;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->galTow = getUByte32_LEnd(message->buffer, offset + 4);
                msg->fGalTow = getIByte32_LEnd(message->buffer, offset + 8);
                msg->galWno = getIByte16_LEnd(message->buffer, offset + 12);
                msg->leapS = message->buffer[offset + 14];
                msg->galTowValid = message->buffer[offset + 15] & 0x01;
                msg->galWnoValid = (message->buffer[offset + 15] >> 1) & 0x01;
                msg->leapSValid = (message->buffer[offset + 15] >> 2) & 0x01;
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] galTow: "); Serial.println(msg->galTow);
                Serial.print("[DEBUG] fGalTow: "); Serial.println(msg->fGalTow);
                Serial.print("[DEBUG] galWno: "); Serial.println(msg->galWno);
                Serial.print("[DEBUG] leapS: "); Serial.println(msg->leapS);
                Serial.print("[DEBUG] galTowValid: "); Serial.println(msg->galTowValid);
                Serial.print("[DEBUG] galWnoValid: "); Serial.println(msg->galWnoValid);
                Serial.print("[DEBUG] leapSValid: "); Serial.println(msg->leapSValid);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMEGLO: // UBX-NAV-TIMEGLO
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMEGLO");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimeglo == NULL)
                {
                    gnss_ubx_nav_timeglo_t *ptr = (gnss_ubx_nav_timeglo_t *)malloc(sizeof(gnss_ubx_nav_timeglo_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMEGLO");
                        #endif
                        break;
                    }

                    handle->ubxNavTimeglo = ptr;
                }

                gnss_ubx_nav_timeglo_t *msg = handle->ubxNavTimeglo;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->TOD = getUByte32_LEnd(message->buffer, offset + 4);
                msg->fTOD = getIByte32_LEnd(message->buffer, offset + 8);
                msg->Nt = getUByte16_LEnd(message->buffer, offset + 12);
                msg->N4 = message->buffer[offset + 14];
                msg->todValid = message->buffer[offset + 15] & 0x01;
                msg->dateValid = (message->buffer[offset + 15] >> 1) & 0x01;
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] TOD: "); Serial.println(msg->TOD);
                Serial.print("[DEBUG] fTOD: "); Serial.println(msg->fTOD);
                Serial.print("[DEBUG] Nt: "); Serial.println(msg->Nt);
                Serial.print("[DEBUG] N4: "); Serial.println(msg->N4);
                Serial.print("[DEBUG] todValid: "); Serial.println(msg->todValid);
                Serial.print("[DEBUG] dateValid: "); Serial.println(msg->dateValid);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMEGPS: // UBX-NAV-TIMEGAL
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMEGPS");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimegps == NULL)
                {
                    gnss_ubx_nav_timegps_t *ptr = (gnss_ubx_nav_timegps_t *)malloc(sizeof(gnss_ubx_nav_timegps_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMEGPS");
                        #endif
                        break;
                    }

                    handle->ubxNavTimegps = ptr;
                }

                gnss_ubx_nav_timegps_t *msg = handle->ubxNavTimegps;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->fTow = getIByte32_LEnd(message->buffer, offset + 4);
                msg->week = getIByte16_LEnd(message->buffer, offset + 8);
                msg->leapS = message->buffer[offset + 10];
                msg->towValid = message->buffer[offset + 11] & 0x01;
                msg->weekValid = (message->buffer[offset + 11] >> 1) & 0x01;
                msg->leapSValid = (message->buffer[offset + 11] >> 2) & 0x01;
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 12);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] fTow: "); Serial.println(msg->fTow);
                Serial.print("[DEBUG] week: "); Serial.println(msg->week);
                Serial.print("[DEBUG] leapS: "); Serial.println(msg->leapS);
                Serial.print("[DEBUG] towValid: "); Serial.println(msg->towValid);
                Serial.print("[DEBUG] weekValid: "); Serial.println(msg->weekValid);
                Serial.print("[DEBUG] leapSValid: "); Serial.println(msg->leapSValid);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMELS: // UBX-NAV-TIMELS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMELS");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimels == NULL)
                {
                    gnss_ubx_nav_timels_t *ptr = (gnss_ubx_nav_timels_t *)malloc(sizeof(gnss_ubx_nav_timels_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMELS");
                        #endif
                        break;
                    }

                    handle->ubxNavTimels = ptr;
                }

                gnss_ubx_nav_timels_t *msg = handle->ubxNavTimels;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->version = message->buffer[offset + 4];
                msg->srcOfCurrLs = message->buffer[offset + 8];
                msg->currLs = message->buffer[offset + 9];
                msg->srcOfLsChange = message->buffer[offset + 10];
                msg->lsChange = message->buffer[offset + 11];
                msg->timeToLsEvent = getIByte32_LEnd(message->buffer, offset + 12);
                msg->dateOfLsGpsWn = getUByte16_LEnd(message->buffer, offset + 16);
                msg->dateOfLsGpsDn = getUByte16_LEnd(message->buffer, offset + 18);
                msg->validCurrLs = message->buffer[offset + 23] & 0x01;
                msg->validTimeToLsEvent = (message->buffer[offset + 23] >> 1) & 0x01;

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] srcOfCurrLs: "); Serial.println(msg->srcOfCurrLs);
                Serial.print("[DEBUG] currLs: "); Serial.println(msg->currLs);
                Serial.print("[DEBUG] srcOfLsChange: "); Serial.println(msg->srcOfLsChange);
                Serial.print("[DEBUG] lsChange: "); Serial.println(msg->lsChange);
                Serial.print("[DEBUG] timeToLsEvent: "); Serial.println(msg->timeToLsEvent);
                Serial.print("[DEBUG] dateOfLsGpsWn: "); Serial.println(msg->dateOfLsGpsWn);
                Serial.print("[DEBUG] dateOfLsGpsDn: "); Serial.println(msg->dateOfLsGpsDn);
                Serial.print("[DEBUG] validCurrLs: "); Serial.println(msg->validCurrLs);
                Serial.print("[DEBUG] validTimeToLsEvent: "); Serial.println(msg->validTimeToLsEvent);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMENAVIC: // UBX-NAV-TIMENAVIC
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMENAVIC");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimenavic == NULL)
                {
                    gnss_ubx_nav_timenavic_t *ptr = (gnss_ubx_nav_timenavic_t *)malloc(sizeof(gnss_ubx_nav_timenavic_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMENAVIC");
                        #endif
                        break;
                    }

                    handle->ubxNavTimenavic = ptr;
                }

                gnss_ubx_nav_timenavic_t *msg = handle->ubxNavTimenavic;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->navicTow = getUByte32_LEnd(message->buffer, offset + 4);
                msg->fNavicTow = getIByte32_LEnd(message->buffer, offset + 8);
                msg->navicWno = getIByte16_LEnd(message->buffer, offset + 12);
                msg->leapS = message->buffer[offset + 14];
                msg->navicTowValid = message->buffer[offset + 15] & 0x01;
                msg->navicWnoValid = (message->buffer[offset + 15] >> 1) & 0x01;
                msg->leapSValid = (message->buffer[offset + 15] >> 2) & 0x01;
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] navicTow: "); Serial.println(msg->navicTow);
                Serial.print("[DEBUG] fNavicTow: "); Serial.println(msg->fNavicTow);
                Serial.print("[DEBUG] navicWno: "); Serial.println(msg->navicWno);
                Serial.print("[DEBUG] leapS: "); Serial.println(msg->leapS);
                Serial.print("[DEBUG] navicTowValid: "); Serial.println(msg->navicTowValid);
                Serial.print("[DEBUG] navicWnoValid: "); Serial.println(msg->navicWnoValid);
                Serial.print("[DEBUG] leapSValid: "); Serial.println(msg->leapSValid);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMEQZSS: // UBX-NAV-TIMEQZSS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMEQZSS");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimeqzss == NULL)
                {
                    gnss_ubx_nav_timeqzss_t *ptr = (gnss_ubx_nav_timeqzss_t *)malloc(sizeof(gnss_ubx_nav_timeqzss_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMEQZSS");
                        #endif
                        break;
                    }

                    handle->ubxNavTimeqzss = ptr;
                }

                gnss_ubx_nav_timeqzss_t *msg = handle->ubxNavTimeqzss;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->qzssTow = getUByte32_LEnd(message->buffer, offset + 4);
                msg->fQzssTow = getIByte32_LEnd(message->buffer, offset + 8);
                msg->qzssWno = getIByte16_LEnd(message->buffer, offset + 12);
                msg->leapS = message->buffer[offset + 14];
                msg->qzssTowValid = message->buffer[offset + 15] & 0x01;
                msg->qzssWnoValid = (message->buffer[offset + 15] >> 1) & 0x01;
                msg->leapSValid = (message->buffer[offset + 15] >> 2) & 0x01;
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] qzssTow: "); Serial.println(msg->qzssTow);
                Serial.print("[DEBUG] fQzssTow: "); Serial.println(msg->fQzssTow);
                Serial.print("[DEBUG] qzssWno: "); Serial.println(msg->qzssWno);
                Serial.print("[DEBUG] leapS: "); Serial.println(msg->leapS);
                Serial.print("[DEBUG] qzssTowValid: "); Serial.println(msg->qzssTowValid);
                Serial.print("[DEBUG] qzssWnoValid: "); Serial.println(msg->qzssWnoValid);
                Serial.print("[DEBUG] leapSValid: "); Serial.println(msg->leapSValid);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_TIMEUTC: // UBX-NAV-TIMEUTC
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-TIMEUTC");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavTimeutc == NULL)
                {
                    gnss_ubx_nav_timeutc_t *ptr = (gnss_ubx_nav_timeutc_t *)malloc(sizeof(gnss_ubx_nav_timeutc_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-TIMEUTC");
                        #endif
                        break;
                    }

                    handle->ubxNavTimeutc = ptr;
                }

                gnss_ubx_nav_timeutc_t *msg = handle->ubxNavTimeutc;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 4);
                msg->nano = getIByte32_LEnd(message->buffer, offset + 8);
                msg->year = getUByte16_LEnd(message->buffer, offset + 12);
                msg->month = message->buffer[offset + 14];
                msg->day = message->buffer[offset + 15];
                msg->hour = message->buffer[offset + 16];
                msg->min = message->buffer[offset + 17];
                msg->sec = message->buffer[offset + 18];
                msg->validTOW = message->buffer[offset + 19] & 0x01;
                msg->validWKN = (message->buffer[offset + 19] >> 1) & 0x01;
                msg->validUTC = (message->buffer[offset + 19] >> 2) & 0x01;
                msg->authStatus = (message->buffer[offset + 19] >> 3) & 0x01;
                msg->utcStandard = (message->buffer[offset + 19] >> 4) & 0x0F;

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] nano: "); Serial.println(msg->nano);
                Serial.print("[DEBUG] year: "); Serial.println(msg->year);
                Serial.print("[DEBUG] month: "); Serial.println(msg->month);
                Serial.print("[DEBUG] day: "); Serial.println(msg->day);
                Serial.print("[DEBUG] hour: "); Serial.println(msg->hour);
                Serial.print("[DEBUG] min: "); Serial.println(msg->min);
                Serial.print("[DEBUG] sec: "); Serial.println(msg->sec);
                Serial.print("[DEBUG] validTOW: "); Serial.println(msg->validTOW);
                Serial.print("[DEBUG] validWKN: "); Serial.println(msg->validWKN);
                Serial.print("[DEBUG] validUTC: "); Serial.println(msg->validUTC);
                Serial.print("[DEBUG] authStatus: "); Serial.println(msg->authStatus);
                Serial.print("[DEBUG] utcStandard: "); Serial.println(msg->utcStandard);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_VELECEF: // UBX-NAV-VELECEF
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-VELECEF");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavVelecef == NULL)
                {
                    gnss_ubx_nav_velecef_t *ptr = (gnss_ubx_nav_velecef_t *)malloc(sizeof(gnss_ubx_nav_velecef_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-VELECEF");
                        #endif
                        break;
                    }

                    handle->ubxNavVelecef = ptr;
                }

                gnss_ubx_nav_velecef_t *msg = handle->ubxNavVelecef;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->ecefVX = getIByte32_LEnd(message->buffer, offset + 4);
                msg->ecefVY = getIByte32_LEnd(message->buffer, offset + 8);
                msg->ecefVZ = getIByte32_LEnd(message->buffer, offset + 12);
                msg->sAcc = getUByte32_LEnd(message->buffer, offset + 16);

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] ecefVX: "); Serial.println(msg->ecefVX);
                Serial.print("[DEBUG] ecefVY: "); Serial.println(msg->ecefVY);
                Serial.print("[DEBUG] ecefVZ: "); Serial.println(msg->ecefVZ);
                Serial.print("[DEBUG] sAcc: "); Serial.println(msg->sAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_NAV_VELNED: // UBX-NAV-VELNED
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-VELNED");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxNavVelned == NULL)
                {
                    gnss_ubx_nav_velned_t *ptr = (gnss_ubx_nav_velned_t *)malloc(sizeof(gnss_ubx_nav_velned_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-NAV-VELNED");
                        #endif
                        break;
                    }

                    handle->ubxNavVelned = ptr;
                }

                gnss_ubx_nav_velned_t *msg = handle->ubxNavVelned;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timestamp = getUByte32_LEnd(message->buffer, offset);
                if (timestamp == msg->iTOW)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->iTOW = timestamp;
                msg->velN = getIByte32_LEnd(message->buffer, offset + 4);
                msg->velE = getIByte32_LEnd(message->buffer, offset + 8);
                msg->velD = getIByte32_LEnd(message->buffer, offset + 12);
                msg->speed = getUByte32_LEnd(message->buffer, offset + 16);
                msg->gSpeed = getUByte32_LEnd(message->buffer, offset + 20);
                msg->heading = getIByte32_LEnd(message->buffer, offset + 24) / 100000.0;
                msg->sAcc = getUByte32_LEnd(message->buffer, offset + 28);
                msg->cAcc = getUByte32_LEnd(message->buffer, offset + 32) / 100000.0;

                #ifdef DEBUG
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] velN: "); Serial.println(msg->velN);
                Serial.print("[DEBUG] velE: "); Serial.println(msg->velE);
                Serial.print("[DEBUG] velD: "); Serial.println(msg->velD);
                Serial.print("[DEBUG] speed: "); Serial.println(msg->speed);
                Serial.print("[DEBUG] gSpeed: "); Serial.println(msg->gSpeed);
                Serial.print("[DEBUG] heading: "); Serial.println(msg->heading);
                Serial.print("[DEBUG] sAcc: "); Serial.println(msg->sAcc);
                Serial.print("[DEBUG] cAcc: "); Serial.println(msg->cAcc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_RXM_MEAS20: // UBX-RXM-MEAS20
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-RXM-MEAS20");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxRxmMeas20 == NULL)
                {
                    gnss_ubx_rxm_meas20_t *ptr = (gnss_ubx_rxm_meas20_t *)malloc(sizeof(gnss_ubx_rxm_meas20_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-RXM-MEAS20");
                        #endif
                        break;
                    }

                    handle->ubxRxmMeas20 = ptr;
                }

                gnss_ubx_rxm_meas20_t *msg = handle->ubxRxmMeas20;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                if (message->length > 26)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Payload greater than 20 bytes");
                    #endif
                }

                msg->stale = false;

                for (uint8_t j = 0; j < message->length - 2 - offset; j++)
                {
                    msg->payload[j] = message->buffer[offset + j];

                    #ifdef DEBUG
                    Serial.print(msg->payload[j]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_RXM_MEAS50: // UBX-RXM-MEAS50
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-RXM-MEAS50");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxRxmMeas50 == NULL)
                {
                    gnss_ubx_rxm_meas50_t *ptr = (gnss_ubx_rxm_meas50_t *)malloc(sizeof(gnss_ubx_rxm_meas50_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-RXM-MEAS50");
                        #endif
                        break;
                    }

                    handle->ubxRxmMeas50 = ptr;
                }

                gnss_ubx_rxm_meas50_t *msg = handle->ubxRxmMeas50;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                if (message->length > 56)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Payload greater than 50 bytes");
                    #endif
                }

                msg->stale = false;

                for (uint8_t j = 0; j < message->length - 2 - offset; j++)
                {
                    msg->payload[j] = message->buffer[offset + j];

                    #ifdef DEBUG
                    Serial.print(msg->payload[j]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_RXM_MEASC12: // UBX-RXM-MEASC12
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-RXM-MEASC12");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxRxmMeasc12 == NULL)
                {
                    gnss_ubx_rxm_measc12_t *ptr = (gnss_ubx_rxm_measc12_t *)malloc(sizeof(gnss_ubx_rxm_measc12_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-RXM-MEASC12");
                        #endif
                        break;
                    }

                    handle->ubxRxmMeasc12 = ptr;
                }

                gnss_ubx_rxm_measc12_t *msg = handle->ubxRxmMeasc12;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                if (message->length > 18)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Payload greater than 12 bytes");
                    #endif
                }

                msg->stale = false;

                for (uint8_t j = 0; j < message->length - 2 - offset; j++)
                {
                    msg->payload[j] = message->buffer[offset + j];

                    #ifdef DEBUG
                    Serial.print(msg->payload[j]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_RXM_MEASD12: // UBX-RXM-MEASD12
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-RXM-MEASD12");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxRxmMeasd12 == NULL)
                {
                    gnss_ubx_rxm_measd12_t *ptr = (gnss_ubx_rxm_measd12_t *)malloc(sizeof(gnss_ubx_rxm_measd12_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-RXM-MEASD12");
                        #endif
                        break;
                    }

                    handle->ubxRxmMeasd12 = ptr;
                }

                gnss_ubx_rxm_measd12_t *msg = handle->ubxRxmMeasd12;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                if (message->length > 18)
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Payload greater than 12 bytes");
                    #endif
                }

                msg->stale = false;

                for (uint8_t j = 0; j < message->length - 2 - offset; j++)
                {
                    msg->payload[j] = message->buffer[offset + j];

                    #ifdef DEBUG
                    Serial.print(msg->payload[j]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_RXM_MEASX: // UBX-RXM-MEASX
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-NAV-MEASX");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxRxmMeasx == NULL)
                {
                    gnss_ubx_rxm_measx_t *ptr = (gnss_ubx_rxm_measx_t *)malloc(sizeof(gnss_ubx_rxm_measx_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-RXM-MEASX");
                        #endif
                        break;
                    }

                    handle->ubxRxmMeasx = ptr;
                }

                gnss_ubx_rxm_measx_t *msg = handle->ubxRxmMeasx;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->version = message->buffer[offset];
                msg->gpsTOW = getUByte32_LEnd(message->buffer, offset + 4);
                msg->gloTOW = getUByte32_LEnd(message->buffer, offset + 8);
                msg->bdsTOW = getUByte32_LEnd(message->buffer, offset + 12);
                msg->qzssTOW = getUByte32_LEnd(message->buffer, offset + 20);
                msg->gpsTOWacc = getUByte16_LEnd(message->buffer, offset + 24) / 16.0; // or 2 ^ -4
                msg->gloTOWacc = getUByte16_LEnd(message->buffer, offset + 26) / 16.0; // or 2 ^ -4
                msg->bdsTOWacc = getUByte16_LEnd(message->buffer, offset + 28) / 16.0; // or 2 ^ -4
                msg->qzssTOWacc = getUByte16_LEnd(message->buffer, offset + 32) / 16.0; // or 2 ^ -4
                msg->numSV = message->buffer[offset + 34];
                msg->towSet = message->buffer[offset + 35] & 0x03;

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] gpsTOW: "); Serial.println(msg->gpsTOW);
                Serial.print("[DEBUG] gloTOW: "); Serial.println(msg->gloTOW);
                Serial.print("[DEBUG] bdsTOW: "); Serial.println(msg->bdsTOW);
                Serial.print("[DEBUG] qzssTOW: "); Serial.println(msg->qzssTOW);
                Serial.print("[DEBUG] gpsTOWacc: "); Serial.println(msg->gpsTOWacc);
                Serial.print("[DEBUG] gloTOWacc: "); Serial.println(msg->gloTOWacc);
                Serial.print("[DEBUG] bdsTOWacc: "); Serial.println(msg->bdsTOWacc);
                Serial.print("[DEBUG] qzssTOWacc: "); Serial.println(msg->qzssTOWacc);
                Serial.print("[DEBUG] numSV: "); Serial.println(msg->numSV);
                Serial.print("[DEBUG] towSet: "); Serial.println(msg->towSet);
                #endif

                for (uint8_t i = 0; i < msg->numSV; i++)
                {
                    msg->sv[i].gnssID = message->buffer[offset + 44 + i * 24];
                    msg->sv[i].svID = message->buffer[offset + 45 + i * 24];
                    msg->sv[i].cNo = message->buffer[offset + 46 + i * 24];
                    msg->sv[i].mpathIndic = message->buffer[offset + 47 + i * 24];
                    msg->sv[i].dopplerMS = getIByte32_LEnd(message->buffer, offset + 48 + i * 24) * 0.04;
                    msg->sv[i].dopplerHz = getIByte32_LEnd(message->buffer, offset + 52 + i * 24) * 0.2;
                    msg->sv[i].wholeChips = getUByte16_LEnd(message->buffer, offset + 56 + i * 24);
                    msg->sv[i].fracChips = getUByte16_LEnd(message->buffer, offset + 58 + i * 24);
                    msg->sv[i].codePhase = getUByte32_LEnd(message->buffer, offset + 60 + i * 24) / 2097152.0;
                    msg->sv[i].intCodePhase = message->buffer[offset + 64 + i * 24];
                    msg->sv[i].pseuRangeRMSErr = message->buffer[offset + 65 + i * 24];

                    #ifdef DEBUG
                    Serial.print("[DEBUG] gnssId: "); Serial.println(msg->sv[i].gnssID);
                    Serial.print("[DEBUG] svID: "); Serial.println(msg->sv[i].svID);
                    Serial.print("[DEBUG] cNo: "); Serial.println(msg->sv[i].cNo);
                    Serial.print("[DEBUG] mpathIndic: "); Serial.println(msg->sv[i].mpathIndic);
                    Serial.print("[DEBUG] dopplerMS: "); Serial.println(msg->sv[i].dopplerMS);
                    Serial.print("[DEBUG] dopplerHz: "); Serial.println(msg->sv[i].dopplerHz);
                    Serial.print("[DEBUG] wholeChips: "); Serial.println(msg->sv[i].wholeChips);
                    Serial.print("[DEBUG] fracChips: "); Serial.println(msg->sv[i].fracChips);
                    Serial.print("[DEBUG] codePhase: "); Serial.println(msg->sv[i].codePhase);
                    Serial.print("[DEBUG] intCodePhase: "); Serial.println(msg->sv[i].intCodePhase);
                    Serial.print("[DEBUG] pseuRangeRMSErr: "); Serial.println(msg->sv[i].pseuRangeRMSErr);
                    #endif

                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_RXM_RLM: // UBX-RXM-RLM
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-RXM-RLM");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxRxmRlm == NULL)
                {
                    gnss_ubx_rxm_rlm_t *ptr = (gnss_ubx_rxm_rlm_t *)malloc(sizeof(gnss_ubx_rxm_rlm_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-RXM-RLM");
                        #endif
                        break;
                    }

                    handle->ubxRxmRlm = ptr;
                }

                gnss_ubx_rxm_rlm_t *msg = handle->ubxRxmRlm;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset];
                msg->type = message->buffer[offset + 1];
                msg->svID = message->buffer[offset + 2];

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] type: "); Serial.println(msg->type);
                Serial.print("[DEBUG] svID: "); Serial.println(msg->svID);
                Serial.print("[DEBUG] beacon: ");
                #endif

                for (uint8_t i = 0; i < 8; i++)
                {
                    msg->beacon[i] = message->buffer[offset + 4 + i];
                    #ifdef DEBUG
                    Serial.print(msg->beacon[i]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.println();
                #endif

                msg->message = message->buffer[offset + 12];

                #ifdef DEBUG
                Serial.print("[DEBUG] message: "); Serial.println(msg->message);
                Serial.print("[DEBUG] params: ");
                #endif

                for (uint8_t i = 0; i < (msg->type == 0x01 ? 2 : 12); i++)
                {
                    msg->params[i] = message->buffer[offset + 13 + i];
                    #ifdef DEBUG
                    Serial.print(msg->params[i]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.println();
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_RXM_SFRBX: // UBX-RXM-SFRBX
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-RXM-SFRBX");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxRxmSfrbx == NULL)
                {
                    gnss_ubx_rxm_sfrbx_t *ptr = (gnss_ubx_rxm_sfrbx_t *)malloc(sizeof(gnss_ubx_rxm_sfrbx_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-RXM-SFRBX");
                        #endif
                        break;
                    }

                    handle->ubxRxmSfrbx = ptr;
                }

                gnss_ubx_rxm_sfrbx_t *msg = handle->ubxRxmSfrbx;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->gnssID = message->buffer[offset];
                msg->svID = message->buffer[offset + 1];
                msg->sigID = message->buffer[offset + 2];
                msg->freqID = message->buffer[offset + 3];
                msg->numWords = message->buffer[offset + 4];
                msg->chn = message->buffer[offset + 5];
                msg->version = message->buffer[offset + 6];

                #ifdef DEBUG
                Serial.print("[DEBUG] gnssID: "); Serial.println(msg->gnssID);
                Serial.print("[DEBUG] svID: "); Serial.println(msg->svID);
                Serial.print("[DEBUG] sigID: "); Serial.println(msg->sigID);
                Serial.print("[DEBUG] freqID: "); Serial.println(msg->freqID);
                Serial.print("[DEBUG] numWords: "); Serial.println(msg->numWords);
                Serial.print("[DEBUG] chn: "); Serial.println(msg->chn);
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] dwrd: ");
                #endif

                for (uint8_t i = 0; i < msg->numWords * 10; i++)
                {
                    msg->dwrd[i] = message->buffer[offset + 8 + i];
                    #ifdef DEBUG
                    Serial.print(msg->dwrd[i]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.println();
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_SEC_SIG: // UBX-SEC-SIG
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-SEC-SIG");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxSecSig == NULL)
                {
                    gnss_ubx_sec_sig_t *ptr = (gnss_ubx_sec_sig_t *)malloc(sizeof(gnss_ubx_sec_sig_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-SEC-SIG");
                        #endif
                        break;
                    }

                    handle->ubxSecSig = ptr;
                }

                gnss_ubx_sec_sig_t *msg = handle->ubxSecSig;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->version = message->buffer[offset];
                msg->jamDetEna = message->buffer[offset + 4] & 0x01;
                msg->jammingState = (message->buffer[offset + 4] >> 1) & 0x03;
                msg->spfDetEna = message->buffer[offset + 8] & 0x01;
                msg->spoofingState = (message->buffer[offset + 8] >> 1) & 0x07;

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] jamDetEnabled: "); Serial.println(msg->jamDetEna);
                Serial.print("[DEBUG] jammingState: "); Serial.println(msg->jammingState);
                Serial.print("[DEBUG] spfDetEnabled: "); Serial.println(msg->spfDetEna);
                Serial.print("[DEBUG] spoofingState: "); Serial.println(msg->spoofingState);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_SEC_SIGLOG: // UBX-SEC-SIGLOG
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-SEC-SIGLOG");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxSecSiglog == NULL)
                {
                    gnss_ubx_sec_siglog_t *ptr = (gnss_ubx_sec_siglog_t *)malloc(sizeof(gnss_ubx_sec_siglog_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-SEC-SIG");
                        #endif
                        break;
                    }

                    handle->ubxSecSiglog = ptr;
                }

                gnss_ubx_sec_siglog_t *msg = handle->ubxSecSiglog;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->version = message->buffer[offset];
                msg->numEvents = message->buffer[offset + 1];

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] numEvents: "); Serial.println(msg->numEvents);
                #endif

                for (uint8_t i = 0; i < message->buffer[offset + 1]; i++)
                {
                    msg->event[i].timeElapsed = getUByte32_LEnd(message->buffer, offset + 8 + i * 8);
                    msg->event[i].detectionType = message->buffer[offset + 12 + i * 8];
                    msg->event[i].eventType = message->buffer[offset + 13 + i * 8];

                    #ifdef DEBUG
                    Serial.print("[DEBUG] timeElapsed: "); Serial.println(msg->event[i].timeElapsed);
                    Serial.print("[DEBUG] detectionType: "); Serial.println(msg->event[i].detectionType);
                    Serial.print("[DEBUG] eventType: "); Serial.println(msg->event[i].eventType);
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_SEC_UNIQID: // UBX-SEC-UNIQID
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-SEC-UNIQID");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxSecUniqid == NULL)
                {
                    gnss_ubx_sec_uniqid_t *ptr = (gnss_ubx_sec_uniqid_t *)malloc(sizeof(gnss_ubx_sec_uniqid_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-SEC-UNIQID");
                        #endif
                        break;
                    }

                    handle->ubxSecUniqid = ptr;
                }

                gnss_ubx_sec_uniqid_t *msg = handle->ubxSecUniqid;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset];

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] uniqueID: ");
                #endif

                for (uint8_t i = 0; i < 6; i++)
                {
                    msg->uniqueID[i] = message->buffer[offset + 4 + i];
                    #ifdef DEBUG
                    Serial.print(msg->uniqueID[i]); Serial.print(" ");
                    #endif
                }

                #ifdef DEBUG
                Serial.println();
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_TIM_TM2: // UBX-TIM-TM2
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-TIM-TM2");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxTimTm2 == NULL)
                {
                    gnss_ubx_tim_tm2_t *ptr = (gnss_ubx_tim_tm2_t *)malloc(sizeof(gnss_ubx_tim_tm2_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-TIM-TM2");
                        #endif
                        break;
                    }

                    handle->ubxTimTm2 = ptr;
                }

                gnss_ubx_tim_tm2_t *msg = handle->ubxTimTm2;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->ch = message->buffer[offset];
                msg->mode = message->buffer[offset + 1] & 0x01;
                msg->run = (message->buffer[offset + 1] >> 1) & 0x01;
                msg->newFallingEdge = (message->buffer[offset + 1] >> 2) & 0x01;
                msg->timeBase = (message->buffer[offset + 1] >> 3) & 0x03;
                msg->utc = (message->buffer[offset + 1] >> 5) & 0x01;
                msg->time = (message->buffer[offset + 1] >> 6) & 0x01;
                msg->newRisingEdge = (message->buffer[offset + 1] >> 7) & 0x01;
                msg->count = getUByte16_LEnd(message->buffer, offset + 2);
                msg->wnR = getUByte16_LEnd(message->buffer, offset + 4);
                msg->wnF = getUByte16_LEnd(message->buffer, offset + 6);
                msg->towMsR = getUByte32_LEnd(message->buffer, offset + 8);
                msg->towSubMsR = getUByte32_LEnd(message->buffer, offset + 12);
                msg->towMsF = getUByte32_LEnd(message->buffer, offset + 16);
                msg->towSubMsF = getUByte32_LEnd(message->buffer, offset + 20);
                msg->accEst = getUByte32_LEnd(message->buffer, offset + 24);

                #ifdef DEBUG
                Serial.print("[DEBUG] ch: "); Serial.println(msg->ch);
                Serial.print("[DEBUG] mode: "); Serial.println(msg->mode);
                Serial.print("[DEBUG] run: "); Serial.println(msg->run);
                Serial.print("[DEBUG] newFallingEdge: "); Serial.println(msg->newFallingEdge);
                Serial.print("[DEBUG] timeBase: "); Serial.println(msg->timeBase);
                Serial.print("[DEBUG] utc: "); Serial.println(msg->utc);
                Serial.print("[DEBUG] time: "); Serial.println(msg->time);
                Serial.print("[DEBUG] newRisingEdge: "); Serial.println(msg->newRisingEdge);
                Serial.print("[DEBUG] wnR: "); Serial.println(msg->wnR);
                Serial.print("[DEBUG] wnF: "); Serial.println(msg->wnF);
                Serial.print("[DEBUG] towMsR: "); Serial.println(msg->towMsR);
                Serial.print("[DEBUG] towSubMsR: "); Serial.println(msg->towSubMsR);
                Serial.print("[DEBUG] towMsF: "); Serial.println(msg->towMsF);
                Serial.print("[DEBUG] towSubMsF: "); Serial.println(msg->towSubMsF);
                Serial.print("[DEBUG] accEst: "); Serial.println(msg->accEst);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_TIM_TP: // UBX-TIM-TP
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-TIM-TP");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxTimTp == NULL)
                {
                    gnss_ubx_tim_tp_t *ptr = (gnss_ubx_tim_tp_t *)malloc(sizeof(gnss_ubx_tim_tp_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-TIM-TP");
                        #endif
                        break;
                    }

                    handle->ubxTimTp = ptr;
                }

                gnss_ubx_tim_tp_t *msg = handle->ubxTimTp;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->towMS = getUByte32_LEnd(message->buffer, offset);
                msg->towSubMS = getUByte32_LEnd(message->buffer, offset + 4);
                msg->qErr = getIByte32_LEnd(message->buffer, offset + 8);
                msg->week = getUByte16_LEnd(message->buffer, offset + 12);
                msg->timeBase = message->buffer[offset + 14] & 0x01;
                msg->utc = (message->buffer[offset + 14] >> 1) & 0x01;
                msg->raim = (message->buffer[offset + 14] >> 2) & 0x03;
                msg->qErrInvalid = (message->buffer[offset + 14] >> 4) & 0x01;
                msg->TpNotLocked = (message->buffer[offset + 14] >> 5) & 0x01;
                msg->timeRefGnss = message->buffer[offset + 15] & 0x0F;
                msg->utcStandard = (message->buffer[offset + 15] >> 4) & 0x0F;

                #ifdef DEBUG
                Serial.print("[DEBUG] towMS: "); Serial.println(msg->towMS);
                Serial.print("[DEBUG] towSubMS: "); Serial.println(msg->towSubMS);
                Serial.print("[DEBUG] qErr: "); Serial.println(msg->qErr);
                Serial.print("[DEBUG] week: "); Serial.println(msg->week);
                Serial.print("[DEBUG] timeBase: "); Serial.println(msg->timeBase);
                Serial.print("[DEBUG] utc: "); Serial.println(msg->utc);
                Serial.print("[DEBUG] raim: "); Serial.println(msg->raim);
                Serial.print("[DEBUG] qErrInvalid: "); Serial.println(msg->qErrInvalid);
                Serial.print("[DEBUG] TpNotLocked: "); Serial.println(msg->TpNotLocked);
                Serial.print("[DEBUG] timeRefGnss: "); Serial.println(msg->timeRefGnss);
                Serial.print("[DEBUG] utcStandard: "); Serial.println(msg->utcStandard);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_TIM_VRFY: // UBX-TIM-VRFY
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-TIM-VRFY");
                #endif

                int32_t timeCheck; // Normal timestamp is uint32_t

                // Allocate memory for the message if not already done
                if (handle->ubxTimVrfy == NULL)
                {
                    gnss_ubx_tim_vrfy_t *ptr = (gnss_ubx_tim_vrfy_t *)malloc(sizeof(gnss_ubx_tim_vrfy_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-TIM-VRFY");
                        #endif
                        break;
                    }

                    handle->ubxTimVrfy = ptr;
                }

                gnss_ubx_tim_vrfy_t *msg = handle->ubxTimVrfy;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                timeCheck = getIByte32_LEnd(message->buffer, offset);
                if (timeCheck == msg->itow)
                {
                    msg->stale = true;
                    break;
                }
                else
                    msg->stale = false;

                msg->itow = getIByte32_LEnd(message->buffer, offset);
                msg->frac = getIByte32_LEnd(message->buffer, offset + 4);
                msg->deltaMs = getIByte32_LEnd(message->buffer, offset + 8);
                msg->deltaNs = getIByte32_LEnd(message->buffer, offset + 12);
                msg->wno = getUByte16_LEnd(message->buffer, offset + 16);
                msg->src = message->buffer[offset + 17] & 0x07;

                #ifdef DEBUG
                Serial.print("[DEBUG] itow: "); Serial.println(msg->itow);
                Serial.print("[DEBUG] frac: "); Serial.println(msg->frac);
                Serial.print("[DEBUG] deltaMs: "); Serial.println(msg->deltaMs);
                Serial.print("[DEBUG] deltaNs: "); Serial.println(msg->deltaNs);
                Serial.print("[DEBUG] wno: "); Serial.println(msg->wno);
                Serial.print("[DEBUG] src: "); Serial.println(msg->src);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_INF_DEBUG: // UBX-INF-DEBUG
                Serial.println("[DEBUG] UBX-INF-DEBUG");
                j = 6;
                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_create_info_node(handle, message, GNSS_UBX_INF_DEBUG);

                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);

                break;
            case GNSS_UBX_INF_ERROR: // UBX-INF-ERROR
                Serial.println("[DEBUG] UBX-INF-ERROR");
                j = 6;
                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_create_info_node(handle, message, GNSS_UBX_INF_ERROR);

                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);

                break;
            case GNSS_UBX_INF_NOTICE: // UBX-INF-NOTICE
                Serial.println("[DEBUG] UBX-INF-NOTICE");
                j = 6;
                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_create_info_node(handle, message, GNSS_UBX_INF_NOTICE);

                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);

                break;
            case GNSS_UBX_INF_TEST: // UBX-INF-TEST
                Serial.println("[DEBUG] UBX-INF-TEST");
                j = 6;
                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_create_info_node(handle, message, GNSS_UBX_INF_TEST);

                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);

                break;
            case GNSS_UBX_INF_WARNING: // UBX-INF-WARNING
                Serial.println("[DEBUG] UBX-INF-WARNING");
                j = 6;
                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                gnss_create_info_node(handle, message, GNSS_UBX_INF_WARNING);

                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);

                break;
            case GNSS_UBX_LOG_BATCH: // UBX-LOG-BATCH
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-LOG-BATCH");
                #endif

                gnss_ubx_log_batch_t *msg = handle->ubxLogBatch[handle->batchQueue];

                handle->batchQueue++; // Iterate to next batch message

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset + 0];
                msg->extraPVT = message->buffer[offset + 1] & 0x01;
                msg->extraODO = (message->buffer[offset + 1] >> 1) & 0x01;
                msg->msgCnt = getUByte16_LEnd(message->buffer, offset + 2);

                msg->iTOW = getUByte32_LEnd(message->buffer, offset + 4);;
                msg->validDate = message->buffer[offset + 15] & 0x01;
                msg->validTime = (message->buffer[offset + 15] >> 1) & 0x01;

                if (msg->validDate)
                {
                    msg->yearUTC = getUByte16_LEnd(message->buffer, offset + 8);
                    msg->monthUTC = message->buffer[offset + 10];
                    msg->dayUTC = message->buffer[offset + 11];
                }
                else
                {
                    msg->yearUTC = 0;
                    msg->monthUTC = 0;
                    msg->dayUTC = 0;
                }

                if (msg->validTime)
                {
                    msg->hoursUTC = message->buffer[offset + 12];
                    msg->minutesUTC = message->buffer[offset + 13];
                    msg->secondsUTC = message->buffer[offset + 14];
                }
                else
                {
                    msg->hoursUTC = 0;
                    msg->minutesUTC = 0;
                    msg->secondsUTC = 0;
                }

                msg->tAcc = getUByte32_LEnd(message->buffer, offset + 16);
                msg->nanosUTC = getIByte32_LEnd(message->buffer, offset + 20);
                msg->fixType = message->buffer[offset + 24];
                msg->gnssFixOK = message->buffer[offset + 25] & 0x01;
                msg->diffSoln = (message->buffer[offset + 25] >> 1) & 0x01;
                msg->psmState = (message->buffer[offset + 25] >> 2) & 0x07;

                msg->flags2 = message->buffer[offset + 26];
                msg->numSV = message->buffer[offset + 27];
                
                msg->longitude = (getIByte32_LEnd(message->buffer, offset + 28)) / 10000000.0;
                msg->latitude = (getIByte32_LEnd(message->buffer, offset + 32)) / 10000000.0;

                msg->heightEllip = getIByte32_LEnd(message->buffer, offset + 36);
                msg->hMSL = getIByte32_LEnd(message->buffer, offset + 40);
                msg->hAcc = getUByte32_LEnd(message->buffer, offset + 44);
                msg->vAcc = getUByte32_LEnd(message->buffer, offset + 48);
                msg->velN = getIByte32_LEnd(message->buffer, offset + 52);
                msg->velE = getIByte32_LEnd(message->buffer, offset + 56);
                msg->velD = getIByte32_LEnd(message->buffer, offset + 60);
                msg->gSpeed = getIByte32_LEnd(message->buffer, offset + 64);
                msg->course = getIByte32_LEnd(message->buffer, offset + 68) / 100000.0;
                msg->sAcc = getUByte32_LEnd(message->buffer, offset + 72);
                msg->headAcc = getUByte32_LEnd(message->buffer, offset + 76) / 100000.0;
                msg->pdop = getUByte16_LEnd(message->buffer, offset + 80) / 100.0;

                msg->distance = getUByte32_LEnd(message->buffer, offset + 84);
                msg->totalDistance = getUByte32_LEnd(message->buffer, offset + 88);
                msg->distanceStd = getUByte32_LEnd(message->buffer, offset + 92);
                
                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] extraPVT: "); Serial.println(msg->extraPVT);
                Serial.print("[DEBUG] extraODO: "); Serial.println(msg->extraODO);
                Serial.print("[DEBUG] msgCnt: "); Serial.println(msg->msgCnt);
                Serial.print("[DEBUG] year: "); Serial.println(msg->yearUTC);
                Serial.print("[DEBUG] month: "); Serial.println(msg->monthUTC);
                Serial.print("[DEBUG] day: "); Serial.println(msg->dayUTC);
                Serial.print("[DEBUG] hour: "); Serial.println(msg->hoursUTC);
                Serial.print("[DEBUG] min: "); Serial.println(msg->minutesUTC);
                Serial.print("[DEBUG] sec: "); Serial.println(msg->secondsUTC);
                Serial.print("[DEBUG] validDate: "); Serial.println(msg->validDate);
                Serial.print("[DEBUG] validTime: "); Serial.println(msg->validTime);
                Serial.print("[DEBUG] iTOW: "); Serial.println(msg->iTOW);
                Serial.print("[DEBUG] tAcc: "); Serial.println(msg->tAcc);
                Serial.print("[DEBUG] nano: "); Serial.println(msg->nanosUTC);
                Serial.print("[DEBUG] fixType: "); Serial.println(msg->fixType);
                Serial.print("[DEBUG] gnssFixOK: "); Serial.println(msg->gnssFixOK);
                Serial.print("[DEBUG] diffSoln: "); Serial.println(msg->diffSoln);
                Serial.print("[DEBUG] psmState: "); Serial.println(msg->psmState);
                Serial.print("[DEBUG] flags2: "); Serial.println(msg->flags2);
                Serial.print("[DEBUG] numSV: "); Serial.println(msg->numSV);
                Serial.print("[DEBUG] lon: "); Serial.println(msg->longitude, 6);
                Serial.print("[DEBUG] lat: "); Serial.println(msg->latitude, 6);
                Serial.print("[DEBUG] height: "); Serial.println(msg->heightEllip);
                Serial.print("[DEBUG] hMSL: "); Serial.println(msg->hMSL);
                Serial.print("[DEBUG] hAcc: "); Serial.println(msg->hAcc);
                Serial.print("[DEBUG] vAcc: "); Serial.println(msg->vAcc);
                Serial.print("[DEBUG] velN: "); Serial.println(msg->velN);
                Serial.print("[DEBUG] velE: "); Serial.println(msg->velE);
                Serial.print("[DEBUG] velD: "); Serial.println(msg->velD);
                Serial.print("[DEBUG] gSpeed: "); Serial.println(msg->gSpeed);
                Serial.print("[DEBUG] headMot: "); Serial.println(msg->course);
                Serial.print("[DEBUG] sAcc: "); Serial.println(msg->sAcc);
                Serial.print("[DEBUG] headAcc: "); Serial.println(msg->headAcc);
                Serial.print("[DEBUG] pDOP: "); Serial.println(msg->pdop);
                Serial.print("[DEBUG] distance: "); Serial.println(msg->distance);
                Serial.print("[DEBUG] totalDistance: "); Serial.println(msg->totalDistance);
                Serial.print("[DEBUG] distanceStd: "); Serial.println(msg->distanceStd);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_LOG_FINDTIME: // UBX-LOG-FINDTIME
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-LOG-FINDTIME");
                #endif

                if (handle->ubxLogFindtime == NULL)
                {
                    gnss_ubx_log_findtime_t *ptr = (gnss_ubx_log_findtime_t *)malloc(sizeof(gnss_ubx_log_findtime_t));
                    
                    if (handle->ubxLogFindtime == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-LOG-FINDTIME message");
                        #endif
                        return 1;
                    }

                    handle->ubxLogFindtime = ptr;

                }

                gnss_ubx_log_findtime_t *msg = handle->ubxLogFindtime;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset + 0];
                msg->type = message->buffer[offset + 1];
                msg->entryNumber = getUByte32_LEnd(message->buffer, offset + 4);

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] type: "); Serial.println(msg->type);
                Serial.print("[DEBUG] entryNumber: "); Serial.println(msg->entryNumber);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_LOG_INFO: // UBX-LOG-INFO
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-LOG-INFO");
                #endif

                if (handle->ubxLogInfo == NULL)
                {
                    gnss_ubx_log_info_t *ptr = (gnss_ubx_log_info_t *)malloc(sizeof(gnss_ubx_log_info_t));
                    
                    if (handle->ubxLogInfo == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Failed to allocate initial memory for UBX-LOG-INFO message");
                        #endif
                        return 1;
                    }

                    handle->ubxLogInfo = ptr;

                }

                gnss_ubx_log_info_t *msg = handle->ubxLogInfo;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset + 0];
                msg->filestoreCapacity = getUByte32_LEnd(message->buffer, offset + 4);
                msg->currentMaxLogSize = getUByte32_LEnd(message->buffer, offset + 16);
                msg->currentLogSize = getUByte32_LEnd(message->buffer, offset + 20);
                msg->entryCount = getUByte32_LEnd(message->buffer, offset + 24);
                msg->oldestYear = getUByte16_LEnd(message->buffer, offset + 28);
                msg->oldestMonth = message->buffer[offset + 30];
                msg->oldestDay = message->buffer[offset + 31];
                msg->oldestHour = message->buffer[offset + 32];
                msg->oldestMinute = message->buffer[offset + 33];
                msg->oldestSecond = message->buffer[offset + 34];
                msg->newestYear = getUByte16_LEnd(message->buffer, offset + 36);
                msg->newestMonth = message->buffer[offset + 38];
                msg->newestDay = message->buffer[offset + 39];
                msg->newestHour = message->buffer[offset + 40];
                msg->newestMinute = message->buffer[offset + 41];
                msg->newestSecond = message->buffer[offset + 42];
                msg->recording = (message->buffer[offset + 44] >> 3) & 0x01;
                msg->inactive = (message->buffer[offset + 44] >> 4) & 0x01;
                msg->circular = (message->buffer[offset + 44] >> 5) & 0x01;

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] filestoreCapacity: "); Serial.println(msg->filestoreCapacity);
                Serial.print("[DEBUG] currentMaxLogSize: "); Serial.println(msg->currentMaxLogSize);
                Serial.print("[DEBUG] currentLogSize: "); Serial.println(msg->currentLogSize);
                Serial.print("[DEBUG] entryCount: "); Serial.println(msg->entryCount);
                Serial.print("[DEBUG] oldestYear: "); Serial.println(msg->oldestYear);
                Serial.print("[DEBUG] oldestMonth: "); Serial.println(msg->oldestMonth);
                Serial.print("[DEBUG] oldestDay: "); Serial.println(msg->oldestDay);
                Serial.print("[DEBUG] oldestHour: "); Serial.println(msg->oldestHour);
                Serial.print("[DEBUG] oldestMinute: "); Serial.println(msg->oldestMinute);
                Serial.print("[DEBUG] oldestSecond: "); Serial.println(msg->oldestSecond);
                Serial.print("[DEBUG] newestYear: "); Serial.println(msg->newestYear);
                Serial.print("[DEBUG] newestMonth: "); Serial.println(msg->newestMonth);
                Serial.print("[DEBUG] newestDay: "); Serial.println(msg->newestDay);
                Serial.print("[DEBUG] newestHour: "); Serial.println(msg->newestHour);
                Serial.print("[DEBUG] newestMinute: "); Serial.println(msg->newestMinute);
                Serial.print("[DEBUG] newestSecond: "); Serial.println(msg->newestSecond);
                Serial.print("[DEBUG] recording: "); Serial.println(msg->recording);
                Serial.print("[DEBUG] inactive: "); Serial.println(msg->inactive);
                Serial.print("[DEBUG] circular: "); Serial.println(msg->circular);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_BATCH: // UBX-MON-BATCH
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-BATCH");
                #endif

                gnss_ubx_mon_batch_t *msg = handle->ubxMonBatch;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset + 0];
                msg->fillLevel = getUByte16_LEnd(message->buffer, offset + 4);
                msg->dropsAll = getUByte16_LEnd(message->buffer, offset + 6);
                msg->dropsSinceMon = getUByte16_LEnd(message->buffer, offset + 8);
                msg->nextMsgCnt = getUByte16_LEnd(message->buffer, offset + 10);

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] fillLevel: "); Serial.println(msg->fillLevel);
                Serial.print("[DEBUG] dropsAll: "); Serial.println(msg->dropsAll);
                Serial.print("[DEBUG] dropsSinceMon: "); Serial.println(msg->dropsSinceMon);
                Serial.print("[DEBUG] nextMsgCnt: "); Serial.println(msg->nextMsgCnt);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_COMMS: // UBX-MON-COMMS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-COMMS");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonComms == NULL)
                {
                    gnss_ubx_mon_comms_t *ptr = (gnss_ubx_mon_comms_t *)malloc(sizeof(gnss_ubx_mon_comms_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-COMMS");
                        #endif
                        break;
                    }

                    handle->ubxMonComms = ptr;
                }

                gnss_ubx_mon_comms_t *msg = handle->ubxMonComms;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->version = message->buffer[offset + 0];
                msg->nPorts = message->buffer[offset + 1];
                msg->mem = message->buffer[offset + 2] & 0x01;
                msg->alloc = (message->buffer[offset + 2] >> 1) & 0x01;
                msg->protIds = getUByte32_LEnd(message->buffer, offset + 4);

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] nPorts: "); Serial.println(msg->nPorts);
                Serial.print("[DEBUG] mem: "); Serial.println(msg->mem);
                Serial.print("[DEBUG] alloc: "); Serial.println(msg->alloc);
                Serial.print("[DEBUG] protIds: "); Serial.println(msg->protIds);
                #endif

                for (uint8_t i = 0; i < msg->nPorts; i++)
                {
                    msg->ports[i].portID = getUByte16_LEnd(message->buffer, offset + 8 + 40 * i);
                    msg->ports[i].txPending = getUByte16_LEnd(message->buffer, offset + 10 + 40 * i);
                    msg->ports[i].txBytes = getUByte32_LEnd(message->buffer, offset + 12 + 40 * i);
                    msg->ports[i].txUsage = message->buffer[offset + 16 + 40 * i];
                    msg->ports[i].txPeakUsage = message->buffer[offset + 17 + 40 * i];
                    msg->ports[i].rxPending = getUByte16_LEnd(message->buffer, offset + 18 + 40 * i);
                    msg->ports[i].rxBytes = getUByte32_LEnd(message->buffer, offset + 20 + 40 * i);
                    msg->ports[i].rxUsage = message->buffer[offset + 24 + 40 * i];
                    msg->ports[i].rxPeakUsage = message->buffer[offset + 25 + 40 * i];
                    msg->ports[i].overrunErrs = getUByte16_LEnd(message->buffer, offset + 26 + 40 * i);
                    msg->ports[i].msgs[0] = getUByte16_LEnd(message->buffer, offset + 28 + 40 * i);
                    msg->ports[i].msgs[1] = getUByte16_LEnd(message->buffer, offset + 30 + 40 * i);
                    msg->ports[i].msgs[2] = getUByte16_LEnd(message->buffer, offset + 32 + 40 * i);
                    msg->ports[i].msgs[3] = getUByte16_LEnd(message->buffer, offset + 34 + 40 * i);
                    msg->ports[i].skipped = getUByte32_LEnd(message->buffer, offset + 44 + 40 * i);

                    #ifdef DEBUG
                    Serial.print("[DEBUG] portID: "); Serial.println(msg->ports[i].portID);
                    Serial.print("[DEBUG] txPending: "); Serial.println(msg->ports[i].txPending);
                    Serial.print("[DEBUG] txBytes: "); Serial.println(msg->ports[i].txBytes);
                    Serial.print("[DEBUG] txUsage: "); Serial.println(msg->ports[i].txUsage);
                    Serial.print("[DEBUG] txPeakUsage: "); Serial.println(msg->ports[i].txPeakUsage);
                    Serial.print("[DEBUG] rxPending: "); Serial.println(msg->ports[i].rxPending);
                    Serial.print("[DEBUG] rxBytes: "); Serial.println(msg->ports[i].rxBytes);
                    Serial.print("[DEBUG] rxUsage: "); Serial.println(msg->ports[i].rxUsage);
                    Serial.print("[DEBUG] rxPeakUsage: "); Serial.println(msg->ports[i].rxPeakUsage);
                    Serial.print("[DEBUG] overrunErrs: "); Serial.println(msg->ports[i].overrunErrs);
                    Serial.print("[DEBUG] msgs[0]: "); Serial.println(msg->ports[i].msgs[0]);
                    Serial.print("[DEBUG] msgs[1]: "); Serial.println(msg->ports[i].msgs[1]);
                    Serial.print("[DEBUG] msgs[2]: "); Serial.println(msg->ports[i].msgs[2]);
                    Serial.print("[DEBUG] msgs[3]: "); Serial.println(msg->ports[i].msgs[3]);
                    Serial.print("[DEBUG] skipped: "); Serial.println(msg->ports[i].skipped);
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_GNSS: // UBX-MON-GNSS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-GNSS");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonGnss == NULL)
                {
                    gnss_ubx_mon_gnss_t *ptr = (gnss_ubx_mon_gnss_t *)malloc(sizeof(gnss_ubx_mon_gnss_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-GNSS");
                        #endif
                        break;
                    }

                    handle->ubxMonGnss = ptr;
                }

                gnss_ubx_mon_gnss_t *msg = handle->ubxMonGnss;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset + 0];
                msg->GPSSup = message->buffer[offset + 1] & 0x01;
                msg->glonassSup = (message->buffer[offset + 1] >> 1) & 0x01;
                msg->beidouSup = (message->buffer[offset + 1] >> 2) & 0x01;
                msg->galileoSup = (message->buffer[offset + 1] >> 3) & 0x01;
                msg->GPSDef = message->buffer[offset + 2] & 0x01;
                msg->glonassDef = (message->buffer[offset + 2] >> 1) & 0x01;
                msg->beidouDef = (message->buffer[offset + 2] >> 2) & 0x01;
                msg->galileoDef = (message->buffer[offset + 2] >> 3) & 0x01;
                msg->GPSEna = message->buffer[offset + 3] & 0x01;
                msg->glonassEna = (message->buffer[offset + 3] >> 1) & 0x01;
                msg->beidouEna = (message->buffer[offset + 3] >> 2) & 0x01;
                msg->galileoEna = (message->buffer[offset + 3] >> 3) & 0x01;
                msg->simultaneous = message->buffer[offset + 4];

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] GPSSup: "); Serial.println(msg->GPSSup);
                Serial.print("[DEBUG] glonassSup: "); Serial.println(msg->glonassSup);
                Serial.print("[DEBUG] beidouSup: "); Serial.println(msg->beidouSup);
                Serial.print("[DEBUG] galileoSup: "); Serial.println(msg->galileoSup);
                Serial.print("[DEBUG] GPSDef: "); Serial.println(msg->GPSDef);
                Serial.print("[DEBUG] glonassDef: "); Serial.println(msg->glonassDef);
                Serial.print("[DEBUG] beidouDef: "); Serial.println(msg->beidouDef);
                Serial.print("[DEBUG] galileoDef: "); Serial.println(msg->galileoDef);
                Serial.print("[DEBUG] GPSEna: "); Serial.println(msg->GPSEna);
                Serial.print("[DEBUG] glonassEna: "); Serial.println(msg->glonassEna);
                Serial.print("[DEBUG] beidouEna: "); Serial.println(msg->beidouEna);
                Serial.print("[DEBUG] galileoEna: "); Serial.println(msg->galileoEna);
                Serial.print("[DEBUG] simultaneous: "); Serial.println(msg->simultaneous);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_HW3: // UBX-MON-HW3
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-HW3");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonHw3 == NULL)
                {
                    gnss_ubx_mon_hw3_t *ptr = (gnss_ubx_mon_hw3_t *)malloc(sizeof(gnss_ubx_mon_hw3_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-HW3");
                        #endif
                        break;
                    }

                    handle->ubxMonHw3 = ptr;
                }

                gnss_ubx_mon_hw3_t *msg = handle->ubxMonHw3;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->version = message->buffer[offset + 0];
                msg->nPins = message->buffer[offset + 1];
                msg->rtcCalib = message->buffer[offset + 2] & 0x01;
                msg->safeBoot = (message->buffer[offset + 2] >> 1) & 0x01;
                msg->xtalAbsent = (message->buffer[offset + 2] >> 2) & 0x01;
                for (uint8_t t = 0; t < 10; t++)
                    msg->hwVersion[t] = message->buffer[offset + 3 + t];

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] nPins: "); Serial.println(msg->nPins);
                Serial.print("[DEBUG] rtcCalib: "); Serial.println(msg->rtcCalib);
                Serial.print("[DEBUG] safeBoot: "); Serial.println(msg->safeBoot);
                Serial.print("[DEBUG] xtalAbsent: "); Serial.println(msg->xtalAbsent);
                Serial.print("[DEBUG] hwVersion: ");
                for (uint8_t t = 0; t < 10; t++)
                    Serial.print(msg->hwVersion[t]);
                Serial.println();
                #endif

                for (uint8_t i = 0; i < msg->nPins; i++)
                {
                    msg->pins[i].pinId = message->buffer[offset + 23 + 6 * i];
                    msg->pins[i].periphPIO = message->buffer[offset + 24 + 6 * i] & 0x01;
                    msg->pins[i].pinBank = (message->buffer[offset + 24 + 6 * i] >> 1) & 0x07;
                    msg->pins[i].direction = (message->buffer[offset + 24 + 6 * i] >> 4) & 0x01;
                    msg->pins[i].value = (message->buffer[offset + 24 + 6 * i] >> 5) & 0x01;
                    msg->pins[i].vpManager = (message->buffer[offset + 24 + 6 * i] >> 6) & 0x01;
                    msg->pins[i].pioIrq = (message->buffer[offset + 24 + 6 * i] >> 7) & 0x01;
                    msg->pins[i].pioPullHigh = message->buffer[offset + 25 + 6 * i] & 0x01;
                    msg->pins[i].pioPullLow = (message->buffer[offset + 25 + 6 * i] >> 1) & 0x01;
                    msg->pins[i].vp = message->buffer[offset + 26 + 6 * i];

                    #ifdef DEBUG
                    Serial.print("[DEBUG] pinId: "); Serial.println(msg->pins[i].pinId);
                    Serial.print("[DEBUG] periphPIO: "); Serial.println(msg->pins[i].periphPIO);
                    Serial.print("[DEBUG] pinBank: "); Serial.println(msg->pins[i].pinBank);
                    Serial.print("[DEBUG] direction: "); Serial.println(msg->pins[i].direction);
                    Serial.print("[DEBUG] value: "); Serial.println(msg->pins[i].value);
                    Serial.print("[DEBUG] vpManager: "); Serial.println(msg->pins[i].vpManager);
                    Serial.print("[DEBUG] pioIrq: "); Serial.println(msg->pins[i].pioIrq);
                    Serial.print("[DEBUG] pioPullHigh: "); Serial.println(msg->pins[i].pioPullHigh);
                    Serial.print("[DEBUG] pioPullLow: "); Serial.println(msg->pins[i].pioPullLow);
                    Serial.print("[DEBUG] vp: "); Serial.println(msg->pins[i].vp);
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_PATCH: // UBX-MON-PATCH
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-PATCH");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonPatch == NULL)
                {
                    gnss_ubx_mon_patch_t *ptr = (gnss_ubx_mon_patch_t *)malloc(sizeof(gnss_ubx_mon_patch_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-PATCH");
                        #endif
                        break;
                    }

                    handle->ubxMonPatch = ptr;
                }

                gnss_ubx_mon_patch_t *msg = handle->ubxMonPatch;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = getUByte16_LEnd(message->buffer, offset + 0);
                msg->nEntries = getUByte16_LEnd(message->buffer, offset + 2);

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] nEntries: "); Serial.println(msg->nEntries);
                #endif

                for (uint16_t i = 0; i < msg->nEntries; i++)
                {
                    msg->entries[i].activated = message->buffer[offset + 4 + 16 * i];
                    msg->entries[i].location = (message->buffer[offset + 4 + 16 * i] >> 1) & 0x03;
                    msg->entries[i].comparatorNumber = message->buffer[offset + 8 + 16 * i];
                    msg->entries[i].patchAddress = message->buffer[offset + 12 + 16 * i];
                    msg->entries[i].patchData = message->buffer[offset + 16 + 16 * i];

                    #ifdef DEBUG
                    Serial.print("[DEBUG] activated: "); Serial.println(msg->entries[i].activated);
                    Serial.print("[DEBUG] location: "); Serial.println(msg->entries[i].location);
                    Serial.print("[DEBUG] comparatorNumber: "); Serial.println(msg->entries[i].comparatorNumber);
                    Serial.print("[DEBUG] patchAddress: "); Serial.println(msg->entries[i].patchAddress);
                    Serial.print("[DEBUG] patchData: "); Serial.println(msg->entries[i].patchData);
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_RCVRSTAT: // UBX-MON-RCVRSTAT
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-RCVRSTAT");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonRcvrstat == NULL)
                {
                    gnss_ubx_mon_rcvrstat_t *ptr = (gnss_ubx_mon_rcvrstat_t *)malloc(sizeof(gnss_ubx_mon_rcvrstat_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-RCVRSTAT");
                        #endif
                        break;
                    }

                    handle->ubxMonRcvrstat = ptr;
                }

                gnss_ubx_mon_rcvrstat_t *msg = handle->ubxMonRcvrstat;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->version = message->buffer[offset + 0];

                msg->sigSbasEnVal = message->buffer[offset + 1] & 0x01;
                msg->sigSbasEnSrc = (message->buffer[offset + 1] >> 1) & 0x07;
                msg->sigSbasL1caEnVal = (message->buffer[offset + 1] >> 4) & 0x01;
                msg->sigSbasL1caEnSrc = (message->buffer[offset + 1] >> 5) & 0x07;

                msg->sigNavicEnVal = message->buffer[offset + 2] & 0x01;
                msg->sigNavicEnSrc = (message->buffer[offset + 2] >> 1) & 0x07;
                msg->sigNavicL5EnVal = (message->buffer[offset + 2] >> 4) & 0x01;
                msg->sigNavicL5EnSrc = (message->buffer[offset + 2] >> 5) & 0x07;

                msg->sigGpsEnVal = message->buffer[offset + 4] & 0x01;
                msg->sigGpsEnSrc = (message->buffer[offset + 4] >> 1) & 0x07;
                msg->sigGpsL1caEnVal = (message->buffer[offset + 4] >> 4) & 0x01;
                msg->sigGpsL1caEnSrc = (message->buffer[offset + 4] >> 5) & 0x07;

                msg->sigGpsL1cEnVal = message->buffer[offset + 5] & 0x01;
                msg->sigGpsL1cEnSrc = (message->buffer[offset + 5] >> 1) & 0x07;
                msg->sigGpsL2cEnVal = (message->buffer[offset + 5] >> 4) & 0x01;
                msg->sigGpsL2cEnSrc = (message->buffer[offset + 5] >> 5) & 0x07;

                msg->sigGpsL5EnVal = message->buffer[offset + 6] & 0x01;
                msg->sigGpsL5EnSrc = (message->buffer[offset + 6] >> 1) & 0x07;

                msg->sigGalEnVal = message->buffer[offset + 8] & 0x01;
                msg->sigGalEnSrc = (message->buffer[offset + 8] >> 1) & 0x07;
                msg->sigGalE1EnVal = (message->buffer[offset + 8] >> 4) & 0x01;
                msg->sigGalE1EnSrc = (message->buffer[offset + 8] >> 5) & 0x07;

                msg->sigGalE5aEnVal = message->buffer[offset + 9] & 0x01;
                msg->sigGalE5aEnSrc = (message->buffer[offset + 9] >> 1) & 0x07;
                msg->sigGalE5bEnVal = (message->buffer[offset + 9] >> 4) & 0x01;
                msg->sigGalE5bEnSrc = (message->buffer[offset + 9] >> 5) & 0x07;

                msg->sigGalE6EnVal = message->buffer[offset + 10] & 0x01;
                msg->sigGalE6EnSrc = (message->buffer[offset + 10] >> 1) & 0x07;

                msg->sigQzssEnVal = message->buffer[offset + 12] & 0x01;
                msg->sigQzssEnSrc = (message->buffer[offset + 12] >> 1) & 0x07;
                msg->sigQzssL1caEnVal = (message->buffer[offset + 12] >> 4) & 0x01;
                msg->sigQzssL1caEnSrc = (message->buffer[offset + 12] >> 5) & 0x07;

                msg->sigQzssL1cEnVal = message->buffer[offset + 13] & 0x01;
                msg->sigQzssL1cEnSrc = (message->buffer[offset + 13] >> 1) & 0x07;
                msg->sigQzssL1sEnVal = (message->buffer[offset + 13] >> 4) & 0x01;
                msg->sigQzssL1sEnSrc = (message->buffer[offset + 13] >> 5) & 0x07;

                msg->sigQzssL2cEnVal = message->buffer[offset + 14] & 0x01;
                msg->sigQzssL2cEnSrc = (message->buffer[offset + 14] >> 1) & 0x07;
                msg->sigQzssL5EnVal = (message->buffer[offset + 14] >> 4) & 0x01;
                msg->sigQzssL5EnSrc = (message->buffer[offset + 14] >> 5) & 0x07;

                msg->sigBdsEnVal = message->buffer[offset + 16] & 0x01;
                msg->sigBdsEnSrc = (message->buffer[offset + 16] >> 1) & 0x07;
                msg->sigBdsB1iEnVal = (message->buffer[offset + 16] >> 4) & 0x01;
                msg->sigBdsB1iEnSrc = (message->buffer[offset + 16] >> 5) & 0x07;

                msg->sigBdsB1cEnVal = message->buffer[offset + 17] & 0x01;
                msg->sigBdsB1cEnSrc = (message->buffer[offset + 17] >> 1) & 0x07;
                msg->sigBdsB2EnVal = (message->buffer[offset + 17] >> 4) & 0x01;
                msg->sigBdsB2EnSrc = (message->buffer[offset + 17] >> 5) & 0x07;

                msg->sigBdsB2aEnVal = message->buffer[offset + 18] & 0x01;
                msg->sigBdsB2aEnSrc = (message->buffer[offset + 18] >> 1) & 0x07;

                msg->sigGloEnVal = message->buffer[offset + 20] & 0x01;
                msg->sigGloEnSrc = (message->buffer[offset + 20] >> 1) & 0x07;
                msg->sigGloL1EnVal = (message->buffer[offset + 20] >> 4) & 0x01;
                msg->sigGloL1EnSrc = (message->buffer[offset + 20] >> 5) & 0x07;

                msg->sigGloL2EnVal = message->buffer[offset + 21] & 0x01;
                msg->sigGloL2EnSrc = (message->buffer[offset + 21] >> 1) & 0x07;
                msg->sigGloL3EnVal = (message->buffer[offset + 21] >> 4) & 0x01;
                msg->sigGloL3EnSrc = (message->buffer[offset + 21] >> 5) & 0x07;

                msg->lnaLnaModeRegVal = message->buffer[offset + 22] & 0x07;
                msg->lnaLnaModeCfgVal = (message->buffer[offset + 22] >> 4) & 0x0F;
                msg->lnaLnaModeSrc = (message->buffer[offset + 23] >> 5) & 0x07;

                msg->uartEnableVal = message->buffer[offset + 36] & 0x01;
                msg->uartEnableSrc = (message->buffer[offset + 36] >> 1) & 0x07;
                msg->uartRemapedVal = (message->buffer[offset + 36] >> 4) & 0x01;
                msg->uartRemapedSrc = (message->buffer[offset + 36] >> 5) & 0x07;

                msg->uartDataBitsVal = message->buffer[offset + 37] & 0x01;
                msg->uartDataBitsSrc = (message->buffer[offset + 37] >> 1) & 0x07;
                msg->uartStopBitsVal = (message->buffer[offset + 37] >> 4) & 0x03;
                msg->uartStopBitsSrc = ((message->buffer[offset + 37] >> 6) & 0x07) | (message->buffer[offset + 38] & 0x01) << 2;

                msg->uartParityBitsVal = (message->buffer[offset + 38] >> 1) & 0x03;
                msg->uartParityBitsSrc = (message->buffer[offset + 38] >> 3) & 0x07;

                msg->uartBaudrateVal = getUByte32_LEnd(message->buffer, offset + 40) & 0x000FFFFF;
                msg->uartBaudrateSrc = (message->buffer[offset + 42] >> 4) & 0x07;

                msg->spiEnableVal = message->buffer[offset + 44] & 0x01;
                msg->spiEnableSrc = (message->buffer[offset + 44] >> 1) & 0x07;
                msg->spiExtendedTimeoutVal = (message->buffer[offset + 44] >> 4) & 0x01;
                msg->spiExtendedTimeoutSrc = (message->buffer[offset + 44] >> 5) & 0x07;

                msg->spiCPolarityVal = message->buffer[offset + 45] & 0x01;
                msg->spiCPolaritySrc = (message->buffer[offset + 45] >> 1) & 0x07;
                msg->spiCPhaseVal = (message->buffer[offset + 45] >> 4) & 0x01;
                msg->spiCPhaseSrc = (message->buffer[offset + 45] >> 5) & 0x07;

                msg->spiMaxFfVal = message->buffer[offset + 46];

                msg->spiMaxFfSrc = message->buffer[offset + 47] & 0x07;

                msg->i2cEnableVal = message->buffer[offset + 48] & 0x01;
                msg->i2cEnableSrc = (message->buffer[offset + 48] >> 1) & 0x07;
                msg->i2cExtendedTimeoutVal = (message->buffer[offset + 48] >> 4) & 0x01;
                msg->i2cExtendedTimeoutSrc = (message->buffer[offset + 48] >> 5) & 0x07;

                msg->i2cRemapVal = message->buffer[offset + 49] & 0x01;
                msg->i2cRemapSrc = (message->buffer[offset + 49] >> 1) & 0x07;
                msg->i2cAddressVal = ((message->buffer[offset + 49] >> 4) & 0x0F) | ((message->buffer[offset + 50] & 0x0F) << 4);

                msg->i2cAddressSrc = (message->buffer[offset + 50] >> 4) & 0x07;

                msg->psmOperateModeVal = message->buffer[offset + 53] & 0x03;
                msg->psmOperateModeSrc = (message->buffer[offset + 53] >> 2) & 0x03;
                msg->psmOperateModeState = (message->buffer[offset + 53] >> 5) & 0x03;

                msg->antSupSmStatusVal = message->buffer[offset + 54] & 0x03;
                msg->antSupAPowerVal = (message->buffer[offset + 54] >> 2) & 0x03;

                msg->antSupSwitchPinVal = message->buffer[offset + 55] & 0x1F;
                msg->antSupSwitchPinSrc = (message->buffer[offset + 55] >> 5) & 0x07;

                msg->antSupShortPinVal = message->buffer[offset + 56] & 0x1F;
                msg->antSupShortPinSrc = (message->buffer[offset + 56] >> 5) & 0x07;

                msg->antSupOpenPinVal = message->buffer[offset + 57] & 0x1F;
                msg->antSupOpenPinSrc = (message->buffer[offset + 57] >> 5) & 0x07;

                msg->antSupRecIntPinVal = message->buffer[offset + 62];
                msg->antSupRecIntPinSrc = (message->buffer[offset + 63] >> 5) & 0x07;

                msg->antSupVoltctrlPinVal = message->buffer[offset + 64] & 0x01;
                msg->antSupVoltctrlPinSrc = (message->buffer[offset + 64] >> 1) & 0x07;
                msg->antSupShortDetVal = (message->buffer[offset + 64] >> 4) & 0x01;
                msg->antSupShortDetSrc = (message->buffer[offset + 64] >> 5) & 0x07;

                msg->antSupShortDetPolVal = message->buffer[offset + 65] & 0x01;
                msg->antSupShortDetPolSrc = (message->buffer[offset + 65] >> 1) & 0x07;
                msg->antSupOpenDetVal = (message->buffer[offset + 65] >> 4) & 0x01;
                msg->antSupOpenDetSrc = (message->buffer[offset + 65] >> 5) & 0x07;

                msg->antSupOpenDetPolVal = message->buffer[offset + 66] & 0x01;
                msg->antSupOpenDetPolSrc = (message->buffer[offset + 66] >> 1) & 0x07;
                msg->antSupPwrDownVal = (message->buffer[offset + 66] >> 4) & 0x01;
                msg->antSupPwrDownSrc = (message->buffer[offset + 66] >> 5) & 0x07;

                msg->antSupPwrDownPolVal = message->buffer[offset + 67] & 0x01;
                msg->antSupPwrDownPolSrc = (message->buffer[offset + 67] >> 1) & 0x07;
                msg->antSupRecoverVal = (message->buffer[offset + 67] >> 4) & 0x01;
                msg->antSupRecoverSrc = (message->buffer[offset + 67] >> 5) & 0x07;

                msg->antSupShortUsVal = getUByte16_LEnd(message->buffer, offset + 68);
                msg->antSupShortUsSrc = (message->buffer[offset + 71] >> 5) & 0x07;

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] sigSbasEnVal: "); Serial.println(msg->sigSbasEnVal);
                Serial.print("[DEBUG] sigSbasEnSrc: "); Serial.println(msg->sigSbasEnSrc);
                Serial.print("[DEBUG] sigSbasL1caEnVal: "); Serial.println(msg->sigSbasL1caEnVal);
                Serial.print("[DEBUG] sigSbasL1caEnSrc: "); Serial.println(msg->sigSbasL1caEnSrc);
                Serial.print("[DEBUG] sigNavicEnVal: "); Serial.println(msg->sigNavicEnVal);
                Serial.print("[DEBUG] sigNavicEnSrc: "); Serial.println(msg->sigNavicEnSrc);
                Serial.print("[DEBUG] sigNavicL5EnVal: "); Serial.println(msg->sigNavicL5EnVal);
                Serial.print("[DEBUG] sigNavicL5EnSrc: "); Serial.println(msg->sigNavicL5EnSrc);
                Serial.print("[DEBUG] sigGpsEnVal: "); Serial.println(msg->sigGpsEnVal);
                Serial.print("[DEBUG] sigGpsEnSrc: "); Serial.println(msg->sigGpsEnSrc);
                Serial.print("[DEBUG] sigGpsL1caEnVal: "); Serial.println(msg->sigGpsL1caEnVal);
                Serial.print("[DEBUG] sigGpsL1caEnSrc: "); Serial.println(msg->sigGpsL1caEnSrc);
                Serial.print("[DEBUG] sigGpsL1cEnVal: "); Serial.println(msg->sigGpsL1cEnVal);
                Serial.print("[DEBUG] sigGpsL1cEnSrc: "); Serial.println(msg->sigGpsL1cEnSrc);
                Serial.print("[DEBUG] sigGpsL2cEnVal: "); Serial.println(msg->sigGpsL2cEnVal);
                Serial.print("[DEBUG] sigGpsL2cEnSrc: "); Serial.println(msg->sigGpsL2cEnSrc);
                Serial.print("[DEBUG] sigGpsL5EnVal: "); Serial.println(msg->sigGpsL5EnVal);
                Serial.print("[DEBUG] sigGpsL5EnSrc: "); Serial.println(msg->sigGpsL5EnSrc);
                Serial.print("[DEBUG] sigGalEnVal: "); Serial.println(msg->sigGalEnVal);
                Serial.print("[DEBUG] sigGalEnSrc: "); Serial.println(msg->sigGalEnSrc);
                Serial.print("[DEBUG] sigGalE1EnVal: "); Serial.println(msg->sigGalE1EnVal);
                Serial.print("[DEBUG] sigGalE1EnSrc: "); Serial.println(msg->sigGalE1EnSrc);
                Serial.print("[DEBUG] sigGalE5aEnVal: "); Serial.println(msg->sigGalE5aEnVal);
                Serial.print("[DEBUG] sigGalE5aEnSrc: "); Serial.println(msg->sigGalE5aEnSrc);
                Serial.print("[DEBUG] sigGalE5bEnVal: "); Serial.println(msg->sigGalE5bEnVal);
                Serial.print("[DEBUG] sigGalE5bEnSrc: "); Serial.println(msg->sigGalE5bEnSrc);
                Serial.print("[DEBUG] sigGalE6EnVal: "); Serial.println(msg->sigGalE6EnVal);
                Serial.print("[DEBUG] sigGalE6EnSrc: "); Serial.println(msg->sigGalE6EnSrc);
                Serial.print("[DEBUG] sigQzssEnVal: "); Serial.println(msg->sigQzssEnVal);
                Serial.print("[DEBUG] sigQzssEnSrc: "); Serial.println(msg->sigQzssEnSrc);
                Serial.print("[DEBUG] sigQzssL1caEnVal: "); Serial.println(msg->sigQzssL1caEnVal);
                Serial.print("[DEBUG] sigQzssL1caEnSrc: "); Serial.println(msg->sigQzssL1caEnSrc);
                Serial.print("[DEBUG] sigQzssL1cEnVal: "); Serial.println(msg->sigQzssL1cEnVal);
                Serial.print("[DEBUG] sigQzssL1cEnSrc: "); Serial.println(msg->sigQzssL1cEnSrc);
                Serial.print("[DEBUG] sigQzssL1sEnVal: "); Serial.println(msg->sigQzssL1sEnVal);
                Serial.print("[DEBUG] sigQzssL1sEnSrc: "); Serial.println(msg->sigQzssL1sEnSrc);
                Serial.print("[DEBUG] sigQzssL2cEnVal: "); Serial.println(msg->sigQzssL2cEnVal);
                Serial.print("[DEBUG] sigQzssL2cEnSrc: "); Serial.println(msg->sigQzssL2cEnSrc);
                Serial.print("[DEBUG] sigQzssL5EnVal: "); Serial.println(msg->sigQzssL5EnVal);
                Serial.print("[DEBUG] sigQzssL5EnSrc: "); Serial.println(msg->sigQzssL5EnSrc);
                Serial.print("[DEBUG] sigBdsEnVal: "); Serial.println(msg->sigBdsEnVal);
                Serial.print("[DEBUG] sigBdsEnSrc: "); Serial.println(msg->sigBdsEnSrc);
                Serial.print("[DEBUG] sigBdsB1iEnVal: "); Serial.println(msg->sigBdsB1iEnVal);
                Serial.print("[DEBUG] sigBdsB1iEnSrc: "); Serial.println(msg->sigBdsB1iEnSrc);
                Serial.print("[DEBUG] sigBdsB1cEnVal: "); Serial.println(msg->sigBdsB1cEnVal);
                Serial.print("[DEBUG] sigBdsB1cEnSrc: "); Serial.println(msg->sigBdsB1cEnSrc);
                Serial.print("[DEBUG] sigBdsB2EnVal: "); Serial.println(msg->sigBdsB2EnVal);
                Serial.print("[DEBUG] sigBdsB2EnSrc: "); Serial.println(msg->sigBdsB2EnSrc);
                Serial.print("[DEBUG] sigBdsB2aEnVal: "); Serial.println(msg->sigBdsB2aEnVal);
                Serial.print("[DEBUG] sigBdsB2aEnSrc: "); Serial.println(msg->sigBdsB2aEnSrc);
                Serial.print("[DEBUG] sigGloEnVal: "); Serial.println(msg->sigGloEnVal);
                Serial.print("[DEBUG] sigGloEnSrc: "); Serial.println(msg->sigGloEnSrc);
                Serial.print("[DEBUG] sigGloL1EnVal: "); Serial.println(msg->sigGloL1EnVal);
                Serial.print("[DEBUG] sigGloL1EnSrc: "); Serial.println(msg->sigGloL1EnSrc);
                Serial.print("[DEBUG] sigGloL2EnVal: "); Serial.println(msg->sigGloL2EnVal);
                Serial.print("[DEBUG] sigGloL2EnSrc: "); Serial.println(msg->sigGloL2EnSrc);
                Serial.print("[DEBUG] sigGloL3EnVal: "); Serial.println(msg->sigGloL3EnVal);
                Serial.print("[DEBUG] sigGloL3EnSrc: "); Serial.println(msg->sigGloL3EnSrc);
                Serial.print("[DEBUG] lnaLnaModeRegVal: "); Serial.println(msg->lnaLnaModeRegVal);
                Serial.print("[DEBUG] lnaLnaModeCfgVal: "); Serial.println(msg->lnaLnaModeCfgVal);
                Serial.print("[DEBUG] lnaLnaModeSrc: "); Serial.println(msg->lnaLnaModeSrc);
                Serial.print("[DEBUG] uartEnableVal: "); Serial.println(msg->uartEnableVal);
                Serial.print("[DEBUG] uartEnableSrc: "); Serial.println(msg->uartEnableSrc);
                Serial.print("[DEBUG] uartRemapedVal: "); Serial.println(msg->uartRemapedVal);
                Serial.print("[DEBUG] uartRemapedSrc: "); Serial.println(msg->uartRemapedSrc);
                Serial.print("[DEBUG] uartDataBitsVal: "); Serial.println(msg->uartDataBitsVal);
                Serial.print("[DEBUG] uartDataBitsSrc: "); Serial.println(msg->uartDataBitsSrc);
                Serial.print("[DEBUG] uartStopBitsVal: "); Serial.println(msg->uartStopBitsVal);
                Serial.print("[DEBUG] uartStopBitsSrc: "); Serial.println(msg->uartStopBitsSrc);
                Serial.print("[DEBUG] uartParityBitsVal: "); Serial.println(msg->uartParityBitsVal);
                Serial.print("[DEBUG] uartParityBitsSrc: "); Serial.println(msg->uartParityBitsSrc);
                Serial.print("[DEBUG] uartBaudrateVal: "); Serial.println(msg->uartBaudrateVal);
                Serial.print("[DEBUG] uartBaudrateSrc: "); Serial.println(msg->uartBaudrateSrc);
                Serial.print("[DEBUG] spiEnableVal: "); Serial.println(msg->spiEnableVal);
                Serial.print("[DEBUG] spiEnableSrc: "); Serial.println(msg->spiEnableSrc);
                Serial.print("[DEBUG] spiExtendedTimeoutVal: "); Serial.println(msg->spiExtendedTimeoutVal);
                Serial.print("[DEBUG] spiExtendedTimeoutSrc: "); Serial.println(msg->spiExtendedTimeoutSrc);
                Serial.print("[DEBUG] spiCPolarityVal: "); Serial.println(msg->spiCPolarityVal);
                Serial.print("[DEBUG] spiCPolaritySrc: "); Serial.println(msg->spiCPolaritySrc);
                Serial.print("[DEBUG] spiCPhaseVal: "); Serial.println(msg->spiCPhaseVal);
                Serial.print("[DEBUG] spiCPhaseSrc: "); Serial.println(msg->spiCPhaseSrc);
                Serial.print("[DEBUG] spiMaxFfVal: "); Serial.println(msg->spiMaxFfVal);
                Serial.print("[DEBUG] spiMaxFfSrc: "); Serial.println(msg->spiMaxFfSrc);
                Serial.print("[DEBUG] i2cEnableVal: "); Serial.println(msg->i2cEnableVal);
                Serial.print("[DEBUG] i2cEnableSrc: "); Serial.println(msg->i2cEnableSrc);
                Serial.print("[DEBUG] i2cExtendedTimeoutVal: "); Serial.println(msg->i2cExtendedTimeoutVal);
                Serial.print("[DEBUG] i2cExtendedTimeoutSrc: "); Serial.println(msg->i2cExtendedTimeoutSrc);
                Serial.print("[DEBUG] i2cRemapVal: "); Serial.println(msg->i2cRemapVal);
                Serial.print("[DEBUG] i2cRemapSrc: "); Serial.println(msg->i2cRemapSrc);
                Serial.print("[DEBUG] i2cAddressVal: "); Serial.println(msg->i2cAddressVal);
                Serial.print("[DEBUG] i2cAddressSrc: "); Serial.println(msg->i2cAddressSrc);
                Serial.print("[DEBUG] psmOperateModeVal: "); Serial.println(msg->psmOperateModeVal);
                Serial.print("[DEBUG] psmOperateModeSrc: "); Serial.println(msg->psmOperateModeSrc);
                Serial.print("[DEBUG] psmOperateModeState: "); Serial.println(msg->psmOperateModeState);
                Serial.print("[DEBUG] antSupSmStatusVal: "); Serial.println(msg->antSupSmStatusVal);
                Serial.print("[DEBUG] antSupAPowerVal: "); Serial.println(msg->antSupAPowerVal);
                Serial.print("[DEBUG] antSupSwitchPinVal: "); Serial.println(msg->antSupSwitchPinVal);
                Serial.print("[DEBUG] antSupSwitchPinSrc: "); Serial.println(msg->antSupSwitchPinSrc);
                Serial.print("[DEBUG] antSupShortPinVal: "); Serial.println(msg->antSupShortPinVal);
                Serial.print("[DEBUG] antSupShortPinSrc: "); Serial.println(msg->antSupShortPinSrc);
                Serial.print("[DEBUG] antSupOpenPinVal: "); Serial.println(msg->antSupOpenPinVal);
                Serial.print("[DEBUG] antSupOpenPinSrc: "); Serial.println(msg->antSupOpenPinSrc);
                Serial.print("[DEBUG] antSupRecIntPinVal: "); Serial.println(msg->antSupRecIntPinVal);
                Serial.print("[DEBUG] antSupRecIntPinSrc: "); Serial.println(msg->antSupRecIntPinSrc);
                Serial.print("[DEBUG] antSupVoltctrlPinVal: "); Serial.println(msg->antSupVoltctrlPinVal);
                Serial.print("[DEBUG] antSupVoltctrlPinSrc: "); Serial.println(msg->antSupVoltctrlPinSrc);
                Serial.print("[DEBUG] antSupShortDetVal: "); Serial.println(msg->antSupShortDetVal);
                Serial.print("[DEBUG] antSupShortDetSrc: "); Serial.println(msg->antSupShortDetSrc);
                Serial.print("[DEBUG] antSupShortDetPolVal: "); Serial.println(msg->antSupShortDetPolVal);
                Serial.print("[DEBUG] antSupShortDetPolSrc: "); Serial.println(msg->antSupShortDetPolSrc);
                Serial.print("[DEBUG] antSupOpenDetVal: "); Serial.println(msg->antSupOpenDetVal);
                Serial.print("[DEBUG] antSupOpenDetSrc: "); Serial.println(msg->antSupOpenDetSrc);
                Serial.print("[DEBUG] antSupOpenDetPolVal: "); Serial.println(msg->antSupOpenDetPolVal);
                Serial.print("[DEBUG] antSupOpenDetPolSrc: "); Serial.println(msg->antSupOpenDetPolSrc);
                Serial.print("[DEBUG] antSupPwrDownVal: "); Serial.println(msg->antSupPwrDownVal);
                Serial.print("[DEBUG] antSupPwrDownSrc: "); Serial.println(msg->antSupPwrDownSrc);
                Serial.print("[DEBUG] antSupPwrDownPolVal: "); Serial.println(msg->antSupPwrDownPolVal);
                Serial.print("[DEBUG] antSupPwrDownPolSrc: "); Serial.println(msg->antSupPwrDownPolSrc);
                Serial.print("[DEBUG] antSupRecoverVal: "); Serial.println(msg->antSupRecoverVal);
                Serial.print("[DEBUG] antSupRecoverSrc: "); Serial.println(msg->antSupRecoverSrc);
                Serial.print("[DEBUG] antSupShortUsVal: "); Serial.println(msg->antSupShortUsVal);
                Serial.print("[DEBUG] antSupShortUsSrc: "); Serial.println(msg->antSupShortUsSrc);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_RF: // UBX-MON-RF
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-RF");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonRf == NULL)
                {
                    gnss_ubx_mon_rf_t *ptr = (gnss_ubx_mon_rf_t *)malloc(sizeof(gnss_ubx_mon_rf_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-RF");
                        #endif
                        break;
                    }

                    handle->ubxMonRf = ptr;
                }

                gnss_ubx_mon_rf_t *msg = handle->ubxMonRf;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->version = message->buffer[offset + 0];
                msg->nBlocks = message->buffer[offset + 1];

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] nBlocks: "); Serial.println(msg->nBlocks);
                #endif

                for (uint8_t i = 0; i < msg->nBlocks; i++)
                {
                    msg->blocks[i].blockId = message->buffer[offset + 4 + 24 * i];
                    msg->blocks[i].jammingState = message->buffer[offset + 5 + 24 * i] & 0x03;
                    msg->blocks[i].antStatus = message->buffer[offset + 6 + 24 * i];
                    msg->blocks[i].antPower = message->buffer[offset + 7 + 24 * i];
                    msg->blocks[i].postStatus = getUByte32_LEnd(message->buffer, offset + 8 + 24 * i);
                    msg->blocks[i].noisePerMS = getUByte16_LEnd(message->buffer, offset + 16 + 24 * i);
                    msg->blocks[i].agcCnt = getUByte16_LEnd(message->buffer, offset + 18 + 24 * i);
                    msg->blocks[i].cwSuppression = message->buffer[offset + 20 + 24 * i];
                    msg->blocks[i].ofsI = message->buffer[offset + 21 + 24 * i];
                    msg->blocks[i].magI = message->buffer[offset + 22 + 24 * i];
                    msg->blocks[i].ofsQ = message->buffer[offset + 23 + 24 * i];
                    msg->blocks[i].magQ = message->buffer[offset + 24 + 24 * i];

                    #ifdef DEBUG
                    Serial.print("[DEBUG] blockId: "); Serial.println(msg->blocks[i].blockId);
                    Serial.print("[DEBUG] jammingState: "); Serial.println(msg->blocks[i].jammingState);
                    Serial.print("[DEBUG] antStatus: "); Serial.println(msg->blocks[i].antStatus);
                    Serial.print("[DEBUG] antPower: "); Serial.println(msg->blocks[i].antPower);
                    Serial.print("[DEBUG] postStatus: "); Serial.println(msg->blocks[i].postStatus);
                    Serial.print("[DEBUG] noisePerMS: "); Serial.println(msg->blocks[i].noisePerMS);
                    Serial.print("[DEBUG] agcCnt: "); Serial.println(msg->blocks[i].agcCnt);
                    Serial.print("[DEBUG] cwSuppression: "); Serial.println(msg->blocks[i].cwSuppression);
                    Serial.print("[DEBUG] ofsI: "); Serial.println(msg->blocks[i].ofsI);
                    Serial.print("[DEBUG] magI: "); Serial.println(msg->blocks[i].magI);
                    Serial.print("[DEBUG] ofsQ: "); Serial.println(msg->blocks[i].ofsQ);
                    Serial.print("[DEBUG] magQ: "); Serial.println(msg->blocks[i].magQ);
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_RXR: // UBX-MON-RXR
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-RXR");
                #endif

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                handle->awake = message->buffer[offset + 0] & 0x01;

                #ifdef DEBUG
                Serial.print("[DEBUG] awake: "); Serial.println(handle->awake);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_SPAN: // UBX-MON-SPAN
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-SPAN");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonSpan == NULL)
                {
                    gnss_ubx_mon_span_t *ptr = (gnss_ubx_mon_span_t *)malloc(sizeof(gnss_ubx_mon_span_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-SPAN");
                        #endif
                        break;
                    }

                    handle->ubxMonSpan = ptr;
                }

                gnss_ubx_mon_span_t *msg = handle->ubxMonSpan;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                msg->stale = false;

                msg->version = message->buffer[offset + 0];
                msg->numRfBlocks = message->buffer[offset + 1];

                #ifdef DEBUG
                Serial.print("[DEBUG] version: "); Serial.println(msg->version);
                Serial.print("[DEBUG] numRfBlocks: "); Serial.println(msg->numRfBlocks);
                #endif

                for (uint8_t i = 0; i < msg->numRfBlocks; i++)
                {
                    for (uint16_t t = 0; t < 256; t++)
                    {
                        msg->rfBlocks[i].spectrum[t] = message->buffer[offset + 4 + t + 272 * i];
                        
                        #ifdef DEBUG
                        Serial.print("[DEBUG] Spectrum["); Serial.print(t); Serial.print("]: ");
                        Serial.println(msg->rfBlocks[i].spectrum[t]);
                        #endif
                    }
                    msg->rfBlocks[i].span = getUByte32_LEnd(message->buffer, offset + 260 + 272 * i);
                    msg->rfBlocks[i].res = getUByte32_LEnd(message->buffer, offset + 264 + 272 * i);
                    msg->rfBlocks[i].center = getUByte32_LEnd(message->buffer, offset + 268 + 272 * i);
                    msg->rfBlocks[i].pga = message->buffer[offset + 272 + 272 * i];

                    #ifdef DEBUG
                    Serial.print("[DEBUG] span: "); Serial.println(msg->rfBlocks[i].span);
                    Serial.print("[DEBUG] res: "); Serial.println(msg->rfBlocks[i].res);
                    Serial.print("[DEBUG] center: "); Serial.println(msg->rfBlocks[i].center);
                    Serial.print("[DEBUG] pga: "); Serial.println(msg->rfBlocks[i].pga);
                    #endif
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_MON_VER: // UBX-MON-VER
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-MON-VER");
                #endif

                // Allocate memory for the message if not already done
                if (handle->ubxMonVer == NULL)
                {
                    gnss_ubx_mon_ver_t *ptr = (gnss_ubx_mon_ver_t *)malloc(sizeof(gnss_ubx_mon_ver_t));
                    
                    if (ptr == NULL)
                    {
                        #ifdef DEBUG
                        Serial.println("[DEBUG] Memory allocation failed for UBX-MON-VER");
                        #endif
                        break;
                    }

                    handle->ubxMonVer = ptr;
                }

                gnss_ubx_mon_ver_t *msg = handle->ubxMonVer;

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                #ifdef DEBUG
                Serial.print("[DEBUG] swVersion: ");
                #endif

                for (uint8_t i = 0; i < 30; i++)
                {
                    msg->swVersion[i] = message->buffer[offset + 0 + i];
                    #ifdef DEBUG
                    Serial.print(msg->swVersion[i]);
                    #endif
                }
                
                #ifdef DEBUG
                Serial.println();
                Serial.print("[DEBUG] hwVersion: ");
                #endif

                for (uint8_t i = 0; i < 10; i++)
                {
                    msg->hwVersion[i] = message->buffer[offset + 30 + i];
                    #ifdef DEBUG
                    Serial.print(msg->hwVersion[i]);
                    #endif
                }

                #ifdef DEBUG
                Serial.println();
                #endif

                uint8_t extensions = (message->length - 48) / 30;
                    
                for (uint8_t t = 0; t < extensions; t++)
                {

                    #ifdef DEBUG
                    Serial.print("[DEBUG] extension: ");
                    #endif

                    for (uint8_t i = 0; i < 30; i++)
                    {
                        msg->ext[t].extension[i] = message->buffer[offset + 40 + 30 * t + i];
                        #ifdef DEBUG
                        Serial.print(msg->ext[t].extension[i]);
                        #endif
                    }

                    #ifdef DEBUG
                    Serial.println();
                    #endif
                }
                
                #ifdef DEBUG
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            case GNSS_UBX_UPD_SOS: // UBX-UPD-SOS
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] UBX-UPD-SOS");
                #endif

                offset = 6;

                // Check checksum first
                if (gnss_ubx_checksum(message) != ((message->buffer[message->length - 2] << 8) | message->buffer[message->length - 1]))
                {
                    #ifdef DEBUG
                    Serial.println("[DEBUG] Message failed to pass checksum");
                    #endif
                    break;
                }

                if (message->buffer[offset + 0] == 2)
                    handle->backupCreation = message->buffer[offset + 4];
                else if (message->buffer[offset + 0] == 3)
                {
                    handle->restoreResponse = message->buffer[offset + 4];
                    if (handle->restoreResponse != 3) // Only clear backup if this message wasn't polled
                    {
                        if (gnss_clear_backup(handle))
                        {
                            #ifdef DEBUG
                            Serial.println("[DEBUG] Failed to clear backup after receiving System Restored from Backup message");
                            #endif
                        }
                    }
                }
                
                #ifdef DEBUG
                Serial.print("[DEBUG] cmd: "); Serial.println(message->buffer[offset + 0]);
                Serial.print("[DEBUG] response: "); Serial.println(message->buffer[offset + 4]);
                Serial.print("[DEBUG] Checksum: "); Serial.print(message->buffer[message->length - 2]); Serial.print(" "); Serial.println(message->buffer[message->length - 1]);
                #endif
                break;
            }
            default:
                break;
        }

        // After parsing message, clear it and increment message trackers
        memset(message->buffer, 0x00, message->length);
        message->length = 0;

        if (handle->next_message == MSG_BUFFER_COUNT - 1)
            handle->next_message = 0;
        else
            handle->next_message += 1;

        #ifdef DEBUG
        Serial.print("[DEBUG] next_message: "); Serial.println(handle->next_message);
        #endif

    }

    handle->pending_messages = 0;

    #ifdef DEBUG
    Serial.print("[DEBUG] pending_messages: "); Serial.println(handle->pending_messages);
    #endif
    

    return 0;
}

/****************************************************************************
 * @brief Receive and Parse messages - gnss_rx() followed by gnss_parse_buffer() and then gnss_parse_messages() for convenience.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: Failed to receive messages
 * 2: Failed to parse buffer to messages
 * 3: Failed to parse messages
 ****************************************************************************/
uint8_t gnss_rec_and_parse(gnss_t *handle)
{

    if(gnss_rx(handle))
        return 1;

    if(gnss_parse_buffer(handle))
        return 2;

    if(gnss_parse_messages(handle))
        return 3;

    return 0;

}






/****************************************************************************
 * Generic Commands
 ****************************************************************************/





/****************************************************************************
 * @brief Send UBX-LOG-RETRIEVEBATCH to poll UBX-LOG-BATCH and (optionally) UBX-MON-BATCH.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: Failed to send message
 * 2: Failed to receive messages
 ****************************************************************************/
uint8_t gnss_retrieve_batch(gnss_t *handle, gnss_ubx_mon_batch_t *monMsg, gnss_ubx_log_batch_t ***batchMsg)
{

    uint8_t data[4];

    data[0] = 0x00; // Message Version
    data[1] = 0x01; // Always request UBX-MON-BATCH
    data[2] = 0x00; // Reserved
    data[3] = 0x00;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 1000000); // 1000ms

    // Check if memory has been allocated
    if (handle->ubxMonBatch == NULL)
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Batch Configuration hasn't been setup yet");
        #endif
        return 3;
    }

    // Initialize Values
    handle->ubxMonBatch->version = 1;
    handle->batchQueue = 0;

    for (uint16_t i = 0; i < 600; i++)
        handle->ubxLogBatch[i]->version = 1;

    // Send out poll request
    if (gnss_ubx_msg(handle, (GNSS_UBX_LOG_RETRIEVEBATCH >> 8) & 0xFF, GNSS_UBX_LOG_RETRIEVEBATCH & 0xFF, 4, data, 0))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-LOG-RETRIEVEBATCH");
        #endif
        return 1;
    }

    timer_start(&gen_timer);

    while (handle->ubxMonBatch->version != 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Timed out waiting for UBX-MON-BATCH");
            #endif
            return 4;
        }

        gnss_rec_and_parse(handle);

    }

    timer_reset(&gen_timer);

    // Read until all messages have been received
    while (handle->ubxLogBatch[handle->ubxMonBatch->fillLevel - 1]->version == 1)
    {
        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Timed out waiting for UBX-LOG-BATCH");
            #endif
            return 4;
        }

        gnss_rec_and_parse(handle);

    }

    *monMsg = *handle->ubxMonBatch;
    *batchMsg = handle->ubxLogBatch;

    return 0;

}

/****************************************************************************
 * @brief Send UBX-LOG-CREATE to create a log file.
 * @note Untested!
 * @param handle Handle for ublox gnss module.
 * @param circular Overwrite old entries once log becomes full.
 * @param logSize 0: Max Safe Size, 1: Minimum Size, 2: User Defined Size
 * @param userDefinedSize Max size in bytes of log. Applicable if User Defined Log Size is selected.
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_create_log(gnss_t *handle, bool circular, uint8_t logSize, uint32_t userDefinedSize)
{

    uint8_t data[16];

    data[0] = 0x00; // Message version
    data[1] = circular;
    data[2] = 0x00; // Reserved
    data[3] = logSize;
    data[4] = userDefinedSize & 0xFF; // Convert to little endian
    data[5] = (userDefinedSize >> 8) & 0xFF;
    data[6] = (userDefinedSize >> 16) & 0xFF;
    data[7] = (userDefinedSize >> 24) & 0xFF;

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_LOG_CREATE >> 8) & 0xFF, GNSS_UBX_LOG_CREATE & 0xFF, 16, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-LOG-CREATE");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Send UBX-LOG-ERASE to deactivate the log system and erase all log data.
 * @note Untested!
 * @param handle Handle for ublox gnss module.
 * @param circular Overwrite old entries once log becomes full.
 * @param logSize 0: Max Safe Size, 1: Minimum Size, 2: User Defined Size
 * @param userDefinedSize Max size in bytes of log. Applicable if User Defined Log Size is selected.
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_erase_log(gnss_t *handle, bool circular, uint8_t logSize, uint32_t userDefinedSize)
{

    uint8_t data = 0x00;

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_LOG_ERASE >> 8) & 0xFF, GNSS_UBX_LOG_ERASE & 0xFF, 0, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-LOG-ERASE");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Send UBX-LOG-FINDTIME to search for an entry at the desired time.
 * @note Untested!
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @param year Year of UTC Time
 * @param month Month of UTC Time
 * @param day Day of UTC Time
 * @param hour Hour of UTC Time
 * @param minute Minute of UTC Time
 * @param second Second of UTC Time
 * @return 0: Success
 * 1: Failed to initialize memory for message object
 * 2: Failed to send message
 ****************************************************************************/
uint8_t gnss_find_log_time(gnss_t *handle, gnss_ubx_log_findtime_t *message, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{

    if (handle->ubxLogFindtime == NULL)
    {
        handle->ubxLogFindtime = (gnss_ubx_log_findtime_t *)malloc(sizeof(gnss_ubx_log_findtime_t));
        
        if (handle->ubxLogFindtime == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-LOG-FINDTIME message");
            #endif
            return 1;
        }

    }

    uint8_t data[16];

    data[0] = 0x00; // Message version
    data[1] = 0x00; // Request
    data[2] = year & 0xFF; // Convert to little endian
    data[3] = (year >> 8) & 0xFF;
    data[4] = month;
    data[5] = day;
    data[6] = hour;
    data[7] = minute;
    data[8] = second;

    gnss_ubx_log_findtime_t *msg = handle->ubxLogFindtime;

    msg->version = 0x00; // Initialize value to check if it changed

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_LOG_FINDTIME >> 8) & 0xFF, GNSS_UBX_LOG_FINDTIME & 0xFF, 9, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-LOG-FINDTIME");
        #endif
        return 2;
    }

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

    timer_start(&gen_timer);

    while (msg->version == 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] FINDTIME request timed out");
            #endif
            return 2;
        }

        gnss_rec_and_parse(handle);

    }

    *message = *msg;

    return 0;

}

/****************************************************************************
 * @brief Send UBX-LOG-INFO to retrieve log information.
 * @note Untested!
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: Failed to initialize memory for message object
 * 2: Failed to send message
 ****************************************************************************/
uint8_t gnss_get_log_time(gnss_t *handle, gnss_ubx_log_info_t *message)
{

    if (handle->ubxLogInfo == NULL)
    {
        handle->ubxLogInfo = (gnss_ubx_log_info_t *)malloc(sizeof(gnss_ubx_log_info_t));
        
        if (handle->ubxLogInfo == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-LOG-INFO message");
            #endif
            return 1;
        }

    }

    uint8_t data = 0x00;

    gnss_ubx_log_info_t *msg = handle->ubxLogInfo;

    msg->version = 0x00; // Initialize value to check if it changed

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_LOG_INFO >> 8) & 0xFF, GNSS_UBX_LOG_INFO & 0xFF, 0, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-LOG-INFO");
        #endif
        return 2;
    }

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

    timer_start(&gen_timer);

    while (msg->version == 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] INFO request timed out");
            #endif
            return 2;
        }

        gnss_rec_and_parse(handle);

    }

    *message = *msg;

    return 0;

}

/****************************************************************************
 * @brief Send UBX-LOG-RETRIEVE to retrieve log data. Log recording will be disabled (per the datasheet) during retrieval
 * @note Untested!
 * @param handle Handle for ublox gnss module.
 * @param startNumber Index of first log entry to retrieve.
 * @param entryCount Number of log entries to retrieve in cluding the first. Maximum 256.
 * @return 0: Success
 * 1: Failed to disable log recording prior to retrieval
 * 2: Failed to send message
 * 3: Failed to re-enable log recording
 ****************************************************************************/
/*
uint8_t gnss_retrieve_log(gnss_t *handle, uint32_t startNumber, uint16_t entryCount)
{

    if (entryCount > 256)
        entryCount = 256;

    // Disable log recording
    uint8_t cfgData = 0x00;

    if (gnss_cfg_set(handle, GNSS_RAM, GNSS_CFG_LOGFILTER_RECORD_ENA, &cfgData, 1, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to disable log recording");
        #endif
        return 1;
    }

    uint8_t data[12];

    data[0] = startNumber & 0xFF;
    data[1] = (startNumber >> 8) & 0xFF;
    data[2] = (startNumber >> 16) & 0xFF;
    data[3] = (startNumber >> 24) & 0xFF;
    data[4] = entryCount & 0xFF;
    data[5] = (entryCount >> 8) & 0xFF;
    data[6] = 0x00;
    data[7] = 0x00;
    data[8] = 0x00; // Message Version
    data[9] = 0x00; // Reserved
    data[10] = 0x00; // Reserved
    data[11] = 0x00; // Reserved

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_LOG_RETRIEVE >> 8) & 0xFF, GNSS_UBX_LOG_RETRIEVE & 0xFF, 12, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-LOG-RETRIEVE");
        #endif
        return 2;
    }

    // TODO: Add Waiting for data check here

    // Re-enable log recording
    cfgData = 0x01;
    
    if (gnss_cfg_set(gnss_t *handle, GNSS_RAM, GNSS_CFG_LOGFILTER_RECORD_ENA, &cfgData, 1, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to re-enable log recording");
        #endif
        return 3;
    }
    
    return 0;

}
*/
/****************************************************************************
 * @brief Send UBX-NAV-RESETODO command
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_reset_odo(gnss_t *handle)
{

    uint8_t data = 0x00;

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_NAV_RESETODO >> 8) & 0xFF, GNSS_UBX_NAV_RESETODO & 0xFF, 0x0000, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-NAV-RESETODO");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Send UBX-RXM-PMREQ command. Used to request a power management related task.
 * @param handle Handle for ublox gnss module.
 * @param duration Duration of the requested task (ms). Maximum of 12 days, set to 0 to wait for a wakeup signal on a pin.
 * @param backup Set to put the receiver into backup mode.
 * @param force Set for minimum power consumption.
 * @param uartrx Wake up receiver if there's a falling or rising edge on the UART RX pin.
 * @param extint0 Wake up receiver if there's a falling or rising edge on the EXTINT0 pin.
 * @param extint1 Wake up receiver if there's a falling or rising edge on the EXTINT1 pin.
 * @param spics Wake up receiver if there's a falling or rising edge on the SPI CS pin.
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_pm_req(gnss_t *handle, uint32_t duration, bool backup, bool force, bool uartrx, bool extint0, bool extint1, bool spics)
{

    uint8_t data[16];

    data[0] = 0x00; // Message version
    data[1] = 0x00; // Reserved
    data[2] = 0x00; // Reserved
    data[3] = 0x00; // Reserved
    data[4] = duration & 0xFF; // Convert to little endian
    data[5] = (duration >> 8) & 0xFF;
    data[6] = (duration >> 16) & 0xFF;
    data[7] = (duration >> 24) & 0xFF;
    data[8] = 0x00 | (uint8_t)backup | ((uint8_t)force << 1);
    data[9] = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x00 | ((uint8_t)uartrx << 3) | ((uint8_t)extint0 << 5) | ((uint8_t)extint1 << 6) | ((uint8_t)spics << 7);
    data[13] = 0x00;
    data[14] = 0x00;
    data[15] = 0x00;

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_RXM_PMREQ >> 8) & 0xFF, GNSS_UBX_RXM_PMREQ & 0xFF, 16, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-RXM-PMREQ");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Send UBX-UPD-SOS command to create backup in flash.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: Failed to send message
 * 2: Timed out waiting for ACK
 ****************************************************************************/
uint8_t gnss_create_backup(gnss_t *handle)
{

    uint8_t data[4];

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 1000000); // 1000ms

    // Initialize value
    handle->backupCreation = 0;

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_UPD_SOS >> 8) & 0xFF, GNSS_UBX_UPD_SOS & 0xFF, 4, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-UPD-SOS");
        #endif
        return 1;
    }

    timer_start(&gen_timer);

    while (handle->backupCreation == 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Timed out waiting for UBX-UPD-SOS acknowledgement");
            #endif
            return 2;
        }

        gnss_rec_and_parse(handle);

    }

    return 0;

}

/****************************************************************************
 * @brief Send UBX-UPD-SOS command to clear backup from flash.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: Failed to send message
 ****************************************************************************/
uint8_t gnss_clear_backup(gnss_t *handle)
{

    uint8_t data[4];

    data[0] = 0x01;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    // Send command
    if (gnss_ubx_msg(handle, (GNSS_UBX_UPD_SOS >> 8) & 0xFF, GNSS_UBX_UPD_SOS & 0xFF, 4, data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-UPD-SOS");
        #endif
        return 1;
    }

    return 0;

}

/****************************************************************************
 * @brief Send UBX-UPD-SOS to poll system restored from backup status. Will erase backup consequently.
 * Backup status can be queried by handle.restoreResponse at any time, as this value is stored upon reception of the status messge.
 * @param handle Handle for ublox gnss module.
 * @return 0: Success
 * 1: Failed to send message
 * 2: Timed out waiting for System Restored from Backup message
 ****************************************************************************/
uint8_t gnss_get_backup_status(gnss_t *handle)
{

    uint8_t data = 0x00;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 1000000); // 1000ms

    handle->restoreResponse = 0;

    // Send out poll request
    if (gnss_ubx_msg(handle, (GNSS_UBX_UPD_SOS >> 8) & 0xFF, GNSS_UBX_UPD_SOS & 0xFF, 0, &data, 1))
    {
        #ifdef DEBUG
        Serial.println("[DEBUG] Failed to send UBX-UPD-SOS");
        #endif
        return 1;
    }

    timer_start(&gen_timer);

    while (handle->restoreResponse == 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Timed out waiting for UBX-UPD-SOS");
            #endif
            return 2;
        }

        gnss_rec_and_parse(handle);

    }

    return 0;

}





/****************************************************************************
 * Message Retrieval
 ****************************************************************************/





/****************************************************************************
 * @brief Retrieve next info (debug, warning, error, etc...) message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available
 ****************************************************************************/
uint8_t gnss_get_msg_info(gnss_t *handle, gnss_info_t *message)
{

    if (handle->info == NULL)
        return 1;

    *message = *(handle->info->data);

    // Free allocate message node
    gnss_info_node_t *current = handle->info;

    if (current->next == NULL) // Head
    {
        free(handle->info->data);
        free(handle->info);
        handle->info = NULL;
    }
    else
    {
        handle->info = handle->info->next;

        free(current->data);
        free(current);
    }

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-DTM message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_dtm(gnss_t *handle, gnss_nmea_std_dtm_t *message)
{

    // Allocate memory
    if (handle->nmeaStdDtm == NULL)
    {
        handle->nmeaStdDtm = (gnss_nmea_std_dtm_t *)malloc(sizeof(gnss_nmea_std_dtm_t));
        
        if (handle->nmeaStdDtm == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-DTM message");
            #endif
            return 2;
        }

        handle->nmeaStdDtm->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_dtm_t *msg = handle->nmeaStdDtm; // Simplify call

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_DTM);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] DTM poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GBS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_gbs(gnss_t *handle, gnss_nmea_std_gbs_t *message)
{

    if (handle->nmeaStdGbs == NULL)
    {
        handle->nmeaStdGbs = (gnss_nmea_std_gbs_t *)malloc(sizeof(gnss_nmea_std_gbs_t));
        
        if (handle->nmeaStdGbs == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GBS message");
            #endif
            return 2;
        }

        handle->nmeaStdGbs->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_gbs_t *msg = handle->nmeaStdGbs;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GBS);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GBS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GGA message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_gga(gnss_t *handle, gnss_nmea_std_gga_t *message)
{

    if (handle->nmeaStdGga == NULL)
    {
        handle->nmeaStdGga = (gnss_nmea_std_gga_t *)malloc(sizeof(gnss_nmea_std_gga_t));
        
        if (handle->nmeaStdGga == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GGA message");
            #endif
            return 2;
        }

        handle->nmeaStdGga->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_gga_t *msg = handle->nmeaStdGga;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GGA);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GGA poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GLL message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_gll(gnss_t *handle, gnss_nmea_std_gll_t *message)
{

    if (handle->nmeaStdGll == NULL)
    {
        handle->nmeaStdGll = (gnss_nmea_std_gll_t *)malloc(sizeof(gnss_nmea_std_gll_t));
        
        if (handle->nmeaStdGll == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GLL message");
            #endif
            return 2;
        }

        handle->nmeaStdGll->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_gll_t *msg = handle->nmeaStdGll;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GLL);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GLL poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GNS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_gns(gnss_t *handle, gnss_nmea_std_gns_t *message)
{

    if (handle->nmeaStdGns == NULL)
    {
        handle->nmeaStdGns = (gnss_nmea_std_gns_t *)malloc(sizeof(gnss_nmea_std_gns_t));
        
        if (handle->nmeaStdGns == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GNS message");
            #endif
            return 2;
        }

        handle->nmeaStdGns->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_gns_t *msg = handle->nmeaStdGns;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GNS);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GNS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GRS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_grs(gnss_t *handle, gnss_nmea_std_grs_t *message)
{

    if (handle->nmeaStdGrs == NULL)
    {
        handle->nmeaStdGrs = (gnss_nmea_std_grs_t *)malloc(sizeof(gnss_nmea_std_grs_t));
        
        if (handle->nmeaStdGrs == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GRS message");
            #endif
            return 2;
        }

        handle->nmeaStdGrs->periodic = false; // Periodic should be set using gnss_set_msg_auto

        // Must initialize array of pointers
        for (uint8_t i = 0; i < 8; i++)
            handle->nmeaStdGrs->grs[i] = NULL;
    }

    gnss_nmea_std_grs_t *msg = handle->nmeaStdGrs;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GRS);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GRS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GSA message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_gsa(gnss_t *handle, gnss_nmea_std_gsa_t *message)
{

    if (handle->nmeaStdGsa == NULL)
    {
        handle->nmeaStdGsa = (gnss_nmea_std_gsa_t *)malloc(sizeof(gnss_nmea_std_gsa_t));
        
        if (handle->nmeaStdGsa == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GSA message");
            #endif
            return 2;
        }

        handle->nmeaStdGsa->periodic = false; // Periodic should be set using gnss_set_msg_auto

        // Must initialize array of pointers
        for (uint8_t i = 0; i < 8; i++)
            handle->nmeaStdGsa->gsa[i] = NULL;
    }

    gnss_nmea_std_gsa_t *msg = handle->nmeaStdGsa;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GSA);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GSA poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GST message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_gst(gnss_t *handle, gnss_nmea_std_gst_t *message)
{

    if (handle->nmeaStdGst == NULL)
    {
        handle->nmeaStdGst = (gnss_nmea_std_gst_t *)malloc(sizeof(gnss_nmea_std_gst_t));
        
        if (handle->nmeaStdGst == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GST message");
            #endif
            return 2;
        }

        handle->nmeaStdGst->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_gst_t *msg = handle->nmeaStdGst;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GST);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GST poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-GSV message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_gsv(gnss_t *handle, gnss_nmea_std_gsv_t *message)
{

    if (handle->nmeaStdGsv == NULL)
    {
        handle->nmeaStdGsv = (gnss_nmea_std_gsv_t *)malloc(sizeof(gnss_nmea_std_gsv_t));
        
        if (handle->nmeaStdGsv == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-GSV message");
            #endif
            return 2;
        }

        handle->nmeaStdGsv->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_gsv_t *msg = handle->nmeaStdGsv;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_GSV);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] GSV poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-RMC message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_rmc(gnss_t *handle, gnss_nmea_std_rmc_t *message)
{

    if (handle->nmeaStdRmc == NULL)
    {
        handle->nmeaStdRmc = (gnss_nmea_std_rmc_t *)malloc(sizeof(gnss_nmea_std_rmc_t));
        
        if (handle->nmeaStdRmc == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-RMC message");
            #endif
            return 2;
        }

        handle->nmeaStdRmc->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_rmc_t *msg = handle->nmeaStdRmc;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_RMC);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] RMC poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-VLW message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_vlw(gnss_t *handle, gnss_nmea_std_vlw_t *message)
{

    if (handle->nmeaStdVlw == NULL)
    {
        handle->nmeaStdVlw = (gnss_nmea_std_vlw_t *)malloc(sizeof(gnss_nmea_std_vlw_t));
        
        if (handle->nmeaStdVlw == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-VLW message");
            #endif
            return 2;
        }

        handle->nmeaStdVlw->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_vlw_t *msg = handle->nmeaStdVlw;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_VLW);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] VLW poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-VTG message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_vtg(gnss_t *handle, gnss_nmea_std_vtg_t *message)
{

    if (handle->nmeaStdVtg == NULL)
    {
        handle->nmeaStdVtg = (gnss_nmea_std_vtg_t *)malloc(sizeof(gnss_nmea_std_vtg_t));
        
        if (handle->nmeaStdVtg == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-VTG message");
            #endif
            return 2;
        }

        handle->nmeaStdVtg->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_vtg_t *msg = handle->nmeaStdVtg;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_VTG);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] VTG poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-Standard-ZDA message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nmea_zda(gnss_t *handle, gnss_nmea_std_zda_t *message)
{

    if (handle->nmeaStdZda == NULL)
    {
        handle->nmeaStdZda = (gnss_nmea_std_zda_t *)malloc(sizeof(gnss_nmea_std_zda_t));
        
        if (handle->nmeaStdZda == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-Standard-ZDA message");
            #endif
            return 2;
        }

        handle->nmeaStdZda->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_std_zda_t *msg = handle->nmeaStdZda;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_nmea_msg(handle, handle->mainTalker, GNSS_NMEA_STANDARD_ZDA);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] ZDA poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-PUBX-POSITION message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_pubx_position(gnss_t *handle, gnss_nmea_pubx_pos_t *message)
{

    if (handle->nmeaPubxPos == NULL)
    {
        handle->nmeaPubxPos = (gnss_nmea_pubx_pos_t *)malloc(sizeof(gnss_nmea_pubx_pos_t));
        
        if (handle->nmeaPubxPos == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-POSITION message");
            #endif
            return 2;
        }

        handle->nmeaPubxPos->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_pubx_pos_t *msg = handle->nmeaPubxPos;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data[] = {'0', '0'};

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_pubx_msg(handle, data, 2);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] PUBX-POSITION poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-PUBX-SVSTATUS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_pubx_svstatus(gnss_t *handle, gnss_nmea_pubx_svstatus_t *message)
{

    if (handle->nmeaPubxSvstatus == NULL)
    {
        handle->nmeaPubxSvstatus = (gnss_nmea_pubx_svstatus_t *)malloc(sizeof(gnss_nmea_pubx_svstatus_t));
        
        if (handle->nmeaPubxSvstatus == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-SVSTATUS message");
            #endif
            return 2;
        }

        handle->nmeaPubxSvstatus->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_pubx_svstatus_t *msg = handle->nmeaPubxSvstatus;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data[] = {'0', '3'};

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_pubx_msg(handle, data, 2);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] PUBX-SVSTATUS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve NMEA-PUBX-TIME message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_pubx_time(gnss_t *handle, gnss_nmea_pubx_time_t *message)
{

    if (handle->nmeaPubxTime == NULL)
    {
        handle->nmeaPubxTime = (gnss_nmea_pubx_time_t *)malloc(sizeof(gnss_nmea_pubx_time_t));
        
        if (handle->nmeaPubxTime == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for NMEA-PUBX-TIME message");
            #endif
            return 2;
        }

        handle->nmeaPubxTime->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_nmea_pubx_time_t *msg = handle->nmeaPubxTime;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data[] = {'0', '4'};

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_pubx_msg(handle, data, 2);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] PUBX-TIME poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-AOPSTATUS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_aopstatus(gnss_t *handle, gnss_ubx_nav_aopstatus_t *message)
{

    if (handle->ubxNavAopstatus == NULL)
    {
        handle->ubxNavAopstatus = (gnss_ubx_nav_aopstatus_t *)malloc(sizeof(gnss_ubx_nav_aopstatus_t));
        
        if (handle->ubxNavAopstatus == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-AOPSTATUS message");
            #endif
            return 2;
        }

        handle->ubxNavAopstatus->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_aopstatus_t *msg = handle->ubxNavAopstatus;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_AOPSTATUS >> 8) & 0xFF, GNSS_UBX_NAV_AOPSTATUS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] AOPSTATUS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-CLOCK message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_clock(gnss_t *handle, gnss_ubx_nav_clock_t *message)
{

    if (handle->ubxNavClock == NULL)
    {
        handle->ubxNavClock = (gnss_ubx_nav_clock_t *)malloc(sizeof(gnss_ubx_nav_clock_t));
        
        if (handle->ubxNavClock == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-CLOCK message");
            #endif
            return 2;
        }

        handle->ubxNavClock->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_clock_t *msg = handle->ubxNavClock;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_CLOCK >> 8) & 0xFF, GNSS_UBX_NAV_CLOCK & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] CLOCK poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-COV message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_cov(gnss_t *handle, gnss_ubx_nav_cov_t *message)
{

    if (handle->ubxNavCov == NULL)
    {
        handle->ubxNavCov = (gnss_ubx_nav_cov_t *)malloc(sizeof(gnss_ubx_nav_cov_t));
        
        if (handle->ubxNavCov == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-COV message");
            #endif
            return 2;
        }

        handle->ubxNavCov->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_cov_t *msg = handle->ubxNavCov;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_COV >> 8) & 0xFF, GNSS_UBX_NAV_COV & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] COV poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-DOP message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_dop(gnss_t *handle, gnss_ubx_nav_dop_t *message)
{

    if (handle->ubxNavDop == NULL)
    {
        handle->ubxNavDop = (gnss_ubx_nav_dop_t *)malloc(sizeof(gnss_ubx_nav_dop_t));
        
        if (handle->ubxNavDop == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-DOP message");
            #endif
            return 2;
        }

        handle->ubxNavDop->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_dop_t *msg = handle->ubxNavDop;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_DOP >> 8) & 0xFF, GNSS_UBX_NAV_DOP & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] DOP poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-EOE message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_eoe(gnss_t *handle, gnss_ubx_nav_eoe_t *message)
{

    if (handle->ubxNavEoe == NULL)
    {
        handle->ubxNavEoe = (gnss_ubx_nav_eoe_t *)malloc(sizeof(gnss_ubx_nav_eoe_t));
        
        if (handle->ubxNavEoe == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-EOE message");
            #endif
            return 2;
        }

        handle->ubxNavEoe->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_eoe_t *msg = handle->ubxNavEoe;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_EOE >> 8) & 0xFF, GNSS_UBX_NAV_EOE & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] EOE poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-ODO message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_odo(gnss_t *handle, gnss_ubx_nav_odo_t *message)
{

    if (handle->ubxNavOdo == NULL)
    {
        handle->ubxNavOdo = (gnss_ubx_nav_odo_t *)malloc(sizeof(gnss_ubx_nav_odo_t));
        
        if (handle->ubxNavOdo == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-ODO message");
            #endif
            return 2;
        }

        handle->ubxNavOdo->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_odo_t *msg = handle->ubxNavOdo;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_ODO >> 8) & 0xFF, GNSS_UBX_NAV_ODO & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] ODO poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-ORB message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_orb(gnss_t *handle, gnss_ubx_nav_orb_t *message)
{

    if (handle->ubxNavOrb == NULL)
    {
        handle->ubxNavOrb = (gnss_ubx_nav_orb_t *)malloc(sizeof(gnss_ubx_nav_orb_t));
        
        if (handle->ubxNavOrb == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-ORB message");
            #endif
            return 2;
        }

        handle->ubxNavOrb->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_orb_t *msg = handle->ubxNavOrb;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_ORB >> 8) & 0xFF, GNSS_UBX_NAV_ORB & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] ORB poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-PL message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_pl(gnss_t *handle, gnss_ubx_nav_pl_t *message)
{

    if (handle->ubxNavPl == NULL)
    {
        handle->ubxNavPl = (gnss_ubx_nav_pl_t *)malloc(sizeof(gnss_ubx_nav_pl_t));
        
        if (handle->ubxNavPl == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-PL message");
            #endif
            return 2;
        }

        handle->ubxNavPl->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_pl_t *msg = handle->ubxNavPl;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_PL >> 8) & 0xFF, GNSS_UBX_NAV_PL & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] PL poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-POSECEF message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_posecef(gnss_t *handle, gnss_ubx_nav_posecef_t *message)
{

    if (handle->ubxNavPosecef == NULL)
    {
        handle->ubxNavPosecef = (gnss_ubx_nav_posecef_t *)malloc(sizeof(gnss_ubx_nav_posecef_t));
        
        if (handle->ubxNavPosecef == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-POSECEF message");
            #endif
            return 2;
        }

        handle->ubxNavPosecef->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_posecef_t *msg = handle->ubxNavPosecef;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_POSECEF >> 8) & 0xFF, GNSS_UBX_NAV_POSECEF & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] POSECEF poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-POSLLH message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_posllh(gnss_t *handle, gnss_ubx_nav_posllh_t *message)
{

    if (handle->ubxNavPosllh == NULL)
    {
        handle->ubxNavPosllh = (gnss_ubx_nav_posllh_t *)malloc(sizeof(gnss_ubx_nav_posllh_t));
        
        if (handle->ubxNavPosllh == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-POSLLH message");
            #endif
            return 2;
        }

        handle->ubxNavPosllh->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_posllh_t *msg = handle->ubxNavPosllh;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_POSLLH >> 8) & 0xFF, GNSS_UBX_NAV_POSLLH & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] POSLLH poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-PVT message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_pvt(gnss_t *handle, gnss_ubx_nav_pvt_t *message)
{

    if (handle->ubxNavPvt == NULL)
    {
        handle->ubxNavPvt = (gnss_ubx_nav_pvt_t *)malloc(sizeof(gnss_ubx_nav_pvt_t));
        
        if (handle->ubxNavPvt == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-PVT message");
            #endif
            return 2;
        }

        handle->ubxNavPvt->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_pvt_t *msg = handle->ubxNavPvt;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_PVT >> 8) & 0xFF, GNSS_UBX_NAV_PVT & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] PVT poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-SAT message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_sat(gnss_t *handle, gnss_ubx_nav_sat_t *message)
{

    if (handle->ubxNavSat == NULL)
    {
        handle->ubxNavSat = (gnss_ubx_nav_sat_t *)malloc(sizeof(gnss_ubx_nav_sat_t));
        
        if (handle->ubxNavSat == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SAT message");
            #endif
            return 2;
        }

        handle->ubxNavSat->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_sat_t *msg = handle->ubxNavSat;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_SAT >> 8) & 0xFF, GNSS_UBX_NAV_SAT & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SAT poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-SBAS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_sbas(gnss_t *handle, gnss_ubx_nav_sbas_t *message)
{

    if (handle->ubxNavSbas == NULL)
    {
        handle->ubxNavSbas = (gnss_ubx_nav_sbas_t *)malloc(sizeof(gnss_ubx_nav_sbas_t));
        
        if (handle->ubxNavSbas == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SBAS message");
            #endif
            return 2;
        }

        handle->ubxNavSbas->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_sbas_t *msg = handle->ubxNavSbas;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_SBAS >> 8) & 0xFF, GNSS_UBX_NAV_SBAS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SBAS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-SIG message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_sig(gnss_t *handle, gnss_ubx_nav_sig_t *message)
{

    if (handle->ubxNavSig == NULL)
    {
        handle->ubxNavSig = (gnss_ubx_nav_sig_t *)malloc(sizeof(gnss_ubx_nav_sig_t));
        
        if (handle->ubxNavSig == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SIG message");
            #endif
            return 2;
        }

        handle->ubxNavSig->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_sig_t *msg = handle->ubxNavSig;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_SIG >> 8) & 0xFF, GNSS_UBX_NAV_SIG & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SIG poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-SLAS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_slas(gnss_t *handle, gnss_ubx_nav_slas_t *message)
{

    if (handle->ubxNavSlas == NULL)
    {
        handle->ubxNavSlas = (gnss_ubx_nav_slas_t *)malloc(sizeof(gnss_ubx_nav_slas_t));
        
        if (handle->ubxNavSlas == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-SLAS message");
            #endif
            return 2;
        }

        handle->ubxNavSlas->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_slas_t *msg = handle->ubxNavSlas;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_SLAS >> 8) & 0xFF, GNSS_UBX_NAV_SLAS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SLAS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-STATUS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_status(gnss_t *handle, gnss_ubx_nav_status_t *message)
{

    if (handle->ubxNavStatus == NULL)
    {
        handle->ubxNavStatus = (gnss_ubx_nav_status_t *)malloc(sizeof(gnss_ubx_nav_status_t));
        
        if (handle->ubxNavStatus == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-STATUS message");
            #endif
            return 2;
        }

        handle->ubxNavStatus->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_status_t *msg = handle->ubxNavStatus;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_STATUS >> 8) & 0xFF, GNSS_UBX_NAV_STATUS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] STATUS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMEBDS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timebds(gnss_t *handle, gnss_ubx_nav_timebds_t *message)
{

    if (handle->ubxNavTimebds == NULL)
    {
        handle->ubxNavTimebds = (gnss_ubx_nav_timebds_t *)malloc(sizeof(gnss_ubx_nav_timebds_t));
        
        if (handle->ubxNavTimebds == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEBDS message");
            #endif
            return 2;
        }

        handle->ubxNavTimebds->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timebds_t *msg = handle->ubxNavTimebds;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMEBDS >> 8) & 0xFF, GNSS_UBX_NAV_TIMEBDS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMEBDS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMEGAL message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timegal(gnss_t *handle, gnss_ubx_nav_timegal_t *message)
{

    if (handle->ubxNavTimegal == NULL)
    {
        handle->ubxNavTimegal = (gnss_ubx_nav_timegal_t *)malloc(sizeof(gnss_ubx_nav_timegal_t));
        
        if (handle->ubxNavTimegal == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEGAL message");
            #endif
            return 2;
        }

        handle->ubxNavTimegal->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timegal_t *msg = handle->ubxNavTimegal;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMEGAL >> 8) & 0xFF, GNSS_UBX_NAV_TIMEGAL & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMEGAL poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMEGLO message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timeglo(gnss_t *handle, gnss_ubx_nav_timeglo_t *message)
{

    if (handle->ubxNavTimeglo == NULL)
    {
        handle->ubxNavTimeglo = (gnss_ubx_nav_timeglo_t *)malloc(sizeof(gnss_ubx_nav_timeglo_t));
        
        if (handle->ubxNavTimeglo == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEGLO message");
            #endif
            return 2;
        }

        handle->ubxNavTimeglo->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timeglo_t *msg = handle->ubxNavTimeglo;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMEGLO >> 8) & 0xFF, GNSS_UBX_NAV_TIMEGLO & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMEGLO poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMEGPS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timegps(gnss_t *handle, gnss_ubx_nav_timegps_t *message)
{

    if (handle->ubxNavTimegps == NULL)
    {
        handle->ubxNavTimegps = (gnss_ubx_nav_timegps_t *)malloc(sizeof(gnss_ubx_nav_timegps_t));
        
        if (handle->ubxNavTimegps == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEGPS message");
            #endif
            return 2;
        }

        handle->ubxNavTimegps->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timegps_t *msg = handle->ubxNavTimegps;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMEGPS >> 8) & 0xFF, GNSS_UBX_NAV_TIMEGPS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMEGPS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMELS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timels(gnss_t *handle, gnss_ubx_nav_timels_t *message)
{

    if (handle->ubxNavTimels == NULL)
    {
        handle->ubxNavTimels = (gnss_ubx_nav_timels_t *)malloc(sizeof(gnss_ubx_nav_timels_t));
        
        if (handle->ubxNavTimels == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMELS message");
            #endif
            return 2;
        }

        handle->ubxNavTimels->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timels_t *msg = handle->ubxNavTimels;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMELS >> 8) & 0xFF, GNSS_UBX_NAV_TIMELS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMELS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMENAVIC message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timenavic(gnss_t *handle, gnss_ubx_nav_timenavic_t *message)
{

    if (handle->ubxNavTimenavic == NULL)
    {
        handle->ubxNavTimenavic = (gnss_ubx_nav_timenavic_t *)malloc(sizeof(gnss_ubx_nav_timenavic_t));
        
        if (handle->ubxNavTimenavic == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMENAVIC message");
            #endif
            return 2;
        }

        handle->ubxNavTimenavic->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timenavic_t *msg = handle->ubxNavTimenavic;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMENAVIC >> 8) & 0xFF, GNSS_UBX_NAV_TIMENAVIC & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMENAVIC poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMEQZSS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timeqzss(gnss_t *handle, gnss_ubx_nav_timeqzss_t *message)
{

    if (handle->ubxNavTimeqzss == NULL)
    {
        handle->ubxNavTimeqzss = (gnss_ubx_nav_timeqzss_t *)malloc(sizeof(gnss_ubx_nav_timeqzss_t));
        
        if (handle->ubxNavTimeqzss == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEQZSS message");
            #endif
            return 2;
        }

        handle->ubxNavTimeqzss->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timeqzss_t *msg = handle->ubxNavTimeqzss;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMEQZSS >> 8) & 0xFF, GNSS_UBX_NAV_TIMEQZSS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMEQZSS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-TIMEUTC message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_timeutc(gnss_t *handle, gnss_ubx_nav_timeutc_t *message)
{

    if (handle->ubxNavTimeutc == NULL)
    {
        handle->ubxNavTimeutc = (gnss_ubx_nav_timeutc_t *)malloc(sizeof(gnss_ubx_nav_timeutc_t));
        
        if (handle->ubxNavTimeutc == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-TIMEUTC message");
            #endif
            return 2;
        }

        handle->ubxNavTimeutc->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_timeutc_t *msg = handle->ubxNavTimeutc;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_TIMEUTC >> 8) & 0xFF, GNSS_UBX_NAV_TIMEUTC & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TIMEUTC poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-VELECEF message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_velecef(gnss_t *handle, gnss_ubx_nav_velecef_t *message)
{

    if (handle->ubxNavVelecef == NULL)
    {
        handle->ubxNavVelecef = (gnss_ubx_nav_velecef_t *)malloc(sizeof(gnss_ubx_nav_velecef_t));
        
        if (handle->ubxNavVelecef == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-VELECEF message");
            #endif
            return 2;
        }

        handle->ubxNavVelecef->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_velecef_t *msg = handle->ubxNavVelecef;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_VELECEF >> 8) & 0xFF, GNSS_UBX_NAV_VELECEF & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] VELECEF poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-NAV-VELNED message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_nav_velned(gnss_t *handle, gnss_ubx_nav_velned_t *message)
{

    if (handle->ubxNavVelned == NULL)
    {
        handle->ubxNavVelned = (gnss_ubx_nav_velned_t *)malloc(sizeof(gnss_ubx_nav_velned_t));
        
        if (handle->ubxNavVelned == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-NAV-VELNED message");
            #endif
            return 2;
        }

        handle->ubxNavVelned->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_nav_velned_t *msg = handle->ubxNavVelned;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_NAV_VELNED >> 8) & 0xFF, GNSS_UBX_NAV_VELNED & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] VELNED poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-RXM-MEAS20 message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_rxm_meas20(gnss_t *handle, gnss_ubx_rxm_meas20_t *message)
{

    if (handle->ubxRxmMeas20 == NULL)
    {
        handle->ubxRxmMeas20 = (gnss_ubx_rxm_meas20_t *)malloc(sizeof(gnss_ubx_rxm_meas20_t));
        
        if (handle->ubxRxmMeas20 == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEAS20 message");
            #endif
            return 2;
        }

        handle->ubxRxmMeas20->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_rxm_meas20_t *msg = handle->ubxRxmMeas20;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_RXM_MEAS20 >> 8) & 0xFF, GNSS_UBX_RXM_MEAS20 & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] MEAS20 poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-RXM-MEAS50 message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_rxm_meas50(gnss_t *handle, gnss_ubx_rxm_meas50_t *message)
{

    if (handle->ubxRxmMeas50 == NULL)
    {
        handle->ubxRxmMeas50 = (gnss_ubx_rxm_meas50_t *)malloc(sizeof(gnss_ubx_rxm_meas50_t));
        
        if (handle->ubxRxmMeas50 == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEAS50 message");
            #endif
            return 2;
        }

        handle->ubxRxmMeas50->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_rxm_meas50_t *msg = handle->ubxRxmMeas50;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_RXM_MEAS50 >> 8) & 0xFF, GNSS_UBX_RXM_MEAS50 & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] MEAS50 poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-RXM-MEASC12 message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_rxm_measc12(gnss_t *handle, gnss_ubx_rxm_measc12_t *message)
{

    if (handle->ubxRxmMeasc12 == NULL)
    {
        handle->ubxRxmMeasc12 = (gnss_ubx_rxm_measc12_t *)malloc(sizeof(gnss_ubx_rxm_measc12_t));
        
        if (handle->ubxRxmMeasc12 == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEASC12 message");
            #endif
            return 2;
        }

        handle->ubxRxmMeasc12->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_rxm_measc12_t *msg = handle->ubxRxmMeasc12;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_RXM_MEASC12 >> 8) & 0xFF, GNSS_UBX_RXM_MEASC12 & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] MEASC12 poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-RXM-MEASD12 message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_rxm_measd12(gnss_t *handle, gnss_ubx_rxm_measd12_t *message)
{

    if (handle->ubxRxmMeasd12 == NULL)
    {
        handle->ubxRxmMeasd12 = (gnss_ubx_rxm_measd12_t *)malloc(sizeof(gnss_ubx_rxm_measd12_t));
        
        if (handle->ubxRxmMeasd12 == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEASD12 message");
            #endif
            return 2;
        }

        handle->ubxRxmMeasd12->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_rxm_measd12_t *msg = handle->ubxRxmMeasd12;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_RXM_MEASD12 >> 8) & 0xFF, GNSS_UBX_RXM_MEASD12 & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] MEASD12 poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-RXM-MEASX message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_rxm_measx(gnss_t *handle, gnss_ubx_rxm_measx_t *message)
{

    if (handle->ubxRxmMeasx == NULL)
    {
        handle->ubxRxmMeasx = (gnss_ubx_rxm_measx_t *)malloc(sizeof(gnss_ubx_rxm_measx_t));
        
        if (handle->ubxRxmMeasx == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-MEASX message");
            #endif
            return 2;
        }

        handle->ubxRxmMeasx->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_rxm_measx_t *msg = handle->ubxRxmMeasx;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_RXM_MEASX >> 8) & 0xFF, GNSS_UBX_RXM_MEASX & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] MEASX poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-RXM-RLM message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_rxm_rlm(gnss_t *handle, gnss_ubx_rxm_rlm_t *message)
{

    if (handle->ubxRxmRlm == NULL)
    {
        handle->ubxRxmRlm = (gnss_ubx_rxm_rlm_t *)malloc(sizeof(gnss_ubx_rxm_rlm_t));
        
        if (handle->ubxRxmRlm == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-RLM message");
            #endif
            return 2;
        }

        handle->ubxRxmRlm->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_rxm_rlm_t *msg = handle->ubxRxmRlm;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_RXM_RLM >> 8) & 0xFF, GNSS_UBX_RXM_RLM & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] RLM poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-RXM-SFRBX message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_rxm_sfrbx(gnss_t *handle, gnss_ubx_rxm_sfrbx_t *message)
{

    if (handle->ubxRxmSfrbx == NULL)
    {
        handle->ubxRxmSfrbx = (gnss_ubx_rxm_sfrbx_t *)malloc(sizeof(gnss_ubx_rxm_sfrbx_t));
        
        if (handle->ubxRxmSfrbx == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-RXM-SFRBX message");
            #endif
            return 2;
        }

        handle->ubxRxmSfrbx->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_rxm_sfrbx_t *msg = handle->ubxRxmSfrbx;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_RXM_SFRBX >> 8) & 0xFF, GNSS_UBX_RXM_SFRBX & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SFRBX poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-TIM-TM2 message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_tim_tm2(gnss_t *handle, gnss_ubx_tim_tm2_t *message)
{

    if (handle->ubxTimTm2 == NULL)
    {
        handle->ubxTimTm2 = (gnss_ubx_tim_tm2_t *)malloc(sizeof(gnss_ubx_tim_tm2_t));
        
        if (handle->ubxTimTm2 == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-TIM-TM2 message");
            #endif
            return 2;
        }

        handle->ubxTimTm2->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_tim_tm2_t *msg = handle->ubxTimTm2;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_TIM_TM2 >> 8) & 0xFF, GNSS_UBX_TIM_TM2 & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TM2 poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-TIM-TP message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_tim_tp(gnss_t *handle, gnss_ubx_tim_tp_t *message)
{

    if (handle->ubxTimTp == NULL)
    {
        handle->ubxTimTp = (gnss_ubx_tim_tp_t *)malloc(sizeof(gnss_ubx_tim_tp_t));
        
        if (handle->ubxTimTp == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-TIM-TP message");
            #endif
            return 2;
        }

        handle->ubxTimTp->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_tim_tp_t *msg = handle->ubxTimTp;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_TIM_TP >> 8) & 0xFF, GNSS_UBX_TIM_TP & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] TP poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-TIM-VRFY message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_tim_vrfy(gnss_t *handle, gnss_ubx_tim_vrfy_t *message)
{

    if (handle->ubxTimVrfy == NULL)
    {
        handle->ubxTimVrfy = (gnss_ubx_tim_vrfy_t *)malloc(sizeof(gnss_ubx_tim_vrfy_t));
        
        if (handle->ubxTimVrfy == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-TIM-VRFY message");
            #endif
            return 2;
        }

        handle->ubxTimVrfy->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_tim_vrfy_t *msg = handle->ubxTimVrfy;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_TIM_VRFY >> 8) & 0xFF, GNSS_UBX_TIM_VRFY & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] VRFY poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-COMMS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_comms(gnss_t *handle, gnss_ubx_mon_comms_t *message)
{

    if (handle->ubxMonComms == NULL)
    {
        handle->ubxMonComms = (gnss_ubx_mon_comms_t *)malloc(sizeof(gnss_ubx_mon_comms_t));
        
        if (handle->ubxMonComms == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-COMMS message");
            #endif
            return 2;
        }

        handle->ubxMonComms->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_mon_comms_t *msg = handle->ubxMonComms;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_MON_COMMS >> 8) & 0xFF, GNSS_UBX_MON_COMMS & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] COMMS poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-GNSS message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: Failed to initialize memory for message object
 * 2: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_gnss(gnss_t *handle, gnss_ubx_mon_gnss_t *message)
{

    if (handle->ubxMonGnss == NULL)
    {
        handle->ubxMonGnss = (gnss_ubx_mon_gnss_t *)malloc(sizeof(gnss_ubx_mon_gnss_t));
        
        if (handle->ubxMonGnss == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-GNSS message");
            #endif
            return 1;
        }

    }

    gnss_ubx_mon_gnss_t *msg = handle->ubxMonGnss;


    uint8_t data = 0x00;

    msg->version = 0x01;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

    // Send out poll request
    gnss_ubx_msg(handle, (GNSS_UBX_MON_GNSS >> 8) & 0xFF, GNSS_UBX_MON_GNSS & 0xFF, 0x0000, &data, 1);

    timer_start(&gen_timer);

    while (msg->version == 1)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] GNSS poll request timed out");
            #endif
            return 2;
        }

        gnss_rec_and_parse(handle);

    }

    *message = *msg;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-HW3 message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_hw3(gnss_t *handle, gnss_ubx_mon_hw3_t *message)
{

    if (handle->ubxMonHw3 == NULL)
    {
        handle->ubxMonHw3 = (gnss_ubx_mon_hw3_t *)malloc(sizeof(gnss_ubx_mon_hw3_t));
        
        if (handle->ubxMonHw3 == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-HW3 message");
            #endif
            return 2;
        }

        handle->ubxMonHw3->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_mon_hw3_t *msg = handle->ubxMonHw3;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_MON_HW3 >> 8) & 0xFF, GNSS_UBX_MON_HW3 & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] HW3 poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-PATCH message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: Failed to initialize memory for message object
 * 2: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_patch(gnss_t *handle, gnss_ubx_mon_patch_t *message)
{

    if (handle->ubxMonPatch == NULL)
    {
        handle->ubxMonPatch = (gnss_ubx_mon_patch_t *)malloc(sizeof(gnss_ubx_mon_patch_t));
        
        if (handle->ubxMonPatch == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-PATCH message");
            #endif
            return 1;
        }

    }

    gnss_ubx_mon_patch_t *msg = handle->ubxMonPatch;


    uint8_t data = 0x00;

    msg->version = 0x00;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

    // Send out poll request
    gnss_ubx_msg(handle, (GNSS_UBX_MON_PATCH >> 8) & 0xFF, GNSS_UBX_MON_PATCH & 0xFF, 0x0000, &data, 1);

    timer_start(&gen_timer);

    while (msg->version == 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] PATCH poll request timed out");
            #endif
            return 2;
        }

        gnss_rec_and_parse(handle);

    }

    *message = *msg;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-RCVRSTAT message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_rcvrstat(gnss_t *handle, gnss_ubx_mon_rcvrstat_t *message)
{

    if (handle->ubxMonRcvrstat == NULL)
    {
        handle->ubxMonRcvrstat = (gnss_ubx_mon_rcvrstat_t *)malloc(sizeof(gnss_ubx_mon_rcvrstat_t));
        
        if (handle->ubxMonRcvrstat == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-RCVRSTAT message");
            #endif
            return 2;
        }

    }

    gnss_ubx_mon_rcvrstat_t *msg = handle->ubxMonRcvrstat;

    uint8_t data = 0x00;

    msg->version = 0x00;

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

    // Send out poll request
    gnss_ubx_msg(handle, (GNSS_UBX_MON_RCVRSTAT >> 8) & 0xFF, GNSS_UBX_MON_RCVRSTAT & 0xFF, 0x0000, &data, 1);

    timer_start(&gen_timer);

    while (msg->version == 0)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] RCVRSTAT poll request timed out");
            #endif
            return 3;
        }

        gnss_rec_and_parse(handle);

    }

    *message = *msg;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-RF message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_rf(gnss_t *handle, gnss_ubx_mon_rf_t *message)
{

    if (handle->ubxMonRf == NULL)
    {
        handle->ubxMonRf = (gnss_ubx_mon_rf_t *)malloc(sizeof(gnss_ubx_mon_rf_t));
        
        if (handle->ubxMonRf == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-RF message");
            #endif
            return 2;
        }

        handle->ubxMonRf->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_mon_rf_t *msg = handle->ubxMonRf;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_MON_RF >> 8) & 0xFF, GNSS_UBX_MON_RF & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] RF poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-SPAN message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_span(gnss_t *handle, gnss_ubx_mon_span_t *message)
{

    if (handle->ubxMonSpan == NULL)
    {
        handle->ubxMonSpan = (gnss_ubx_mon_span_t *)malloc(sizeof(gnss_ubx_mon_span_t));
        
        if (handle->ubxMonSpan == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-SPAN message");
            #endif
            return 2;
        }

        handle->ubxMonSpan->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_mon_span_t *msg = handle->ubxMonSpan;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_MON_SPAN >> 8) & 0xFF, GNSS_UBX_MON_SPAN & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SPAN poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-MON-VER message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: Failed to initialize memory for message object
 * 2: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_mon_version(gnss_t *handle, gnss_ubx_mon_ver_t *message)
{

    if (handle->ubxMonVer == NULL)
    {
        handle->ubxMonVer = (gnss_ubx_mon_ver_t *)malloc(sizeof(gnss_ubx_mon_ver_t));
        
        if (handle->ubxMonVer == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-MON-VER message");
            #endif
            return 1;
        }

    }

    gnss_ubx_mon_ver_t *msg = handle->ubxMonVer;


    uint8_t data = 0x00;

    memset(msg->swVersion, 0x255, 30);

    timer_handle_t gen_timer;
    timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

    // Send out poll request
    gnss_ubx_msg(handle, (GNSS_UBX_MON_VER >> 8) & 0xFF, GNSS_UBX_MON_VER & 0xFF, 0x0000, &data, 1);

    timer_start(&gen_timer);

    while (msg->swVersion[0] == 255)
    {

        if (timer_check_exp(&gen_timer))
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] VER poll request timed out");
            #endif
            return 2;
        }

        gnss_rec_and_parse(handle);

    }

    *message = *msg;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-SEC-SIG message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_sec_sig(gnss_t *handle, gnss_ubx_sec_sig_t *message)
{

    if (handle->ubxSecSig == NULL)
    {
        handle->ubxSecSig = (gnss_ubx_sec_sig_t *)malloc(sizeof(gnss_ubx_sec_sig_t));
        
        if (handle->ubxSecSig == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-SEC-SIG message");
            #endif
            return 2;
        }

        handle->ubxSecSig->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_sec_sig_t *msg = handle->ubxSecSig;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_SEC_SIG >> 8) & 0xFF, GNSS_UBX_SEC_SIG & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SEC_SIG poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

/****************************************************************************
 * @brief Retrieve UBX-SEC-SIGLOG message.
 * @param handle Handle for ublox gnss module.
 * @param message Pointer for message object to copy data to.
 * @return 0: Success
 * 1: No messages available for periodic message (data is stale/has already been read)
 * 2: Failed to initialize memory for message object
 * 3: Timed out waiting for return message
 ****************************************************************************/
uint8_t gnss_get_sec_siglog(gnss_t *handle, gnss_ubx_sec_siglog_t *message)
{

    if (handle->ubxSecSiglog == NULL)
    {
        handle->ubxSecSiglog = (gnss_ubx_sec_siglog_t *)malloc(sizeof(gnss_ubx_sec_siglog_t));
        
        if (handle->ubxSecSiglog == NULL)
        {
            #ifdef DEBUG
            Serial.println("[DEBUG] Failed to allocate initial memory for UBX-SEC-SIGLOG message");
            #endif
            return 2;
        }

        handle->ubxSecSiglog->periodic = false; // Periodic should be set using gnss_set_msg_auto
    }

    gnss_ubx_sec_siglog_t *msg = handle->ubxSecSiglog;

    if (msg->periodic) // Configured for periodic message
    {

        if (msg->stale)
            return 1;

    }
    else // Message is polled
    {

        uint8_t data = 0x00;

        msg->stale = true;

        timer_handle_t gen_timer;
        timer_init(&gen_timer, 750000); // 750ms, 250 tends to be too quick, 500 misses some messages

        // Send out poll request
        gnss_ubx_msg(handle, (GNSS_UBX_SEC_SIGLOG >> 8) & 0xFF, GNSS_UBX_SEC_SIGLOG & 0xFF, 0x0000, &data, 1);

        timer_start(&gen_timer);

        while (msg->stale)
        {

            if (timer_check_exp(&gen_timer))
            {
                #ifdef DEBUG
                Serial.println("[DEBUG] SEC_SIGLOG poll request timed out");
                #endif
                return 3;
            }

            gnss_rec_and_parse(handle);

        }
    }

    *message = *msg;

    msg->stale = true;

    return 0;

}

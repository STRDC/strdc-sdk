/*
 * GNSS-UBLOX Module H File.
 *
 * @file        gnss-ublox.h
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

#ifndef GNSS_UBLOX_H
#define GNSS_UBLOX_H

#include "i2c_master.h"
#include "spi_master.h"
#include "serial_master.h"
#include "timer.h"
#include "gpio.h"

#include "gnss-ublox_addrs.h"

#define GNSS_BUFFER_SIZE 2048
#define MSG_BUFFER_SIZE 1024

// Bus Types
#define GNSS_I2C 0
#define GNSS_UART 1
#define GNSS_SPI 2

// UART Baudrate Options
#define GNSS_UART_BAUD_4800 4800
#define GNSS_UART_BAUD_9600 9600
#define GNSS_UART_BAUD_19200 19200
#define GNSS_UART_BAUD_38400 38400
#define GNSS_UART_BAUD_57600 57600
#define GNSS_UART_BAUD_115200 115200
#define GNSS_UART_BAUD_230400 230400
#define GNSS_UART_BAUD_460800 460800
#define GNSS_UART_BAUD_921600 921600

// Reset Types
#define GNSS_RST_HOT 0x0000
#define GNSS_RST_WARM 0x0001
#define GNSS_RST_COLD 0xFFFF

// Reset Modes
#define GNSS_RST_HW 0x00
#define GNSS_RST_SW 0x01
#define GNSS_RST_SW_GNSS 0x02
#define GNSS_RST_HW_SHUTDOWN 0x04
#define GNSS_RST_GNSS_STOP 0x08
#define GNSS_RST_GNSS_START 0x09

// Memory Locations for cfg set
#define GNSS_RAM 1
#define GNSS_BBR 2
#define GNSS_BBR_RAM 3
#define GNSS_FLASH 4
#define GNSS_FLASH_RAM 5
#define GNSS_FLASH_BBR 6
#define GNSS_ALL_MEM 7

// Options for CFG-NAVSPG-FIXMODE
#define GNSS_FIX_2DONLY 1
#define GNSS_FIX_3DONLY 2
#define GNSS_FIX_AUTO 3

// Options for CFG-NAVSPG-UTCSTANDARD
#define GNSS_UTCSTANDARD_AUTO 0
#define GNSS_UTCSTANDARD_USNO 3
#define GNSS_UTCSTANDARD_EU 5
#define GNSS_UTCSTANDARD_SU 6
#define GNSS_UTCSTANDARD_NTSC 7
#define GNSS_UTCSTANDARD_NPLI 8
#define GNSS_UTCSTANDARD_NICT 9

// Options for CFG-NAVSPG-DYNMODEL
#define GNSS_DYNMODEL_PORT 0
#define GNSS_DYNMODEL_STAT 2
#define GNSS_DYNMODEL_PED 3
#define GNSS_DYNMODEL_AUTOMOT 4
#define GNSS_DYNMODEL_SEA 5
#define GNSS_DYNMODEL_AIR1 6
#define GNSS_DYNMODEL_AIR2 7
#define GNSS_DYNMODEL_AIR4 8
#define GNSS_DYNMODEL_WRIST 9
#define GNSS_DYNMODEL_BIKE 10
#define GNSS_DYNMODEL_MOWER 11
#define GNSS_DYNMODEL_ESCOOTER 12

// Options for CFG-NAVSPG-SIGATTCOMP (specific dBHz options can be selected using the number of dBHz e.g. 08DBHZ = 8)
#define GNSS_SIGATTCOMP_DIS 0
#define GNSS_SIGATTCOMP_AUTO 255

// Options for CFG-ODO-PROFILE
#define GNSS_ODO_PROFILE_RUN 0
#define GNSS_ODO_PROFILE_CYCL 1
#define GNSS_ODO_PROFILE_SWIM 2
#define GNSS_ODO_PROFILE_CAR 3
#define GNSS_ODO_PROFILE_CUSTOM 4

// Options for CFG-GEOFENCE-CONFLVL
#define GNSS_GEOFENCE_CONFLVL_L000 0
#define GNSS_GEOFENCE_CONFLVL_L680 1
#define GNSS_GEOFENCE_CONFLVL_L950 2
#define GNSS_GEOFENCE_CONFLVL_L997 3
#define GNSS_GEOFENCE_CONFLVL_L9999 4
#define GNSS_GEOFENCE_CONFLVL_L999999 5

// Options for CFG-GEOFENCE-PINPOL
#define GNSS_GEOFENCE_PINPOL_LOW_IN 0 // PIO low means inside geofence
#define GNSS_GEOFENCE_PINPOL_LOW_OUT 1 // PIO low means outside geofence

// Options for Chip selection on certain functions (e.g. gnss_enable_ready())
#define GNSS_SAM_M10Q 0
#define GNSS_NEO_M9N 1
#define GNSS_NEO_M9V 2

// UBX GNSSIDs
#define GNSS_UBX_ID_GPS 0x00
#define GNSS_UBX_ID_SBAS 0x01
#define GNSS_UBX_ID_GAL 0x02
#define GNSS_UBX_ID_BDS 0x03
#define GNSS_UBX_ID_QZSS 0x05
#define GNSS_UBX_ID_GLO 0x06
#define GNSS_UBX_ID_NAVIC 0x07

// NMEA System IDs (4.11 only)
#define GNSS_NMEA_SYS_ID_GPS 0x01
#define GNSS_NMEA_SYS_ID_SBAS 0x01
#define GNSS_NMEA_SYS_ID_GAL 0x03
#define GNSS_NMEA_SYS_ID_BDS 0x04
#define GNSS_NMEA_SYS_ID_QZSS 0x05
#define GNSS_NMEA_SYS_ID_GLO 0x02
#define GNSS_NMEA_SYS_ID_NAVIC 0x06

typedef struct {

    uint8_t buffer[MSG_BUFFER_SIZE]; // Message Buffer - Eats up a lot of memory but necessary to deal with some larger messages
    uint16_t length; // Length of Message

} gnss_msg_t; // Handler for BNO08x

typedef struct {
    
    uint8_t gnssID;
    uint8_t svID;
    uint8_t health; // 0: Unknown, 1: Healthy, 2: Not Healthy
    uint8_t visibility; // 0: Unknown, 1: Below Horizon, 2: Above Horizon, 3: Above Elevation Mask
    uint8_t ephUsability; // How long receiver will be able to use stored ephemeris data from now on. 31: Unknown, 30: More than 450 min, 30>n>0: Between (n-1)*15 and n*15 min, 0: Ephemeris can no longer be used
    uint8_t ephSource; // 0: Not Available, 1: GNSS transmission, 2: External Aiding, 3-7: Other
    uint8_t almUsability; // How long receiver will be able to use stored almanac data from now on. 31: Unknown, 30: More than 450 min, 30>n>0: Between (n-1)*15 and n*15 min, 0: Almanac can no longer be used
    uint8_t almSource; // 0: Not Available, 1: GNSS transmission, 2: External Aiding, 3-7: Other
    uint8_t anoAopUsability; // How long receiver will be able to use orbit data from now on. 31: Unknown, 30: More than 450 min, 30>n>0: Between (n-1)*15 and n*15 min, 0: Data can no longer be used
    uint8_t type; // Type of orbit data. 0: No Orbit Data available, 1: AssistNow Offline Data, 2: AssistNow Autonomous Data, 3-7: Other orbit data

} gnss_sv_orb_t;

typedef struct {
    
    uint8_t gnssID;
    uint8_t svID;
    uint8_t cno; // Carrier to noise ratio (signal strength)
    int8_t elev; // Elevation (+/- 90 deg), unknown if out of range
    int16_t azim; // Azimuth (0-360 deg), unknown if elevation is out of range
    float prRes; // Pseudorange residual (m)
    uint8_t qualityInd; // Signal Quality Indicator. 0: No Signal, 1: Searching Signal, 2: Signal Acquired, 3: Signal detected but unusable, 4: Code locked and time synched, 5-7: Code and carrier locked and time synched
    bool svUsed; // Signal currently used for navigation
    uint8_t health; // 0: Unknown, 1: Healthy, 2: Not Healthy
    bool diffCorr; // Differential Correction data available for this SV
    bool smoothed; // Carrier Smoothed Pseudorange used
    uint8_t orbitSource; // 0: No orbit info available, 1: Ephemeris used, 2: Almanac used, 3: AssistNow Offline Orbit used, 4: AssistNow Autonomous Orbit used, 5-7: Other Orbit Info used
    
    bool ephAvail; // Ephemeris avaialable for this SV
    bool almAvail; // Almanac avaialable for this SV
    bool anoAvail; // AssistNow Offline data avaialable for this SV
    bool aopAvail; // AssistNow Autonomous data avaialable for this SV

    bool sbasCorrUsed; // SBAS Corrections have been used for a signal within this GNSS
    bool rtcmCorrUsed; // RTCM Corrections have been used for a signal within this GNSS
    bool slasCorrUsed; // QZSS Corrections have been used for a signal within this GNSS
    bool spartnCorrUsed; // SPARTN Corrections have been used for a signal within this GNSS
    bool prCorrUsed; // Pseudorange Corrections have been used for a signal within this GNSS
    bool crCorrUsed; // Carrier Corrections have been used for a signal within this GNSS
    bool doCorrUsed; // Range Rate (Doppler) Corrections have been used for a signal within this GNSS
    bool clasCorrUsed; // CLAS Corrections have been used for a signal within this GNSS

} gnss_sv_sat_t;

typedef struct {

    uint8_t svID; // SV ID
    uint8_t udre; // Monitoring Status
    uint8_t svSys; // System (same as SYS)
    uint8_t svService; // Services available (same as SERVICE)
    int16_t prc; // Pseudorange correction (cm)
    int16_t ic; // Ionosphere correction (cm)

} gnss_sv_sbas_t;

typedef struct {

    uint8_t gnssID;
    uint8_t svID; // SV ID
    uint8_t sigID; // New Style Signal Identifier
    uint8_t freqID; // Only used for GLONASS, Frequency Slot + 7 (0 - 13)
    float prRes; // Pseudorange residual (m)
    uint8_t cno; // Carrier to noise ratio (signal strength)
    uint8_t qualityInd; // Signal Quality Indicator. 0: No Signal, 1: Searching Signal, 2: Signal Acquired, 3: Signal detected but unusable, 4: Code locked and time synched, 5-7: Code and carrier locked and time synched
    uint8_t corrSource; // Correction Source. 0: No Corrections, 1: SBAS, 2: BeiDou, 3: RTCM2, 4: RTCM3 OSR, 5: RTCM3 SSR, 6: QZSS SLAS, 7: SPARTN, 8: CLAS
    uint8_t ionoModel; // Ionospheric Model. 0: No Model, 1: Klobuchar Model transmitted by GPS, 2: SBAS Model, 3: Klobuchar Model transmitted by BeiDou, 8: Iono Delay Derived from Dual Frequency Observations
    uint8_t health; // 0: Unknown, 1: Healthy, 2: Not Healthy
    bool prSmoothed; // Pseudorange has been smoothed
    bool prUsed; // Pseudorange was used
    bool crUsed; // Carrier Range was used
    bool doUsed; // Range Rate (Doppler) was used
    bool prCorrUsed; // Pseudorange Corrections have been used for a signal within this GNSS
    bool crCorrUsed; // Carrier Corrections have been used for a signal within this GNSS
    bool doCorrUsed; // Range Rate (Doppler) Corrections have been used for a signal within this GNSS
    bool authStatus; // Only OSNMA provides authentication for E1/NAV message. 0: Unknown, 1: Authenticated

} gnss_sv_sig_t;

typedef struct {

    uint8_t gnssID;
    uint8_t svID; // SV ID
    int16_t prc; // Pseudorange Correction (cm)

} gnss_sv_slas_t;

typedef struct {

    uint8_t gnssID;
    uint8_t svID;
    uint8_t cNo; // Carrier noise ration (0 - 63)
    uint8_t mpathIndic; // Multipath index. 0: Not measured, 1: Low, 2: Medium, 3: High
    
    float dopplerMS; // Doppler Measurement (m/s)
    float dopplerHz; // Doppler Measurement (Hz)
    uint16_t wholeChips; // Whole value of code phase measurement (0 - 1022)
    uint16_t fracChips; // Fractional value of code phase measurement (0 - 1023)
    double codePhase; // Code phase (ms)
    uint8_t intCodePhase; // Integer portion of code phase
    uint8_t pseuRangeRMSErr; // Pseudorange RMS error index (0 - 63)

} gnss_sv_measx_t;

typedef struct {

    uint8_t svID;
    uint8_t elv; // Elevation (<= 90) (deg)
    uint16_t az; // Azimuth (0 - 359) (deg)
    uint8_t cno; // Signal Strength (0 - 99) (dBHz)

} gnss_sv_gsv_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    char datum[3];
    double latOffset;
    double lonOffset;
    double altOffset;
    char refDatum[3];

} gnss_nmea_std_dtm_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    float errLat; // m
    float errLon; // m
    float errAlt; // m
    uint8_t svID; // Most likely failed satellite
    float bias; // Of most likely failed satellite
    float stddev; // Std Dev of estimated bias
    uint8_t systemID;
    uint8_t signalID;

} gnss_nmea_std_gbs_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    double latitude;
    double longitude;

    uint8_t quality;
    uint8_t numSV;
    float hdop;

    double altitude; // m
    double sep; // Geoid separation (m)

    uint8_t diffAge; // s
    char diffStation[32];

} gnss_nmea_std_gga_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    double latitude;
    double longitude;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    char status;
    char posMode;

} gnss_nmea_std_gll_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    double latitude;
    double longitude;

    char posMode;

    uint8_t numSV;

    float hdop;

    double altitude; // m
    double sep; // Geoid separation (m)

    uint8_t diffAge; // s
    char diffStation[32];

    char navStatus;

} gnss_nmea_std_gns_t;

typedef struct {

    bool stale; // For each individual message

    uint8_t talkerID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    float residual[12]; // Range residuals for SVs used. SV order matches order from GSA message. (m)

    uint8_t systemID;
    uint8_t signalID;

} gnss_grs_t;

typedef struct {

    bool stale; // For entire message
    bool periodic;

    gnss_grs_t *grs[8];

} gnss_nmea_std_grs_t;

typedef struct {

    bool stale; // For each individual message

    uint8_t talkerID;

    char opMode;
    uint8_t navMode;

    uint8_t svID[12];

    float pdop;
    float hdop;
    float vdop;

    uint8_t systemID;

} gnss_gsa_t;

typedef struct {

    bool stale; // For entire message
    bool periodic;

    gnss_gsa_t *gsa[8];

} gnss_nmea_std_gsa_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    float rangeRms; // m
    float stdMajor; // m
    float stdMinor; // m
    float orient; // deg
    float stdLat; // m
    float stdLong; // m
    float stdAlt; // m

} gnss_nmea_std_gst_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    uint8_t numMsg; // Number of messages (total number of GSV messages to be output)
    uint8_t msgNum; // Number of this message
    uint8_t numSV;

    gnss_sv_gsv_t sv[4];

    uint8_t signalID;

} gnss_nmea_std_gsv_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    char beacon[15]; // Beacon ID

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    char code; // Message code to identify RLM message service

    char body[4]; // Data Parameters provided by RLSP

} gnss_nmea_std_rlm_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    char status;

    double latitude;
    double longitude;

    float spd; // Speed over ground (knots)
    float cog; // Cource over ground (deg)

    uint8_t dayUTC;
    uint8_t monthUTC;
    uint8_t yearUTC;

    float mv; // Magnetic variation (deg)

    char posMode;
    char navStatus;

} gnss_nmea_std_rmc_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    float tgd; // Total cumulative ground distance (nautical mi)
    float gd; // Ground distance since reset (nautical mi)

} gnss_nmea_std_vlw_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    float cogt; // Course over ground (true) (deg)
    float cogm; // Course over ground (magnetic) (deg)
    float sogn; // Speed over ground (knots)
    float sogk; // Speed over ground (km/h)
    char posMode;

} gnss_nmea_std_vtg_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t talkerID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    uint8_t dayUTC;
    uint8_t monthUTC;
    uint16_t yearUTC;

} gnss_nmea_std_zda_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t msgID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    double latitude;
    double longitude;

    double altRef;

    uint8_t navStat[2];

    float hAcc; // Horizontal Accuracy (mm)
    float vAcc; // Vertical Accuracy (mm)

    float sog; // Speed over ground (knots)
    float cog; // Cource over ground (deg)

    float vVel; // Vertical velocity (positive downwards) (m/s)

    uint8_t diffAge; // s
    
    float hdop; // Horizontal Dilution of Precision
    float vdop; // Vertical Dilution of Precision
    float tdop; // Time Dilution of Precision
    
    uint8_t numSvs; // Number of satellites used in nav solution

    uint8_t dr; // DR used

} gnss_nmea_pubx_pos_t;

typedef struct {

    uint8_t sv; // Satellite ID according to UBX svID mapping
    char status; // -: Not used, U: Used, e: Ephemeris available, but not used for nav

    uint16_t az; // Satellite azmuth (0 - 359) (deg)
    uint8_t el; // Satellite elevation (<= 90) (deg)
    uint8_t cno; // Signal strength (C/N0, 0 - 99), blank when not tracking (dBHz)
    uint8_t lck; // Satellite carrier lock time (0 - 64) (s) 0: Code lock only, 64: lock for 64 seconds or more

} gnss_sv_svstatus_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t msgID;
    uint8_t n; // Number of GNSS satellites tracked

    gnss_sv_svstatus_t sv[255];

} gnss_nmea_pubx_svstatus_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t msgID;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    uint8_t milsUTC;

    uint8_t dayUTC;
    uint8_t monthUTC;
    uint8_t yearUTC;

    float utcTow; // UTC time of week
    uint16_t utcWk; // UTC week number

    uint8_t leapSec; // Leap seconds (s)

    float clkBias; // Clock bias (ns)
    float clkDrift; // Clock drift (ns/s)

    uint8_t tpGran; // Timepulse Granularity

} gnss_nmea_pubx_time_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    bool useAOP; // AOP enabled
    uint8_t status; // Assist Now Autonomous subsystem is idle (0) or running (!0)

} gnss_ubx_nav_aopstatus_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    int32_t bias; // Clock bias (ns)
    int32_t drift; // Clock drift (ns/s)
    uint32_t tAcc; // Time Accuracy Estimate (ns)
    uint32_t fAcc; // Frequency Accuracy Estimate (ps/s)

} gnss_ubx_nav_clock_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t version; // Message Version
    uint8_t posCovValid; // Position Covariance Matrix Validity Flag
    uint8_t velCovValid; // Velocity Covariance Matrix Validity Flag
    float posCovNN; // Position Covariance Matrix Value p_NN
    float posCovNE; // Position Covariance Matrix Value p_NE
    float posCovND; // Position Covariance Matrix Value p_ND
    float posCovEE; // Position Covariance Matrix Value p_EE
    float posCovED; // Position Covariance Matrix Value p_ED
    float posCovDD; // Position Covariance Matrix Value p_DD
    float velCovNN; // Velocity Covariance Matrix Value p_NN
    float velCovNE; // Velocity Covariance Matrix Value p_NE
    float velCovND; // Velocity Covariance Matrix Value p_ND
    float velCovEE; // Velocity Covariance Matrix Value p_EE
    float velCovED; // Velocity Covariance Matrix Value p_ED
    float velCovDD; // Velocity Covariance Matrix Value p_DD


} gnss_ubx_nav_cov_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    float gdop; // Geometric Dilution of Precision
    float pdop; // Position Dilution of Precision
    float tdop; // Time Dilution of Precision
    float vdop; // Vertical Dilution of Precision
    float hdop; // Horizontal Dilution of Precision
    float ndop; // Northing Dilution of Precision
    float edop; // Easting Dilution of Precision

} gnss_ubx_nav_dop_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

} gnss_ubx_nav_eoe_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint32_t distance; // Ground distance since last reset (m)
    uint32_t totalDistance; // Total cumulative ground distance (m)
    uint32_t distanceStd; // Ground distance accuracy (1 - sigma) (m)

} gnss_ubx_nav_odo_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t version;
    uint8_t numSv; // Number of Satellites in database

    gnss_sv_orb_t sv[255];

} gnss_ubx_nav_orb_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t msgVersion; // Message Version
    uint8_t tmirCoeff; // Target Misleading Information Risk (TMIR) [%MI/epoch] Integer
    int8_t tmirExp; // TMIR Exponent (tmirCoeff * 10 ^ tmirExp)
    bool plPosValid; // Position Protection Level Validity
    uint8_t plPosFrame; // Position Protection Level Frame. 0: Invalid, 1: North-East-Down, 2: Longitudinal-Lateral-Vertical, 3: HorizSemiMajorAxis-HorizSemiMinorAxis-Vertical
    bool plVelValid; // Velocity Protection Level Validity
    uint8_t plVelFrame; // Velocity Protection Level Frame. 0: Invalid, 1: North-East-Down, 2: Longitudinal-Lateral-Vertical, 3: HorizSemiMajorAxis-HorizSemiMinorAxis-Vertical
    bool plTimeValid; // Time Protection Level Validity
    uint8_t plPosInvReason; // Position Protection Level Invalidity Reason. 0: Not Available, 1-29: Solution not trustworthy, 30-100: PL not verified for this receiver
    uint8_t plVelInvReason; // Velocity Protection Level Invalidity Reason. 0: Not Available, 1-29: Solution not trustworthy, 30-100: PL not verified for this receiver
    uint8_t plTimeInvReason; // Time Protection Level Invalidity Reason. 0: Not Available, 1-29: Solution not trustworthy, 30-100: PL not verified for this receiver

    uint32_t plPos1; // First axis of Position Protection Level Value (mm)
    uint32_t plPos2; // Second axis of Position Protection Level Value (mm)
    uint32_t plPos3; // Third axis of Position Protection Level Value (mm)
    uint32_t plVel1; // First axis of Velocity Protection Level Value (mm)
    uint32_t plVel2; // Second axis of Velocity Protection Level Value (mm)
    uint32_t plVel3; // Third axis of Velocity Protection Level Value (mm)

    float plPosHorizOrient; // Orientation of HorizSemiMajorAxis (deg clockwise from true North), 0 if plPosFrame != 3
    float plVelHorizOrient; // Orientation of HorizSemiMajorAxis (deg clockwise from true North), 0 if plVelFrame != 3
    uint32_t plTime; // Time protection level value with respect to TMIR

} gnss_ubx_nav_pl_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    int32_t ecefX; // Earth Centered - Earth Fixed X (cm)
    int32_t ecefY; // Earth Centered - Earth Fixed Y (cm)
    int32_t ecefZ; // Earth Centered - Earth Fixed Z (cm)

    uint32_t pAcc; // Position Accuracy (cm)

} gnss_ubx_nav_posecef_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    double longitude;
    double latitude;

    double heightEllip; // Height above ellipsiod (mm)
    double hMSL; // Above mean sea level (mm)

    uint32_t hAcc; // Horizontal Accuracy (mm)
    uint32_t vAcc; // Vertical Accuracy (mm)


} gnss_ubx_nav_posllh_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t dayUTC;
    uint8_t monthUTC;
    uint8_t yearUTC;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    int32_t nanosUTC;

    double longitude;
    double latitude;

    int32_t heightEllip; // Height above ellipsiod (mm)
    int32_t hMSL; // Above mean sea level (mm)

    uint32_t tAcc; // Accuracy (ns)

    uint8_t fixType; // See UBX-NAV-PVT: 0 = no fix, 1 = dead reckon only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckon, 5 = time only
    bool fixOK; // Achieved Fix within DOP and ACC limits

    uint8_t psmState; // Power Save State

    bool headVehValid;

    bool validDate;
    bool validTime;
    bool fullyResolved;
    bool validMag;

    bool diffSoln;

    bool confirmedAvai;
    bool confirmedDate;
    bool confirmedTime;

    bool invalidLlh;

    uint8_t lastCorrAge;

    bool authTime;

    uint8_t carrSoln; // Carrier Phase Range Solution Status. 0: No carrier phase range solution, 1: carrier phase range solution with floating ambiguities, 2: carrier phase range solution with fixed ambiguities

    uint8_t numSV; // Number of Satellites in View - Any combination of GNSS??

    uint32_t hAcc; // Horizontal Accuracy (mm)
    uint32_t vAcc; // Vertical Accuracy (mm)

    int32_t velN; // NED North Velocity (mm/s)
    int32_t velE; // NED East Velocity (mm/s)
    int32_t velD; // NED Down Velocity (mm/s)

    int32_t gSpeed; // Speed over ground (mm/s)
    double course; // GPS derived course over ground (deg)

    uint32_t sAcc; // Speed Accuracy (mm/s)
    double headAcc; // Heading (and course) Accuracy (deg)

    double pdop; // Position Dilution of Precision

    double heading;

    float magDec; // Magnetic Declination
    float magAcc; // Magnetic Declination Accuracy


} gnss_ubx_nav_pvt_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t version; // Message Version
    uint8_t numSv;

    gnss_sv_sat_t sv[255];

} gnss_ubx_nav_sat_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t geo; // PRN Number of the GEO where correction and integrity data is used from
    uint8_t mode; // SBAS Mode. 0: Disabled, 1: Enabled Integrity, 3: Enabled Test Mode
    int8_t sys; // SBAS System. -1: Unknown, 0: WAAS, 1: EGNOS, 2: MSAS, 3: GAGAN, 16: GPS
    bool ranging; // GEO may be used as ranging source
    bool corrections; // GEO is providing correction data
    bool integrity; // GEO is providing integrity
    bool testMode; // GEO is in test mode
    bool bad; // Problem with signal or broadcast data indicated
    uint8_t cnt; // Number of SV data
    uint8_t integrityUsed; // SBAS Integrity Used. 0: Unknown, 1: Integrity information is not available or SBAS integrity not enabled, 2: Receiver uses only GPS satellites for which integrity information is available

    gnss_sv_sbas_t sv[255];

} gnss_ubx_nav_sbas_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t version; // Message Version
    uint8_t numSv;

    gnss_sv_sig_t sv[255];

} gnss_ubx_nav_sig_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t version; // Message Version

    double gmsLon; // Longitude of ground monitoring station used
    double gmsLat; // Latitude of ground monitoring station used
    uint8_t gmsCode; // Code of ground monitoring station used according to QZSS SLAS Interface Specification
    uint8_t qzssSvID; // Satellite Identifier of QZS/GEO Correction data used

    bool gmsAvailable; // Ground Monitoring Station Available
    bool qzssSvAvailable; // Correction Providing QZSS SV Available
    bool testMode; // Currently used QZSS SV in test mode

    uint8_t cnt;

    gnss_sv_slas_t sv[255];

} gnss_ubx_nav_slas_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t gpsFix; // See UBX-NAV-PVT: 0 = no fix, 1 = dead reckon only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckon, 5 = time only
    bool gpsFixOK; // Position and Velocity valid and within DOP and ACC Masks
    bool diffSoln; // Differential Corrections applied
    bool wknSet; // Week Number valid
    bool towSet; // Time of Week valid
    bool diffCorr; // Differential Corrections available
    bool carrSolnValid; // Valid carrSoln
    uint8_t mapMatching; // Map Matching Status. 0: None, 1: Valid but not used, 2: Valid and used, 3: Valid and used - Enabling dead reckoning
    uint8_t psmState; // Power Save Mode State. 0: Acquisition (or disabled), 1: Tracking, 2: Power Optimized Tracking, 3: Inactive
    uint8_t spoofDetState; // Spoofing Detection State. 0: Unknown or deactivated, 1: No Spoofing indicated, 2: Spoofing Indicated, 3: Multiple Spoofing Indications
    uint8_t carrSoln; // Carrier Phase Range Solution Status. 0: No Carrier Phase Range Solution, 1: Carrier Phase Range Solution with Floating Ambiguities, 2: Carrier Phase Range Solution with Fixed Ambiguities

    uint32_t ttff; // Time to first fix (ms)
    uint32_t msss; // Time since Startup/Reset (ms)

} gnss_ubx_nav_status_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint32_t SOW; // BDS Time of Week (s)
    int32_t fSOW; // Fractional part of SOW (range of +/- 500000000). BDS time of week in seconds = SOW + fSOW * 1e-9
    int16_t week; // BDS Week Number of the nav epoch
    int8_t leapS; // BDS Leap Seconds (BDS-UTC) (s)
    bool sowValid; // Valid SOW and fSOW
    bool weekValid; // Valid Week
    bool leapSValid; // Valid leap second
    uint32_t tAcc; // Time Accuracy Estimate

} gnss_ubx_nav_timebds_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint32_t galTow; // Galileo Time of Week (s)
    int32_t fGalTow; // Fractional part of Galileo (range of +/- 500000000). Galileo time of week in seconds = galTow + fGalTow * 1e-9
    int16_t galWno; // Galileo Week Number
    int8_t leapS; // Galileo Leap Seconds (Galileo-UTC) (s)
    bool galTowValid; // Valid galTow and fGalTow
    bool galWnoValid; // Valid Week
    bool leapSValid; // Valid leap second
    uint32_t tAcc; // Time Accuracy Estimate

} gnss_ubx_nav_timegal_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint32_t TOD; // GLONASS Time of Week (s)
    int32_t fTOD; // Fractional part of TOD (range of +/- 500000000) (ns). GLONASS time of week in seconds = TOD + fTOD * 1e-9
    uint16_t Nt; // Current day of the year indicated by N4 (1-1461) (days), 1 = January 1st of the first year, 1461 = December 31st of the third year
    uint8_t N4; // Four-year interval number starting from 1996 (1=1996, 2=2000, 3=2004)
    bool todValid; // Valid TOD and fTOD
    bool dateValid; // Valid N4 and Nt
    uint32_t tAcc; // Time Accuracy Estimate (ns)

} gnss_ubx_nav_timeglo_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    int32_t fTow; // Fractional part of GPS (range of +/- 500000000). GPS time of week in seconds = iTow + fTow * 1e-9
    int16_t week; // GPS Week Number
    int8_t leapS; // GPS Leap Seconds (GPS-UTC) (s)
    bool towValid; // Valid iTOW and fTow
    bool weekValid; // Valid Week
    bool leapSValid; // Valid leap second
    uint32_t tAcc; // Time Accuracy Estimate

} gnss_ubx_nav_timegps_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t version;
    uint8_t srcOfCurrLs;
    int8_t currLs;
    uint8_t srcOfLsChange;
    int8_t lsChange;
    int32_t timeToLsEvent;
    uint16_t dateOfLsGpsWn;
    uint16_t dateOfLsGpsDn;
    bool validCurrLs;
    bool validTimeToLsEvent;

} gnss_ubx_nav_timels_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint32_t qzssTow; // QZSS Time of Week (s)
    int32_t fQzssTow; // Fractional part of QZSS (range of +/- 500000000). QZSS time of week in seconds = qzssTow + fQzssTow * 1e-9
    int16_t qzssWno; // QZSS Week Number
    int8_t leapS; // QZSS Leap Seconds (QZSS-UTC) (s)
    bool qzssTowValid; // Valid qzssTow and fQzssTow
    bool qzssWnoValid; // Valid Week
    bool leapSValid; // Valid leap second
    uint32_t tAcc; // Time Accuracy Estimate

} gnss_ubx_nav_timeqzss_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    uint8_t tAcc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    bool validTOW;
    bool validWKN;
    bool validUTC;
    bool authStatus;
    uint8_t utcStandard;

} gnss_ubx_nav_timeutc_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    int32_t ecefVX;
    int32_t ecefVY;
    int32_t ecefVZ;
    uint32_t sAcc;

} gnss_ubx_nav_velecef_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t iTOW; // GPS time of week (ms) use for message epoch timestamp only

    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t speed;
    uint32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t cAcc;

} gnss_ubx_nav_velned_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t payload[20];

} gnss_ubx_rxm_meas20_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t payload[50];

} gnss_ubx_rxm_meas50_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t payload[12];

} gnss_ubx_rxm_measc12_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t payload[12];

} gnss_ubx_rxm_measd12_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t version; // Message version
    uint32_t gpsTOW; // GPS measurement ref time (ms)
    uint32_t gloTOW; // GLONASS measurement ref time (ms)
    uint32_t bdsTOW; // BeiDou measurement ref time (ms)
    uint32_t qzssTOW; // QZSS measurement ref time (ms)

    float gpsTOWacc; // GPS measurement ref time accuracy (ms)
    float gloTOWacc; // GLONASS measurement ref time accuracy (ms)
    float bdsTOWacc; // BeiDou measurement ref time accuracy (ms)
    float qzssTOWacc; // QZSS measurement ref time accuracy (ms)

    uint8_t numSV;
    bool towSet; // TOW set

    gnss_sv_measx_t sv[255];

} gnss_ubx_rxm_measx_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t version; // Message version
    uint8_t type; // Message type (0x01 for short-RLM, 0x02 for long)
    uint8_t svID;
    uint8_t beacon[8]; // Beacon identifier, MSB first
    uint8_t message; // Message code
    uint8_t params[12]; // Parameters (2 bytes if short message, 12 if long), MSB first

} gnss_ubx_rxm_rlm_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t gnssID;
    uint8_t svID;
    uint8_t sigID; // Signal identifier
    uint8_t freqID; // Only used for GLONASS, frequency slot + 7 (0 - 13)
    uint8_t numWords; // Number of data words in message (max 10)
    uint8_t chn; // Tracking channel number message was received on
    uint8_t version; // Message version
    uint8_t dwrd[40]; // Data words, combined into one long array, but words are only 4 bytes long

} gnss_ubx_rxm_sfrbx_t;

typedef struct {

    uint8_t version; // Message version
    uint8_t uniqueID[6]; // Unique chip ID

} gnss_ubx_sec_uniqid_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t ch; // Channel (e.g. EXTINT) where pulse was measured
    
    bool mode; // 0: Single, 1: Stopped
    bool run; // 0: Armed, 1: Stopped
    bool newFallingEdge; // New falling edge detected
    uint8_t timeBase; // 0: Time base is receiver time, 1: Time base is GNSS time, 2: Time base is UTC
    bool utc; // 0: UTC not available, 1: UTC available
    bool time; // 0: Time is not valid, 1: Time is valid (Valid GNSS fix)
    bool newRisingEdge; // New rising edge detected

    uint16_t count; // Rising edge counter
    uint16_t wnR; // Week number of last rising edge
    uint16_t wnF; // Week number of last falling edge
    uint32_t towMsR; // Tow of rising edge (ms)
    uint32_t towSubMsR; // Millisecond fraction of tow of rising edge (ns)
    uint32_t towMsF; // Tow of falling edge (ms)
    uint32_t towSubMsF; // Millisecond fraction of tow of falling edge (ns)
    uint32_t accEst; // Accuracy estimate

} gnss_ubx_tim_tm2_t;

typedef struct {

    bool stale;
    bool periodic;

    uint32_t towMS; // (ms)
    double towSubMS; // Sub milisecond portion (2^-32 ms)
    int32_t qErr;
    uint16_t week;
    bool timeBase;
    bool utc;
    uint8_t raim;
    bool qErrInvalid;
    bool TpNotLocked;
    uint8_t timeRefGnss;
    uint8_t utcStandard;

} gnss_ubx_tim_tp_t;

typedef struct {

    bool stale;
    bool periodic;

    int32_t itow;
    int32_t frac;
    int32_t deltaMs;
    int32_t deltaNs;
    uint16_t wno;
    uint8_t src;

} gnss_ubx_tim_vrfy_t;

typedef struct {

    uint8_t version;
    bool extraPVT;
    bool extraODO;
    uint16_t msgCnt;
    uint32_t iTOW;

    uint16_t dayUTC;
    uint8_t monthUTC;
    uint8_t yearUTC;

    uint8_t hoursUTC;
    uint8_t minutesUTC;
    uint8_t secondsUTC;
    int32_t nanosUTC;

    bool validDate;
    bool validTime;

    uint32_t tAcc; // Accuracy (ns)
    uint8_t fixType; // See UBX-NAV-PVT: 0 = no fix, 1 = dead reckon only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckon, 5 = time only
    bool gnssFixOK; // Achieved Fix within DOP and ACC limits
    bool diffSoln;
    uint8_t psmState; // Power Save State
    uint8_t flags2;
    uint8_t numSV; // Number of Satellites in View - Any combination of GNSS??

    double longitude;
    double latitude;

    int32_t heightEllip; // Height above ellipsiod (mm)
    int32_t hMSL; // Above mean sea level (mm)

    uint32_t hAcc; // Horizontal Accuracy (mm)
    uint32_t vAcc; // Vertical Accuracy (mm)

    int32_t velN; // NED North Velocity (mm/s)
    int32_t velE; // NED East Velocity (mm/s)
    int32_t velD; // NED Down Velocity (mm/s)

    int32_t gSpeed; // Speed over ground (mm/s)
    double course; // GPS derived course over ground (deg)

    uint32_t sAcc; // Speed Accuracy (mm/s)
    double headAcc; // Heading (and course) Accuracy (deg)

    double pdop; // Position Dilution of Precision

    uint32_t distance; // Ground Distance (m) since last reset - Only valid with extraODO
    uint32_t totalDistance; // Total Cumulative Ground Distance (m) - Only valid with extraODO
    uint32_t distanceStd; // Ground distance accuracy (1 - sigma) - Only valid with extraODO

} gnss_ubx_log_batch_t;

typedef struct {

    uint8_t version;
    uint8_t type;

    uint32_t entryNumber; // Index of first log entry of requested time or < given time. 0xFFFFFFFF if not found. Zero-based

} gnss_ubx_log_findtime_t;

typedef struct {

    uint8_t version;

    uint32_t filestoreCapacity;
    uint32_t currentMaxLogSize;
    uint32_t currentLogSize;
    uint32_t entryCount;

    uint16_t oldestYear;
    uint8_t oldestMonth;
    uint8_t oldestDay;
    uint8_t oldestHour;
    uint8_t oldestMinute;
    uint8_t oldestSecond;

    uint16_t newestYear;
    uint8_t newestMonth;
    uint8_t newestDay;
    uint8_t newestHour;
    uint8_t newestMinute;
    uint8_t newestSecond;

    uint8_t recording;
    uint8_t inactive;
    uint8_t circular;

} gnss_ubx_log_info_t;

typedef struct {
    
    uint8_t version;

    uint16_t fillLevel; // Current buffer fill level
    uint16_t dropsAll; // Number of dropped epochs since startup
    uint16_t dropsSinceMon; // Number of dropped epochs since last MON-BATCH message
    uint16_t nextMsgCnt; // The next retrieved UBX-LOG-BATCH will have this msgCnt value

} gnss_ubx_mon_batch_t;

typedef struct {

    uint16_t portID;
    uint16_t txPending;
    uint32_t txBytes;
    uint8_t txUsage;
    uint8_t txPeakUsage;
    uint16_t rxPending;
    uint32_t rxBytes;
    uint8_t rxUsage;
    uint8_t rxPeakUsage;
    uint16_t overrunErrs;
    uint16_t msgs[4];
    uint32_t skipped;

} gnss_ports_comms_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t version;
    uint8_t nPorts;
    bool mem;
    bool alloc;
    uint8_t protIds;

    gnss_ports_comms_t ports[32];

} gnss_ubx_mon_comms_t;

typedef struct {

    uint8_t version;
    bool GPSSup;
    bool glonassSup;
    bool beidouSup;
    bool galileoSup;
    bool GPSDef;
    bool glonassDef;
    bool beidouDef;
    bool galileoDef;
    bool GPSEna;
    bool glonassEna;
    bool beidouEna;
    bool galileoEna;
    uint8_t simultaneous;

} gnss_ubx_mon_gnss_t;

typedef struct {

    uint8_t pinId;
    bool periphPIO;
    uint8_t pinBank;
    bool direction;
    bool value;
    bool vpManager;
    bool pioIrq;
    bool pioPullHigh;
    bool pioPullLow;
    uint8_t vp;

} gnss_pins_hw3_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t version;
    uint8_t nPins;
    bool rtcCalib;
    bool safeBoot;
    bool xtalAbsent;
    char hwVersion[10];

    gnss_pins_hw3_t pins[32];

} gnss_ubx_mon_hw3_t;

typedef struct {

    bool activated;
    uint8_t location;
    uint32_t comparatorNumber;
    uint32_t patchAddress;
    uint32_t patchData;

} gnss_entries_patch_t;

typedef struct {

    uint16_t version;
    uint16_t nEntries;

    gnss_entries_patch_t entries[32];

} gnss_ubx_mon_patch_t;

typedef struct {

    uint8_t blockId;
    uint8_t jammingState;
    uint8_t antStatus;
    uint8_t antPower;
    uint32_t postStatus;
    uint16_t noisePerMS;
    uint16_t agcCnt;
    uint8_t cwSuppression;
    int8_t ofsI;
    uint8_t magI;
    int8_t ofsQ;
    uint8_t magQ;

} gnss_blocks_rf_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t version;
    uint8_t nBlocks;

    gnss_blocks_rf_t blocks[255];

} gnss_ubx_mon_rf_t;

typedef struct {

    uint8_t spectrum[256]; // (2 ^ -2 dB)
    uint32_t span;
    uint32_t res;
    uint32_t center;
    uint8_t pga;

} gnss_rf_blocks_span_t;

typedef struct {

    bool stale;
    bool periodic;

    uint8_t version;
    uint8_t numRfBlocks;

    gnss_rf_blocks_span_t rfBlocks[32];

} gnss_ubx_mon_span_t;

typedef struct {

    char extension[30];

} gnss_extension_ver_t;

typedef struct {

    char swVersion[30];
    char hwVersion[10];

    gnss_extension_ver_t ext[32];

} gnss_ubx_mon_ver_t;

typedef struct {

    uint8_t msgClass; // Class
    uint8_t msgID; // ID
    uint8_t length;
    char payload[128];

} gnss_info_t;

typedef struct {

    uint8_t version; // Message Version. 0: Request, 1: Response
    uint8_t layer; // Layer data was retrieved from. 0: RAM, 1: BBR, 2: Flash, 7: Default
    uint16_t position; // Number of config items skipped in the result set (same as request)
    uint16_t dataLength; // Length of Key ID and Value pair data
    uint8_t data[32]; // Key ID and Value pairs

} gnss_cfg_msg_t;

typedef struct gnss_info_node_t {

    gnss_info_t *data;
    struct gnss_info_node_t *next;

} gnss_info_node_t;

typedef struct {

    uint8_t msgClass; // Class
    uint8_t msgID; // ID
    uint16_t length;
    uint8_t payload[32];

    uint8_t retry;

} gnss_tx_msg_t;

typedef struct gnss_tx_msg_node_t {

    gnss_tx_msg_t *data;
    struct gnss_tx_msg_node_t *next;

} gnss_tx_msg_node_t;

// CFG-TP-PULSE_DEF
#define GNSS_PULSE_PERIOD 0
#define GNSS_PULSE_FREQ 1

// CFG-TP-PULSE_LENGTH_DEF
#define GNSS_PULSE_RATIO 0
#define GNSS_PULSE_LENGTH 1

// CFG-TP-TIMEGRID_TP1
#define GNSS_PULSE_UTC 0
#define GNSS_PULSE_GPS 1
#define GNSS_PULSE_GLO 2
#define GNSS_PULSE_BDS 3
#define GNSS_PULSE_GAL 4
#define GNSS_PULSE_NAVIC 5
#define GNSS_PULSE_LOCAL 15

typedef struct {

    bool pulseDef; // Determines whether time pulse is interpreted as a frequency or period
    bool lengthDef; // Determines whether time pulse length is interpreted in us or %
    int16_t antDelay; // Antenna Cable Delay in (ns)
    uint32_t period; // Time Pulse Period (us) (only if pulseDef is period)
    uint32_t periodLock; // Time Pulse Period when locked to GNSS time (us) (only if pulseDef is period and useLocked set)
    uint32_t freq; // Time Pulse Frequency (Hz) (only if pulseDef is freq)
    uint32_t freqLock; // Time Pulse Frequency when locked to GNSS time (Hz) (only if pulseDef is freq and useLocked set)
    uint32_t pulseLength; // Time Pulse Length (us) (only if lengthDef is length)
    uint32_t pulseLengthLock; // Time Pulse length when locked to GNSS time (us) (only if lengthDef is length and useLocked set)
    double duty; // Time Pulse Duty Cycle (%) (only if lengthDef is ratio)
    double dutyLock; // Time Pulse Duty Cycle when locked to GNSS time (%) (only if lengthDef is ratio and useLocked set)
    int32_t userDelay; // User-configurable Time Pulse Delay (ns)
    bool enable; // Enable Time Pulse
    bool syncGnss; // Sync time pulse to GNSS time (1) or local clock (0)
    bool useLocked; // Use locked parameters when possible
    bool alignTOW; // Align pulse to top of second (only if syncGnss set)
    bool polarity; // Falling edge at top of second (0) or rising edge at top of second (1)
    uint8_t timeGrid; // Time Grid to use (only if syncGnss set)

} gnss_pulse_cfg_t;

// CFG-PM-OPERATEMODE
#define GNSS_PM_FULL 0
#define GNSS_PM_PSMOO 1
#define GNSS_PM_PSMCT 2

typedef struct {

    uint8_t mode;
    uint32_t posUpdatePeriod; // Position Update Period for PSMOO - Time between successive position fixes (s). Must be >= 5 but less than seconds in a week. If 0, receiver will never retry a fix and wait for external events
    uint32_t acqPeriod; // Acquisition Period if previously failed to achieve a fix - Time before retry after failed position fix (s)
    uint32_t gridOffset; // Position Update Period Grid Offset Relative to GPS start of week (s). Not used in PSMCT
    uint16_t onTime; // Time to stay in Tracking State (s) - If set to 0, receiver will only briefly enter tracking state after acquisition. How long the receiver stays in Tracking before POT (PSMCT) or Inactive for Update (PSMOO)
    uint8_t minAcqTime; // Minimum time to spend in Acquisition State (s) - Minimum time to spend in acquisition even if signals are insufficient
    uint8_t maxAcqTime; // Maximum time to spend in Acquisition State (s)
    bool doNotEnterOff; // Enable to prevent receiver from entering Inactive State after failing to achieve a fix
    bool waitTimeFix; // Disable to wait for normal fix OK before starting ONTIME, Enable for time fix
    bool updateEph; // Update ephemeris regularly (wakeup)
    // Pay close attention to the EXTINT pin levels, the integration manual says the pullup is disabled when EXTINT is enabled but it often isn't
    uint8_t extIntSel; // EXTINT pin select (if multiple exist on package), otherwise leave blank
    bool extIntWake; // EXTINT Pin Control (Wake) - Awake as long as EXTINT is HIGH
    bool extIntBackup; // EXTINT Pin Control (Backup) - Force BACKUP mode when EXTINT is LOW
    bool extIntInactive; // EXTINT Pin Control (Inactive) - Force backup if EXTINT is inactive for longer than extIntInactivity
    uint32_t extIntInactivity; // Inactivity timeout on ESTINT pin if enabled (ms)
    bool limitPeakCurr; // Limit Peak Current

} gnss_psm_cfg_t;

typedef struct {

    bool enable; // Enable Data Batching - Requires maxEntries to be set
    bool pioEnable; // Enable PIO Notification when buffer fill level exceeds warnThresh
    uint16_t maxEntries; // Maximum Entries in Buffer (num epochs) - Will be rejected if exceeds available memory
    uint16_t warnThresh; // Buffer fill level that triggers PIO notification
    bool pioActiveLow; // Polarity for PIO, set for active low, otherwise active high
    uint8_t pioID; // ID of PIO for buffer fill level notification
    bool extraPVT; // Include additional PVT information in batch messages (see interface description)
    bool extraODO; // Include additional ODO information in batch messages (see interface description)

} gnss_batch_cfg_t;

typedef struct {

    bool gps_ena;
    bool gps_l1ca_ena;
    bool sbas_ena;
    bool sbas_l1ca_ena;
    bool gal_ena;
    bool gal_e1_ena;
    bool bds_ena;
    bool bds_b1_ena;
    bool bds_b1c_ena;
    bool qzss_ena;
    bool qzss_l1ca_ena;
    bool qzss_l1s_ena;
    bool glo_ena;
    bool glo_l1_ena;

} gnss_signal_cfg_t;

typedef struct {

    bool enable;
    bool psmPerWake; // Record only single position per PSM on/off mode wake-up period
    uint16_t minInterval; // Minimum time interval between regularly logged positions (s)
    uint16_t timeThreshold; // Log if time difference is greater than this threshold (s), 0 to ignore
    uint16_t speedThreshold; // Log if speed is greater than this threshold (m/s), 0 to ignore. Still conforms to minimum time interval
    uint32_t posThreshold; // Log if 3D position difference is greater than this threshold (m), 0 to ignore. Still conforms to minimum time interval

} gnss_log_cfg_t;

typedef struct {

    uint8_t confLvl; // Required confidence level (must be one of the options in the interface description: 0 - 5)
    bool usePio; // PIO combined fence state output
    bool pinPol; // PIO pin polarity
    uint8_t pin; // PIO pin number

    bool useFence1; // Use first geofence
    int32_t latFence1; // Latitude of the first geofence circle center (1e-7 deg)
    int32_t lonFence1; // Longitude of the first geofence circle center (1e-7 deg)
    uint32_t radFence1; // Radius of the first geofence circle (0.01m)

    bool useFence2; // Use second geofence
    int32_t latFence2; // Latitude of the second geofence circle center (1e-7 deg)
    int32_t lonFence2; // Longitude of the second geofence circle center (1e-7 deg)
    uint32_t radFence2; // Radius of the second geofence circle (0.01m)

    bool useFence3; // Use third geofence
    int32_t latFence3; // Latitude of the third geofence circle center (1e-7 deg)
    int32_t lonFence3; // Longitude of the third geofence circle center (1e-7 deg)
    uint32_t radFence3; // Radius of the third geofence circle (0.01m)

    bool useFence4; // Use fourth geofence
    int32_t latFence4; // Latitude of the fourth geofence circle center (1e-7 deg)
    int32_t lonFence4; // Longitude of the fourth geofence circle center (1e-7 deg)
    uint32_t radFence4; // Radius of the fourth geofence circle (0.01m)

} gnss_geofence_cfg_t;

#define MSG_BUFFER_COUNT 32 // Max messages to store in buffer

// Talker IDs for NMEA Messages
#define GNSS_TALKER_GPS 0x01
#define GNSS_TALKER_GLONASS 0x02
#define GNSS_TALKER_GNSS 0x03
#define GNSS_TALKER_GALILEO 0x04
#define GNSS_TALKER_BEIDOU 0x05
#define GNSS_TALKER_NAVIC 0x06
#define GNSS_TALKER_QZSS 0x07

typedef struct {

    void *bus; // Pointer to communication peripheral bus
    uint8_t busType; // Bus type (e.g. GNSS_I2C, GNSS_UART)
    uint8_t busAddr; // Address for I2C, SS for SPI, Not used for UART
    uint8_t pinRst; // Reset Pin

    uint8_t buffer[GNSS_BUFFER_SIZE]; // General Rx Buffer
    uint16_t buffLength; // Length of data in General Rx Buffer

    gnss_msg_t messages[MSG_BUFFER_COUNT]; // Waiting Messages
    uint8_t pending_messages; // Complete messages needed to be parsed
    uint8_t next_message; // Next message to be read
    uint8_t write_message; // Next message to be written to

    uint8_t mainTalker; // Main TalkerID for NMEA polling messages

    bool awake; // Set by UBX-MON-RXR

    bool backupCreation; // Set by UBX-UPD-SOS
    uint8_t restoreResponse; // Set by UBX-UPD-SOS

    gnss_tx_msg_node_t *txMsg;

    gnss_info_node_t *info;

    gnss_cfg_msg_t *cfgMsg;

    gnss_nmea_std_dtm_t *nmeaStdDtm;
    gnss_nmea_std_gbs_t *nmeaStdGbs;
    gnss_nmea_std_gga_t *nmeaStdGga;
    gnss_nmea_std_gll_t *nmeaStdGll;
    gnss_nmea_std_gns_t *nmeaStdGns;
    gnss_nmea_std_grs_t *nmeaStdGrs;
    gnss_nmea_std_gsa_t *nmeaStdGsa;
    gnss_nmea_std_gst_t *nmeaStdGst;
    gnss_nmea_std_gsv_t *nmeaStdGsv;
    gnss_nmea_std_rlm_t *nmeaStdRlm;
    gnss_nmea_std_rmc_t *nmeaStdRmc;
    gnss_nmea_std_vlw_t *nmeaStdVlw;
    gnss_nmea_std_vtg_t *nmeaStdVtg;
    gnss_nmea_std_zda_t *nmeaStdZda;

    gnss_nmea_pubx_pos_t *nmeaPubxPos;
    gnss_nmea_pubx_svstatus_t *nmeaPubxSvstatus;
    gnss_nmea_pubx_time_t *nmeaPubxTime;

    gnss_ubx_nav_aopstatus_t *ubxNavAopstatus;
    gnss_ubx_nav_clock_t *ubxNavClock;
    gnss_ubx_nav_cov_t *ubxNavCov;
    gnss_ubx_nav_dop_t *ubxNavDop;
    gnss_ubx_nav_eoe_t *ubxNavEoe;
    gnss_ubx_nav_odo_t *ubxNavOdo;
    gnss_ubx_nav_orb_t *ubxNavOrb;
    gnss_ubx_nav_pl_t *ubxNavPl;
    gnss_ubx_nav_posecef_t *ubxNavPosecef;
    gnss_ubx_nav_posllh_t *ubxNavPosllh;
    gnss_ubx_nav_pvt_t *ubxNavPvt;
    gnss_ubx_nav_sat_t *ubxNavSat;
    gnss_ubx_nav_sbas_t *ubxNavSbas;
    gnss_ubx_nav_sig_t *ubxNavSig;
    gnss_ubx_nav_slas_t *ubxNavSlas;
    gnss_ubx_nav_status_t *ubxNavStatus;
    gnss_ubx_nav_timebds_t *ubxNavTimebds;
    gnss_ubx_nav_timegal_t *ubxNavTimegal;
    gnss_ubx_nav_timeglo_t *ubxNavTimeglo;
    gnss_ubx_nav_timegps_t *ubxNavTimegps;
    gnss_ubx_nav_timels_t *ubxNavTimels;
    gnss_ubx_nav_timeqzss_t *ubxNavTimeqzss;
    gnss_ubx_nav_timeutc_t *ubxNavTimeutc;
    gnss_ubx_nav_velecef_t *ubxNavVelecef;
    gnss_ubx_nav_velned_t *ubxNavVelned;

    gnss_ubx_rxm_meas20_t *ubxRxmMeas20;
    gnss_ubx_rxm_meas50_t *ubxRxmMeas50;
    gnss_ubx_rxm_measc12_t *ubxRxmMeasc12;
    gnss_ubx_rxm_measd12_t *ubxRxmMeasd12;
    gnss_ubx_rxm_measx_t *ubxRxmMeasx;
    gnss_ubx_rxm_rlm_t *ubxRxmRlm;
    gnss_ubx_rxm_sfrbx_t *ubxRxmSfrbx;

    gnss_ubx_sec_uniqid_t *ubxSecUniqid;

    gnss_ubx_tim_tm2_t *ubxTimTm2;
    gnss_ubx_tim_tp_t *ubxTimTp;
    gnss_ubx_tim_vrfy_t *ubxTimVrfy;

    gnss_ubx_log_batch_t **ubxLogBatch;
    gnss_ubx_log_findtime_t *ubxLogFindtime;
    gnss_ubx_log_info_t *ubxLogInfo;

    uint16_t batchQueue;

    gnss_ubx_mon_batch_t *ubxMonBatch;
    gnss_ubx_mon_comms_t *ubxMonComms;
    gnss_ubx_mon_gnss_t *ubxMonGnss;
    gnss_ubx_mon_hw3_t *ubxMonHw3;
    gnss_ubx_mon_patch_t *ubxMonPatch;
    gnss_ubx_mon_rf_t *ubxMonRf;
    gnss_ubx_mon_span_t *ubxMonSpan;
    gnss_ubx_mon_ver_t *ubxMonVer;

} gnss_t; // Handler for GNSS-UBLOX


//////////////////////

/****************************************************************************
 * Executive Functions
 ****************************************************************************/

uint8_t gnss_init(gnss_t *, uint32_t, uint8_t = GNSS_TALKER_GNSS);
void gnss_reset_hw(gnss_t *);
uint8_t gnss_reset_sw(gnss_t *, uint16_t, uint8_t);
void* allocate_msg(void **, size_t);
void free_all_msg(gnss_t *);

/****************************************************************************
 * Configuration Modifications
 ****************************************************************************/

uint8_t gnss_ubx_cfg_clear(gnss_t *, uint8_t);
uint8_t gnss_ubx_cfg_save(gnss_t *, uint8_t);
uint8_t gnss_ubx_cfg_load(gnss_t *);
uint8_t gnss_cfg_del(gnss_t *, uint8_t, uint32_t);
uint8_t gnss_cfg_set(gnss_t *, uint8_t, uint32_t, uint8_t *, uint8_t, bool = 1);
uint8_t gnss_cfg_get(gnss_t *, gnss_cfg_msg_t *, uint32_t, uint8_t);
uint8_t gnss_pubx_set_config(gnss_t *, uint8_t, uint16_t, uint16_t, uint32_t);
uint8_t gnss_pubx_set_msg_rate(gnss_t *, uint16_t, bool, bool, bool, bool, bool);

/****************************************************************************
 * Configuration Specifics
 ****************************************************************************/

uint8_t gnss_set_msg_auto(gnss_t *, uint16_t, uint8_t, uint8_t);
uint8_t gnss_set_nav_rate(gnss_t *, uint16_t, uint16_t, uint8_t);
uint8_t gnss_enable_rdy(gnss_t *, uint8_t, bool, uint8_t, uint8_t, uint8_t);
uint8_t gnss_set_batch(gnss_t *, gnss_batch_cfg_t *, uint8_t);
uint8_t gnss_set_pulse(gnss_t *, gnss_pulse_cfg_t *, uint8_t);
uint8_t gnss_set_psm(gnss_t *, gnss_psm_cfg_t *, uint8_t);
uint8_t gnss_set_signals(gnss_t *, gnss_signal_cfg_t *);
uint8_t gnss_set_dynamic_model(gnss_t *, uint8_t);
uint8_t gnss_set_odo_model(gnss_t *, uint8_t);
uint8_t gnss_set_static_hold(gnss_t *, uint8_t, uint16_t);
uint8_t gnss_set_uart_baud(gnss_t *, uint32_t);
uint8_t gnss_set_logging(gnss_t *, gnss_log_cfg_t *);
uint8_t gnss_set_geofencing(gnss_t *, gnss_geofence_cfg_t *, uint8_t);

/****************************************************************************
 * Messaging
 ****************************************************************************/

uint8_t gnss_rx(gnss_t *);

/****************************************************************************
 * Messaging Operations
 ****************************************************************************/

uint8_t gnss_parse_messages(gnss_t *);
uint8_t gnss_rec_and_parse(gnss_t *);

/****************************************************************************
 * Generic Commands
 ****************************************************************************/

uint8_t gnss_reset_odo(gnss_t *);
uint8_t gnss_retrieve_batch(gnss_t *, gnss_ubx_mon_batch_t *, gnss_ubx_log_batch_t ***);
uint8_t gnss_pm_req(gnss_t *, uint32_t, bool, bool, bool, bool, bool, bool);
uint8_t gnss_create_backup(gnss_t *);
uint8_t gnss_clear_backup(gnss_t *);
uint8_t gnss_get_backup_status(gnss_t *);

uint8_t gnss_create_log(gnss_t *, bool, uint8_t, uint32_t);
uint8_t gnss_erase_log(gnss_t *, bool, uint8_t, uint32_t);
uint8_t gnss_find_log_time(gnss_t *, gnss_ubx_log_findtime_t *, uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t gnss_get_log_time(gnss_t *, gnss_ubx_log_info_t *);
uint8_t gnss_retrieve_log(gnss_t *, uint32_t, uint16_t);

/****************************************************************************
 * Message Retrieval
 ****************************************************************************/

uint8_t gnss_get_msg_info(gnss_t *, gnss_info_t *);
uint8_t gnss_get_dtm(gnss_t *, gnss_nmea_std_dtm_t *);
uint8_t gnss_get_gbs(gnss_t *, gnss_nmea_std_gbs_t *);
uint8_t gnss_get_gga(gnss_t *, gnss_nmea_std_gga_t *);
uint8_t gnss_get_gll(gnss_t *, gnss_nmea_std_gll_t *);
uint8_t gnss_get_gns(gnss_t *, gnss_nmea_std_gns_t *);
uint8_t gnss_get_grs(gnss_t *, gnss_nmea_std_grs_t *);
uint8_t gnss_get_gsa(gnss_t *, gnss_nmea_std_gsa_t *);
uint8_t gnss_get_gst(gnss_t *, gnss_nmea_std_gst_t *);
uint8_t gnss_get_gsv(gnss_t *, gnss_nmea_std_gsv_t *);
uint8_t gnss_get_rmc(gnss_t *, gnss_nmea_std_rmc_t *);
uint8_t gnss_get_vlw(gnss_t *, gnss_nmea_std_vlw_t *);
uint8_t gnss_get_vtg(gnss_t *, gnss_nmea_std_vtg_t *);
uint8_t gnss_get_zda(gnss_t *, gnss_nmea_std_zda_t *);
uint8_t gnss_get_position(gnss_t *, gnss_nmea_pubx_pos_t *);
uint8_t gnss_get_svstatus(gnss_t *, gnss_nmea_pubx_svstatus_t *);
uint8_t gnss_get_time(gnss_t *, gnss_nmea_pubx_time_t *);
uint8_t gnss_get_aopstatus(gnss_t *, gnss_ubx_nav_aopstatus_t *);
uint8_t gnss_get_clock(gnss_t *, gnss_ubx_nav_clock_t *);
uint8_t gnss_get_cov(gnss_t *, gnss_ubx_nav_cov_t *);
uint8_t gnss_get_dop(gnss_t *, gnss_ubx_nav_dop_t *);
uint8_t gnss_get_eoe(gnss_t *, gnss_ubx_nav_eoe_t *);
uint8_t gnss_get_odo(gnss_t *, gnss_ubx_nav_odo_t *);
uint8_t gnss_get_orb(gnss_t *, gnss_ubx_nav_orb_t *);
uint8_t gnss_get_pl(gnss_t *, gnss_ubx_nav_pl_t *);
uint8_t gnss_get_posecef(gnss_t *, gnss_ubx_nav_posecef_t *);
uint8_t gnss_get_posllh(gnss_t *, gnss_ubx_nav_posllh_t *);
uint8_t gnss_get_pvt(gnss_t *, gnss_ubx_nav_pvt_t *);
uint8_t gnss_get_sat(gnss_t *, gnss_ubx_nav_sat_t *);
uint8_t gnss_get_sbas(gnss_t *, gnss_ubx_nav_sbas_t *);
uint8_t gnss_get_sig(gnss_t *, gnss_ubx_nav_sig_t *);
uint8_t gnss_get_slas(gnss_t *, gnss_ubx_nav_slas_t *);
uint8_t gnss_get_status(gnss_t *, gnss_ubx_nav_status_t *);
uint8_t gnss_get_timebds(gnss_t *, gnss_ubx_nav_timebds_t *);
uint8_t gnss_get_timegal(gnss_t *, gnss_ubx_nav_timegal_t *);
uint8_t gnss_get_timeglo(gnss_t *, gnss_ubx_nav_timeglo_t *);
uint8_t gnss_get_timegps(gnss_t *, gnss_ubx_nav_timegps_t *);
uint8_t gnss_get_timels(gnss_t *, gnss_ubx_nav_timels_t *);
uint8_t gnss_get_timeqzss(gnss_t *, gnss_ubx_nav_timeqzss_t *);
uint8_t gnss_get_timeutc(gnss_t *, gnss_ubx_nav_timeutc_t *);
uint8_t gnss_get_velecef(gnss_t *, gnss_ubx_nav_velecef_t *);
uint8_t gnss_get_velned(gnss_t *, gnss_ubx_nav_velned_t *);
uint8_t gnss_get_meas20(gnss_t *, gnss_ubx_rxm_meas20_t *);
uint8_t gnss_get_meas50(gnss_t *, gnss_ubx_rxm_meas50_t *);
uint8_t gnss_get_measc12(gnss_t *, gnss_ubx_rxm_measc12_t *);
uint8_t gnss_get_measd12(gnss_t *, gnss_ubx_rxm_measd12_t *);
uint8_t gnss_get_measx(gnss_t *, gnss_ubx_rxm_measx_t *);
uint8_t gnss_get_rlm(gnss_t *, gnss_ubx_rxm_rlm_t *);
uint8_t gnss_get_sfrbx(gnss_t *, gnss_ubx_rxm_sfrbx_t *);
uint8_t gnss_get_tm2(gnss_t *, gnss_ubx_tim_tm2_t *);
uint8_t gnss_get_tp(gnss_t *, gnss_ubx_tim_tp_t *);
uint8_t gnss_get_vrfy(gnss_t *, gnss_ubx_tim_vrfy_t *);
uint8_t gnss_get_comms(gnss_t *, gnss_ubx_mon_comms_t *);
uint8_t gnss_get_gnss(gnss_t *, gnss_ubx_mon_gnss_t *);
uint8_t gnss_get_hw3(gnss_t *, gnss_ubx_mon_hw3_t *);
uint8_t gnss_get_patch(gnss_t *, gnss_ubx_mon_patch_t *);
uint8_t gnss_get_rf(gnss_t *, gnss_ubx_mon_rf_t *);
uint8_t gnss_get_span(gnss_t *, gnss_ubx_mon_span_t *);
uint8_t gnss_get_version(gnss_t *, gnss_ubx_mon_ver_t *);

#endif
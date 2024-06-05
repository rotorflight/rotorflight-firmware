/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "types.h"
#include "platform.h"

#include "pg/pg.h"

typedef enum {
    GPS_LATITUDE,
    GPS_LONGITUDE,
} gpsCoordinateType_e;

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MSP,
} gpsProvider_e;

typedef enum {
    SBAS_AUTO = 0,
    SBAS_EGNOS,
    SBAS_WAAS,
    SBAS_MSAS,
    SBAS_GAGAN,
    SBAS_NONE,
} sbasMode_e;

#define SBAS_MODE_MAX SBAS_GAGAN

typedef enum {
    UBLOX_AIRBORNE = 0,
    UBLOX_PEDESTRIAN,
    UBLOX_DYNAMIC,
} ubloxMode_e;

typedef enum {
    GPS_BAUDRATE_115200 = 0,
    GPS_BAUDRATE_57600,
    GPS_BAUDRATE_38400,
    GPS_BAUDRATE_19200,
    GPS_BAUDRATE_9600,
} gpsBaudRate_e;

typedef enum {
    GPS_AUTOCONFIG_OFF = 0,
    GPS_AUTOCONFIG_ON,
} gpsAutoConfig_e;

typedef enum {
    GPS_AUTOBAUD_OFF = 0,
    GPS_AUTOBAUD_ON,
} gpsAutoBaud_e;

typedef enum {
    UBLOX_ACK_IDLE = 0,
    UBLOX_ACK_WAITING,
    UBLOX_ACK_GOT_ACK,
    UBLOX_ACK_GOT_NACK,
} ubloxAckState_e;

#define GPS_BAUDRATE_MAX GPS_BAUDRATE_9600

typedef struct {
    uint8_t     provider;
    uint8_t     sbasMode;
    uint8_t     autoConfig;
    uint8_t     autoBaud;
    uint8_t     gps_ublox_use_galileo;
    uint8_t     gps_ublox_mode;
    uint8_t     gps_set_home_point_once;
    uint8_t     gps_use_3d_speed;
    uint8_t     sbas_integrity;
} gpsConfig_t;

PG_DECLARE(gpsConfig_t, gpsConfig);

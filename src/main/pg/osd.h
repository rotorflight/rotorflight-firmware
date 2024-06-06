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

#ifdef USE_OSD_PROFILES
#define OSD_PROFILE_COUNT           3
#else
#define OSD_PROFILE_COUNT           1
#endif

#define OSD_PROFILE_NAME_LENGTH     6
#define OSD_RCCHANNELS_COUNT        4
#define OSD_TIMER_COUNT             2
#define OSD_ITEM_COUNT              61

#ifdef USE_OSD

typedef struct osdConfig_s {
    uint16_t    cap_alarm;
    uint16_t    alt_alarm;
    uint8_t     rssi_alarm;
    uint8_t     units;
    uint16_t    timers[OSD_TIMER_COUNT];
    uint32_t    enabledWarnings;
    uint8_t     ahMaxPitch;
    uint8_t     ahMaxRoll;
    uint32_t    enabled_stats;
    int8_t      esc_temp_alarm;
    int16_t     esc_rpm_alarm;
    int16_t     esc_current_alarm;
    uint8_t     core_temp_alarm;
    uint8_t     ahInvert;                           // invert the artificial horizon
    uint8_t     osdProfileIndex;
    uint8_t     overlay_radio_mode;
    char        profile[OSD_PROFILE_COUNT][OSD_PROFILE_NAME_LENGTH + 1];
    uint16_t    link_quality_alarm;
    int16_t     rssi_dbm_alarm;
    uint8_t     gps_sats_show_hdop;
    int8_t      rcChannels[OSD_RCCHANNELS_COUNT];   // RC channel values to display, -1 if none
    uint8_t     displayPortDevice;                  // osdDisplayPortDevice_e
    uint16_t    distance_alarm;
    uint8_t     logo_on_arming;                     // show the logo on arming
    uint8_t     logo_on_arming_duration;            // display duration in 0.1s units
    uint8_t     camera_frame_width;                 // The width of the box for the camera frame element
    uint8_t     camera_frame_height;                // The height of the box for the camera frame element
    uint16_t    framerate_hz;
    uint8_t     cms_background_type;                // For supporting devices, determines whether the CMS background is transparent or opaque
    uint8_t     stat_show_cell_value;
} osdConfig_t;

PG_DECLARE(osdConfig_t, osdConfig);

typedef struct {
    uint16_t item_pos[OSD_ITEM_COUNT];
} osdElementConfig_t;

PG_DECLARE(osdElementConfig_t, osdElementConfig);

#endif

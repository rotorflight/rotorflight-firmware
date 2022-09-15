/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"

#ifndef DEFAULT_FEATURES
#define DEFAULT_FEATURES 0
#endif
#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE 0
#endif

typedef enum {
    FEATURE_RX_PPM                 = BIT(0),
    FEATURE_UNUSED_1               = BIT(1),
    FEATURE_UNUSED_2               = BIT(2),
    FEATURE_RX_SERIAL              = BIT(3),
    FEATURE_UNUSED_4               = BIT(4),
    FEATURE_UNUSED_5               = BIT(5),
    FEATURE_SOFTSERIAL             = BIT(6),
    FEATURE_GPS                    = BIT(7),
    FEATURE_UNUSED_8               = BIT(8),
    FEATURE_RANGEFINDER            = BIT(9),
    FEATURE_TELEMETRY              = BIT(10),
    FEATURE_UNUSED_11              = BIT(11),
    FEATURE_UNUSED_12              = BIT(12),
    FEATURE_RX_PARALLEL_PWM        = BIT(13),
    FEATURE_RX_MSP                 = BIT(14),
    FEATURE_RSSI_ADC               = BIT(15),
    FEATURE_LED_STRIP              = BIT(16),
    FEATURE_DASHBOARD              = BIT(17),
    FEATURE_OSD                    = BIT(18),
    FEATURE_UNUSED_19              = BIT(19),
    FEATURE_UNUSED_20              = BIT(20),
    FEATURE_UNUSED_21              = BIT(21),
    FEATURE_UNUSED_22              = BIT(22),
    FEATURE_UNUSED_23              = BIT(23),
    FEATURE_UNUSED_24              = BIT(24),
    FEATURE_RX_SPI                 = BIT(25),
    FEATURE_GOVERNOR               = BIT(26),
    FEATURE_ESC_SENSOR             = BIT(27),
    FEATURE_FREQ_SENSOR            = BIT(28),
    FEATURE_UNUSED_29              = BIT(29),
    FEATURE_RPM_FILTER             = BIT(30),
} features_e;

#define UNUSED_FEATURES ( \
    FEATURE_UNUSED_1  | \
    FEATURE_UNUSED_2  | \
    FEATURE_UNUSED_4  | \
    FEATURE_UNUSED_5  | \
    FEATURE_UNUSED_8  | \
    FEATURE_UNUSED_11 | \
    FEATURE_UNUSED_12 | \
    FEATURE_UNUSED_19 | \
    FEATURE_UNUSED_20 | \
    FEATURE_UNUSED_21 | \
    FEATURE_UNUSED_22 | \
    FEATURE_UNUSED_23 | \
    FEATURE_UNUSED_24 | \
    FEATURE_UNUSED_29 | \
    0)

typedef struct featureConfig_s {
    uint32_t enabledFeatures;
} featureConfig_t;

PG_DECLARE(featureConfig_t, featureConfig);

void featureInit(void);
bool featureIsEnabled(const uint32_t mask);
bool featureIsConfigured(const uint32_t mask);
void featureEnableImmediate(const uint32_t mask);
void featureDisableImmediate(const uint32_t mask);
void featureConfigSet(const uint32_t mask);
void featureConfigClear(const uint32_t mask);
void featureConfigReplace(const uint32_t mask);

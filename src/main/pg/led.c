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

#include "types.h"
#include "platform.h"

#include "config/config.h"

#include "pg/pg_ids.h"
#include "pg/led.h"

#ifndef LED0_PIN
#define LED0_PIN NONE
#endif

#ifndef LED1_PIN
#define LED1_PIN NONE
#endif

#ifndef LED2_PIN
#define LED2_PIN NONE
#endif

void pgResetFn_statusLedConfig(statusLedConfig_t *statusLedConfig)
{
    statusLedConfig->ioTags[0] = IO_TAG(LED0_PIN);
    statusLedConfig->ioTags[1] = IO_TAG(LED1_PIN);
    statusLedConfig->ioTags[2] = IO_TAG(LED2_PIN);

    statusLedConfig->inversion = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
    ;
}

PG_REGISTER_WITH_RESET_FN(statusLedConfig_t, statusLedConfig, PG_STATUS_LED_CONFIG, 0);


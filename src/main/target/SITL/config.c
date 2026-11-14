/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that they will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "config/config.h"
#include "config/feature.h"
#include "io/serial.h"
#include "pg/motor.h"

/*
 * SITL configuration defaults. Like every default, they apply on a config
 * reset only: an existing eeprom.bin keeps what it stored.
 *
 * - A second MSP port on UART2 (TCP 5762), so the Configurator can stay
 *   connected while a simulator bridge or a test drives UART1 (5761). The
 *   TCP UARTs take one client each.
 * - Rotor speed from the FDM packet: the frequency sensor is on and the main
 *   motor has 2 poles, so FDM motor_erpm[0] reads 1:1 as motor rpm and the
 *   governor and the RPM filter have a source out of the box. A model that
 *   sends real electrical rpm sets motor_poles to the motor's pole count.
 */
void targetConfiguration(void)
{
    serialPortConfig_t *uart2 = serialFindPortConfigurationMutable(SERIAL_PORT_USART2);
    if (uart2) {
        uart2->functionMask = FUNCTION_MSP;
    }

    featureConfigSet(FEATURE_FREQ_SENSOR);
    motorConfigMutable()->motorPoleCount[0] = 2;
}

#endif

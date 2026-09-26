/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "config/config.h"
#include "config_helper.h"
#include "io/serial.h"
#include "pg/battery.h"

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    {SERIAL_PORT_USB_VCP, FUNCTION_MSP},
};

static void forceUsbVcpAsMsp(void)
{
    serialConfig_t *serialConfig = serialConfigMutable();

    for (int index = 0; index < SERIAL_PORT_COUNT; index++)
    {
        if (serialConfig->portConfigs[index].identifier == SERIAL_PORT_USB_VCP)
        {
            serialConfig->portConfigs[index].functionMask |= FUNCTION_MSP;
        }
        else
        {
            serialConfig->portConfigs[index].functionMask &= ~FUNCTION_MSP;
        }
    }
}

void targetConfiguration(void)
{
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
    forceUsbVcpAsMsp();

    // Enable battery monitoring. The batteryConfig defaults set
    // voltageMeterSource/currentMeterSource to *_NONE, which makes the FC
    // report "no battery". This board has VBAT on PC3 and a current sensor on
    // PC2, both on ADC1.
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;
}

void targetValidateConfiguration(void)
{
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
    forceUsbVcpAsMsp();

    // Force ADC as the battery/current source. targetConfiguration() already
    // sets this, but it runs BEFORE any saved config is loaded, so a config
    // saved on a previous flash (where voltageMeterSource = NONE) would
    // override it. Re-applying here guarantees VBAT is read from PC3/PC2 on
    // every boot, regardless of what is stored in flash.
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;
}

#endif

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

/*
 * Frequency sensor for the SITL, replacing drivers/freq.c (which measures
 * pulses on timer inputs). The rotor speed comes from the FDM packet's
 * motor_erpm[] instead (see updateState() in target.c), so the governor and
 * the RPM filter have the fast RPM source they would have on a board.
 */

#include <math.h>

#include "platform.h"

#if defined(USE_FREQ_SENSOR)

#include "drivers/freq.h"
#include "pg/freq.h"

static volatile float freqHz[FREQ_SENSOR_PORT_COUNT];
static bool freqInitialised;

void freqInit(const freqConfig_t *freqConfig)
{
    UNUSED(freqConfig);
    freqInitialised = true;
}

void freqUpdate(void)
{
}

void fakeFreqSet(uint8_t port, float hz)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        freqHz[port] = hz > 0 ? hz : 0;
    }
}

float getFreqSensorFreq(uint8_t port)
{
    return (port < FREQ_SENSOR_PORT_COUNT) ? freqHz[port] : 0;
}

uint32_t getFreqSensorRPM(uint8_t port)
{
    return lrintf(getFreqSensorFreq(port) * 60);
}

bool isFreqSensorPortInitialized(uint8_t port)
{
    return freqInitialised && port < FREQ_SENSOR_PORT_COUNT;
}

bool isFreqSensorInitialized(void)
{
    return freqInitialised;
}

#endif

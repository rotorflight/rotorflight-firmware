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
#include "drivers/time.h"
#include "pg/freq.h"

// Written by the UDP thread (fakeFreqSet), read by the firmware tasks; always
// through simStoreFloat()/simLoadFloat()
static float freqHz[FREQ_SENSOR_PORT_COUNT];
static bool freqInitialised;

// Simulated signal: one edge per period, pin high for the first half
static float freqPhase[FREQ_SENSOR_PORT_COUNT];
static uint16_t freqEdges[FREQ_SENSOR_PORT_COUNT];
static timeUs_t freqLastUpdateUs;

void freqInit(const freqConfig_t *freqConfig)
{
    UNUSED(freqConfig);
    freqInitialised = true;
}

void freqUpdate(void)
{
    const timeUs_t now = micros();
    const float dt = freqLastUpdateUs ? (now - freqLastUpdateUs) * 1e-6f : 0;
    freqLastUpdateUs = now;

    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        const float hz = simLoadFloat(&freqHz[port]);
        freqPhase[port] += hz * dt;
        const float periods = floorf(freqPhase[port]);
        freqEdges[port] += (uint16_t)periods;   // wraps like the hardware counter
        freqPhase[port] -= periods;
    }
}

void fakeFreqSet(uint8_t port, float hz)
{
    // Up to the real sensor's FREQ_INPUT_MAXHZ_DEFAULT; NaN, infinity and
    // anything above read as a stopped shaft
    if (port < FREQ_SENSOR_PORT_COUNT) {
        const bool valid = simIsFinite(hz) && hz > 0 && hz <= FREQ_INPUT_MAXHZ_DEFAULT;
        simStoreFloat(&freqHz[port], valid ? hz : 0);
    }
}

float getFreqSensorFreq(uint8_t port)
{
    return (port < FREQ_SENSOR_PORT_COUNT) ? simLoadFloat(&freqHz[port]) : 0;
}

uint32_t getFreqSensorRPM(uint8_t port)
{
    return lrintf(getFreqSensorFreq(port) * 60);
}

uint16_t getFreqSensorEdgeCount(uint8_t port)
{
    return (port < FREQ_SENSOR_PORT_COUNT) ? freqEdges[port] : 0;
}

bool getFreqSensorPinState(uint8_t port)
{
    return freqInitialised && port < FREQ_SENSOR_PORT_COUNT && freqPhase[port] < 0.5f;
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

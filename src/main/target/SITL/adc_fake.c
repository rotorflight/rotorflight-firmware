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
 * Board ADC for the SITL, replacing drivers/adc.c and pg/adc.c (STM32 ADC
 * instances, DMA and pins). The voltages come from the FDM packet's optional
 * tail (see fdm_packet in target.h): pack voltage and current, BEC and bus
 * rails, MCU supply and temperature.
 *
 * adcGetChannel() returns the raw 12-bit sample that the configured scale
 * would turn into the simulated value, so the firmware's own conversion,
 * filtering and battery logic run unchanged and the scale settings behave
 * as on a board.
 */

#include <math.h>

#include "platform.h"

#if defined(USE_ADC)

#include "common/maths.h"

#include "drivers/adc.h"

#include "pg/adc.h"
#include "pg/pg_ids.h"
#include "pg/current.h"
#include "pg/voltage.h"

#include "sensors/adcinternal.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

#include "target/SITL/target.h"

PG_REGISTER_WITH_RESET_FN(adcConfig_t, adcConfig, PG_ADC_CONFIG, 0);

void pgResetFn_adcConfig(adcConfig_t *config)
{
    config->vbat.enabled = true;
    config->current.enabled = true;
    config->vbec.enabled = true;
    config->vbus.enabled = true;
    config->device = ADC_DEV_TO_CFG(ADCDEV_1);
}

// Simulated inputs, written by the UDP thread and read by the firmware tasks,
// always through simStoreFloat()/simLoadFloat(). A board with nothing
// connected reads 0.
static float simVbat;           // V
static float simIbat;           // A
static float simVbec;           // V
static float simVbus;           // V
static float simMcuVref = 3.3f; // V
static float simMcuTemp = 25.0f;    // degC

// adcInternalReadVrefint() reports millivolts in a uint16_t
#define SIM_MCU_VREF_MAX_MV 65535

static bool channelEnabled[ADC_CHANNEL_COUNT];

void adcInit(const adcConfig_t *config)
{
    channelEnabled[ADC_BATTERY] = config->vbat.enabled;
    channelEnabled[ADC_CURRENT] = config->current.enabled;
    channelEnabled[ADC_RSSI] = config->rssi.enabled;
    channelEnabled[ADC_VBEC] = config->vbec.enabled;
    channelEnabled[ADC_VBUS] = config->vbus.enabled;
    channelEnabled[ADC_VEXT] = config->vext.enabled;
}

bool adcIsEnabled(uint8_t channel)
{
    return channel < ADC_CHANNEL_COUNT && channelEnabled[channel];
}

static uint16_t rawSample(float value)
{
    return (uint16_t)constrainf(lrintf(value), 0, 4095);
}

// Inverse of voltageSensorADCtoVoltage() (sensors/voltage.c), in mV.
static uint16_t voltageSample(voltageSensorADC_e sensor, float volts)
{
    const voltageSensorADCConfig_t *config = voltageSensorADCConfig(sensor);
    const float fullScale = config->scale * getVrefMv();
    if (fullScale <= 0) {
        return 0;
    }
    const float divisor = fmaxf(config->divider * config->divmul, 1) * 4095;
    return rawSample(volts * 1000 * divisor / fullScale);
}

// Inverse of currentSensorADCToCurrent() (sensors/current.c), in mA.
static uint16_t currentSample(float amps)
{
    const currentSensorADCConfig_t *config = currentSensorADCConfig(CURRENT_SENSOR_ADC_BAT);
    if (!config->scale) {
        return 0;
    }
    const float microvolts = (amps * 1000 - config->offset) * config->scale;
    return rawSample(microvolts * 0.4095f / getVrefMv());
}

uint16_t adcGetChannel(uint8_t channel)
{
    if (!adcIsEnabled(channel)) {
        return 0;
    }
    switch (channel) {
    case ADC_BATTERY:
        return voltageSample(VOLTAGE_SENSOR_ADC_BAT, simLoadFloat(&simVbat));
    case ADC_CURRENT:
        return currentSample(simLoadFloat(&simIbat));
    case ADC_VBEC:
        return voltageSample(VOLTAGE_SENSOR_ADC_BEC, simLoadFloat(&simVbec));
    case ADC_VBUS:
        return voltageSample(VOLTAGE_SENSOR_ADC_BUS, simLoadFloat(&simVbus));
    default:
        return 0;
    }
}

#if defined(USE_ADC_INTERNAL)
// The internal channels carry the values themselves: millivolts, and
// degrees Celsius offset by 1000 to stay unsigned.
bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
}

uint16_t adcInternalReadVrefint(void)
{
    return (uint16_t)lrintf(simLoadFloat(&simMcuVref) * 1000);
}

uint16_t adcInternalReadTempsensor(void)
{
    return (uint16_t)lrintf(simLoadFloat(&simMcuTemp) + 1000);
}

uint16_t adcInternalCompensateVref(uint16_t vrefAdcValue)
{
    return vrefAdcValue;
}

int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue)
{
    UNUSED(vrefValue);
    return (int16_t)tempAdcValue - 1000;
}
#endif

void simAdcUpdate(const fdm_packet *pkt, int size)
{
    // An out-of-range Vref keeps the previous one: Vref scales every other ADC reading
    if (size >= FDM_PACKET_SIZE_MCU && simIsFinite(pkt->mcu_vref) && simIsFinite(pkt->mcu_temp) &&
        pkt->mcu_vref > 0 && pkt->mcu_vref * 1000 <= SIM_MCU_VREF_MAX_MV) {
        simStoreFloat(&simMcuVref, pkt->mcu_vref);
        simStoreFloat(&simMcuTemp, pkt->mcu_temp);
    }
    if (size >= FDM_PACKET_SIZE_RAILS && simIsFinite(pkt->vbec) && simIsFinite(pkt->vbus)) {
        simStoreFloat(&simVbec, pkt->vbec);
        simStoreFloat(&simVbus, pkt->vbus);
    }
    if (size >= FDM_PACKET_SIZE_PACK && simIsFinite(pkt->vbat) && simIsFinite(pkt->ibat)) {
        simStoreFloat(&simVbat, pkt->vbat);
        simStoreFloat(&simIbat, pkt->ibat);
    }
}

#else

void simAdcUpdate(const fdm_packet *pkt, int size)
{
    UNUSED(pkt);
    UNUSED(size);
}

#endif

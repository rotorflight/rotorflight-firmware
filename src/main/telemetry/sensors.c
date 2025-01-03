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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/pilot.h"
#include "pg/telemetry.h"

#include "build/debug.h"

#include "common/crc.h"

#include "sensors/battery.h"
#include "sensors/voltage.h"
#include "sensors/current.h"
#include "sensors/esc_sensor.h"
#include "sensors/adcinternal.h"
#include "sensors/acceleration.h"

#include "flight/position.h"
#include "flight/governor.h"
#include "flight/rescue.h"
#include "flight/mixer.h"
#include "flight/imu.h"

#include "io/gps.h"
#include "io/ledstrip.h"

#include "fc/rc_modes.h"
#include "fc/rc_adjustments.h"
#include "fc/runtime_config.h"

#include "scheduler/scheduler.h"

#include "telemetry/sensors.h"


/** Sensor configuration **/

telemetrySensorConfig_t telemetrySensorConfig =
{
    .batVoltageScale        = 10,
    .batCurrentScale        = 10,
    .escVoltageScale        = 10,
    .escCurrentScale        = 10,
    .becVoltageScale        = 10,
    .becCurrentScale        = 10,
    .escTempScale           = 10,
    .attitudeScale          = 100,
    .accelScale             = 100,
};


/** Sensor functions **/

static int getVoltage(voltageMeterId_e id)
{
    voltageMeter_t voltage;
    return voltageMeterRead(id, &voltage) ? voltage.voltage / telemetrySensorConfig.batVoltageScale : 0;
}

static int getCurrent(currentMeterId_e id)
{
    currentMeter_t current;
    return currentMeterRead(id, &current) ? current.current / telemetrySensorConfig.batCurrentScale : 0;
}

static int getEscSensorValue(uint8_t motor, uint8_t id)
{
    escSensorData_t * data = getEscSensorData(motor);

    if (data) {
        switch (id) {
            case 1:
                return data->voltage / telemetrySensorConfig.escVoltageScale;
            case 2:
                return data->current / telemetrySensorConfig.escCurrentScale;
            case 3:
                return data->consumption;
            case 4:
                return data->erpm;
            case 5:
                return data->pwm;
            case 6:
                return data->throttle;
            case 7:
                return data->temperature / telemetrySensorConfig.escTempScale;
            case 8:
                return data->temperature2 / telemetrySensorConfig.escTempScale;
            case 9:
                return data->bec_voltage / telemetrySensorConfig.becVoltageScale;
            case 10:
                return data->bec_current / telemetrySensorConfig.becCurrentScale;
            case 11:
                return data->status;
            case 12:
                return data->id;
        }
    }

    return 0;
}

static uint32_t getTupleHash(uint32_t a, uint32_t b)
{
    uint32_t data[2] = { a, b };
    return fnv_update(0x42424242, data, sizeof(data));
}


int telemetrySensorValue(sensor_id_e id)
{
    switch (id) {
        case TELEM_NONE:
            return 0;

        case TELEM_HEARTBEAT:
            return millis() % 60000;

        case TELEM_BATTERY:
            return millis();
        case TELEM_BATTERY_VOLTAGE:
            return getBatteryVoltage();
        case TELEM_BATTERY_CURRENT:
            return getBatteryCurrent();
        case TELEM_BATTERY_CONSUMPTION:
            return getBatteryCapacityUsed();
        case TELEM_BATTERY_CHARGE_LEVEL:
            return calculateBatteryPercentageRemaining();
        case TELEM_BATTERY_CELL_COUNT:
            return getBatteryCellCount();
        case TELEM_BATTERY_CELL_VOLTAGE:
            return getBatteryAverageCellVoltage();
        case TELEM_BATTERY_CELL_VOLTAGES:
            return 0;

        case TELEM_CONTROL:
            return millis();
        case TELEM_PITCH_CONTROL:
            return lrintf(mixerGetInput(MIXER_IN_STABILIZED_PITCH) * 120);
        case TELEM_ROLL_CONTROL:
            return lrintf(mixerGetInput(MIXER_IN_STABILIZED_ROLL) * 120);
        case TELEM_YAW_CONTROL:
            return lrintf(mixerGetInput(MIXER_IN_STABILIZED_YAW) * 240);
        case TELEM_COLLECTIVE_CONTROL:
            return lrintf(mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE) * 120);
        case TELEM_THROTTLE_CONTROL:
            return lrintf(mixerGetInput(MIXER_IN_STABILIZED_THROTTLE) * 100);

        case TELEM_ESC1_DATA:
        case TELEM_ESC1_VOLTAGE:
        case TELEM_ESC1_CURRENT:
        case TELEM_ESC1_CAPACITY:
        case TELEM_ESC1_ERPM:
        case TELEM_ESC1_POWER:
        case TELEM_ESC1_THROTTLE:
        case TELEM_ESC1_TEMP1:
        case TELEM_ESC1_TEMP2:
        case TELEM_ESC1_BEC_VOLTAGE:
        case TELEM_ESC1_BEC_CURRENT:
        case TELEM_ESC1_STATUS:
        case TELEM_ESC1_MODEL:
            return getEscSensorValue(0, id - TELEM_ESC1_DATA);

        case TELEM_ESC2_DATA:
        case TELEM_ESC2_VOLTAGE:
        case TELEM_ESC2_CURRENT:
        case TELEM_ESC2_CAPACITY:
        case TELEM_ESC2_ERPM:
        case TELEM_ESC2_POWER:
        case TELEM_ESC2_THROTTLE:
        case TELEM_ESC2_TEMP1:
        case TELEM_ESC2_TEMP2:
        case TELEM_ESC2_BEC_VOLTAGE:
        case TELEM_ESC2_BEC_CURRENT:
        case TELEM_ESC2_STATUS:
        case TELEM_ESC2_MODEL:
            return getEscSensorValue(1, id - TELEM_ESC2_DATA);

        case TELEM_ESC_VOLTAGE:
            return getVoltage(VOLTAGE_METER_ID_ESC_COMBINED);
        case TELEM_BEC_VOLTAGE:
            return getVoltage(VOLTAGE_METER_ID_BEC);
        case TELEM_BUS_VOLTAGE:
            return getVoltage(VOLTAGE_METER_ID_BUS);
        case TELEM_MCU_VOLTAGE:
            return getVoltage(VOLTAGE_METER_ID_MCU);

        case TELEM_ESC_CURRENT:
            return getCurrent(CURRENT_METER_ID_ESC_COMBINED);
        case TELEM_BEC_CURRENT:
            return getCurrent(CURRENT_METER_ID_BEC);
        case TELEM_BUS_CURRENT:
            return getCurrent(CURRENT_METER_ID_BUS);
        case TELEM_MCU_CURRENT:
            return getCurrent(CURRENT_METER_ID_MCU);

        case TELEM_MCU_TEMP:
            return getCoreTemperatureCelsius();

        case TELEM_ESC_TEMP:
            return getEscSensorValue(ESC_SENSOR_COMBINED, 7);
        case TELEM_BEC_TEMP:
            return getEscSensorValue(ESC_SENSOR_COMBINED, 8);
        case TELEM_AIR_TEMP:
        case TELEM_MOTOR_TEMP:
        case TELEM_BATTERY_TEMP:
        case TELEM_EXHAUST_TEMP:
            return 0;

        case TELEM_HEADING:
            return attitude.values.yaw;
        case TELEM_ALTITUDE:
            return getEstimatedAltitudeCm();
        case TELEM_VARIOMETER:
            return getEstimatedVarioCms();

        case TELEM_HEADSPEED:
            return getHeadSpeed();
        case TELEM_TAILSPEED:
            return getTailSpeed();

        case TELEM_MOTOR_RPM:
        case TELEM_TRANS_RPM:
            return 0;

        case TELEM_ATTITUDE:
            return millis();
        case TELEM_ATTITUDE_PITCH:
            return 10 * (int)attitude.values.pitch / telemetrySensorConfig.attitudeScale;
        case TELEM_ATTITUDE_ROLL:
            return 10 * (int)attitude.values.roll / telemetrySensorConfig.attitudeScale;
        case TELEM_ATTITUDE_YAW:
            return 10 * (int)attitude.values.yaw / telemetrySensorConfig.attitudeScale;

        case TELEM_ACCEL:
            return millis();
        case TELEM_ACCEL_X:
            return lrintf(1000 * acc.accADC[0] * acc.dev.acc_1G_rec / telemetrySensorConfig.accelScale);
        case TELEM_ACCEL_Y:
            return lrintf(1000 * acc.accADC[1] * acc.dev.acc_1G_rec / telemetrySensorConfig.accelScale);
        case TELEM_ACCEL_Z:
            return lrintf(1000 * acc.accADC[2] * acc.dev.acc_1G_rec / telemetrySensorConfig.accelScale);

        case TELEM_GPS:
            return millis();
        case TELEM_GPS_SATS:
            return gpsSol.numSat;
        case TELEM_GPS_PDOP:
            return 0;
        case TELEM_GPS_HDOP:
            return gpsSol.hdop;
        case TELEM_GPS_VDOP:
            return 0;
        case TELEM_GPS_COORD:
            return getTupleHash(gpsSol.llh.lat, gpsSol.llh.lon);
        case TELEM_GPS_ALTITUDE:
            return gpsSol.llh.altCm;
        case TELEM_GPS_HEADING:
            return gpsSol.groundCourse;
        case TELEM_GPS_GROUNDSPEED:
            return gpsSol.groundSpeed;
        case TELEM_GPS_HOME_DISTANCE:
            return GPS_distanceToHome;
        case TELEM_GPS_HOME_DIRECTION:
            return GPS_directionToHome;
        case TELEM_GPS_DATE_TIME:
            return 0;

        case TELEM_LOAD:
            return millis();
        case TELEM_CPU_LOAD:
            return getAverageCPULoadPercent();
        case TELEM_SYS_LOAD:
            return getAverageSystemLoadPercent();
        case TELEM_RT_LOAD:
            return getMaxRealTimeLoadPercent();

        case TELEM_MODEL_ID:
            return pilotConfig()->modelId;
        case TELEM_FLIGHT_MODE:
            return flightModeFlags;
        case TELEM_ARMING_FLAGS:
            return armingFlags;
        case TELEM_ARMING_DISABLE_FLAGS:
            return getArmingDisableFlags();
        case TELEM_RESCUE_STATE:
            return getRescueState();
        case TELEM_GOVERNOR_STATE:
            return getGovernorState();
        case TELEM_GOVERNOR_FLAGS:
            return 0;

        case TELEM_PID_PROFILE:
            return getCurrentPidProfileIndex() + 1;
        case TELEM_RATES_PROFILE:
            return getCurrentControlRateProfileIndex() + 1;
        case TELEM_LED_PROFILE:
        case TELEM_BATTERY_PROFILE:
            return 0;

        case TELEM_ADJFUNC:
            return getAdjustmentsRangeName() ?
                getTupleHash(getAdjustmentsRangeFunc(), getAdjustmentsRangeValue()) : 0;

        case TELEM_DEBUG_0:
            return debug[0];
        case TELEM_DEBUG_1:
            return debug[1];
        case TELEM_DEBUG_2:
            return debug[2];
        case TELEM_DEBUG_3:
            return debug[3];
        case TELEM_DEBUG_4:
            return debug[4];
        case TELEM_DEBUG_5:
            return debug[5];
        case TELEM_DEBUG_6:
            return debug[6];
        case TELEM_DEBUG_7:
            return debug[7];

        default:
            return 0;
    }

    return 0;
}


bool telemetrySensorActive(sensor_id_e id)
{
    switch (id) {
        case TELEM_NONE:
            return false;

        case TELEM_HEARTBEAT:
            return true;

        case TELEM_BATTERY:
        case TELEM_BATTERY_VOLTAGE:
        case TELEM_BATTERY_CURRENT:
        case TELEM_BATTERY_CONSUMPTION:
        case TELEM_BATTERY_CHARGE_LEVEL:
        case TELEM_BATTERY_CELL_COUNT:
        case TELEM_BATTERY_CELL_VOLTAGE:
            return true;

        case TELEM_BATTERY_CELL_VOLTAGES:
            return false;

        case TELEM_CONTROL:
        case TELEM_ROLL_CONTROL:
        case TELEM_PITCH_CONTROL:
        case TELEM_YAW_CONTROL:
        case TELEM_COLLECTIVE_CONTROL:
        case TELEM_THROTTLE_CONTROL:
            return true;

        case TELEM_ESC1_DATA:
        case TELEM_ESC1_VOLTAGE:
        case TELEM_ESC1_CURRENT:
        case TELEM_ESC1_CAPACITY:
        case TELEM_ESC1_ERPM:
        case TELEM_ESC1_POWER:
        case TELEM_ESC1_THROTTLE:
        case TELEM_ESC1_TEMP1:
        case TELEM_ESC1_TEMP2:
        case TELEM_ESC1_BEC_VOLTAGE:
        case TELEM_ESC1_BEC_CURRENT:
        case TELEM_ESC1_STATUS:
        case TELEM_ESC1_MODEL:
            return true;

        case TELEM_ESC2_DATA:
        case TELEM_ESC2_VOLTAGE:
        case TELEM_ESC2_CURRENT:
        case TELEM_ESC2_CAPACITY:
        case TELEM_ESC2_ERPM:
        case TELEM_ESC2_POWER:
        case TELEM_ESC2_THROTTLE:
        case TELEM_ESC2_TEMP1:
        case TELEM_ESC2_TEMP2:
        case TELEM_ESC2_BEC_VOLTAGE:
        case TELEM_ESC2_BEC_CURRENT:
        case TELEM_ESC2_STATUS:
        case TELEM_ESC2_MODEL:
            return true;

        case TELEM_ESC_VOLTAGE:
        case TELEM_BEC_VOLTAGE:
        case TELEM_BUS_VOLTAGE:
        case TELEM_MCU_VOLTAGE:
            return true;

        case TELEM_ESC_CURRENT:
        case TELEM_BEC_CURRENT:
        case TELEM_BUS_CURRENT:
        case TELEM_MCU_CURRENT:
            return true;

        case TELEM_MCU_TEMP:
        case TELEM_ESC_TEMP:
        case TELEM_BEC_TEMP:
            return true;
        case TELEM_AIR_TEMP:
        case TELEM_MOTOR_TEMP:
        case TELEM_BATTERY_TEMP:
        case TELEM_EXHAUST_TEMP:
            return false;

        case TELEM_HEADING:
        case TELEM_ALTITUDE:
        case TELEM_VARIOMETER:
            return true;

        case TELEM_HEADSPEED:
        case TELEM_TAILSPEED:
            return true;

        case TELEM_MOTOR_RPM:
        case TELEM_TRANS_RPM:
            return false;

        case TELEM_ATTITUDE:
        case TELEM_ATTITUDE_PITCH:
        case TELEM_ATTITUDE_ROLL:
        case TELEM_ATTITUDE_YAW:
            return true;

        case TELEM_ACCEL:
        case TELEM_ACCEL_X:
        case TELEM_ACCEL_Y:
        case TELEM_ACCEL_Z:
            return true;

        case TELEM_GPS:
        case TELEM_GPS_SATS:
        case TELEM_GPS_HDOP:
        case TELEM_GPS_COORD:
        case TELEM_GPS_ALTITUDE:
        case TELEM_GPS_HEADING:
        case TELEM_GPS_GROUNDSPEED:
        case TELEM_GPS_HOME_DISTANCE:
        case TELEM_GPS_HOME_DIRECTION:
            return true;

        case TELEM_GPS_PDOP:
        case TELEM_GPS_VDOP:
        case TELEM_GPS_DATE_TIME:
            return false;

        case TELEM_LOAD:
        case TELEM_CPU_LOAD:
        case TELEM_SYS_LOAD:
        case TELEM_RT_LOAD:
            return true;

        case TELEM_MODEL_ID:
        case TELEM_FLIGHT_MODE:
        case TELEM_ARMING_FLAGS:
        case TELEM_ARMING_DISABLE_FLAGS:
        case TELEM_RESCUE_STATE:
        case TELEM_GOVERNOR_STATE:
        case TELEM_GOVERNOR_FLAGS:
            return true;

        case TELEM_PID_PROFILE:
        case TELEM_RATES_PROFILE:
            return true;

        case TELEM_BATTERY_PROFILE:
        case TELEM_LED_PROFILE:
            return false;

        case TELEM_ADJFUNC:
            return true;

        case TELEM_DEBUG_0:
        case TELEM_DEBUG_1:
        case TELEM_DEBUG_2:
        case TELEM_DEBUG_3:
        case TELEM_DEBUG_4:
        case TELEM_DEBUG_5:
        case TELEM_DEBUG_6:
        case TELEM_DEBUG_7:
            return debugMode;

        default:
            return false;
    }

    return false;
}


/** Legacy sensors **/

static uint32_t telemetry_legacy_sensors = 0;

sensor_e telemetryGetLegacySensor(sensor_id_e sensor_id)
{
    switch (sensor_id)
    {
        case TELEM_BATTERY_VOLTAGE:
            return SENSOR_VOLTAGE;
        case TELEM_BATTERY_CURRENT:
            return SENSOR_CURRENT;
        case TELEM_BATTERY_CONSUMPTION:
            return SENSOR_CAP_USED;
        case TELEM_BATTERY_CHARGE_LEVEL:
            return SENSOR_FUEL;
        case TELEM_FLIGHT_MODE:
            return SENSOR_MODE;
        case TELEM_ACCEL_X:
            return SENSOR_ACC_X;
        case TELEM_ACCEL_Y:
            return SENSOR_ACC_Y;
        case TELEM_ACCEL_Z:
            return SENSOR_ACC_Z;
        case TELEM_ATTITUDE_PITCH:
            return SENSOR_PITCH;
        case TELEM_ATTITUDE_ROLL:
            return SENSOR_ROLL;
        case TELEM_ATTITUDE_YAW:
            return SENSOR_HEADING;
        case TELEM_ALTITUDE:
            return SENSOR_ALTITUDE;
        case TELEM_VARIOMETER:
            return SENSOR_VARIO;
        case TELEM_GPS_COORD:
            return SENSOR_LAT_LONG;
        case TELEM_GPS_GROUNDSPEED:
            return SENSOR_GROUND_SPEED;
        default:
            return 0;
    }
}

bool telemetryIsSensorEnabled(uint32_t sensor_bits)
{
    return (telemetry_legacy_sensors & sensor_bits);
}

void INIT_CODE legacySensorInit(void)
{
    telemetry_legacy_sensors = 0;

    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        sensor_id_e id = telemetryConfig()->telemetry_sensors[i];
        telemetry_legacy_sensors |= telemetryGetLegacySensor(id);
    }
}

#endif /* USE_TELEMETRY */

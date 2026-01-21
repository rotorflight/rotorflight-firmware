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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "common/sensor_alignment.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/castle_telemetry_decode.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/system.h"

#include "fc/rc_rates.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/dispatch.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"
#include "flight/motors.h"
#include "flight/servos.h"
#include "flight/position.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/vtx.h"

#include "msp/msp_box.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/displayport_profiles.h"
#include "pg/gyrodev.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/sdcard.h"
#include "pg/vtx_table.h"
#include "pg/freq.h"
#include "pg/system.h"
#include "pg/pilot.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "config.h"

#include "drivers/dshot.h"

static bool configIsDirty; /* someone indicated that the config is modified and it is not yet saved */

static bool rebootRequired = false;  // set if a config change requires a reboot to take effect

static bool eepromWriteInProgress = false;

pidProfile_t *currentPidProfile;

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

bool isEepromWriteInProgress(void)
{
    return eepromWriteInProgress;
}

uint8_t getCurrentPidProfileIndex(void)
{
    return systemConfig()->pidProfileIndex;
}

static void loadPidProfile(void)
{
    currentPidProfile = pidProfilesMutable(systemConfig()->pidProfileIndex);
}

uint8_t getCurrentControlRateProfileIndex(void)
{
    return systemConfig()->activeRateProfile;
}

uint16_t getCurrentMinthrottle(void)
{
    return motorConfig()->minthrottle;
}

void resetConfig(void)
{
    pgResetAll();

#if defined(USE_TARGET_CONFIG)
    targetConfiguration();
#endif
}

static void activateConfig(void)
{
    loadPidProfile();
    loadControlRateProfile();

    initRcProcessing();
    adjustmentRangeInit();

    pidChangeProfile(currentPidProfile);

    rcControlsInit();

    failsafeReset();
#ifdef USE_ACC
    setAccelerationTrims(&accelerometerConfigMutable()->accZero);
    accInitFilters();
#endif

    imuConfigure();

#if defined(USE_LED_STRIP_STATUS_MODE)
    reevaluateLedConfig();
#endif

    initActiveBoxIds();
}

static void adjustFilterLimit(uint16_t *parm, uint16_t maxValue, uint16_t resetValue)
{
    if (*parm > maxValue) {
        *parm = resetValue;
    }
}

static void validateAndFixRatesSettings(void)
{
    for (unsigned profileIndex = 0; profileIndex < CONTROL_RATE_PROFILE_COUNT; profileIndex++) {
        const ratesType_e ratesType = controlRateProfilesMutable(profileIndex)->rates_type;
        for (unsigned axis = FD_ROLL; axis <= FD_YAW; axis++) {
            controlRateProfilesMutable(profileIndex)->rcRates[axis] = constrain(controlRateProfilesMutable(profileIndex)->rcRates[axis], 0, ratesSettingLimits[ratesType].rc_rate_limit);
            controlRateProfilesMutable(profileIndex)->sRates[axis] = constrain(controlRateProfilesMutable(profileIndex)->sRates[axis], 0, ratesSettingLimits[ratesType].srate_limit);
            controlRateProfilesMutable(profileIndex)->rcExpo[axis] = constrain(controlRateProfilesMutable(profileIndex)->rcExpo[axis], 0, ratesSettingLimits[ratesType].expo_limit);
        }
    }
}

static void validateAndFixPositionConfig(void)
{

}

static void validateAndFixConfig(void)
{
    if (!isSerialConfigValid(serialConfig())) {
        pgResetFn_serialConfig(serialConfigMutable());
    }

#if defined(USE_GPS)
    const serialPortConfig_t *gpsSerial = findSerialPortConfig(FUNCTION_GPS);
    if (gpsConfig()->provider == GPS_MSP && gpsSerial) {
        serialRemovePort(gpsSerial->identifier);
    }
#endif
    if (
#if defined(USE_GPS)
        gpsConfig()->provider != GPS_MSP && !gpsSerial &&
#endif
        true) {
        featureDisableImmediate(FEATURE_GPS);
    }

    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_STANDARD ||
            motorConfig()->dev.motorPwmProtocol == PWM_TYPE_CASTLE_LINK) {
        if (!motorConfig()->dev.useUnsyncedPwm) {
            motorConfigMutable()->dev.useUnsyncedPwm = true;
        }
        if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_STANDARD) {
            if (motorConfig()->dev.motorPwmRate > MOTORS_MAX_PWM_RATE) {
                motorConfigMutable()->dev.motorPwmRate = MOTORS_MAX_PWM_RATE;
            }
        } else {
#ifdef USE_TELEMETRY_CASTLE
            if (motorConfig()->dev.motorPwmRate > CASTLE_PWM_HZ_MAX) {
                motorConfigMutable()->dev.motorPwmRate = CASTLE_PWM_HZ_MAX;
            }
#endif
        }
    }

    validateAndFixGyroConfig();

#if defined(USE_MAG)
    buildAlignmentFromStandardAlignment(&compassConfigMutable()->mag_customAlignment, compassConfig()->mag_alignment);
#endif
    buildAlignmentFromStandardAlignment(&gyroDeviceConfigMutable(0)->customAlignment, gyroDeviceConfig(0)->alignment);
#if defined(USE_MULTI_GYRO)
    buildAlignmentFromStandardAlignment(&gyroDeviceConfigMutable(1)->customAlignment, gyroDeviceConfig(1)->alignment);
#endif

#ifdef USE_ACC
    if (accelerometerConfig()->accZero.values.roll != 0 ||
        accelerometerConfig()->accZero.values.pitch != 0 ||
        accelerometerConfig()->accZero.values.yaw != 0) {
        accelerometerConfigMutable()->accZero.values.calibrationCompleted = 1;
    }
#endif // USE_ACC

    if (!(featureIsConfigured(FEATURE_RX_PARALLEL_PWM) || featureIsConfigured(FEATURE_RX_PPM) || featureIsConfigured(FEATURE_RX_SERIAL) || featureIsConfigured(FEATURE_RX_MSP) || featureIsConfigured(FEATURE_RX_SPI))) {
        featureEnableImmediate(DEFAULT_RX_FEATURE);
    }

    if (featureIsConfigured(FEATURE_RX_PPM)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_SPI);
    }

    if (featureIsConfigured(FEATURE_RX_MSP)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

    if (featureIsConfigured(FEATURE_RX_SERIAL)) {
        featureDisableImmediate(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

#ifdef USE_RX_SPI
    if (featureIsConfigured(FEATURE_RX_SPI)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_MSP);
    }
#endif // USE_RX_SPI

    if (featureIsConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

#if defined(USE_ADC)
    if (featureIsConfigured(FEATURE_RSSI_ADC)) {
        rxConfigMutable()->rssi_channel = 0;
        rxConfigMutable()->rssi_src_frame_errors = false;
    } else
#endif
    if (rxConfigMutable()->rssi_channel
#if defined(USE_PWM) || defined(USE_PPM)
        || featureIsConfigured(FEATURE_RX_PPM) || featureIsConfigured(FEATURE_RX_PARALLEL_PWM)
#endif
        ) {
        rxConfigMutable()->rssi_src_frame_errors = false;
    }

    if (rcControlsConfig()->rc_max_throttle && rcControlsConfig()->rc_min_throttle) {
        if (rcControlsConfig()->rc_max_throttle < rcControlsConfig()->rc_min_throttle + 10) {
            rcControlsConfigMutable()->rc_max_throttle = rcControlsConfig()->rc_min_throttle + 10;
        }
    }

    if (!featureIsConfigured(FEATURE_GPS)
#if !defined(USE_GPS) || !defined(USE_GPS_RESCUE)
        || true
#endif
        ) {

#ifdef USE_GPS_RESCUE
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;
        }
#endif

        if (isModeActivationConditionPresent(BOXGPSRESCUE)) {
            removeModeActivationCondition(BOXGPSRESCUE);
        }
    }

#if defined(USE_ESC_SENSOR)
    if (!findSerialPortConfig(FUNCTION_ESC_SENSOR) && !isMotorProtocolCastlePWM()) {
        featureDisableImmediate(FEATURE_ESC_SENSOR);
    }

    if (featureIsConfigured(FEATURE_ESC_SENSOR)) {
        validateAndFixEscSensorConfig();
    }
#endif

    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->linkedTo) {
            if (mac->modeId == BOXARM || isModeActivationConditionLinked(mac->linkedTo)) {
                removeModeActivationCondition(mac->modeId);
            }
        }
    }

#if defined(USE_DSHOT_TELEMETRY) && defined(USE_DSHOT_BITBANG)
    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_PROSHOT1000 && motorConfig()->dev.useDshotTelemetry &&
        motorConfig()->dev.useDshotBitbang == DSHOT_BITBANG_ON) {
        motorConfigMutable()->dev.useDshotBitbang = DSHOT_BITBANG_AUTO;
    }
#endif

#ifdef USE_ADC
    // Enabled if resource defined
    adcConfigMutable()->vbat.enabled = true;
    adcConfigMutable()->current.enabled = true;

    adcConfigMutable()->vbec.enabled = true;
    adcConfigMutable()->vbus.enabled = true;
    adcConfigMutable()->vext.enabled = true;

    adcConfigMutable()->rssi.enabled = featureIsEnabled(FEATURE_RSSI_ADC);

#ifdef USE_RX_SPI
    // The FrSky D SPI RX sends RSSI_ADC_PIN (if configured) as A2
    adcConfigMutable()->rssi.enabled |= (featureIsEnabled(FEATURE_RX_SPI) &&
        rxSpiConfig()->rx_spi_protocol == RX_SPI_FRSKY_D);

    // The FrSky D and X SPI RX sends EXT_ADC_PIN (if configured) as A1
    adcConfigMutable()->vext.enabled |= (featureIsEnabled(FEATURE_RX_SPI) && (
        rxSpiConfig()->rx_spi_protocol == RX_SPI_FRSKY_D ||
        rxSpiConfig()->rx_spi_protocol == RX_SPI_FRSKY_X ||
        rxSpiConfig()->rx_spi_protocol == RX_SPI_FRSKY_X_V2 ||
        rxSpiConfig()->rx_spi_protocol == RX_SPI_FRSKY_X_LBT ||
        rxSpiConfig()->rx_spi_protocol == RX_SPI_FRSKY_X_LBT_V2));
#endif
#endif // USE_ADC


// clear features that are not supported.
// I have kept them all here in one place, some could be moved to sections of code above.

    featureDisableImmediate(UNUSED_FEATURES);

#ifndef USE_PPM
    featureDisableImmediate(FEATURE_RX_PPM);
#endif

#ifndef USE_SERIAL_RX
    featureDisableImmediate(FEATURE_RX_SERIAL);
#endif

#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
    featureDisableImmediate(FEATURE_SOFTSERIAL);
#endif

#ifndef USE_RANGEFINDER
    featureDisableImmediate(FEATURE_RANGEFINDER);
#endif

#ifndef USE_TELEMETRY
    featureDisableImmediate(FEATURE_TELEMETRY);
#endif

#ifndef USE_PWM
    featureDisableImmediate(FEATURE_RX_PARALLEL_PWM);
#endif

#ifndef USE_RX_MSP
    featureDisableImmediate(FEATURE_RX_MSP);
#endif

#ifndef USE_LED_STRIP
    featureDisableImmediate(FEATURE_LED_STRIP);
#endif

#ifndef USE_DASHBOARD
    featureDisableImmediate(FEATURE_DASHBOARD);
#endif

#ifndef USE_OSD
    featureDisableImmediate(FEATURE_OSD);
#endif

#ifndef USE_RX_SPI
    featureDisableImmediate(FEATURE_RX_SPI);
#endif

#ifndef USE_ESC_SENSOR
    featureDisableImmediate(FEATURE_ESC_SENSOR);
#endif

#ifndef USE_FREQ_SENSOR
    featureDisableImmediate(FEATURE_FREQ_SENSOR);
#endif

#ifndef USE_DYN_NOTCH_FILTER
    featureDisableImmediate(FEATURE_DYN_NOTCH);
#endif

#ifndef USE_RPM_FILTER
    featureDisableImmediate(FEATURE_RPM_FILTER);
#endif

#if !defined(USE_ADC)
    featureDisableImmediate(FEATURE_RSSI_ADC);
#endif

#ifdef USE_RPM_FILTER
    validateAndFixRPMFilterConfig();
#endif

#if defined(USE_BEEPER)
#ifdef USE_TIMER
    if (beeperDevConfig()->frequency && !timerGetConfiguredByTag(beeperDevConfig()->ioTag)) {
        beeperDevConfigMutable()->frequency = 0;
    }
#endif

    if (beeperConfig()->beeper_off_flags & ~BEEPER_ALLOWED_MODES) {
        beeperConfigMutable()->beeper_off_flags = 0;
    }

#ifdef USE_DSHOT
    if (beeperConfig()->dshotBeaconOffFlags & ~DSHOT_BEACON_ALLOWED_MODES) {
        beeperConfigMutable()->dshotBeaconOffFlags = 0;
    }

    if (beeperConfig()->dshotBeaconTone < DSHOT_CMD_BEACON1
        || beeperConfig()->dshotBeaconTone > DSHOT_CMD_BEACON5) {
        beeperConfigMutable()->dshotBeaconTone = DSHOT_CMD_BEACON1;
    }
#endif
#endif

    bool configuredMotorProtocolDshot = checkMotorProtocolDshot(&motorConfig()->dev);
#if defined(USE_DSHOT)
    // If using DSHOT protocol disable unsynched PWM as it's meaningless
    if (configuredMotorProtocolDshot) {
        motorConfigMutable()->dev.useUnsyncedPwm = false;
    }

#if defined(USE_DSHOT_TELEMETRY)
    bool nChannelTimerUsed = false;
    for (unsigned i = 0; i < getMotorCount(); i++) {
        const ioTag_t tag = motorConfig()->dev.ioTags[i];
        if (tag) {
            const timerHardware_t *timer = timerGetConfiguredByTag(tag);
            if (timer && timer->output & TIMER_OUTPUT_N_CHANNEL) {
                nChannelTimerUsed = true;

                break;
            }
        }
    }

    if ((!configuredMotorProtocolDshot || (motorConfig()->dev.useDshotBitbang == DSHOT_BITBANG_OFF && (motorConfig()->dev.useBurstDshot == DSHOT_DMAR_ON || nChannelTimerUsed))) && motorConfig()->dev.useDshotTelemetry) {
        motorConfigMutable()->dev.useDshotTelemetry = false;
    }
#endif // USE_DSHOT_TELEMETRY
#endif // USE_DSHOT

#if defined(USE_OSD)
    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
         const uint16_t t = osdConfig()->timers[i];
         if (OSD_TIMER_SRC(t) >= OSD_TIMER_SRC_COUNT ||
                 OSD_TIMER_PRECISION(t) >= OSD_TIMER_PREC_COUNT) {
             osdConfigMutable()->timers[i] = osdTimerDefault[i];
         }
     }
#endif

#if defined(USE_VTX_COMMON) && defined(USE_VTX_TABLE)
    // reset vtx band, channel, power if outside range specified by vtxtable
    if (vtxSettingsConfig()->channel > vtxTableConfig()->channels) {
        vtxSettingsConfigMutable()->channel = 0;
        if (vtxSettingsConfig()->band > 0) {
            vtxSettingsConfigMutable()->freq = 0; // band/channel determined frequency can't be valid anymore
        }
    }
    if (vtxSettingsConfig()->band > vtxTableConfig()->bands) {
        vtxSettingsConfigMutable()->band = 0;
        vtxSettingsConfigMutable()->freq = 0; // band/channel determined frequency can't be valid anymore
    }
    if (vtxSettingsConfig()->power > vtxTableConfig()->powerLevels) {
        vtxSettingsConfigMutable()->power = 0;
    }
#endif

    validateAndFixRatesSettings();  // constrain the various rates settings to limits imposed by the rates type

    // validate that the minimum battery cell voltage is less than the maximum cell voltage
    // reset to defaults if not
    if (batteryConfig()->vbatmincellvoltage >=  batteryConfig()->vbatmaxcellvoltage) {
        batteryConfigMutable()->vbatmincellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MIN;
        batteryConfigMutable()->vbatmaxcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MAX;
    }

#ifdef USE_MSP_DISPLAYPORT
    // validate that displayport_msp_serial is referencing a valid UART that actually has MSP enabled
    if (displayPortProfileMsp()->displayPortSerial != SERIAL_PORT_NONE) {
        const serialPortConfig_t *portConfig = serialFindPortConfiguration(displayPortProfileMsp()->displayPortSerial);
        if (!portConfig || !(portConfig->functionMask & FUNCTION_MSP)
#ifndef USE_MSP_PUSH_OVER_VCP
            || (portConfig->identifier == SERIAL_PORT_USB_VCP)
#endif
            ) {
            displayPortProfileMspMutable()->displayPortSerial = SERIAL_PORT_NONE;
        }
    }
#endif

#if defined(TARGET_VALIDATECONFIG)
    // This should be done at the end of the validation
    targetValidateConfiguration();
#endif

    validateAndFixPositionConfig();
    validateAndFixServoConfig();
    validateAndFixMixerConfig();
    validateAndFixRxConfig();
}

void validateAndFixGyroConfig(void)
{
    if (gyro.sampleRateHz > 0)
    {
        uint32_t pidDenom = pidConfig()->pid_process_denom;
        uint32_t filtDenom = pidConfig()->filter_process_denom;

        // Loop rate restrictions based on motor protocol. Motor times have safety margin
        uint32_t motorUpdateRestriction = 0;

#if defined(STM32F411xE)
        /* If bidirectional DSHOT is being used on an F411 then force DSHOT300. The motor update restrictions then applied
         * will automatically consider the loop time and adjust pid_process_denom appropriately
         */
        if (motorConfig()->dev.useDshotTelemetry && (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_DSHOT600)) {
            motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT300;
        }
#endif

        switch (motorConfig()->dev.motorPwmProtocol) {
            case PWM_TYPE_STANDARD:
            case PWM_TYPE_CASTLE_LINK:
                motorUpdateRestriction = MOTORS_MAX_PWM_RATE;
                break;
            case PWM_TYPE_ONESHOT125:
                motorUpdateRestriction = 2000;
                break;
            case PWM_TYPE_ONESHOT42:
                motorUpdateRestriction = 10000;
                break;
            case PWM_TYPE_MULTISHOT:
                motorUpdateRestriction = 32000;
                break;
            case PWM_TYPE_DSHOT150:
                motorUpdateRestriction = 4000;
                break;
            case PWM_TYPE_DSHOT300:
                motorUpdateRestriction = 10000;
                break;
            case PWM_TYPE_DSHOT600:
            case PWM_TYPE_PROSHOT1000:
                motorUpdateRestriction = 32000;
                break;
        }

        if (motorUpdateRestriction) {
            if (motorConfig()->dev.useUnsyncedPwm) {
                if (!checkMotorProtocolDshot(&motorConfig()->dev)) {
                    motorConfigMutable()->dev.motorPwmRate = MIN(motorConfig()->dev.motorPwmRate, motorUpdateRestriction);
                }
            } else {
                if (motorConfig()->dev.useDshotTelemetry) {
                    motorUpdateRestriction /= 2;
                }
                while (gyro.sampleRateHz / pidDenom > motorUpdateRestriction && pidDenom < MAX_PID_PROCESS_DENOM) {
                    pidDenom++;
                }
            }
        }

        // Maximum PID loop speed
        while (gyro.sampleRateHz / pidDenom > MAX_PID_PROCESS_SPEED && pidDenom < MAX_PID_PROCESS_DENOM) {
            pidDenom++;
        }
        // Minimum PID loop speed
        while (gyro.sampleRateHz / pidDenom < MIN_PID_PROCESS_SPEED && pidDenom > 1) {
            pidDenom--;
        }

        // Save any changes
        pidConfigMutable()->pid_process_denom = pidDenom;

        // Check filter denom
        if (filtDenom > 0) {
            if (filtDenom < pidDenom) {
                while (pidDenom % filtDenom) filtDenom++;
            } else {
                filtDenom = pidDenom;
            }
            pidConfigMutable()->filter_process_denom = filtDenom;
        }
        else {
            filtDenom = pidDenom;
        }

        // Fix gyro filter limits
        uint16_t decimation_limit = lrintf(0.5f * gyro.sampleRateHz / pidDenom);
        uint16_t cutoff_limit = lrintf(0.45f * gyro.sampleRateHz / filtDenom);

        adjustFilterLimit(&gyroConfigMutable()->gyro_decimation_hz, decimation_limit, decimation_limit);
        adjustFilterLimit(&gyroConfigMutable()->gyro_lpf1_static_hz, cutoff_limit, cutoff_limit);
        adjustFilterLimit(&gyroConfigMutable()->gyro_lpf2_static_hz, cutoff_limit, cutoff_limit);
        adjustFilterLimit(&gyroConfigMutable()->gyro_soft_notch_hz_1, cutoff_limit, cutoff_limit);
        adjustFilterLimit(&gyroConfigMutable()->gyro_soft_notch_cutoff_1, cutoff_limit, 0);
        adjustFilterLimit(&gyroConfigMutable()->gyro_soft_notch_hz_2, cutoff_limit, cutoff_limit);
        adjustFilterLimit(&gyroConfigMutable()->gyro_soft_notch_cutoff_2, cutoff_limit, 0);

        if (gyroConfig()->gyro_lpf1_static_hz == 0) {
            gyroConfigMutable()->gyro_lpf1_type = LPF_NONE;
        }
        if (gyroConfig()->gyro_lpf2_static_hz == 0) {
            gyroConfigMutable()->gyro_lpf2_type = LPF_NONE;
        }

#ifdef USE_DYN_LPF
        // Prevent invalid dynamic lowpass filter
        if (gyroConfig()->gyro_lpf1_dyn_min_hz > gyroConfig()->gyro_lpf1_dyn_max_hz) {
            gyroConfigMutable()->gyro_lpf1_dyn_min_hz = 0;
            gyroConfigMutable()->gyro_lpf1_dyn_max_hz = 0;
        }
        else if (gyroConfig()->gyro_lpf1_dyn_min_hz > gyroConfig()->gyro_lpf1_static_hz) {
            gyroConfigMutable()->gyro_lpf1_dyn_min_hz = 0;
            gyroConfigMutable()->gyro_lpf1_dyn_max_hz = 0;
        }
        else if (gyroConfig()->gyro_lpf1_dyn_max_hz < gyroConfig()->gyro_lpf1_static_hz) {
            gyroConfigMutable()->gyro_lpf1_dyn_min_hz = 0;
            gyroConfigMutable()->gyro_lpf1_dyn_max_hz = 0;
        }
#endif

        // Prevent invalid notch cutoff
        if (gyroConfig()->gyro_soft_notch_cutoff_1 >= gyroConfig()->gyro_soft_notch_hz_1) {
            gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
        }
        if (gyroConfig()->gyro_soft_notch_cutoff_2 >= gyroConfig()->gyro_soft_notch_hz_2) {
            gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
        }
    }

#ifdef USE_BLACKBOX
#ifndef USE_FLASHFS
    if (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH) {
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
    }
#endif // USE_FLASHFS

    if (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD) {
#if defined(USE_SDCARD)
        if (!sdcardConfig()->mode)
#endif
        {
            blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
        }
    }
#endif // USE_BLACKBOX

    if (systemConfig()->activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = 0;
    }
    loadControlRateProfile();

    if (systemConfig()->pidProfileIndex >= PID_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = 0;
    }
    loadPidProfile();
}

bool readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    bool success = loadEEPROM();

    featureInit();

    validateAndFixConfig();

    activateConfig();

    resumeRxSignal();

    return success;
}

void writeUnmodifiedConfigToEEPROM(void)
{
    validateAndFixConfig();

    suspendRxSignal();
    eepromWriteInProgress = true;
    writeConfigToEEPROM();
    eepromWriteInProgress = false;
    resumeRxSignal();
    configIsDirty = false;
}

void writeEEPROM(void)
{
#ifdef USE_RX_SPI
    rxSpiStop(); // some rx spi protocols use hardware timer, which needs to be stopped before writing to eeprom
#endif
    systemConfigMutable()->configurationState = CONFIGURATION_STATE_CONFIGURED;

    writeUnmodifiedConfigToEEPROM();
}

void dispatchConfigWrite(struct dispatchEntry_s* self)
{
    UNUSED(self);

    writeUnmodifiedConfigToEEPROM();
    beeperConfirmationBeeps(1);
}

dispatchEntry_t writeConfigEntry =
{
    dispatchConfigWrite, 0, NULL, false
};

void writeEEPROMDelayed(int delayUs)
{
    dispatchAdd(&writeConfigEntry, delayUs);
}

bool resetEEPROM(bool useCustomDefaults)
{
#if !defined(USE_CUSTOM_DEFAULTS)
    UNUSED(useCustomDefaults);
#else
    if (useCustomDefaults) {
        if (!resetConfigToCustomDefaults()) {
            return false;
        }
    } else
#endif
    {
        resetConfig();
    }

    writeUnmodifiedConfigToEEPROM();

    return true;
}

void saveConfigAndNotify(void)
{
    // The write to EEPROM will cause a big delay in the current task, so ignore
    schedulerIgnoreTaskExecTime();

    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void setConfigDirty(void)
{
    configIsDirty = true;
}

bool isConfigDirty(void)
{
    return configIsDirty;
}

void changePidProfile(uint8_t pidProfileIndex)
{
    // The config switch will cause a big enough delay in the current task to upset the scheduler
    schedulerIgnoreTaskExecTime();

    if (pidProfileIndex < PID_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = pidProfileIndex;
        loadPidProfile();
        pidChangeProfile(currentPidProfile);
    }

    beeperConfirmationBeeps(pidProfileIndex + 1);
}

bool isSystemConfigured(void)
{
    return systemConfig()->configurationState == CONFIGURATION_STATE_CONFIGURED;
}

void setRebootRequired(void)
{
    rebootRequired = true;
    setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED);
}

bool getRebootRequired(void)
{
    return rebootRequired;
}

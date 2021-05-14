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
#include "blackbox/blackbox_fielddefs.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/freq.h"

#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"

#if defined(USE_GYRO_DATA_ANALYSE)
#include "flight/gyroanalyse.h"
#endif

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"
#include "flight/trainer.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/motors.h"
#include "io/pidaudio.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/statusindicator.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#include "core.h"


enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
};

#define GYRO_WATCHDOG_DELAY 80 //  delay for gyro sync

static FAST_RAM_ZERO_INIT uint8_t pidUpdateCounter;


static timeUs_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

static int lastArmingDisabledReason = 0;
static timeUs_t lastDisarmTimeUs;
static int tryingToArm = ARMING_DELAYED_DISARMED;

static bool isCalibrating(void)
{
    return (sensors(SENSOR_GYRO) && !gyroIsCalibrationComplete())
#ifdef USE_ACC
        || (sensors(SENSOR_ACC) && !accIsCalibrationComplete())
#endif
#ifdef USE_BARO
        || (sensors(SENSOR_BARO) && !baroIsCalibrationComplete())
#endif
#ifdef USE_MAG
        || (sensors(SENSOR_MAG) && !compassIsCalibrationComplete())
#endif
        ;
}

void resetArmingDisabled(void)
{
    lastArmingDisabledReason = 0;
}

#ifdef USE_ACC
static bool accNeedsCalibration(void)
{
    if (sensors(SENSOR_ACC)) {

        // Check to see if the ACC has already been calibrated
        if (accHasBeenCalibrated()) {
            return false;
        }

        // We've determined that there's a detected ACC that has not
        // yet been calibrated. Check to see if anything is using the
        // ACC that would be affected by the lack of calibration.

        // Check for any configured modes that use the ACC
        if (isModeActivationConditionPresent(BOXANGLE) ||
            isModeActivationConditionPresent(BOXHORIZON) ||
            isModeActivationConditionPresent(BOXRESCUE) ||
            isModeActivationConditionPresent(BOXGPSRESCUE) ||
            isModeActivationConditionPresent(BOXCAMSTAB) ||
            isModeActivationConditionPresent(BOXCALIB) ||
            isModeActivationConditionPresent(BOXACROTRAINER)) {

            return true;
        }

#ifdef USE_OSD
        // Check for any enabled OSD elements that need the ACC
        if (featureIsEnabled(FEATURE_OSD)) {
            if (osdNeedsAccelerometer()) {
                return true;
            }
        }
#endif

#ifdef USE_GPS_RESCUE
        // Check if failsafe will use GPS Rescue
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            return true;
        }
#endif
    }

    return false;
}
#endif

void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        // Check if the power on arming grace time has elapsed
        if ((getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME) && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000)
#ifdef USE_DSHOT
            // We also need to prevent arming until it's possible to send DSHOT commands.
            && (!isMotorProtocolDshot() || dshotCommandsAreEnabled(DSHOT_CMD_TYPE_INLINE))
#endif
        ) {
            // If so, unset the grace time arming disable flag
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }

        // If switch is used for arming then check it is not defaulting to on when the RX link recovers from a fault
        if (!isUsingSticksForArming()) {
            static bool hadRx = false;
            const bool haveRx = rxIsReceivingSignal();

            const bool justGotRxBack = !hadRx && haveRx;

            if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
                // If the RX has just started to receive a signal again and the arm switch is on, apply arming restriction
                setArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
                // If RX signal is OK and the arm switch is off, remove arming restriction
                unsetArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            }

            hadRx = haveRx;
        }

        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        }

        if (calculateThrottleStatus() != THROTTLE_LOW) {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_THROTTLE);
        }

        if (!isUpright()) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ANGLE);
        }

        if (getAverageSystemLoadPercent() > LOAD_PERCENTAGE_ONE) {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_LOAD);
        }

        if (isCalibrating()) {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_CALIBRATING);
        }

        if (isModeActivationConditionPresent(BOXPREARM)) {
            if (IS_RC_MODE_ACTIVE(BOXPREARM) && !ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
                unsetArmingDisabled(ARMING_DISABLED_NOPREARM);
            } else {
                setArmingDisabled(ARMING_DISABLED_NOPREARM);
            }
        }

#ifdef USE_GPS_RESCUE
        if (gpsRescueIsConfigured()) {
            if (gpsRescueConfig()->allowArmingWithoutFix || STATE(GPS_FIX) || ARMING_FLAG(WAS_EVER_ARMED)) {
                unsetArmingDisabled(ARMING_DISABLED_GPS);
            } else {
                setArmingDisabled(ARMING_DISABLED_GPS);
            }
            if (IS_RC_MODE_ACTIVE(BOXGPSRESCUE)) {
                setArmingDisabled(ARMING_DISABLED_RESC);
            } else {
                unsetArmingDisabled(ARMING_DISABLED_RESC);
            }
        }
#endif

#ifdef USE_RPM_FILTER
        if (featureIsEnabled(FEATURE_RPM_FILTER) && !isRpmSourceActive()) {
            setArmingDisabled(ARMING_DISABLED_RPMFILTER);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_RPMFILTER);
        }
#endif

#ifdef USE_DSHOT_BITBANG
        if (isDshotBitbangActive(&motorConfig()->dev) && dshotBitbangGetStatus() != DSHOT_BITBANG_STATUS_OK) {
            setArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        }
#endif

        if (IS_RC_MODE_ACTIVE(BOXPARALYZE)) {
            setArmingDisabled(ARMING_DISABLED_PARALYZE);
        }

#ifdef USE_ACC
        if (accNeedsCalibration()) {
            setArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        }
#endif

        if (!isMotorProtocolEnabled()) {
            setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }

        if (!isUsingSticksForArming()) {
            if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                unsetArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
            }

            /* Ignore ARMING_DISABLED_CALIBRATING if we are going to calibrate gyro on first arm */
            bool ignoreGyro = armingConfig()->gyro_cal_on_first_arm
                && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_CALIBRATING));

            // If arming is disabled and the ARM switch is on
            if (isArmingDisabled()
                && !ignoreGyro
                && IS_RC_MODE_ACTIVE(BOXARM)) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            } else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            }
        }

        if (isArmingDisabled()) {
            warningLedFlash();
        } else {
            warningLedDisable();
        }

        warningLedUpdate();
    }
}

void disarm(flightLogDisarmReason_e reason)
{
    if (ARMING_FLAG(ARMED)) {
        ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
        DISABLE_ARMING_FLAG(ARMED);
        lastDisarmTimeUs = micros();

#ifdef USE_BLACKBOX
        flightLogEvent_disarm_t eventData;
        eventData.reason = reason;
        blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, (flightLogEventData_t*)&eventData);

        if (blackboxConfig()->device && blackboxConfig()->mode != BLACKBOX_MODE_ALWAYS_ON) { // Close the log upon disarm except when logging mode is ALWAYS ON
            blackboxFinish();
        }
#else
        UNUSED(reason);
#endif
        BEEP_OFF;

#ifdef USE_PERSISTENT_STATS
        statsOnDisarm();
#endif

        // if ARMING_DISABLED_CRASH_DETECTED is set then we want to play it's beep pattern instead
        if (!(getArmingDisableFlags() & ARMING_DISABLED_CRASH_DETECTED)) {
            beeper(BEEPER_DISARMING);      // emit disarm tone
        }
    }
}

void tryArm(void)
{
    if (armingConfig()->gyro_cal_on_first_arm) {
        gyroStartCalibration(true);
    }

    updateArmingStatus();

    if (!isArmingDisabled()) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }

        const timeUs_t currentTimeUs = micros();

#ifdef USE_DSHOT
        if (currentTimeUs - getLastDshotBeaconCommandTimeUs() < DSHOT_BEACON_GUARD_DELAY_US) {
            if (tryingToArm == ARMING_DELAYED_DISARMED) {
                tryingToArm = ARMING_DELAYED_NORMAL;
            }
            return;
        }
#endif

#ifdef USE_OSD
        osdSuppressStats(false);
#endif
        ENABLE_ARMING_FLAG(ARMED);

        resetTryingToArm();

#ifdef USE_ACRO_TRAINER
        acroTrainerReset();
#endif

        if (isModeActivationConditionPresent(BOXPREARM)) {
            ENABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
        }

#if defined(USE_GYRO_DATA_ANALYSE)
        resetMaxFFT();
#endif

        disarmAt = currentTimeUs + armingConfig()->auto_disarm_delay * 1e6;   // start disarm timeout, will be extended when throttle is nonzero

        lastArmingDisabledReason = 0;

#ifdef USE_GPS
        GPS_reset_home_position();

        //beep to indicate arming
        if (featureIsEnabled(FEATURE_GPS)) {
            if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
                beeper(BEEPER_ARMING_GPS_FIX);
            } else {
                beeper(BEEPER_ARMING_GPS_NO_FIX);
            }
        } else {
            beeper(BEEPER_ARMING);
        }
#else
        beeper(BEEPER_ARMING);
#endif

#ifdef USE_PERSISTENT_STATS
        statsOnArm();
#endif
    } else {
       resetTryingToArm();
        if (!isFirstArmingGyroCalibrationRunning()) {
            int armingDisabledReason = ffs(getArmingDisableFlags());
            if (lastArmingDisabledReason != armingDisabledReason) {
                lastArmingDisabledReason = armingDisabledReason;

                beeperWarningBeeps(armingDisabledReason);
            }
        }
    }
}

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void handleInflightCalibrationStickPosition(void)
{
    if (AccInflightCalibrationMeasurementDone) {
        // trigger saving into eeprom after landing
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if (AccInflightCalibrationArmed) {
            beeper(BEEPER_ACC_CALIBRATION);
        } else {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
        }
    }
}

static void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcData[THROTTLE] > rxConfig()->mincheck && !IS_RC_MODE_ACTIVE(BOXARM)) {   // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (IS_RC_MODE_ACTIVE(BOXCALIB)) {      // Use the Calib Option to activate : Calib = TRUE measurement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
            InflightcalibratingA = 50;
        AccInflightCalibrationActive = true;
    } else if (AccInflightCalibrationMeasurementDone && !ARMING_FLAG(ARMED)) {
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    }
}

#ifdef USE_VTX_CONTROL
static bool canUpdateVTX(void)
{
#ifdef USE_VTX_RTC6705
    return vtxRTC6705CanUpdate();
#endif
    return true;
}
#endif

#if defined(USE_GPS_RESCUE)
// determine if the R/P/Y stick deflection exceeds the specified limit - integer math is good enough here.
bool areSticksActive(uint8_t stickPercentLimit)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis ++) {
        const uint8_t deadband = axis == FD_YAW ? rcControlsConfig()->yaw_deadband : rcControlsConfig()->deadband;
        uint8_t stickPercent = 0;
        if ((rcData[axis] >= PWM_RANGE_MAX) || (rcData[axis] <= PWM_RANGE_MIN)) {
            stickPercent = 100;
        } else {
            if (rcData[axis] > (rxConfig()->midrc + deadband)) {
                stickPercent = ((rcData[axis] - rxConfig()->midrc - deadband) * 100) / (PWM_RANGE_MAX - rxConfig()->midrc - deadband);
            } else if (rcData[axis] < (rxConfig()->midrc - deadband)) {
                stickPercent = ((rxConfig()->midrc - deadband - rcData[axis]) * 100) / (rxConfig()->midrc - deadband - PWM_RANGE_MIN);
            }
        }
        if (stickPercent >= stickPercentLimit) {
            return true;
        }
    }
    return false;
}
#endif


// calculate the throttle stick percent - integer math is good enough here.
// returns negative values for reversed thrust in 3D mode
int8_t calculateThrottlePercent(void)
{
    int channelData = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);

    return constrain(((channelData - rxConfig()->mincheck) * 100) / (PWM_RANGE_MAX - rxConfig()->mincheck), 0, 100);
}

uint8_t calculateThrottlePercentAbs(void)
{
    return ABS(calculateThrottlePercent());
}


/*
 * processRx called from taskUpdateRxMain
 */
bool processRx(timeUs_t currentTimeUs)
{
    static bool armedBeeperOn = false;
#ifdef USE_TELEMETRY
    static bool sharedPortTelemetryEnabled = false;
#endif

    timeDelta_t frameAgeUs;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);

    DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(frameDeltaUs / 10, INT16_MAX));
    DEBUG_SET(DEBUG_RX_TIMING, 1, MIN(frameAgeUs / 10, INT16_MAX));

    if (!calculateRxChannelsAndUpdateFailsafe(currentTimeUs)) {
        return false;
    }

    updateRcRefreshRate(currentTimeUs);

    updateRSSI(currentTimeUs);

    if (currentTimeUs > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
        failsafeStartMonitoring();
    }
    failsafeUpdateState();

    const throttleStatus_e throttleStatus = calculateThrottleStatus();

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    const timeUs_t autoDisarmDelayUs = armingConfig()->auto_disarm_delay * 1e6;
    if (ARMING_FLAG(ARMED)
        && !FLIGHT_MODE(GPS_RESCUE_MODE)  // disable auto-disarm when GPS Rescue is active
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if ((autoDisarmDelayUs > 0) && (currentTimeUs > disarmAt)) {
                    // auto-disarm configured and delay is over
                    disarm(DISARM_REASON_THROTTLE_TIMEOUT);
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low - extend disarm time
                disarmAt = currentTimeUs + autoDisarmDelayUs;

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    } else {
        disarmAt = currentTimeUs + autoDisarmDelayUs;  // extend auto-disarm timer
    }

    if (!(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))
#ifdef USE_CMS
        && !cmsInMenu
#endif
        ) {
        processRcStickPositions();
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    updateActivatedModes();

    if (!cliMode && !(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))) {
        processRcAdjustments(currentControlRateProfile);
    }

    if (sensors(SENSOR_ACC)) {
        if (IS_RC_MODE_ACTIVE(BOXRESCUE)) {
            ENABLE_FLIGHT_MODE(RESCUE_MODE);
        } else {
            DISABLE_FLIGHT_MODE(RESCUE_MODE);
        }

        if (IS_RC_MODE_ACTIVE(BOXANGLE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
            DISABLE_FLIGHT_MODE(HORIZON_MODE);
        } else {
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            if (IS_RC_MODE_ACTIVE(BOXHORIZON)) {
                ENABLE_FLIGHT_MODE(HORIZON_MODE);
            } else {
                DISABLE_FLIGHT_MODE(HORIZON_MODE);
            }
        }
    }

#ifdef USE_GPS_RESCUE
    if (ARMING_FLAG(ARMED) && (IS_RC_MODE_ACTIVE(BOXGPSRESCUE) || (failsafeIsActive() && failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE))) {
        ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    } else {
        DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    }
#endif

    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | RESCUE_MODE | FAILSAFE_MODE)) {
        LED1_ON;
        // increase frequency of attitude task to reduce drift when in angle or horizon mode
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(500));
    } else {
        LED1_OFF;
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(100));
    }

    if (!IS_RC_MODE_ACTIVE(BOXPREARM) && ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
        DISABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
    }

    if (IS_RC_MODE_ACTIVE(BOXPASSTHRU)) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        bool enableSharedPortTelemetry = (!isModeActivationConditionPresent(BOXTELEMETRY) && ARMING_FLAG(ARMED)) || (isModeActivationConditionPresent(BOXTELEMETRY) && IS_RC_MODE_ACTIVE(BOXTELEMETRY));
        if (enableSharedPortTelemetry && !sharedPortTelemetryEnabled) {
            mspSerialReleaseSharedTelemetryPorts();
            telemetryCheckState();

            sharedPortTelemetryEnabled = true;
        } else if (!enableSharedPortTelemetry && sharedPortTelemetryEnabled) {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();

            sharedPortTelemetryEnabled = false;
        }
    }
#endif

#ifdef USE_VTX_CONTROL
    vtxUpdateActivatedChannel();

    if (canUpdateVTX()) {
        handleVTXControlButton();
    }
#endif

#ifdef USE_ACRO_TRAINER
    acroTrainerSetState(IS_RC_MODE_ACTIVE(BOXACROTRAINER) && sensors(SENSOR_ACC));
#endif

#ifdef USE_RC_SMOOTHING_FILTER
    if (ARMING_FLAG(ARMED) && !rcSmoothingInitializationComplete()) {
        beeper(BEEPER_RC_SMOOTHING_INIT_FAIL);
    }
#endif

    return true;
}

static FAST_CODE void subTaskPidController(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {startTime = micros();}
    // PID - note this is function pointer set by setPIDController()
    pidController(currentPidProfile, currentTimeUs);
    DEBUG_SET(DEBUG_PIDLOOP, 1, micros() - startTime);

#ifdef USE_PID_AUDIO
    if (isModeActivationConditionPresent(BOXPIDAUDIO)) {
        pidAudioUpdate();
    }
#endif
}

static FAST_CODE_NOINLINE void subTaskPidSubprocesses(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    if (debugMode == DEBUG_PIDLOOP) {
        startTime = micros();
    }

#ifdef USE_DYN_LPF
    dynLpfUpdate(currentTimeUs, getHeadSpeedRatio());
#endif

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

#ifdef USE_BLACKBOX
    if (!cliMode && blackboxConfig()->device) {
        blackboxUpdate(currentTimeUs);
    }
#endif

    DEBUG_SET(DEBUG_PIDLOOP, 3, micros() - startTime);
}

#ifdef USE_TELEMETRY
#define GYRO_TEMP_READ_DELAY_US 3e6    // Only read the gyro temp every 3 seconds
void subTaskTelemetryPollSensors(timeUs_t currentTimeUs)
{
    static timeUs_t lastGyroTempTimeUs = 0;

    if (cmpTimeUs(currentTimeUs, lastGyroTempTimeUs) >= GYRO_TEMP_READ_DELAY_US) {
        // Read out gyro temperature if used for telemmetry
        gyroReadTemperature();
        lastGyroTempTimeUs = currentTimeUs;
    }
}
#endif

static FAST_CODE_NOINLINE void subTaskMixerUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    uint32_t startTime = 0;
    if (debugMode == DEBUG_CYCLETIME) {
        startTime = micros();
        static uint32_t previousUpdateTime;
        const uint32_t currentDeltaTime = startTime - previousUpdateTime;
        debug[2] = currentDeltaTime;
        debug[3] = currentDeltaTime - pidGetLooptime();
        previousUpdateTime = startTime;
    } else if (debugMode == DEBUG_PIDLOOP) {
        startTime = micros();
    }

    mixerUpdate();

#ifdef USE_SERVOS
    servoUpdate();
#endif
#ifdef USE_MOTOR
    motorUpdate();
#endif

    DEBUG_SET(DEBUG_PIDLOOP, 2, micros() - startTime);
}

static FAST_CODE_NOINLINE void subTaskRcCommand(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    if (isUsingSticksForArming() && rcData[THROTTLE] <= rxConfig()->mincheck) {
        resetYawAxis();
    }

    processRcCommand();
}

FAST_CODE void taskGyroSample(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    gyroUpdate();
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        pidUpdateCounter = 0;
    }
    pidUpdateCounter++;
}

FAST_CODE bool gyroFilterReady(void)
{
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        return true;
    } else {
        return false;
    }
}

FAST_CODE bool pidLoopReady(void)
{
    if ((pidUpdateCounter % activePidLoopDenom) == (activePidLoopDenom / 2)) {
        return true;
    }
    return false;
}

FAST_CODE void taskFiltering(timeUs_t currentTimeUs)
{
    gyroFiltering(currentTimeUs);

}

// Function for loop trigger
FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)
{

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_GYROPID_SYNC)
    if (lockMainPID() != 0) return;
#endif

    // DEBUG_PIDLOOP, timings for:
    // 0 - gyroUpdate()
    // 1 - subTaskPidController()
    // 2 - subTaskMixerUpdate()
    // 3 - subTaskPidSubprocesses()
    DEBUG_SET(DEBUG_PIDLOOP, 0, micros() - currentTimeUs);

    subTaskRcCommand(currentTimeUs);
    subTaskPidController(currentTimeUs);
    subTaskMixerUpdate(currentTimeUs);
    subTaskPidSubprocesses(currentTimeUs);

    if (debugMode == DEBUG_CYCLETIME) {
        DEBUG_SET(DEBUG_CYCLETIME, 0, getTaskDeltaTimeUs(TASK_SELF));
        DEBUG_SET(DEBUG_CYCLETIME, 1, getAverageSystemLoadPercent());
    }
}

timeUs_t getLastDisarmTimeUs(void)
{
    return lastDisarmTimeUs;
}

bool isTryingToArm()
{
    return (tryingToArm != ARMING_DELAYED_DISARMED);
}

void resetTryingToArm()
{
    tryingToArm = ARMING_DELAYED_DISARMED;
}


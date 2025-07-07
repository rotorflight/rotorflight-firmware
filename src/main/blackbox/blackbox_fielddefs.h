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

typedef enum FlightLogFieldCondition {
    FLIGHT_LOG_FIELD_CONDITION_ALWAYS = 0,

    FLIGHT_LOG_FIELD_CONDITION_COMMAND,
    FLIGHT_LOG_FIELD_CONDITION_SETPOINT,
    FLIGHT_LOG_FIELD_CONDITION_MIXER,
    FLIGHT_LOG_FIELD_CONDITION_PID,
    FLIGHT_LOG_FIELD_CONDITION_BOOST,
    FLIGHT_LOG_FIELD_CONDITION_HSI,

    FLIGHT_LOG_FIELD_CONDITION_ATTITUDE,
    FLIGHT_LOG_FIELD_CONDITION_GYRAW,
    FLIGHT_LOG_FIELD_CONDITION_GYRO,
    FLIGHT_LOG_FIELD_CONDITION_ACC,
    FLIGHT_LOG_FIELD_CONDITION_MAG,
    FLIGHT_LOG_FIELD_CONDITION_ALT,
    FLIGHT_LOG_FIELD_CONDITION_RSSI,
    FLIGHT_LOG_FIELD_CONDITION_VOLTAGE,
    FLIGHT_LOG_FIELD_CONDITION_CURRENT,
    FLIGHT_LOG_FIELD_CONDITION_VBEC,
    FLIGHT_LOG_FIELD_CONDITION_VBUS,

    FLIGHT_LOG_FIELD_CONDITION_HEADSPEED,
    FLIGHT_LOG_FIELD_CONDITION_TAILSPEED,
    FLIGHT_LOG_FIELD_CONDITION_GOVERNOR,

    FLIGHT_LOG_FIELD_CONDITION_TMCU,
    FLIGHT_LOG_FIELD_CONDITION_TESC,
    FLIGHT_LOG_FIELD_CONDITION_TBEC,
    FLIGHT_LOG_FIELD_CONDITION_TESC2,

    FLIGHT_LOG_FIELD_CONDITION_ESC_TELEM,
    FLIGHT_LOG_FIELD_CONDITION_BEC_TELEM,

    FLIGHT_LOG_FIELD_CONDITION_ESC2_TELEM,

    FLIGHT_LOG_FIELD_CONDITION_MOTOR_1,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_2,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_3,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_4,

    FLIGHT_LOG_FIELD_CONDITION_SERVO_1,
    FLIGHT_LOG_FIELD_CONDITION_SERVO_2,
    FLIGHT_LOG_FIELD_CONDITION_SERVO_3,
    FLIGHT_LOG_FIELD_CONDITION_SERVO_4,
    FLIGHT_LOG_FIELD_CONDITION_SERVO_5,
    FLIGHT_LOG_FIELD_CONDITION_SERVO_6,
    FLIGHT_LOG_FIELD_CONDITION_SERVO_7,
    FLIGHT_LOG_FIELD_CONDITION_SERVO_8,

    FLIGHT_LOG_FIELD_CONDITION_DEBUG,

    FLIGHT_LOG_FIELD_CONDITION_NOT_EVERY_FRAME,

    FLIGHT_LOG_FIELD_CONDITION_NEVER,

    FLIGHT_LOG_FIELD_CONDITION_COUNT
} FlightLogFieldCondition;

typedef enum FlightLogFieldSelect_e {
    FLIGHT_LOG_FIELD_SELECT_COMMAND,
    FLIGHT_LOG_FIELD_SELECT_SETPOINT,
    FLIGHT_LOG_FIELD_SELECT_MIXER,
    FLIGHT_LOG_FIELD_SELECT_PID,
    FLIGHT_LOG_FIELD_SELECT_ATTITUDE,
    FLIGHT_LOG_FIELD_SELECT_GYRAW,
    FLIGHT_LOG_FIELD_SELECT_GYRO,
    FLIGHT_LOG_FIELD_SELECT_ACC,
    FLIGHT_LOG_FIELD_SELECT_MAG,
    FLIGHT_LOG_FIELD_SELECT_ALT,
    FLIGHT_LOG_FIELD_SELECT_BATTERY,
    FLIGHT_LOG_FIELD_SELECT_RSSI,
    FLIGHT_LOG_FIELD_SELECT_GPS,
    FLIGHT_LOG_FIELD_SELECT_RPM,
    FLIGHT_LOG_FIELD_SELECT_MOTOR,
    FLIGHT_LOG_FIELD_SELECT_SERVO,
    FLIGHT_LOG_FIELD_SELECT_VBEC,
    FLIGHT_LOG_FIELD_SELECT_VBUS,
    FLIGHT_LOG_FIELD_SELECT_TEMP,
    FLIGHT_LOG_FIELD_SELECT_ESC,
    FLIGHT_LOG_FIELD_SELECT_BEC,
    FLIGHT_LOG_FIELD_SELECT_ESC2,
    FLIGHT_LOG_FIELD_SELECT_GOV,
    FLIGHT_LOG_FIELD_SELECT_COUNT
} FlightLogFieldSelect_e;

typedef enum {
    FLIGHT_LOG_EVENT_SYNC_BEEP = 0,
    FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT = 13,
    FLIGHT_LOG_EVENT_LOGGING_RESUME = 14,
    FLIGHT_LOG_EVENT_DISARM = 15,
    FLIGHT_LOG_EVENT_FLIGHTMODE = 30, // Add new event type for flight mode status.
    FLIGHT_LOG_EVENT_GOVSTATE = 50,   // Add new event type for main motor governor state.
    FLIGHT_LOG_EVENT_RESCUE_STATE = 51,
    FLIGHT_LOG_EVENT_AIRBORNE_STATE = 52,
    FLIGHT_LOG_EVENT_CUSTOM_DATA = 100,
    FLIGHT_LOG_EVENT_CUSTOM_STRING = 101,
    FLIGHT_LOG_EVENT_LOG_END = 255
} FlightLogEvent;

typedef enum FlightLogFieldPredictor {
    //No prediction:
    FLIGHT_LOG_FIELD_PREDICTOR_0              = 0,

    //Predict that the field is the same as last frame:
    FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS       = 1,

    //Predict that the slope between this field and the previous item is the same as that between the past two history items:
    FLIGHT_LOG_FIELD_PREDICTOR_LINEAR         = 2,

    //Predict that this field is the same as the average of the last two history items:
    FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2      = 3,

    //Predict that this field is minthrottle
    FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE    = 4,

    //Predict that this field is the same as motor 0
    FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0        = 5,

    //This field always increments
    FLIGHT_LOG_FIELD_PREDICTOR_INC            = 6,

    //Predict this GPS co-ordinate is the GPS home co-ordinate (or no prediction if that coordinate is not set)
    FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD     = 7,

    //Predict 1500
    FLIGHT_LOG_FIELD_PREDICTOR_1500           = 8,

    //Predict vbatref, the reference ADC level stored in the header
    FLIGHT_LOG_FIELD_PREDICTOR_VBATREF        = 9,

    //Predict the last time value written in the main stream
    FLIGHT_LOG_FIELD_PREDICTOR_LAST_MAIN_FRAME_TIME = 10,

    //Predict that this field is the minimum motor output
    FLIGHT_LOG_FIELD_PREDICTOR_MINMOTOR       = 11

} FlightLogFieldPredictor;

typedef enum FlightLogFieldEncoding {
    FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB       = 0, // Signed variable-byte
    FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB     = 1, // Unsigned variable-byte
    FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT       = 3, // Unsigned variable-byte but we negate the value before storing, value is 14 bits
    FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB       = 6,
    FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32       = 7,
    FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16       = 8,
    FLIGHT_LOG_FIELD_ENCODING_NULL            = 9, // Nothing is written to the file, take value to be zero
    FLIGHT_LOG_FIELD_ENCODING_TAG2_3SVARIABLE = 10
} FlightLogFieldEncoding;

typedef enum FlightLogFieldSign {
    FLIGHT_LOG_FIELD_UNSIGNED = 0,
    FLIGHT_LOG_FIELD_SIGNED   = 1
} FlightLogFieldSign;

typedef struct flightLogEvent_syncBeep_s {
    uint32_t time;
} flightLogEvent_syncBeep_t;

typedef struct flightLogEvent_disarm_s {
    uint32_t reason;
} flightLogEvent_disarm_t;

typedef struct flightLogEvent_flightMode_s {
    uint32_t flags;
    uint32_t lastFlags;
} flightLogEvent_flightMode_t;

typedef struct flightLogEvent_govState_s {
    uint8_t govState;
} flightLogEvent_govState_t;

typedef struct flightLogEvent_rescueState_s {
    uint8_t rescueState;
} flightLogEvent_rescueState_t;

typedef struct flightLogEvent_airborneState_s {
    uint8_t airborneState;
} flightLogEvent_airborneState_t;

typedef struct flightLogEvent_inflightAdjustment_s {
    int32_t newValue;
    float newFloatValue;
    uint8_t adjustmentFunction;
    bool floatFlag;
} flightLogEvent_inflightAdjustment_t;

typedef struct flightLogEvent_customData_s {
    const uint8_t *buffer;
    uint8_t length;
} flightLogEvent_customData_t;

typedef struct flightLogEvent_customString_s {
    const char *buffer;
} flightLogEvent_customString_t;

typedef struct flightLogEvent_loggingResume_s {
    uint32_t logIteration;
    uint32_t currentTime;
} flightLogEvent_loggingResume_t;

#define FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG 128

typedef union flightLogEventData_u {
    flightLogEvent_syncBeep_t syncBeep;
    flightLogEvent_flightMode_t flightMode;
    flightLogEvent_govState_t govState;
    flightLogEvent_rescueState_t rescueState;
    flightLogEvent_airborneState_t airborneState;
    flightLogEvent_disarm_t disarm;
    flightLogEvent_inflightAdjustment_t inflightAdjustment;
    flightLogEvent_customData_t data;
    flightLogEvent_customString_t string;
    flightLogEvent_loggingResume_t loggingResume;
} flightLogEventData_t;

typedef struct flightLogEvent_s {
    FlightLogEvent event;
    flightLogEventData_t data;
} flightLogEvent_t;

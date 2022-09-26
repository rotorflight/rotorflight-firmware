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

#ifdef USE_BLACKBOX

#include "blackbox.h"
#include "blackbox_encoding.h"
#include "blackbox_fielddefs.h"
#include "blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/encoding.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"
#include "common/printf.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "fc/board_info.h"
#include "fc/rc_rates.h"
#include "fc/parameter_names.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"
#include "flight/governor.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#else
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SERIAL
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 2);

PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
    .sample_rate = BLACKBOX_RATE_QUARTER,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .fields = BIT(FLIGHT_LOG_FIELD_SELECT_SETPOINT) |
              BIT(FLIGHT_LOG_FIELD_SELECT_MIXER) |
              BIT(FLIGHT_LOG_FIELD_SELECT_PID) |
              BIT(FLIGHT_LOG_FIELD_SELECT_GYRO) |
              BIT(FLIGHT_LOG_FIELD_SELECT_BATTERY) |
              BIT(FLIGHT_LOG_FIELD_SELECT_RSSI) |
              BIT(FLIGHT_LOG_FIELD_SELECT_MOTOR) |
              BIT(FLIGHT_LOG_FIELD_SELECT_SERVO),
    .mode = BLACKBOX_MODE_NORMAL,
);

STATIC_ASSERT((sizeof(blackboxConfig()->fields) * 8) >= FLIGHT_LOG_FIELD_SELECT_COUNT, too_many_flight_log_fields_selections);

#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:
#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define FIELD_SELECT(x) CONCAT(FLIGHT_LOG_FIELD_SELECT_, x)

#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

#define ENCODING_NULL FLIGHT_LOG_FIELD_ENCODING_NULL

static const char blackboxHeader[] =
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Data version:2\n";

static const char* const blackboxFieldHeaderNames[] = {
    "name",
    "signed",
    "predictor",
    "encoding",
    "predictor",
    "encoding"
};

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_s {
    const char *name;
    // If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
    int8_t fieldNameIndex;

    // Each member of this array will be the value to print for this field for the given header index
    uint8_t arr[1];
} blackboxFieldDefinition_t;

#define BLACKBOX_DELTA_FIELD_HEADER_COUNT       ARRAYLEN(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT      (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)

typedef struct blackboxSimpleFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxSimpleFieldDefinition_t;

typedef struct blackboxConditionalFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxConditionalFieldDefinition_t;

typedef struct blackboxDeltaFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] =
{
    /* loop iteration doesn't appear in P frames since it always increments */
    {"loopIteration", -1, UNSIGNED, .Ipredict = PREDICT(0),    .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(INC),           .Pencode = ENCODING_NULL,        CONDITION(ALWAYS)},

    /* Time advances pretty steadily so the P-frame prediction is a straight line */
    {"time",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(LINEAR),        .Pencode = ENCODING(SIGNED_VB),  CONDITION(ALWAYS)},

    /* RC commands are encoded together as a group in P-frames: */
    {"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},
    {"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},
    {"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},
    {"rcCommand",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},

    /* setpoint - define 4 fields like RC command */
    {"setpoint",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},
    {"setpoint",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},
    {"setpoint",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},
    {"setpoint",    3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},

    /* Mixer inputs */
    {"mixer",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},
    {"mixer",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},
    {"mixer",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},
    {"mixer",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},

    /* PID control terms */
    {"axisP",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisP",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisP",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisI",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisI",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisI",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisD",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisD",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisD",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisF",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(PID)},
    {"axisF",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(PID)},
    {"axisF",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(PID)},

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroRAW",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRAW)},
    {"gyroRAW",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRAW)},
    {"gyroRAW",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRAW)},

    {"gyroADC",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRO)},
    {"gyroADC",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRO)},
    {"gyroADC",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRO)},

    {"accADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(ACC)},
    {"accADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(ACC)},
    {"accADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(ACC)},
#
#ifdef USE_MAG
    {"magADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(MAG)},
    {"magADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(MAG)},
    {"magADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(MAG)},
#endif

#ifdef USE_BARO
    {"BaroAlt",    -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(BARO)},
#endif

    {"vbatLatest",     -1, UNSIGNED, .Ipredict = PREDICT(VBATREF),  .Iencode = ENCODING(NEG_14BIT),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(VOLTAGE)},
    {"amperageLatest", -1, UNSIGNED, .Ipredict = PREDICT(0),        .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(CURRENT)},

    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(RSSI)},

    {"headspeed",  -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(HEADSPEED)},
    {"tailspeed",  -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(TAILSPEED)},

    {"motor",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_1)},
    {"motor",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_2)},
    {"motor",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_3)},
    {"motor",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_4)},

    {"servo",       0, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_1)},
    {"servo",       1, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_2)},
    {"servo",       2, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_3)},
    {"servo",       3, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_4)},
    {"servo",       4, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_5)},
    {"servo",       5, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_6)},
    {"servo",       6, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_7)},
    {"servo",       7, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_8)},

    {"debug",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},

};

#ifdef USE_GPS
// GPS position/vel frame
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_EVERY_FRAME)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};

// GPS home frame
static const blackboxSimpleFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home",           0, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)},
    {"GPS_home",           1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)}
};
#endif

// Rarely-updated fields
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
    {"flightModeFlags",       -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},
    {"stateFlags",            -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},

    {"failsafePhase",         -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxSignalReceived",      -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)}
};

typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_PREPARE_LOG_FILE,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
    BLACKBOX_STATE_SEND_GPS_H_HEADER,
    BLACKBOX_STATE_SEND_GPS_G_HEADER,
    BLACKBOX_STATE_SEND_SLOW_HEADER,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_CACHE_FLUSH,
    BLACKBOX_STATE_PAUSED,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_SHUTTING_DOWN,
    BLACKBOX_STATE_START_ERASE,
    BLACKBOX_STATE_ERASING,
    BLACKBOX_STATE_ERASED
} BlackboxState;


typedef struct blackboxMainState_s {
    uint32_t time;

    int16_t command[4];
    int16_t setpoint[4];
    int16_t mixer[4];

    int32_t axisPID_P[XYZ_AXIS_COUNT];
    int32_t axisPID_I[XYZ_AXIS_COUNT];
    int32_t axisPID_D[XYZ_AXIS_COUNT];
    int32_t axisPID_F[XYZ_AXIS_COUNT];

    int16_t gyroRAW[XYZ_AXIS_COUNT];
    int16_t gyroADC[XYZ_AXIS_COUNT];
    int16_t accADC[XYZ_AXIS_COUNT];
#ifdef USE_MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
#ifdef USE_BARO
    int32_t baro;
#endif

    uint16_t voltage;
    uint16_t current;

    uint16_t rssi;

    uint16_t headspeed;
    uint16_t tailspeed;

    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];

    int32_t debug[DEBUG_VALUE_COUNT];

} blackboxMainState_t;

typedef struct blackboxGpsState_s {
    int32_t GPS_home[2];
    int32_t GPS_coord[2];
    uint8_t GPS_numSat;
} blackboxGpsState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_s {
    uint32_t flightModeFlags; // extend this data size (from uint16_t)
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived;
    bool rxFlightChannelsValid;
} __attribute__((__packed__)) blackboxSlowState_t; // We pack this struct so that padding doesn't interfere with memcmp()

//From rc_controls.c
extern boxBitmask_t rcModeActivationMask;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static uint32_t blackboxLastArmingBeep = 0;
static uint32_t blackboxLastFlightModeFlags = 0; // New event tracking of flight modes
static uint8_t  blackboxLastGovState = 0;

static struct {
    uint32_t headerIndex;

    /* Since these fields are used during different blackbox states (never simultaneously) we can
     * overlap them to save on RAM
     */
    union {
        int fieldIndex;
        uint32_t startTime;
    } u;
} xmitState;

// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static uint32_t blackboxConditionCache;

STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_LAST, too_many_flight_log_conditions);

static uint32_t blackboxIteration;
static uint16_t blackboxLoopIndex;
static uint16_t blackboxPFrameIndex;
static uint16_t blackboxIFrameIndex;
// number of flight loop iterations before logging I-frame
// typically 32 for 1kHz loop, 64 for 2kHz loop etc
static int16_t blackboxIInterval = 0;
// number of flight loop iterations before logging P-frame
static int8_t blackboxPInterval = 0;
static int32_t blackboxSInterval = 0;
static int32_t blackboxSlowFrameIterationTimer;
static bool blackboxLoggedAnyFrames;

/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;

static blackboxGpsState_t gpsHistory;
static blackboxSlowState_t slowHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxMainState_t* blackboxHistory[3];


/**
 * Return true if it is safe to edit the Blackbox configuration.
 */
bool blackboxMayEditConfig(void)
{
    return blackboxState <= BLACKBOX_STATE_STOPPED;
}

static bool blackboxIsLoggingEnabled(void)
{
    return (blackboxConfig()->device && (
        (blackboxConfig()->mode == BLACKBOX_MODE_NORMAL && ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXBLACKBOX)) ||
        (blackboxConfig()->mode == BLACKBOX_MODE_ARMED && ARMING_FLAG(ARMED)) ||
        (blackboxConfig()->mode == BLACKBOX_MODE_SWITCH && IS_RC_MODE_ACTIVE(BOXBLACKBOX))));
}

static bool blackboxIsLoggingPaused(void)
{
    return (blackboxConfig()->mode == BLACKBOX_MODE_NORMAL && !IS_RC_MODE_ACTIVE(BOXBLACKBOX));
}

static bool blackboxIsOnlyLoggingIntraframes(void)
{
    return (blackboxPInterval == 0);
}

static bool isFieldEnabled(FlightLogFieldSelect_e field)
{
    return (blackboxConfig()->fields & BIT(field));
}

static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition) {
    case CONDITION(ALWAYS):
        return true;

    case CONDITION(COMMAND):
        return isFieldEnabled(FIELD_SELECT(COMMAND));

    case CONDITION(SETPOINT):
        return isFieldEnabled(FIELD_SELECT(SETPOINT));

    case CONDITION(MIXER):
        return isFieldEnabled(FIELD_SELECT(MIXER));

    case CONDITION(PID):
        return isFieldEnabled(FIELD_SELECT(PID));

    case CONDITION(GYRAW):
        return isFieldEnabled(FIELD_SELECT(GYRAW));

    case CONDITION(GYRO):
        return isFieldEnabled(FIELD_SELECT(GYRO));

    case CONDITION(ACC):
        return sensors(SENSOR_ACC) && isFieldEnabled(FIELD_SELECT(ACC));

    case CONDITION(MAG):
#ifdef USE_MAG
        return sensors(SENSOR_MAG) && isFieldEnabled(FIELD_SELECT(MAG));
#else
        return false;
#endif

    case CONDITION(BARO):
#ifdef USE_BARO
        return sensors(SENSOR_BARO) && isFieldEnabled(FIELD_SELECT(BARO));
#else
        return false;
#endif

    case CONDITION(HEADSPEED):
        return (getMotorCount() >= 1) && isFieldEnabled(FIELD_SELECT(RPM));
    case CONDITION(TAILSPEED):
        return (getMotorCount() >= 2) && isFieldEnabled(FIELD_SELECT(RPM));

    case CONDITION(MOTOR_1):
        return (getMotorCount() >= 1) && isFieldEnabled(FIELD_SELECT(MOTOR));
    case CONDITION(MOTOR_2):
        return (getMotorCount() >= 2) && isFieldEnabled(FIELD_SELECT(MOTOR));
    case CONDITION(MOTOR_3):
        return (getMotorCount() >= 3) && isFieldEnabled(FIELD_SELECT(MOTOR));
    case CONDITION(MOTOR_4):
        return (getMotorCount() >= 4) && isFieldEnabled(FIELD_SELECT(MOTOR));

    case CONDITION(SERVO_1):
        return (getServoCount() >= 1) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_2):
        return (getServoCount() >= 2) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_3):
        return (getServoCount() >= 3) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_4):
        return (getServoCount() >= 4) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_5):
        return (getServoCount() >= 5) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_6):
        return (getServoCount() >= 6) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_7):
        return (getServoCount() >= 7) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_8):
        return (getServoCount() >= 8) && isFieldEnabled(FIELD_SELECT(SERVO));

    case CONDITION(RSSI):
        return isRssiConfigured() && isFieldEnabled(FIELD_SELECT(RSSI));

    case CONDITION(VOLTAGE):
        return (batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE) && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(CURRENT):
        return (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(DEBUG):
        return (debugMode != DEBUG_NONE);

    case CONDITION(NOT_EVERY_FRAME):
        return (blackboxPInterval != blackboxIInterval);

    case CONDITION(NEVER):
        return false;

    default:
        return false;
    }
}

static void blackboxBuildConditionCache(void)
{
    blackboxConditionCache = 0;
    for (FlightLogFieldCondition cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
        if (testBlackboxConditionUncached(cond)) {
            blackboxConditionCache |= BIT(cond);
        }
    }
}

static bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & BIT(condition));
}

static void blackboxSetState(BlackboxState newState)
{
    //Perform initial setup required for the new state
    switch (newState) {
    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        blackboxLoggedAnyFrames = false;
        break;
    case BLACKBOX_STATE_SEND_HEADER:
        blackboxHeaderBudget = 0;
        xmitState.headerIndex = 0;
        xmitState.u.startTime = millis();
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
    case BLACKBOX_STATE_SEND_GPS_H_HEADER:
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        xmitState.headerIndex = 0;
        break;
    case BLACKBOX_STATE_RUNNING:
        blackboxSlowFrameIterationTimer = blackboxSInterval; //Force a slow frame to be written on the first iteration
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        xmitState.u.startTime = millis();
        break;
    default:
        ;
    }
    blackboxState = newState;
}

static void writeIntraframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    const int motorCount = getMotorCount();
    const int servoCount = getServoCount();

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    if (testBlackboxCondition(CONDITION(COMMAND))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->command, 4);
    }

    if (testBlackboxCondition(CONDITION(SETPOINT))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->setpoint, 4);
    }

    if (testBlackboxCondition(CONDITION(MIXER))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->mixer, 4);
    }

    if (testBlackboxCondition(CONDITION(PID))) {
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_D, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_F, XYZ_AXIS_COUNT);
    }

    if (testBlackboxCondition(CONDITION(GYRAW))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroRAW, XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(GYRO))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(ACC))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->accADC, XYZ_AXIS_COUNT);
    }
#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);
    }
#endif
#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(BARO))) {
        blackboxWriteSignedVB(blackboxCurrent->baro);
    }
#endif

    if (testBlackboxCondition(CONDITION(VOLTAGE))) {
        blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->voltage) & 0x3FFF);
    }
    if (testBlackboxCondition(CONDITION(CURRENT))) {
        blackboxWriteUnsignedVB(blackboxCurrent->current);
    }
    if (testBlackboxCondition(CONDITION(RSSI))) {
        blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    }

    if (testBlackboxCondition(CONDITION(HEADSPEED))) {
        blackboxWriteUnsignedVB(blackboxCurrent->headspeed);
    }
    if (testBlackboxCondition(CONDITION(TAILSPEED))) {
        blackboxWriteUnsignedVB(blackboxCurrent->tailspeed);
    }

    if (isFieldEnabled(FIELD_SELECT(MOTOR))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->motor, motorCount);
    }
    if (isFieldEnabled(FIELD_SELECT(SERVO))) {
        for (int i = 0; i < servoCount; i++) {
            blackboxWriteSignedVB(blackboxCurrent->servo[i] - 1500);
        }
    }

    if (testBlackboxCondition(CONDITION(DEBUG))) {
        blackboxWriteSignedVBArray(blackboxCurrent->debug, DEBUG_VALUE_COUNT);
    }

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
    int16_t *curr  = (int16_t*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
    int16_t *prev1 = (int16_t*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
    int16_t *prev2 = (int16_t*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);

    for (int i = 0; i < count; i++) {
        // Predictor is the average of the previous two history states
        int32_t predictor = (prev1[i] + prev2[i]) / 2;

        blackboxWriteSignedVB(curr[i] - predictor);
    }
}

#define CALC_DELTAS(delta, next, prev, count) do {  \
    for (int i = 0; i < (count); i++)               \
        delta[i] = next[i] - prev[i];               \
} while(0)


static void writeInterframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxPrev = blackboxHistory[1];

    const int motorCount = getMotorCount();
    const int servoCount = getServoCount();

    int32_t deltas[8];

    blackboxWrite('P');

    // No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    if (testBlackboxCondition(CONDITION(COMMAND))) {
        CALC_DELTAS(deltas, blackboxCurrent->command, blackboxPrev->command, 4);
        blackboxWriteTag8_4S16(deltas);
    }

    if (testBlackboxCondition(CONDITION(SETPOINT))) {
        CALC_DELTAS(deltas, blackboxCurrent->setpoint, blackboxPrev->setpoint, 4);
        blackboxWriteTag8_4S16(deltas);
    }

    if (testBlackboxCondition(CONDITION(MIXER))) {
        CALC_DELTAS(deltas, blackboxCurrent->mixer, blackboxPrev->mixer, 4);
        blackboxWriteTag8_4S16(deltas);
    }

    if (testBlackboxCondition(CONDITION(PID))) {
        CALC_DELTAS(deltas, blackboxCurrent->axisPID_P, blackboxPrev->axisPID_P, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);

        CALC_DELTAS(deltas, blackboxCurrent->axisPID_I, blackboxPrev->axisPID_I, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);

        CALC_DELTAS(deltas, blackboxCurrent->axisPID_D, blackboxPrev->axisPID_D, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);

        CALC_DELTAS(deltas, blackboxCurrent->axisPID_F, blackboxPrev->axisPID_F, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);
    }

    // Since gyro and acc are noisy, base their predictions on the average of the history
    if (testBlackboxCondition(CONDITION(GYRAW))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroRAW), XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(GYRO))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC), XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(ACC))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADC), XYZ_AXIS_COUNT);
    }

    // Check for sensors that are updated periodically (so deltas are normally zero)
    int packedFieldCount = 0;

#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG))) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            deltas[packedFieldCount++] = blackboxCurrent->magADC[i] - blackboxPrev->magADC[i];
        }
    }
#endif
#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(BARO))) {
        deltas[packedFieldCount++] = blackboxCurrent->baro - blackboxPrev->baro;
    }
#endif

    if (testBlackboxCondition(CONDITION(VOLTAGE))) {
        deltas[packedFieldCount++] = (int32_t) blackboxCurrent->voltage - blackboxPrev->voltage;
    }
    if (testBlackboxCondition(CONDITION(CURRENT))) {
        deltas[packedFieldCount++] = (int32_t) blackboxCurrent->current - blackboxPrev->current;
    }
    if (testBlackboxCondition(CONDITION(RSSI))) {
        deltas[packedFieldCount++] = (int32_t) blackboxCurrent->rssi - blackboxPrev->rssi;
    }

    blackboxWriteTag8_8SVB(deltas, packedFieldCount);

    if (testBlackboxCondition(CONDITION(HEADSPEED))) {
        int32_t predictor = (blackboxHistory[1]->headspeed + blackboxHistory[2]->headspeed) / 2;
        blackboxWriteSignedVB(blackboxCurrent->headspeed - predictor);
    }
    if (testBlackboxCondition(CONDITION(TAILSPEED))) {
        int32_t predictor = (blackboxHistory[1]->tailspeed + blackboxHistory[2]->tailspeed) / 2;
        blackboxWriteSignedVB(blackboxCurrent->tailspeed - predictor);
    }

    if (isFieldEnabled(FIELD_SELECT(MOTOR))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor), motorCount);
    }
    if (isFieldEnabled(FIELD_SELECT(SERVO))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, servo), servoCount);
    }

    if (testBlackboxCondition(CONDITION(DEBUG))) {
        CALC_DELTAS(deltas, blackboxCurrent->debug, blackboxPrev->debug, DEBUG_VALUE_COUNT);
        blackboxWriteSignedVBArray(deltas, DEBUG_VALUE_COUNT);
    }

    // Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
static void writeSlowFrame(void)
{
    int32_t values[3];

    blackboxWrite('S');

    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags);

    /*
     * Most of the time these three values will be able to pack into one byte for us:
     */
    values[0] = slowHistory.failsafePhase;
    values[1] = slowHistory.rxSignalReceived ? 1 : 0;
    values[2] = slowHistory.rxFlightChannelsValid ? 1 : 0;
    blackboxWriteTag2_3S32(values);

    blackboxSlowFrameIterationTimer = 0;
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(blackboxSlowState_t *slow)
{
    memcpy(&slow->flightModeFlags, &rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slow->stateFlags = stateFlags;
    slow->failsafePhase = failsafePhase();
    slow->rxSignalReceived = rxIsReceivingSignal();
    slow->rxFlightChannelsValid = rxAreFlightChannelsValid();
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than blackboxSInterval logging iterations
 * since the field was last logged.
 */
static bool writeSlowFrameIfNeeded(void)
{
    // Write the slow frame peridocially so it can be recovered if we ever lose sync
    bool shouldWrite = blackboxSlowFrameIterationTimer >= blackboxSInterval;

    if (shouldWrite) {
        loadSlowState(&slowHistory);
    } else {
        blackboxSlowState_t newSlowState;

        loadSlowState(&newSlowState);

        // Only write a slow frame if it was different from the previous state
        if (memcmp(&newSlowState, &slowHistory, sizeof(slowHistory)) != 0) {
            // Use the new state as our new history
            memcpy(&slowHistory, &newSlowState, sizeof(slowHistory));
            shouldWrite = true;
        }
    }

    if (shouldWrite) {
        writeSlowFrame();
    }
    return shouldWrite;
}

void blackboxValidateConfig(void)
{
    // If we've chosen an unsupported device, change the device to serial
    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
#endif
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
#endif
    case BLACKBOX_DEVICE_SERIAL:
        // Device supported, leave the setting alone
        break;

    default:
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;
    }
}

static void blackboxResetIterationTimers(void)
{
    blackboxIteration = 0;
    blackboxLoopIndex = 0;
    blackboxIFrameIndex = 0;
    blackboxPFrameIndex = 0;
    blackboxSlowFrameIterationTimer = 0;
}

/**
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
static void blackboxStart(void)
{
    blackboxValidateConfig();

    if (!blackboxDeviceOpen()) {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
        return;
    }

    memset(&gpsHistory, 0, sizeof(gpsHistory));

    blackboxHistory[0] = &blackboxHistoryRing[0];
    blackboxHistory[1] = &blackboxHistoryRing[1];
    blackboxHistory[2] = &blackboxHistoryRing[2];

    vbatReference = getBatteryVoltageLatest();

    //No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

    /*
     * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
     * must always agree with the logged data, the results of these tests must not change during logging. So
     * cache those now.
     */
    blackboxBuildConditionCache();
    blackboxResetIterationTimers();

    /*
     * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
     * it finally plays the beep for this arming event.
     */
    blackboxLastArmingBeep = getArmingBeepTimeMicros();
    memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags)); // record startup status
    blackboxLastGovState = getGovernorState();

    blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
}

void blackboxCheckEnabler(void)
{
    if (!blackboxIsLoggingEnabled()) {
        switch (blackboxState) {
        case BLACKBOX_STATE_DISABLED:
        case BLACKBOX_STATE_STOPPED:
        case BLACKBOX_STATE_SHUTTING_DOWN:
            // We're already stopped/shutting down
            break;
        case BLACKBOX_STATE_RUNNING:
        case BLACKBOX_STATE_PAUSED:
            blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);
            FALLTHROUGH;
        default:
            blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
        }
    }
}

#ifdef USE_GPS
static void writeGPSHomeFrame(void)
{
    blackboxWrite('H');

    blackboxWriteSignedVB(GPS_home[0]);
    blackboxWriteSignedVB(GPS_home[1]);
    //TODO it'd be great if we could grab the GPS current time and write that too

    gpsHistory.GPS_home[0] = GPS_home[0];
    gpsHistory.GPS_home[1] = GPS_home[1];
}

static void writeGPSFrame(timeUs_t currentTimeUs)
{
    blackboxWrite('G');

    /*
     * If we're logging every frame, then a GPS frame always appears just after a frame with the
     * currentTime timestamp in the log, so the reader can just use that timestamp for the GPS frame.
     *
     * If we're not logging every frame, we need to store the time of this GPS frame.
     */
    if (testBlackboxCondition(CONDITION(NOT_EVERY_FRAME))) {
        // Predict the time of the last frame in the main log
        blackboxWriteUnsignedVB(currentTimeUs - blackboxHistory[1]->time);
    }

    blackboxWriteUnsignedVB(gpsSol.numSat);
    blackboxWriteSignedVB(gpsSol.llh.lat - gpsHistory.GPS_home[GPS_LATITUDE]);
    blackboxWriteSignedVB(gpsSol.llh.lon - gpsHistory.GPS_home[GPS_LONGITUDE]);
    blackboxWriteUnsignedVB(gpsSol.llh.altCm / 10); // was originally designed to transport meters in int16, but +-3276.7m is a good compromise
    blackboxWriteUnsignedVB(gpsSol.groundSpeed);
    blackboxWriteUnsignedVB(gpsSol.groundCourse);

    gpsHistory.GPS_numSat = gpsSol.numSat;
    gpsHistory.GPS_coord[GPS_LATITUDE] = gpsSol.llh.lat;
    gpsHistory.GPS_coord[GPS_LONGITUDE] = gpsSol.llh.lon;
}
#endif

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(timeUs_t currentTimeUs)
{
#ifndef UNIT_TEST
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxCurrent->time = currentTimeUs;

    for (int i = 0; i < 4; i++) {
        blackboxCurrent->command[i] = lrintf(rcCommand[i]);
    }

    for (int i = 0; i < 4; i++) {
        blackboxCurrent->setpoint[i] = lrintf(getSetpoint(i));
    }

    blackboxCurrent->mixer[0] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_ROLL) * 1000);
    blackboxCurrent->mixer[1] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_PITCH) * 1000);
    blackboxCurrent->mixer[2] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_YAW) * 1000);
    blackboxCurrent->mixer[3] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE) * 1000);

    const pidAxisData_t *pidData = pidGetAxisData();

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_P[i] = lrintf(pidData[i].P * 1000);
        blackboxCurrent->axisPID_I[i] = lrintf(pidData[i].I * 1000);
        blackboxCurrent->axisPID_D[i] = lrintf(pidData[i].D * 1000);
        blackboxCurrent->axisPID_F[i] = lrintf(pidData[i].F * 1000);
    }

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->gyroADC[i] = lrintf(gyro.gyroADCf[i]);
        blackboxCurrent->gyroRAW[i] = lrintf(gyro.gyroADCd[i]);
#ifdef USE_ACC
        blackboxCurrent->accADC[i] = lrintf(acc.accADC[i]);
#endif
#ifdef USE_MAG
        blackboxCurrent->magADC[i] = lrintf(mag.magADC[i]);
#endif
    }

#ifdef USE_BARO
    blackboxCurrent->baro = baro.BaroAlt;
#endif

    blackboxCurrent->voltage = getBatteryVoltageLatest();
    blackboxCurrent->current = constrain(getAmperageLatest(), 0, 50000); // 500Amps
    blackboxCurrent->rssi = getRssi();

    blackboxCurrent->headspeed = lrintf(getHeadSpeed());
    blackboxCurrent->tailspeed = lrintf(getTailSpeed());

    for (int i = 0; i < getMotorCount(); i++) {
        blackboxCurrent->motor[i] = getMotorOutput(i);
    }

    for (int i = 0; i < getServoCount(); i++) {
        blackboxCurrent->servo[i] = getServoOutput(i);
    }

    for (int i = 0; i < DEBUG_VALUE_COUNT; i++) {
        blackboxCurrent->debug[i] = debug[i];
    }

#else
    UNUSED(currentTimeUs);
#endif // UNIT_TEST
}

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static bool sendFieldDefinition(char mainFrameChar, char deltaFrameChar, const void *fieldDefinitions,
        const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    unsigned int headerCount;
    static bool needComma = false;
    size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
    size_t conditionsStride = (char*) secondCondition - (char*) conditions;

    if (deltaFrameChar) {
        headerCount = BLACKBOX_DELTA_FIELD_HEADER_COUNT;
    } else {
        headerCount = BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;
    }

    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */

    // On our first call we need to print the name of the header and a colon
    if (xmitState.u.fieldIndex == -1) {
        if (xmitState.headerIndex >= headerCount) {
            return false; //Someone probably called us again after we had already completed transmission
        }

        uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[xmitState.headerIndex]);

        if (blackboxDeviceReserveBufferSpace(charsToBeWritten) != BLACKBOX_RESERVE_SUCCESS) {
            return true; // Try again later
        }

        blackboxHeaderBudget -= blackboxPrintf("H Field %c %s:", xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar : mainFrameChar, blackboxFieldHeaderNames[xmitState.headerIndex]);

        xmitState.u.fieldIndex++;
        needComma = false;
    }

    // The longest we expect an integer to be as a string:
    const uint32_t LONGEST_INTEGER_STRLEN = 2;

    for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
        def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
            // First (over)estimate the length of the string we want to print

            int32_t bytesToWrite = 1; // Leading comma

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                bytesToWrite += strlen(def->name) + strlen("[]") + LONGEST_INTEGER_STRLEN;
            } else {
                //The other headers are integers
                bytesToWrite += LONGEST_INTEGER_STRLEN;
            }

            // Now perform the write if the buffer is large enough
            if (blackboxDeviceReserveBufferSpace(bytesToWrite) != BLACKBOX_RESERVE_SUCCESS) {
                // Ran out of space!
                return true;
            }

            blackboxHeaderBudget -= bytesToWrite;

            if (needComma) {
                blackboxWrite(',');
            } else {
                needComma = true;
            }

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                blackboxWriteString(def->name);

                // Do we need to print an index in brackets after the name?
                if (def->fieldNameIndex != -1) {
                    blackboxPrintf("[%d]", def->fieldNameIndex);
                }
            } else {
                //The other headers are integers
                blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
            }
        }
    }

    // Did we complete this line?
    if (xmitState.u.fieldIndex == fieldCount && blackboxDeviceReserveBufferSpace(1) == BLACKBOX_RESERVE_SUCCESS) {
        blackboxHeaderBudget--;
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }

    return xmitState.headerIndex < headerCount;
}

// Buf must be at least FORMATTED_DATE_TIME_BUFSIZE
static char *blackboxGetStartDateTime(char *buf)
{
#ifdef USE_RTC_TIME
    dateTime_t dt;
    // rtcGetDateTime will fill dt with 0000-01-01T00:00:00
    // when time is not known.
    rtcGetDateTime(&dt);
    dateTimeFormatLocal(buf, &dt);
#else
    buf = "0000-01-01T00:00:00.000";
#endif

    return buf;
}

#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: \
                                                blackboxPrintfHeaderLine(name, format, __VA_ARGS__); \
                                                break;
#define BLACKBOX_PRINT_HEADER_LINE_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo(void)
{
#ifndef UNIT_TEST
    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    if (blackboxDeviceReserveBufferSpace(64) != BLACKBOX_RESERVE_SUCCESS) {
        return false;
    }

    char buf[128];  // datetime and rpm filter

    const controlRateConfig_t *currentControlRateProfile = controlRateProfiles(systemConfig()->activeRateProfile);
    switch (xmitState.headerIndex) {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "Rotorflight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "%s %s (%s) %s",    FC_FIRMWARE_NAME, FC_VERSION_STRING, shortGitRevision, targetName);
        BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                buildDate, buildTime);
#ifdef USE_BOARD_INFO
        BLACKBOX_PRINT_HEADER_LINE("Board information", "%s %s",            getManufacturerId(), getBoardName());
#endif
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              blackboxGetStartDateTime(buf));
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      pilotConfig()->name);
        BLACKBOX_PRINT_HEADER_LINE("I interval", "%d",                      blackboxIInterval);
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d",                      blackboxPInterval);
        BLACKBOX_PRINT_HEADER_LINE("P ratio", "%d",                         (uint16_t)(blackboxIInterval / blackboxPInterval));
        BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d",                     motorConfig()->minthrottle);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d",                     motorConfig()->maxthrottle);
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale","0x%x",                     castFloatBytesToInt(1.0f));
#if defined(USE_ACC)
        BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u",                          acc.dev.acc_1G);
#endif

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VOLTAGE)) {
                blackboxPrintfHeaderLine("vbat_scale", "%u", voltageSensorADCConfig(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale);
            } else {
                xmitState.headerIndex += 2; // Skip the next two vbat fields too
            }
            );

        BLACKBOX_PRINT_HEADER_LINE("vbatcellvoltage", "%u,%u,%u",           batteryConfig()->vbatmincellvoltage,
                                                                            batteryConfig()->vbatwarningcellvoltage,
                                                                            batteryConfig()->vbatmaxcellvoltage);
        BLACKBOX_PRINT_HEADER_LINE("vbatref", "%u",                         vbatReference);

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            if (batteryConfig()->currentMeterSource == CURRENT_METER_ADC) {
                blackboxPrintfHeaderLine("currentSensor", "%d,%d",currentSensorADCConfig()->offset, currentSensorADCConfig()->scale);
            }
            );

        BLACKBOX_PRINT_HEADER_LINE("looptime", "%d",                        gyro.sampleLooptime);
        BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom", "%d",                 1);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_PID_PROCESS_DENOM, "%d",      activePidLoopDenom);
        BLACKBOX_PRINT_HEADER_LINE("rc_rates", "%d,%d,%d",                  currentControlRateProfile->rcRates[ROLL],
                                                                            currentControlRateProfile->rcRates[PITCH],
                                                                            currentControlRateProfile->rcRates[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rc_expo", "%d,%d,%d",                   currentControlRateProfile->rcExpo[ROLL],
                                                                            currentControlRateProfile->rcExpo[PITCH],
                                                                            currentControlRateProfile->rcExpo[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rates", "%d,%d,%d",                     currentControlRateProfile->rates[ROLL],
                                                                            currentControlRateProfile->rates[PITCH],
                                                                            currentControlRateProfile->rates[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rate_limits", "%d,%d,%d",               currentControlRateProfile->rate_limit[ROLL],
                                                                            currentControlRateProfile->rate_limit[PITCH],
                                                                            currentControlRateProfile->rate_limit[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rollPID", "%d,%d,%d",                   currentPidProfile->pid[PID_ROLL].P,
                                                                            currentPidProfile->pid[PID_ROLL].I,
                                                                            currentPidProfile->pid[PID_ROLL].D);
        BLACKBOX_PRINT_HEADER_LINE("pitchPID", "%d,%d,%d",                  currentPidProfile->pid[PID_PITCH].P,
                                                                            currentPidProfile->pid[PID_PITCH].I,
                                                                            currentPidProfile->pid[PID_PITCH].D);
        BLACKBOX_PRINT_HEADER_LINE("yawPID", "%d,%d,%d",                    currentPidProfile->pid[PID_YAW].P,
                                                                            currentPidProfile->pid[PID_YAW].I,
                                                                            currentPidProfile->pid[PID_YAW].D);

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DEADBAND, "%d",               rcControlsConfig()->deadband);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_YAW_DEADBAND, "%d",           rcControlsConfig()->yaw_deadband);

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_TO_USE, "%d",            gyroConfig()->gyro_to_use);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_HARDWARE_LPF, "%d",      gyroConfig()->gyro_hardware_lpf);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF1_TYPE, "%d",         gyroConfig()->gyro_lpf1_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF1_STATIC_HZ, "%d",    gyroConfig()->gyro_lpf1_static_hz);
#ifdef USE_DYN_LPF
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf1_dyn_hz", "%d,%d",             gyroConfig()->gyro_lpf1_dyn_min_hz,
                                                                            gyroConfig()->gyro_lpf1_dyn_max_hz);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF2_TYPE, "%d",         gyroConfig()->gyro_lpf2_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_LPF2_STATIC_HZ, "%d",    gyroConfig()->gyro_lpf2_static_hz);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_hz", "%d,%d",                gyroConfig()->gyro_soft_notch_hz_1,
                                                                            gyroConfig()->gyro_soft_notch_hz_2);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_cutoff", "%d,%d",            gyroConfig()->gyro_soft_notch_cutoff_1,
                                                                            gyroConfig()->gyro_soft_notch_cutoff_2);

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF1_TYPE, "%d",        gyroConfig()->dterm_lpf1_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF1_STATIC_HZ, "%d",   gyroConfig()->dterm_lpf1_static_hz);
#ifdef USE_DYN_LPF
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf1_dyn_hz", "%d,%d",            gyroConfig()->dterm_lpf1_dyn_min_hz,
                                                                            gyroConfig()->dterm_lpf1_dyn_max_hz);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF2_TYPE, "%d",        gyroConfig()->dterm_lpf2_type);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_LPF2_STATIC_HZ, "%d",   gyroConfig()->dterm_lpf2_static_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_NOTCH_HZ, "%d",         gyroConfig()->dterm_notch_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DTERM_NOTCH_CUTOFF, "%d",     gyroConfig()->dterm_notch_cutoff);

#ifdef USE_DYN_NOTCH_FILTER
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_MAX_HZ, "%d",       dynNotchConfig()->dyn_notch_max_hz);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_COUNT, "%d",        dynNotchConfig()->dyn_notch_count);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_Q, "%d",            dynNotchConfig()->dyn_notch_q);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DYN_NOTCH_MIN_HZ, "%d",       dynNotchConfig()->dyn_notch_min_hz);
#endif
#ifdef USE_DSHOT_TELEMETRY
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DSHOT_BIDIR, "%d",            motorConfig()->dev.useDshotTelemetry);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MOTOR_POLES, "%d",            motorConfig()->motorPoleCount);
#endif
#ifdef USE_RPM_FILTER
        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            char *ptr = buf;
            for (int i=0; i<RPM_FILTER_BANK_COUNT; i++) {
                if (i > 0)
                    *ptr++ = ',';
                ptr += tfp_sprintf(ptr, "%d", rpmFilterConfig()->filter_bank_motor_index[i]);
            }
            blackboxPrintfHeaderLine("gyro_rpm_filter_bank_motor_index", buf);
        );
        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            char *ptr = buf;
            for (int i=0; i<RPM_FILTER_BANK_COUNT; i++) {
                if (i > 0)
                    *ptr++ = ',';
                ptr += tfp_sprintf(ptr, "%d", rpmFilterConfig()->filter_bank_gear_ratio[i]);
            }
            blackboxPrintfHeaderLine("gyro_rpm_filter_bank_gear_ratio", buf);
        );
        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            char *ptr = buf;
            for (int i=0; i<RPM_FILTER_BANK_COUNT; i++) {
                if (i > 0)
                    *ptr++ = ',';
                ptr += tfp_sprintf(ptr, "%d", rpmFilterConfig()->filter_bank_notch_q[i]);
            }
            blackboxPrintfHeaderLine("gyro_rpm_filter_bank_notch_q", buf);
        );
        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            char *ptr = buf;
            for (int i=0; i<RPM_FILTER_BANK_COUNT; i++) {
                if (i > 0)
                    *ptr++ = ',';
                ptr += tfp_sprintf(ptr, "%d", rpmFilterConfig()->filter_bank_min_hz[i]);
            }
            blackboxPrintfHeaderLine("gyro_rpm_filter_bank_min_hz", buf);
        );
        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            char *ptr = buf;
            for (int i=0; i<RPM_FILTER_BANK_COUNT; i++) {
                if (i > 0)
                    *ptr++ = ',';
                ptr += tfp_sprintf(ptr, "%d", rpmFilterConfig()->filter_bank_max_hz[i]);
            }
            blackboxPrintfHeaderLine("gyro_rpm_filter_bank_max_hz", buf);
        );
#endif
#if defined(USE_ACC)
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ACC_LPF_HZ, "%d",        (int)(accelerometerConfig()->acc_lpf_hz * 100.0f));
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_ACC_HARDWARE, "%d",            accelerometerConfig()->acc_hardware);
#endif
#ifdef USE_BARO
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_BARO_HARDWARE, "%d",           barometerConfig()->baro_hardware);
#endif
#ifdef USE_MAG
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MAG_HARDWARE, "%d",           compassConfig()->mag_hardware);
#endif
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_GYRO_CAL_ON_FIRST_ARM, "%d",  armingConfig()->gyro_cal_on_first_arm);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_SERIAL_RX_PROVIDER, "%d",     rxConfig()->serialrx_provider);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_USE_UNSYNCED_PWM, "%d",       motorConfig()->dev.useUnsyncedPwm);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MOTOR_PWM_PROTOCOL, "%d",     motorConfig()->dev.motorPwmProtocol);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_MOTOR_PWM_RATE, "%d",         motorConfig()->dev.motorPwmRate);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_DEBUG_MODE, "%d",             debugMode);
        BLACKBOX_PRINT_HEADER_LINE("features", "%d",                        featureConfig()->enabledFeatures);

        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_RATES_TYPE, "%d",             currentControlRateProfile->rates_type);

        BLACKBOX_PRINT_HEADER_LINE("fields_mask", "%d",                     blackboxConfig()->fields);

        default:
            return true;
    }

    xmitState.headerIndex++;
#endif // UNIT_TEST
    return false;
}

/**
 * Write the given event to the log immediately
 */
void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data)
{
    // Only allow events to be logged after headers have been written
    if (!(blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED)) {
        return;
    }

    //Shared header for event frames
    blackboxWrite('E');
    blackboxWrite(event);

    //Now serialize the data for this specific frame type
    switch (event) {
    case FLIGHT_LOG_EVENT_SYNC_BEEP:
        blackboxWriteUnsignedVB(data->syncBeep.time);
        break;
    case FLIGHT_LOG_EVENT_FLIGHTMODE:
        blackboxWriteUnsignedVB(data->flightMode.flags);
        blackboxWriteUnsignedVB(data->flightMode.lastFlags);
        break;
    case FLIGHT_LOG_EVENT_GOVSTATE:
        blackboxWriteUnsignedVB(data->govState.govState);
        break;
    case FLIGHT_LOG_EVENT_DISARM:
        blackboxWriteUnsignedVB(data->disarm.reason);
        break;
    case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
        if (data->inflightAdjustment.floatFlag) {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
            blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
        } else {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction);
            blackboxWriteSignedVB(data->inflightAdjustment.newValue);
        }
        break;
    case FLIGHT_LOG_EVENT_LOGGING_RESUME:
        blackboxWriteUnsignedVB(data->loggingResume.logIteration);
        blackboxWriteUnsignedVB(data->loggingResume.currentTime);
        break;
    case FLIGHT_LOG_EVENT_LOG_END:
        blackboxWriteString("End of log");
        blackboxWrite(0);
        break;
    default:
        break;
    }
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
static void blackboxCheckAndLogArmingBeep(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        flightLogEvent_syncBeep_t eventData;
        eventData.time = blackboxLastArmingBeep;
        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, (flightLogEventData_t *)&eventData);
    }
}

/* monitor the flight mode event status and trigger an event record if the state changes */
static void blackboxCheckAndLogFlightMode(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (memcmp(&rcModeActivationMask, &blackboxLastFlightModeFlags, sizeof(blackboxLastFlightModeFlags))) {
        flightLogEvent_flightMode_t eventData; // Add new data for current flight mode flags
        eventData.lastFlags = blackboxLastFlightModeFlags;
        memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags));
        memcpy(&eventData.flags, &rcModeActivationMask, sizeof(eventData.flags));
        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE, (flightLogEventData_t *)&eventData);
    }

    if (getGovernorState() != blackboxLastGovState) {
        blackboxLastGovState = getGovernorState();
        flightLogEvent_govState_t eventData;
        eventData.govState = blackboxLastGovState;
        blackboxLogEvent(FLIGHT_LOG_EVENT_GOVSTATE, (flightLogEventData_t *)&eventData);
    }
}

static bool blackboxShouldLogPFrame(void)
{
    return blackboxPFrameIndex == 0 && blackboxPInterval != 0;
}

static bool blackboxShouldLogIFrame(void)
{
    return blackboxLoopIndex == 0;
}

/*
 * If the GPS home point has been updated, or every 128 I-frames (~10 seconds), write the
 * GPS home position.
 *
 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
 * still be interpreted correctly.
 */
#ifdef USE_GPS
static bool blackboxShouldLogGpsHomeFrame(void)
{
    if ((GPS_home[0] != gpsHistory.GPS_home[0] || GPS_home[1] != gpsHistory.GPS_home[1]
        || (blackboxPFrameIndex == blackboxIInterval / 2 && blackboxIFrameIndex % 128 == 0)) && isFieldEnabled(FIELD_SELECT(GPS))) {
        return true;
    }
    return false;
}
#endif // GPS

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
static void blackboxAdvanceIterationTimers(void)
{
    ++blackboxSlowFrameIterationTimer;
    ++blackboxIteration;

    if (++blackboxLoopIndex >= blackboxIInterval) {
        blackboxLoopIndex = 0;
        blackboxIFrameIndex++;
        blackboxPFrameIndex = 0;
    } else if (++blackboxPFrameIndex >= blackboxPInterval) {
        blackboxPFrameIndex = 0;
    }
}

// Called once every FC loop in order to log the current state
static void blackboxLogIteration(timeUs_t currentTimeUs)
{
    // Write a keyframe every blackboxIInterval frames so we can resynchronise upon missing frames
    if (blackboxShouldLogIFrame()) {
        /*
         * Don't log a slow frame if the slow data didn't change ("I" frames are already large enough without adding
         * an additional item to write at the same time). Unless we're *only* logging "I" frames, then we have no choice.
         */
        if (blackboxIsOnlyLoggingIntraframes()) {
            writeSlowFrameIfNeeded();
        }

        loadMainState(currentTimeUs);
        writeIntraframe();
    } else {
        blackboxCheckAndLogArmingBeep();
        blackboxCheckAndLogFlightMode(); // Check for FlightMode status change event

        if (blackboxShouldLogPFrame()) {
            /*
             * We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
             * So only log slow frames during loop iterations where we log a main frame.
             */
            writeSlowFrameIfNeeded();

            loadMainState(currentTimeUs);
            writeInterframe();
        }
#ifdef USE_GPS
        if (featureIsEnabled(FEATURE_GPS) && isFieldEnabled(FIELD_SELECT(GPS))) {
            if (blackboxShouldLogGpsHomeFrame()) {
                writeGPSHomeFrame();
                writeGPSFrame(currentTimeUs);
            } else if (gpsSol.numSat != gpsHistory.GPS_numSat
                    || gpsSol.llh.lat != gpsHistory.GPS_coord[GPS_LATITUDE]
                    || gpsSol.llh.lon != gpsHistory.GPS_coord[GPS_LONGITUDE]) {
                //We could check for velocity changes as well but I doubt it changes independent of position
                writeGPSFrame(currentTimeUs);
            }
        }
#endif
    }

    //Flush every iteration so that our runtime variance is minimized
    blackboxDeviceFlush();
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
void blackboxUpdate(timeUs_t currentTimeUs)
{
    static BlackboxState cacheFlushNextState;

    blackboxCheckEnabler();

    switch (blackboxState) {
    case BLACKBOX_STATE_STOPPED:
        if (blackboxIsLoggingEnabled()) {
            blackboxOpen();
            blackboxStart();
        }
#ifdef USE_FLASHFS
        if (IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            blackboxSetState(BLACKBOX_STATE_START_ERASE);
        }
#endif
        break;
    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        if (blackboxDeviceBeginLog()) {
            blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

        /*
         * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
         * buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
         */
        if (millis() > xmitState.u.startTime + 100) {
            if (blackboxDeviceReserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BLACKBOX_RESERVE_SUCCESS) {
                for (int i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
                    blackboxWrite(blackboxHeader[xmitState.headerIndex]);
                    blackboxHeaderBudget--;
                }
                if (blackboxHeader[xmitState.headerIndex] == '\0') {
                    blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                }
            }
        }
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAYLEN(blackboxMainFields),
                &blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
#ifdef USE_GPS
            if (featureIsEnabled(FEATURE_GPS) && isFieldEnabled(FIELD_SELECT(GPS))) {
                blackboxSetState(BLACKBOX_STATE_SEND_GPS_H_HEADER);
            } else
#endif
                blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;
#ifdef USE_GPS
    case BLACKBOX_STATE_SEND_GPS_H_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('H', 0, blackboxGpsHFields, blackboxGpsHFields + 1, ARRAYLEN(blackboxGpsHFields),
                NULL, NULL) && isFieldEnabled(FIELD_SELECT(GPS))) {
            blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('G', 0, blackboxGpsGFields, blackboxGpsGFields + 1, ARRAYLEN(blackboxGpsGFields),
                &blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition) && isFieldEnabled(FIELD_SELECT(GPS))) {
            blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;
#endif
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAYLEN(blackboxSlowFields),
                NULL, NULL)) {
            cacheFlushNextState = BLACKBOX_STATE_SEND_SYSINFO;
            blackboxSetState(BLACKBOX_STATE_CACHE_FLUSH);
        }
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0

        //Keep writing chunks of the system info headers until it returns true to signal completion
        if (blackboxWriteSysinfo()) {
            /*
             * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
             * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
             * could wipe out the end of the header if we weren't careful)
             */
            cacheFlushNextState = BLACKBOX_STATE_RUNNING;
            blackboxSetState(BLACKBOX_STATE_CACHE_FLUSH);
        }
        break;
    case BLACKBOX_STATE_CACHE_FLUSH:
        // Flush the cache and wait until all possible entries have been written to the media
        if (blackboxDeviceFlushForceComplete()) {
            blackboxSetState(cacheFlushNextState);
        }
        break;
    case BLACKBOX_STATE_PAUSED:
        // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
        if (!blackboxIsLoggingPaused() && blackboxShouldLogIFrame()) {
            // Write a log entry so the decoder is aware that our large time/iteration skip is intended
            flightLogEvent_loggingResume_t resume;

            resume.logIteration = blackboxIteration;
            resume.currentTime = currentTimeUs;

            blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t *) &resume);
            blackboxSetState(BLACKBOX_STATE_RUNNING);

            blackboxLogIteration(currentTimeUs);
        }
        // Keep the logging timers ticking so our log iteration continues to advance
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_RUNNING:
        // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
        if (blackboxIsLoggingPaused()) {
            blackboxSetState(BLACKBOX_STATE_PAUSED);
        } else {
            blackboxLogIteration(currentTimeUs);
        }
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        //On entry of this state, startTime is set
        /*
         * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
         * since releasing the port clears the Tx buffer.
         *
         * Don't wait longer than it could possibly take if something funky happens.
         */
        if (blackboxDeviceEndLog(blackboxLoggedAnyFrames) && (millis() > xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || blackboxDeviceFlushForce())) {
            blackboxDeviceClose();
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        }
        break;
#ifdef USE_FLASHFS
    case BLACKBOX_STATE_START_ERASE:
        blackboxEraseAll();
        blackboxSetState(BLACKBOX_STATE_ERASING);
        beeper(BEEPER_BLACKBOX_ERASE);
        break;
    case BLACKBOX_STATE_ERASING:
        if (isBlackboxErased()) {
            //Done erasing
            blackboxSetState(BLACKBOX_STATE_ERASED);
            beeper(BEEPER_BLACKBOX_ERASE);
        }
        break;
    case BLACKBOX_STATE_ERASED:
        if (!IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        }
    break;
#endif
    default:
        break;
    }

    // Did we run out of room on the device? Stop!
    if (isBlackboxDeviceFull()) {
#ifdef USE_FLASHFS
        if (blackboxState != BLACKBOX_STATE_ERASING
            && blackboxState != BLACKBOX_STATE_START_ERASE
            && blackboxState != BLACKBOX_STATE_ERASED)
#endif
        {
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        }
    }
}

int blackboxCalculatePDenom(int rateNum, int rateDenom)
{
    return blackboxIInterval * rateNum / rateDenom;
}

uint8_t blackboxGetRateDenom(void)
{
    return blackboxPInterval;

}

uint16_t blackboxGetPRatio(void) {
    return blackboxIInterval / blackboxPInterval;
}

uint8_t blackboxCalculateSampleRate(uint16_t pRatio) {
    return LOG2(32000 / (gyro.targetLooptime * pRatio));
}

/**
 * Call during system startup to initialize the blackbox.
 */
void blackboxInit(void)
{
    blackboxResetIterationTimers();

    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    // targetPidLooptime is 1000 for 1kHz loop, 500 for 2kHz loop etc, targetPidLooptime is rounded for short looptimes
    blackboxIInterval = (uint16_t)(32 * 1000 / gyro.targetLooptime);

    blackboxPInterval = 1 << blackboxConfig()->sample_rate;
    if (blackboxPInterval > blackboxIInterval) {
        blackboxPInterval = 0; // log only I frames if logging frequency is too low
    }

    if (blackboxConfig()->device) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    } else {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
    }
    blackboxSInterval = blackboxIInterval * 256; // S-frame is written every 256*32 = 8192ms, approx every 8 seconds
}
#endif

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

#include "platform.h"

#include "common/time.h"

#include "pg/esc_sensor.h"

typedef struct {
    uint8_t   age;              // Data age
    uint16_t  pwm;              // Output duty cycle 0.1%
    uint16_t  throttle;         // Input setpoint 0.1%
    uint32_t  erpm;             // eRPM
    uint32_t  voltage;          // mV
    uint32_t  current;          // mA
    uint32_t  consumption;      // mAh
    int16_t   temperature;      // 0.1°C
    int16_t   temperature2;     // 0.1°C
    uint32_t  bec_voltage;      // mV
    uint32_t  bec_current;      // mA
    uint32_t  status;           // status / fault codes
    uint8_t   id;               // ESC id / flags
} escSensorData_t;

typedef enum {
    ESC_WRITE_STATE_IDLE = 0,
    ESC_WRITE_STATE_QUEUED,
    ESC_WRITE_STATE_RUNNING,
    ESC_WRITE_STATE_VERIFYING,
    ESC_WRITE_STATE_DONE,
    ESC_WRITE_STATE_FAILED,
} escWriteState_e;

typedef enum {
    ESC_WRITE_ERROR_NONE = 0,
    ESC_WRITE_ERROR_INVALID,
    ESC_WRITE_ERROR_ARMED,
    ESC_WRITE_ERROR_BUSY,
    ESC_WRITE_ERROR_IO,
    ESC_WRITE_ERROR_VERIFY,
    ESC_WRITE_ERROR_TIMEOUT,
} escWriteError_e;

enum {
    ESC_INFO_FLAG_ACTIVE = (1 << 0),
    ESC_INFO_FLAG_SELECTED = (1 << 1),
    ESC_INFO_FLAG_NAME_GENERIC = (1 << 2),
    ESC_INFO_FLAG_MODEL_GENERIC = (1 << 3),
};

enum {
    ESC_DETAIL_FLAG_VERSION_GENERIC = (1 << 0),
    ESC_DETAIL_FLAG_FIRMWARE_GENERIC = (1 << 1),
};

enum {
    ESC_CAP_TELEMETRY = (1 << 0),
    ESC_CAP_PARAM_READ = (1 << 1),
    ESC_CAP_PARAM_WRITE = (1 << 2),
    ESC_CAP_FORWARD_PROGRAMMING = (1 << 3),
};

#define ESC_MSP_PARAM_CHUNK_SIZE 64

typedef struct {
    uint8_t escId;
    uint8_t protocol;
    uint8_t signature;
    uint8_t flags;
    uint8_t capabilities;
    uint8_t parameterBytes;
    uint8_t maxChunkSize;
} escInfo_t;

typedef struct {
    uint16_t opId;
    uint8_t escId;
    uint8_t protocol;
    uint8_t signature;
    uint8_t state;
    uint8_t error;
} escWriteStatus_t;

typedef struct {
    uint8_t escId;
    uint8_t totalLength;
    uint8_t offset;
    uint8_t chunkLength;
} escParamChunk_t;

#define ESC_DATA_INVALID 255

#define ESC_BATTERY_AGE_MAX 10

#define ESC_SENSOR_COMBINED 255

bool escSensorInit(void);
void escSensorProcess(timeUs_t currentTime);

void validateAndFixEscSensorConfig(void);

bool isEscSensorActive(void);

uint32_t getEscSensorRPM(uint8_t motorNumber);
escSensorData_t *getEscSensorData(uint8_t motorNumber);
bool escGetInfo(uint8_t requestedEscId, escInfo_t *info);
bool escGetNameInfo(uint8_t requestedEscId, const char **name, const char **model, uint8_t *flags);
bool escGetDetailInfo(uint8_t requestedEscId, const char **version, const char **firmware, uint8_t *flags);
void escGetWriteStatus(escWriteStatus_t *status);
bool escReadParamChunk(uint8_t requestedEscId, uint8_t offset, uint8_t maxLength, escParamChunk_t *chunk, uint8_t *data);
bool escBeginParamWrite(uint8_t requestedEscId);
bool escWriteParamChunk(uint8_t requestedEscId, uint8_t offset, const uint8_t *data, uint8_t length);
bool escCommitStagedParamWrite(uint8_t requestedEscId);

uint8_t escGetParamBufferLength(void);
uint8_t escSelect4WIfById(uint8_t id);
uint8_t *escGetParamBuffer(void);
uint8_t *escGetParamUpdBuffer(void);
bool escCommitParameters(void);

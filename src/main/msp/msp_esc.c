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
 * along with Rotorflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "msp/msp_esc.h"
#include "msp/msp_protocol_v2_betaflight.h"

#include "sensors/esc_sensor.h"

#define MSP_ESC_TEXT_LIMIT 31

#if defined(USE_ESC_SENSOR)
typedef struct {
    uint8_t escId;
    uint8_t offset;
    uint8_t length;
} mspEscChunkRequest_t;

static mspResult_e mspEscReadOptionalEscId(sbuf_t *src, uint8_t *escId)
{
    const int bytesRemaining = sbufBytesRemaining(src);

    if (bytesRemaining == 0) {
        *escId = ESC_SENSOR_COMBINED;
        return MSP_RESULT_ACK;
    }

    if (bytesRemaining != 1) {
        return MSP_RESULT_ERROR;
    }

    *escId = sbufReadU8(src);
    return MSP_RESULT_ACK;
}

static uint8_t mspEscClampStringLength(const char *value)
{
    if (!value) {
        return 0;
    }

    const size_t length = strlen(value);
    return length > MSP_ESC_TEXT_LIMIT ? MSP_ESC_TEXT_LIMIT : length;
}

static mspResult_e mspEscReadChunkRequest(sbuf_t *src, mspEscChunkRequest_t *request)
{
    if (!request || sbufBytesRemaining(src) < 3) {
        return MSP_RESULT_ERROR;
    }

    request->escId = sbufReadU8(src);
    request->offset = sbufReadU8(src);
    request->length = sbufReadU8(src);

    if (request->length == 0 || request->length > ESC_MSP_PARAM_CHUNK_SIZE) {
        return MSP_RESULT_ERROR;
    }

    return MSP_RESULT_ACK;
}
#endif

mspResult_e mspEscProcessCommand(int16_t cmdMSP, sbuf_t *src, sbuf_t *dst)
{
#if defined(USE_ESC_SENSOR)
    switch (cmdMSP) {
    case MSP2_GET_ESC_INFO:
    {
        uint8_t escId = ESC_SENSOR_COMBINED;
        escInfo_t info;

        const mspResult_e status = mspEscReadOptionalEscId(src, &escId);
        if (status != MSP_RESULT_ACK) {
            return status;
        }

        if (!escGetInfo(escId, &info)) {
            return MSP_RESULT_ERROR;
        }

        sbufWriteU8(dst, info.escId);
        sbufWriteU8(dst, info.protocol);
        sbufWriteU8(dst, info.signature);
        sbufWriteU8(dst, info.flags);
        sbufWriteU8(dst, info.capabilities);
        sbufWriteU8(dst, info.parameterBytes);
        sbufWriteU8(dst, info.maxChunkSize);
        return MSP_RESULT_ACK;
    }

    case MSP2_GET_ESC_NAME:
    {
        uint8_t escId = ESC_SENSOR_COMBINED;
        escInfo_t info;
        uint8_t flags = 0;
        const char *name = NULL;
        const char *model = NULL;

        const mspResult_e status = mspEscReadOptionalEscId(src, &escId);
        if (status != MSP_RESULT_ACK) {
            return status;
        }

        if (!escGetInfo(escId, &info)) {
            return MSP_RESULT_ERROR;
        }
        if (!escGetNameInfo(escId, &name, &model, &flags)) {
            return MSP_RESULT_ERROR;
        }

        const uint8_t nameLength = mspEscClampStringLength(name);
        const uint8_t modelLength = mspEscClampStringLength(model);

        sbufWriteU8(dst, info.escId);
        sbufWriteU8(dst, flags);
        sbufWriteU8(dst, nameLength);
        if (nameLength > 0) {
            sbufWriteData(dst, name, nameLength);
        }
        sbufWriteU8(dst, modelLength);
        if (modelLength > 0) {
            sbufWriteData(dst, model, modelLength);
        }
        return MSP_RESULT_ACK;
    }

    case MSP2_GET_ESC_DETAILS:
    {
        uint8_t escId = ESC_SENSOR_COMBINED;
        escInfo_t info;
        uint8_t flags = 0;
        const char *version = NULL;
        const char *firmware = NULL;

        const mspResult_e status = mspEscReadOptionalEscId(src, &escId);
        if (status != MSP_RESULT_ACK) {
            return status;
        }

        if (!escGetInfo(escId, &info)) {
            return MSP_RESULT_ERROR;
        }
        if (!escGetDetailInfo(escId, &version, &firmware, &flags)) {
            return MSP_RESULT_ERROR;
        }

        const uint8_t versionLength = mspEscClampStringLength(version);
        const uint8_t firmwareLength = mspEscClampStringLength(firmware);

        sbufWriteU8(dst, info.escId);
        sbufWriteU8(dst, flags);
        sbufWriteU8(dst, versionLength);
        if (versionLength > 0) {
            sbufWriteData(dst, version, versionLength);
        }
        sbufWriteU8(dst, firmwareLength);
        if (firmwareLength > 0) {
            sbufWriteData(dst, firmware, firmwareLength);
        }
        return MSP_RESULT_ACK;
    }

    case MSP2_GET_ESC_WRITE_STATUS:
    {
        escWriteStatus_t status;

        if (sbufBytesRemaining(src) != 0) {
            return MSP_RESULT_ERROR;
        }

        escGetWriteStatus(&status);

        sbufWriteU16(dst, status.opId);
        sbufWriteU8(dst, status.escId);
        sbufWriteU8(dst, status.protocol);
        sbufWriteU8(dst, status.signature);
        sbufWriteU8(dst, status.state);
        sbufWriteU8(dst, status.error);
        return MSP_RESULT_ACK;
    }

    case MSP2_GET_ESC_PARAM_DATA:
    {
        mspEscChunkRequest_t request;
        escParamChunk_t chunk;
        uint8_t buffer[ESC_MSP_PARAM_CHUNK_SIZE];

        const mspResult_e status = mspEscReadChunkRequest(src, &request);
        if (status != MSP_RESULT_ACK) {
            return status;
        }
        if (sbufBytesRemaining(src) != 0) {
            return MSP_RESULT_ERROR;
        }

        if (!escReadParamChunk(request.escId, request.offset, request.length, &chunk, buffer)) {
            return MSP_RESULT_ERROR;
        }

        sbufWriteU8(dst, chunk.escId);
        sbufWriteU8(dst, chunk.totalLength);
        sbufWriteU8(dst, chunk.offset);
        sbufWriteU8(dst, chunk.chunkLength);
        sbufWriteData(dst, buffer, chunk.chunkLength);
        return MSP_RESULT_ACK;
    }

    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
#else
    UNUSED(src);
    UNUSED(dst);

    switch (cmdMSP) {
    case MSP2_GET_ESC_INFO:
    case MSP2_GET_ESC_NAME:
    case MSP2_GET_ESC_DETAILS:
    case MSP2_GET_ESC_WRITE_STATUS:
    case MSP2_GET_ESC_PARAM_DATA:
        return MSP_RESULT_ERROR;
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
#endif
}

mspResult_e mspEscProcessSetCommand(int16_t cmdMSP, sbuf_t *src)
{
#if defined(USE_ESC_SENSOR)
    switch (cmdMSP) {
    case MSP2_SET_ESC_PARAM_BEGIN:
    {
        if (sbufBytesRemaining(src) != 1) {
            return MSP_RESULT_ERROR;
        }

        return escBeginParamWrite(sbufReadU8(src)) ? MSP_RESULT_ACK : MSP_RESULT_ERROR;
    }

    case MSP2_SET_ESC_PARAM_DATA:
    {
        mspEscChunkRequest_t request;
        uint8_t buffer[ESC_MSP_PARAM_CHUNK_SIZE];

        const mspResult_e status = mspEscReadChunkRequest(src, &request);
        if (status != MSP_RESULT_ACK || sbufBytesRemaining(src) != request.length) {
            return MSP_RESULT_ERROR;
        }

        sbufReadData(src, buffer, request.length);
        return escWriteParamChunk(request.escId, request.offset, buffer, request.length) ? MSP_RESULT_ACK : MSP_RESULT_ERROR;
    }

    case MSP2_SET_ESC_PARAM_COMMIT:
    {
        if (sbufBytesRemaining(src) != 1) {
            return MSP_RESULT_ERROR;
        }

        return escCommitStagedParamWrite(sbufReadU8(src)) ? MSP_RESULT_ACK : MSP_RESULT_ERROR;
    }

    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
#else
    UNUSED(src);

    switch (cmdMSP) {
    case MSP2_SET_ESC_PARAM_BEGIN:
    case MSP2_SET_ESC_PARAM_DATA:
    case MSP2_SET_ESC_PARAM_COMMIT:
        return MSP_RESULT_ERROR;
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
#endif
}

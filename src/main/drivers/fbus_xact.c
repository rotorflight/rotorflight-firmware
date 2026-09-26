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

#include "platform.h"

#include "fbus_master.h"
#include "fbus_xact.h"

#include <stdbool.h>
#include <string.h>

#include "common/time.h"
#include "common/utils.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "rx/frsky_crc.h"
#include "rx/fbus.h"

// XACT read/write frames ride on the same downlink frame used for telemetry polling.
#define FBUS_FRAME_ID_DATA 0x10

// XACT module state
static bool xactInitialized = false;

// XACT servo programming queue
#define XACT_QUEUE_SIZE 8
static xactServoParam_t xactQueue[XACT_QUEUE_SIZE];
static uint8_t xactQueueHead = 0;
static uint8_t xactQueueTail = 0;
static uint8_t xactQueueCount = 0;

// XACT servo tracking (separate from general sensor discovery)
static xactServo_t xactServos[XACT_MAX_SERVOS];
static uint8_t xactServoCount = 0;

// XACT servo parameter storage (indexed by physical ID)
static xactServoParams_t xactServoParams[XACT_MAX_SERVOS];

// Parameter reading state machine
typedef enum {
    XACT_READ_STATE_IDLE = 0,
    XACT_READ_STATE_READING,
    XACT_READ_STATE_WAIT_POLL,  // Wait to send 0x10 poll frame after 0x30 read
    XACT_READ_STATE_COMPLETE
} xactReadState_e;

static xactReadState_e xactReadState = XACT_READ_STATE_IDLE;
static uint8_t xactReadServoIndex = 0;
static uint8_t xactReadParamIndex = 0;
static uint8_t xactLastReadPhyID = 0;  // Track last read physical ID for polling
static uint8_t xactLastReadFieldId = 0;  // Track last read field ID for response matching

// Not every field is supported by every servo (e.g. older units may not answer a Firmware
// Version or Working Mode/Max Angle read at all). Give a field this long to answer before
// skipping it and moving on, rather than blocking the rest of the read forever -- mirrors
// FrSky's own "XAct" ETHOS tool giving up on an unresponsive field after a few tries.
#define XACT_FIELD_READ_TIMEOUT_US 200000  // 200ms
static timeUs_t xactFieldReadStartUs = 0;

// Field IDs every XACT servo supports, read in order. Firmware version is read early (matching
// FrSky's own tool) since it decides whether the "series 65" extended fields below get read too.
static const uint8_t xactReadFieldIdsBase[] = {
    XACT_FIELD_PHYSICAL_ID,
    XACT_FIELD_APP_ID_BASE,
    XACT_FIELD_FIRMWARE_VERSION,
    XACT_FIELD_DATA_RATE,
    XACT_FIELD_RANGE,
    XACT_FIELD_DIRECTION,
    XACT_FIELD_PULSE_TYPE,
    XACT_FIELD_CHANNEL,
    XACT_FIELD_CENTER,
    XACT_FIELD_HOLDING_STRENGTH,
    XACT_FIELD_OPERATION_SMOOTHING,
    XACT_FIELD_DEADBAND,
};
#define XACT_READ_PARAM_COUNT_BASE (sizeof(xactReadFieldIdsBase) / sizeof(xactReadFieldIdsBase[0]))

// Only read for "series 65"+ servos (firmwareVersion >= XACT_SERIES65_MIN_FIRMWARE)
static const uint8_t xactReadFieldIdsExtended[] = {
    XACT_FIELD_WORKING_MODE,
    XACT_FIELD_MAX_ANGLE,
};
#define XACT_READ_PARAM_COUNT_EXTENDED (sizeof(xactReadFieldIdsExtended) / sizeof(xactReadFieldIdsExtended[0]))

// Total number of fields to read for a servo, once its firmware version (read early in the
// base list) tells us whether the extended fields apply
static uint8_t xactReadTotalParamCount(uint8_t servoIndex)
{
    uint8_t count = XACT_READ_PARAM_COUNT_BASE;
    if (xactServoParams[servoIndex].hasExtendedParams) {
        count += XACT_READ_PARAM_COUNT_EXTENDED;
    }
    return count;
}

// Field ID for a given position across the base+extended read sequence
static uint8_t xactReadFieldIdAt(uint8_t index)
{
    if (index < XACT_READ_PARAM_COUNT_BASE) {
        return xactReadFieldIdsBase[index];
    }
    return xactReadFieldIdsExtended[index - XACT_READ_PARAM_COUNT_BASE];
}

// Move past the current field (whether it was actually answered or timed out) and either start
// reading the next one, jump to another discovered servo that hasn't been read yet, or mark
// idle. Shared by the success path (fbusXactNotifyResponse) and the per-field timeout path in
// fbusXactProcessQueue.
static void xactAdvanceReadField(void)
{
    xactReadParamIndex++;

    if (xactReadParamIndex < xactReadTotalParamCount(xactReadServoIndex)) {
        xactReadState = XACT_READ_STATE_READING;
        return;
    }

    xactServos[xactReadServoIndex].paramsReady = true;

    // Keep going: read any other discovered servo that hasn't been read yet, so the servo
    // list can show identifying details (e.g. Channel) for every servo found, not just
    // whichever one happens to be selected.
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (!xactServos[i].paramsReady) {
            xactReadServoIndex = i;
            xactReadParamIndex = 0;
            xactReadState = XACT_READ_STATE_READING;
            return;
        }
    }

    xactReadState = XACT_READ_STATE_COMPLETE;
}

// Helper function to fill physical ID check bits
static void xactPhyIDFillCheckBits(uint8_t *phyIDByte)
{
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 1) ^ fbusGetBit(*phyIDByte, 2)) << 5;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 3) ^ fbusGetBit(*phyIDByte, 4)) << 6;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 4)) << 7;
}

// Initialize XACT servo module
void fbusXactInit(void)
{
    xactInitialized = true;
    xactQueueHead = 0;
    xactQueueTail = 0;
    xactQueueCount = 0;
    memset(xactQueue, 0, sizeof(xactQueue));
    memset(xactServos, 0, sizeof(xactServos));
    memset(xactServoParams, 0, sizeof(xactServoParams));
    xactServoCount = 0;
    xactReadState = XACT_READ_STATE_IDLE;
}

// Clear all discovered XACT servos
void fbusXactClearDiscoveredServos(void)
{
    memset(xactServos, 0, sizeof(xactServos));
    xactServoCount = 0;
    xactReadState = XACT_READ_STATE_IDLE;

    // Restart FBUS master sensor discovery so it re-scans physical IDs
    fbusMasterStartDiscovery();
}

// Track an XACT servo (called when servo data is received)
void fbusXactTrackServo(uint8_t phyID, uint16_t appId, timeUs_t currentTimeUs)
{
    // Check if this servo is already tracked
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            // A different App ID showing up for a Physical ID we already track means two
            // servos are answering the same bus address -- flag it rather than silently
            // overwriting one servo's identity with the other's.
            if (xactServos[i].appId != appId) {
                xactServos[i].appIdConflict = true;
            }
            xactServos[i].appId = appId;
            xactServos[i].lastSeenUs = currentTimeUs;
            return;
        }
    }

    // Add new servo if space available
    if (xactServoCount < XACT_MAX_SERVOS) {
        xactServos[xactServoCount].phyID = phyID;
        xactServos[xactServoCount].appId = appId;
        xactServos[xactServoCount].lastSeenUs = currentTimeUs;
        xactServos[xactServoCount].paramsReady = false;
        xactServos[xactServoCount].appIdConflict = false;
        const uint8_t newServoIndex = xactServoCount;
        xactServoCount++;

        // Opportunistically read every discovered servo's parameters in the background (not
        // just whichever one gets selected) so the GUI's servo list can show identifying
        // details like Channel for all of them. Only start immediately if the read pipeline
        // is free -- if it's already busy reading a different servo, xactAdvanceReadField()
        // picks this one up as soon as that one finishes.
        if (xactReadState == XACT_READ_STATE_IDLE || xactReadState == XACT_READ_STATE_COMPLETE) {
            xactReadState = XACT_READ_STATE_READING;
            xactReadServoIndex = newServoIndex;
            xactReadParamIndex = 0;
        }
    }
}

// Start a new sensor discovery phase
void fbusXactStartSensorDiscovery(void)
{
    // Clear existing discovered XACT servos and restart FBUS master scanning
    fbusXactClearDiscoveredServos();
}

// Write a parameter to an XACT servo using physical ID
bool fbusXactWriteUplinkFramePhyID(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data)
{
    if (!xactInitialized) {
        return false;
    }

    // Check if queue is full
    if (xactQueueCount >= XACT_QUEUE_SIZE) {
        return false;
    }

    // Add to queue
    xactQueue[xactQueueTail].phyID = phyID;
    xactQueue[xactQueueTail].fieldId = fieldId;
    xactQueue[xactQueueTail].appId = appId;
    xactQueue[xactQueueTail].data = data;

    xactQueueTail = (xactQueueTail + 1) % XACT_QUEUE_SIZE;
    xactQueueCount++;

    return true;
}

// Get the number of discovered XACT servos
uint8_t fbusXactGetDiscoveredServoCount(void)
{
    return xactServoCount;
}

// Get a discovered XACT servo physical ID by index
uint8_t fbusXactGetDiscoveredServoPhyID(uint8_t index)
{
    if (index >= xactServoCount) {
        return 0;
    }
    return xactServos[index].phyID;
}

// Check if XACT module is initialized
bool fbusXactIsInitialized(void)
{
    return xactInitialized;
}

// Process XACT servo programming queue (called from FBUS master update)
// Frame format: Length PhysID PRIM FIELDID APPID2 APPID1 DATA1 CRC
bool fbusXactProcessQueue(fbusMasterDownlink_t *downlink)
{
    if (!xactInitialized) {
        return false;
    }

    // No XACT traffic while armed: writes queued just before arming and background reads wait
    // until disarm, and the downlink slot stays with telemetry polling. A read waiting for its
    // answer is sent again after disarm rather than timing out and being skipped.
    if (ARMING_FLAG(ARMED)) {
        if (xactReadState == XACT_READ_STATE_WAIT_POLL) {
            xactReadState = XACT_READ_STATE_READING;
        }
        return false;
    }

    // Get servo pointer for both read and poll operations
    xactServo_t *servo = (xactServoCount > 0) ? &xactServos[xactReadServoIndex] : NULL;

    // Priority 1: Check if we need to send 0x10 poll frame after 0x30 read
    if (xactReadState == XACT_READ_STATE_WAIT_POLL && servo != NULL) {
        // This field hasn't answered in time -- skip it (leaving its cached value at whatever
        // it already was, typically 0) and move on rather than blocking every later field.
        if (cmpTimeUs(micros(), xactFieldReadStartUs) > XACT_FIELD_READ_TIMEOUT_US) {
            xactAdvanceReadField();
            return false;
        }

        // Send 0x10 DATA frame to poll for the response
        downlink->length = 0x08;
        downlink->phyID = xactLastReadPhyID;
        downlink->prim = FBUS_FRAME_ID_DATA;  // 0x10 to poll for response
        downlink->appId = servo->appId;       // Include servo App ID
        downlink->data[0] = 0;
        downlink->data[1] = 0;
        downlink->data[2] = 0;
        downlink->data[3] = 0;

        xactPhyIDFillCheckBits(&downlink->phyID);
        uint8_t crc = frskyCheckSum((uint8_t *)&downlink->phyID, 0x08);
        downlink->crc = crc;

        // Stay in WAIT_POLL state - will be advanced by fbusXactNotifyResponse()

        return true;
    }

    // Priority 2: Check if we need to read parameters
    if (xactReadState == XACT_READ_STATE_READING && servo != NULL) {
        if (xactReadParamIndex < xactReadTotalParamCount(xactReadServoIndex)) {
            // Send READ command for next parameter
            uint8_t fieldId = xactReadFieldIdAt(xactReadParamIndex);

            downlink->length = 0x08;
            downlink->phyID = servo->phyID;
            downlink->prim = XACT_FRAME_TYPE_READ;  // 0x30 for read
            downlink->appId = servo->appId;         // Use appId field for App ID
            downlink->data[0] = fieldId;            // Field ID in data[0]
            downlink->data[1] = 0;                  // Unused
            downlink->data[2] = 0;                  // Unused for read
            downlink->data[3] = 0;                  // Unused

            xactPhyIDFillCheckBits(&downlink->phyID);
            uint8_t crc = frskyCheckSum((uint8_t *)&downlink->phyID, 0x08);
            downlink->crc = crc;

            // Save the physical ID and field ID for the next poll frame
            xactLastReadPhyID = servo->phyID;
            xactLastReadFieldId = fieldId;
            xactFieldReadStartUs = micros();

            // Transition to wait for poll state (don't increment param index yet)
            xactReadState = XACT_READ_STATE_WAIT_POLL;

            return true;
        }
    }

    // Priority 3: Process write queue
    if (xactQueueCount > 0) {
        // Get next item from queue
        xactServoParam_t *param = &xactQueue[xactQueueHead];

        // Prepare downlink frame for XACT servo programming
        downlink->length = 0x08;  // 8 bytes payload
        downlink->phyID = param->phyID;
        downlink->prim = XACT_FRAME_TYPE_WRITE;  // 0x31 for write

        // Pack the frame data
        downlink->appId = param->appId;          // Use appId field for App ID
        downlink->data[0] = param->fieldId;      // Field ID in data[0]
        downlink->data[1] = param->data & 0xFF;  // DATA1 low byte in data[1]
        downlink->data[2] = (param->data >> 8) & 0xFF; // DATA1 high byte in data[2]
        downlink->data[3] = 0;                   // Unused

        // Fill physical ID check bits
        xactPhyIDFillCheckBits(&downlink->phyID);

        // Calculate CRC
        uint8_t crc = frskyCheckSum((uint8_t *)&downlink->phyID, 0x08);
        downlink->crc = crc;

        // Remove from queue
        xactQueueHead = (xactQueueHead + 1) % XACT_QUEUE_SIZE;
        xactQueueCount--;

        return true;
    }

    return false;
}

// Get queue count (for monitoring)
uint8_t fbusXactGetQueueCount(void)
{
    return xactQueueCount;
}

// Get servo parameters for a specific physical ID
bool fbusXactGetServoParams(uint8_t phyID, xactServoParams_t *params)
{
    if (!xactInitialized || params == NULL) {
        return false;
    }

    // Find the servo by physical ID
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            // Copy the stored parameters
            memcpy(params, &xactServoParams[i], sizeof(xactServoParams_t));
            return true;
        }
    }

    return false;
}

// Check whether a full parameter read has completed for a discovered servo
bool fbusXactIsServoParamsReady(uint8_t phyID)
{
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            return xactServos[i].paramsReady;
        }
    }

    return false;
}

// Check whether this Physical ID has reported more than one App ID (see fbus_xact.h)
bool fbusXactHasServoConflict(uint8_t phyID)
{
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            return xactServos[i].appIdConflict;
        }
    }

    return false;
}

// Check whether another discovered servo (different Physical ID) shares this one's App ID
// (see fbus_xact.h for why this matters)
bool fbusXactHasDuplicateAppId(uint8_t phyID)
{
    uint16_t appId = 0;
    bool found = false;

    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            appId = xactServos[i].appId;
            found = true;
            break;
        }
    }

    if (!found) {
        return false;
    }

    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID != phyID && xactServos[i].appId == appId) {
            return true;
        }
    }

    return false;
}

// (Re)start a full parameter read for an already-discovered servo
bool fbusXactRequestParamsRead(uint8_t phyID)
{
    if (!xactInitialized) {
        return false;
    }

    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            // Don't restart a read that's already in flight for this same servo
            const bool alreadyReadingThisServo = (xactReadServoIndex == i) &&
                (xactReadState == XACT_READ_STATE_READING || xactReadState == XACT_READ_STATE_WAIT_POLL);

            if (!alreadyReadingThisServo) {
                xactReadServoIndex = i;
                xactReadParamIndex = 0;
                xactServos[i].paramsReady = false;
                xactReadState = XACT_READ_STATE_READING;
            }

            return true;
        }
    }

    return false;
}

// Set a specific servo parameter field (stores value from read response)
bool fbusXactSetServoParam(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data)
{
    UNUSED(appId);
    if (!xactInitialized) {
        return false;
    }

    // Find the servo by physical ID
    int8_t servoIndex = -1;
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            servoIndex = i;
            break;
        }
    }

    if (servoIndex < 0) {
        return false;
    }

    // Update the parameter in storage based on field ID
    xactServoParams_t *params = &xactServoParams[servoIndex];
    switch (fieldId) {
        case XACT_FIELD_PHYSICAL_ID:
            params->physicalId = data;
            break;
        case XACT_FIELD_APP_ID_BASE:
            params->appIdOffset = (uint8_t)data;
            break;
        case XACT_FIELD_DATA_RATE:
            params->dataRate = (uint16_t)data;
            break;
        case XACT_FIELD_RANGE:
            params->range = (uint8_t)data;
            break;
        case XACT_FIELD_DIRECTION:
            params->direction = (uint8_t)data;
            break;
        case XACT_FIELD_PULSE_TYPE:
            params->pulseType = (uint8_t)data;
            break;
        case XACT_FIELD_CHANNEL:
            params->channel = (uint8_t)data;
            break;
        case XACT_FIELD_CENTER:
            params->center = (int8_t)data;
            break;
        case XACT_FIELD_HOLDING_STRENGTH:
            params->holdingStrength = (uint8_t)data;
            break;
        case XACT_FIELD_OPERATION_SMOOTHING:
            params->operationSmoothing = (uint8_t)data;
            break;
        case XACT_FIELD_DEADBAND:
            params->deadband = (uint8_t)data;
            break;
        case XACT_FIELD_FIRMWARE_VERSION:
            params->firmwareVersion = (uint8_t)data;
            params->hasExtendedParams = (params->firmwareVersion >= XACT_SERIES65_MIN_FIRMWARE);
            break;
        case XACT_FIELD_WORKING_MODE:
            params->workingMode = (uint8_t)data;
            break;
        case XACT_FIELD_MAX_ANGLE:
            params->maxAngle = (uint16_t)data;
            break;
        default:
            return false;
    }

    // Only store the value - do NOT queue write command
    // Writing is triggered explicitly via MSP command calling fbusXactWriteUplinkFramePhyID
    return true;
}

// Compare and write all parameters if different from cache
bool fbusXactCompareAndWriteParams(uint8_t phyID, uint16_t appId, const xactServoParams_t *newParams)
{
    if (!xactInitialized || newParams == NULL) {
        return false;
    }

    // Find the servo by physical ID
    int8_t servoIndex = -1;
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            servoIndex = i;
            break;
        }
    }

    if (servoIndex < 0) {
        return false;
    }

    // Get cached parameters
    xactServoParams_t *cachedParams = &xactServoParams[servoIndex];

    // XACT write commands are addressed by App ID, not exclusively by Physical ID -- real
    // hardware testing confirmed that two servos sharing an App ID BOTH act on a write meant
    // for just one of them, regardless of their (different) Physical IDs. Refuse to touch
    // anything until this is resolved, unless this write is itself changing App ID to
    // something that's actually unique among the servos currently tracked.
    if (fbusXactHasDuplicateAppId(phyID)) {
        const bool changingAppId = cachedParams->appIdOffset != newParams->appIdOffset;
        if (!changingAppId) {
            return false;
        }

        const uint16_t prospectiveAppId = FBUS_SERVO_DATA_BASE + newParams->appIdOffset;
        for (uint8_t i = 0; i < xactServoCount; i++) {
            if (xactServos[i].phyID != phyID && xactServos[i].appId == prospectiveAppId) {
                return false;  // still not unique -- refuse
            }
        }
    }

    bool hasChanges = false;

    // Track current phyID and appId - these may be updated if PHYSICAL_ID or APP_ID_BASE change
    uint8_t currentPhyID = phyID;
    uint16_t currentAppId = appId;

    // Compare each parameter and queue writes for differences
    if (cachedParams->physicalId != newParams->physicalId) {
        // Refuse to rename onto a Physical ID another discovered servo already has -- unlike
        // the App ID guard above, this only skips this one field rather than the whole write:
        // Physical ID isn't what write commands are actually addressed by (App ID is), so a
        // bad rename here doesn't make the other fields' writes unsafe, it would just create a
        // second Physical ID collision.
        bool physicalIdTaken = false;
        for (uint8_t i = 0; i < xactServoCount; i++) {
            if (i != (uint8_t)servoIndex && xactServos[i].phyID == newParams->physicalId) {
                physicalIdTaken = true;
                break;
            }
        }

        if (!physicalIdTaken) {
            fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_PHYSICAL_ID, currentAppId, newParams->physicalId);
            cachedParams->physicalId = newParams->physicalId;
            // Update currentPhyID for subsequent writes, and keep the tracking key (which the
            // discovery list and every fbusXactGetServoParams()/-CompareAndWriteParams() lookup
            // is keyed by) in sync with the rename -- otherwise this servo keeps appearing
            // under its old Physical ID until the next full rescan, and further reads/writes
            // addressed to the new one won't find it.
            currentPhyID = newParams->physicalId;
            xactServos[servoIndex].phyID = newParams->physicalId;
            hasChanges = true;
        }
    }

    if (cachedParams->appIdOffset != newParams->appIdOffset) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_APP_ID_BASE, currentAppId, newParams->appIdOffset);
        cachedParams->appIdOffset = newParams->appIdOffset;
        // Update currentAppId for subsequent writes (base + offset)
        currentAppId = FBUS_SERVO_DATA_BASE + newParams->appIdOffset;
        hasChanges = true;

        // We changed this ourselves, so pre-arm the tracked App ID to match: the next telemetry
        // frame reporting it is expected, not a sign of a second servo colliding on this bus address.
        xactServos[servoIndex].appId = currentAppId;
        xactServos[servoIndex].appIdConflict = false;
    }

    if (cachedParams->dataRate != newParams->dataRate) {
        // Data rate is 16-bit, write low byte then high byte
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_DATA_RATE, currentAppId, newParams->dataRate);
        cachedParams->dataRate = newParams->dataRate;
        hasChanges = true;
    }

    if (cachedParams->range != newParams->range) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_RANGE, currentAppId, newParams->range);
        cachedParams->range = newParams->range;
        hasChanges = true;
    }

    if (cachedParams->direction != newParams->direction) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_DIRECTION, currentAppId, newParams->direction);
        cachedParams->direction = newParams->direction;
        hasChanges = true;
    }

    if (cachedParams->pulseType != newParams->pulseType) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_PULSE_TYPE, currentAppId, newParams->pulseType);
        cachedParams->pulseType = newParams->pulseType;
        hasChanges = true;
    }

    if (cachedParams->channel != newParams->channel) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_CHANNEL, currentAppId, newParams->channel);
        cachedParams->channel = newParams->channel;
        hasChanges = true;
    }

    if (cachedParams->center != newParams->center) {
        // (uint16_t)(uint8_t) round-trips a negative int8_t through the wire's single data
        // byte correctly -- fbusXactProcessQueue truncates this to data & 0xFF when framing.
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_CENTER, currentAppId, (uint16_t)(uint8_t)newParams->center);
        cachedParams->center = newParams->center;
        hasChanges = true;
    }

    if (cachedParams->holdingStrength != newParams->holdingStrength) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_HOLDING_STRENGTH, currentAppId, newParams->holdingStrength);
        cachedParams->holdingStrength = newParams->holdingStrength;
        // FrSky's own tool always re-pins the paired 0x12 field to 10 whenever Holding
        // Strength changes; mirror that exactly rather than exposing 0x12 as its own field.
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_HOLDING_STRENGTH_COMPANION, currentAppId, 10);
        hasChanges = true;
    }

    if (cachedParams->operationSmoothing != newParams->operationSmoothing) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_OPERATION_SMOOTHING, currentAppId, newParams->operationSmoothing);
        cachedParams->operationSmoothing = newParams->operationSmoothing;
        hasChanges = true;
    }

    if (cachedParams->deadband != newParams->deadband) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_DEADBAND, currentAppId, newParams->deadband);
        cachedParams->deadband = newParams->deadband;
        hasChanges = true;
    }

    // Working Mode/Max Angle only exist on "series 65"+ servos -- never write them to a servo
    // that hasn't reported that it supports them.
    if (cachedParams->hasExtendedParams) {
        if (cachedParams->workingMode != newParams->workingMode) {
            fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_WORKING_MODE, currentAppId, newParams->workingMode);
            cachedParams->workingMode = newParams->workingMode;
            hasChanges = true;
        }

        if (cachedParams->maxAngle != newParams->maxAngle) {
            fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_MAX_ANGLE, currentAppId, newParams->maxAngle);
            cachedParams->maxAngle = newParams->maxAngle;
            hasChanges = true;
        }
    }

    // If any parameters were changed, commit them to flash using FrSky's own two-step
    // sequence: prime with 0x15=7, then the actual save command.
    if (hasChanges) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_SAVE_PRIME, currentAppId, 7);
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_WRITE_FLASH, currentAppId, 0);
    }

    return hasChanges;
}

// Check if XACT is currently busy (reading or writing)
bool fbusXactIsBusy(void)
{
    if (!xactInitialized) {
        return false;
    }

    // XACT is busy if:
    // 1. Currently reading parameters
    // 2. Waiting for a read response
    // 3. Has items in the write queue
    return (xactReadState == XACT_READ_STATE_READING ||
            xactReadState == XACT_READ_STATE_WAIT_POLL ||
            xactQueueCount > 0);
}

// Notify that a response was received for the current read operation
void fbusXactNotifyResponse(uint8_t phyID, uint8_t fieldId)
{
    if (!xactInitialized) {
        return;
    }

    // Only process if we're waiting for a response
    if (xactReadState != XACT_READ_STATE_WAIT_POLL) {
        return;
    }

    // Verify this response matches what we're waiting for
    if (phyID != xactLastReadPhyID || fieldId != xactLastReadFieldId) {
        return;
    }

    xactAdvanceReadField();
}

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

#pragma once

#include "platform.h"
#include "common/time.h"
#include "drivers/fbus_master.h"

// XACT servo programming frame types (PRIM field)
#define XACT_FRAME_TYPE_READ     0x30
#define XACT_FRAME_TYPE_WRITE    0x31
#define XACT_FRAME_TYPE_RESPONSE 0x32

// XACT servo field IDs (FIELDID). Field set, names, and ranges are taken directly from
// FrSky's own ETHOS "XAct" Device Config Lua script (org.frsky-ethos.xact, v2.0.2) -- see
// its basic.lua for the authoritative parameter list this mirrors.
// Frame format: Length PhysID PRIM APPID(16-bit) FIELDID DATA1 ... CRC
#define XACT_FIELD_PHYSICAL_ID    0x00  // Physical ID (0-26)
#define XACT_FIELD_APP_ID_BASE    0x01  // App ID base addr + offset (0-15)
#define XACT_FIELD_DATA_RATE      0x02  // Data rate, ms (10-60000)
#define XACT_FIELD_RANGE          0x04  // Range: 0=120°, 1=90°, 2=180°
#define XACT_FIELD_DIRECTION      0x05  // Direction: 0=clockwise, 1=anticlockwise
#define XACT_FIELD_PULSE_TYPE     0x06  // Pulse type: 0=1500us, 1=760us
#define XACT_FIELD_CHANNEL        0x07  // Channel: 0=CH1 (wire is 0-based, UI is 1-based)
#define XACT_FIELD_CENTER         0x08  // Center, signed (-125..125)
#define XACT_FIELD_HOLDING_STRENGTH 0x11  // Holding strength (4-15)
// 0x12 is paired with Holding Strength: FrSky's own tool always re-writes it to 10 whenever
// 0x11 changes, and never reads or displays it as its own field -- do the same rather than
// exposing it as an independently editable parameter.
#define XACT_FIELD_HOLDING_STRENGTH_COMPANION 0x12
#define XACT_FIELD_OPERATION_SMOOTHING 0x13  // Operation smoothing (0-50)
// 0x15 is NOT a tunable parameter -- it's the priming byte (value 7) of the two-step
// save-to-flash sequence, sent immediately before XACT_FIELD_WRITE_FLASH. Never read.
#define XACT_FIELD_SAVE_PRIME     0x15
#define XACT_FIELD_DEADBAND       0x21  // Deadband (0-90)
#define XACT_FIELD_FIRMWARE_VERSION 0xFE  // Firmware version, read-only
// "Series 65" servos (firmwareVersion >= this) additionally support Working Mode/Max Angle.
#define XACT_SERIES65_MIN_FIRMWARE 40
#define XACT_FIELD_WORKING_MODE   0x40  // Working mode: 0=Angle, 1=Range, 2=Rotate (series 65+ only)
#define XACT_FIELD_MAX_ANGLE      0x41  // Max angle, degrees (0-359, series 65+ only)
#define XACT_FIELD_WRITE_FLASH    0x30  // Save changes to flash (commit step; see SAVE_PRIME above)

// XACT servo data ID range (from fbus_sensor.h)
#define FBUS_SERVO_DATA_BASE 0x6800
#define FBUS_SERVO_DATA_END  0x680F

// Maximum number of XACT servos to track
#define XACT_MAX_SERVOS 16

// XACT servo programming structure
// All fields are uint8_t to match the frame format
typedef struct {
    uint8_t phyID;        // Physical ID
    uint8_t fieldId;      // Field ID (parameter to write)
    uint16_t appId;       // Application ID (APPID2 << 8 | APPID1)
    uint16_t data;         // Data value (DATA1)
} xactServoParam_t;

// XACT servo tracking structure
typedef struct {
    uint8_t phyID;
    uint16_t appId;
    timeUs_t lastSeenUs;
    bool paramsReady;      // true once a full parameter read has completed for this servo
    bool appIdConflict;    // true if frames for this Physical ID have reported more than one
                            // App ID -- almost certainly two servos sharing the same Physical
                            // ID and colliding on the bus, not one servo
} xactServo_t;

// XACT servo parameter storage. Field set/names/ranges mirror FrSky's own "XAct" ETHOS Device
// Config Lua script (see the XACT_FIELD_* comments above) rather than a from-scratch guess.
typedef struct {
    uint8_t physicalId;         // 0x00
    uint8_t appIdOffset;        // 0x01
    uint8_t firmwareVersion;    // 0xFE, read-only
    uint16_t dataRate;          // 0x02 (16-bit value)
    uint8_t range;              // 0x04
    uint8_t direction;          // 0x05
    uint8_t pulseType;          // 0x06
    uint8_t channel;            // 0x07
    int8_t center;              // 0x08 -- signed
    uint8_t holdingStrength;    // 0x11
    uint8_t operationSmoothing; // 0x13
    uint8_t deadband;           // 0x21
    // Only meaningful when hasExtendedParams is true (firmwareVersion >= XACT_SERIES65_MIN_FIRMWARE)
    bool hasExtendedParams;
    uint8_t workingMode;        // 0x40
    uint16_t maxAngle;          // 0x41
} xactServoParams_t;

// Initialize XACT servo module
void fbusXactInit(void);

// Clear all discovered XACT servos
void fbusXactClearDiscoveredServos(void);

// Start a new sensor discovery phase
void fbusXactStartSensorDiscovery(void);

// Track an XACT servo (called when servo data is received)
void fbusXactTrackServo(uint8_t phyID, uint16_t appId, timeUs_t currentTimeUs);

// Write a parameter to an XACT servo using physical ID
// fieldId: XACT_FIELD_* constant
// appId: Application ID (16-bit)
// data: Data value (8-bit)
bool fbusXactWriteUplinkFramePhyID(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data);

// Get the number of discovered XACT servos
uint8_t fbusXactGetDiscoveredServoCount(void);

// Get a discovered XACT servo physical ID by index
uint8_t fbusXactGetDiscoveredServoPhyID(uint8_t index);

// Get servo parameters for a specific physical ID
bool fbusXactGetServoParams(uint8_t phyID, xactServoParams_t *params);

// Check whether a full parameter read has completed for a discovered servo. When multiple
// servos are discovered, only the first one is read automatically -- callers that want to
// look at (or write to) any other discovered servo should call fbusXactRequestParamsRead()
// for it first, then poll this until it returns true.
bool fbusXactIsServoParamsReady(uint8_t phyID);

// (Re)start a full parameter read for an already-discovered servo. Safe to call repeatedly;
// a read already in progress for this same servo is left alone. Returns false if phyID isn't
// a currently-discovered servo.
bool fbusXactRequestParamsRead(uint8_t phyID);

// Check whether frames for this Physical ID have reported more than one App ID since it was
// discovered -- a strong signal that two servos are configured with the same Physical ID and
// colliding on the bus, rather than there being one servo at this address.
bool fbusXactHasServoConflict(uint8_t phyID);

// Check whether another discovered servo (a different Physical ID) shares this one's App ID.
// XACT write/read commands are addressed by App ID, not exclusively by Physical ID -- real
// hardware testing confirmed that two servos sharing an App ID BOTH act on a command meant for
// just one of them, regardless of their (different) Physical IDs. fbusXactCompareAndWriteParams
// refuses to write while this is true, rather than silently reprogramming the wrong servo(s).
bool fbusXactHasDuplicateAppId(uint8_t phyID);

// Set a specific servo parameter field
bool fbusXactSetServoParam(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data);

// Compare and write all parameters if different from cache. Returns false (no writes sent)
// without changing anything if fbusXactHasDuplicateAppId(phyID) is true.
bool fbusXactCompareAndWriteParams(uint8_t phyID, uint16_t appId, const xactServoParams_t *newParams);

// Check if XACT module is initialized
bool fbusXactIsInitialized(void);

// Process XACT servo programming queue (internal use)
bool fbusXactProcessQueue(fbusMasterDownlink_t *downlink);

// Get queue count (for monitoring)
uint8_t fbusXactGetQueueCount(void);

// Check if XACT is currently busy (reading or writing)
bool fbusXactIsBusy(void);

// Notify that a response was received for the current read operation
void fbusXactNotifyResponse(uint8_t phyID, uint8_t fieldId);

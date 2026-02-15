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

 //this uses SBUS out and SPORT/FBUS_in

#include "fbus_master.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "build/build_config.h"
#include "common/maths.h"
#include "common/time.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "drivers/sbus_output.h"
#include "pg/bus_servo.h"
#include "pg/fbus_master.h"
#include "pg/sbus_output.h"
#include "pg/servos.h"
#include "rx/frsky_crc.h"
#include "io/serial.h"
#include "platform.h"

#define FBUS_MASTER_BUFFER_SIZE 64
#define GET_BIT(value, bit) ((value >> bit) & 1)

enum {
    FBUS_FRAME_ID_NULL = 0x00,
    FBUS_FRAME_ID_DATA = 0x10,
    FBUS_FRAME_ID_WORKING_STATE = 0x21,
    FBUS_FRAME_IDLE_STATE = 0x22,
    FBUS_FRAME_ID_READ = 0x30,
    FBUS_FRAME_ID_WRITE = 0x31,
    FBUS_FRAME_ID_RESPONSE = 0x32,
    FBUS_FRAME_ID_OTA_START = 0xF0,
    FBUS_FRAME_ID_OTA_DATA = 0xF1,
    FBUS_FRAME_ID_OTA_STOP = 0xF2
};

serialPort_t *fbusMasterPort = NULL;

#define FC_COMMON_ID 0x1B
#define FBUS_MAX_PHYS_ID 0x1B

#define FBUS_MASTER_SCAN_COUNTER 3

typedef enum {
    FBUS_MASTER_SCAN_PHY_ID = 0,
    FBUS_MASTER_QUERY_PHY_ID,
}fbusMasterTelemetryState_e;

typedef enum {
    FBUS_MASTER_TELEMETRY = 0,
    FBUS_MASTER_OTA,
}fbusMasterPayloadState_e;

uint8_t phsIdList[FBUS_MAX_PHYS_ID] = {0};
uint8_t physIdsfound = 0;
uint8_t physIdCnt = 0;
uint8_t currentPhysId = 0;
uint8_t scanCounter = 0;
uint8_t readIngoreBytes = 0;
uint8_t readBytes = 0;
uint8_t buffer[FBUS_MASTER_BUFFER_SIZE] = {0};

fbusMasterPayloadState_e fbusMasterPayloadState = FBUS_MASTER_TELEMETRY;
fbusMasterTelemetryState_e fbusMasterTelemetryState = FBUS_MASTER_SCAN_PHY_ID;

static void smartportMasterPhyIDFillCheckBits(uint8_t *phyIDByte)
{
    *phyIDByte |= (GET_BIT(*phyIDByte, 0) ^ GET_BIT(*phyIDByte, 1) ^ GET_BIT(*phyIDByte, 2)) << 5;
    *phyIDByte |= (GET_BIT(*phyIDByte, 2) ^ GET_BIT(*phyIDByte, 3) ^ GET_BIT(*phyIDByte, 4)) << 6;
    *phyIDByte |= (GET_BIT(*phyIDByte, 0) ^ GET_BIT(*phyIDByte, 2) ^ GET_BIT(*phyIDByte, 4)) << 7;
}

static int8_t smartportMasterStripPhyIDCheckBits(uint8_t phyID)
{
    uint8_t smartportPhyID = phyID & 0x1F;
    uint8_t phyIDCheck = smartportPhyID;
    smartportMasterPhyIDFillCheckBits(&phyIDCheck);
    return phyID == phyIDCheck ? smartportPhyID : -1;
}

static void fbusMasterPrepareFrame(fbusMasterFrame_t *frame, uint16_t *channels)
{
    // Clear the control.c16 structure
    memset(&frame->c16, 0, sizeof(fbusMasterFrame_t));

    frame->c16.length = FBUS_CONTROL16_LENGTH;
    frame->c16.type = FBUS_CONTROL16_TYPE;

    frame->c16.channels.chan0 = channels[0];
    frame->c16.channels.chan1 = channels[1];
    frame->c16.channels.chan2 = channels[2];
    frame->c16.channels.chan3 = channels[3];
    frame->c16.channels.chan4 = channels[4];
    frame->c16.channels.chan5 = channels[5];
    frame->c16.channels.chan6 = channels[6];
    frame->c16.channels.chan7 = channels[7];
    frame->c16.channels.chan8 = channels[8];
    frame->c16.channels.chan9 = channels[9];
    frame->c16.channels.chan10 = channels[10];
    frame->c16.channels.chan11 = channels[11];
    frame->c16.channels.chan12 = channels[12];
    frame->c16.channels.chan13 = channels[13];
    frame->c16.channels.chan14 = channels[14];
    frame->c16.channels.chan15 = channels[15];

    frame->c16.channels.flags = channels[16] ? BIT(0) : 0;
    frame->c16.channels.flags |= channels[17] ? BIT(1) : 0;

    // Set RSSI (example value)
    frame->c16.rssi = 100; //ToDo

    uint8_t crc = frskyCheckSum((uint8_t *)&(frame->c16.type), FBUS_MASTER_CONTROL_FRAME_PAYLOAD_SIZE);
    frame->c16.crc = crc;

    switch (fbusMasterPayloadState) {
        case FBUS_MASTER_TELEMETRY:
            memset(&frame->downlink, 0, sizeof(fbusMasterDownlink_t));
            frame->downlink.length = FBUS_DOWNLINK_PAYLOAD_SIZE;
            
            switch (fbusMasterTelemetryState) {
                case FBUS_MASTER_SCAN_PHY_ID:
                    currentPhysId = currentPhysId == FC_COMMON_ID ? currentPhysId + 1 : currentPhysId;
                    if (currentPhysId > FBUS_MAX_PHYS_ID) {
                        currentPhysId = 0;
                        scanCounter++;
                        if (scanCounter >= FBUS_MASTER_SCAN_COUNTER) {
                            scanCounter = 0;
                            fbusMasterTelemetryState = FBUS_MASTER_QUERY_PHY_ID;
                            break;
                        }
                    }
        
                    frame->downlink.phyID = currentPhysId;
                    frame->downlink.prim = FBUS_FRAME_ID_DATA;
                    currentPhysId++;
                    break;
                case FBUS_MASTER_QUERY_PHY_ID:
                    currentPhysId = phsIdList[physIdCnt];
                    frame->downlink.phyID = currentPhysId;
                    frame->downlink.prim = FBUS_FRAME_ID_DATA;
                    physIdCnt = physIdCnt == physIdsfound-1 ? 0 : physIdCnt + 1;
                    break;
                
                default:
                    break;
            }
    
            smartportMasterPhyIDFillCheckBits(&frame->downlink.phyID);
            crc = frskyCheckSum((uint8_t *)&frame->downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
            frame->downlink.crc = crc;
    
            break;
    
        case FBUS_MASTER_OTA:
            //ToDo
            break;
        
        default:
            break;
    }

}

static void processDownlinkFrame(uint8_t *data)
{
    fbusMasterDownlink_t downlink;
    memcpy(&downlink, data, sizeof(downlink));
    uint8_t chkSum = frskyCheckSum((uint8_t *)&downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
    if (chkSum == downlink.crc) {
        downlink.phyID = smartportMasterStripPhyIDCheckBits(downlink.phyID);
        if (fbusMasterTelemetryState == FBUS_MASTER_SCAN_PHY_ID) {
            bool alreadyInList = false;
            for (uint8_t i = 0; i < physIdsfound; i++) {
                if (phsIdList[i] == downlink.phyID) {
                    alreadyInList = true;
                    break;
                }
            }
            if (!alreadyInList) {
                phsIdList[physIdsfound++] = downlink.phyID;
            }
        }
    }    
}

static FAST_CODE void dataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    // don't listen to self
    if (readIngoreBytes > 0) {
        readIngoreBytes--;
        return;
    }

    buffer[readBytes++] = c;

    if (readBytes >= FBUS_MASTER_BUFFER_SIZE) {
        //sync error
        readBytes = 0;
    } else {
        if (readBytes >= 10 && buffer[0] == FBUS_DOWNLINK_PAYLOAD_SIZE) {
            //process frame, reset readBytes
            processDownlinkFrame(buffer);
            readBytes = 0;
        }
    }
}

static float fbusMasterGetChannelValue(uint8_t channel)
{
    const busServoSourceType_e source_type = busServoConfig()->sourceType[channel];
    switch (source_type) {
        case BUS_SERVO_SOURCE_RX:
            return sbusOutGetRX(channel);
        case BUS_SERVO_SOURCE_MIXER:
            // Use the same servo-parameter-aware function
            return sbusOutGetValueMixer(channel);
    }
    return 0;
}

static uint16_t fbusMasterConvertToSbus(float value)
{
    // For analog channels (0-15), convert microseconds to SBUS range (192-1792)
    // Bus servo range: (1000 -> BUS_SERVO_MIN_SIGNAL) to (2000 -> BUS_SERVO_MAX_SIGNAL) -> SBUS 192-1792
    const float scaledValue = scaleRangef(value, BUS_SERVO_MIN_SIGNAL, BUS_SERVO_MAX_SIGNAL, 192, 1792);
    return constrain(nearbyintf(scaledValue), FBUS_MIN, FBUS_MAX);
}

void fbusMasterUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (!fbusMasterPort)
        return;

    // Check TX Buff is free
    if (serialTxBytesFree(fbusMasterPort) <= sizeof(fbusMasterFrame_t)) {
        return;
    }

    // Start sending.
    fbusMasterFrame_t frame;
    uint16_t channels[FBUS_MASTER_CHANNELS];
    for (int ch = 0; ch < FBUS_MASTER_CHANNELS; ch++) {
        float value = fbusMasterGetChannelValue(ch);
        channels[ch] = fbusMasterConvertToSbus(value);
        
        // Store the output value for getServoOutput() to retrieve
        setBusServoOutput(ch, value);
    }
    fbusMasterPrepareFrame(&frame, channels);

    // serial output
    serialWriteBuf(fbusMasterPort, (const uint8_t *)&frame, sizeof(frame));
    readIngoreBytes = sizeof(frame);
    readBytes = 0;
}

bool fbusMasterIsEnabled(void) 
{ 
    return fbusMasterPort != NULL;
}

void fbusMasterInit(void)
{
    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_FBUS_MASTER);

    if (!portConfig) {
        fbusMasterPort = NULL;
        return;
    }

    serialReceiveCallbackPtr callback = dataReceive;
    fbusMasterPort = openSerialPort(
        portConfig->identifier, FUNCTION_FBUS_MASTER, callback, NULL, 460800, MODE_RXTX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (fbusMasterConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            SERIAL_BIDIR |
            (fbusMasterConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}


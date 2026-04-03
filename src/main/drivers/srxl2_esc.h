/*
 * This file is part of Rotorflight (rotorflight.org)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef USE_SRXL2_ESC

#include "common/time.h"

#include "io/serial.h"

#define SRXL2_ESC_PORT_BAUDRATE_DEFAULT    115200
#define SRXL2_ESC_PORT_BAUDRATE_HIGH       400000
#define SRXL2_ESC_DRIVER_TASK_FREQ_HZ      4000

struct sbuf_s;
/*
 * To fully isolate the SMART ESC driver from the global RX subsystem we
 * provide driver-local configuration/runtime types. These intentionally
 * mirror the small subset of fields used by the SMART ESC driver so the
 * driver can operate without depending on `rx/rx.h` types at link time.
 */
struct srxl2esc_runtimeState_s;
typedef float (*srxl2esc_readRawDataFnPtr)(const struct srxl2esc_runtimeState_s *rs, uint8_t chan);
typedef uint8_t (*srxl2esc_frameStatusFnPtr)(struct srxl2esc_runtimeState_s *rs);
typedef bool (*srxl2esc_processFrameFnPtr)(const struct srxl2esc_runtimeState_s *rs);
typedef timeUs_t srxl2esc_getFrameTimeUsFn(void);

typedef struct {
	uint8_t srxl2_unit_id; /* unit id used by SRXL2 handshake */
	bool    pinSwap;       /* pin swap hint used when opening serial port */
} srxl2esc_config_t;

typedef struct srxl2esc_runtimeState_s {
	uint16_t                    	*channelData;      	/* channel storage */
	uint8_t                      	channelCount;     	/* number of channels */
	uint16_t                     	rxRefreshRate;    	/* refresh period */
	srxl2esc_readRawDataFnPtr  		rcReadRawFn;      	/* read raw channel helper */
	srxl2esc_frameStatusFnPtr  		rcFrameStatusFn;  	/* frame-status callback */
	srxl2esc_processFrameFnPtr 		rcProcessFrameFn; 	/* process-frame callback */
	srxl2esc_getFrameTimeUsFn 		*rcFrameTimeUsFn;  	/* timestamp helper */
	timeUs_t                     	lastRcFrameTimeUs;
} srxl2esc_runtimeState_t;

typedef struct {
	uint8_t  sensorId;
	uint8_t  secondaryId;
	uint8_t  data[14];
	uint32_t timestampUs;
	bool     valid;
} srxl2escTelemetrySnapshot_t;

void validateAndFixSrxl2escConfig(void);
void srxl2esc_poll(void);
void srxl2esc_service(void);
bool srxl2escInit(const srxl2esc_config_t *escConfig);
void srxl2escWriteData(void *data, int len);
unsigned srxl2escGetTelemetryHistoryCount(void);
unsigned srxl2escCopyTelemetryHistory(
	unsigned idx,
	uint8_t *dst,
	unsigned maxLen,
	uint8_t *outLen,
	uint32_t *outTimestamp,
	uint8_t *outSensorId,
	uint8_t *outSecondaryId,
	uint32_t *outCount);
bool srxl2escGetLatestTelemetry(uint8_t *sensorId, uint8_t *secondaryId, uint8_t *dst, unsigned maxLen);
bool srxl2escCopyLatestTelemetry(srxl2escTelemetrySnapshot_t *out, uint32_t *outSeq);

void srxl2escDataReceive(uint16_t c, void *ctx);
void srxl2escIdle(void);
void srxl2escAttachPort(serialPort_t *p);

bool srxl2escDriverInit(void);
void srxl2escDriverTask(timeUs_t currentTimeUs);
bool srxl2escDriverIsReady(void);
#endif
#pragma once
#ifdef USE_SMART_ESC
#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"

#include "io/serial.h"

#define SMARTESC_PORT_BAUDRATE_DEFAULT    115200
#define SMARTESC_PORT_BAUDRATE_HIGH       400000
#define SMARTESC_DRIVER_TASK_FREQ_HZ      4000

struct sbuf_s;
/*
 * To fully isolate the SMART ESC driver from the global RX subsystem we
 * provide driver-local configuration/runtime types. These intentionally
 * mirror the small subset of fields used by the SMART ESC driver so the
 * driver can operate without depending on `rx/rx.h` types at link time.
 */
struct smartesc_rxRuntimeState_s;
typedef float (*smartesc_rcReadRawDataFnPtr)(const struct smartesc_rxRuntimeState_s *rs, uint8_t chan);
typedef uint8_t (*smartesc_rcFrameStatusFnPtr)(struct smartesc_rxRuntimeState_s *rs);
typedef bool (*smartesc_rcProcessFrameFnPtr)(const struct smartesc_rxRuntimeState_s *rs);
typedef timeUs_t smartesc_rcGetFrameTimeUsFn(void);

typedef struct {
	uint8_t srxl2_unit_id; /* unit id used by SRXL2 handshake */
	bool    pinSwap;       /* pin swap hint used when opening serial port */
} srxl2_escConfig_t;

typedef struct smartesc_rxRuntimeState_s {
	uint16_t                    *channelData;      /* channel storage */
	uint8_t                      channelCount;     /* number of channels */
	uint16_t                     rxRefreshRate;    /* refresh period */
	smartesc_rcReadRawDataFnPtr  rcReadRawFn;      /* read raw channel helper */
	smartesc_rcFrameStatusFnPtr  rcFrameStatusFn;  /* frame-status callback */
	smartesc_rcProcessFrameFnPtr rcProcessFrameFn; /* process-frame callback */
	smartesc_rcGetFrameTimeUsFn *rcFrameTimeUsFn;  /* timestamp helper */
	timeUs_t                     lastRcFrameTimeUs;
} smartesc_rxRuntimeState_t;

void validateAndFixSmartescConfig();
void smartesc_poll(void);
void smartesc_service(void);
bool smartescInit(const srxl2_escConfig_t *escConfig, smartesc_rxRuntimeState_t *smartescRs);
void smartescWriteData(const void *data, int len);
unsigned smartescGetTelemetryHistoryCount(void);
unsigned smartescCopyTelemetryHistory(unsigned idx, uint8_t *dst, unsigned maxLen, uint8_t *outLen, uint32_t *outTimestamp, uint8_t *outSensorId, uint8_t *outSecondaryId, uint32_t *outCount);
bool smartescGetLatestTelemetry(uint8_t *sensorId, uint8_t *secondaryId, uint8_t *dst, unsigned maxLen);

void smartescDataReceive(uint16_t c, void *ctx);
void smartescIdle(void);
void smartescAttachPort(serialPort_t *p);

bool smartescDriverInit(void);
void smartescDriverTask(timeUs_t currentTimeUs);
bool smartescDriverIsReady(void);
#endif
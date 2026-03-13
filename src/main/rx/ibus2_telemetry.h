#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "drivers/serial.h"
#include "common/time.h"

typedef struct {
    uint32_t queueCount;
    uint32_t sendCount;
    uint32_t txCompleteCount;
    uint32_t timeoutDropCount;
    uint32_t rxBusyCount;
    uint32_t broadcastCount;
    uint32_t addressDropCount;
    uint8_t lastAddress;
    uint8_t lastCommandCode;
    uint8_t lastResponseCode;
    uint8_t requiredResources;
    uint8_t responseByte0;
    uint8_t responseByte1;
    uint8_t responseByte2;
    uint8_t responseByte3;
    uint8_t responseByte14;
    uint8_t responseByte15;
    uint16_t lastParamType;
    uint16_t lastSendDelayUs;
    uint16_t maxSendDelayUs;
    bool pendingCommand;
    bool transmitting;
} ibus2TelemetryDebug_t;

void ibus2TelemetryInit(serialPort_t *port);
void ibus2TelemetryReset(void);
void ibus2TelemetrySetRequiredResources(uint8_t requiredResources);
void ibus2TelemetryUpdateAddress(const uint8_t *frame, size_t frameLen);
void ibus2TelemetryQueueCommand(const uint8_t *frame, size_t frameLen, timeUs_t receivedAtUs);
void ibus2TelemetryGetDebug(ibus2TelemetryDebug_t *debug);
bool ibus2TelemetryPending(void);
bool ibus2TelemetryProcess(timeUs_t nowUs);

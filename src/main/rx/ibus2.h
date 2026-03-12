#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/serial.h"

#include "rx/rx.h"

bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState);
serialPort_t *ibus2GetRxSerialPort(void);

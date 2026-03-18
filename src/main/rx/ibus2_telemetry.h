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

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "drivers/serial.h"
#include "common/time.h"

#ifdef USE_SERIALRX_IBUS2
void ibus2TelemetryInit(serialPort_t *port);
void ibus2TelemetryReset(void);
void ibus2TelemetryUpdateAddress(const uint8_t *frame, size_t frameLen);
void ibus2TelemetryQueueCommand(const uint8_t *frame, size_t frameLen, timeUs_t receivedAtUs);
bool ibus2TelemetryPending(void);
bool ibus2TelemetryProcess(timeUs_t nowUs);
#endif

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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_INPUT_BACKUP

#include "drivers/rx_input_backup.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "pg/rx_input_backup.h"

#ifdef USE_RX_INPUT_BACKUP_SBUS
#include "drivers/rx_input_backup_sbus.h"
#endif

#ifdef USE_RX_INPUT_BACKUP_FBUS
#include "drivers/rx_input_backup_fbus.h"
#endif

#ifdef USE_RX_INPUT_BACKUP_FPORT
#include "drivers/rx_input_backup_fport.h"
#endif

// How long without a decoded frame before the backup link is considered down.
// ~3 missed frames at a typical ~6-14ms/frame rate - same margin the original
// SBUS-only driver used, kept here since it's a property of "how stale is too
// stale to trust", not of any one protocol's framing.
#define RX_INPUT_BACKUP_STALE_MS 50

static serialPort_t *rxInputBackupPort = NULL;
static rxInputBackupOps_t rxInputBackupOps;

static float rxInputBackupChannel[RX_INPUT_BACKUP_MAX_CHANNEL];
static timeMs_t rxInputBackupLastValidFrameMs = 0;

// False until the first genuinely valid frame has been decoded. Without this,
// rxInputBackupIsActive() would read as "active" for up to RX_INPUT_BACKUP_STALE_MS
// right after boot/config-change purely because rxInputBackupLastValidFrameMs's
// zero-init happens to be within that window of millis()'s own startup value -
// reporting the backup available (and, if the main link were already down at that
// moment, feeding zeroed channels) before any real frame has ever been seen.
static bool rxInputBackupHasValidFrame = false;

bool rxInputBackupIsEnabled(void)
{
    return rxInputBackupPort != NULL;
}

// Decodes any newly-completed frame (via the selected provider's update()) and
// refreshes freshness state. Must be called every cycle regardless of whether the
// main RX link is up or the backup is currently "needed" - it used to be called
// only as a side effect of rxInputBackupIsActive(), which rx.c only evaluates once
// the main link is already down (short-circuiting `!rxSignalReceived && ...`).
// That starved this of any real-time decoding whenever the main link was healthy,
// leaving diagnostics/MSP polling as the only thing driving it (once every poll
// interval instead of every cycle) and meaning the very first backup frame used at
// the instant of a real failover could already be stale.
void rxInputBackupPoll(void)
{
    if (!rxInputBackupIsEnabled()) {
        return;
    }

    if (rxInputBackupOps.update(rxInputBackupChannel, rxInputBackupOps.channelCount)) {
        rxInputBackupHasValidFrame = true;
        rxInputBackupLastValidFrameMs = millis();
    }

    if (rxInputBackupHasValidFrame && (timeMs_t)(millis() - rxInputBackupLastValidFrameMs) >= RX_INPUT_BACKUP_STALE_MS) {
        // No explicit parser reset here (unlike the pre-refactor SBUS-only driver) -
        // each provider's own frame-timing logic already self-heals from a stale gap
        // the moment bytes resume (see e.g. rx_input_backup_sbus.c's frameTime check),
        // so an external reset call was never load-bearing for correctness, only an
        // (unnecessary) hygiene step this generic layer would otherwise have to poke
        // back into provider-private state to perform.
        rxInputBackupHasValidFrame = false;
    }
}

bool rxInputBackupIsActive(void)
{
    if (!rxInputBackupIsEnabled() || !rxInputBackupHasValidFrame) {
        return false;
    }

    return (timeMs_t)(millis() - rxInputBackupLastValidFrameMs) < RX_INPUT_BACKUP_STALE_MS;
}

uint8_t rxInputBackupGetChannelCount(void)
{
    return rxInputBackupOps.channelCount;
}

rxInputBackupProvider_e rxInputBackupGetProvider(void)
{
    return rxInputBackupConfig()->provider;
}

float rxInputBackupGetChannel(uint8_t channel)
{
    if (channel >= rxInputBackupOps.channelCount) {
        return 0;
    }
    return rxInputBackupChannel[channel];
}

void rxInputBackupInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_INPUT_BACKUP);
    if (!portConfig) {
        rxInputBackupPort = NULL;
        return;
    }

    rxInputBackupOps = (rxInputBackupOps_t){ 0 };
    bool providerReady = false;

    switch (rxInputBackupConfig()->provider) {
#ifdef USE_RX_INPUT_BACKUP_SBUS
    case RX_INPUT_BACKUP_SBUS:
        providerReady = rxInputBackupSbusInit(&rxInputBackupOps);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_FBUS
    case RX_INPUT_BACKUP_FBUS:
        providerReady = rxInputBackupFbusInit(&rxInputBackupOps);
        break;
    case RX_INPUT_BACKUP_FPORT2:
        providerReady = rxInputBackupFport2Init(&rxInputBackupOps);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_FPORT
    case RX_INPUT_BACKUP_FPORT:
        providerReady = rxInputBackupFportInit(&rxInputBackupOps);
        break;
#endif
    case RX_INPUT_BACKUP_NONE:
    default:
        break;
    }

    if (!providerReady) {
        rxInputBackupPort = NULL;
        return;
    }

    rxInputBackupLastValidFrameMs = 0;
    rxInputBackupHasValidFrame = false;

    // Only pinSwap is applied generically here - inverted/halfDuplex are
    // protocol-specific (see rxInputBackupOps_t's own comment) and already
    // baked into rxInputBackupOps.portOptions by the provider's Init function.
    rxInputBackupPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_INPUT_BACKUP,
        rxInputBackupOps.isrFn,
        NULL,
        rxInputBackupOps.baudRate,
        MODE_RX,
        rxInputBackupOps.portOptions |
            (rxInputBackupConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

#endif // USE_RX_INPUT_BACKUP

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

#include "types.h"

#include "dprintf.h"

#ifdef USE_SERIAL_DPRINTF

#include "drivers/serial.h"

static serialPort_t * debugSerialPort = NULL;

/*
 * In the code to be debugged, call this init function with suitable
 * parameters, for example:
 *
 *      initDebugSerial(SERIAL_PORT_USART6, 921600);
 *
 * The selected serial port should not be used elsewhere.
 */

void initDebugSerial(serialPortIdentifier_e port, uint32_t baudRate)
{
    debugSerialPort = openSerialPort(port, FUNCTION_DPRINTF, NULL, NULL, baudRate, MODE_TX, 0);
    dprintf("\r\nDebug port ready\r\n");
}

/*
 * This works exactly the same as printf, but into the selected serial port.
 * Nearly all common printf formats are supported.
 * For end-of-line, use "\r\n"
 */

int dprintf(const char *fmt, ...)
{
    if (debugSerialPort) {
        va_list va;
        va_start(va, fmt);
        int written = tfp_format(debugSerialPort, (putcf)serialWrite, fmt, va);
        va_end(va);
        return written;
    }

    return 0;
}

#endif

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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * printValuePointer() and getMinMax() of cli/cli.c, copied from cli.c by the test Makefile
 * (cli_print_value.inc), with the CLI output going to a buffer.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "cli/settings.h"

// tfp_format() of the CLI. printf.c also has the serial and ITM outputs, which are not in the unit test.
#define ITM_SendChar(c) UNUSED(c)
#include "common/printf.c"

serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function, serialReceiveCallbackPtr rxCallback,
    void *rxCallbackData, uint32_t baudrate, portMode_e mode, portOptions_e options)
{
    UNUSED(identifier);
    UNUSED(function);
    UNUSED(rxCallback);
    UNUSED(rxCallbackData);
    UNUSED(baudrate);
    UNUSED(mode);
    UNUSED(options);
    return NULL;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    UNUSED(instance);
    UNUSED(ch);
}

#include "blackbox_params_unittest_cli.h"

static char *cliOut;
static bool cliCorrupted;

static void cliPutc(void *p, char c)
{
    (void)p;
    *cliOut++ = c;
}

static void cliPrintf(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    tfp_format(NULL, cliPutc, format, va);
    va_end(va);
}

static void cliPrint(const char *str)
{
    while (*str) {
        *cliOut++ = *str++;
    }
}

static void cliPrintLinefeed(void)
{
}

static void cliPrintError(const char *cmdName, const char *format, ...)
{
    (void)cmdName;
    (void)format;
    cliCorrupted = true;
}

#include "cli_print_value.inc"

bool cliTestPrintValue(char *out, const clivalue_t *var, const void *valuePointer)
{
    cliOut = out;
    cliCorrupted = false;
    printValuePointer("get", var, valuePointer, false);
    *cliOut = 0;
    return !cliCorrupted;
}

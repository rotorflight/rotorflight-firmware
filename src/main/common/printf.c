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

 /*
 * Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "typeconversion.h"

#include "printf.h"

#undef REQUIRE_PRINTF_LONG_SUPPORT

static putc_f   stdout_putf = NULL;
static void *   stdout_putp = NULL;

// print bf, padded from left to at least n characters.
// padding is zero ('0') if z!=0, space (' ') otherwise
static int putchw(void *putp, putc_f putf, int n, char z, char *bf)
{
    int written = 0;
    char fc = z ? '0' : ' ';
    char ch;
    char *p = bf;
    while (*p++ && n > 0)
        n--;
    while (n-- > 0) {
        putf(putp, fc); written++;
    }
    while ((ch = *bf++)) {
        putf(putp, ch); written++;
    }
    return written;
}

int tfp_format(void *putp, putc_f putf, const char *fmt, va_list va)
{
    int written = 0;
    char bf[21];
    char ch;

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putf(putp, ch);
            written++;
        } else {
            char lz = 0;
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
                case 0:
                    goto abort;
                case 'u':
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                    else
#endif
                        ui2a(va_arg(va, unsigned int), 10, 0, bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                case 'd':
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        li2a(va_arg(va, long int), bf);
                    else
#endif
                        i2a(va_arg(va, int), bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                case 'x':
                case 'X':
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                    else
#endif
                        ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                case 'c':
                    putf(putp, (char) (va_arg(va, int))); written++;
                    break;
                case 's':
                    {
                        char *str = va_arg(va, char *);
                        written += putchw(putp, putf, w, 0, str ? str : "(null)");
                    }
                    break;
                case '%':
                    putf(putp, ch); written++;
                    break;
                case 'n':
                    *va_arg(va, int*) = written;
                    break;
                default:
                    break;
            }
        }
    }
abort:
    return written;
}


static void str_putc(void *p, char c)
{
    *(*((char **)p))++ = c;
}

int tfp_sprintf(char *s, const char *fmt, ...)
{
    int written = 0;

    if (s) {
        va_list va;
        va_start(va, fmt);
        written = tfp_format(&s, str_putc, fmt, va);
        str_putc(&s, 0);
        va_end(va);
    }

    return written;
}

static void serial_putc(void *p, char c)
{
    serialWrite((serialPort_t *)p, c);
}

void printfSerialInit(serialPortIdentifier_e port, uint32_t baudRate, portOptions_e options)
{
    stdout_putp = openSerialPort(port, FUNCTION_PRINTF, NULL, NULL, baudRate, MODE_TX, options);
    stdout_putf = serial_putc;
}

static void itm_putc(void *p, char c)
{
    UNUSED(p);
    ITM_SendChar(c);
}

void printfITMInit(void)
{
    stdout_putp = ITM;
    stdout_putf = itm_putc;
}

int tfp_printf(const char *fmt, ...)
{
    int written = 0;

    if (stdout_putf && stdout_putp) {
        va_list va;
        va_start(va, fmt);
        written = tfp_format(stdout_putp, stdout_putf, fmt, va);
        va_end(va);
    }

    return written;
}


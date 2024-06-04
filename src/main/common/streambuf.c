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

#include <string.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "streambuf.h"

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end)
{
    sbuf->ptr = ptr;
    sbuf->end = end;
    return sbuf;
}

#define SBUFPUSH(dst, val)      ((*(dst)->ptr++ = (val)))
#define SBUFPOP(src)            ((*(src)->ptr++))

void sbufWriteU8(sbuf_t *dst, uint8_t val)
{
    SBUFPUSH(dst,val);
}

void sbufWriteU16(sbuf_t *dst, uint16_t val)
{
    SBUFPUSH(dst, val >> 0);
    SBUFPUSH(dst, val >> 8);
}

void sbufWriteU16BE(sbuf_t *dst, uint16_t val)
{
    SBUFPUSH(dst, val >> 8);
    SBUFPUSH(dst, val >> 0);
}

void sbufWriteU24(sbuf_t *dst, uint32_t val)
{
    SBUFPUSH(dst, val >> 0);
    SBUFPUSH(dst, val >> 8);
    SBUFPUSH(dst, val >> 16);
}

void sbufWriteU24BE(sbuf_t *dst, uint32_t val)
{
    SBUFPUSH(dst, val >> 16);
    SBUFPUSH(dst, val >> 8);
    SBUFPUSH(dst, val >> 0);
}

void sbufWriteU32(sbuf_t *dst, uint32_t val)
{
    SBUFPUSH(dst, val >> 0);
    SBUFPUSH(dst, val >> 8);
    SBUFPUSH(dst, val >> 16);
    SBUFPUSH(dst, val >> 24);
}

void sbufWriteU32BE(sbuf_t *dst, uint32_t val)
{
    SBUFPUSH(dst, val >> 24);
    SBUFPUSH(dst, val >> 16);
    SBUFPUSH(dst, val >> 8);
    SBUFPUSH(dst, val >> 0);
}

void sbufWriteU64(sbuf_t *dst, uint64_t val)
{
    SBUFPUSH(dst, val >> 0);
    SBUFPUSH(dst, val >> 8);
    SBUFPUSH(dst, val >> 16);
    SBUFPUSH(dst, val >> 24);
    SBUFPUSH(dst, val >> 32);
    SBUFPUSH(dst, val >> 40);
    SBUFPUSH(dst, val >> 48);
    SBUFPUSH(dst, val >> 56);
}

void sbufWriteU64BE(sbuf_t *dst, uint64_t val)
{
    SBUFPUSH(dst, val >> 56);
    SBUFPUSH(dst, val >> 48);
    SBUFPUSH(dst, val >> 40);
    SBUFPUSH(dst, val >> 32);
    SBUFPUSH(dst, val >> 24);
    SBUFPUSH(dst, val >> 16);
    SBUFPUSH(dst, val >> 8);
    SBUFPUSH(dst, val >> 0);
}

void sbufWriteFloat(sbuf_t *dst, float val)
{
    sbufWriteU32(dst, REINTERPRET_CAST(val, uint32_t));
}


void sbufFill(sbuf_t *dst, uint8_t data, int len)
{
    memset(dst->ptr, data, len);
    dst->ptr += len;
}

void sbufWriteData(sbuf_t *dst, const void *data, int len)
{
    memcpy(dst->ptr, data, len);
    dst->ptr += len;
}

void sbufWriteString(sbuf_t *dst, const char *string)
{
    while (*string) { SBUFPUSH(dst, *string++); }
}

void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string)
{
    do { SBUFPUSH(dst, *string); } while (*string++);
}

uint8_t sbufReadU8(sbuf_t *src)
{
    return SBUFPOP(src);
}

uint16_t sbufReadU16(sbuf_t *src)
{
    uint32_t ret;
    ret  = SBUFPOP(src) << 0;
    ret |= SBUFPOP(src) << 8;
    return ret;
}

uint16_t sbufReadU16BE(sbuf_t *src)
{
    uint32_t ret;
    ret  = SBUFPOP(src) << 8;
    ret |= SBUFPOP(src) << 0;
    return ret;
}

uint32_t sbufReadU24(sbuf_t *src)
{
    uint32_t ret;
    ret  = SBUFPOP(src) << 0;
    ret |= SBUFPOP(src) << 8;
    ret |= SBUFPOP(src) << 16;
    return ret;
}

uint32_t sbufReadU24BE(sbuf_t *src)
{
    uint32_t ret;
    ret  = SBUFPOP(src) << 16;
    ret |= SBUFPOP(src) << 8;
    ret |= SBUFPOP(src) << 0;
    return ret;
}

uint32_t sbufReadU32(sbuf_t *src)
{
    uint32_t ret;
    ret  = SBUFPOP(src) <<  0;
    ret |= SBUFPOP(src) <<  8;
    ret |= SBUFPOP(src) << 16;
    ret |= SBUFPOP(src) << 24;
    return ret;
}

uint32_t sbufReadU32BE(sbuf_t *src)
{
    uint32_t ret;
    ret  = SBUFPOP(src) << 24;
    ret |= SBUFPOP(src) << 16;
    ret |= SBUFPOP(src) <<  8;
    ret |= SBUFPOP(src) <<  0;
    return ret;
}

uint64_t sbufReadU64(sbuf_t *src)
{
    uint64_t a = sbufReadU32(src);
    uint64_t b = sbufReadU32(src);
    return a | (b << 32);
}

uint64_t sbufReadU64BE(sbuf_t *src)
{
    uint64_t a = sbufReadU32(src);
    uint64_t b = sbufReadU32(src);
    return (a << 32) | b;
}

float sbufReadFloat(sbuf_t *src)
{
    return REINTERPRET_CAST(sbufReadU32(src), float);
}

void sbufReadData(sbuf_t *src, void *data, int len)
{
    memcpy(data, src->ptr, len);
}

// modifies streambuf so that written data are prepared for reading
void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
{
    buf->end = buf->ptr;
    buf->ptr = base;
}

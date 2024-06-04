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

#include <stdint.h>

// simple buffer-based serializer/deserializer without implicit size check

typedef struct sbuf_s {
    uint8_t *ptr;          // data pointer must be first (sbuf_t* is equivalent to uint8_t **)
    uint8_t *end;
} sbuf_t;

/* Init function */
sbuf_t * sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end);

/* Write Unsigned Integers */
void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufWriteU24(sbuf_t *dst, uint32_t val);
void sbufWriteU64(sbuf_t *dst, uint64_t val);

/* Write Unsigned Integers in Big-Endian */
void sbufWriteU16BE(sbuf_t *dst, uint16_t val);
void sbufWriteU24BE(sbuf_t *dst, uint32_t val);
void sbufWriteU32BE(sbuf_t *dst, uint32_t val);
void sbufWriteU64BE(sbuf_t *dst, uint64_t val);

/* Compat */
static inline void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val) { sbufWriteU16BE(dst, val); }
static inline void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val) { sbufWriteU32BE(dst, val); }

/* Write Signed Integers */
static inline void sbufWriteS8(sbuf_t *dst, int8_t val)         { sbufWriteU8(dst, val); }
static inline void sbufWriteS16(sbuf_t *dst, int16_t val)       { sbufWriteU16(dst, val); }
static inline void sbufWriteS24(sbuf_t *dst, int32_t val)       { sbufWriteU24(dst, val); }
static inline void sbufWriteS32(sbuf_t *dst, int32_t val)       { sbufWriteU32(dst, val); }
static inline void sbufWriteS64(sbuf_t *dst, int64_t val)       { sbufWriteU64(dst, val); }

/* Write Signed Integers in Big-Endian */
static inline void sbufWriteS16BE(sbuf_t *dst, int16_t val)     { sbufWriteU16BE(dst, val); }
static inline void sbufWriteS24BE(sbuf_t *dst, int32_t val)     { sbufWriteU24BE(dst, val); }
static inline void sbufWriteS32BE(sbuf_t *dst, int32_t val)     { sbufWriteU32BE(dst, val); }
static inline void sbufWriteS64BE(sbuf_t *dst, int64_t val)     { sbufWriteU64BE(dst, val); }

/* Write Float */
void sbufWriteFloat(sbuf_t *dst, float val);

/* Write Data and strings */
void sbufFill(sbuf_t *dst, uint8_t data, int len);
void sbufWriteData(sbuf_t *dst, const void *data, int len);
void sbufWriteString(sbuf_t *dst, const char *string);
void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string);

/* Read Unsigned Integers */
uint8_t sbufReadU8(sbuf_t *src);
uint16_t sbufReadU16(sbuf_t *src);
uint32_t sbufReadU32(sbuf_t *src);
uint64_t sbufReadU64(sbuf_t *src);

/* Read Big-Ending Unsigned Integers */
uint8_t sbufReadU8BE(sbuf_t *src);
uint16_t sbufReadU16BE(sbuf_t *src);
uint32_t sbufReadU32BE(sbuf_t *src);
uint64_t sbufReadU64BE(sbuf_t *src);

/* Read Signed Integers */
static inline int8_t sbufReadS8(sbuf_t *src)        { return sbufReadU8(src); }
static inline int16_t sbufReadS16(sbuf_t *src)      { return sbufReadU16(src); }
static inline int32_t sbufReadS32(sbuf_t *src)      { return sbufReadU32(src); }
static inline int64_t sbufReadS64(sbuf_t *src)      { return sbufReadU64(src); }

/* Read Big-Endian Signed Integers */
static inline int16_t sbufReadS16BE(sbuf_t *src)    { return sbufReadU16BE(src); }
static inline int32_t sbufReadS32BE(sbuf_t *src)    { return sbufReadU32BE(src); }
static inline int64_t sbufReadS64BE(sbuf_t *src)    { return sbufReadU64BE(src); }

/* Read float */
float sbufReadFloat(sbuf_t *src);

/* Read Data and Strings */
void sbufReadData(sbuf_t *dst, void *data, int len);

/* Space left in the buffer */
static inline int sbufBytesRemaining(sbuf_t *buf) { return (buf->end - buf->ptr); }

/* Get buffer pointer */
static inline uint8_t * sbufPtr(sbuf_t *buf) { return buf->ptr; }
static inline const uint8_t * sbufConstPtr(const sbuf_t *buf) { return buf->ptr; }

/* Move pointer forward */
static inline void sbufAdvance(sbuf_t *buf, int size) { buf->ptr += size; }

/* Set to another position */
static inline void sbufReset(sbuf_t *buf, uint8_t *ptr) { buf->ptr = ptr; }

/* Prepare buffer for reading */
void sbufSwitchToReader(sbuf_t *buf, uint8_t * base);

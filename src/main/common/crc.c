/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "streambuf.h"

#include "common/crc.h"


uint8_t crc8_calc(uint8_t crc, uint8_t data, uint8_t poly)
{
    crc ^= data;

    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ poly;
        } else {
            crc = crc << 1;
        }
    }

    return crc;
}

uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly)
{
    const uint8_t *ptr = data;
    const uint8_t *pend = ptr + length;

    while (ptr != pend)
        crc = crc8_calc(crc, *ptr++, poly);

    return crc;
}

void crc8_sbuf_append(sbuf_t *dst, const void *data, uint8_t poly)
{
    const uint8_t *ptr = data;
    const uint8_t *pend = sbufPtr(dst);
    uint8_t crc = 0;

    while (ptr != pend)
        crc = crc8_calc(crc, *ptr++, poly);

    sbufWriteU8(dst, crc);
}

uint16_t crc16_calc(uint16_t crc, uint8_t data, uint16_t poly)
{
    crc ^= (uint16_t)data << 8;

    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ poly;
        } else {
            crc = crc << 1;
        }
    }

    return crc;
}

uint16_t crc16_update(uint16_t crc, const void *data, uint32_t length, uint16_t poly)
{
    const uint8_t *ptr = data;
    const uint8_t *pend = ptr + length;

    while (ptr != pend)
        crc = crc16_calc(crc, *ptr++, poly);

    return crc;
}

void crc16_sbuf_append(sbuf_t *dst, const void *data, uint16_t poly)
{
    const uint8_t *ptr = data;
    const uint8_t *pend = sbufPtr(dst);
    uint16_t crc = 0;

    while (ptr != pend)
        crc = crc16_calc(crc, *ptr++, poly);

    sbufWriteU16(dst, crc);
}

uint8_t crc8_xor_update(uint8_t crc, const void *data, uint32_t length)
{
    const uint8_t *ptr = data;
    const uint8_t *pend = ptr + length;

    while (ptr != pend)
        crc^= *ptr++;

    return crc;
}

void crc8_xor_sbuf_append(sbuf_t *dst, const void *data)
{
    const uint8_t *ptr = data;
    const uint8_t *pend = sbufPtr(dst);
    uint8_t crc = 0;

    while (ptr != pend)
        crc^= *ptr++;

    sbufWriteU8(dst, crc);
}

// Fowler–Noll–Vo hash function; see https://en.wikipedia.org/wiki/Fowler–Noll–Vo_hash_function
uint32_t fnv_update(uint32_t hash, const void *data, uint32_t length)
{
    const uint8_t *ptr = data;
    const uint8_t *pend = ptr + length;

    while (ptr != pend) {
        hash *= FNV_PRIME;
        hash ^= *ptr++;
    }

    return hash;
}


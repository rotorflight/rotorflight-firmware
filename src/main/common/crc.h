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

#pragma once

struct sbuf_s;

uint8_t crc8_calc(uint8_t crc, uint8_t data, uint8_t poly);
uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);
void crc8_sbuf_append(struct sbuf_s *dst, const void *data, uint8_t poly);

#define crc8_kiss_update(crc, data, len)            crc8_update((crc), (data), (len), 0x07)

#define crc8_dvb_s2(crc, data)                      crc8_calc((crc), (data), 0xD5)
#define crc8_dvb_s2_update(crc, data, len)          crc8_update((crc), (data), (len), 0xD5)
#define crc8_dvb_s2_sbuf_append(dst, data)          crc8_sbuf_append((dst), (data), 0xD5)

#define crc8_poly_0xba(crc, data)                   crc8_calc((crc), (data), 0xBA)
#define crc8_poly_0xba_sbuf_append(dst, data)       crc8_sbuf_append((dst), (data), 0xBA)

uint16_t crc16_calc(uint16_t crc, uint8_t data, uint16_t poly);
uint16_t crc16_update(uint16_t crc, const void *data, uint32_t length, uint16_t poly);
void crc16_sbuf_append(struct sbuf_s *dst, const void *data, uint16_t poly);

#define crc16_ccitt(crc, data)                      crc16_calc((crc), (data), 0x1021)
#define crc16_ccitt_update(crc, data, len)          crc16_update((crc), (data), (len), 0x1021)
#define crc16_ccitt_sbuf_append(dst, data)          crc16_sbuf_append((dst), (data), 0x1021)

uint8_t crc8_xor_update(uint8_t crc, const void *data, uint32_t length);
void crc8_xor_sbuf_append(struct sbuf_s *dst, const void *data);

#define FNV_PRIME           16777619
#define FNV_OFFSET_BASIS    2166136261

uint32_t fnv_update(uint32_t hash, const void *data, uint32_t length);

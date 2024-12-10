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

#define FLASHFS_WRITE_BUFFER_SIZE 256
#define FLASHFS_WRITE_BUFFER_USABLE (FLASHFS_WRITE_BUFFER_SIZE - 1)

void flashfsEraseCompletely(void);
void flashfsEraseRange(uint32_t start, uint32_t end);

uint32_t flashfsGetSize(void);
uint32_t flashfsGetOffset(void);
uint32_t flashfsGetWriteBufferFreeSpace(void);
uint32_t flashfsGetWriteBufferSize(void);
int flashfsIdentifyStartOfFreeSpace(void);
struct flashGeometry_s;
const struct flashGeometry_s* flashfsGetGeometry(void);

void flashfsSeekPhysical(uint32_t offset);

void flashfsWriteByte(uint8_t byte);
void flashfsWrite(const uint8_t *data, unsigned int len);

int flashfsReadAbs(uint32_t offset, uint8_t *data, unsigned int len);
int flashfsReadPhysical(uint32_t offset, uint8_t *data, unsigned int len);

bool flashfsFlushAsync(void);
void flashfsFlushSync(void);
void flashfsEraseAsync(void);

void flashfsClose(void);
void flashfsInit(void);
bool flashfsIsSupported(void);

bool flashfsIsReady(void);
bool flashfsIsEOF(void);

void flashfsFillEntireFlash(void);
bool flashfsVerifyEntireFlash(void);

#ifdef USE_FLASHFS_LOOP
void flashfsLoopInitialErase();

uint32_t flashfsGetHeadAddress();
uint32_t flashfsGetTailAddress();
#endif

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

#include "drivers/flash.h"
#include "pg/flash.h"
#include <cstdint>

class FlashInterface {
  public:
    virtual ~FlashInterface() {};

    virtual void flashPreInit(const flashConfig_t *flashConfig) = 0;
    virtual bool flashInit(const flashConfig_t *flashConfig) = 0;

    virtual bool flashIsReady(void) = 0;
    virtual bool flashWaitForReady(void) = 0;
    virtual void flashEraseSector(uint32_t address) = 0;
    virtual void flashEraseCompletely(void) = 0;
    virtual void flashPageProgramBegin(uint32_t address,
                                       void (*callback)(uint32_t arg)) = 0;
    virtual uint32_t flashPageProgramContinue(const uint8_t **buffers,
                                              uint32_t *bufferSizes,
                                              uint32_t bufferCount) = 0;
    virtual void flashPageProgramFinish(void) = 0;
    virtual void flashPageProgram(uint32_t address, const uint8_t *data,
                                  uint32_t length,
                                  void (*callback)(uint32_t length)) = 0;
    virtual int flashReadBytes(uint32_t address, uint8_t *buffer,
                               uint32_t length) = 0;
    virtual void flashFlush(void) = 0;
    virtual const flashGeometry_t *flashGetGeometry(void) = 0;

    virtual void flashPartitionSet(uint8_t index, uint32_t startSector,
                                   uint32_t endSector) = 0;
    virtual flashPartition_t *
    flashPartitionFindByType(flashPartitionType_e type) = 0;
    virtual const flashPartition_t *
    flashPartitionFindByIndex(uint8_t index) = 0;
    virtual const char *
    flashPartitionGetTypeName(flashPartitionType_e type) = 0;
    virtual int flashPartitionCount(void) = 0;
};

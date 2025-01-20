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
#include "gmock/gmock.h"

#include "flash_interface.h"
#include "flash_emulator.h"

class FlashMock : public FlashInterface {
  public:
    MOCK_METHOD(void, flashPreInit, (const flashConfig_t *flashConfig), (override));
    MOCK_METHOD(bool, flashInit, (const flashConfig_t *flashConfig), (override));
    MOCK_METHOD(bool, flashIsReady, (), (override));
    MOCK_METHOD(bool, flashWaitForReady, (), (override));
    MOCK_METHOD(void, flashEraseSector, (uint32_t address), (override));
    MOCK_METHOD(void, flashEraseCompletely, (), (override));
    MOCK_METHOD(void, flashPageProgramBegin,
                (uint32_t address, void (*callback)(uint32_t arg)), (override));
    MOCK_METHOD(uint32_t, flashPageProgramContinue,
                (const uint8_t **buffers, uint32_t *bufferSizes,
                 uint32_t bufferCount), (override));
    MOCK_METHOD(void, flashPageProgramFinish, (), (override));
    MOCK_METHOD(void, flashPageProgram,
                (uint32_t address, const uint8_t *data, uint32_t length,
                 void (*callback)(uint32_t length)), (override));
    MOCK_METHOD(int, flashReadBytes,
                (uint32_t address, uint8_t *buffer, uint32_t length), (override));
    MOCK_METHOD(void, flashFlush, (), (override));
    MOCK_METHOD(const flashGeometry_t *, flashGetGeometry, (), (override));
    MOCK_METHOD(void, flashPartitionSet,
                (uint8_t index, uint32_t startSector, uint32_t endSector), (override));
    MOCK_METHOD(flashPartition_t *, flashPartitionFindByType,
                (flashPartitionType_e type), (override));
    MOCK_METHOD(const flashPartition_t *, flashPartitionFindByIndex,
                (uint8_t index), (override));
    MOCK_METHOD(const char *, flashPartitionGetTypeName,
                (flashPartitionType_e type), (override));
    MOCK_METHOD(int, flashPartitionCount, (), (override));
};

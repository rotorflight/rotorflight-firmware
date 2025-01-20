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
#include "flash_interface.h"
#include <cstdint>
#include <memory>
#include <cassert>

class FlashEmulator : public FlashInterface {
  public:
    // Implement Flash Interface
    void flashPreInit(const flashConfig_t *flashConfig) override;
    bool flashInit(const flashConfig_t *flashConfig) override;

    bool flashIsReady(void) override;
    bool flashWaitForReady(void) override;
    void flashEraseSector(uint32_t address) override;
    void flashEraseCompletely(void) override;
    void flashPageProgramBegin(uint32_t address,
                               void (*callback)(uint32_t arg)) override;
    uint32_t flashPageProgramContinue(const uint8_t **buffers,
                                      uint32_t *bufferSizes,
                                      uint32_t bufferCount) override;
    void flashPageProgramFinish(void) override;
    void flashPageProgram(uint32_t address, const uint8_t *data,
                          uint32_t length,
                          void (*callback)(uint32_t length)) override;
    int flashReadBytes(uint32_t address, uint8_t *buffer,
                       uint32_t length) override;
    void flashFlush(void) override;
    const flashGeometry_t *flashGetGeometry(void) override;

    void flashPartitionSet(uint8_t index, uint32_t startSector,
                           uint32_t endSector) override;
    flashPartition_t *
    flashPartitionFindByType(flashPartitionType_e type) override;
    const flashPartition_t *flashPartitionFindByIndex(uint8_t index) override;
    const char *flashPartitionGetTypeName(flashPartitionType_e type) override;
    int flashPartitionCount(void) override;

    ~FlashEmulator() override {}

    // Other internal stuff
  public:
    enum FlashType {
        kFlashW25N01G,
        kFlashW25Q128FV,
        kFlashM25P16,
        kFlashRAM,
    };

    // Full constructor
    FlashEmulator(FlashType flash_type, uint16_t page_size,
                  uint16_t pages_per_sector, uint16_t sectors,
                  uint16_t flashfs_start, uint16_t flashfs_size)
        : kFlashType(flash_type), kPageSize(page_size),
          kPagesPerSector(pages_per_sector), kSectors(sectors),
          kFlashFSStartSector(flashfs_start),
          kFlashFSSizeInSectors(flashfs_size),

          kFlashFSPartition({.type = FLASH_PARTITION_TYPE_FLASHFS,
                             .startSector = flashfs_start,
                             .endSector = static_cast<uint16_t>(
                                 flashfs_start + flashfs_size - 1)}),
          kFlashGeometry({.sectors = kSectors,
                          .pageSize = kPageSize,
                          .sectorSize = kSectorSize,
                          .totalSize = kFlashSize,
                          .pagesPerSector = kPagesPerSector,
                          .flashType = flash_type == kFlashW25N01G
                                           ? FLASH_TYPE_NAND
                                           : FLASH_TYPE_NOR}) {

        // flash.c and its user (flashfs.c) assumes flashfs partition starts
        // from 0. Let's make sure the test assume the same.
        assert(flashfs_start == 0);

        memory_ = std::make_unique<uint8_t[]>(kFlashSize);
        memset(memory_.get(), 0xff, kFlashSize);
        write_buffer_ = std::make_unique<uint8_t[]>(kPageSize);
    }

    // Default geometry
    FlashEmulator(FlashType flash_type)
        : FlashEmulator(flash_type, /*page_size=*/2048, /*pages_per_sector=*/4,
                        /*sectors=*/64, /*flashfs_start=*/0,
                        /*flashfs_size=*/16) {}

    // Default type and geometry
    FlashEmulator() : FlashEmulator(kFlashW25N01G) {}

  public:
    const FlashType kFlashType;
    const uint16_t kPageSize;
    const uint16_t kPagesPerSector;
    const uint16_t kSectors;
    const uint32_t kSectorSize = kPageSize * kPagesPerSector;
    const uint32_t kFlashSize = kSectors * kSectorSize;
    const uint16_t kFlashFSStartSector;
    const uint16_t kFlashFSStart = kFlashFSStartSector * kSectorSize;
    const uint16_t kFlashFSSizeInSectors;
    const uint32_t kFlashFSSize = kFlashFSSizeInSectors * kSectorSize;
    const uint32_t kFlashFSEnd = kFlashFSStart + kFlashFSSize;  // This is end+1

    flashPartition_t kFlashFSPartition;
    const flashGeometry_t kFlashGeometry;

    std::unique_ptr<uint8_t[]> memory_ = nullptr;
    void (*write_callback_)(uint32_t arg) = NULL;

    uint32_t writepoint_ = 0;
    std::unique_ptr<uint8_t[]> write_buffer_ = nullptr;
    uint32_t write_buffer_tail_ = 0;

    // Internal flash state.
    // enum is thread safe (lock-free)
    enum FlashState {
        kFlashStateIdle,
        kFlashStateProgramming,
        kFlashStateErasing,
    } flash_state_ = kFlashStateIdle;

    // Reuseable actual "program" cmd
    void Program();
    // Helper functions for testing
    uint32_t GetSectorIndex(uint32_t address);
    uint32_t GetPageIndex(uint32_t address);
    bool IsErased(uint32_t address, uint32_t length);
    bool IsValidFlashFSAddress(uint32_t address);
    void Fill(uint32_t address, uint8_t byte, uint32_t length);
    void FillSector(uint16_t sector, uint8_t byte, uint16_t count);
};

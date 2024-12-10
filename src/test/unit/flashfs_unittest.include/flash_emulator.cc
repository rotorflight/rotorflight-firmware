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

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "drivers/flash.h"
#include "pg/flash.h"

#include "flash_emulator.h"

void FlashEmulator::flashPreInit(const flashConfig_t *flashConfig) {
    UNUSED(flashConfig);
}
bool FlashEmulator::flashInit(const flashConfig_t *flashConfig) {
    UNUSED(flashConfig);
    return true;
}
bool FlashEmulator::flashIsReady(void) {
    return flash_state_ == kFlashStateIdle;
}
bool FlashEmulator::flashWaitForReady(void) {
    while (flash_state_ != kFlashStateIdle)
        ;
    return true;
}
void FlashEmulator::flashEraseSector(uint32_t address) {
    // Both w25n01g driver and w25q128fv driver wait for idle here.
    // The logic behind m25p16 driver is unclear.
    flashWaitForReady();
    assert(flash_state_ == kFlashStateIdle);
    flash_state_ = kFlashStateErasing;
    std::thread erase_thread([&, address]() {
        // Wait for erase time
        switch (kFlashType) {
        case kFlashW25N01G:
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            break;
        case kFlashW25Q128FV:
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            break;
        case kFlashM25P16:
            std::this_thread::sleep_for(std::chrono::seconds(3));
            break;
        case kFlashRAM:
            break;
        }
        uint32_t sector = GetSectorIndex(address);
        uint32_t base_address = sector * kSectorSize;
        memset(&memory_[base_address], 0xff, kSectorSize);
        flash_state_ = kFlashStateIdle;
    });
    erase_thread.detach();
};
void FlashEmulator::flashEraseCompletely(void) {
    assert(flash_state_ == kFlashStateIdle);
    flash_state_ = kFlashStateErasing;
    std::thread erase_thread([&]() {
        // Wait for erase time
        switch (kFlashType) {
        case kFlashW25N01G:
            std::this_thread::sleep_for(
                std::chrono::milliseconds(10 * kSectors));
            break;
        case kFlashW25Q128FV:
            std::this_thread::sleep_for(std::chrono::seconds(200));
            break;
        case kFlashM25P16:
            std::this_thread::sleep_for(std::chrono::seconds(40));
            break;
        case kFlashRAM:
            break;
        }
        memset(&memory_[0], 0xff, kFlashSize);
        flash_state_ = kFlashStateIdle;
    });
    erase_thread.detach();
}
void FlashEmulator::flashPageProgramBegin(uint32_t address,
                                          void (*callback)(uint32_t arg)) {
    assert(flash_state_ == kFlashStateIdle);
    writepoint_ = address;
    write_callback_ = callback;
    write_buffer_tail_ = 0;
    memset(&write_buffer_[0], 0x0, kPageSize);
    //    printf("ProgramBegin: address[0x%x]", address);
}
uint32_t FlashEmulator::flashPageProgramContinue(const uint8_t **buffers,
                                                 uint32_t *bufferSizes,
                                                 uint32_t bufferCount) {
    assert(flash_state_ == kFlashStateIdle);
    uint32_t written = 0;
    for (uint32_t i = 0; i < bufferCount; i++) {
        for (uint32_t j = 0; j < bufferSizes[i]; j++) {
            write_buffer_[write_buffer_tail_] = buffers[i][j];
            assert(write_buffer_tail_ <= kPageSize);
            write_buffer_tail_++;
            written++;
        }
    }

    //    printf("ProgramContinue: written[0x%x]", written);
    return written;
}
void FlashEmulator::flashPageProgramFinish(void) {
    //    printf("ProgramFinish\n");
    if (write_buffer_tail_ != 0)
        Program();
}
void FlashEmulator::flashPageProgram(uint32_t address, const uint8_t *data,
                                     uint32_t length,
                                     void (*callback)(uint32_t length)) {
    flashPageProgramBegin(address, callback);
    flashPageProgramContinue(&data, &length, 1);
    flashPageProgramFinish();
}
int FlashEmulator::flashReadBytes(uint32_t address, uint8_t *buffer,
                                  uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        assert(IsValidFlashFSAddress(address));
        *buffer = memory_[address];
        buffer++;
        address++;
    }
    return length;
}
void FlashEmulator::flashFlush(void) {}
const flashGeometry_t *FlashEmulator::flashGetGeometry(void) {
    return &kFlashGeometry;
}

void FlashEmulator::flashPartitionSet(uint8_t index, uint32_t startSector,
                                      uint32_t endSector) {
    // NOP
    UNUSED(index);
    UNUSED(startSector);
    UNUSED(endSector);
}
flashPartition_t *
FlashEmulator::flashPartitionFindByType(flashPartitionType_e type) {
    if (type == FLASH_PARTITION_TYPE_FLASHFS)
        return &kFlashFSPartition;
    return NULL;
}
const flashPartition_t *
FlashEmulator::flashPartitionFindByIndex(uint8_t index) {
    if (index == 0)
        return &kFlashFSPartition;
    return NULL;
}
const char *
FlashEmulator::flashPartitionGetTypeName(flashPartitionType_e type) {
    UNUSED(type);
    return "FakeTypeFlashfs";
}
int FlashEmulator::flashPartitionCount(void) { return 1; }

void FlashEmulator::Program() {
    assert(flash_state_ == kFlashStateIdle);
    flash_state_ = kFlashStateProgramming;
    std::thread program_thread([&]() {
        // Wait for program time
        switch (kFlashType) {
        case kFlashW25N01G:
            std::this_thread::sleep_for(std::chrono::microseconds(700));
            break;
        case kFlashW25Q128FV:
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            break;
        case kFlashM25P16:
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            break;
        case kFlashRAM:
            break;
        }
        // writepoint_ within size
        assert(IsValidFlashFSAddress(writepoint_));

        // Write to erased page only
        assert(IsErased(writepoint_, write_buffer_tail_));
        memcpy(&memory_[writepoint_], &write_buffer_[0], write_buffer_tail_);

        if (write_callback_ != NULL) {
            write_callback_(write_buffer_tail_);
        }
        // If the state becomes idle before callback is called, the flashfs will
        // overrun and multiple callbacks in different threads will have race.
        // When running on the real hardware, callback is called from ISR. So by
        // nature the write can't overrun. Here let's emulate by clearing the
        // state after callback's done.
        flash_state_ = kFlashStateIdle;
    });

    program_thread.detach();
}
uint32_t FlashEmulator::GetSectorIndex(uint32_t address) {
    assert(IsValidFlashFSAddress(address));
    return address / kSectorSize;
}
uint32_t FlashEmulator::GetPageIndex(uint32_t address) {
    assert(IsValidFlashFSAddress(address));
    return address / kPageSize;
}
bool FlashEmulator::IsErased(uint32_t address, uint32_t length) {
    for (uint32_t i = 0; i < length; ++i) {
        if (memory_[address + i] != 0xff) {
            printf("offset address[0x%x] + i[%d] = %02x\n", address, i,
                   memory_[address + i]);
            return false;
        }
    }
    return true;
}
bool FlashEmulator::IsValidFlashFSAddress(uint32_t address) {
    return address >= kFlashFSStartSector * kSectorSize &&
           address <
               (kFlashFSStartSector + kFlashFSSizeInSectors) * kSectorSize;
}
void FlashEmulator::Fill(uint32_t address, uint8_t byte, uint32_t length) {
    memset(&memory_[address], byte, length);
}
void FlashEmulator::FillSector(uint16_t sector, uint8_t byte, uint16_t count) {
    memset(&memory_[sector * kSectorSize], byte, count * kSectorSize);
}

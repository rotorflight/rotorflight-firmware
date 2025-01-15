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

#include <stdbool.h>
#include <stdint.h>

#include <limits.h>

extern "C" {
#include "io/flashfs.h"
#include "pg/blackbox.h"

extern uint32_t flashfsSize;
extern uint32_t headAddress;
extern uint32_t tailAddress;
}

#include "flashfs_unittest.include/flash_c_stub.h"
#include "flashfs_unittest.include/flash_emulator.h"
#include "flashfs_unittest.include/flash_mock.h"

#include "unittest_macros.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

/*
 * There are some weird logic behind flashfs.c and flash.c. This unittest is
 * written to accomedate them
 *   * flashfs (and some other places) assumes the "flashfs" partition starts
 *     from sector 0. This is made true by the partition allocator.
 *   * flashfs can't handle EOF gracefully if writes are not aligned to
 *     BLOCK_SIZE (e.g., flashfs will attempt to write over EOF).
 *   * ProgramBegin() ProgramContinue() ProgramFinish() aren't page-align
 *     checked.
 *
 */

void pgReset(void) {
    // We will use the real config variable
    extern blackboxConfig_t pgResetTemplate_blackboxConfig;
    memcpy(blackboxConfigMutable(), &pgResetTemplate_blackboxConfig,
           sizeof(blackboxConfig_t));
}

class FlashFSTestBase : public ::testing::Test {
  public:
    void SetUp() override
    {
        pgReset();
        flash_emulator_ = std::make_shared<FlashEmulator>();
        g_flash_stub = flash_emulator_;
        page_size_ = flash_emulator_->kPageSize;
        sector_size_ = flash_emulator_->kSectorSize;
        flashfs_size_ = flash_emulator_->kFlashFSSize;
    }

    uint16_t page_size_;
    uint16_t sector_size_;
    uint32_t flashfs_size_;
    std::shared_ptr<FlashEmulator> flash_emulator_;
};

TEST_F(FlashFSTestBase, flashfsInit)
{
    flashfsInit();
    EXPECT_EQ(flashfsSize, flashfs_size_);
}

TEST_F(FlashFSTestBase, flashfsIdentifyStartOfFreeSpace)
{
    flashfsInit();

    constexpr uint32_t kExpectedWritepoint = 16 * 1024;
    constexpr uint32_t kFillSize = kExpectedWritepoint - 60;
    flash_emulator_->Fill(0, 0x55, kFillSize);

    uint32_t writepoint = flashfsIdentifyStartOfFreeSpace();
    EXPECT_EQ(writepoint, kExpectedWritepoint);
}

TEST_F(FlashFSTestBase, flashfsWrite)
{
    constexpr uint32_t kExpectedWritepoint1 = 16 * 1024;
    constexpr uint8_t kByte1 = 0x33;
    // Pre-fill some data
    constexpr uint32_t kFillSize = kExpectedWritepoint1 - 60;
    flash_emulator_->Fill(0, 0x55, kFillSize);

    flashfsInit();
    EXPECT_EQ(tailAddress, kExpectedWritepoint1);
    flashfsWriteByte(0x33);
    flashfsFlushSync();
    flashfsClose();
    EXPECT_EQ(flash_emulator_->memory_[kExpectedWritepoint1], kByte1);

    const uint32_t kExpectedWritepoint2 = kExpectedWritepoint1 + page_size_;
    flashfsInit();
    EXPECT_EQ(tailAddress, kExpectedWritepoint2);
}

TEST_F(FlashFSTestBase, flashfsWriteOverFlashSize)
{
    flashfsInit();
    blackboxConfigMutable()->rollingErase = false;

    // Unexpectedly, the flashfs can't handle EOF if writes are not aligned to
    // BLOCK_SIZE (2048) Let's just ignore this bug.
    // constexpr uint32_t kBufferSize = FLASHFS_WRITE_BUFFER_USABLE - 10;
    constexpr uint32_t kBufferSize = 128;
    constexpr uint8_t kByte = 0x44;
    auto buffer = std::make_unique<uint8_t[]>(kBufferSize);
    memset(buffer.get(), kByte, kBufferSize);

    EXPECT_EQ(tailAddress, 0);

    uint32_t written = 0;
    do {
        flashfsWrite(buffer.get(), kBufferSize);
        flashfsFlushSync();
        written += kBufferSize;
    } while (written <= flash_emulator_->kFlashFSSize + 5000);

    // If LOOP_FLASHFS is enabled, we don't write the last page + maybe flashfs
    // write buffer size.
    for (uint32_t i = 0; i < flash_emulator_->kFlashFSSize - page_size_ -
                                 FLASHFS_WRITE_BUFFER_SIZE;
         i++) {
        ASSERT_EQ(flash_emulator_->memory_[i], kByte)
            << "Mismatch address " << std::hex << i;
    }
}

class FlashFSBandwidthTest
    : public FlashFSTestBase,
      public testing::WithParamInterface<FlashEmulator::FlashType> {
  public:
    void SetUp() override
    {
        pgReset();
        // Create 64KiB flash emulator
        flash_emulator_ =
            std::make_shared<FlashEmulator>(GetParam(), 2048, 4, 8, 0, 8);
        g_flash_stub = flash_emulator_;
        flashfsInit();
    };
};

TEST_P(FlashFSBandwidthTest, WriteBandwidth)
{
    constexpr uint32_t kBufferSize = 128;
    constexpr uint8_t kByte = 0x44;
    auto buffer = std::make_unique<uint8_t[]>(kBufferSize);
    memset(buffer.get(), kByte, kBufferSize);

    EXPECT_EQ(tailAddress, 0);

    auto start = std::chrono::system_clock::now();

    uint32_t written = 0;
    do {
        flashfsWrite(buffer.get(), kBufferSize);
        flashfsFlushSync();
        written += kBufferSize;
    } while (written < flash_emulator_->kFlashFSSize);

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Write Bandwidth = " << 64 / elapsed_seconds.count()
              << " KiB/s." << std::endl;
    std::cout << "This is just an estimate based on the worst case from spec."
              << std::endl;
}

INSTANTIATE_TEST_SUITE_P(DISABLED_AllFlashTypes, FlashFSBandwidthTest,
                         testing::Values(FlashEmulator::kFlashW25N01G,
                                         FlashEmulator::kFlashW25Q128FV,
                                         FlashEmulator::kFlashM25P16));

class FlashFSLoopTest : public FlashFSTestBase {};

TEST_F(FlashFSLoopTest, StartFromZero)
{
    // Test when data starts from 0.
    flashfsInit();
    EXPECT_EQ(headAddress, 0);
    EXPECT_EQ(tailAddress, 0);

    // Fill begining of sector 0
    flash_emulator_->Fill(0, 0x55, 5);
    flashfsInit();
    EXPECT_EQ(headAddress, 0);
    EXPECT_EQ(tailAddress, page_size_);

    // Fill sector 0 and begining of sector 1
    flash_emulator_->FillSector(flash_emulator_->kFlashFSStartSector, 0x55, 1);
    flash_emulator_->Fill(sector_size_, 0x55, 5);
    flashfsInit();
    EXPECT_EQ(headAddress, 0);
    EXPECT_EQ(tailAddress, sector_size_ + page_size_);
    EXPECT_TRUE(flash_emulator_->IsErased(tailAddress, sector_size_));

    EXPECT_FALSE(flashfsIsEOF());
}

TEST_F(FlashFSLoopTest, Flat)
{
    // Test when data stripe is not wrapped.
    // Fill sector 1 and 2.
    flash_emulator_->Fill(1 * sector_size_, 0x55,
                          sector_size_);
    flash_emulator_->Fill(2 * sector_size_, 0x55, 5);

    flashfsInit();
    EXPECT_EQ(headAddress, sector_size_);
    EXPECT_EQ(tailAddress, 2 * sector_size_ + page_size_);
    EXPECT_TRUE(flash_emulator_->IsErased(tailAddress, sector_size_));

    EXPECT_FALSE(flashfsIsEOF());
}

TEST_F(FlashFSLoopTest, Wrapped1)
{
    // Test when data region is wrapped.
    // Fill sector -1 and partially 0.
    const uint32_t kStartOfLastSector =
        (flash_emulator_->kFlashFSStartSector +
         flash_emulator_->kFlashFSSizeInSectors - 1) *
        sector_size_;

    flash_emulator_->Fill(0, 0x55, 5);
    flash_emulator_->Fill(kStartOfLastSector, 0x55,
                          sector_size_);

    flashfsInit();
    EXPECT_EQ(headAddress, kStartOfLastSector);
    EXPECT_EQ(tailAddress, page_size_);
    EXPECT_TRUE(flash_emulator_->IsErased(tailAddress, sector_size_));
}

TEST_F(FlashFSLoopTest, Wrapped2)
{
    // Test when data region is wrapped.
    // Fill all sectors except 0.
    flash_emulator_->Fill(sector_size_, 0x55,
                          flash_emulator_->kFlashFSSize -
                              sector_size_);

    flashfsInit();
    EXPECT_EQ(headAddress, sector_size_);
    EXPECT_EQ(tailAddress, 0);
    EXPECT_TRUE(flash_emulator_->IsErased(tailAddress, sector_size_));
}

TEST_F(FlashFSLoopTest, Wrapped3)
{
    // Test when data region is wrapped.

    const uint16_t kBoundarySector = 4;
    const uint32_t kEmptyStart =
        kBoundarySector * sector_size_ - page_size_;
    const uint32_t kEmptyStop = kBoundarySector * sector_size_;

    // Fill all sectors except [kEmptyStart, kEmptyStop). The unfilled size = 1 page.
    flash_emulator_->Fill(flash_emulator_->kFlashFSStart, 0x55,
                          kEmptyStart - flash_emulator_->kFlashFSStart);
    flash_emulator_->Fill(kEmptyStop, 0x55,
                          flash_emulator_->kFlashFSEnd - kEmptyStop);

    flashfsInit();
    EXPECT_EQ(headAddress, kEmptyStop);
    EXPECT_EQ(tailAddress, kEmptyStart);
    EXPECT_TRUE(flash_emulator_->IsErased(tailAddress, page_size_));
}

TEST_F(FlashFSLoopTest, Full)
{
    // Test when flash is fully written
    flash_emulator_->Fill(0, 0x55, flash_emulator_->kFlashFSSize);

    flashfsInit();
    EXPECT_EQ(headAddress, 0);
    EXPECT_EQ(tailAddress, flash_emulator_->kFlashFSSize - page_size_);
    EXPECT_TRUE(flashfsIsEOF());

    // Fill all sectors except [0, page_size_), this is abnormal
    // and is also considered full.
    flash_emulator_->Fill(page_size_, 0x55,
                          flash_emulator_->kFlashFSSize - page_size_);

    flashfsInit();
    EXPECT_EQ(headAddress, 0);
    EXPECT_EQ(tailAddress, flash_emulator_->kFlashFSSize - page_size_);
    EXPECT_TRUE(flashfsIsEOF());
}

TEST_F(FlashFSLoopTest, WriteTest)
{
    // Test a wrapped write
    // We will start writing from the last (physical) sector.
    const uint32_t kFillStart = flashfs_size_ - sector_size_;

    flash_emulator_->Fill(kFillStart, 0x44, page_size_);
    flashfsInit();

    EXPECT_EQ(headAddress, kFillStart);
    EXPECT_EQ(tailAddress, kFillStart + page_size_);

    // Now let's write one sector, we should end at sector 0 page 0
    constexpr uint32_t kBufferSize = 128;
    auto buffer = std::make_unique<uint8_t[]>(kBufferSize);
    memset(buffer.get(), 0x55, kBufferSize);

    uint32_t written = 0;
    do {
        flashfsWrite(buffer.get(), kBufferSize);
        flashfsFlushSync();
        written += kBufferSize;
    } while (written < sector_size_);

    EXPECT_EQ(tailAddress, page_size_);

    // [kFillStart, kFillStart + page_size_) should contain original 0x44
    for (uint32_t i = kFillStart; i < kFillStart + page_size_; i++) {
        ASSERT_EQ(flash_emulator_->memory_[i], 0x44)
            << "Mismatch address " << std::hex << i;
    }

    // [kFillStart + page_size_, FlashFSEnd) should contain 0x55
    for (uint32_t i = kFillStart + page_size_; i < flashfs_size_; i++) {
        ASSERT_EQ(flash_emulator_->memory_[i], 0x55)
            << "Mismatch address " << std::hex << i;
    }

    // [0, page_size) should also contain 0x55
    for (uint32_t i = 0; i < page_size_; i++) {
        ASSERT_EQ(flash_emulator_->memory_[i], 0x55)
            << "Mismatch address " << std::hex << i;
    }
}

TEST_F(FlashFSLoopTest, WrappedRead)
{
    const uint16_t kBoundarySector = 4;
    const uint32_t kEmptyStart =
        kBoundarySector * sector_size_ - page_size_;
    const uint32_t kEmptyStop = kBoundarySector * sector_size_;

    // Fill all sectors except [kEmptyStart, kEmptyStop). The unfilled size = 1 page.
    flash_emulator_->Fill(flash_emulator_->kFlashFSStart, 0x55,
                          kEmptyStart - flash_emulator_->kFlashFSStart);
    flash_emulator_->Fill(kEmptyStop, 0x55,
                          flash_emulator_->kFlashFSEnd - kEmptyStop);
    flashfsInit();
    EXPECT_EQ(headAddress, kEmptyStop);
    EXPECT_EQ(tailAddress, kEmptyStart);
    // Read from -page_size and read 2* page_size.
    const uint32_t kPhysicalAddress = flash_emulator_->kFlashFSEnd - page_size_;
    const uint32_t kLogicalAddress = kPhysicalAddress - headAddress;
    const uint32_t kSize = 2 * page_size_;
    std::unique_ptr<uint8_t[]> buffer = std::make_unique<uint8_t[]>(kSize);
    EXPECT_GT(kPhysicalAddress, headAddress);
    EXPECT_GT(kPhysicalAddress, tailAddress);
    EXPECT_GT(headAddress, tailAddress);
    flashfsReadAbs(kLogicalAddress, buffer.get(), kSize);
    for (uint32_t i = 0; i < page_size_; i++) {
        ASSERT_EQ(buffer[i], 0x55)
            << "Read mismatch. Address " << std::hex << i;
    }
}

class FlashFSLoopInitialEraseTest : public FlashFSTestBase {
    // In this test, flashfsEraseAsync() has to run in the background.
    void SetUp() override {
        FlashFSTestBase::SetUp();
    }
    void TearDown() override {
        while(!flashfsIsReady()) {
            flashfsEraseAsync();
}
        FlashFSTestBase::TearDown();
    }
};

TEST_F(FlashFSLoopInitialEraseTest, Normal)
{
    // Initial erase happens in the contiguous area.
    const uint16_t kBoundarySector = 4;
    const uint32_t kEmptyStart =
        kBoundarySector * sector_size_ - page_size_;
    const uint32_t kEmptyStop = kBoundarySector * sector_size_;

    // Fill all sectors except [kEmptyStart, kEmptyStop). The size = 1 page.
    flash_emulator_->Fill(flash_emulator_->kFlashFSStart, 0x55,
                          kEmptyStart - flash_emulator_->kFlashFSStart);
    flash_emulator_->Fill(kEmptyStop, 0x55,
                          flash_emulator_->kFlashFSEnd - kEmptyStop);

    flashfsInit();
    EXPECT_EQ(headAddress, kEmptyStop);
    EXPECT_EQ(tailAddress, kEmptyStart);

    // Now let's try to auto erase
    const uint16_t initial_erase_kb = 16;

    blackboxConfigMutable()->initialEraseFreeSpaceKiB = initial_erase_kb;
    flashfsLoopInitialErase();

    while (!flashfsIsReady())
        flashfsEraseAsync();

    EXPECT_EQ(headAddress, kEmptyStop + initial_erase_kb * 1024);
    EXPECT_TRUE(flash_emulator_->IsErased(kEmptyStop, initial_erase_kb * 1024));
}

TEST_F(FlashFSLoopInitialEraseTest, Wrapped)
{
    // Initial erase happens in the wrap boundary -- the last sector and the
    // first sector.
    const uint16_t kBoundarySector = flash_emulator_->kFlashFSSizeInSectors - 1;
    const uint32_t kEmptyStart = kBoundarySector * sector_size_ - page_size_;
    const uint32_t kEmptyStop = kBoundarySector * sector_size_;

    // Fill all sectors except [kEmptyStart, kEmptyStop). The size = 1 page.
    flash_emulator_->Fill(flash_emulator_->kFlashFSStart, 0x55,
                          kEmptyStart - flash_emulator_->kFlashFSStart);
    flash_emulator_->Fill(kEmptyStop, 0x55,
                          flash_emulator_->kFlashFSEnd - kEmptyStop);

    flashfsInit();
    EXPECT_TRUE(flashfsIsReady());
    EXPECT_EQ(headAddress, kEmptyStop);
    EXPECT_EQ(tailAddress, kEmptyStart);

    // Now let's try to auto erase
    const uint32_t initial_erase = sector_size_ * 2;
    const uint16_t initial_erase_kb = initial_erase / 1024;
    ASSERT_EQ(initial_erase_kb * 1024, initial_erase)
        << "This test expects the exact erase size to be a multiple of 1024.";

    blackboxConfigMutable()->initialEraseFreeSpaceKiB = initial_erase_kb;
    flashfsLoopInitialErase();
    while (!flashfsIsReady())
        flashfsEraseAsync();

    // wrapped kEmptyStop + sector_size_ * 2
    EXPECT_EQ(headAddress, sector_size_);
    EXPECT_TRUE(flash_emulator_->IsErased(kEmptyStop, sector_size_));
    EXPECT_TRUE(flash_emulator_->IsErased(0, sector_size_));
}

TEST_F(FlashFSLoopInitialEraseTest, UnalignedSize)
{
    const uint16_t kBoundarySector = 4;
    const uint32_t kEmptyStart =
        kBoundarySector * sector_size_ - page_size_;
    const uint32_t kEmptyStop = kBoundarySector * sector_size_;

    // Fill all sectors except [kEmptyStart, kEmptyStop). The size = 1 page.
    flash_emulator_->Fill(flash_emulator_->kFlashFSStart, 0x55,
                          kEmptyStart - flash_emulator_->kFlashFSStart);
    flash_emulator_->Fill(kEmptyStop, 0x55,
                          flash_emulator_->kFlashFSEnd - kEmptyStop);

    flashfsInit();
    EXPECT_EQ(headAddress, kEmptyStop);
    EXPECT_EQ(tailAddress, kEmptyStart);

    // Now let's try to auto erase
    uint16_t erase_kb = 4;
    ASSERT_GT(erase_kb * 1024, page_size_)
        << "This test only makes sense if erasing more than a page size";
    ASSERT_LT(erase_kb * 1024, sector_size_)
        << "This test only makes sense if erasing less than a sector size";
    blackboxConfigMutable()->initialEraseFreeSpaceKiB = erase_kb;
    flashfsLoopInitialErase();
    while (!flashfsIsReady()) {
        flashfsEraseAsync();
    }

    EXPECT_EQ(headAddress, kEmptyStop + sector_size_);
    EXPECT_TRUE(flash_emulator_->IsErased(kEmptyStop, sector_size_));
}

TEST(InitialErase, U16KiBOverflow)
{
    auto flash_emulator = std::make_shared<FlashEmulator>(
        FlashEmulator::kFlashW25N01G, /*page_size=*/2048,
        /*pages_per_sector=*/64,
        /*sectors=*/1024, /*flashfs_start=*/0,
        /*flashfs_size=*/1024);
    g_flash_stub = flash_emulator;
    ASSERT_EQ(flash_emulator->kFlashSize, 128 * 1024 * 1024);

    // Fill every bytes
    flash_emulator->Fill(flash_emulator->kFlashFSStart, 0x55,
                          flash_emulator->kFlashFSSize);

    flashfsInit();
    EXPECT_EQ(headAddress, 0);
    EXPECT_EQ(tailAddress, flash_emulator->kFlashFSSize - 2048);

    // Setting UINT16_MAX is equal to erasrasing 64MiB
    blackboxConfigMutable()->initialEraseFreeSpaceKiB = UINT16_MAX;
    flashfsLoopInitialErase();
    while (!flashfsIsReady()) {
        flashfsEraseAsync();
    }

    EXPECT_TRUE(flash_emulator->IsErased(0, 64 * 1024 * 1024));
}


class FlashFSLoopRollingEraseTest : public FlashFSLoopInitialEraseTest { };

TEST_F(FlashFSLoopRollingEraseTest, flashfsWriteOverFlashSize)
{
    flashfsInit();
    EXPECT_TRUE(flashfsIsReady());
    blackboxConfigMutable()->rollingErase = true;

    constexpr uint32_t kBufferSize = 128;
    constexpr uint8_t kByte = 0x44;
    auto buffer = std::make_unique<uint8_t[]>(kBufferSize);
    memset(buffer.get(), kByte, kBufferSize);

    EXPECT_EQ(headAddress, 0);
    EXPECT_EQ(tailAddress, 0);

    uint32_t written = 0;
    do {
        flashfsWrite(buffer.get(), kBufferSize);
        flashfsFlushSync();
        written += kBufferSize;
        flashfsEraseAsync();
    } while (written <= flash_emulator_->kFlashFSSize * 2);

    // This test is non-deterministic.
    // After writing the full flashfs space once, it will start rolling erasing,
    // where a lot of writes can be silently dropped (we are writing way faster
    // than the actual BBlog). In the end, we check if there's less than 2
    // erased sector and call it success.
    uint32_t erased = 0;
    uint32_t current = -1;
    for (uint32_t i = 0; i < flash_emulator_->kFlashFSSize; i++) {
        ASSERT_TRUE(flash_emulator_->memory_[i] == kByte ||
                    flash_emulator_->memory_[i] == 0xff)
            << "Mismatch address " << std::hex << i;
        if (current!=flash_emulator_->memory_[i]) {
            current = flash_emulator_->memory_[i];
            std::cout << "offset " << std::hex << i << " = " << current << std::endl;
        }
        if (flash_emulator_->memory_[i] == 0xff) {
            erased++;
        }
    }
    EXPECT_LE(erased, sector_size_ * 2);
}

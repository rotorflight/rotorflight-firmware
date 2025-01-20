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

/**
 * This provides a stream interface to a flash chip if one is present.
 *
 * On statup, call flashfsInit() after initialising the flash chip in order to init the filesystem. This will
 * result in the file pointer being pointed at the first free block found, or at the end of the device if the
 * flash chip is full.
 *
 * Note that bits can only be set to 0 when writing, not back to 1 from 0. You must erase sectors in order
 * to bring bits back to 1 again.
 *
 * In future, we can add support for multiple different flash chips by adding a flash device driver vtable
 * and make calls through that, at the moment flashfs just calls m25p16_* routines explicitly.
 */

/**
 * With USE_FLASHFS_LOOP enabled, the data stream will be wrapped. Flashfs will
 * keep at least 1 page free to identify the stream start and stream end.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"
#include "common/printf.h"
#include "drivers/flash.h"
#include "drivers/light_led.h"

#include "io/flashfs.h"

#include "pg/blackbox.h"

/*
 * How rolling erase works:
 *   1. When start programming (flush) a page, if erase is needed and can be
 *      started, set FLASHFS_ROLLING_ERASE_PENDING.
 *   2. No more flush can be done when FLASHFS_ROLLING_ERASE_PENDING.
 *   3. When page program finishes, EraseAsync task picks up the erase task and
 *      sets FLASHFS_ROLLING_ERASING.
 *   4. When erase finishes, EraseAsync task resets state to FLASHFS_IDLE.
 */
typedef enum {
    FLASHFS_IDLE,
    FLASHFS_ALL_ERASING,
    FLASHFS_INITIAL_ERASING,
    FLASHFS_ROLLING_ERASE_PENDING,
    FLASHFS_ROLLING_ERASING,
} flashfsState_e;

static const flashPartition_t *flashPartition = NULL;
static const flashGeometry_t *flashGeometry = NULL;
STATIC_UNIT_TESTED uint32_t flashfsSize = 0;
static flashfsState_e flashfsState = FLASHFS_IDLE;
static flashSector_t eraseSectorCurrent = 0;
static uint16_t initialEraseSectors = 0;

static DMA_DATA_ZERO_INIT uint8_t flashWriteBuffer[FLASHFS_WRITE_BUFFER_SIZE];

/* The position of our head and tail in the circular flash write buffer.
 *
 * The head is the index that a byte would be inserted into on writing, while the tail is the index of the
 * oldest byte that has yet to be written to flash.
 *
 * When the circular buffer is empty, head == tail
 *
 * The tail is advanced once a write is complete up to the location behind head. The tail is advanced
 * by a callback from the FLASH write routine. This prevents data being overwritten whilst a write is in progress.
 */
static uint16_t bufferHead = 0;
static volatile uint16_t bufferTail = 0;

/* Track if there is new data to write. Until the contents of the buffer have been completely
 * written flashfsFlushAsync() will be repeatedly called. The tail pointer is only updated
 * once an asynchronous write has completed. To do so any earlier could result in data being
 * overwritten in the ring buffer. This routine checks that flashfsFlushAsync() should attempt
 * to write new data and avoids it writing old data during the race condition that occurs if
 * its called again before the previous write to FLASH has completed.
  */
static volatile bool dataWritten = true;

//#define CHECK_FLASH

#ifdef CHECK_FLASH
// Write an incrementing sequence of bytes instead of the requested data and verify
DMA_DATA uint8_t checkFlashBuffer[FLASHFS_WRITE_BUFFER_SIZE];
uint32_t checkFlashPtr = 0;
uint32_t checkFlashLen = 0;
uint8_t checkFlashWrite = 0x00;
uint8_t checkFlashExpected = 0x00;
uint32_t checkFlashErrors = 0;
#endif

// The position of head address. headAddress can only be at sector boundary.
STATIC_UNIT_TESTED uint32_t headAddress = 0;
// The position of the buffer's tail in the overall flash address space:
STATIC_UNIT_TESTED uint32_t tailAddress = 0;

static inline uint32_t flashfsAddressShift(uint32_t address, int32_t offset) {
#ifdef USE_FLASHFS_LOOP
    return (address + offset + flashfsSize) % flashfsSize;
#else
    return address + offset;
#endif
}

static void flashfsClearBuffer(void)
{
    bufferTail = bufferHead = 0;
}

static bool flashfsBufferIsEmpty(void)
{
    return bufferTail == bufferHead;
}

static void flashfsSetTailAddress(uint32_t address)
{
    tailAddress = address;
}

void flashfsEraseCompletely(void)
{
    if (flashGeometry->sectors > 0 && flashPartitionCount() > 0) {
        // if there's a single FLASHFS partition and it uses the entire flash then do a full erase
        const bool doFullErase = (flashPartitionCount() == 1) && (FLASH_PARTITION_SECTOR_COUNT(flashPartition) == flashGeometry->sectors);
        if (doFullErase) {
            flashEraseCompletely();
        } else {
            // start asynchronous erase of all sectors
            eraseSectorCurrent = flashPartition->startSector;
            flashfsState = FLASHFS_ALL_ERASING;
        }
    }

    flashfsClearBuffer();

    headAddress = 0;
    flashfsSetTailAddress(0);
}

/**
 * Start and end must lie on sector boundaries, or they will be rounded out to sector boundaries such that
 * all the bytes in the range [start...end) are erased.
 */
void flashfsEraseRange(uint32_t start, uint32_t end)
{
    if (flashGeometry->sectorSize <= 0)
        return;

    // Round the start down to a sector boundary
    int startSector = start / flashGeometry->sectorSize;

    // And the end upward
    int endSector = end / flashGeometry->sectorSize;
    int endRemainder = end % flashGeometry->sectorSize;

    if (endRemainder > 0) {
        endSector++;
    }

    for (int sectorIndex = startSector; sectorIndex < endSector; sectorIndex++) {
        uint32_t sectorAddress = sectorIndex * flashGeometry->sectorSize;
        flashEraseSector(sectorAddress);
    }
}

/**
 * Return true if the flash is not currently occupied with an operation.
 */
bool flashfsIsReady(void)
{
    // Check for flash chip existence first, then check if idle and ready.

    return (flashfsIsSupported() && (flashfsState == FLASHFS_IDLE) && flashIsReady());
}

bool flashfsIsSupported(void)
{
    return flashfsSize > 0;
}

uint32_t flashfsGetSize(void)
{
    return flashfsSize;
}

static uint32_t flashfsTransmitBufferUsed(void)
{
    if (bufferHead >= bufferTail)
        return bufferHead - bufferTail;

    return FLASHFS_WRITE_BUFFER_SIZE - bufferTail + bufferHead;
}

/**
 * Get the size of the largest single write that flashfs could ever accept without blocking or data loss.
 */
uint32_t flashfsGetWriteBufferSize(void)
{
    return FLASHFS_WRITE_BUFFER_USABLE;
}

/**
 * Get the number of bytes that can currently be written to flashfs without any blocking or data loss.
 */
uint32_t flashfsGetWriteBufferFreeSpace(void)
{
    return flashfsGetWriteBufferSize() - flashfsTransmitBufferUsed();
}

/**
 * Called after bytes have been written from the buffer to advance the position of the tail by the given amount.
 */
static void flashfsAdvanceTailInBuffer(uint32_t delta)
{
    bufferTail += delta;

    // Wrap tail around the end of the buffer
    if (bufferTail >= FLASHFS_WRITE_BUFFER_SIZE) {
        bufferTail -= FLASHFS_WRITE_BUFFER_SIZE;
    }
}

/**
 * Write the given buffers to flash sequentially at the current tail address, advancing the tail address after
 * each write.
 *
 * In synchronous mode, waits for the flash to become ready before writing so that every byte requested can be written.
 *
 * In asynchronous mode, if the flash is busy, then the write is aborted and the routine returns immediately.
 * In this case the returned number of bytes written will be less than the total amount requested.
 *
 * Modifies the supplied buffer pointers and sizes to reflect how many bytes remain in each of them.
 *
 * bufferCount: the number of buffers provided
 * buffers: an array of pointers to the beginning of buffers
 * bufferSizes: an array of the sizes of those buffers
 * sync: true if we should wait for the device to be idle before writes, otherwise if the device is busy the
 *       write will be aborted and this routine will return immediately.
 *
 * Returns the number of bytes written
 */
void flashfsWriteCallback(uint32_t arg)
{
    // Advance the cursor in the file system to match the bytes we wrote
    flashfsSetTailAddress(flashfsAddressShift(tailAddress, arg));

    // Free bytes in the ring buffer
    flashfsAdvanceTailInBuffer(arg);

    // Mark that data has been written from the buffer
    dataWritten = true;
}

static uint32_t flashfsWriteBuffers(uint8_t const **buffers, uint32_t *bufferSizes, int bufferCount, bool sync)
{
    uint32_t bytesWritten;

    // It's OK to overwrite the buffer addresses/lengths being passed in

    // If sync is true, block until the FLASH device is ready, otherwise return
    // 0 if the device isn't ready
    if (sync) {
        while (!flashIsReady());
        /*
         * Wait for any flashfs erase to complete.
         * Note: we shouldn't reach inside since there's no such real world
         * scenario (sync = 1 and flashfsState != FLASHFS_IDLE).
         */
        while (flashfsState != FLASHFS_IDLE) {
            flashfsEraseAsync();
        }
    } else {
        if (!flashIsReady()) {
            return 0;
        }
        /*
         * Also bail out if we are running any of the erases.
         * There are a few cases:
         *   * FLASHFS_INITIAL_ERASING: logging is running ("switch" mode) and
         *     initial erase has triggered or running. We can't write.
         *   * FLASHFS_ROLLING_ERASE_PENDING: rolling erase is pending. We
         *     can't write. (If we write, those bytes will be discarded on
         *     erase).
         *   * FLASHFS_ROLLING_ERASING: technically we can write (under this
         *     state and flashIsReady()), because rolling erase erases only 1
         *     sector. For simplicity, we wait for the task to update the state.
         *   * FLASHFS_ALL_ERASING: we can't write. We may land between erase
         *     sectors.
         */
        if (flashfsState != FLASHFS_IDLE) {
            return 0;
        }
    }

    // Are we at EOF already? Abort.
    if (flashfsIsEOF()) {
#ifdef USE_FLASHFS_LOOP
        // If EOF, request an rolling erase unconditionally
        if (blackboxConfig()->rollingErase) {
            flashfsState = FLASHFS_ROLLING_ERASE_PENDING;
        }
#endif
        return 0;
    }

#ifdef CHECK_FLASH
    checkFlashPtr = tailAddress;
#endif

#ifdef USE_FLASHFS_LOOP
    // Check if rolling erase is needed. Why here? Because we can only
    // erase when a page (aligned) program is completed.
    const uint32_t freeSpace = flashfsSize - flashfsGetOffset();
    if (blackboxConfig()->rollingErase &&
        freeSpace < flashGeometry->sectorSize) {
        // See if we are finishing a page.
        uint32_t bytes_to_write = 0;
        if (bufferCount > 0)
            bytes_to_write += bufferSizes[0];
        if (bufferCount > 1)
            bytes_to_write += bufferSizes[1];
        if (tailAddress / flashGeometry->pageSize !=
            (tailAddress + bytes_to_write) / flashGeometry->pageSize) {
            // We will write to or write over a page boundary. We can erase when
            // this write is done.
            flashfsState = FLASHFS_ROLLING_ERASE_PENDING;
        }
    }

#endif

    flashPageProgramBegin(tailAddress, flashfsWriteCallback);

    /* Mark that data has yet to be written. There is no race condition as the DMA engine is known
     * to be idle at this point
     */
    dataWritten = false;

    bytesWritten = flashPageProgramContinue(buffers, bufferSizes, bufferCount);

#ifdef CHECK_FLASH
    checkFlashLen = bytesWritten;
#endif

    flashPageProgramFinish();

    return bytesWritten;
}

/*
 * Since the buffered data might wrap around the end of the circular buffer, we can have two segments of data to write,
 * an initial portion and a possible wrapped portion.
 *
 * This routine will fill the details of those buffers into the provided arrays, which must be at least 2 elements long.
 */
static int flashfsGetDirtyDataBuffers(uint8_t const *buffers[], uint32_t bufferSizes[])
{
    buffers[0] = flashWriteBuffer + bufferTail;
    buffers[1] = flashWriteBuffer + 0;

    if (bufferHead > bufferTail) {
        bufferSizes[0] = bufferHead - bufferTail;
        bufferSizes[1] = 0;
        return 1;
    } else if (bufferHead < bufferTail) {
        bufferSizes[0] = FLASHFS_WRITE_BUFFER_SIZE - bufferTail;
        bufferSizes[1] = bufferHead;
        if (bufferSizes[1] == 0) {
            return 1;
        } else {
            return 2;
        }
    }

    bufferSizes[0] = 0;
    bufferSizes[1] = 0;

    return 0;
}


static bool flashfsNewData()
{
    return dataWritten;
}


/**
 * Get the current usage of the volume, including the buffered ones.
 */
uint32_t flashfsGetOffset(void)
{
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];

    // Dirty data in the buffers contributes to the offset

    flashfsGetDirtyDataBuffers(buffers, bufferSizes);

    return flashfsAddressShift(tailAddress + bufferSizes[0] + bufferSizes[1],
                               -headAddress);
}

/**
 * If the flash is ready to accept writes, flush the buffer to it.
 *
 * Returns true if all data in the buffer has been flushed to the device, or false if
 * there is still data to be written (call flush again later).
 */
bool flashfsFlushAsync(void)
{
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];
    int bufCount;

    if (flashfsBufferIsEmpty()) {
        return true; // Nothing to flush
    }

    if (!flashfsNewData() || !flashIsReady()) {
        // The previous write has yet to complete
        return false;
    }

#ifdef CHECK_FLASH
    // Verify the data written last time
    if (checkFlashLen) {
        while (!flashIsReady());
        flashReadBytes(checkFlashPtr, checkFlashBuffer, checkFlashLen);

        for (uint32_t i = 0; i < checkFlashLen; i++) {
            if (checkFlashBuffer[i] != checkFlashExpected++) {
                checkFlashErrors++; // <-- insert breakpoint here to catch errors
            }
        }
    }
#endif

    bufCount = flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    if (bufCount) {
        flashfsWriteBuffers(buffers, bufferSizes, bufCount, false);
    }

    return flashfsBufferIsEmpty();
}

/**
 * Wait for the flash to become ready and begin flushing any buffered data to flash.
 *
 * The flash will still be busy some time after this sync completes, but space will
 * be freed up to accept more writes in the write buffer.
 */
void flashfsFlushSync(void)
{
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];
    int bufCount;

    if (flashfsBufferIsEmpty()) {
        return; // Nothing to flush
    }

    bufCount = flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    if (bufCount) {
        flashfsWriteBuffers(buffers, bufferSizes, bufCount, true);
    }

    while (!flashIsReady());
}

/**
 *  Asynchronously erase the flash: Check if ready and then erase sector.
 */
void flashfsEraseAsync(void)
{
    if ((flashfsIsSupported() && flashIsReady())) {
        if (flashfsState == FLASHFS_ALL_ERASING) {
            if (eraseSectorCurrent <= flashPartition->endSector) {
                // Erase sector
                uint32_t sectorAddress =
                    eraseSectorCurrent * flashGeometry->sectorSize;
                flashEraseSector(sectorAddress);
                eraseSectorCurrent++;
                LED1_TOGGLE;
            } else {
                // Done erasing
                flashfsState = FLASHFS_IDLE;
                LED1_OFF;
            }
        } else if (flashfsState == FLASHFS_INITIAL_ERASING) {
            if (initialEraseSectors > 0) {
                flashEraseSector(headAddress);
                initialEraseSectors--;
                // We immediately set the head before erase is completed.
                // This should be fine since write will be blocked until this
                // state is cleared.
                headAddress =
                    flashfsAddressShift(headAddress, flashGeometry->sectorSize);
                LED1_TOGGLE;
            } else {
                flashfsState = FLASHFS_IDLE;
                LED1_OFF;
            }
        } else if (flashfsState == FLASHFS_ROLLING_ERASE_PENDING) {
            flashEraseSector(headAddress);
            headAddress =
                flashfsAddressShift(headAddress, flashGeometry->sectorSize);
            flashfsState = FLASHFS_ROLLING_ERASING;
            LED1_TOGGLE;
        } else if (flashfsState == FLASHFS_ROLLING_ERASING) {
            flashfsState = FLASHFS_IDLE;
            LED1_OFF;
        }
    }
}

void flashfsSeekAbs(uint32_t offset)
{
    flashfsFlushSync();

    flashfsSetTailAddress(flashfsAddressShift(headAddress, offset));
}

void flashfsSeekPhysical(uint32_t offset)
{
    flashfsFlushSync();

    flashfsSetTailAddress(offset);
}

/**
 * Write the given byte asynchronously to the flash. If the buffer overflows, data is silently discarded.
 */
void flashfsWriteByte(uint8_t byte)
{
#ifdef CHECK_FLASH
    byte = checkFlashWrite++;
#endif

    flashWriteBuffer[bufferHead++] = byte;

    if (bufferHead >= FLASHFS_WRITE_BUFFER_SIZE) {
        bufferHead = 0;
    }
}

/**
 * Write the given buffer to the flash either synchronously or asynchronously depending on the 'sync' parameter.
 *
 * If writing asynchronously, data will be silently discarded if the buffer overflows.
 * If writing synchronously, the routine will block waiting for the flash to become ready so will never drop data.
 */
void flashfsWrite(const uint8_t *data, unsigned int len)
{
    // Buffer up the data the user supplied instead of writing it right away
    for (unsigned int i = 0; i < len; i++) {
        flashfsWriteByte(data[i]);
    }
}

/**
 * Read `len` bytes from the given physical address into the supplied buffer.
 *
 * Returns the number of bytes actually read which may be less than that requested.
 */
int flashfsReadPhysical(uint32_t address, uint8_t *buffer, unsigned int len)
{
    int bytesRead;

    // Did caller try to read past the end of the volume?
    if (address + len > flashfsSize) {
        // Truncate their request
        len = flashfsSize - address;
    }

    // Since the read could overlap data in our dirty buffers, force a sync to clear those first
    flashfsFlushSync();

    bytesRead = flashReadBytes(address, buffer, len);

    return bytesRead;
}

/**
 * Read `len` bytes from the given address into the supplied buffer.
 *
 * Returns the number of bytes actually read which may be less than that requested.
 */
int flashfsReadAbs(uint32_t address, uint8_t *buffer, unsigned int len)
{
    uint32_t physicalAddress = flashfsAddressShift(headAddress, address);
    uint16_t len1 = len, len2 = 0;
    int bytesRead;

    // Wrapped read?
    if (physicalAddress + len1 > flashfsSize) {
        len1 = flashfsSize - physicalAddress;
        len2 = len - len1;
    }

    flashfsFlushSync();

    bytesRead = flashReadBytes(physicalAddress, buffer, len1);
    if (len2) {
        // Second section
        bytesRead += flashReadBytes(0, buffer + len1, len2);
    }

    return bytesRead;
}

// This function takes a physical address
static bool flashfsIsPageErased(uint32_t address){
    enum {
        /* We don't expect valid data to ever contain this many consecutive uint32_t's of all 1 bits: */
        EMPTY_PAGE_TEST_SIZE_INTS = 4, // i.e. 16 bytes
        EMPTY_PAGE_TEST_SIZE_BYTES = EMPTY_PAGE_TEST_SIZE_INTS * sizeof(uint32_t)
    };

    STATIC_DMA_DATA_AUTO union {
        uint8_t bytes[EMPTY_PAGE_TEST_SIZE_BYTES];
        uint32_t ints[EMPTY_PAGE_TEST_SIZE_INTS];
    } testBuffer;

    if (flashReadBytes(address, testBuffer.bytes, EMPTY_PAGE_TEST_SIZE_BYTES) < EMPTY_PAGE_TEST_SIZE_BYTES) {
        // Unexpected timeout from flash, so bail early (reporting the device fuller than it really is)
        return false;
    }

    // Checking the buffer 4 bytes at a time like this is probably faster than byte-by-byte, but I didn't benchmark it :)
    for (uint8_t i = 0; i < EMPTY_PAGE_TEST_SIZE_INTS; i++) {
        if (testBuffer.ints[i] != 0xFFFFFFFF) {
            return false;
        }
    }
    return true;
}

/**
 * Find the absolute address of the start of the free space on the device.
 * `headAddress` must be setup prior to this function.
 */
int flashfsIdentifyStartOfFreeSpace(void)
{
    /* Find the start of the free space on the device by examining the beginning of blocks with a binary search,
     * looking for ones that appear to be erased. We can achieve this with good accuracy because an erased block
     * is all bits set to 1, which pretty much never appears in reasonable size substrings of blackbox logs.
     *
     * To do better we might write a volume header instead, which would mark how much free space remains. But keeping
     * a header up to date while logging would incur more writes to the flash, which would consume precious write
     * bandwidth and block more often.
     */


    const uint16_t pageSize = flashGeometry->pageSize;

    int left = 0; // Smallest page index in the search region
    int right = flashfsSize / pageSize; // One past the largest page index in the search region
#ifdef USE_FLASHFS_LOOP
    right--;
    // We must leave one empty page to:
    //  1. identify empty space
    //  2. differenciate between full and empty
#endif
    int mid;
    int result = right;

    while (left < right) {
        mid = (left + right) / 2;

        uint32_t address = flashfsAddressShift(headAddress, mid * pageSize);
        if (flashfsIsPageErased(address)) {
            /* This empty page might be the leftmost empty page in the volume, but we'll need to continue the
             * search leftwards to find out:
             */
            result = mid;

            right = mid;
        } else {
            left = mid + 1;
        }
    }

    return flashfsAddressShift(headAddress, result * pageSize);
}

/**
 * Returns true if the file pointer is at the end of the device.
 */
bool flashfsIsEOF(void)
{
#ifdef USE_FLASHFS_LOOP
    // In case of using LOOP_FLASHFS, we need
    //  * 1 free page before a sector to identify boundary.
    //  * +flashfs buffer size to avoid writing to that 1 free page.
    uint32_t persistedBytes = flashfsAddressShift(tailAddress, -headAddress);

    return persistedBytes >=
           flashfsSize - flashGeometry->pageSize - FLASHFS_WRITE_BUFFER_SIZE;

#else
    return tailAddress >= flashfsSize;
#endif
}

void flashfsClose(void)
{
    switch(flashGeometry->flashType) {
        case FLASH_TYPE_NAND:
            flashFlush();
            /* FALL THROUGH */
        case FLASH_TYPE_NOR: {
            flashfsClearBuffer();

            const uint16_t pageSize = flashGeometry->pageSize;
            const uint32_t padding = (tailAddress % pageSize == 0
                                          ? 0
                                          : pageSize - tailAddress % pageSize);

            flashfsSetTailAddress(flashfsAddressShift(tailAddress, padding));
            break;
        }
    }
}

/*
 * Locate the start physical address of the used space. Return 0 if all space
 * are unused.
 */
uint32_t flashfsIdentifyStartOfUsedSpace() {
    // Locate the boundary between erased and filled.
    // This can only be at the sector boundary.

    // We skip the startSector because the calculation is a bit different and we
    // will use that as fallback.
    for (uint16_t sector = flashPartition->startSector + 1;
         sector <= flashPartition->endSector; sector++) {
        uint32_t startAddress = sector * flashGeometry->sectorSize;
        uint32_t endAddress = startAddress - flashGeometry->pageSize;
        if (flashfsIsPageErased(endAddress) && !flashfsIsPageErased(startAddress)) {
            return sector * flashGeometry->sectorSize;
        }
    }

    // fallback
    return flashPartition->startSector * flashGeometry->sectorSize;
}

/**
 * Call after initializing the flash chip in order to set up the filesystem.
 */
void flashfsInit(void)
{
    flashfsSize = 0;

    flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);
    flashGeometry = flashGetGeometry();

    if (!flashPartition) {
        return;
    }

    flashfsSize = FLASH_PARTITION_SECTOR_COUNT(flashPartition) * flashGeometry->sectorSize;

#ifdef USE_FLASHFS_LOOP
    headAddress = flashfsIdentifyStartOfUsedSpace();
#endif

    // Start the file pointer off at the beginning of free space so caller can start writing immediately
    flashfsSeekPhysical(flashfsIdentifyStartOfFreeSpace());

    flashfsState = FLASHFS_IDLE;
}

#ifdef USE_FLASH_TOOLS

void flashfsFillEntireFlash(void)
{
    flashfsEraseCompletely();
    while(flashfsState != FLASHFS_IDLE) {
        flashfsEraseAsync();
    }
    flashfsInit();

    uint32_t address = 0;
    flashfsSeekPhysical(address);

    const int bufferSize = 32;
    char buffer[bufferSize + 1];

    const uint32_t testLimit = flashfsGetSize();

    for (address = 0; address < testLimit; address += bufferSize) {
        tfp_sprintf(buffer, "%08x >> **0123456789ABCDEF**", address);
        flashfsWrite((uint8_t*)buffer, strlen(buffer));
        flashfsFlushSync();
    }
    flashfsClose();
}

bool flashfsVerifyEntireFlash(void)
{
    flashfsInit();
    uint32_t address = 0;

    const int bufferSize = 32;
    char buffer[bufferSize + 1];

    uint32_t testLimit = flashfsGetSize();
#ifdef USE_FLASHFS_LOOP
    testLimit -= flashGeometry->pageSize + FLASHFS_WRITE_BUFFER_SIZE;
#endif
    char expectedBuffer[bufferSize + 1];

    flashfsSeekPhysical(0);

    int verificationFailures = 0;
    for (address = 0; address < testLimit; address += bufferSize) {
        tfp_sprintf(expectedBuffer, "%08x >> **0123456789ABCDEF**", address);

        memset(buffer, 0, sizeof(buffer));
        int bytesRead = flashfsReadPhysical(address, (uint8_t *)buffer, bufferSize);

        int result = strncmp(buffer, expectedBuffer, bufferSize);
        if (result != 0 || bytesRead != bufferSize) {
            verificationFailures++;
        }
    }
    return verificationFailures == 0;
}
#endif // USE_FLASH_TOOLS

#ifdef USE_FLASHFS_LOOP
void flashfsLoopInitialErase()
{
    if (flashfsState != FLASHFS_IDLE) {
        return;
    }

    const int32_t bytesNeeded =
        flashfsGetOffset() + blackboxConfig()->initialEraseFreeSpaceKiB * 1024 -
        flashfsSize;

    if (bytesNeeded <= 0) {
        return;
    }
    const uint32_t sectorSize = flashGeometry->sectorSize;
    initialEraseSectors = (bytesNeeded + sectorSize - 1) / sectorSize;

    flashfsState = FLASHFS_INITIAL_ERASING;
}
#endif

uint32_t flashfsGetHeadAddress()
{
    return headAddress;
}

uint32_t flashfsGetTailAddress()
{
    return tailAddress;
}

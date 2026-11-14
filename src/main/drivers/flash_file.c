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

/*
 * A NOR flash chip backed by a file, for SITL: the onboard dataflash that
 * blackbox logs to on a real FC, so flashfs, the blackbox FLASH device, the
 * CLI's flash_* commands and the Configurator's dataflash download/erase all
 * run unchanged against it.
 *
 * The file holds the chip from address 0 up to the highest byte ever
 * written; everything past its end reads as erased (0xFF), so an empty chip
 * is an empty file and the file only grows as logs are written. A full erase
 * truncates it. Programming ANDs into what is there, as NOR flash does, so
 * a double write shows up as corrupt data rather than silently succeeding.
 *
 * Every program is flushed to disk before it completes, so a log survives
 * SITL being killed mid-flight - which is how the launchers stop it.
 *
 * Only ever called from the main loop (flashfs, blackbox, MSP, CLI), so it
 * needs no locking.
 */

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FLASH_FILE

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/flash_file.h"

#ifndef FLASH_FILE_NAME
#define FLASH_FILE_NAME "blackbox_flash.bin"
#endif

// A 128 MiB NOR part with 64 KiB sectors and 256-byte pages - the size of a
// W25N01G, with the simpler NOR programming model of an M25P16.
#define FLASH_FILE_PAGE_SIZE        256
#define FLASH_FILE_SECTOR_SIZE      (64 * 1024)
#ifndef FLASH_FILE_SECTORS
#define FLASH_FILE_SECTORS          2048    // override for a small chip to test filling it up
#endif

#define FLASH_FILE_CHUNK            4096

static FILE *flashFile;
static uint32_t flashFileLength;    // bytes present in the file; beyond reads as 0xFF
static uint8_t erased[FLASH_FILE_CHUNK];    // all 0xFF, set up in flashFileDetect()

// Set by a failed erase, write or flush, cleared by the next successful erase.
// While set, programming reports nothing written, so flashfs keeps the data in
// its buffer and a sector whose erase failed is not programmed. (Reporting the
// device as not ready instead would hang flashfs, which busy-waits on it.)
static bool flashFileFailed;

static void flashFileSetFailed(const char *what, uint32_t address)
{
    if (!flashFileFailed) {
        fprintf(stderr, "[flash_file] %s '%s' at %u failed: %s; writes refused until the next erase\n",
                what, FLASH_FILE_NAME, (unsigned)address, strerror(errno));
    }
    flashFileFailed = true;
}

// False also once a failed erase has lost the file: every access then fails
// (reads come back erased) instead of crashing.
static bool flashFileSeek(uint32_t address)
{
    return flashFile && fseek(flashFile, (long)address, SEEK_SET) == 0;
}

// Grow the file to `length` bytes, filling the gap with erased bytes.
static bool flashFileExtend(uint32_t length)
{
    if (length <= flashFileLength) {
        return true;
    }
    if (!flashFileSeek(flashFileLength)) {
        return false;
    }
    while (flashFileLength < length) {
        const uint32_t chunk = MIN(length - flashFileLength, (uint32_t)FLASH_FILE_CHUNK);
        if (fwrite(erased, 1, chunk, flashFile) != chunk) {
            return false;
        }
        flashFileLength += chunk;
    }
    return true;
}

static bool flashFileIsReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    return true;
}

static bool flashFileWaitForReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    return true;
}

static void flashFileEraseRange(uint32_t start, uint32_t end)
{
    end = MIN(end, flashFileLength);
    if (start >= end) {
        // Beyond the end of the file the chip reads erased already
        flashFileFailed = (flashFile == NULL);
        return;
    }
    if (!flashFileSeek(start)) {
        flashFileSetFailed("erasing", start);
        return;
    }
    for (uint32_t address = start; address < end; ) {
        const uint32_t chunk = MIN(end - address, (uint32_t)FLASH_FILE_CHUNK);
        if (fwrite(erased, 1, chunk, flashFile) != chunk) {
            flashFileSetFailed("erasing", address);
            return;
        }
        address += chunk;
    }
    if (fflush(flashFile) != 0) {
        flashFileSetFailed("erasing", start);
        return;
    }
    flashFileFailed = false;
}

static void flashFileEraseSector(flashDevice_t *fdevice, uint32_t address)
{
    const uint32_t start = address - (address % fdevice->geometry.sectorSize);
    flashFileEraseRange(start, start + fdevice->geometry.sectorSize);
}

static void flashFileEraseCompletely(flashDevice_t *fdevice)
{
    UNUSED(fdevice);

    // Reopening for writing truncates: an empty file is an erased chip. Once
    // a failed erase has lost the stream there is none to reopen: open anew.
    FILE *reopened = flashFile ? freopen(FLASH_FILE_NAME, "wb+", flashFile) : fopen(FLASH_FILE_NAME, "wb+");
    if (!reopened) {
        flashFile = NULL;
        flashFileSetFailed("erasing", 0);
        return;
    }
    flashFile = reopened;
    flashFileLength = 0;
    flashFileFailed = false;
    printf("[flash_file] erased '%s'\n", FLASH_FILE_NAME);
}

// NOR programming can only clear bits: AND the new data into the old.
// Returns the bytes programmed, fewer than `length` when the file fails.
static uint32_t flashFileProgram(uint32_t address, const uint8_t *data, uint32_t length)
{
    uint8_t merged[FLASH_FILE_PAGE_SIZE];
    uint32_t done = 0;

    if (flashFileFailed) {
        return 0;
    }
    while (done < length) {
        const uint32_t chunk = MIN(length - done, (uint32_t)sizeof(merged));

        if (!flashFileExtend(address + chunk) ||
            !flashFileSeek(address) || fread(merged, 1, chunk, flashFile) != chunk) {
            flashFileSetFailed("reading back", address);
            break;
        }
        for (uint32_t i = 0; i < chunk; i++) {
            merged[i] &= data[done + i];
        }
        // A read followed by a write on the same stream needs a seek between them.
        if (!flashFileSeek(address) || fwrite(merged, 1, chunk, flashFile) != chunk) {
            flashFileSetFailed("writing", address);
            break;
        }

        address += chunk;
        done += chunk;
    }
    return done;
}

static void flashFilePageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}

// Completes synchronously, so the callback runs before this returns - as a
// real chip's DMA-completion callback may on fast hardware. flashfs is ready
// for that: it clears its "written" flag before calling this.
static uint32_t flashFilePageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    uint32_t written = 0;

    // Report only what reached the file: flashfs keeps the rest in its buffer
    // and offers it again. Programming the same bytes twice is harmless (AND).
    for (uint32_t i = 0; i < bufferCount; i++) {
        const uint32_t done = flashFileProgram(fdevice->currentWriteAddress + written, buffers[i], bufferSizes[i]);
        written += done;
        if (done < bufferSizes[i]) {
            break;
        }
    }
    if (written && fflush(flashFile) != 0) {
        flashFileSetFailed("flushing", fdevice->currentWriteAddress);
        written = 0;
    }

    fdevice->callbackArg = written;
    fdevice->currentWriteAddress += written;
    if (fdevice->callback) {
        fdevice->callback(written);
    }

    return written;
}

static void flashFilePageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

static void flashFilePageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    flashFilePageProgramBegin(fdevice, address, callback);
    flashFilePageProgramContinue(fdevice, &data, &length, 1);
    flashFilePageProgramFinish(fdevice);
}

static int flashFileReadBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    UNUSED(fdevice);

    uint32_t fromFile = 0;
    if (address < flashFileLength) {
        fromFile = MIN(length, flashFileLength - address);
        if (!flashFileSeek(address) || fread(buffer, 1, fromFile, flashFile) != fromFile) {
            return 0;
        }
    }
    memset(buffer + fromFile, 0xFF, length - fromFile);

    return length;
}

static const flashGeometry_t *flashFileGetGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

static const flashVTable_t flashFileVTable = {
    .isReady = flashFileIsReady,
    .waitForReady = flashFileWaitForReady,
    .eraseSector = flashFileEraseSector,
    .eraseCompletely = flashFileEraseCompletely,
    .pageProgramBegin = flashFilePageProgramBegin,
    .pageProgramContinue = flashFilePageProgramContinue,
    .pageProgramFinish = flashFilePageProgramFinish,
    .pageProgram = flashFilePageProgram,
    .flush = NULL,
    .readBytes = flashFileReadBytes,
    .getGeometry = flashFileGetGeometry,
    .suspend = NULL,
    .resume = NULL,
    .isSuspended = NULL,
};

bool flashFileDetect(flashDevice_t *fdevice)
{
    // Binary mode: text mode on Windows would translate the log's bytes.
    flashFile = fopen(FLASH_FILE_NAME, "rb+");
    if (!flashFile) {
        flashFile = fopen(FLASH_FILE_NAME, "wb+");
    }
    if (!flashFile) {
        fprintf(stderr, "[flash_file] cannot open '%s' - no onboard flash\n", FLASH_FILE_NAME);
        return false;
    }

    memset(erased, 0xFF, sizeof(erased));

    fseek(flashFile, 0, SEEK_END);
    const long size = ftell(flashFile);
    flashFileLength = size > 0 ? (uint32_t)size : 0;

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.pageSize = FLASH_FILE_PAGE_SIZE;
    fdevice->geometry.sectorSize = FLASH_FILE_SECTOR_SIZE;
    fdevice->geometry.sectors = FLASH_FILE_SECTORS;
    fdevice->geometry.pagesPerSector = FLASH_FILE_SECTOR_SIZE / FLASH_FILE_PAGE_SIZE;
    fdevice->geometry.totalSize = FLASH_FILE_SECTOR_SIZE * FLASH_FILE_SECTORS;
    fdevice->isLargeFlash = true;
    fdevice->couldBeBusy = false;
    fdevice->vTable = &flashFileVTable;

    if (flashFileLength > fdevice->geometry.totalSize) {
        flashFileLength = fdevice->geometry.totalSize;
    }

    printf("[flash_file] '%s': %u of %u bytes on file\n", FLASH_FILE_NAME,
        (unsigned)flashFileLength, (unsigned)fdevice->geometry.totalSize);
    return true;
}

#endif // USE_FLASH_FILE

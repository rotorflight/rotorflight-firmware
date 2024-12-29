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

/*
 * Author: jflyper@github.com
 */
#include <ctype.h>

#include "platform.h"
#include "emfat.h"
#include "emfat_file.h"

#include "common/printf.h"
#include "common/strtol.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/flash.h"
#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"

#include "io/flashfs.h"

#include "pg/flash.h"
#include "pg/pilot.h"

#include "msc/usbd_storage.h"

#define FILESYSTEM_SIZE_MB 256
#define HDR_BUF_SIZE 32

#ifdef USE_EMFAT_AUTORUN
static const char autorun_file[] =
    "[autorun]\r\n"
    "icon=icon.ico\r\n"
    "label=Rotorflight Blackbox\r\n" ;
#define AUTORUN_SIZE (sizeof(autorun_file) - 1)
#define EMFAT_INCR_AUTORUN 1
#else
#define EMFAT_INCR_AUTORUN 0
#endif

#ifdef USE_EMFAT_README
static const char readme_file[] =
    "This is readme file\r\n";
#define README_SIZE  (sizeof(readme_file) - 1)
#define EMFAT_INCR_README 1
#else
#define EMFAT_INCR_README 0
#endif

#ifdef USE_EMFAT_ICON
static const char icon_file[] =
{
    #include "icon.hex"
};
#define ICON_SIZE    (sizeof(icon_file))
#define EMFAT_INCR_ICON 1
#else
#define EMFAT_INCR_ICON 0
#endif

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(1,1,2018, 13,0,0)
#define CMA { CMA_TIME, CMA_TIME, CMA_TIME }

#if defined (USE_EMFAT_AUTORUN) || defined (USE_EMFAT_ICON) || defined (USE_EMFAT_README)
static void memory_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry)
{
    int len;

    if (offset > entry->curr_size) {
        return;
    }

    if (offset + size > entry->curr_size) {
        len = entry->curr_size - offset;
    } else {
        len = size;
    }

    memcpy(dest, &((char *)entry->user_data)[offset], len);
}
#endif

static void bblog_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry)
{
    UNUSED(entry);

    flashfsReadAbs(offset, dest, size);
}

static const emfat_entry_t entriesPredefined[] =
{
    // name           dir    attr         lvl offset  size             max_size        user                time  read               write
    { "",             true,  0,           0,  0,      0,               0,              0,                  CMA,  NULL,              NULL, { 0 } },
#ifdef USE_EMFAT_AUTORUN
    { "autorun.inf",  false, ATTR_HIDDEN, 1,  0,      AUTORUN_SIZE,    AUTORUN_SIZE,   (long)autorun_file, CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_ICON
    { "icon.ico",     false, ATTR_HIDDEN, 1,  0,      ICON_SIZE,       ICON_SIZE,      (long)icon_file,    CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_README
    { "readme.txt",   false, 0,           1,  0,      README_SIZE,     1024*1024,      (long)readme_file,  CMA,  memory_read_proc,  NULL, { 0 } },
#endif
    { FC_FIRMWARE_IDENTIFIER "_ALL.BBL", 0,     0,           1,  0,      0,               0,              0,                  CMA,  bblog_read_proc,   NULL, { 0 } },
    { "PADDING.TXT",  0,     ATTR_HIDDEN, 1,  0,      0,               0,              0,                  CMA,  NULL,              NULL, { 0 } },
};

#define PREDEFINED_ENTRY_COUNT (1 + EMFAT_INCR_AUTORUN + EMFAT_INCR_ICON + EMFAT_INCR_README)
#define APPENDED_ENTRY_COUNT 2

#define EMFAT_MAX_LOG_ENTRY 100
#define EMFAT_MAX_ENTRY (PREDEFINED_ENTRY_COUNT + EMFAT_MAX_LOG_ENTRY + APPENDED_ENTRY_COUNT)

static emfat_entry_t entries[1 + EMFAT_MAX_ENTRY];

emfat_t emfat;
static uint32_t cmaTime = CMA_TIME;

// The craft name to compose log filenames. It includes a separator "_".
static char craft_name[MAX_NAME_LENGTH + 2];

static void emfat_set_entry_cma(emfat_entry_t *entry)
{
    // Set file creation/modification/access times to be the same, either the default date or that from the RTC
    // In practise this will be when the filesystem is mounted as the date is passed from the host over USB
    entry->cma_time[0] = cmaTime;
    entry->cma_time[1] = cmaTime;
    entry->cma_time[2] = cmaTime;
}

#ifdef USE_FLASHFS
static inline int emfat_decode_bits(uint32_t v, uint8_t starting_bit, uint8_t length)
{
    return (v >> starting_bit) & ((1 << length) - 1);
}
static inline int emfat_decode_year(uint32_t cma)
{
    return emfat_decode_bits(cma, 9 + 16, 7) + 1980;
}
static inline int emfat_decode_month(uint32_t cma)
{
    return emfat_decode_bits(cma, 5 + 16, 4);
}
static inline int emfat_decode_day(uint32_t cma)
{
    return emfat_decode_bits(cma, 0 + 16, 5);
}
static inline int emfat_decode_hour(uint32_t cma)
{
    return emfat_decode_bits(cma, 11, 5);
}
static inline int emfat_decode_minute(uint32_t cma)
{
    return emfat_decode_bits(cma, 5, 6);
}
static inline int emfat_decode_second(uint32_t cma)
{
    return emfat_decode_bits(cma, 0, 5) << 1;
}

/*
 * Change illegal filename charactors to '_'.
 */
void legalize_filename(char *name)
{
    for (int i = 0; name[i] != '\0'; i++) {
        if (isalpha(name[i])) {
            continue;
        }
        if (isdigit(name[i])) {
            continue;
        }
        switch (name[i]) {
        case ' ':
        case '$':
        case '%':
        case '-':
        case '_':
        case '@':
        case '~':
        case '`':
        case '!':
        case '(':
        case ')':
        case '{':
        case '}':
        case '^':
        case '#':
        case '&':
        case '.': // Spec says only 1 dot is allowed. We don't care this rule.
            continue;
        }

        name[i] = '_';
    }
}

static void emfat_add_log(emfat_entry_t *entry, int number, uint32_t offset,
                          uint32_t size)
{
    static char logNames[1 + EMFAT_MAX_LOG_ENTRY]
                        [4 + MAX_NAME_LENGTH + 1 + 1 + 9 + 6 + 4 + 1];

    if (entry->cma_time[0] == cmaTime) {
        // Unrecognized timestamp
        tfp_sprintf(logNames[number], FC_FIRMWARE_IDENTIFIER "%s_%03d.bbl",
                    craft_name,
                    number + 1);
    } else {
        // Recognized timestamp, create a meaningful filename.
        tfp_sprintf(logNames[number],
                    FC_FIRMWARE_IDENTIFIER "%s_%04d%02d%02d_%02d%02d%02d.bbl",
                    craft_name,
                    emfat_decode_year(entry->cma_time[0]),
                    emfat_decode_month(entry->cma_time[0]),
                    emfat_decode_day(entry->cma_time[0]),
                    emfat_decode_hour(entry->cma_time[0]),
                    emfat_decode_minute(entry->cma_time[0]),
                    emfat_decode_second(entry->cma_time[0]));
    }
    entry->name = logNames[number];
    entry->level = 1;
    entry->offset = offset;
    entry->curr_size = size;
    entry->max_size = entry->curr_size;
    entry->readcb = bblog_read_proc;
    // Set file modification/access times to be the same as the creation time
    entry->cma_time[1] = entry->cma_time[0];
    entry->cma_time[2] = entry->cma_time[0];
}

static int emfat_find_log(emfat_entry_t *entry, int maxCount, int flashfsUsedSpace)
{
    static uint8_t buffer[HDR_BUF_SIZE];
    int lastOffset = 0;
    int currOffset = 0;
    int buffOffset;
    int hdrOffset;
    int fileNumber = 0;
    int logCount = 0;
    char *logHeader = "H Product:Blackbox";
    int lenLogHeader = strlen(logHeader);
    char *timeHeader = "H Log start datetime:";
    int lenTimeHeader = strlen(timeHeader);
    int timeHeaderMatched = 0;

    for ( ; currOffset < flashfsUsedSpace ; currOffset += 2048) { // XXX 2048 = FREE_BLOCK_SIZE in io/flashfs.c

        mscSetActive();
        mscActivityLed();

        flashfsReadAbs(currOffset, buffer, HDR_BUF_SIZE);

        if (strncmp((char *)buffer, logHeader, lenLogHeader)) {
            continue;
        }

        // The length of the previous record is now known
        if (lastOffset != currOffset) {
            // Record the previous entry
            emfat_add_log(entry++, fileNumber++, lastOffset, currOffset - lastOffset);

            logCount++;
        }

        // Find the "Log start datetime" entry, example encoding "H Log start datetime:2019-08-15T13:18:22.199+00:00"

        buffOffset = lenLogHeader;
        hdrOffset = currOffset;

        // Set the default timestamp for this log entry in case the timestamp is not found
        entry->cma_time[0] = cmaTime;

        // Search for the timestamp record
        while (true) {
            if (buffer[buffOffset++] == timeHeader[timeHeaderMatched]) {
                // This matches the header we're looking for so far
                if (++timeHeaderMatched == lenTimeHeader) {
                    // Complete match so read date/time into buffer
                    flashfsReadAbs(hdrOffset + buffOffset, buffer, HDR_BUF_SIZE);

                    // Extract the time values to create the CMA time
                    char *nextToken = (char *)buffer;
                    int year = strtoul(nextToken, &nextToken, 10);
                    int month = strtoul(++nextToken, &nextToken, 10);
                    int day = strtoul(++nextToken, &nextToken, 10);
                    int hour = strtoul(++nextToken, &nextToken, 10);
                    int min = strtoul(++nextToken, &nextToken, 10);
                    int sec = strtoul(++nextToken, NULL, 10);

                    // Set the file creation time
                    if (year) {
                        entry->cma_time[0] = EMFAT_ENCODE_CMA_TIME(day, month, year, hour, min, sec);
                    }

                    break;
                }
            } else {
                timeHeaderMatched = 0;
            }

            if (buffOffset == HDR_BUF_SIZE) {
                // Read the next portion of the header
                hdrOffset += HDR_BUF_SIZE;

                // Check for flash overflow
                if (hdrOffset > flashfsUsedSpace) {
                    break;
                }

                flashfsReadAbs(hdrOffset, buffer, HDR_BUF_SIZE);
                buffOffset = 0;
            }
        }

        if (fileNumber == maxCount) {
            break;
        }

        lastOffset = currOffset;
    }

    // Now add the final entry
    if (fileNumber != maxCount && lastOffset != currOffset) {
        emfat_add_log(entry, fileNumber, lastOffset, currOffset - lastOffset);
        ++logCount;
    }

    return logCount;
}
#endif  // USE_FLASHFS

void emfat_init_files(void)
{
    int flashfsUsedSpace = 0;
    int entryIndex = PREDEFINED_ENTRY_COUNT;
    emfat_entry_t *entry;
    memset(entries, 0, sizeof(entries));

#ifdef USE_PERSISTENT_MSC_RTC
    rtcTime_t mscRebootRtc;
    if (rtcPersistRead(&mscRebootRtc)) {
        const int32_t rtcSeconds = rtcTimeGetSeconds(&mscRebootRtc);
        cmaTime = emfat_cma_time_from_unix((uint32_t)rtcSeconds);
    }
#endif

    // create the predefined entries
    for (size_t i = 0 ; i < PREDEFINED_ENTRY_COUNT ; i++) {
        entries[i] = entriesPredefined[i];
        // These entries have timestamps corresponding to when the filesystem is mounted
        emfat_set_entry_cma(&entries[i]);
    }

#ifdef USE_FLASHFS
    if (pilotConfig()->name[0]) {
        tfp_sprintf(craft_name, "_%s", pilotConfig()->name);
        legalize_filename(craft_name);
    } else {
        craft_name[0] = 0;
    }

    flashInit(flashConfig());
    flashfsInit();
    LED0_OFF;

    flashfsUsedSpace = flashfsIdentifyStartOfFreeSpace();

    // Detect and create entries for each individual log
    const int logCount = emfat_find_log(&entries[PREDEFINED_ENTRY_COUNT], EMFAT_MAX_LOG_ENTRY, flashfsUsedSpace);

    entryIndex += logCount;

    if (logCount > 0) {
        // Create the all logs entry that represents all used flash space to
        // allow downloading the entire log in one file
        entries[entryIndex] = entriesPredefined[PREDEFINED_ENTRY_COUNT];
        entry = &entries[entryIndex];
        entry->curr_size = flashfsUsedSpace;
        entry->max_size = entry->curr_size;
        // This entry has timestamps corresponding to when the filesystem is mounted
        emfat_set_entry_cma(entry);
        ++entryIndex;
    }
#endif // USE_FLASHFS

    // Padding file to fill out the filesystem size to FILESYSTEM_SIZE_MB
    if (flashfsUsedSpace * 2 < FILESYSTEM_SIZE_MB * 1024 * 1024) {
        entries[entryIndex] = entriesPredefined[PREDEFINED_ENTRY_COUNT + 1];
        entry = &entries[entryIndex];
        // used space is doubled because of the individual files plus the single complete file
        entry->curr_size = (FILESYSTEM_SIZE_MB * 1024 * 1024) - (flashfsUsedSpace * 2);
        entry->max_size = entry->curr_size;
        // This entry has timestamps corresponding to when the filesystem is mounted
        emfat_set_entry_cma(entry);
    }

    emfat_init(&emfat, "RTFL       ", entries);
    LED0_OFF;
}

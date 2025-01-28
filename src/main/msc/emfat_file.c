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

#define EMFAT_MAX_LOG_ENTRY 100

#define FILESYSTEM_MIN_SIZE_MB 64

#define HDR_BUF_SIZE 32

#ifdef USE_EMFAT_AUTORUN
static const char autorun_file[] =
    "[autorun]\r\n"
    "icon=icon.ico\r\n"
    "label=Rotorflight Blackbox\r\n" ;
#define AUTORUN_SIZE (sizeof(autorun_file) - 1)
#endif

#ifdef USE_EMFAT_README
static const char readme_file[] =
    "This is readme file\r\n";
#define README_SIZE  (sizeof(readme_file) - 1)
#endif

#ifdef USE_EMFAT_ICON
static const char icon_file[] =
{
    #include "icon.hex"
};
#define ICON_SIZE    (sizeof(icon_file))
#endif

enum {
    EMFAT_ENTRY_DIR = 0,
#ifdef USE_EMFAT_AUTORUN
    EMFAT_ENTRY_AUTORUN,
#endif
#ifdef USE_EMFAT_ICON
    EMFAT_ENTRY_ICON,
#endif
#ifdef USE_EMFAT_README
    EMFAT_ENTRY_README,
#endif
    EMFAT_PREDEF_FILE_COUNT,

    // Two extra entries
    EMFAT_ENTRY_ALL_LOGS = EMFAT_PREDEF_FILE_COUNT,
    EMFAT_ENTRY_PADDING,

    EMFAT_PREDEF_ENTRY_COUNT
};

// RTFL_002_20251012_141213.bbl
//     1234567890123456789012345
#define EMFAT_MAX_NAME_LENGTH (MAX_NAME_LENGTH + 25)

#define EMFAT_MAX_ENTRY (EMFAT_MAX_LOG_ENTRY + EMFAT_PREDEF_ENTRY_COUNT + 1)

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(1,1,2000, 0,0,0)
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
    // index                    name            dir    attr         lvl offset  size             max_size        user                time  read               write
    [EMFAT_ENTRY_DIR]       = { "",             true,  0,           0,  0,      0,               0,              0,                  CMA,  NULL,              NULL, { 0 } },
#ifdef USE_EMFAT_AUTORUN
    [EMFAT_ENTRY_AUTORUN]   = { "autorun.inf",  false, ATTR_HIDDEN, 1,  0,      AUTORUN_SIZE,    AUTORUN_SIZE,   (long)autorun_file, CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_ICON
    [EMFAT_ENTRY_ICON]      = { "icon.ico",     false, ATTR_HIDDEN, 1,  0,      ICON_SIZE,       ICON_SIZE,      (long)icon_file,    CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_README
    [EMFAT_ENTRY_README]    = { "readme.txt",   false, 0,           1,  0,      README_SIZE,     1024*1024,      (long)readme_file,  CMA,  memory_read_proc,  NULL, { 0 } },
#endif
    [EMFAT_ENTRY_ALL_LOGS]  = { "",             false, 0,           1,  0,      0,               0,              0,                  CMA,  bblog_read_proc,   NULL, { 0 } },
    [EMFAT_ENTRY_PADDING]   = { "padding.nul",  false, ATTR_HIDDEN, 1,  0,      0,               0,              0,                  CMA,  NULL,              NULL, { 0 } },
};

static char logFileNames[EMFAT_MAX_LOG_ENTRY + 1][EMFAT_MAX_NAME_LENGTH];

static emfat_entry_t entries[EMFAT_MAX_ENTRY];

emfat_t emfat;

static uint32_t cmaTime = CMA_TIME;

static char logPrefix[MAX_NAME_LENGTH + 1];

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

static void emfat_set_log_file_name(emfat_entry_t *entry, int number)
{
    if (entry->cma_time[0] == cmaTime && number > 0) {
        tfp_sprintf(logFileNames[number],
                    "%s_%03d.bbl",
                    logPrefix,
                    number);
    } else {
        tfp_sprintf(logFileNames[number],
                    "%s%s_%04d%02d%02d_%02d%02d%02d.bbl",
                    logPrefix,
                    number == 0 ? "_all" : "",
                    emfat_decode_year(entry->cma_time[0]),
                    emfat_decode_month(entry->cma_time[0]),
                    emfat_decode_day(entry->cma_time[0]),
                    emfat_decode_hour(entry->cma_time[0]),
                    emfat_decode_minute(entry->cma_time[0]),
                    emfat_decode_second(entry->cma_time[0]));
    }

    entry->name = logFileNames[number];
}

static void emfat_set_entry_size(emfat_entry_t *entry, uint32_t offset, uint32_t size)
{
    entry->level = 1;
    entry->offset = offset;
    entry->curr_size = size;
    entry->max_size = entry->curr_size;
}

static void emfat_set_log_entry(emfat_entry_t *entry, uint32_t offset, uint32_t size)
{
    emfat_set_entry_size(entry, offset, size);

    // Log data reader
    entry->readcb = bblog_read_proc;

    // Set file modification/access times to be the same as the creation time
    entry->cma_time[1] = entry->cma_time[0];
    entry->cma_time[2] = entry->cma_time[0];
}

static int emfat_find_log(emfat_entry_t *entry, int maxCount, int flashfsUsedSpace)
{
    static uint8_t buffer[HDR_BUF_SIZE];

    const char *logHeader = "H Product:Blackbox";
    int lenLogHeader = strlen(logHeader);
    const char *timeHeader = "H Log start datetime:";
    int lenTimeHeader = strlen(timeHeader);

    int timeHeaderMatched = 0;
    int lastOffset = -1;
    int currOffset = 0;
    int buffOffset;
    int hdrOffset;
    int fileNumber = 1;
    int logCount = 0;

    for ( ; currOffset < flashfsUsedSpace ; currOffset += flashGetGeometry()->pageSize) {

        mscSetActive();
        mscActivityLed();

        flashfsReadAbs(currOffset, buffer, HDR_BUF_SIZE);

        if (strncmp((char *)buffer, logHeader, lenLogHeader)) {
            continue;
        }

        // The length of the previous record is now known
        if (lastOffset != -1) {
            // Record the previous entry
            emfat_set_log_file_name(entry, fileNumber);
            emfat_set_log_entry(entry, lastOffset, currOffset - lastOffset);

            entry++;
            logCount++;
            fileNumber++;
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
    if (fileNumber < maxCount && lastOffset != currOffset) {
        emfat_set_log_file_name(entry, fileNumber);
        emfat_set_log_entry(entry, lastOffset, currOffset - lastOffset);
        logCount++;
    }

    return logCount;
}
#endif  // USE_FLASHFS

void emfat_init_files(void)
{
    emfat_entry_t *entry = entries;

    memset(entries, 0, sizeof(entries));

#ifdef USE_PERSISTENT_MSC_RTC
    rtcTime_t mscRebootRtc;
    if (rtcPersistRead(&mscRebootRtc)) {
        const int32_t rtcSeconds = rtcTimeGetSeconds(&mscRebootRtc);
        cmaTime = emfat_cma_time_from_unix((uint32_t)rtcSeconds);
    }
#endif

    // create the predefined entries
    for (size_t i = 0 ; i < EMFAT_PREDEF_FILE_COUNT ; i++) {
        memcpy(entry, &entriesPredefined[i], sizeof(emfat_entry_t));
        emfat_set_entry_cma(entry);
        entry++;
    }

#ifdef USE_FLASHFS
    strcpy(logPrefix, pilotConfig()->name[0] ? pilotConfig()->name : "rtfl");
    legalize_filename(logPrefix);

    flashInit(flashConfig());
    flashfsInit();
    LED0_OFF;

    const int flashfsUsedSpace = flashfsGetOffset();

    // Detect and create entries for each individual log
    const int logCount = emfat_find_log(entry, EMFAT_MAX_LOG_ENTRY, flashfsUsedSpace);
    entry += logCount;

    if (logCount > 0) {
        // Create the all logs entry that represents all used flash space to
        // allow downloading the entire log in one file
        memcpy(entry, &entriesPredefined[EMFAT_ENTRY_ALL_LOGS], sizeof(emfat_entry_t));

        emfat_set_entry_cma(entry);
        emfat_set_log_file_name(entry, 0);
        emfat_set_log_entry(entry, 0, flashfsUsedSpace);

        entry++;
    }
#endif // USE_FLASHFS

    // Padding file to fill out the filesystem size to FILESYSTEM_MIN_SIZE_MB
    const int padding = FILESYSTEM_MIN_SIZE_MB * 1024 * 1024 - flashfsUsedSpace * 2;

    if (padding > 0) {
        memcpy(entry, &entriesPredefined[EMFAT_ENTRY_PADDING], sizeof(emfat_entry_t));

        emfat_set_entry_cma(entry);
        emfat_set_entry_size(entry, 0, padding);

        entry++;
    }

    emfat_init(&emfat, "Rotorflight", entries);

    LED0_OFF;
}

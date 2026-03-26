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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/bitarray.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

#include "pg/piniobox.h"

#include "msp_box.h"


#define BOXITEM(boxid, name, perm)   { .boxId = boxid, .boxName = name, .permanentId = perm, }

static const box_t boxes[CHECKBOX_ITEM_COUNT] =
{
    BOXITEM(BOXARM, "ARM", 0),
    BOXITEM(BOXANGLE, "ANGLE", 1),
    BOXITEM(BOXHORIZON, "HORIZON", 2),
    BOXITEM(BOXALTHOLD, "ALTHOLD", 3),
//    BOXITEM(BOXANTIGRAVITY, "ANTI GRAVITY", 4),
//    BOXITEM(BOXMAG, "MAG", 5),
//    BOXITEM(BOXCAMSTAB, "CAMSTAB", 8),
//    BOXITEM(BOXCAMTRIG, "CAMTRIG", 9),
//    BOXITEM(BOXGPSHOME, "GPS HOME", 10),
//    BOXITEM(BOXGPSHOLD, "GPS HOLD", 11),
//    BOXITEM(BOXPASSTHRU, "PASSTHRU", 12),
    BOXITEM(BOXBEEPERON, "BEEPER", 13),
//    BOXITEM(BOXLEDMAX, "LEDMAX", 14),
    BOXITEM(BOXLEDLOW, "LEDLOW", 15),
//    BOXITEM(BOXLLIGHTS, "LLIGHTS", 16),
    BOXITEM(BOXCALIB, "CALIB", 17),
//    BOXITEM(BOXGOV, "GOVERNOR", 18),
    BOXITEM(BOXOSD, "OSD DISABLE", 19),
    BOXITEM(BOXTELEMETRY, "TELEMETRY", 20),
//    BOXITEM(BOXGTUNE, "GTUNE", 21),
//    BOXITEM(BOXRANGEFINDER, "RANGEFINDER", 22),
//    BOXITEM(BOXSERVO1, "SERVO1", 23),
//    BOXITEM(BOXSERVO2, "SERVO2", 24),
//    BOXITEM(BOXSERVO3, "SERVO3", 25),
    BOXITEM(BOXBLACKBOX, "BLACKBOX", 26),
    BOXITEM(BOXFAILSAFE, "FAILSAFE", 27),
//    BOXITEM(BOXAIRMODE, "AIR MODE", 28),
//    BOXITEM(BOX3D, "3D DISABLE / SWITCH", 29),
//    BOXITEM(BOXFPVANGLEMIX, "FPV ANGLE MIX", 30),
    BOXITEM(BOXBLACKBOXERASE, "BLACKBOX ERASE", 31),
    BOXITEM(BOXCAMERA1, "CAMERA CONTROL 1", 32),
    BOXITEM(BOXCAMERA2, "CAMERA CONTROL 2", 33),
    BOXITEM(BOXCAMERA3, "CAMERA CONTROL 3", 34),
//    BOXITEM(BOXFLIPOVERAFTERCRASH, "FLIP OVER AFTER CRASH", 35),
    BOXITEM(BOXPREARM, "PREARM", 36),
    BOXITEM(BOXBEEPGPSCOUNT, "GPS BEEP SATELLITE COUNT", 37),
//    BOXITEM(BOX3DONASWITCH, "3D ON A SWITCH", 38),
    BOXITEM(BOXVTXPITMODE, "VTX PIT MODE", 39),
    BOXITEM(BOXUSER1, "USER1", 40),
    BOXITEM(BOXUSER2, "USER2", 41),
    BOXITEM(BOXUSER3, "USER3", 42),
    BOXITEM(BOXUSER4, "USER4", 43),
//    BOXITEM(BOXPIDAUDIO, "PID AUDIO", 44),
    BOXITEM(BOXPARALYZE, "PARALYZE", 45),
    BOXITEM(BOXGPSRESCUE, "GPS RESCUE", 46),
    BOXITEM(BOXTRAINER, "TRAINER", 47),
    BOXITEM(BOXVTXCONTROLDISABLE, "VTX CONTROL DISABLE", 48),
//    BOXITEM(BOXLAUNCHCONTROL, "LAUNCH CONTROL", 49),
//    BOXITEM(BOXMSPOVERRIDE, "MSP OVERRIDE", 50),
    BOXITEM(BOXSTICKCOMMANDDISABLE, "STICK COMMANDS DISABLE", 51),
    BOXITEM(BOXBEEPERMUTE, "BEEPER MUTE", 52),
    BOXITEM(BOXRESCUE, "RESCUE", 53),
//    BOXITEM(BOXAUTOROTATION, "AUTOROTATION", 54),
    BOXITEM(BOXGOVFALLBACK, "GOVERNOR FALLBACK", 55),
    BOXITEM(BOXGOVSUSPEND, "GOVERNOR SUSPEND", 56),
    BOXITEM(BOXGOVBYPASS, "GOVERNOR BYPASS", 57),
};

// mask of enabled IDs, calculated on startup based on enabled features. boxId_e is used as bit index

static boxBitmask_t activeBoxIds;

const box_t *findBoxByBoxId(boxId_e boxId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->boxId == boxId)
            return candidate;
    }
    return NULL;
}

const box_t *findBoxByPermanentId(uint8_t permanentId)
{
    for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
        const box_t *candidate = &boxes[i];
        if (candidate->permanentId == permanentId)
            return candidate;
    }
    return NULL;
}

static bool activeBoxIdGet(boxId_e boxId)
{
    if (boxId > sizeof(activeBoxIds) * 8) {
        return false;
    }

    return bitArrayGet(&activeBoxIds, boxId);
}

void serializeBoxNameFn(sbuf_t *dst, const box_t *box)
{
#if defined(USE_CUSTOM_BOX_NAMES)
    if (box->boxId == BOXUSER1 && strlen(modeActivationConfig()->box_user_1_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_1_name);
    } else if (box->boxId == BOXUSER2 && strlen(modeActivationConfig()->box_user_2_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_2_name);
    } else if (box->boxId == BOXUSER3 && strlen(modeActivationConfig()->box_user_3_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_3_name);
    } else if (box->boxId == BOXUSER4 && strlen(modeActivationConfig()->box_user_4_name) > 0) {
        sbufWriteString(dst, modeActivationConfig()->box_user_4_name);
    } else
#endif
    {
        sbufWriteString(dst, box->boxName);
    }
    sbufWriteU8(dst, ';');
}

void serializeBoxPermanentIdFn(sbuf_t *dst, const box_t *box)
{
    sbufWriteU8(dst, box->permanentId);
}

// serialize 'page' of boxNames.
// Each page contains at most 32 boxes
void serializeBoxReply(sbuf_t *dst, int page, serializeBoxFn *serializeBox)
{
    unsigned boxIdx = 0;
    unsigned pageStart = page * 32;
    unsigned pageEnd = pageStart + 32;
    for (boxId_e id = 0; id < CHECKBOX_ITEM_COUNT; id++) {
        if (activeBoxIdGet(id)) {
            if (boxIdx >= pageStart && boxIdx < pageEnd) {
                (*serializeBox)(dst, findBoxByBoxId(id));
            }
            boxIdx++;                 // count active boxes
        }
    }
}

void initActiveBoxIds(void)
{
    // calculate used boxes based on features and set corresponding activeBoxIds bits
    boxBitmask_t ena;  // temporary variable to collect result
    memset(&ena, 0, sizeof(ena));

    // macro to enable boxId (BoxidMaskEnable). Reference to ena is hidden, local use only
#define BME(boxId) do { bitArraySet(&ena, boxId); } while (0)
    BME(BOXARM);
    BME(BOXPREARM);
    BME(BOXPARALYZE);
    BME(BOXFAILSAFE);

    if (sensors(SENSOR_ACC)) {
        BME(BOXANGLE);
        BME(BOXHORIZON);
        BME(BOXRESCUE);
#ifdef USE_ACRO_TRAINER
        BME(BOXTRAINER);
#endif
    }

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)) {
#ifdef USE_GPS_RESCUE
        BME(BOXGPSRESCUE);
#endif
        BME(BOXBEEPGPSCOUNT);
    }
#endif

    BME(BOXBEEPERON);
    BME(BOXBEEPERMUTE);

#ifdef USE_LED_STRIP
    if (featureIsEnabled(FEATURE_LED_STRIP)) {
        BME(BOXLEDLOW);
    }
#endif

#ifdef USE_BLACKBOX
    BME(BOXBLACKBOX);
#ifdef USE_FLASHFS
    BME(BOXBLACKBOXERASE);
#endif
#endif

    BME(BOXOSD);

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        BME(BOXTELEMETRY);
    }
#endif

#ifdef USE_RCDEVICE
    BME(BOXCAMERA1);
    BME(BOXCAMERA2);
    BME(BOXCAMERA3);
#endif

#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    BME(BOXVTXPITMODE);
    BME(BOXVTXCONTROLDISABLE);
#endif

    BME(BOXGOVFALLBACK);
    BME(BOXGOVSUSPEND);
    BME(BOXGOVBYPASS);

#ifdef USE_PINIOBOX
    // Turn BOXUSERx only if pinioBox facility monitors them, as the facility is the only BOXUSERx observer.
    // Note that pinioBoxConfig can be set to monitor any box.
    for (int i = 0; i < PINIO_COUNT; i++) {
        if (pinioBoxConfig()->permanentId[i] != PERMANENT_ID_NONE) {
            const box_t *box = findBoxByPermanentId(pinioBoxConfig()->permanentId[i]);
            if (box) {
                switch(box->boxId) {
                case BOXUSER1:
                case BOXUSER2:
                case BOXUSER3:
                case BOXUSER4:
                    BME(box->boxId);
                    break;
                default:
                    break;
                }
            }
        }
    }
#endif

    BME(BOXSTICKCOMMANDDISABLE);

#undef BME
    // check that all enabled IDs are in boxes array (check may be skipped when using findBoxById() functions)
    for (boxId_e boxId = 0;  boxId < CHECKBOX_ITEM_COUNT; boxId++)
        if (bitArrayGet(&ena, boxId)
            && findBoxByBoxId(boxId) == NULL)
            bitArrayClr(&ena, boxId);                 // this should not happen, but handle it gracefully

    activeBoxIds = ena;                               // set global variable
}

// return state of given boxId box, handling ARM and FLIGHT_MODE
bool getBoxIdState(boxId_e boxid)
{
    const uint8_t boxIdToFlightModeMap[] = BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER;

    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode
    STATIC_ASSERT(ARRAYLEN(boxIdToFlightModeMap) == BOXID_FLIGHTMODE_LAST + 1, FLIGHT_MODE_BOXID_MAP_INITIALIZER_does_not_match_boxId_e);

    if (boxid == BOXARM) {
        return ARMING_FLAG(ARMED);
    } else if (boxid <= BOXID_FLIGHTMODE_LAST) {
        return FLIGHT_MODE(1 << boxIdToFlightModeMap[boxid]);
    } else {
        return IS_RC_MODE_ACTIVE(boxid);
    }
}

// pack used flightModeFlags into supplied array
// returns number of bits used
int packFlightModeFlags(boxBitmask_t *mspFlightModeFlags)
{
    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    memset(mspFlightModeFlags, 0, sizeof(boxBitmask_t));
    // map boxId_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    unsigned mspBoxIdx = 0;           // index of active boxId (matches sent permanentId and boxNames)
    for (boxId_e boxId = 0; boxId < CHECKBOX_ITEM_COUNT; boxId++) {
        if (activeBoxIdGet(boxId)) {
            if (getBoxIdState(boxId))
                bitArraySet(mspFlightModeFlags, mspBoxIdx);       // box is enabled
            mspBoxIdx++;                                          // box is active, count it
        }
    }
    // return count of used bits
    return mspBoxIdx;
}

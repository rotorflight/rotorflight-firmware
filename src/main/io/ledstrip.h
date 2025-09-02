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

#pragma once

#include <stdlib.h>

#include "common/color.h"
#include "common/time.h"

#include "drivers/io_types.h"
#include "drivers/light_ws2811strip.h"

#include "pg/adjustments.h"
#include "pg/ledstrip.h"

#define LED_POS_OFFSET                  0
#define LED_FUNCTION_OFFSET             8
#define LED_OVERLAY_OFFSET             12
#define LED_COLOR_OFFSET               20
#define LED_DIRECTION_OFFSET           24
#define LED_PARAMS_OFFSET              30
#define LED_BLINKPATTERN_OFFSET        33
#define LED_BLINKPAUSE_OFFSET          49
#define LED_ALTCOLOR_OFFSET            53

#define LED_POS_BITCNT                  8
#define LED_FUNCTION_BITCNT             4
#define LED_OVERLAY_BITCNT              8
#define LED_COLOR_BITCNT                4
#define LED_DIRECTION_BITCNT            6
#define LED_PARAMS_BITCNT               3
#define LED_BLINKPATTERN_BITCNT        16
#define LED_BLINKPAUSE_BITCNT           4
#define LED_ALTCOLOR_BITCNT             4

#define LED_FLAG_OVERLAY_MASK ((1 << LED_OVERLAY_BITCNT) - 1)
#define LED_FLAG_DIRECTION_MASK ((1 << LED_DIRECTION_BITCNT) - 1)

#define LED_MOV_POS(pos) ((pos) << LED_POS_OFFSET)
#define LED_MOV_FUNCTION(func) ((func) << LED_FUNCTION_OFFSET)
#define LED_MOV_OVERLAY(overlay) ((overlay) << LED_OVERLAY_OFFSET)
#define LED_MOV_COLOR(colorId) ((colorId) << LED_COLOR_OFFSET)
#define LED_MOV_DIRECTION(direction) ((direction) << LED_DIRECTION_OFFSET)
#define LED_MOV_PARAMS(param) ((param) << LED_PARAMS_OFFSET)
#define LED_MOV_BLINKPATTERN(pattern) ((pattern) << LED_BLINKPATTERN_OFFSET)
#define LED_MOV_BLINKPAUSE(pause) ((pause) << LED_BLINKPAUSE_OFFSET)
#define LED_MOV_ALTCOLOR(altColorId) ((altColorId) << LED_ALTCOLOR_OFFSET)

#define LED_BIT_MASK(len) ((1 << (len)) - 1)

#define LED_POS_MASK LED_MOV_POS(((1 << LED_POS_BITCNT) - 1))
#define LED_FUNCTION_MASK LED_MOV_FUNCTION(((1 << LED_FUNCTION_BITCNT) - 1))
#define LED_OVERLAY_MASK LED_MOV_OVERLAY(LED_FLAG_OVERLAY_MASK)
#define LED_COLOR_MASK LED_MOV_COLOR(((1 << LED_COLOR_BITCNT) - 1))
#define LED_DIRECTION_MASK LED_MOV_DIRECTION(LED_FLAG_DIRECTION_MASK)
#define LED_PARAMS_MASK LED_MOV_PARAMS(((1 << LED_PARAMS_BITCNT) - 1))
#define LED_BLINKPATTERN_MASK LED_MOV_BLINKPATTERN(((1 << LED_BLINKPATTERN_BITCNT) - 1))
#define LED_BLINKPAUSE_MASK LED_MOV_BLINKPAUSE(((1 << LED_BLINKPAUSE_BITCNT) - 1))
#define LED_ALTCOLOR_MASK LED_MOV_ALTCOLOR(((1 << LED_ALTCOLOR_BITCNT) - 1))

#define LED_FLAG_OVERLAY(id) (1 << (id))
#define LED_FLAG_DIRECTION(id) (1 << (id))

#define LED_X_BIT_OFFSET 4
#define LED_Y_BIT_OFFSET 0
#define LED_XY_MASK      0x0F
#define CALCULATE_LED_XY(x, y) ((((x) & LED_XY_MASK) << LED_X_BIT_OFFSET) | (((y) & LED_XY_MASK) << LED_Y_BIT_OFFSET))

typedef enum {
    COLOR_BLACK = 0,
    COLOR_WHITE,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_LIME_GREEN,
    COLOR_GREEN,
    COLOR_MINT_GREEN,
    COLOR_CYAN,
    COLOR_LIGHT_BLUE,
    COLOR_BLUE,
    COLOR_DARK_VIOLET,
    COLOR_MAGENTA,
    COLOR_DEEP_PINK,
    COLOR_COUNT
} colorId_e;

typedef enum {
    LED_MODE_ORIENTATION = 0,
    LED_MODE_HORIZON,
    LED_MODE_ANGLE,
    LED_MODE_RESCUE,
    LED_SPECIAL,
    LED_AUX_CHANNEL
} ledModeIndex_e;

typedef enum {
    LED_SCOLOR_DISARMED = 0,
    LED_SCOLOR_ARMED,
    LED_SCOLOR_ANIMATION,
    LED_SCOLOR_BACKGROUND,
    LED_SCOLOR_BLINKBACKGROUND,
    LED_SCOLOR_GPSNOSATS,
    LED_SCOLOR_GPSNOLOCK,
    LED_SCOLOR_GPSLOCKED
} ledSpecialColorIds_e;

typedef enum {
    LED_DIRECTION_NORTH = 0,
    LED_DIRECTION_EAST,
    LED_DIRECTION_SOUTH,
    LED_DIRECTION_WEST,
    LED_DIRECTION_UP,
    LED_DIRECTION_DOWN
} ledDirectionId_e;

typedef enum {
    LED_FUNCTION_COLOR,
    LED_FUNCTION_FLIGHT_MODE,
    LED_FUNCTION_ARM_STATE,
    LED_FUNCTION_BATTERY,
    LED_FUNCTION_RSSI,
    LED_FUNCTION_GPS,
    LED_FUNCTION_THRUST_RING
} ledBaseFunctionId_e;

typedef enum {
    LED_OVERLAY_THROTTLE,
    LED_OVERLAY_LARSON_SCANNER,
    LED_OVERLAY_BLINK,
    LED_OVERLAY_VTX,
    LED_OVERLAY_INDICATOR,
    LED_OVERLAY_WARNING,
    LED_OVERLAY_FLICKER,
    LED_OVERLAY_DIMMER
} ledOverlayId_e;

typedef enum {
    LED_PROFILE_RACE = 0,
    LED_PROFILE_BEACON,
#ifdef USE_LED_STRIP_STATUS_MODE
    LED_PROFILE_STATUS,
    LED_PROFILE_STATUS_ALT,
#endif
    LED_PROFILE_COUNT
} ledProfile_e;

typedef struct ledCounts_s {
    uint8_t count;
    uint8_t ring;
    uint8_t larson;
    uint8_t ringSeqLen;
    uint8_t blinkPauses[LED_MAX_STRIP_LENGTH];
} ledCounts_t;


#define LF(name) LED_FUNCTION_ ## name
#define LO(name) LED_FLAG_OVERLAY(LED_OVERLAY_ ## name)
#define LD(name) LED_FLAG_DIRECTION(LED_DIRECTION_ ## name)
#define DEFINE_LED(x, y, col, dir, func, ol, params, blinkPattern, blinkPause, altColor) (LED_MOV_POS(CALCULATE_LED_XY(x, y)) | LED_MOV_COLOR(col) | LED_MOV_DIRECTION(dir) | LED_MOV_FUNCTION(func) | LED_MOV_OVERLAY(ol) | LED_MOV_PARAMS(params) | LED_MOV_BLINKPATTERN(blinkPattern) | LED_MOV_BLINKPAUSE(blinkPause) | LED_MOV_ALTCOLOR(altColor))

static inline uint8_t ledGetXY(const ledConfig_t *lcfg)            { return ((*lcfg >> LED_POS_OFFSET) & LED_BIT_MASK(LED_POS_BITCNT)); }
static inline uint8_t ledGetX(const ledConfig_t *lcfg)             { return ((*lcfg >> (LED_POS_OFFSET + LED_X_BIT_OFFSET)) & LED_XY_MASK); }
static inline uint8_t ledGetY(const ledConfig_t *lcfg)             { return ((*lcfg >> (LED_POS_OFFSET + LED_Y_BIT_OFFSET)) & LED_XY_MASK); }
static inline uint8_t ledGetFunction(const ledConfig_t *lcfg)      { return ((*lcfg >> LED_FUNCTION_OFFSET) & LED_BIT_MASK(LED_FUNCTION_BITCNT)); }
static inline uint8_t ledGetOverlay(const ledConfig_t *lcfg)       { return ((*lcfg >> LED_OVERLAY_OFFSET) & LED_BIT_MASK(LED_OVERLAY_BITCNT)); }
static inline uint8_t ledGetColor(const ledConfig_t *lcfg)         { return ((*lcfg >> LED_COLOR_OFFSET) & LED_BIT_MASK(LED_COLOR_BITCNT)); }
static inline uint8_t ledGetDirection(const ledConfig_t *lcfg)     { return ((*lcfg >> LED_DIRECTION_OFFSET) & LED_BIT_MASK(LED_DIRECTION_BITCNT)); }
static inline uint8_t ledGetParams(const ledConfig_t *lcfg)        { return ((*lcfg >> LED_PARAMS_OFFSET) & LED_BIT_MASK(LED_PARAMS_BITCNT)); }
static inline uint16_t ledGetBlinkPattern(const ledConfig_t *lcfg) { return ((*lcfg >> LED_BLINKPATTERN_OFFSET) & LED_BIT_MASK(LED_BLINKPATTERN_BITCNT)); }
static inline uint8_t ledGetBlinkPause(const ledConfig_t *lcfg)    { return ((*lcfg >> LED_BLINKPAUSE_OFFSET) & LED_BIT_MASK(LED_BLINKPAUSE_BITCNT)); }
static inline uint8_t ledGetAltColor(const ledConfig_t *lcfg)      { return ((*lcfg >> LED_ALTCOLOR_OFFSET) & LED_BIT_MASK(LED_ALTCOLOR_BITCNT)); }

static inline bool ledGetOverlayBit(const ledConfig_t *lcfg, int id) { return ((ledGetOverlay(lcfg) >> id) & 1); }
static inline bool ledGetDirectionBit(const ledConfig_t *lcfg, int id) { return ((ledGetDirection(lcfg) >> id) & 1); }

static inline int randomInt(int min, int max) { return min + rand() % (max - min); }

bool parseColor(int index, const char *colorConfig);

bool parseLedStripConfig(int ledIndex, const char *config);
void generateLedConfig(ledConfig_t *ledConfig, char *ledConfigBuffer, size_t bufferSize);
void reevaluateLedConfig(void);

void ledStripInit(void);
void ledStripEnable(void);
void ledStripDisable(void);
void ledStripUpdate(timeUs_t currentTimeUs);

bool setModeColor(ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex);

void applyDefaultLedStripConfig(ledConfig_t *ledConfig);
void applyDefaultColors(hsvColor_t *colors);
void applyDefaultModeColors(modeColorIndexes_t *modeColors);
void applyDefaultSpecialColors(specialColorIndexes_t *specialColors);

void updateRequiredOverlay(void);

uint8_t getLedProfile(void);
void setLedProfile(uint8_t profile);

ADJFUN_DECLARE(LED_PROFILE)

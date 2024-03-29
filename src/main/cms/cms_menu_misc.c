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
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_ledstrip.h"

#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/time.h"

#include "fc/rc_controls.h"

#include "flight/mixer.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "cms_menu_misc.h"

//
// Misc
//

static const void *cmsx_menuRcOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    inhibitSaveMenu();

    return NULL;
}

static const void *cmsx_menuRcConfirmBack(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);

    if (self && ((self->flags & OSD_MENU_ELEMENT_MASK) == OME_Back)) {
        return NULL;
    } else {
        return MENU_CHAIN_BACK;
    }
}

static int16_t rcDataInt[8];

static const void *cmsx_menuRcOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

    for (int i = 0; i < 8; i++) {
        rcDataInt[i] = lrintf(rcCommand[i]);
    }
    rcDataInt[THROTTLE] = lrintf(getThrottleCommand());

    return NULL;
}

//
// RC preview
//
static const OSD_Entry cmsx_menuRcEntries[] =
{
    { "-- RC PREV --", OME_Label, NULL, NULL},

    { "ROLL",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[ROLL],       1, 2500, 0 } },
    { "PITCH", OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[PITCH],      1, 2500, 0 } },
    { "YAW",   OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[YAW],        1, 2500, 0 } },
    { "COLL",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[COLLECTIVE], 1, 2500, 0 } },

    { "THR",   OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[THROTTLE],   1, 2500, 0 } },
    { "AUX1",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[AUX1],       1, 2500, 0 } },
    { "AUX2",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[AUX2],       1, 2500, 0 } },
    { "AUX3",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[AUX3],       1, 2500, 0 } },

    { "BACK",  OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuRcPreview = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XRCPREV",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuRcOnEnter,
    .onExit = cmsx_menuRcConfirmBack,
    .onDisplayUpdate = cmsx_menuRcOnDisplayUpdate,
    .entries = cmsx_menuRcEntries
};

static uint16_t motorConfig_minthrottle;

static const void *cmsx_menuMiscOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    motorConfig_minthrottle = motorConfig()->minthrottle;

    return NULL;
}

static const void *cmsx_menuMiscOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    motorConfigMutable()->minthrottle = motorConfig_minthrottle;

    return NULL;
}

static const OSD_Entry menuMiscEntries[]=
{
    { "-- MISC --", OME_Label, NULL, NULL },

    { "MIN THR",       OME_UINT16 | REBOOT_REQUIRED,  NULL,          &(OSD_UINT16_t){ &motorConfig_minthrottle,            1000, 2000, 1 } },
    { "RC PREV",       OME_Submenu, cmsMenuChange, &cmsx_menuRcPreview},

    { "BACK", OME_Back, NULL, NULL},
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuMisc = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XMISC",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuMiscOnEnter,
    .onExit = cmsx_menuMiscOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuMiscEntries
};

#endif // CMS

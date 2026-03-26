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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"
#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "config/config.h"
#include "fc/rc_rates.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/pg.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "cli/settings.h"

//
// PID
//
static pidProfile_t *pidProfile;
static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[MAX_PROFILE_NAME_LENGTH + 5];
static uint8_t tempPid[3][3];
static uint16_t tempPidF[3];
static uint16_t tempPidO[3];
static uint16_t tempPidB[3];

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[MAX_RATE_PROFILE_NAME_LENGTH + 5];
static controlRateConfig_t rateProfile;

#ifdef USE_MULTI_GYRO
static const char * const osdTableGyroToUse[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

static void setProfileIndexString(char *profileString, int profileIndex, char *profileName)
{
    int charIndex = 0;
    profileString[charIndex++] = '1' + profileIndex;

#ifdef USE_PROFILE_NAMES
    const int profileNameLen = strlen(profileName);

    if (profileNameLen > 0) {
        profileString[charIndex++] = ' ';
        profileString[charIndex++] = '(';
        for (int i = 0; i < profileNameLen; i++) {
            profileString[charIndex++] = toupper(profileName[i]);
        }
        profileString[charIndex++] = ')';
    }
#else
    UNUSED(profileName);
#endif

    profileString[charIndex] = '\0';
}

static void cmsx_initPidProfile()
{
    pidProfileIndex =    getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex +1;
    pidProfile =         pidProfilesMutable(pidProfileIndex);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, pidProfile->profileName);
}

void cmsx_updateCurrentPidProfile()
{
  // Update current active PID profile only if it is the same as the one CMS has been working on
  if (pidProfileIndex == getCurrentPidProfileIndex() ) {
    pidLoadProfile(currentPidProfile);
  }
}


static void cmsx_initRateProfile()
{
    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex + 1;
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));

    setProfileIndexString(rateProfileIndexString, rateProfileIndex, controlRateProfilesMutable(rateProfileIndex)->profileName);
}

static const void *cmsx_menuImu_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_initPidProfile();
    cmsx_initRateProfile();

    return NULL;
}

static const void *cmsx_menuImu_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    cmsx_updateCurrentPidProfile();
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

static const void *cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    pidProfileIndex = tmpPidProfileIndex - 1;
    changePidProfile(pidProfileIndex);
    pidProfile = pidProfilesMutable(pidProfileIndex);
    setProfileIndexString(pidProfileIndexString, pidProfileIndex, pidProfile->profileName);

    return NULL;
}

static const void *cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex - 1;
    changeControlRateProfile(rateProfileIndex);
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));
    setProfileIndexString(rateProfileIndexString, rateProfileIndex, controlRateProfilesMutable(rateProfileIndex)->profileName);

    return NULL;
}

static const void *cmsx_PidRead(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = pidProfile->pid[i].P;
        tempPid[i][1] = pidProfile->pid[i].I;
        tempPid[i][2] = pidProfile->pid[i].D;
        tempPidF[i] = pidProfile->pid[i].F;
        tempPidO[i] = pidProfile->pid[i].O;
        tempPidB[i] = pidProfile->pid[i].B;
    }

    return NULL;
}

static const void *cmsx_PidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_PidRead();
    return NULL;
}

static const void *cmsx_PidWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    for (uint8_t i = 0; i < 3; i++) {
        pidProfile->pid[i].P = tempPid[i][0];
        pidProfile->pid[i].I = tempPid[i][1];
        pidProfile->pid[i].D = tempPid[i][2];
        pidProfile->pid[i].F = tempPidF[i];
        pidProfile->pid[i].O = tempPidO[i];
        pidProfile->pid[i].B = tempPidB[i];
    }

    cmsx_updateCurrentPidProfile();
    return NULL;
}

static OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, pidProfileIndexString},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 200, 1 }},
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 200, 1 }},
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 200, 1 }},
    { "ROLL  F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_ROLL],  0, 2000, 1 }},
    { "ROLL  O", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidO[PID_ROLL],  0, 200, 1 }},
    { "ROLL  B", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidB[PID_ROLL],  0, 200, 1 }},

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 200, 1 }},
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 200, 1 }},
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 200, 1 }},
    { "PITCH F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_PITCH], 0, 2000, 1 }},
    { "PITCH O", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidO[PID_PITCH], 0, 200, 1 }},
    { "PITCH B", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidB[PID_PITCH], 0, 200, 1 }},

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 200, 1 }},
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 200, 1 }},
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 200, 1 }},
    { "YAW   F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_YAW],   0, 2000, 1 }},
    { "YAW   O", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidO[PID_YAW],   0, 200, 1 }},
    { "YAW   B", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidB[PID_YAW],   0, 200, 1 }},

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuPid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPidEntries
};

//
// Rate & Expo
//


static const void *cmsx_RateProfileOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return NULL;
}

static const void *cmsx_RateProfileOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);
    return NULL;
}

static const OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString },

    { "RC R RATE",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.rcRates[FD_ROLL],  1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1 }},
    { "RC P RATE",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.rcRates[FD_PITCH], 1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1}},
    { "RC Y RATE",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.rcRates[FD_YAW],   1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1 }},

    { "ROLL SUPER",  OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.sRates[FD_ROLL],    0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX, 1 }},
    { "PITCH SUPER", OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.sRates[FD_PITCH],   0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX, 1 }},
    { "YAW SUPER",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.sRates[FD_YAW],     0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX, 1 }},

    { "RC R EXPO",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.rcExpo[FD_ROLL],   0, 100, 1 }},
    { "RC P EXPO",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.rcExpo[FD_PITCH],  0, 100, 1 }},
    { "RC Y EXPO",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.rcExpo[FD_YAW],    0, 100, 1 }},

    { "ROLL LVL EXPO",  OME_UINT8, NULL, &(OSD_UINT8_t) { &rateProfile.levelExpo[FD_ROLL],  0, 100, 1 }},
    { "PITCH LVL EXPO", OME_UINT8, NULL, &(OSD_UINT8_t) { &rateProfile.levelExpo[FD_PITCH], 0, 100, 1 }},

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuRateProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENURATE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_RateProfileOnEnter,
    .onExit =  cmsx_RateProfileOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuRateProfileEntries
};

/////////////////// Tail/Yaw Profile menu items ///////////////////////

static uint8_t  cmsx_yawStopCW;
static uint8_t  cmsx_yawStopCCW;
static uint8_t  cmsx_yawCollectiveFF;
static uint8_t  cmsx_yawCyclicFF;
static uint8_t  cmsx_yawTTA;
static uint8_t  cmsx_yawPrecompCutoff;

static const void *cmsx_profileYawOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_yawStopCW =         pidProfile->yaw_cw_stop_gain;
    cmsx_yawStopCCW =        pidProfile->yaw_ccw_stop_gain;
    cmsx_yawCyclicFF =       pidProfile->yaw_cyclic_ff_gain;
    cmsx_yawCollectiveFF =   pidProfile->yaw_collective_ff_gain;
    cmsx_yawTTA =            pidProfile->governor.tta_gain;
    cmsx_yawPrecompCutoff =  pidProfile->yaw_precomp_cutoff;

    return NULL;
}

static const void *cmsx_profileYawOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile->yaw_cw_stop_gain =       cmsx_yawStopCW;
    pidProfile->yaw_ccw_stop_gain =      cmsx_yawStopCCW;
    pidProfile->yaw_cyclic_ff_gain =     cmsx_yawCyclicFF;
    pidProfile->yaw_collective_ff_gain = cmsx_yawCollectiveFF;
    pidProfile->governor.tta_gain =      cmsx_yawTTA;
    pidProfile->yaw_precomp_cutoff =     cmsx_yawPrecompCutoff;

    cmsx_updateCurrentPidProfile();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileYawEntries[] = {
    { "-- TAIL --",  OME_Label, NULL, pidProfileIndexString },
    { "STOP CW",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_yawStopCW,       25,    250,   1  }    },
    { "STOP CCW",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_yawStopCCW,      25,    250,   1  }    },
    { "PRECOMP CUT",OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_yawPrecompCutoff,25,    250,   1  }    },
    { "CYCl FF",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_yawCyclicFF,      0,    250,   1  }    },
    { "COLL FF",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_yawCollectiveFF,  0,    250,   1  }    },
    { "TTA GAIN",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_yawTTA,           0,    250,   1  }    },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileYaw = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFYAW",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileYawOnEnter,
    .onExit = cmsx_profileYawOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileYawEntries,
};

/////////////////// PID controller Profile menu items ///////////////////////

static uint8_t  cmsx_pidCyclicCrossCouplingGain;
static uint8_t  cmsx_pidCyclicCrossCouplingRatio;
static uint8_t  cmsx_pidCyclicCrossCouplingCutOff;
static uint8_t  cmsx_pidItermRelaxCutoff[3];

static const void *cmsx_profileCtrlOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_pidCyclicCrossCouplingGain =         pidProfile->cyclic_cross_coupling_gain;
    cmsx_pidCyclicCrossCouplingRatio =        pidProfile->cyclic_cross_coupling_ratio;
    cmsx_pidCyclicCrossCouplingCutOff =       pidProfile->cyclic_cross_coupling_cutoff;

    for (uint8_t i = 0; i < 3; i++) {
      cmsx_pidItermRelaxCutoff[i] = pidProfile->iterm_relax_cutoff[i];
    }

    return NULL;
}

static const void *cmsx_profileCtrlOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile->cyclic_cross_coupling_gain = cmsx_pidCyclicCrossCouplingGain;
    pidProfile->cyclic_cross_coupling_ratio = cmsx_pidCyclicCrossCouplingRatio;
    pidProfile->cyclic_cross_coupling_cutoff= cmsx_pidCyclicCrossCouplingCutOff;

    for (uint8_t i = 0; i < 3; i++) {
      pidProfile->iterm_relax_cutoff[i] = cmsx_pidItermRelaxCutoff[i];
    }

    cmsx_updateCurrentPidProfile();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileCtrlEntries[] = {
    { "- PID CTRL -", OME_Label, NULL, pidProfileIndexString },
    { "CY XC GAIN",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidCyclicCrossCouplingGain,   0,    250,   1  }    },
    { "CY XC RATIO",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidCyclicCrossCouplingRatio,  0,    200,   1  }    },
    { "CY XC CUT",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidCyclicCrossCouplingCutOff, 0,    250,   1  }    },
    { "I RLX CUT R",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidItermRelaxCutoff[0],       1,    100,   1  }    },
    { "I RLX CUT P",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidItermRelaxCutoff[1],       1,    100,   1  }    },
    { "I RLX CUT Y",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidItermRelaxCutoff[2],       1,    100,   1  }    },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileCtrl = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFCTRL",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileCtrlOnEnter,
    .onExit = cmsx_profileCtrlOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileCtrlEntries,
};

/*
*/

/////////////////// PID controller Bandwieth Profile menu items ///////////////////////

static uint8_t  cmsx_pidBandwidth[3];
static uint8_t  cmsx_pidDCutoff[3];
static uint8_t  cmsx_pidBCutoff[3];

static const void *cmsx_profileCtrlBwOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    for (uint8_t i = 0; i < 3; i++) {
      cmsx_pidBandwidth[i] = pidProfile->gyro_cutoff[i];
      cmsx_pidDCutoff[i] =   pidProfile->dterm_cutoff[i];
      cmsx_pidBCutoff[i] =   pidProfile->bterm_cutoff[i];
    }

    return NULL;
}

static const void *cmsx_profileCtrlBwOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    for (uint8_t i = 0; i < 3; i++) {
      pidProfile->gyro_cutoff[i] =  cmsx_pidBandwidth[i];
      pidProfile->dterm_cutoff[i] = cmsx_pidDCutoff[i];
      pidProfile->bterm_cutoff[i] = cmsx_pidBCutoff[i];
    }

    cmsx_updateCurrentPidProfile();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileCtrlBWEntries[] = {
    { "- PID BW -",  OME_Label, NULL, pidProfileIndexString },
    { "BANDWIDTH R", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidBandwidth[0],     0,    250,   1  }    },
    { "BANDWIDTH P", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidBandwidth[1],     0,    250,   1  }    },
    { "BANDWIDTH Y", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidBandwidth[2],     0,    250,   1  }    },
    { "D CUTOFF R",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidDCutoff[0],       0,    250,   1  }    },
    { "D CUTOFf P",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidDCutoff[1],       0,    250,   1  }    },
    { "D CUTOFF Y",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidDCutoff[2],       0,    250,   1  }    },
    { "B CUTOFF R",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidBCutoff[0],       0,    250,   1  }    },
    { "B CUTOFf P",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidBCutoff[1],       0,    250,   1  }    },
    { "B CUTOFF Y",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_pidBCutoff[2],       0,    250,   1  }    },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileCtrlBw = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFCBW",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileCtrlBwOnEnter,
    .onExit = cmsx_profileCtrlBwOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileCtrlBWEntries,
};



/////////////////// Level Modes Profile menu items ///////////////////////

static uint8_t  cmsx_angleStrength;
static uint8_t  cmsx_angleLimit;
static uint8_t  cmsx_horizonStrength;
static uint8_t  cmsx_horizonTransition;

static const void *cmsx_profileLevelOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_angleStrength =     pidProfile->angle.level_strength;
    cmsx_angleLimit =        pidProfile->angle.level_limit;
    cmsx_horizonStrength =   pidProfile->horizon.level_strength;
    cmsx_horizonTransition = pidProfile->horizon.transition;

    return NULL;
}

static const void *cmsx_profileLevelOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile->angle.level_strength =   cmsx_angleStrength;
    pidProfile->angle.level_limit =      cmsx_angleLimit;
    pidProfile->horizon.level_strength = cmsx_horizonStrength;
    pidProfile->horizon.transition =     cmsx_horizonTransition;

    cmsx_updateCurrentPidProfile();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileLevelEntries[] = {
    { "-- LEVEL --", OME_Label, NULL, pidProfileIndexString },
    { "ANGLE STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleStrength,          0,    200,   1  }    },
    { "ANGLE LIMIT", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleLimit,            10,     90,   1  }    },
    { "HORZN STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonStrength,        0,    200,   1  }    },
    { "HORZN TRS",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonTransition,      0,    200,   1  }    },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileLevel = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFLEVEL",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileLevelOnEnter,
    .onExit = cmsx_profileLevelOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileLevelEntries,
};

/////////////////// Rescue Profile menu items ///////////////////////

static uint16_t cmsx_rescuePullupCollective;
static uint8_t  cmsx_rescuePullupTime;
static uint16_t cmsx_rescueClimbCollective;
static uint8_t  cmsx_rescueClimbTime;
static uint16_t cmsx_rescueHoverCollective;
static uint8_t  cmsx_rescueLevelGain;
static uint8_t  cmsx_rescueFlipGain;

static const void *cmsx_profileRescueOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_rescuePullupCollective = pidProfile->rescue.pull_up_collective;
    cmsx_rescuePullupTime =       pidProfile->rescue.pull_up_time;
    cmsx_rescueClimbCollective =  pidProfile->rescue.climb_collective;
    cmsx_rescueClimbTime =        pidProfile->rescue.climb_time;
    cmsx_rescueHoverCollective =  pidProfile->rescue.hover_collective;
    cmsx_rescueLevelGain =        pidProfile->rescue.level_gain;
    cmsx_rescueFlipGain =         pidProfile->rescue.flip_gain;

    return NULL;
}

static const void *cmsx_profileRescueOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile->rescue.pull_up_collective = cmsx_rescuePullupCollective;
    pidProfile->rescue.pull_up_time =       cmsx_rescuePullupTime;
    pidProfile->rescue.climb_collective =   cmsx_rescueClimbCollective;
    pidProfile->rescue.climb_time =         cmsx_rescueClimbTime;
    pidProfile->rescue.hover_collective =   cmsx_rescueHoverCollective;
    pidProfile->rescue.level_gain =         cmsx_rescueLevelGain;
    pidProfile->rescue.flip_gain =          cmsx_rescueFlipGain;

    cmsx_updateCurrentPidProfile();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileRescueEntries[] = {
    { "-- RESCUE --", OME_Label,  NULL, pidProfileIndexString },
    { "PU COLL",      OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_rescuePullupCollective, 0,    1000,  10  }    },
    { "PU TIME",      OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_rescuePullupTime,       0,     250,   1  }    },
    { "CL COLL",      OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_rescueClimbCollective,  0,    1000,  10  }    },
    { "CL TIME",      OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_rescueClimbTime,        0,     250,   1  }    },
    { "HO COLL",      OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_rescueHoverCollective,  0,    1000,  10  }    },
    { "LVL GAIN",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_rescueLevelGain,        5,     250,   1  }    },
    { "FLP GAIN",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_rescueFlipGain,         5,     250,   1  }    },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileRescue = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPRORESC",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileRescueOnEnter,
    .onExit = cmsx_profileRescueOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileRescueEntries,
};

/////////////////// Governor Profile menu items ///////////////////////

static uint16_t  cmsx_govHeadspeed;
static uint8_t   cmsx_govMasterGain;
static uint8_t  cmsx_govP;
static uint8_t  cmsx_govI;
static uint8_t  cmsx_govD;
static uint8_t  cmsx_govF;

static const void *cmsx_profileGovernorOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_govHeadspeed =     pidProfile->governor.headspeed;
    cmsx_govMasterGain =    pidProfile->governor.gain;
    cmsx_govP =             pidProfile->governor.p_gain;
    cmsx_govI =             pidProfile->governor.i_gain;
    cmsx_govD =             pidProfile->governor.d_gain;
    cmsx_govF =             pidProfile->governor.f_gain;

    return NULL;
}

static const void *cmsx_profileGovernorOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile->governor.headspeed = cmsx_govHeadspeed;
    pidProfile->governor.gain =      cmsx_govMasterGain;
    pidProfile->governor.p_gain =    cmsx_govP;
    pidProfile->governor.i_gain =    cmsx_govI;
    pidProfile->governor.d_gain =    cmsx_govD;
    pidProfile->governor.f_gain =    cmsx_govF;

    cmsx_updateCurrentPidProfile();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileGovernorEntries[] = {
    { "-- GOV --", OME_Label,  NULL, pidProfileIndexString },
    { "HEAD SPD",  OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_govHeadspeed,  0, 50000, 1  }    },
    { "GAIN",      OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_govMasterGain, 0,   250, 1  }    },
    { "P",         OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_govP,          0,   250, 1  }    },
    { "I",         OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_govI,          0,   250, 1  }    },
    { "D",         OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_govD,          0,   250, 1  }    },
    { "FF",        OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_govF,          0,   250, 1  }    },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileGovernor = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFGOV",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileGovernorOnEnter,
    .onExit = cmsx_profileGovernorOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileGovernorEntries,
};


static uint16_t gyroConfig_gyro_lpf1_static_hz;
static uint16_t gyroConfig_gyro_lpf2_static_hz;
static uint16_t gyroConfig_gyro_soft_notch_hz_1;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_1;
static uint16_t gyroConfig_gyro_soft_notch_hz_2;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_2;
static uint8_t  gyroConfig_gyro_to_use;

static const void *cmsx_menuGyro_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gyroConfig_gyro_lpf1_static_hz =  gyroConfig()->gyro_lpf1_static_hz;
    gyroConfig_gyro_lpf2_static_hz =  gyroConfig()->gyro_lpf2_static_hz;
    gyroConfig_gyro_soft_notch_hz_1 = gyroConfig()->gyro_soft_notch_hz_1;
    gyroConfig_gyro_soft_notch_cutoff_1 = gyroConfig()->gyro_soft_notch_cutoff_1;
    gyroConfig_gyro_soft_notch_hz_2 = gyroConfig()->gyro_soft_notch_hz_2;
    gyroConfig_gyro_soft_notch_cutoff_2 = gyroConfig()->gyro_soft_notch_cutoff_2;
    gyroConfig_gyro_to_use = gyroConfig()->gyro_to_use;

    return NULL;
}

static const void *cmsx_menuGyro_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gyroConfigMutable()->gyro_lpf1_static_hz =  gyroConfig_gyro_lpf1_static_hz;
    gyroConfigMutable()->gyro_lpf2_static_hz =  gyroConfig_gyro_lpf2_static_hz;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = gyroConfig_gyro_soft_notch_hz_1;
    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = gyroConfig_gyro_soft_notch_cutoff_1;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = gyroConfig_gyro_soft_notch_hz_2;
    gyroConfigMutable()->gyro_soft_notch_cutoff_2 = gyroConfig_gyro_soft_notch_cutoff_2;
    gyroConfigMutable()->gyro_to_use = gyroConfig_gyro_to_use;

    return NULL;
}

static const OSD_Entry cmsx_menuFilterGlobalEntries[] =
{
    { "-- FILTER GLB  --", OME_Label, NULL, NULL },

    { "GYRO LPF1",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lpf1_static_hz, 0, LPF_MAX_HZ, 1 } },
#ifdef USE_GYRO_LPF2
    { "GYRO LPF2",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lpf2_static_hz,  0, LPF_MAX_HZ, 1 } },
#endif
    { "GYRO NF1",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_1,     0, 500, 1 } },
    { "GYRO NF1C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_1, 0, 500, 1 } },
    { "GYRO NF2",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_2,     0, 500, 1 } },
    { "GYRO NF2C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_2, 0, 500, 1 } },
#ifdef USE_MULTI_GYRO
    { "GYRO TO USE",  OME_TAB | REBOOT_REQUIRED,  NULL, &(OSD_TAB_t)    { &gyroConfig_gyro_to_use,  2, osdTableGyroToUse} },
#endif

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuFilterGlobal = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTGLB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuGyro_onEnter,
    .onExit = cmsx_menuGyro_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterGlobalEntries,
};

#if (defined(USE_DYN_NOTCH_FILTER) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)

#ifdef USE_DYN_NOTCH_FILTER
static uint16_t dynFiltNotchMaxHz;
static uint8_t  dynFiltNotchCount;
static uint16_t dynFiltNotchQ;
static uint16_t dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
static uint16_t gyroLpfDynMin;
static uint16_t gyroLpfDynMax;
#endif

static const void *cmsx_menuDynFilt_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

#ifdef USE_DYN_NOTCH_FILTER
    dynFiltNotchCount   = dynNotchConfig()->dyn_notch_count;
    dynFiltNotchQ       = dynNotchConfig()->dyn_notch_q;
    dynFiltNotchMinHz   = dynNotchConfig()->dyn_notch_min_hz;
    dynFiltNotchMaxHz   = dynNotchConfig()->dyn_notch_max_hz;
#endif
#ifdef USE_DYN_LPF
    gyroLpfDynMin       = gyroConfig()->gyro_lpf1_dyn_min_hz;
    gyroLpfDynMax       = gyroConfig()->gyro_lpf1_dyn_max_hz;
#endif

    return NULL;
}

static const void *cmsx_menuDynFilt_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

#ifdef USE_DYN_NOTCH_FILTER
    dynNotchConfigMutable()->dyn_notch_count         = dynFiltNotchCount;
    dynNotchConfigMutable()->dyn_notch_q             = dynFiltNotchQ;
    dynNotchConfigMutable()->dyn_notch_min_hz        = dynFiltNotchMinHz;
    dynNotchConfigMutable()->dyn_notch_max_hz        = dynFiltNotchMaxHz;
#endif
#ifdef USE_DYN_LPF
    gyroConfigMutable()->gyro_lpf1_dyn_min_hz        = gyroLpfDynMin;
    gyroConfigMutable()->gyro_lpf1_dyn_max_hz        = gyroLpfDynMax;
#endif

    return NULL;
}

static const OSD_Entry cmsx_menuDynFiltEntries[] =
{
    { "-- DYN FILT --", OME_Label, NULL, NULL },

#ifdef USE_DYN_NOTCH_FILTER
    { "NOTCH COUNT",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &dynFiltNotchCount,   0, DYN_NOTCH_COUNT_MAX, 1 } },
    { "NOTCH Q",        OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchQ,       10, 100, 1 } },
    { "NOTCH MIN HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMinHz,   60, 250, 1 } },
    { "NOTCH MAX HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMaxHz,   200, 1000, 1 } },
#endif

#ifdef USE_DYN_LPF
    { "GYRO DLPF MIN",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroLpfDynMin,  0, 1000, 1 } },
    { "GYRO DLPF MAX",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroLpfDynMax,  0, 1000, 1 } },
#endif

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuDynFilt = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XDYNFLT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuDynFilt_onEnter,
    .onExit = cmsx_menuDynFilt_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuDynFiltEntries,
};

#endif

#if 0
static const void *cmsx_FilterPerProfileRead(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    return NULL;
}

static const void *cmsx_FilterPerProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    return NULL;
}

static const OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    { "-- FILTER PP  --", OME_Label, NULL, NULL },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuFilterPerProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTPP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_FilterPerProfileRead,
    .onExit = cmsx_FilterPerProfileWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterPerProfileEntries,
};
#endif

#ifdef USE_EXTENDED_CMS_MENUS

static uint8_t cmsx_dstPidProfile;
static uint8_t cmsx_dstControlRateProfile;

static const char * const cmsx_ProfileNames[] = {
    "-",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6"
};

static OSD_TAB_t cmsx_PidProfileTable = { &cmsx_dstPidProfile, PID_PROFILE_COUNT, cmsx_ProfileNames };
static OSD_TAB_t cmsx_ControlRateProfileTable = { &cmsx_dstControlRateProfile, CONTROL_RATE_PROFILE_COUNT, cmsx_ProfileNames };

static const void *cmsx_menuCopyProfile_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_dstPidProfile = 0;
    cmsx_dstControlRateProfile = 0;

    return NULL;
}

static const void *cmsx_CopyPidProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstPidProfile > 0) {
        pidCopyProfile(cmsx_dstPidProfile - 1, pidProfileIndex);
    }

    return NULL;
}

static const void *cmsx_CopyControlRateProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstControlRateProfile > 0) {
        copyControlRateProfile(cmsx_dstControlRateProfile - 1, rateProfileIndex);
    }

    return NULL;
}

static const OSD_Entry cmsx_menuCopyProfileEntries[] =
{
    { "-- COPY PROFILE --", OME_Label, NULL, NULL},

    { "CPY PID PROF TO",   OME_TAB,      NULL,                        &cmsx_PidProfileTable },
    { "COPY PP",           OME_Funcall,  cmsx_CopyPidProfile,         NULL },
    { "CPY RATE PROF TO",  OME_TAB,      NULL,                        &cmsx_ControlRateProfileTable },
    { "COPY RP",           OME_Funcall,  cmsx_CopyControlRateProfile, NULL },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuCopyProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XCPY",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuCopyProfile_onEnter,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuCopyProfileEntries,
};

#endif

static const OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- PROFILE --", OME_Label, NULL, NULL},

    {"PID PROF",  OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpPidProfileIndex, 1, PID_PROFILE_COUNT, 1}},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid},
    {"PID CTRL",  OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileCtrl},
    {"PID BW",    OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileCtrlBw},
    {"TAIL",      OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileYaw},
    {"LEVEL",     OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileLevel},
    {"RESCUE",    OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileRescue},
    {"GOV",       OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileGovernor},
    //{"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile},

    {"RATE PROF", OME_UINT8,   cmsx_rateProfileIndexOnChange, &(OSD_UINT8_t){ &tmpRateProfileIndex, 1, CONTROL_RATE_PROFILE_COUNT, 1}},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile},

    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal},
#if  (defined(USE_DYN_NOTCH_FILTER) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)
    {"DYN FILT",  OME_Submenu, cmsMenuChange,                 &cmsx_menuDynFilt},
#endif

#ifdef USE_EXTENDED_CMS_MENUS
    {"COPY PROF", OME_Submenu, cmsMenuChange,                 &cmsx_menuCopyProfile},
#endif /* USE_EXTENDED_CMS_MENUS */

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuImu = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XIMU",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = cmsx_menuImu_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuImuEntries,
};

#endif // CMS

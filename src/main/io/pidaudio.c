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
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/audio.h"

#include "fc/rc_modes.h"

#include "flight/pid.h"

#include "io/pidaudio.h"

static bool pidAudioEnabled = false;

static pidAudioModes_e pidAudioMode = PID_AUDIO_PIDSUM_XY;

void pidAudioInit(void)
{
    audioSetupIO();
}

void pidAudioStart(void)
{
    audioGenerateWhiteNoise();
}

void pidAudioStop(void)
{
    audioSilence();
}

pidAudioModes_e pidAudioGetMode(void)
{
    return pidAudioMode;
}

void pidAudioSetMode(pidAudioModes_e mode)
{
    pidAudioMode = mode;
}

void FAST_CODE_NOINLINE pidAudioUpdate(void)
{
    bool newState = IS_RC_MODE_ACTIVE(BOXPIDAUDIO);

    if (pidAudioEnabled != newState) {
        if (newState) {
            pidAudioStart();
        } else {
            pidAudioStop();
        }
        pidAudioEnabled = newState;
    }

    if (!pidAudioEnabled) {
        return;
    }

    uint8_t tone = TONE_MID;

    switch (pidAudioMode) {
    case PID_AUDIO_PIDSUM_X:
        {
            const float pidSumX = constrainf(fabsf(getPidSum(FD_ROLL)), 0, 1);
            tone = scaleRange(pidSumX, 0, 1, TONE_MAX, TONE_MIN);
            break;
        }
    case PID_AUDIO_PIDSUM_Y:
        {
            const float pidSumY = constrainf(fabsf(getPidSum(FD_PITCH)), 0, 1);
            tone = scaleRange(pidSumY, 0, 1, TONE_MAX, TONE_MIN);
            break;
        }
    case PID_AUDIO_PIDSUM_XY:
        {
            const uint32_t pidSumXY = constrainf(fabsf(getPidSum(FD_ROLL)) + fabsf(getPidSum(FD_PITCH)), 0, 2);
            tone = scaleRange(pidSumXY, 0, 2, TONE_MAX, TONE_MIN);
            break;
        }
    default:
        break;
    }

    audioPlayTone(tone);
}

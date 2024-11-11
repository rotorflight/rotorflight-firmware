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

#include "types.h"
#include "platform.h"

#include "pg/pg_ids.h"
#include "pg/mixer.h"

#include "config/config_reset.h"


PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_GENERIC_MIXER_CONFIG, 0);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .main_rotor_dir = DIR_CW,
    .tail_rotor_mode = TAIL_MODE_VARIABLE,
    .tail_motor_idle = 0,
    .tail_center_trim = 0,
    .swash_type = SWASH_TYPE_120,
    .swash_ring = 100,
    .swash_phase = 0,
    .swash_pitch_limit = 0,
    .swash_trim = { 0, 0, 0 },
    .swash_tta_precomp = 0,
    .swash_geo_correction = 0,
    .collective_scale_beta_pos = 0,
    .collective_scale_beta_neg = 0,
);

PG_REGISTER_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules, PG_GENERIC_MIXER_RULES, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs, PG_GENERIC_MIXER_INPUTS, 0);

void pgResetFn_mixerInputs(mixerInput_t *input)
{
    for (int i = MIXER_IN_STABILIZED_ROLL; i <= MIXER_IN_STABILIZED_COLLECTIVE; i++) {
        input[i].rate =  250;
        input[i].min  = -1250;
        input[i].max  =  1250;
    }

    input[MIXER_IN_STABILIZED_THROTTLE].rate =  1000;
    input[MIXER_IN_STABILIZED_THROTTLE].min  =  0;
    input[MIXER_IN_STABILIZED_THROTTLE].max  =  1000;

    for (int i = MIXER_IN_RC_COMMAND_ROLL; i <= MIXER_IN_RC_CHANNEL_18; i++) {
        input[i].rate =  1000;
        input[i].min  = -1000;
        input[i].max  =  1000;
    }
}


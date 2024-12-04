/*
 * This file is part of Rotorflight.
 *
 * Rotorflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Rotorflight is distributed in the hope that it
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

#ifdef USE_SERIALRX_FBUS

#define MS2US(ms)   ((ms) * 1000)

bool fbusRxInit(const rxConfig_t *initialRxConfig, rxRuntimeState_t *rxRuntimeState, bool isFBUS);

#endif

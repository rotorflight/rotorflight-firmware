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

#define MSP2_GET_SMARTFUEL_CONFIG           0x4000
#define MSP2_SET_SMARTFUEL_CONFIG           0x4001

#define MSP2_GET_FBUS_SENSORS               0x5F07
#define MSP2_CLEAR_FBUS_SENSORS             0x5F08
#define MSP2_GET_FBUS_MASTER_CONFIG         0x5F09
#define MSP2_SET_FBUS_MASTER_CONFIG         0x5F0A

#define MSP2_GET_CRSF_SENSORS_STATUS        0x5F0B

// action: 0 = poll only, 1 = (re)start a scan, 2 = stop - cycles ESC
// telemetry halfDuplex/pinSwap live for the already-configured protocol and
// reports which combo (if any) produces a valid frame.
#define MSP2_ESC_SENSOR_TRIAL               0x5F0C

#define MSP2_GET_RX_INPUT_BACKUP_STATUS     0x5F0D

// In-flight rate-loop statistics (flight/tune_advisor.c) and their reset.
#define MSP2_GET_TUNE_ADVISOR               0x5F10
#define MSP2_CLEAR_TUNE_ADVISOR             0x5F11

// action: 0 = poll only, 1 = (re)start a scan, 2 = stop/cancel - cycles
// serialrx_inverted/halfDuplex/pinSwap live for the already-configured
// serialrx_provider and reports which combo (if any) produces signal.
#define MSP2_RX_SERIAL_TRIAL                0x5F12

// action: 0 = poll only, 1 = (re)start - detects the right-angle board
// alignment from a level baseline followed by a tail lift, and applies it
// (unsaved) on success.
#define MSP2_BOARD_AUTO_ALIGN               0x5F13


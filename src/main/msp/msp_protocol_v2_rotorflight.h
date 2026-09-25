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

// 0x5F0C, not 0x5F0B - that's claimed by the (separate, sibling) RX serial
// wiring auto-detect branch's MSP2_RX_SERIAL_TRIAL, cut from master
// independently. action: 0 = poll only, 1 = (re)start a scan, 2 = stop -
// cycles ESC telemetry halfDuplex/pinSwap live for the already-configured
// protocol and reports which combo (if any) produces a valid frame.
#define MSP2_ESC_SENSOR_TRIAL               0x5F0C


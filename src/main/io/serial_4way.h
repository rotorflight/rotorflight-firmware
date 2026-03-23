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
 * Author: 4712
*/
#pragma once

#include "drivers/io_types.h"
#include "io/serial_4way_impl.h"

#define imC2 0
#define imSIL_BLB 1
#define imATM_BLB 2
#define imSK 3
#define imARM_BLB 4

extern uint8_t selected_esc;

extern ioMem_t ioMem;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[4];
    uint16_t words[2];
    uint32_t dword;
} uint8_32_u;

//extern uint8_32_u DeviceInfo;

bool isMcuConnected(void);
uint8_t esc4wayInit(void);
struct serialPort_s;
void esc4wayProcess(struct serialPort_s *mspPort);
void esc4wayRelease(void);
/*
 * fwifCmd* functions implement the low-level firmware-interface (FWIF)
 * used by the 4-way serial/bootloader code. They are not part of the
 * public `esc4way*` API surface; the `fwifCmd` prefix indicates these
 * helpers perform direct device/bootloader commands (init/read/write)
 * which are called from the `esc4way` layer. Keeping this prefix makes
 * the difference between higher-level `esc4way*` and lower-level FWIF
 * functions explicit for readers and maintainers.
 */
uint8_32_u *fwifCmdDeviceInitFlash(uint8_t esc_idx);
bool fwifCmdDeviceRead(uint8_t num_bytes, uint8_t *data_buffer, uint32_t addr);
bool fwifCmdDeviceWrite(uint8_t num_bytes, const uint8_t *data_buffer, uint32_t addr);
void esc4wayDeinit(void);

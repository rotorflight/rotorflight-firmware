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

 #pragma once

#include <stdint.h>
#include <string.h>

#ifdef USE_FAST_ITM_SEND
# define ITM_SendU8(port,data)           (ITM->PORT[(port)].u8  = (data))
# define ITM_SendU16(port,data)          (ITM->PORT[(port)].u16 = (data))
# define ITM_SendU32(port,data)          (ITM->PORT[(port)].u32 = (data))
#else
# define ITM_SendU8(port,data)           __ITM_SendU8((port),(data))
# define ITM_SendU16(port,data)          __ITM_SendU16((port),(data))
# define ITM_SendU32(port,data)          __ITM_SendU32((port),(data))
#endif

uint8_t  __ITM_SendU8(const size_t port, uint8_t data);
uint16_t __ITM_SendU16(const size_t port, uint16_t data);
uint32_t __ITM_SendU32(const size_t port, uint32_t data);

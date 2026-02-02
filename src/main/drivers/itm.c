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

#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#ifdef USE_ITM_DEBUG

FAST_CODE uint8_t __ITM_SendU8(const size_t port, const uint8_t data)
{
#ifdef USE_NONBLOCKING_ITM_SEND
    if (ITM->PORT[port].u32 != 0UL)
    {
#else
    if ((ITM->TCR & BIT(0)) && (ITM->TER & BIT(port)))
    {
        while (ITM->PORT[port].u32 == 0UL) __NOP();
#endif
        ITM->PORT[port].u8 = data;
    }
    return data;
}

FAST_CODE uint16_t __ITM_SendU16(const size_t port, const uint16_t data)
{
#ifdef USE_NONBLOCKING_ITM_SEND
    if (ITM->PORT[port].u32 != 0UL)
    {
#else
    if ((ITM->TCR & BIT(0)) && (ITM->TER & BIT(port)))
    {
        while (ITM->PORT[port].u32 == 0UL) __NOP();
#endif
        ITM->PORT[port].u16 = data;
    }
    return data;
}

FAST_CODE uint32_t __ITM_SendU32(const size_t port, const uint32_t data)
{
#ifdef USE_NONBLOCKING_ITM_SEND
    if (ITM->PORT[port].u32 != 0UL)
    {
#else
    if ((ITM->TCR & BIT(0)) && (ITM->TER & BIT(port)))
    {
        while (ITM->PORT[port].u32 == 0UL) __NOP();
#endif
        ITM->PORT[port].u32 = data;
    }
    return data;
}

#endif
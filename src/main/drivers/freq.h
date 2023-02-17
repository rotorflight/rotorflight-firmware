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

#define FREQ_SENSOR_PORT_COUNT 4

struct freqConfig_s;

void freqInit(const struct freqConfig_s *freqConfig);
void freqUpdate(void);

float freqRead(uint8_t port);

uint16_t getFreqSensorRPM(uint8_t port);

bool isFreqSensorPortInitialized(uint8_t port);
bool isFreqSensorInitialized(void);


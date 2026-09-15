#pragma once

#include <stdbool.h>
#include <stdint.h>

void smartFuelInit(void);
void smartFuelUpdate(void);
void validateAndFixSmartFuelConfig(void);
bool smartFuelIsEnabled(void);
uint8_t smartFuelChargeLevel(void);

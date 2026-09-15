#pragma once

#include "pg/pg.h"

typedef struct sportMasterConfig_s {
    uint8_t pinSwap;
    uint8_t inverted;
} sportMasterConfig_t;

PG_DECLARE(sportMasterConfig_t, sportMasterConfig);

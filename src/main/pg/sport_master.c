#include "platform.h"
#include "pg/pg_ids.h"
#include "pg/sport_master.h"

#ifdef USE_SPORT_MASTER

PG_REGISTER_WITH_RESET_FN(sportMasterConfig_t, sportMasterConfig, PG_DRIVER_SPORT_MASTER_CONFIG, 1);

void pgResetFn_sportMasterConfig(sportMasterConfig_t *config)
{
    config->pinSwap = 1;
    config->inverted = 1;
}

#endif

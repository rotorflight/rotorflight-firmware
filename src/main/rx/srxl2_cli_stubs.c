// Lightweight stubs to satisfy CLI references when SRXL2 implementation isn't compiled
#include <stdint.h>

#ifdef USE_SERIALRX_SRXL2

const char *srxl2_debug_messageDM(void)
{
    return "SRXL2 not built";
}

const char *srxl2_debug_lastErrorDM(void)
{
    return "None";
}

#endif

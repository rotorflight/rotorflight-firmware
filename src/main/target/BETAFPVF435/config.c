#include "platform.h"
#include "drivers/motor.h"
#include "pg/motor.h"

void targetConfiguration(void)
{
    // The onboard Bluejay ESCs accept DShot; aircraft settings stay user-configurable.
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT300;
}

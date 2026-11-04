#include "platform.h"
#include "drivers/motor.h"
#include "pg/motor.h"
#include "pg/mixer.h"

void targetConfiguration(void)
{
    // The T15 has two independently driven, one-direction motors.
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT300;
    motorConfigMutable()->dev.useDshotTelemetry = true;
    // Stock T15 rotor magnet counts, physically counted by the owner.
    motorConfigMutable()->motorPoleCount[0] = 12;
    motorConfigMutable()->motorPoleCount[1] = 6;
    mixerConfigMutable()->tail_rotor_mode = TAIL_MODE_MOTORIZED;
}

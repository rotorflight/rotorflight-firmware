"""Check board defaults with production reset functions, without hardware access."""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def read(name):
    return (ROOT / name).read_text(encoding='utf-8')


def function(source, signature):
    start = source.index(signature + '\n{')
    return source[start:source.index('\n}', start) + 2]


def without_includes(source):
    return re.sub(r'^\s*#(?:include|pragma)[^\n]*', '', source, flags=re.M)


def check(compiler, legacy=False):
    config = read('src/main/target/BETAFPVF435/config.c')
    timers = read('src/main/target/BETAFPVF435/target.c')
    if legacy:
        config = subprocess.check_output([
            'git', 'show', 'd95bdc3303ed25879b21cbef07f1169345ae2a32:src/main/target/BETAFPVF435/config.c'
        ], cwd=ROOT, encoding='utf-8')
    prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define USE_TIMER
#define USABLE_TIMER_CHANNEL_COUNT 10
#define MAX_SUPPORTED_MOTORS 4
#define MAX_SUPPORTED_SERVOS 8
#define PG_DECLARE(...)
#define PWM_TYPE_STANDARD 0
#define PWM_TYPE_DSHOT300 1
#define TAIL_MODE_MOTORIZED 1
#define IO_TAG_NONE 0
typedef uint8_t ioTag_t;
typedef unsigned timerUsageFlag_e;
enum { TIM_USE_ANY=0, TIM_USE_MOTOR=4, TIM_USE_SERVO=8 };
enum { TMR1=1, TMR2=2, TMR3=3, TMR4=4, TMR8=8, CH1=1, CH2=2, CH3=3, CH4=4 };
enum { PA2=3, PA3=4, PA8=9, PA9=10, PA10=11, PB0=17, PB1=18, PB6=23, PC6=39, PC8=41 };
typedef struct { unsigned tim; ioTag_t tag; unsigned channel, usageFlags, dmaopt; } timerHardware_t;
#define DEF_TIM(t,ch,p,usage,output,dma,up) {t,p,ch,usage,dma}
'''
    motor_header = without_includes(read('src/main/pg/motor.h'))
    reset = function(read('src/main/pg/motor.c'), 'void pgResetFn_motorConfig(motorConfig_t *motorConfig)')
    servo_reset = function(read('src/main/pg/servos.c'), 'void pgResetFn_servoConfig(servoConfig_t *servoConfig)')
    lookup = function(read('src/main/drivers/timer_common.c'),
                      'ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index)')
    harness = r'''
static motorConfig_t motors;
static motorConfig_t *motorConfigMutable(void) { return &motors; }
static struct { int tail_rotor_mode; } mixer;
#define mixerConfigMutable() (&mixer)
typedef struct { ioTag_t ioTags[MAX_SUPPORTED_SERVOS]; } servoConfig_t;
'''
    main = r'''
int main(void) {
    const ioTag_t expectedPins[] = {PB0, PB1, PA3, PA2};
    const unsigned expectedTimers[] = {TMR3, TMR3, TMR2, TMR2};
    const unsigned expectedChannels[] = {CH3, CH4, CH4, CH3};
    const unsigned expectedDma[] = {1, 2, 7, 6};
    for (unsigned i=0; i<4; ++i) {
        assert(timerHardware[i].tag==expectedPins[i]);
        assert(timerHardware[i].tim==expectedTimers[i]);
        assert(timerHardware[i].channel==expectedChannels[i]);
        assert(timerHardware[i].dmaopt==expectedDma[i]);
    }
    memset(&motors, 0, sizeof motors);
    pgResetFn_motorConfig(&motors);
    targetConfiguration();
    for (unsigned i=0; i<4; ++i) {
        assert(motors.dev.ioTags[i]==expectedPins[i]);
        assert(motors.motorPoleCount[i]==0);
    }
    assert(!motors.dev.useDshotTelemetry);
    servoConfig_t servos;
    memset(&servos, 0xA5, sizeof servos);
    pgResetFn_servoConfig(&servos);
    for (unsigned i=0; i<MAX_SUPPORTED_SERVOS; ++i) assert(servos.ioTags[i]==IO_TAG_NONE);
    /* Board initialization may choose its ESC protocol, never an aircraft's motor or tail settings. */
    for (unsigned poles=0; poles<=32; poles+=2) for (unsigned tail=0; tail<2; ++tail) {
        for (unsigned i=0; i<4; ++i) motors.motorPoleCount[i]=poles;
        motors.dev.useDshotTelemetry=tail;
        mixer.tail_rotor_mode=tail;
        motorConfig_t expected=motors;
        expected.dev.motorPwmProtocol=PWM_TYPE_DSHOT300;
        targetConfiguration();
        assert(memcmp(&motors, &expected, sizeof motors)==0);
        assert(mixer.tail_rotor_mode==(int)tail);
    }
    puts("PASS board connector mapping, production motor/servo reset and aircraft-setting independence");
}
'''
    source = '\n'.join([prefix, motor_header, without_includes(timers), lookup,
                        harness, reset, servo_reset, without_includes(config), main])
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder)
        (path / 'check.c').write_text(source, encoding='utf-8')
        subprocess.run([compiler, '-std=gnu17', str(path / 'check.c'), '-o', str(path / 'check.exe')], check=True, timeout=60)
        result = subprocess.run([str(path / 'check.exe')], capture_output=True, timeout=15)
        if legacy:
            assert result.returncode != 0, 'Previous aircraft-specific defaults escaped the test'
            print('PASS negative control: previous aircraft-specific override is rejected')
        else:
            assert result.returncode == 0, result.stderr.decode(errors='replace')
            print(result.stdout.decode().strip())
    header = read('src/main/target/BETAFPVF435/target.h')
    for name, pin in [('UART1_TX_PIN', 'PA9'), ('UART1_RX_PIN', 'PA10'), ('UART6_TX_PIN', 'PC6')]:
        assert re.search(r'^#define\s+' + name + r'\s+' + pin + r'\s*$', header, re.M), name


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default='clang')
    parser.add_argument('--legacy-control', action='store_true')
    args = parser.parse_args()
    check(args.compiler)
    if args.legacy_control:
        check(args.compiler, legacy=True)

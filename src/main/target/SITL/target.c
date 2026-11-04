/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// This file is host glue: the FDM packet and the host clock are doubles, so
// mixing them with the firmware's float scale factors is intended here.
// (SITL builds with -fsingle-precision-constant, as the STM32 targets do.)
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <sched.h>
#include <time.h>
#if defined(_WIN32) || defined(__MINGW32__)
#include <pthread_time.h>
#endif

#include "common/maths.h"

// Idle time between scheduler passes (simLoopIdle()). 50 us caps the loop at
// 20 kHz and keeps the CPU load down. Hosts whose sleep granularity is coarser
// than a gyro period (WSL1: ~0.7 ms for any sleep) starve the scheduler; there
// SITL_LOOP_SLEEP_US=0 only yields the CPU instead of sleeping.
static uint32_t simLoopSleepUs = 50;

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/adc.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
const timerHardware_t timerHardware[1]; // unused

#include "drivers/accgyro/accgyro_fake.h"
#include "flight/imu.h"

#include "config/feature.h"
#include "config/config.h"
#include "scheduler/scheduler.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"

#include "dyad.h"
#include "target/SITL/udplink.h"

uint32_t SystemCoreClock;

static fdm_packet fdmPkt;
static servo_packet pwmPkt;

static struct timespec start_time;
static double simRate = 1.0;
static pthread_t tcpWorker, udpWorker;
static bool workerRunning = true;
static udpLink_t stateLink, pwmLink;
static pthread_mutex_t updateLock;
static pthread_mutex_t mainLoopLock;

// Guards startup ordering between tcpThread()'s dyad_init() and any other
// thread (e.g. main, via serTcpOpen()) that touches dyad's internal state.
static bool dyadReady = false;
static pthread_mutex_t dyadReadyLock;
static pthread_cond_t dyadReadyCond;

// dyad is not thread-safe: serializes all dyad_*() calls between tcpThread's
// dyad_update() loop and any other thread calling into dyad (e.g. the main
// thread via drivers/serial_tcp.c).
static pthread_mutex_t dyadLock;

void simDyadLock(void) {
    pthread_mutex_lock(&dyadLock);
}

void simDyadUnlock(void) {
    pthread_mutex_unlock(&dyadLock);
}

int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y);

int lockMainPID(void) {
    return pthread_mutex_trylock(&mainLoopLock);
}

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)
void sendMotorUpdate(void) {
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
}
void updateState(const fdm_packet* pkt) {
    static double last_timestamp = 0; // in seconds
    static uint64_t last_realtime = 0; // in uS
    static struct timespec last_ts; // last packet

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    const uint64_t realtime_now = micros64_real();
    if (realtime_now > last_realtime + 500*1e3) { // 500ms timeout
        last_timestamp = pkt->timestamp;
        last_realtime = realtime_now;
        sendMotorUpdate();
        return;
    }

    const double deltaSim = pkt->timestamp - last_timestamp;  // in seconds
    if (deltaSim < 0) { // don't use old packet
        return;
    }

    int16_t x,y,z;
    x = constrain(-pkt->imu_linear_acceleration_xyz[0] * ACC_SCALE, -32767, 32767);
    y = constrain(-pkt->imu_linear_acceleration_xyz[1] * ACC_SCALE, -32767, 32767);
    z = constrain(-pkt->imu_linear_acceleration_xyz[2] * ACC_SCALE, -32767, 32767);
    fakeAccSet(fakeAccDev, x, y, z);
//    printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);

    x = constrain(pkt->imu_angular_velocity_rpy[0] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    y = constrain(-pkt->imu_angular_velocity_rpy[1] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    z = constrain(-pkt->imu_angular_velocity_rpy[2] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    fakeGyroSet(fakeGyroDev, x, y, z);
//    printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);

#if !defined(USE_IMU_CALC)
#if defined(SET_IMU_FROM_EULER)
    // set from Euler
    double qw = pkt->imu_orientation_quat[0];
    double qx = pkt->imu_orientation_quat[1];
    double qy = pkt->imu_orientation_quat[2];
    double qz = pkt->imu_orientation_quat[3];
    double ysqr = qy * qy;
    double xf, yf, zf;

    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    xf = atan2(t0, t1) * RAD2DEG;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    yf = asin(t2) * RAD2DEG; // from wiki

    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    zf = atan2(t3, t4) * RAD2DEG;
    imuSetAttitudeRPY(xf, -yf, zf); // yes! pitch was inverted!!
#else
    imuSetAttitudeQuat(pkt->imu_orientation_quat[0], pkt->imu_orientation_quat[1], pkt->imu_orientation_quat[2], pkt->imu_orientation_quat[3]);
#endif
#endif

#if defined(SIMULATOR_IMU_SYNC)
    imuSetHasNewData(deltaSim*1e6);
    imuUpdateAttitude(micros());
#endif


    // The firmware clock runs at simRate = simulated time / real time. Taken
    // per packet, two packets that arrive bunched (a sender or host that stalls
    // and catches up) give a few ms of sim time over a few us of real time, and
    // the clock races hundreds of times too fast until the next packet - every
    // firmware timer with it. Measure over a window of at least 50 ms of real
    // time instead (time-weighted, so bunching averages out), and clamp.
    if (deltaSim < 0.02 && deltaSim > 0) { // simulator should run faster than 50Hz
        static double windowSim, windowReal;
        struct timespec out_ts;
        timeval_sub(&out_ts, &now_ts, &last_ts);
        windowSim += deltaSim;
        windowReal += out_ts.tv_sec + 1e-9*out_ts.tv_nsec;
        if (windowReal >= 0.05) {
            simRate = constrainf(windowSim / windowReal, 0.05f, 20.0f);
            windowSim = 0;
            windowReal = 0;
        }
    }
//    printf("simRate = %lf, millis64 = %lu, millis64_real = %lu, deltaSim = %lf\n", simRate, millis64(), millis64_real(), deltaSim*1e6);

    last_timestamp = pkt->timestamp;
    last_realtime = micros64_real();

    last_ts.tv_sec = now_ts.tv_sec;
    last_ts.tv_nsec = now_ts.tv_nsec;

    pthread_mutex_unlock(&updateLock); // can send PWM output now

#if defined(SIMULATOR_GYROPID_SYNC)
    pthread_mutex_unlock(&mainLoopLock); // can run main loop
#endif
}

static void* udpThread(void* data) {
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&stateLink, &fdmPkt, sizeof(fdm_packet), 100);
        if (n == sizeof(fdm_packet)) {
//            printf("[data]new fdm %d\n", n);
            updateState(&fdmPkt);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}

static void* tcpThread(void* data) {
    UNUSED(data);

    dyad_init();
    dyad_setTickInterval(0.2f);
    // dyad_update() runs with dyadLock held, so it must not block: with any
    // select() timeout, this loop re-takes the (unfair) mutex right after
    // releasing it and the main thread, which needs the lock for every byte
    // it sends (tcpDataOut()), is starved - an MSP reply then takes seconds.
    // Poll without blocking and sleep outside the lock instead.
    dyad_setUpdateTimeout(0);

    // Signal that dyad's internal state is ready. Other threads (e.g. the
    // main thread, via serTcpOpen()/mspSerialAllocatePorts()) call into
    // dyad_newStream()/dyad_addListener() and must not do so before
    // dyad_init() has completed here, otherwise dyad's uninitialized/garbage
    // internal state gets used, corrupting the heap.
    pthread_mutex_lock(&dyadReadyLock);
    dyadReady = true;
    pthread_cond_signal(&dyadReadyCond);
    pthread_mutex_unlock(&dyadReadyLock);

    while (workerRunning) {
        simDyadLock();
        dyad_update();
        simDyadUnlock();
        delayMicroseconds_real(500);
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}

// system
void systemInit(void) {
    int ret;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    printf("[system]Init...\n");

    SystemCoreClock = 500 * 1e6; // fake 500MHz

    const char *loopSleep = getenv("SITL_LOOP_SLEEP_US");
    if (loopSleep && *loopSleep) {
        simLoopSleepUs = (uint32_t)strtoul(loopSleep, NULL, 10);
    }
    printf("[system]main loop idle: %u us\n", (unsigned)simLoopSleepUs);

    if (pthread_mutex_init(&updateLock, NULL) != 0) {
        printf("Create updateLock error!\n");
        exit(1);
    }

    if (pthread_mutex_init(&mainLoopLock, NULL) != 0) {
        printf("Create mainLoopLock error!\n");
        exit(1);
    }

    if (pthread_mutex_init(&dyadReadyLock, NULL) != 0) {
        printf("Create dyadReadyLock error!\n");
        exit(1);
    }

    if (pthread_cond_init(&dyadReadyCond, NULL) != 0) {
        printf("Create dyadReadyCond error!\n");
        exit(1);
    }

    if (pthread_mutex_init(&dyadLock, NULL) != 0) {
        printf("Create dyadLock error!\n");
        exit(1);
    }

    ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    // Wait until tcpThread() has finished calling dyad_init() before
    // returning, since other code (e.g. serTcpOpen(), called from the main
    // thread later during init()) calls into dyad_newStream()/
    // dyad_addListener() and must not race with dyad's own initialization.
    pthread_mutex_lock(&dyadReadyLock);
    while (!dyadReady) {
        pthread_cond_wait(&dyadReadyCond, &dyadReadyLock);
    }
    pthread_mutex_unlock(&dyadReadyLock);

    ret = udpInit(&pwmLink, "127.0.0.1", 9002, false);
    printf("init PwmOut UDP link...%d\n", ret);

    ret = udpInit(&stateLink, NULL, 9003, true);
    printf("start UDP server...%d\n", ret);

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }

    // Note: task attributes (tasks[].attribute) aren't initialized until
    // tasksInitData() runs, which happens after systemInit() returns (see
    // fc/init.c's init()). So the "serial can't be slowed down" override for
    // TASK_SERIAL is applied later, in fc/tasks.c's tasksInit(), instead of
    // here via rescheduleTask() (which would dereference a NULL attribute
    // pointer this early).
}

void systemResetHard(void){
    printf("[system]Reset!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    pthread_join(udpWorker, NULL);
    exit(0);
}

void systemReset(int reason)
{
    UNUSED(reason);
    systemResetHard();
}

// drivers/adc.c is excluded from the SITL build (no real ADC hardware),
// so provide a stub for the generic caller (blackbox.c, etc.).
bool adcIsEnabled(uint8_t channel)
{
    UNUSED(channel);
    return false;
}

void timerInit(void) {
    printf("[timer]Init...\n");
}

void timerStart(void) {
}

void failureMode(failureMode_e mode) {
    printf("[failureMode]!!! %d\n", mode);
    while (1);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    printf("Failure LED flash for: [failureMode]!!! %d\n", mode);
}

// Time part
// Thanks ArduPilot
uint64_t nanos64_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec*1e9 + ts.tv_nsec) - (start_time.tv_sec*1e9 + start_time.tv_nsec);
}

uint64_t micros64_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t millis64_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

// The firmware clock integrates real time at simRate into a running total, a
// read-modify-write on shared state, and it is read from more than the main
// loop (the UDP and TCP threads). Unguarded, two callers can interleave so one
// sees a `last` newer than its own `now`; the unsigned difference wraps and
// throws the clock far ahead, stalling the scheduler on a gyro boundary that is
// suddenly in the distant future. micros64() and millis64() also used separate
// totals that drifted apart. One locked total for both (ported from Wingflight).
static pthread_mutex_t clockLock = PTHREAD_MUTEX_INITIALIZER;

static uint64_t simNanos64(void)
{
    static uint64_t last = 0;
    static uint64_t out = 0;

    pthread_mutex_lock(&clockLock);
    const uint64_t now = nanos64_real();
    // A host clock read on another core can be a few ns behind the last one
    // (seen on WSL1); the unsigned difference would wrap. Never step back.
    if (now > last) {
        out += (now - last) * simRate;
        last = now;
    }
    const uint64_t result = out;
    pthread_mutex_unlock(&clockLock);

    return result;
}

uint64_t micros64(void) {
    return simNanos64()*1e-3;
}

uint64_t millis64(void) {
    return simNanos64()*1e-6;
}

uint32_t micros(void) {
    return micros64() & 0xFFFFFFFF;
}

// No interrupt context here: the ISR-safe variant is the same clock.
timeUs_t microsISR(void) {
    return micros();
}

uint32_t millis(void) {
    return millis64() & 0xFFFFFFFF;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return clockCycles;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros;
}
uint32_t getCycleCounter(void)
{
    return (uint32_t) (micros64() & 0xFFFFFFFF);
}

void microsleep(uint32_t usec) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void delayMicroseconds(uint32_t us) {
    microsleep(us / simRate);
}

void delayMicroseconds_real(uint32_t us) {
    microsleep(us);
}

void simLoopIdle(void)
{
    if (simLoopSleepUs) {
        microsleep(simLoopSleepUs);
    } else {
        sched_yield();
    }
}

void delay(uint32_t ms) {
    uint64_t start = millis64();

    while ((millis64() - start) < ms) {
        microsleep(1000);
    }
}

// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y) {
    unsigned int s_carry = 0;
    unsigned int ns_carry = 0;
    // Perform the carry for the later subtraction by updating y.
    if (x->tv_nsec < y->tv_nsec) {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        ns_carry += 1000000000 * nsec;
        s_carry += nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive.
    result->tv_sec = x->tv_sec - y->tv_sec - s_carry;
    result->tv_nsec = x->tv_nsec - y->tv_nsec + ns_carry;

    // Return 1 if result is negative.
    return x->tv_sec < y->tv_sec;
}


// PWM part
pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

// real value to send
static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];

static motorDevice_t motorPwmDevice; // Forward

pwmOutputPort_t *pwmGetMotors(void) {
    return motors;
}

static float pwmConvertToInternal(uint8_t mode, float throttle)
{
    float value = motorConfig()->mincommand;

    if (mode == MOTOR_CONTROL_BIDIR) {
        if (throttle != 0)
            value = scaleRangef(throttle, -1, 1, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    } else {
        if (throttle > 0)
            value = scaleRangef(throttle, 0, 1, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }

    return value;
}

static void pwmDisableMotors(void)
{
    motorPwmDevice.enabled = false;
}

static bool pwmEnableMotors(void)
{
    motorPwmDevice.enabled = true;

    return true;
}

static void pwmWriteMotor(uint8_t index, uint8_t mode, float value)
{
    motorsPwm[index] = pwmConvertToInternal(mode, value) - motorConfig()->mincommand;
}

static void pwmShutdownPulsesForAllMotors(void)
{
    motorPwmDevice.enabled = false;
}

bool pwmIsMotorEnabled(uint8_t index) {
    return motors[index].enabled;
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;

    pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;
    pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;
    pwmPkt.motor_speed[1] = motorsPwm[2] / outScale;
    pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;

    // get one "fdm_packet" can only send one "servo_packet"!!
    if (pthread_mutex_trylock(&updateLock) != 0) return;
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
}

static motorDevice_t motorPwmDevice = {
    .vTable = {
        .postInit = motorPostInitNull,
        .enable = pwmEnableMotors,
        .disable = pwmDisableMotors,
        .isMotorEnabled = pwmIsMotorEnabled,
        .updateStart = motorUpdateStartNull,
        .write = pwmWriteMotor,
        .updateComplete = pwmCompleteMotorUpdate,
        .shutdown = pwmShutdownPulsesForAllMotors,
    }
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint8_t motorCount)
{
    UNUSED(motorConfig);

    if (motorCount > 4) {
        return NULL;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        motors[motorIndex].enabled = true;
    }
    motorPwmDevice.count = motorCount; // Never used, but seemingly a right thing to set it anyways.
    motorPwmDevice.initialized = true;
    motorPwmDevice.enabled = false;

    return &motorPwmDevice;
}

// ADC part
uint16_t adcGetChannel(uint8_t channel) {
    UNUSED(channel);
    return 0;
}

// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
static FILE *eepromFd = NULL;

void FLASH_Unlock(void) {
    if (eepromFd != NULL) {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");
        return;
    }

    // open or create (binary mode - text mode on Windows corrupts binary
    // data via CRLF/EOF-byte translation)
    eepromFd = fopen(EEPROM_FILENAME,"rb+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd , 0 , SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[FLASH_Unlock] loaded '%s', size = %zu / %zu\n", EEPROM_FILENAME, lSize, sizeof(eepromData));
        } else {
            fprintf(stderr, "[FLASH_Unlock] failed to load '%s'\n", EEPROM_FILENAME);
            return;
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %zu\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "wb+")) == NULL) {
            fprintf(stderr, "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return;
        }
        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
        }
    }
}

void FLASH_Lock(void) {
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s'\n", EEPROM_FILENAME);
    } else {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
}

FLASH_Status FLASH_ErasePage(uintptr_t Page_Address) {
    UNUSED(Page_Address);
//    printf("[FLASH_ErasePage]%x\n", Page_Address);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t value) {
    if ((addr >= (uintptr_t)eepromData) && (addr < (uintptr_t)ARRAYEND(eepromData))) {
        *((uint32_t*)addr) = value;
        printf("[FLASH_ProgramWord]%p = %08x\n", (void*)addr, *((uint32_t*)addr));
    } else {
            printf("[FLASH_ProgramWord]%p out of range!\n", (void*)addr);
    }
    return FLASH_COMPLETE;
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
    printf("IOConfigGPIO\n");
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    printf("spektrumBind\n");
}

void unusedPinsInit(void)
{
    printf("unusedPinsInit\n");
}

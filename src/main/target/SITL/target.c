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
#if !defined(_WIN32) && !defined(__MINGW32__)
#include <netdb.h>          // getaddrinfo(); ws2tcpip.h on Windows, via serial_tcp.h
#include <arpa/inet.h>      // inet_ntop()
#endif
#include <sched.h>
#include <signal.h>
#if defined(__linux__)
#include <execinfo.h>       // backtrace() for the fault handler
#include <unistd.h>
#endif
#include <time.h>
#if defined(_WIN32) || defined(__MINGW32__)
#include <pthread_time.h>
#endif

#include "common/maths.h"
#include "common/time.h"

// Idle time between scheduler passes (simLoopIdle()). 50 us caps the loop at
// 20 kHz and keeps the CPU load down, but only where the host can sleep that
// briefly: where a 50 us sleep takes much longer (WSL1 ~0.7 ms, cloud VMs,
// Windows), the gyro/PID loop and everything after it starve. Unless
// SITL_LOOP_SLEEP_US sets it, systemInit() measures the host's sleep and falls
// back to 0, which only yields the CPU.
#define SIM_LOOP_SLEEP_DEFAULT_US   50
static uint32_t simLoopSleepUs = SIM_LOOP_SLEEP_DEFAULT_US;

static bool simLockstep;                                // SITL_LOCKSTEP, see simNanos64()
static bool simMainLoopRunning;                         // set on the first scheduler pass
static void simLockstepAdvance(double timestamp);
static uint32_t lockstepPacketSeq;                      // FDM packets taken in lockstep

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#if defined(_WIN32) || defined(__MINGW32__)
#include <timeapi.h>    // timeBeginPeriod(); windows.h comes with winsock2.h above
#endif
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/adc.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"

// Synthetic timer table. SITL has no timers, but the motor and servo pin
// defaults (pg/motor.c, pg/servos.c) come from timerioTagGetByUsage(), and
// motorInit()/servoInit() count the non-zero pins: with no table there are no
// motors and no servos, and nothing is ever sent to the simulator.
// Four motors match servo_packet.motor_speed[4] (motorPwmDevInit() takes at
// most four), eight servos match servo_packet.servo[8].
#define SITL_TIMER_MOTOR_COUNT 4
#define SITL_TIMER_SERVO_COUNT 8
const timerHardware_t timerHardware[SITL_TIMER_MOTOR_COUNT + SITL_TIMER_SERVO_COUNT] = {
    { .tag = 0x01, .usageFlags = TIM_USE_MOTOR },
    { .tag = 0x02, .usageFlags = TIM_USE_MOTOR },
    { .tag = 0x03, .usageFlags = TIM_USE_MOTOR },
    { .tag = 0x04, .usageFlags = TIM_USE_MOTOR },
    { .tag = 0x11, .usageFlags = TIM_USE_SERVO },
    { .tag = 0x12, .usageFlags = TIM_USE_SERVO },
    { .tag = 0x13, .usageFlags = TIM_USE_SERVO },
    { .tag = 0x14, .usageFlags = TIM_USE_SERVO },
    { .tag = 0x15, .usageFlags = TIM_USE_SERVO },
    { .tag = 0x16, .usageFlags = TIM_USE_SERVO },
    { .tag = 0x17, .usageFlags = TIM_USE_SERVO },
    { .tag = 0x18, .usageFlags = TIM_USE_SERVO },
};

#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "flight/imu.h"
#include "flight/servos.h"
#include "flight/mixer.h"

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

// What arrives on the FDM port, dispatched on its length.
static union {
    fdm_packet fdm;
    baro_packet baro;
} statePkt;

STATIC_ASSERT(sizeof(baro_packet) < FDM_PACKET_SIZE_BASE, SITL_state_packets_must_differ_in_size);

#if defined(USE_FAKE_BARO)
// Last baro_packet, in real-time us (0 = never); the FDM-derived pressure waits
// BARO_PACKET_TIMEOUT_US after one.
static uint64_t lastBaroPacketUs;
#define BARO_PACKET_TIMEOUT_US 500000
#endif
static servo_packet pwmPkt;

// servoInit() sizes servoCount from the TIM_USE_SERVO entries above, the
// packet can only carry what fits: keep the two in step.
STATIC_ASSERT(SITL_TIMER_SERVO_COUNT == ARRAYLEN(pwmPkt.servo), SITL_servo_channels_must_match_servo_packet);
STATIC_ASSERT(USABLE_TIMER_CHANNEL_COUNT == ARRAYLEN(timerHardware), SITL_usable_timer_channel_count_must_match_table);

// flight/servos.c arms each servo channel through pwmOutConfig(), whose home
// (drivers/pwm_output.c) SITL does not build. Leave ccr NULL so the register
// writes are skipped; positions are read back with getServoOutput() when the
// output packet is sent.
void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
    UNUSED(timerHardware);
    UNUSED(hz);
    UNUSED(period);
    UNUSED(value);
    UNUSED(inversion);

    channel->ccr = NULL;
    channel->tim = NULL;
}

// servoInit() derives its timebase from timerClock(); any plausible value.
uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return 240000000;
}

static struct timespec start_time;
static double simRate = 1.0;
static pthread_t tcpWorker, udpWorker;
// Cleared by systemResetHard() on the main thread, polled by the UDP and TCP
// threads. Without volatile, LTO may keep it in a register in their loops:
// they never see the reset, the joins hang, and a save never ends the process.
static volatile bool workerRunning = true;

#if defined(__linux__)
// A crash would otherwise end the process without a word, which a test
// harness sees only as a closed connection. Print the signal and a backtrace
// (module+offset, for addr2line) to stderr first.
static void simFaultHandler(int sig)
{
    static const char msg[] = "\nSITL: fatal signal, backtrace:\n";
    if (write(STDERR_FILENO, msg, sizeof(msg) - 1) < 0) {
        // nothing else to do
    }
    void *frames[48];
    const int n = backtrace(frames, ARRAYLEN(frames));
    backtrace_symbols_fd(frames, n, STDERR_FILENO);
    signal(sig, SIG_DFL);
    raise(sig);
}
#endif
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

static void refreshPwmPacket(bool motorsActive);
// The optional tail of the FDM packet (see fdm_packet in target.h): only the
// fields the datagram actually contains are used.
static void updateFdmTail(const fdm_packet *pkt, int size)
{
#if defined(USE_FREQ_SENSOR)
    if (size >= FDM_PACKET_SIZE_ERPM) {
        for (int i = 0; i < 4; i++) {
            fakeFreqSet(i, pkt->motor_erpm[i] / 60.0);
        }
    }
#endif

#if defined(USE_FAKE_BARO)
    // The simulator's own static pressure, or else the ISA troposphere at the
    // FDM altitude (relative to the simulator's origin), so baro altitude
    // tracks the flight either way.
    double pressure;
    if (size >= FDM_PACKET_SIZE_BARO && isfinite(pkt->pressure) && pkt->pressure > 0) {
        pressure = pkt->pressure;
        fakeBaroSet((int32_t)lrint(pressure), 2500);
    } else if (!lastBaroPacketUs || micros64_real() - lastBaroPacketUs >= BARO_PACKET_TIMEOUT_US) {
        const double altitude = -pkt->position_xyz[2];
        pressure = 101325.0 * pow(fmax(1.0 - 2.25577e-5 * altitude, 0.01), 5.25588);
        fakeBaroSet((int32_t)lrint(pressure), 2500);
    }
#endif

    simAdcUpdate(pkt, size);
}

void updateState(const fdm_packet* pkt, int size) {
    static double last_timestamp = 0; // in seconds
    static uint64_t last_realtime = 0; // in uS
    static struct timespec last_ts; // last packet

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    simLockstepAdvance(pkt->timestamp);

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

    // The UDP thread starts before the sensors are initialised: a simulator
    // that is already sending would write through a NULL device and crash
    // the process. Drop sensor data until both fake devices exist.
    if (!fakeAccDev || !fakeGyroDev) {
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

    updateFdmTail(pkt, size);
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
    // Packets that repeat the previous timestamp count too: a sender with a
    // coarse clock (Python's time.monotonic() ticks every 15.6 ms on Windows)
    // sends many of them, and leaving out their real time ran the firmware
    // clock about twice as fast as the simulation.
    if (deltaSim < 0.02 && deltaSim >= 0) { // simulator should run faster than 50Hz
        static double windowSim, windowReal;
        struct timespec out_ts;
        timeval_sub(&out_ts, &now_ts, &last_ts);
        windowSim += deltaSim;
        windowReal += out_ts.tv_sec + out_ts.tv_nsec / 1000000000.0L;
        if (windowReal >= 0.05) {
            simRate = constrainf(windowSim / windowReal, 0.05f, 20.0f);
            windowSim = 0;
            windowReal = 0;
        }
    }
//    printf("simRate = %lf, millis64 = %lu, millis64_real = %lu, deltaSim = %lf\n", simRate, millis64(), millis64_real(), deltaSim*1e6);

    // While disarmed motorWriteAll() skips the motor device, so
    // pwmCompleteMotorUpdate() never runs; servos still move. Send from here.
    if (simLockstepActive()) {
        // The main loop replies once this step has been run (simLoopIdle()).
        __atomic_add_fetch(&lockstepPacketSeq, 1, __ATOMIC_RELEASE);
    } else if (!motorIsEnabled()) {
        refreshPwmPacket(false);
        sendMotorUpdate();
    }

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
        n = udpRecv(&stateLink, &statePkt, sizeof(statePkt), 100);
        if (n == FDM_PACKET_SIZE_BASE || n == FDM_PACKET_SIZE_ERPM || n == FDM_PACKET_SIZE_BARO ||
            n == FDM_PACKET_SIZE_MCU || n == FDM_PACKET_SIZE_RAILS || n == FDM_PACKET_SIZE_PACK) {
            memcpy(&fdmPkt, &statePkt.fdm, n);
            updateState(&fdmPkt, n);
        }
#if defined(USE_FAKE_BARO)
        else if (n == sizeof(baro_packet)) {
            const baro_packet *b = &statePkt.baro;
            if (isfinite(b->pressure_pa) && isfinite(b->temperature_c) && b->pressure_pa > 0) {
                lastBaroPacketUs = micros64_real();
                fakeBaroSet((int32_t)lrint(b->pressure_pa), (int32_t)lrint(b->temperature_c * 100.0));
            }
        }
#endif
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
        delayMicroseconds_real(1000);   // a real sleep on every host, see microsleep()
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}

// system
// A decimal UDP port from the environment, or the default when unset.
static int simEnvPort(const char *name, int def)
{
    const char *v = getenv(name);
    if (!v) {
        return def;
    }
    char *end;
    const long port = strtol(v, &end, 10);
    if (!*v || *end || port < 1 || port > 65535) {
        fprintf(stderr, "SITL: %s='%s' is not a port number 1..65535\n", name, v);
        exit(1);
    }
    return (int)port;
}

// host[:port] from the environment, the host resolved to a dotted IPv4
// address (udpInit() takes inet_addr() input), or the defaults when unset.
static void simEnvEndpoint(const char *name, const char *defHost, int defPort, char *host, size_t hostSize, int *port)
{
    const char *v = getenv(name);
    if (!v) {
        snprintf(host, hostSize, "%s", defHost);
        *port = defPort;
        return;
    }

    char name_[128];
    snprintf(name_, sizeof(name_), "%s", v);
    char *colon = strrchr(name_, ':');
    *port = defPort;
    if (colon) {
        *colon = 0;
        char *end;
        const long p = strtol(colon + 1, &end, 10);
        if (!colon[1] || *end || p < 1 || p > 65535) {
            fprintf(stderr, "SITL: %s='%s': the port is not a number 1..65535\n", name, v);
            exit(1);
        }
        *port = (int)p;
    }
    if (!name_[0]) {
        fprintf(stderr, "SITL: %s='%s': no host\n", name, v);
        exit(1);
    }

    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(name_, NULL, &hints, &res) != 0 || !res) {
        fprintf(stderr, "SITL: %s='%s': cannot resolve '%s' to an IPv4 address\n", name, v, name_);
        exit(1);
    }
    inet_ntop(AF_INET, &((struct sockaddr_in *)res->ai_addr)->sin_addr, host, hostSize);
    freeaddrinfo(res);
}


void systemInit(void) {
    int ret;

#if defined(__linux__)
    signal(SIGSEGV, simFaultHandler);
    signal(SIGBUS, simFaultHandler);
    signal(SIGFPE, simFaultHandler);
    signal(SIGILL, simFaultHandler);
    signal(SIGABRT, simFaultHandler);
#endif

    // Line-buffered, so a log redirected to a file is complete up to the
    // moment the process is killed. The Windows CRT takes _IOLBF as full
    // buffering (a 4 KiB block): unbuffered there.
#if defined(_WIN32) || defined(__MINGW32__)
    setvbuf(stdout, NULL, _IONBF, 0);
#else
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    printf("[system]Init...\n");

    SystemCoreClock = 500 * 1e6; // fake 500MHz

#if defined(_WIN32) || defined(__MINGW32__)
    timeBeginPeriod(1);   // 1 ms sleep granularity instead of 15.6 ms
#endif

    const char *lockstep = getenv("SITL_LOCKSTEP");
    simLockstep = lockstep && *lockstep && strcmp(lockstep, "0") != 0;
    if (simLockstep) {
        printf("[system]lockstep: the firmware clock follows the FDM timestamps\n");
    }

    const char *loopSleep = getenv("SITL_LOOP_SLEEP_US");
    if (loopSleep && *loopSleep) {
        simLoopSleepUs = (uint32_t)strtoul(loopSleep, NULL, 10);
        printf("[system]main loop idle: %u us (SITL_LOOP_SLEEP_US)\n", (unsigned)simLoopSleepUs);
    } else {
#if defined(_WIN32) || defined(__MINGW32__)
        // Windows cannot sleep for less than a timer tick, and a short
        // nanosleep() returns at once, so there is nothing to measure: yield.
        simLoopSleepUs = 0;
        printf("[system]main loop idle: 0 us (Windows)\n");
#else
        // How long does a short sleep really take here? Median of 9.
        uint64_t took[9];
        for (unsigned i = 0; i < ARRAYLEN(took); i++) {
            const uint64_t t0 = nanos64_real();
            const struct timespec ts = { 0, SIM_LOOP_SLEEP_DEFAULT_US * 1000 };
            nanosleep(&ts, NULL);
            took[i] = nanos64_real() - t0;
        }
        for (unsigned i = 1; i < ARRAYLEN(took); i++) {     // insertion sort
            for (unsigned j = i; j > 0 && took[j - 1] > took[j]; j--) {
                const uint64_t t = took[j]; took[j] = took[j - 1]; took[j - 1] = t;
            }
        }
        const unsigned medianUs = (unsigned)(took[ARRAYLEN(took) / 2] / 1000);
        // The scheduler polls for the gyro boundary from 100 us out (scheduler.h):
        // an idle much longer than 50 us overshoots that window and gyro cycles
        // are skipped (a 107 us sleep on the GitHub runner starved the loop).
        simLoopSleepUs = (medianUs * 2 <= 3 * SIM_LOOP_SLEEP_DEFAULT_US) ? SIM_LOOP_SLEEP_DEFAULT_US : 0;
        printf("[system]main loop idle: %u us (a %u us sleep takes %u us here)\n",
               (unsigned)simLoopSleepUs, SIM_LOOP_SLEEP_DEFAULT_US, medianUs);
#endif
    }

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

    // Endpoints: SITL_FDM_PORT (default 9003, all interfaces) and
    // SITL_OUTPUT_ADDR as host[:port] (default 127.0.0.1:9002). Read once;
    // an invalid value ends the process rather than falling back silently.
    const int fdmPort = simEnvPort("SITL_FDM_PORT", 9003);
    ret = udpInit(&stateLink, NULL, fdmPort, true);
    printf("start UDP server on port %d...%d\n", fdmPort, ret);
    if (ret != 0) {
        fprintf(stderr, "SITL: cannot bind the FDM port %d\n", fdmPort);
        exit(1);
    }

    char outHost[64];
    int outPort;
    simEnvEndpoint("SITL_OUTPUT_ADDR", "127.0.0.1", 9002, outHost, sizeof(outHost), &outPort);
    ret = udpInit(&pwmLink, outHost, outPort, false);
    printf("init PwmOut UDP link to %s:%d...%d\n", outHost, outPort, ret);
    if (ret != 0) {
        fprintf(stderr, "SITL: cannot open the output link to %s:%d\n", outHost, outPort);
        exit(1);
    }

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }

#ifdef USE_RTC_TIME
    // No RTC chip, but the host has a clock. A real FC waits for GPS or the
    // Configurator to set its time; without one, every blackbox log would be
    // stamped 0000-01-01 and they could not be told apart.
    struct timespec wallClock;
    clock_gettime(CLOCK_REALTIME, &wallClock);
    rtcTime_t rtcNow = rtcTimeMake(wallClock.tv_sec, wallClock.tv_nsec / 1000000);
    rtcSet(&rtcNow);
#endif

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
// Integer arithmetic only: with -fsingle-precision-constant, `tv_sec*1e9`
// is computed in float, whose step at the host's uptime (CLOCK_MONOTONIC
// counts from boot) is tens of milliseconds - the whole SITL then runs in
// 67 ms clock steps on a Windows host that has been up for days.
uint64_t nanos64_real(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec - start_time.tv_sec) * 1000000000ull + (uint64_t)ts.tv_nsec - (uint64_t)start_time.tv_nsec;
}

uint64_t micros64_real(void) {
    return nanos64_real() / 1000;
}

uint64_t millis64_real(void) {
    return nanos64_real() / 1000000;
}

// The firmware clock integrates real time at simRate into a running total, a
// read-modify-write on shared state, and it is read from more than the main
// loop (the UDP and TCP threads). Unguarded, two callers can interleave so one
// sees a `last` newer than its own `now`; the unsigned difference wraps and
// throws the clock far ahead, stalling the scheduler on a gyro boundary that is
// suddenly in the distant future. micros64() and millis64() also used separate
// totals that drifted apart. One locked total for both (ported from Wingflight).
static pthread_mutex_t clockLock = PTHREAD_MUTEX_INITIALIZER;

// Lockstep (SITL_LOCKSTEP=1): once FDM packets arrive, the firmware's time is
// the simulator's - the packet timestamps - instead of real time scaled by
// simRate. Each FDM packet already releases exactly one output packet (see
// updateLock), so a simulator that steps its physics, sends the state and
// waits for the outputs gets one control step per physics step, at any speed
// and independent of host load. Until the first packet the clock runs in real
// time (boot, MSP); without packets it stands still.
static bool lockstepStarted = false;
static double lockstepTs0;
static uint64_t lockstepBaseNs;
static uint64_t simClockNs = 0;     // the running total of the real-time clock
static uint64_t simClockLast = 0;

static uint64_t simNanos64(void)
{
    // In lockstep the main loop spins on the clock while it waits for the
    // next gyro boundary, which only the next FDM packet brings. Read it
    // without the mutex there: an unfair mutex taken in a tight loop starves
    // the UDP thread that has to advance the clock.
    if (__atomic_load_n(&lockstepStarted, __ATOMIC_ACQUIRE)) {
        return __atomic_load_n(&simClockNs, __ATOMIC_ACQUIRE);
    }

    pthread_mutex_lock(&clockLock);
    uint64_t result;
    if (lockstepStarted) {
        result = simClockNs;
    } else {
        const uint64_t now = nanos64_real();
        // A host clock read on another core can be a few ns behind the last one
        // (seen on WSL1); the unsigned difference would wrap. Never step back.
        if (now > simClockLast) {
            // Before a lockstep hand-over the FDM rate means nothing: the sender
            // waits for replies that only start after the hand-over, so its
            // timestamps crawl and simRate would sink to its floor - slowing init
            // twentyfold, so a slow start (an antivirus scan of a new executable)
            // could keep calibration from finishing for a whole run. Host time.
            simClockNs += (now - simClockLast) * (simLockstep ? 1.0 : simRate);
            simClockLast = now;
        }
        result = simClockNs;
    }
    pthread_mutex_unlock(&clockLock);

    return result;
}

bool simLockstepActive(void)
{
    return __atomic_load_n(&lockstepStarted, __ATOMIC_ACQUIRE);
}

// Lockstep: advance the firmware clock to an FDM packet's timestamp.
static void simLockstepAdvance(double timestamp)
{
    if (!simLockstep) {
        return;
    }
    if (!__atomic_load_n(&lockstepStarted, __ATOMIC_ACQUIRE)) {
        // Hand over only once the main loop runs: init() still delay()s, and a
        // clock frozen until the next packet would never let it finish.
        if (!__atomic_load_n(&simMainLoopRunning, __ATOMIC_ACQUIRE)) {
            return;
        }
        simNanos64();   // bring the real-time total up to date for the hand-over
        pthread_mutex_lock(&clockLock);
        lockstepTs0 = timestamp;
        lockstepBaseNs = simClockNs;
        __atomic_store_n(&lockstepStarted, true, __ATOMIC_RELEASE);
        pthread_mutex_unlock(&clockLock);
        printf("[system]lockstep: hand-over at FDM t=%.4f s, firmware clock %.4f s\n",
               timestamp, lockstepBaseNs / 1e9);
    }
    // Only this (UDP) thread writes the clock from here on.
    const double elapsed = timestamp - lockstepTs0;
    if (elapsed >= 0) {
        const uint64_t ns = lockstepBaseNs + (uint64_t)(elapsed * 1e9);
        if (ns > __atomic_load_n(&simClockNs, __ATOMIC_RELAXED)) {
            __atomic_store_n(&simClockNs, ns, __ATOMIC_RELEASE);
        }
    }
}

uint64_t micros64(void) {
    return simNanos64() / 1000;
}

uint64_t millis64(void) {
    return simNanos64() / 1000000;
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
#if defined(_WIN32) || defined(__MINGW32__)
    // Windows cannot sleep for less than a timer tick (1 ms after
    // timeBeginPeriod(1), else 15.6 ms). The firmware's short delays would
    // each stall the main loop for that long: wait them out instead.
    if (usec < 1000) {
        const uint64_t until = nanos64_real() + (uint64_t)usec * 1000;
        while (nanos64_real() < until) {
            sched_yield();
        }
        return;
    }
    // Longer sleeps: Windows 11 ignores timeBeginPeriod() for a process without a
    // visible window - as the SITL usually runs - so nanosleep() rounds up to the
    // 15.6 ms tick, and the TCP thread's 1 ms pause made every MSP round trip ~31 ms.
    // A high-resolution waitable timer (Windows 10 1803+) sleeps what was asked.
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
    static __thread HANDLE timer;
    static __thread bool timerTried;
    if (!timerTried) {
        timerTried = true;
        timer = CreateWaitableTimerExW(NULL, NULL, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    }
    if (timer) {
        LARGE_INTEGER due;
        due.QuadPart = -(LONGLONG)usec * 10;        // relative, 100 ns units
        if (SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {
            WaitForSingleObject(timer, INFINITE);
            return;
        }
    }
#endif
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000UL;
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
    __atomic_store_n(&simMainLoopRunning, true, __ATOMIC_RELEASE);
    tcpProcessRxCallbacks();

    // Lockstep reply: one output packet per FDM packet, sent after a full
    // scheduler pass that started after the packet - the pass in which the
    // gyro/PID step for the new time runs.
    if (simLockstepActive()) {
        static uint32_t seenSeq, repliedSeq;
        const uint32_t seq = __atomic_load_n(&lockstepPacketSeq, __ATOMIC_ACQUIRE);
        if (seq != repliedSeq) {
            if (seq == seenSeq) {
                refreshPwmPacket(motorIsEnabled());
                sendMotorUpdate();
                repliedSeq = seq;
            } else {
                seenSeq = seq;
            }
        }
    }

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

// Motor outputs as the mixer writes them: 0..1, or -1..1 for bidirectional motors.
// A flight model wants the commanded fraction, not a PWM pulse: mapping through
// min_throttle/max_throttle would add the analog-PWM idle offset (0.07 with the
// usual min_throttle 1070) that a DShot ESC does not have.
static float motorsOutput[MAX_SUPPORTED_MOTORS];

static motorDevice_t motorPwmDevice; // Forward

pwmOutputPort_t *pwmGetMotors(void) {
    return motors;
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
    motorsOutput[index] = (mode == MOTOR_CONTROL_BIDIR) ? constrainf(value, -1, 1) : constrainf(value, 0, 1);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    motorPwmDevice.enabled = false;
}

bool pwmIsMotorEnabled(uint8_t index) {
    return motors[index].enabled;
}

// Fill the output packet from the current motor and servo outputs.
// motor_speed[i] carries M(i+1), the same layout as Wingflight's SITL (the old
// Gazebo multicopter remap 1,2,3,0 is gone). Normal range [0, 1], 3D [-1, 1].
static void refreshPwmPacket(bool motorsActive)
{
    for (unsigned i = 0; i < ARRAYLEN(pwmPkt.motor_speed); i++) {
        pwmPkt.motor_speed[i] = motorsActive ? motorsOutput[i] : 0;
    }

    const unsigned servoCount = MIN(getServoCount(), ARRAYLEN(pwmPkt.servo));
    for (unsigned i = 0; i < ARRAYLEN(pwmPkt.servo); i++) {
        pwmPkt.servo[i] = (i < servoCount) ? getServoOutput(i) : 0;
    }

    // What the PID loop and the governor command, before the swash and tail
    // mixing: for flight models that do not want to invert the servo mixing.
    for (unsigned i = 0; i < ARRAYLEN(pwmPkt.mixer); i++) {
        pwmPkt.mixer[i] = mixerGetInput(MIXER_IN_STABILIZED_ROLL + i);
    }
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    refreshPwmPacket(true);

    if (simLockstepActive()) {
        return;     // the lockstep reply goes out from simLoopIdle()
    }

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

// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
//
// eeprom.bin is read whole on unlock and written whole on lock, through a
// temporary file renamed over the store: a process killed in the middle of a
// save (Rotorflight saves the stats 0.5 s after every disarm) then leaves the
// previous store intact instead of a torn one, which the next start would
// reject and replace with defaults. Binary mode throughout: text mode on
// Windows translates CRLF and EOF bytes.
static bool eepromUnlocked = false;

static void eepromWriteFile(void)
{
    static const char tmpName[] = EEPROM_FILENAME ".tmp";
    FILE *f = fopen(tmpName, "wb");
    if (f == NULL) {
        fprintf(stderr, "[FLASH] cannot create '%s': %s\n", tmpName, strerror(errno));
        return;
    }
    const bool written = fwrite(eepromData, sizeof(eepromData), 1, f) == 1 && fflush(f) == 0;
    fclose(f);
    if (!written) {
        fprintf(stderr, "[FLASH] write to '%s' failed: %s\n", tmpName, strerror(errno));
        remove(tmpName);
        return;
    }
#if defined(_WIN32) || defined(__MINGW32__)
    if (!MoveFileExA(tmpName, EEPROM_FILENAME, MOVEFILE_REPLACE_EXISTING)) {
#else
    if (rename(tmpName, EEPROM_FILENAME) != 0) {
#endif
        fprintf(stderr, "[FLASH] cannot replace '%s'\n", EEPROM_FILENAME);
    }
}

void FLASH_Unlock(void) {
    if (eepromUnlocked) {
        return;
    }
    eepromUnlocked = true;

    FILE *f = fopen(EEPROM_FILENAME, "rb");
    if (f != NULL) {
        fseek(f, 0, SEEK_END);
        const long size = ftell(f);
        rewind(f);
        const size_t n = fread(eepromData, 1, sizeof(eepromData), f);
        fclose(f);
        if (size == (long)sizeof(eepromData) && n == sizeof(eepromData)) {
            printf("[FLASH_Unlock] loaded '%s', size = %zu\n", EEPROM_FILENAME, n);
        } else {
            fprintf(stderr, "[FLASH_Unlock] '%s' is %ld bytes, expected %zu; rewritten on save\n",
                    EEPROM_FILENAME, size, sizeof(eepromData));
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %zu\n", EEPROM_FILENAME, sizeof(eepromData));
        eepromWriteFile();
    }
}

void FLASH_Lock(void) {
    if (eepromUnlocked) {
        eepromWriteFile();
        eepromUnlocked = false;
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

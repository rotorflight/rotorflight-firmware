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

// SITL (software in the loop) simulator

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "common/utils.h"

#define TARGET_BOARD_IDENTIFIER "SITL"

#define SIMULATOR_MULTITHREAD

// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
#undef USE_IMU_CALC

//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC

// file name to save config
#define EEPROM_FILENAME "eeprom.bin"
#define CONFIG_IN_FILE
#define EEPROM_SIZE     32768

#define U_ID_0 0
#define U_ID_1 1
#define U_ID_2 2

#undef TASK_GYROPID_DESIRED_PERIOD
#define TASK_GYROPID_DESIRED_PERIOD     100

#undef SCHEDULER_DELAY_LIMIT
#define SCHEDULER_DELAY_LIMIT           1

#define USE_FAKE_LED

#define USE_ACC
#define USE_FAKE_ACC

#define USE_GYRO
#define USE_FAKE_GYRO

#define USE_MAG
#define USE_FAKE_MAG

#define USE_BARO
#define USE_FAKE_BARO

#define USE_TARGET_CONFIG       // SITL defaults, config.c

// Onboard dataflash for blackbox, backed by a file next to eeprom.bin
// (drivers/flash_file.c). Blackbox logs to it by default, as on an FC with a
// flash chip, and the Configurator can download and erase it over MSP.
#define USE_FLASH_FILE
#define FLASH_FILE_NAME "blackbox_flash.bin"
#define USE_FLASHFS
#define USE_FLASHFS_LOOP        // rolling erase once the flash is full
#define USE_FLASH_TOOLS         // CLI flash_read / flash_write / flash_erase_sector
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// Rotor speed from the FDM packet (freq_fake.c), for the governor and the RPM
// filter, as on the F4/F7 boards.
#define USE_FREQ_SENSOR
#define USE_RPM_FILTER

// ESC sensor task at 250 Hz, as fast boards run it (STM32_UNIFIED: 100-250 Hz). The
// generic default of 50 Hz is slower than the 10 ms reply window of the XDFLY/OMPHOBBY/ZTW
// and FLYROTOR parameter dialogues: a reply arrives, but the request is resent and the
// buffer reset before the next task run can decode it.
#define ESC_SENSOR_TASK_FREQ_HZ 250

#define USE_DYN_NOTCH_FILTER

#define USABLE_TIMER_CHANNEL_COUNT 12  // 4 motors + 8 servos, see timerHardware[] in target.c

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8

//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 8

#define DEFAULT_RX_FEATURE      FEATURE_RX_MSP
#define DEFAULT_FEATURES        (FEATURE_GPS | FEATURE_TELEMETRY)

#define USE_PARAMETER_GROUPS

#undef USE_STACK_CHECK // I think SITL don't need this
#undef USE_DASHBOARD
#undef USE_TELEMETRY_LTM
// Board ADC fed from the FDM packet (adc_fake.c); the pack is measured there
// by default, so a simulator's pack voltage and current work out of the box.
#define USE_ADC
#define USE_ADC_INTERNAL
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#undef USE_VCP
#undef USE_OSD
#undef USE_PPM
#undef USE_PWM
// Serial RX with CRSF over a TCP UART, for a (simulated) radio. CRSF v3 renegotiates
// the baud rate, which a TCP port has no use for.
#undef USE_CRSF_V3
#undef USE_SERIALRX_GHST
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_IBUS2
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef USE_LED_STRIP
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_SMARTPORT
#undef USE_SPORT_MASTER
#undef USE_TELEMETRY_MAVLINK
#undef USE_RESOURCE_MGMT
#undef USE_CMS
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_SRXL
#undef USE_SERIALRX_JETIEXBUS
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_SMARTAUDIO
#undef USE_VTX_TRAMP
#undef USE_CAMERA_CONTROL
#undef USE_GPS_RESCUE
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
// ESC configuration through the 4-way interface needs real ESC signal pins.
#undef USE_BLHELI_FORWARD_PROGRAMMING
#undef USE_AM32_FORWARD_PROGRAMMING

#undef USE_I2C
#undef USE_SPI

#define TARGET_FLASH_SIZE 2048


#define LED_STRIP_TIMER 1
#define SOFTSERIAL_1_TIMER 2
#define SOFTSERIAL_2_TIMER 3

#define DEFIO_NO_PORTS   // suppress 'no pins defined' warning

// belows are internal stuff

extern uint32_t SystemCoreClock;

typedef enum
{
    Mode_TEST = 0x0,
    Mode_Out_PP = 0x10
} GPIO_Mode;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {TEST_IRQ = 0 } IRQn_Type;
typedef enum {
    EXTI_Trigger_Rising = 0x08,
    EXTI_Trigger_Falling = 0x0C,
    EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

#if defined(_WIN32) || defined(__MINGW32__)
// MinGW has no ffs() (POSIX <strings.h>), which the CLI and the arming code use.
#define ffs(x) __builtin_ffs(x)
#endif

// The ARM newlib headers define __packed; glibc and MinGW do not.
#ifndef __packed
#define __packed __attribute__((packed))
#endif

// Register bit helpers that the STM32 CMSIS device headers provide on boards.
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))

typedef struct
{
  uint32_t IDR;
  uint32_t ODR;
  uint32_t BSRR;
  uint32_t BRR;
} GPIO_TypeDef;

#define GPIOA_BASE ((intptr_t)0x0001)

typedef struct
{
    void* test;
    uint32_t CCR1; // unused: present only so common driver code referencing tim->CCR1 (e.g. timerCCR()) type-checks on SITL
} TIM_TypeDef;

typedef struct
{
    void* test;
} TIM_OCInitTypeDef;

typedef struct {
    void* test;
} DMA_TypeDef;

typedef struct {
    void* test;
} DMA_Channel_TypeDef;

typedef struct {
    void* test;
} DMA_InitTypeDef;

uint8_t DMA_GetFlagStatus(void *);
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState );
void DMA_ClearFlag(uint32_t);

typedef struct
{
    void* test;
} SPI_TypeDef;

typedef struct
{
    void* test;
} USART_TypeDef;

#define USART1 ((USART_TypeDef *)0x0001)
#define USART2 ((USART_TypeDef *)0x0002)
#define USART3 ((USART_TypeDef *)0x0003)
#define USART4 ((USART_TypeDef *)0x0004)
#define USART5 ((USART_TypeDef *)0x0005)
#define USART6 ((USART_TypeDef *)0x0006)
#define USART7 ((USART_TypeDef *)0x0007)
#define USART8 ((USART_TypeDef *)0x0008)

#define UART4 ((USART_TypeDef *)0x0004)
#define UART5 ((USART_TypeDef *)0x0005)
#define UART7 ((USART_TypeDef *)0x0007)
#define UART8 ((USART_TypeDef *)0x0008)

typedef struct
{
    void* test;
} I2C_TypeDef;

typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
} FLASH_Status;

typedef struct {
    double timestamp;                   // in seconds
    double imu_angular_velocity_rpy[3]; // rad/s -> range: +/- 8192; +/- 2000 deg/se
    double imu_linear_acceleration_xyz[3];    // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
    double imu_orientation_quat[4];     //w, x, y, z
    double velocity_xyz[3];             // m/s, earth frame
    double position_xyz[3];             // meters, NED from origin
    // Optional tail, told apart by the datagram length: each longer layout is
    // the shorter one plus fields at the end, and fields a packet does not
    // contain are not read.
    double motor_erpm[4];               // 168 bytes: electrical rpm per motor, -> frequency sensor
    double pressure;                    // 176: static pressure, Pa (else ISA from position_xyz[2])
    double mcu_vref;                    // 192: MCU supply rail, V
    double mcu_temp;                    //      MCU die temperature, degC
    double vbec;                        // 208: BEC rail, V
    double vbus;                        //      bus rail, V
    double vbat;                        // 224: pack voltage on the board ADC, V
    double ibat;                        //      pack current on the board ADC, A
} fdm_packet;

#define FDM_PACKET_SIZE_BASE    136
#define FDM_PACKET_SIZE_ERPM    168
#define FDM_PACKET_SIZE_BARO    176
#define FDM_PACKET_SIZE_MCU     192
#define FDM_PACKET_SIZE_RAILS   208
#define FDM_PACKET_SIZE_PACK    224

// Barometer reading on the FDM port, told apart from fdm_packet by its size: the
// layout of Wingflight's SITL, so a simulator written for either firmware works
// with both. While these arrive, the FDM's pressure / ISA fallback stays out of
// the way.
typedef struct {
    double timestamp;                   // in seconds, same clock as fdm_packet
    double pressure_pa;                 // absolute static pressure, Pa
    double temperature_c;               // sensor temperature, degC
} baro_packet;

void fakeFreqSet(uint8_t port, float hz);
void simAdcUpdate(const fdm_packet *pkt, int size);
typedef struct {
    float motor_speed[4];   // M1-M4 in order; normal: [0.0, 1.0], 3D: [-1.0, 1.0]
    float servo[8];         // S1-S8 pulse width in microseconds, 0 for an absent servo
    float mixer[5];         // stabilized mixer inputs: roll, pitch, yaw, collective, throttle
                            // (mixerGetInput(MIXER_IN_STABILIZED_*), dimensionless firmware units)
} servo_packet;             // 68 bytes; the first 48 (motors + servos) are the older layout

void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uintptr_t Page_Address);
FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t Data);

uint64_t nanos64_real(void);
uint64_t micros64_real(void);
uint64_t millis64_real(void);
void delayMicroseconds_real(uint32_t us);
void simLoopIdle(void);
bool simLockstepActive(void);   // SITL_LOCKSTEP and the hand-over done (target.c)
uint64_t micros64(void);
uint64_t millis64(void);

int lockMainPID(void);

// dyad (lib/main/dyad) is not thread-safe: tcpThread() drives dyad_update()
// continuously on a background thread while the main thread (via
// serTcpOpen()/tcpDataOut() in drivers/serial_tcp.c) calls other dyad_*()
// APIs that mutate the same global stream list/listener vectors. All dyad
// API calls from outside dyad's own callbacks must be serialized with these.
void simDyadLock(void);
void simDyadUnlock(void);


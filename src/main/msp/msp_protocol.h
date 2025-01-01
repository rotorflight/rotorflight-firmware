/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * The API version MUST BE incremented when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible. Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

#pragma once

#define MSP_PROTOCOL_VERSION                 0

#define API_VERSION_MAJOR                    13
#define API_VERSION_MINOR                    0
#define API_VERSION_LENGTH                   2

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH  4
#define BOARD_IDENTIFIER_LENGTH              4
#define BOARD_HARDWARE_REVISION_LENGTH       2

/*
 * MSP Command IDs
 */

#define MSP_API_VERSION                      1
#define MSP_FC_VARIANT                       2
#define MSP_FC_VERSION                       3
#define MSP_BOARD_INFO                       4
#define MSP_BUILD_INFO                       5

#define MSP_NAME                             10
#define MSP_SET_NAME                         11
#define MSP_PILOT_CONFIG                     12
#define MSP_SET_PILOT_CONFIG                 13

#define MSP_BATTERY_CONFIG                   32
#define MSP_SET_BATTERY_CONFIG               33
#define MSP_MODE_RANGES                      34
#define MSP_SET_MODE_RANGE                   35
#define MSP_FEATURE_CONFIG                   36
#define MSP_SET_FEATURE_CONFIG               37
#define MSP_BOARD_ALIGNMENT_CONFIG           38
#define MSP_SET_BOARD_ALIGNMENT_CONFIG       39
#define MSP_CURRENT_METER_CONFIG             40
#define MSP_SET_CURRENT_METER_CONFIG         41
#define MSP_MIXER_CONFIG                     42
#define MSP_SET_MIXER_CONFIG                 43
#define MSP_RX_CONFIG                        44
#define MSP_SET_RX_CONFIG                    45
#define MSP_LED_COLORS                       46
#define MSP_SET_LED_COLORS                   47
#define MSP_LED_STRIP_CONFIG                 48
#define MSP_SET_LED_STRIP_CONFIG             49
#define MSP_RSSI_CONFIG                      50
#define MSP_SET_RSSI_CONFIG                  51
#define MSP_ADJUSTMENT_RANGES                52
#define MSP_SET_ADJUSTMENT_RANGE             53
#define MSP_SERIAL_CONFIG                    54
#define MSP_SET_SERIAL_CONFIG                55
#define MSP_VOLTAGE_METER_CONFIG             56
#define MSP_SET_VOLTAGE_METER_CONFIG         57
#define MSP_SONAR_ALTITUDE                   58
#define MSP_DEBUG_CONFIG                     59
#define MSP_SET_DEBUG_CONFIG                 60
#define MSP_ARMING_CONFIG                    61
#define MSP_SET_ARMING_CONFIG                62
#define MSP_RX_MAP                           64
#define MSP_SET_RX_MAP                       65
#define MSP_RC_CONFIG                        66
#define MSP_SET_RC_CONFIG                    67

#define MSP_REBOOT                           68

#define MSP_DATAFLASH_SUMMARY                70
#define MSP_DATAFLASH_READ                   71
#define MSP_DATAFLASH_ERASE                  72
#define MSP_TELEMETRY_CONFIG                 73
#define MSP_SET_TELEMETRY_CONFIG             74
#define MSP_FAILSAFE_CONFIG                  75
#define MSP_SET_FAILSAFE_CONFIG              76
#define MSP_RXFAIL_CONFIG                    77
#define MSP_SET_RXFAIL_CONFIG                78
#define MSP_SDCARD_SUMMARY                   79
#define MSP_BLACKBOX_CONFIG                  80
#define MSP_SET_BLACKBOX_CONFIG              81
#define MSP_TRANSPONDER_CONFIG               82
#define MSP_SET_TRANSPONDER_CONFIG           83
#define MSP_OSD_CONFIG                       84
#define MSP_SET_OSD_CONFIG                   85
#define MSP_OSD_CHAR_READ                    86
#define MSP_OSD_CHAR_WRITE                   87
#define MSP_VTX_CONFIG                       88
#define MSP_SET_VTX_CONFIG                   89
#define MSP_ADVANCED_CONFIG                  90
#define MSP_SET_ADVANCED_CONFIG              91
#define MSP_FILTER_CONFIG                    92
#define MSP_SET_FILTER_CONFIG                93
#define MSP_PID_PROFILE                      94
#define MSP_SET_PID_PROFILE                  95
#define MSP_SENSOR_CONFIG                    96
#define MSP_SET_SENSOR_CONFIG                97
#define MSP_CAMERA_CONTROL                   98
#define MSP_SET_ARMING_DISABLED              99

#define MSP_STATUS                           101
#define MSP_RAW_IMU                          102
#define MSP_SERVO                            103
#define MSP_MOTOR                            104
#define MSP_RC                               105
#define MSP_RAW_GPS                          106
#define MSP_COMP_GPS                         107
#define MSP_ATTITUDE                         108
#define MSP_ALTITUDE                         109
#define MSP_ANALOG                           110
#define MSP_RC_TUNING                        111
#define MSP_PID_TUNING                       112
#define MSP_RC_COMMAND                       113
#define MSP_RX_CHANNELS                      114

#define MSP_BOXNAMES                         116

#define MSP_BOXIDS                           119
#define MSP_SERVO_CONFIGURATIONS             120
#define MSP_NAV_STATUS                       121
#define MSP_NAV_CONFIG                       122
#define MSP_ESC_SENSOR_CONFIG                123

#define MSP_SENSOR_ALIGNMENT                 126
#define MSP_LED_STRIP_MODECOLOR              127
#define MSP_VOLTAGE_METERS                   128
#define MSP_CURRENT_METERS                   129
#define MSP_BATTERY_STATE                    130
#define MSP_MOTOR_CONFIG                     131
#define MSP_GPS_CONFIG                       132

#define MSP_GPS_RESCUE                       135
#define MSP_GPS_RESCUE_PIDS                  136
#define MSP_VTXTABLE_BAND                    137
#define MSP_VTXTABLE_POWERLEVEL              138
#define MSP_MOTOR_TELEMETRY                  139

#define MSP_GOVERNOR_CONFIG                  142
#define MSP_SET_GOVERNOR_CONFIG              143
#define MSP_RPM_FILTER                       144
#define MSP_SET_RPM_FILTER                   145
#define MSP_RESCUE_PROFILE                   146
#define MSP_SET_RESCUE_PROFILE               147
#define MSP_GOVERNOR_PROFILE                 148
#define MSP_SET_GOVERNOR_PROFILE             149
#define MSP_LED_STRIP_SETTINGS               150
#define MSP_SET_LED_STRIP_SETTINGS           151
#define MSP_SBUS_OUTPUT_CONFIG               152
#define MSP_SET_SBUS_OUTPUT_CONFIG           153

#define MSP_EXPERIMENTAL                     158
#define MSP_SET_EXPERIMENTAL                 159

#define MSP_UID                              160

#define MSP_GPSSVINFO                        164
#define MSP_GPSSTATISTICS                    166

#define MSP_MIXER_INPUTS                     170
#define MSP_SET_MIXER_INPUT                  171
#define MSP_MIXER_RULES                      172
#define MSP_SET_MIXER_RULE                   173

#define MSP_OSD_VIDEO_CONFIG                 180
#define MSP_SET_OSD_VIDEO_CONFIG             181
#define MSP_DISPLAYPORT                      182
#define MSP_COPY_PROFILE                     183
#define MSP_BEEPER_CONFIG                    184
#define MSP_SET_BEEPER_CONFIG                185
#define MSP_SET_TX_INFO                      186
#define MSP_TX_INFO                          187

#define MSP_MIXER_OVERRIDE                   190
#define MSP_SET_MIXER_OVERRIDE               191
#define MSP_SERVO_OVERRIDE                   192
#define MSP_SET_SERVO_OVERRIDE               193
#define MSP_MOTOR_OVERRIDE                   194
#define MSP_SET_MOTOR_OVERRIDE               195

#define MSP_SET_RAW_RC                       200
#define MSP_SET_RAW_GPS                      201
#define MSP_SET_PID_TUNING                   202
#define MSP_SET_RC_TUNING                    204
#define MSP_ACC_CALIBRATION                  205
#define MSP_MAG_CALIBRATION                  206
#define MSP_RESET_CONF                       208

#define MSP_SELECT_SETTING                   210
#define MSP_SET_HEADING                      211
#define MSP_SET_SERVO_CONFIGURATION          212

#define MSP_SET_MOTOR                        214
#define MSP_SET_NAV_CONFIG                   215
#define MSP_SET_ESC_SENSOR_CONFIG            216

#define MSP_ESC_PARAMETERS                   217
#define MSP_SET_ESC_PARAMETERS               218

#define MSP_SET_RESET_CURR_PID               219
#define MSP_SET_SENSOR_ALIGNMENT             220
#define MSP_SET_LED_STRIP_MODECOLOR          221
#define MSP_SET_MOTOR_CONFIG                 222
#define MSP_SET_GPS_CONFIG                   223
#define MSP_SET_GPS_RESCUE                   225
#define MSP_SET_GPS_RESCUE_PIDS              226
#define MSP_SET_VTXTABLE_BAND                227
#define MSP_SET_VTXTABLE_POWERLEVEL          228

#define MSP_MULTIPLE_MSP                     230
#define MSP_MODE_RANGES_EXTRA                238
#define MSP_ACC_TRIM                         240
#define MSP_SET_ACC_TRIM                     239

#define MSP_SET_PASSTHROUGH                  245
#define MSP_SET_RTC                          246
#define MSP_RTC                              247
#define MSP_SET_BOARD_INFO                   248
#define MSP_SET_SIGNATURE                    249

#define MSP_EEPROM_WRITE                     250
#define MSP_RESERVE_1                        251
#define MSP_RESERVE_2                        252
#define MSP_DEBUGMSG                         253
#define MSP_DEBUG                            254
#define MSP_V2_FRAME                         255

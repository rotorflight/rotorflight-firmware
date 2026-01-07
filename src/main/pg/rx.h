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

#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"

#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

typedef struct rxConfig_s {
    uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;                 // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t serialrx_inverted;                 // invert the serial RX protocol compared to it's default setting
    uint8_t halfDuplex;                        // allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
    uint8_t pinSwap;                           // swap rx and tx pins around compared to the resource settings
    uint16_t rx_pulse_min;                     // Absolute minimum pulse accepted
    uint16_t rx_pulse_max;                     // Absolute maximum pulse accepted
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_invert;
    int8_t  rssi_offset;                       // offset applied to the RSSI value before it is returned
    uint8_t rssi_src_frame_errors;             // true to use frame drop flags in the rx protocol
    uint8_t rssi_src_frame_lpf_period;         // Period of the cutoff frequency for the source frame RSSI filter (in 0.1 s)
    ioTag_t spektrum_bind_pin_override_ioTag;
    ioTag_t spektrum_bind_plug_ioTag;
    uint8_t spektrum_sat_bind;                 // number of bind pulses for Spektrum satellite receivers
    uint8_t spektrum_sat_bind_autoreset;       // whenever we will reset (exit) binding mode after hard reboot
    uint8_t srxl2_unit_id;                     // Spektrum SRXL2 RX unit id
    uint8_t srxl2_baud_fast;                   // Select Spektrum SRXL2 fast baud rate
    uint8_t sbus_baud_fast;                    // Select SBus fast baud rate
    uint8_t crsf_use_rx_snr;                   // Use RX SNR (in dB) instead of RSSI dBm for CRSF
    uint8_t crsf_use_negotiated_baud;          // Use negotiated baud rate for CRSF V3
} rxConfig_t;

PG_DECLARE(rxConfig_t, rxConfig);

typedef struct rcControlsConfig_s {
    uint16_t rc_center;                 // Stick center. Usually 1500 or 1520, depending on the RC system
    uint16_t rc_deflection;             // Max stick declection in us, applies to RPYC
    uint16_t rc_min_throttle;           // Throttle channel value for 0%
    uint16_t rc_max_throttle;           // Throttle channel value for 100%
    uint8_t  rc_deadband;               // A deadband around the stick center for pitch and roll axis
    uint8_t  rc_yaw_deadband;           // A deadband around the stick center for yaw axis
    uint8_t  rc_smoothness;             // Minimum RPYC smoothing level
    uint8_t  rc_threshold[4];           // Threshold for stick activity
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

typedef struct rxFailsafeChannelConfig_s {
    uint8_t mode;           // See rxFailsafeChannelMode_e
    uint8_t step;
} rxFailsafeChannelConfig_t;

PG_DECLARE_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs);

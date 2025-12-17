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

#include "platform.h"

#if defined(USE_PWM) || defined(USE_PPM) || defined(USE_SERIAL_RX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)

#include "pg/rx.h"
#include "rx.h"

#include "config/config_reset.h"

#include "drivers/io.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "rx/rx_spi.h"

PG_REGISTER_WITH_RESET_FN(rxConfig_t, rxConfig, PG_RX_CONFIG, 3);
void pgResetFn_rxConfig(rxConfig_t *rxConfig)
{
    RESET_CONFIG_2(rxConfig_t, rxConfig,
        .serialrx_provider = SERIALRX_PROVIDER,
        .serialrx_inverted = 0,
        .halfDuplex = 0,
        .pinSwap = 0,
        .rx_pulse_min = RX_PWM_PULSE_MIN,
        .rx_pulse_max = RX_PWM_PULSE_MAX,
        .rssi_src_frame_errors = false,
        .rssi_channel = 0,
        .rssi_scale = RSSI_SCALE_DEFAULT,
        .rssi_offset = 0,
        .rssi_invert = 0,
        .rssi_src_frame_lpf_period = 30,
        .spektrum_bind_pin_override_ioTag = IO_TAG(SPEKTRUM_BIND_PIN),
        .spektrum_bind_plug_ioTag = IO_TAG(BINDPLUG_PIN),
        .spektrum_sat_bind = 0,
        .spektrum_sat_bind_autoreset = 1,
        .srxl2_unit_id = 1,
        .srxl2_baud_fast = true,
        .sbus_baud_fast = false,
        .crsf_use_rx_snr = false,
        .crsf_use_negotiated_baud = false,
    );

#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1C23", rxConfig);
#else
    parseRcChannels("AETRC123", rxConfig);
#endif
}

PG_REGISTER_WITH_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

PG_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig,
    .rc_center = 1500,
    .rc_deflection = 510,
    .rc_min_throttle = 0,
    .rc_max_throttle = 0,
    .rc_deadband = 5,
    .rc_yaw_deadband = 5,
    .rc_smoothness = 50,
    .rc_threshold = { 25, 25, 25, 50 },
);

extern void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs);

PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);

#endif

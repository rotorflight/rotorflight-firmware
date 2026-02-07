/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight, Betaflight and INAV are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define MSP2_BETAFLIGHT_BIND                0x3000
//#define MSP2_MOTOR_OUTPUT_REORDERING        0x3001
//#define MSP2_SET_MOTOR_OUTPUT_REORDERING    0x3002
#define MSP2_SEND_DSHOT_COMMAND             0x3003
#define MSP2_GET_VTX_DEVICE_STATUS          0x3004
#define MSP2_GET_OSD_WARNINGS               0x3005  // returns active OSD warning message text

// Rotorflight-specific (high ID range)
// CMS-over-MSP (structured CMS menu export for radio-side renderers)
#define MSP2_RF_CMS_INFO                    0x3100
#define MSP2_RF_CMS_MENU_GET                0x3101
#define MSP2_RF_CMS_VALUE_GET               0x3102
#define MSP2_RF_CMS_VALUE_SET               0x3103
#define MSP2_RF_CMS_ACTION                  0x3104
#define MSP2_RF_CMS_STR_GET                 0x3105
#define MSP2_RF_CMS_VALUE_META_GET          0x3106
#define MSP2_RF_CMS_SAVE                    0x3107
#define MSP2_RF_CMS_SAVE_NOEXIT             0x3108

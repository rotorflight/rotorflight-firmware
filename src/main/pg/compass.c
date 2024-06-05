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

#include "types.h"
#include "platform.h"

#if defined(USE_MAG)

#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"

#include "pg/pg_ids.h"
#include "pg/compass.h"


PG_REGISTER_WITH_RESET_FN(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 3);

void pgResetFn_compassConfig(compassConfig_t *compassConfig)
{
    compassConfig->mag_hardware = MAG_DEFAULT;
    compassConfig->mag_alignment = ALIGN_DEFAULT;
    memset(&compassConfig->mag_customAlignment, 0x00, sizeof(compassConfig->mag_customAlignment));

// Generate a reasonable default for backward compatibility
// Strategy is
// 1. If SPI device is defined, it will take precedence, assuming it's onboard.
// 2. I2C devices are will be handled by address = 0 (per device default).
// 3. Slave I2C device on SPI gyro

#if defined(USE_SPI) && (defined(USE_MAG_SPI_HMC5883) || defined(USE_MAG_SPI_AK8963))
    compassConfig->mag_busType = BUS_TYPE_SPI;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(MAG_SPI_INSTANCE));
    compassConfig->mag_spi_csn = IO_TAG(MAG_CS_PIN);
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    compassConfig->mag_i2c_address = 0;
#elif defined(USE_MAG_HMC5883) || defined(USE_MAG_QMC5883) || defined(USE_MAG_AK8975) || (defined(USE_MAG_AK8963) && !(defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)))
    compassConfig->mag_busType = BUS_TYPE_I2C;
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(MAG_I2C_INSTANCE);
    compassConfig->mag_i2c_address = 0;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    compassConfig->mag_spi_csn = IO_TAG_NONE;
#elif defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    compassConfig->mag_busType = BUS_TYPE_MPU_SLAVE;
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    compassConfig->mag_i2c_address = 0;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    compassConfig->mag_spi_csn = IO_TAG_NONE;
#else
    compassConfig->mag_hardware = MAG_NONE;
    compassConfig->mag_busType = BUS_TYPE_NONE;
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    compassConfig->mag_i2c_address = 0;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    compassConfig->mag_spi_csn = IO_TAG_NONE;
#endif
    compassConfig->interruptTag = IO_TAG(MAG_INT_EXTI);
}

#endif

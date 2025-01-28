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

/*
 * This file is to be linked with module under test. It provides all c function
 * stubs of flash.c and call the corresponding member methods defined in
 * FlashInterface, which is virtual and can be gmock or fake implementation.
 */

extern "C" {
#include "drivers/flash.h"
}

#include "flash_interface.h"
#include "pg/flash.h"

#include <memory>

#include "gmock/gmock.h"

std::weak_ptr<FlashInterface> g_flash_stub;

void flashPreInit(const flashConfig_t *flashConfig) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashPreInit(flashConfig);
}
bool flashInit(const flashConfig_t *flashConfig) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashInit(flashConfig);
}

bool flashIsReady(void) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashIsReady();
}
bool flashWaitForReady(void) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashWaitForReady();
}
void flashEraseSector(uint32_t address) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashEraseSector(address);
}
void flashEraseCompletely(void) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashEraseCompletely();
}
void flashPageProgramBegin(uint32_t address, void (*callback)(uint32_t arg)) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashPageProgramBegin(address, callback);
}
uint32_t flashPageProgramContinue(const uint8_t **buffers,
                                  uint32_t *bufferSizes, uint32_t bufferCount) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashPageProgramContinue(buffers, bufferSizes,
                                                bufferCount);
}
void flashPageProgramFinish(void) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashPageProgramFinish();
}
void flashPageProgram(uint32_t address, const uint8_t *data, uint32_t length,
                      void (*callback)(uint32_t length)) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashPageProgram(address, data, length, callback);
}
int flashReadBytes(uint32_t address, uint8_t *buffer, uint32_t length) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashReadBytes(address, buffer, length);
}
void flashFlush(void) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashFlush();
}
const flashGeometry_t *flashGetGeometry(void) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashGetGeometry();
}

void flashPartitionSet(uint8_t index, uint32_t startSector,
                       uint32_t endSector) {
    auto flash_intf = g_flash_stub.lock();
    flash_intf->flashPartitionSet(index, startSector, endSector);
}
flashPartition_t *flashPartitionFindByType(flashPartitionType_e type) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashPartitionFindByType(type);
}
const flashPartition_t *flashPartitionFindByIndex(uint8_t index) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashPartitionFindByIndex(index);
}
const char *flashPartitionGetTypeName(flashPartitionType_e type) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashPartitionGetTypeName(type);
}
int flashPartitionCount(void) {
    auto flash_intf = g_flash_stub.lock();
    return flash_intf->flashPartitionCount();
}

// Dummy functions we don't care
bool flashSuspendSupported(void) { return false; }
void flashSuspend(void) {}
void flashResume(void) {}
bool flashIsSuspended(void) { return false; }

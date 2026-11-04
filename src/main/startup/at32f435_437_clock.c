/**
  **************************************************************************
  * @file     at32f435_437_clock.c
  * @brief    system clock config program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "at32f435_437_clock.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

// The SDK uses HEXT_VALUE to report system and peripheral clocks.
uint32_t at32HextHz = AT32_HEXT_DEFAULT_MHZ * 1000000U;

static const struct {
  uint16_t mhz;
  crm_usb_div_type usbDivider;
} clockProfiles[] = {
  {288, CRM_USB_DIV_6},
  {192, CRM_USB_DIV_4},
  {216, CRM_USB_DIV_4_5},
  {240, CRM_USB_DIV_5},
};

static bool pllFactors(uint8_t hextMhz, uint16_t systemMhz, uint16_t *divider, uint16_t *multiplier)
{
  const unsigned source = hextMhz ? hextMhz : HICK_VALUE / 1000000U;
  const unsigned vco = 4U * systemMhz;
  if ((hextMhz && (hextMhz < 4 || hextMhz > AT32_HEXT_MAX_MHZ)) || vco < 500 || vco > 1200) {
    return false;
  }
  for (unsigned m = 1; m <= 15; m++) {
    const unsigned numerator = vco * m;
    const unsigned n = numerator / source;
    if (source >= 2 * m && source <= 16 * m && numerator % source == 0 && n >= 31 && n <= 500) {
      *divider = m;
      *multiplier = n;
      return true;
    }
  }
  return false;
}

static uint8_t bootHextMhz(void)
{
  // An empty backup register means factory default. A stored 1 selects HICK.
  const uint32_t stored = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);
  return stored && stored <= AT32_HEXT_MAX_MHZ + 1U ? stored - 1U : AT32_HEXT_DEFAULT_MHZ;
}

bool systemClockUsesInternalOscillator(void)
{
  return CRM->pllcfg_bit.pllrcs == CRM_PLL_SOURCE_HICK;
}

uint8_t systemClockApplyProfile(uint8_t level, bool clockOutputEnabled, uint8_t hextMhz)
{
  if (clockOutputEnabled) {
    level = 2; // 216 MHz / 8 produces the F405's 27 MHz MCO2.
  }
  if (level >= sizeof(clockProfiles) / sizeof(clockProfiles[0])) {
    level = 0;
  }

  const uint16_t target = clockProfiles[level].mhz;
  uint16_t divider, multiplier;
  if (!pllFactors(hextMhz, target, &divider, &multiplier)) {
    hextMhz = 0;
  }

  const bool oscillatorChanged = bootHextMhz() != hextMhz;
  const bool profileChanged = system_core_clock != target * 1000000U;
  if (oscillatorChanged || profileChanged) {
    persistentObjectWrite(PERSISTENT_OBJECT_HSE_VALUE, (uint32_t)hextMhz + 1U);
    persistentObjectWrite(PERSISTENT_OBJECT_OVERCLOCK_LEVEL, target);
    systemResetHard();
  }
  return hextMhz;
}

static bool startPll(crm_pll_clock_source_type source, uint16_t divider, uint16_t multiplier)
{
  crm_pll_config(source, multiplier, divider, CRM_PLL_FR_4);
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  for (unsigned attempt = 0; attempt < 25000; attempt++) {
    if (crm_flag_get(CRM_PLL_STABLE_FLAG) == SET) {
      return true;
    }
  }
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, FALSE);
  return false;
}

void system_clock_config(void)
{
  const uint32_t requestedMhz = persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL);
  unsigned level = 0;
  for (unsigned i = 1; i < sizeof(clockProfiles) / sizeof(clockProfiles[0]); i++) {
    if (clockProfiles[i].mhz == requestedMhz) {
      level = i;
      break;
    }
  }

  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);
  crm_reset();

  const uint8_t hextMhz = bootHextMhz();
  uint16_t divider, multiplier;
  bool pllLocked = false;
  if (hextMhz && pllFactors(hextMhz, clockProfiles[level].mhz, &divider, &multiplier)) {
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (crm_hext_stable_wait() == SUCCESS) {
      at32HextHz = hextMhz * 1000000U;
      pllLocked = startPll(CRM_PLL_SOURCE_HEXT, divider, multiplier);
    }
  }
  if (!pllLocked) {
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, FALSE);
    if (!pllFactors(0, clockProfiles[level].mhz, &divider, &multiplier) ||
        !startPll(CRM_PLL_SOURCE_HICK, divider, multiplier)) {
      // Peripheral timing is invalid if both oscillator paths fail to lock.
      while (1) {
      }
    }
  }

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_2);
  crm_apb1_div_set(CRM_APB1_DIV_2);
  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
  }
  crm_auto_step_mode_enable(FALSE);

  crm_usb_clock_div_set(clockProfiles[level].usbDivider);
  crm_usb_clock_source_select(systemClockUsesInternalOscillator() ? CRM_USB_CLOCK_SOURCE_HICK : CRM_USB_CLOCK_SOURCE_PLL);
  system_core_clock_update();
}

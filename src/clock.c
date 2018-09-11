/*
 * clock.c
 *
 *  Created on: 10. 8. 2016
 *      Author: priesolv
 */

#include "clock.h"

//  // pri behu na MSI 2,1 MHz je spotreba 261 uA
//
//  // pri behu na HSI 16MHz je spotreba cca 1 mA

void SetHSI16(void)
{
  RCC->CR |= RCC_CR_HSION;  // HSI osc ON
  RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; // set HSI as SYSCLK
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // wait for change

  RCC->CR &= ~RCC_CR_MSION;   // MSI oscilator OFF
}

void SetMSI(msi_cloks_e eMsiRange)
{
  RCC->CR |= RCC_CR_MSION;    // MSI oscilator ON
  RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_MSI; // set MSI as SYSCLK
  RCC->ICSCR = (RCC->ICSCR & (~RCC_ICSCR_MSIRANGE)) | (uint32_t) eMsiRange;
  RCC->CR &= (~RCC_CR_HSION);   // HSI oscilator OFF
}

/*  Performance versus VCORE ranges
* CPU performance | Power performance | VCORE range | Typical Value (V) | Max frequency (MHz) | VDD range
*                 |                   |             |                   |   1 WS   |   0 WS   |
*       High      |        Low        |      1      |         1.8       |    32    |    16    | 1.71 - 3.6
*      Medium     |      Medium       |      2      |         1.5       |    16    |     8    | 1.65 - 3.6
*       Low       |       High        |      3      |         1.2       |    4,2   |    4,2   | 1.65 - 3.6
*/

void SetVoltageRange(range_e eRange)
{
  // prepnuti CORE na Range3 (nelze flashovat)
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock
  while (PWR->CSR & PWR_CSR_VOSF);  // wait for regulatro is ready
  PWR->CR &= ~PWR_CR_VOS;           // reset VOS bits
  PWR->CR |= eRange << 11;          // set voltage range
  while (PWR->CSR & PWR_CSR_VOSF);  // wait for regulatro is ready
}

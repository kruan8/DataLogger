/*
 * common.c
 *
 *  Created on: 9. 8. 2016
 *      Author: priesolv
 */

#include "common_L0.h"

#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_bus.h"

void GPIO_ClockEnable(gpio_pins_e ePortPin)
{
  uint16_t nPort = ((uint32_t)GET_PORT(ePortPin) - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));
  RCC->IOPENR |= (1 << nPort);
}

void GPIO_ClockDisable(gpio_pins_e ePortPin)
{
  uint16_t nPort = ((uint32_t)GET_PORT(ePortPin) - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));
  RCC->IOPENR &= ~(1 << nPort);
}

GPIO_TypeDef* GPIO_GetPort(gpio_pins_e ePortPin)
{
  GPIO_TypeDef* port;
  port = (GPIO_TypeDef*)(GPIOA_BASE + ((ePortPin >> 4) * ((GPIOB_BASE) - (GPIOA_BASE))));
  return port;
}

uint16_t GPIO_GetPin(gpio_pins_e ePortPin)
{
  return (1 << (ePortPin & 0x0F));
}

void GPIO_SetAFpin(gpio_pins_e ePortPin, uint8_t nAF)
{
  uint32_t nPin = ePortPin & 0xF;
  if (nPin < 8)
  {
    MODIFY_REG(GET_PORT(ePortPin)->AFR[0], GPIO_AFRL_AFSEL0 << (nPin << 2), nAF << (nPin << 2));
  }
  else
  {
    nPin -= 8;
    MODIFY_REG(GET_PORT(ePortPin)->AFR[1], GPIO_AFRH_AFSEL8 << (nPin << 2), nAF << (nPin << 2));
  }
}


uint16_t GPIO_GetPinSource(uint16_t GPIO_Pin)
{
  uint16_t pinsource = 0;

  /* Get pinsource */
  while (GPIO_Pin > 1)
  {
    pinsource++;
    GPIO_Pin >>= 1;
  }

  /* Return source */
  return pinsource;
}

void GPIO_ConfigPin(gpio_pins_e ePin, pin_mode_e eMode, pin_output_type_e eOutType, pin_pushpull_e ePull, pin_speed_e eSpeed)
{
  GPIO_ClockEnable(ePin);

  LL_GPIO_SetPinMode(GET_PORT(ePin), GET_PIN(ePin), eMode);

  LL_GPIO_SetPinOutputType(GET_PORT(ePin), GET_PIN(ePin), eOutType);

  LL_GPIO_SetPinPull(GET_PORT(ePin), GET_PIN(ePin), ePull);

  LL_GPIO_SetPinSpeed(GET_PORT(ePin), GET_PIN(ePin), eSpeed);
}

void EXTI_Config(gpio_pins_e ePin, exti_trigger_e eTrigger)
{
  static const uint32_t arrLines[] =
  {
    LL_SYSCFG_EXTI_LINE0,
    LL_SYSCFG_EXTI_LINE1,
    LL_SYSCFG_EXTI_LINE2,
    LL_SYSCFG_EXTI_LINE3,
    LL_SYSCFG_EXTI_LINE4,
    LL_SYSCFG_EXTI_LINE5,
    LL_SYSCFG_EXTI_LINE6,
    LL_SYSCFG_EXTI_LINE7,
    LL_SYSCFG_EXTI_LINE8,
    LL_SYSCFG_EXTI_LINE9,
    LL_SYSCFG_EXTI_LINE10,
    LL_SYSCFG_EXTI_LINE11,
    LL_SYSCFG_EXTI_LINE12,
    LL_SYSCFG_EXTI_LINE13,
    LL_SYSCFG_EXTI_LINE14,
    LL_SYSCFG_EXTI_LINE15,
  };

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  LL_SYSCFG_SetEXTISource(ePin >> 4, arrLines[ePin & 0x0F]);

  uint16_t nPin = GPIO_GetPin(ePin);

  LL_EXTI_EnableIT_0_31(nPin);

  switch (eTrigger)
  {
    case exti_rising:
      /* First Disable Falling Trigger on provided Lines */
      LL_EXTI_DisableFallingTrig_0_31(nPin);
      /* Then Enable Rising Trigger on provided Lines */
      LL_EXTI_EnableRisingTrig_0_31(nPin);
      break;
    case exti_falling:
      /* First Disable Rising Trigger on provided Lines */
      LL_EXTI_DisableRisingTrig_0_31(nPin);
      /* Then Enable Falling Trigger on provided Lines */
      LL_EXTI_EnableFallingTrig_0_31(nPin);
      break;
    case exti_rising_falling:
      LL_EXTI_EnableRisingTrig_0_31(nPin);
      LL_EXTI_EnableFallingTrig_0_31(nPin);
      break;
    default:
      break;
  }

  LL_EXTI_ClearFlag_0_31(nPin);
}


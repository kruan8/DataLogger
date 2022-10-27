/*
 * common.h
 *
 *  Created on: 9. 8. 2016
 *      Author: priesolv
 */

#ifndef SHARE_F4_COMMON_H_
#define SHARE_F4_COMMON_H_


#include "stm32l0xx.h"
#include <stdbool.h>

#include "stm32l0xx_ll_gpio.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define MAX(a,b) ((a) > (b) ? a : b)
#ifndef MIN
  #define MIN(a,b) ((a) < (b) ? a : b)
#endif

typedef enum
{
  PA0 = 0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
  PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
  PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
  PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
  PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
  PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
  PH0, PH1, PH2, PH3, PH4, PH5, PH6, PH7, PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15,
  P_UNUSED = 0xFF
}gpio_pins_e;

typedef enum
{
  mode_input = LL_GPIO_MODE_INPUT,
  mode_output = LL_GPIO_MODE_OUTPUT,
  mode_alternate = LL_GPIO_MODE_ALTERNATE,
  mode_analog = LL_GPIO_MODE_ANALOG
} pin_mode_e;

typedef enum
{
  outtype_pushpull = LL_GPIO_OUTPUT_PUSHPULL,
  outtype_od = LL_GPIO_OUTPUT_OPENDRAIN
} pin_output_type_e;

typedef enum
{
  pushpull_no = LL_GPIO_PULL_NO,
  pushpull_up = LL_GPIO_PULL_UP,
  pushpull_down = LL_GPIO_PULL_DOWN
} pin_pushpull_e;

typedef enum
{
  speed_low = LL_GPIO_SPEED_LOW,
  speed_medium = LL_GPIO_SPEED_MEDIUM,
  speed_high = LL_GPIO_SPEED_FAST,
  speed_veryhigh = LL_GPIO_SPEED_HIGH
} pin_speed_e;

typedef enum
{
  exti_rising = 0x01,
  exti_falling = 0x02,
  exti_rising_falling = 0x03
} exti_trigger_e;

// makra pro rychlou atomickou operaci zapisu
#define GET_PORT(PIN)            ((GPIO_TypeDef*)(GPIOA_BASE + ((PIN >> 4) * ((GPIOB_BASE) - (GPIOA_BASE)))))
#define GET_PIN(PIN)             (1 << (PIN & 0x0F))
#define GPIO_CLK_ENABLE(PORT)    (RCC->IOPENR |= (1 << ((uint32_t)PORT - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE))))

#define GPIO_SETPIN(PIN)         (GET_PORT(PIN)->BSRR = GET_PIN(PIN))
#define GPIO_RESETPIN(PIN)       (GET_PORT(PIN)->BRR = GET_PIN(PIN))

#define  ID1 (*(unsigned long *)0x1FF80050)
#define  ID2 (*(unsigned long *)0x1FF80054)
#define  ID3 (*(unsigned long *)0x1FF80058)

void GPIO_ClockEnable(gpio_pins_e ePortPin);
void GPIO_ClockDisable(gpio_pins_e ePortPin);
GPIO_TypeDef* GPIO_GetPort(gpio_pins_e ePortPin);
uint16_t GPIO_GetPin(gpio_pins_e ePortPin);
void GPIO_SetAFpin(gpio_pins_e ePortPin, uint8_t nAF);

uint16_t GPIO_GetPinSource(uint16_t GPIO_Pin);

void GPIO_ConfigPin(gpio_pins_e ePin, pin_mode_e eMode, pin_output_type_e eOutType, pin_pushpull_e ePull, pin_speed_e eSpeed);

void EXTI_Config(gpio_pins_e ePin, exti_trigger_e eTrigger);

#ifdef __cplusplus
 }
#endif

#endif /* SHARE_F4_COMMON_H_ */

/*
 * App.h
 *
 *  Created on: 9. 11. 2016
 *      Author: vladicek
 */

#ifndef APP_H_
#define APP_H_

#include "stm32l0xx.h"
#include <stdbool.h>


typedef enum
{
  err_ok,
  err_supply,
  err_full_memory,
  err_init_flash_error,
  err_write_flash_error,
} app_error_t;

typedef struct
{
  uint32_t time;
  int16_t  temperature;
} __attribute__((packed)) app_record_t;

void APP_Init(void);
void APP_Measure(void);
void APP_UsartExec(void);
uint32_t APP_GetRecords();
uint32_t APP_FindFlashPosition();
uint32_t APP_GetFreeRecords();
void APP_PrintRecords();
void APP_SupplyOnAndWait();
void APP_SupplyOff();
void APP_SetLPmode(bool bStandby);
void APP_SaveTempOffset(int16_t nOffset);
void APP_SaveInterval(uint32_t nInterval);
uint32_t APP_GetInterval_s(void);
void APP_LogError(app_error_t e_error);

bool App_LoadBackup();
void App_SaveBackup();
void App_ClearBackup();
uint32_t App_CountCRC32HW(uint8_t* buffer, uint16_t size);

#endif /* APP_H_ */

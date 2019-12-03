/*
 * App.c
 *
 *  Created on: 9. 11. 2016
 *  Author: Priesol Vladimir
 */


#include <stdio.h>
#include <string.h>
#include "App.h"
#include "usart.h"
#include "rtc.h"
#include "adc.h"
#include "FlashG25.h"
#include "clock.h"
#include "Eeprom.h"

#define SUPPLY_PIN              (1 << 2)
#define SUPPLY_GPIO_PORT        GPIOA

// tranzistor BSS84 se spina urovni L
#define SUPPLY_ENABLE           (SUPPLY_GPIO_PORT->BRR = SUPPLY_PIN)
#define SUPPLY_DISABLE          (SUPPLY_GPIO_PORT->BSRR = SUPPLY_PIN)

#define RECORD_SIZE             (sizeof(app_record_t))   // 5
#define RECORDS_PER_SECTOR      (4096 / RECORD_SIZE)
#define FULL_SECTOR             (RECORDS_PER_SECTOR * RECORD_SIZE)

#ifdef DEBUG
#define WAKEUP_INTERVAL_S    10  // 10 seconds
#else
#define WAKEUP_INTERVAL_S    (30 * 60)  // 30 minut
#endif

#define MIN_SUPPLY_VOLTAGE_MV   2600  // minimalni povolene napajeci napeti 2,6V (flash pamet)

#define EEPROM_TEMP_OFFSET    0                                        // offset kalibrace teploty (int32)
#define EEPROM_TEMP_ADC      (EEPROM_TEMP_OFFSET + sizeof(int32_t))    // (uint32)
#define EEPROM_INTERVAL_S    (EEPROM_TEMP_ADC + sizeof(int32_t))       // interval probouzeni a mereni (uint32)
#define EEPROM_ERROR         (EEPROM_INTERVAL_S + sizeof(int32_t))     // ulozena chyba zarizeni (uint32)

#define BACKUP_ADDR          0

typedef struct
{
  uint16_t nSector;
  uint16_t nSectorPos;
  uint32_t nCrc;
  // max 5 32bit. hodnot
}backup_t;

static uint16_t    g_nWakeUpInterval_s = WAKEUP_INTERVAL_S;

//static const uint8_t EmptyRecord[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const app_record_t EmptyRecord = { 0xFFFFFFFF, 0xFFFF};

static uint16_t    g_nSector;
static uint16_t    g_nSectorPosition;

static app_error_t g_eError = err_ok;
static uint16_t    g_nVoltage;
static bool        g_bWakedFromStandby;
static backup_t    g_backup;


void APP_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock

  // Set clock & power
  SetMSI(msi_1Mhz);
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);
  SetVoltageRange(range3);

  // identifikace navratu ze standby modu
  if (RCC->CSR & RCC_CSR_PINRSTF)  // test na pin reset
  {
    RCC->CSR |= RCC_CSR_RMVF;
    g_bWakedFromStandby = false;
  }
  else
  {
    if (PWR->CSR & PWR_CSR_SBF) // zarizeni bylo ve Standby modu
    {
      PWR->CR |= PWR_CR_CSBF;  // clear SBF flag

      // WUF je pri standby vymazan resetem, takze neni nastaven
  //    if (PWR->CSR & PWR_CSR_WUF)  // je nastaven WakeUp flag?
  //    {
  //      bWUF = true;
  //      PWR->CR |= PWR_CR_CWUF;  // clear WUF flag
  //
  //
  //    }

      g_bWakedFromStandby = true;
    }
  }

  g_eError = Eeprom_ReadUint32(EEPROM_ERROR);

  // pri probuzeni s chybou opet usiname
  // Todo: a co treba zrusit probouzeni?
  if (g_bWakedFromStandby && g_eError != err_ok)
  {
    return;
  }

  Adc_Init();
  RTC_Init();
  APP_SupplyOnAndWait(); // open supply and wait for flash memory wakewup


  // nacteme konstanty z EEPROM
  Adc_SetTempOffset(Eeprom_ReadUint32(EEPROM_TEMP_OFFSET));

  // nacist interval z EEPROM (0 = neulozena hodnota)
  uint32_t nInterval = Eeprom_ReadUint32(EEPROM_INTERVAL_S);
  if (nInterval)
  {
    g_nWakeUpInterval_s = nInterval;
  }

  // kontrola velikosti napajeciho napeti
  g_nVoltage = Adc_MeasureRefInt_mV();
  if (g_nVoltage < MIN_SUPPLY_VOLTAGE_MV)
  {
    APP_LogError(err_supply);
    return;
  }
//
//
//
//
//  // ---------------------------------------------
//    USART_Init();
//    USART_Putc('>');
//    USART_PrintLine((uint8_t*)"Wakeup SBF");
//    USART_WaitForTC();
//  // ----------------------------------------------

  // -------------------------------------------------------
  if (!FlashG25_Init())
  {
    APP_LogError(err_init_flash_error);
    return;
  }

  // mereni teploty a ulozeni do flash
  if (g_bWakedFromStandby)
  {
    if (!App_LoadBackup())      // natazeni pozice FLASH
    {
      APP_FindFlashPosition();
      g_backup.nSector = g_nSector;
      g_backup.nSectorPos = g_nSectorPosition;
    }

    g_nSector = g_backup.nSector;
    g_nSectorPosition = g_backup.nSectorPos;

    APP_Measure();

    g_backup.nSector = g_nSector;
    g_backup.nSectorPos = g_nSectorPosition;
    App_SaveBackup();
  }

}

void APP_Measure(void)
{
  int16_t temp = Adc_GetTemperature(true);

  // prepare record
  app_record_t record;
  rtc_record_time_t dt;

  RTC_Get(&dt);
  record.time = RTC_GetUnixTimeStamp(&dt);
  record.temperature = temp;

  // zapis do pameti a kontrola obsahu
  FlashG25_PageProgram(g_nSector * G25_SECTOR_SIZE + g_nSectorPosition, (uint8_t*)&record, RECORD_SIZE);

  app_record_t record_check;
  FlashG25_ReadData(g_nSector * G25_SECTOR_SIZE + g_nSectorPosition, (uint8_t*)&record_check, RECORD_SIZE);
  if (memcmp (&record, &record_check, RECORD_SIZE) != 0)
  {
	  APP_LogError(err_write_flash_error);
  }

  g_nSectorPosition += RECORD_SIZE;
  if (g_nSectorPosition >= FULL_SECTOR)
  {
    g_nSectorPosition = 0;
    g_nSector++;
    if (g_nSector >= FlashG25_GetSectors())
    {
    	APP_LogError(err_full_memory);
    }
  }

}

void APP_UsartExec(void)
{
  if (!g_bWakedFromStandby)
  {
    USART_Init();
    APP_FindFlashPosition();
    USART_PrintHeader(APP_GetRecords(), APP_GetFreeRecords(), g_nVoltage, g_eError);
    USART_Putc('>');
    RTC_SetUsartTimer(15000);     // waiting for usart input

    while (RTC_GetUsartTimer())
    {
      USART_ProcessCommand();
    }

    USART_PrintLine((uint8_t*)"Exit to measure mode");
    USART_WaitForTC();
#ifndef DEBUG
    USART_DeInit();
#endif
  }

  APP_SupplyOff();
  RTC_SetWakeUp(APP_GetInterval_s());
  APP_SetLPmode(true);
}

uint32_t APP_GetRecords()
{
  return g_nSector * RECORDS_PER_SECTOR + g_nSectorPosition / RECORD_SIZE;
}

uint32_t APP_GetFreeRecords()
{
  uint32_t nFreeRecords = ((FlashG25_GetSectors() - g_nSector) * G25_SECTOR_SIZE - g_nSectorPosition) / RECORD_SIZE;
  return nFreeRecords;
}

uint32_t APP_FindFlashPosition()
{
  uint8_t buff[RECORD_SIZE];

  // find last used sector;
  bool bFullMemory = true;
  for (g_nSector = 0; g_nSector < FlashG25_GetSectors(); g_nSector++)
  {
    FlashG25_ReadData(G25_SECTOR_SIZE * g_nSector, buff, RECORD_SIZE); // check only first record position
    if (memcmp(buff, &EmptyRecord, sizeof (EmptyRecord)) == 0)   // free position founded
    {
      bFullMemory = false;
      break;
    }
  }

  if (bFullMemory)
  {
	  APP_LogError(err_full_memory);
    return 0;
  }

  // find last record in the sector
  g_nSectorPosition = 0;
  if (g_nSector != 0)
  {
    g_nSector--;
    while (g_nSectorPosition < FULL_SECTOR)
    {
      FlashG25_ReadData(g_nSector * G25_SECTOR_SIZE + g_nSectorPosition, buff, RECORD_SIZE);
      if (memcmp(buff, &EmptyRecord, sizeof (EmptyRecord)) == 0)
      {
        break;
      }

      g_nSectorPosition += RECORD_SIZE;
    }

    if (g_nSectorPosition >= FULL_SECTOR)
    {
      g_nSector++;
      g_nSectorPosition = 0;
    }
  }

  return APP_GetFreeRecords();
}

void APP_PrintRecords()
{
  app_record_t record;
  uint8_t text[30];
  uint32_t nRecords = 0;

  USART_PrintLine((uint8_t*)"Memory report:");

  uint32_t nSectors = FlashG25_GetSectors();
  for (uint16_t sect = 0; sect < nSectors; sect++)
  {
    for (uint16_t pos = 0; pos < FULL_SECTOR; pos += RECORD_SIZE)
    {
      FlashG25_ReadData(sect * G25_SECTOR_SIZE + pos, (uint8_t*) &record, RECORD_SIZE);
      if (memcmp((uint8_t*) &record, &EmptyRecord, sizeof (EmptyRecord)) == 0)
      {
        snprintf((char*)text, sizeof(text), "Number of records:%lu", nRecords);
        USART_PrintLine(text);
        return;
      }

      nRecords++;

      rtc_record_time_t rtime;
      RTC_GetDateTimeFromUnix(&rtime, record.time);
//      RTC_ConvertToStruct(record.time, &rtime);

      snprintf((char*)text, sizeof(text), "%d.%d.%d %02d:%02d=",
          rtime.day, rtime.month, rtime.year, rtime.hour, rtime.min);

      USART_Print(text);
      USART_PrintTemperature(record.temperature);
      USART_PrintNewLine();
    }
  }

  RTC_SetUsartTimer(60000);       // timeout for COM session
}

void APP_SupplyOnAndWait()
{
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // PA2 output mode (spinani tranzistoru pro napajeni MCP9700 + G25D10)
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2)) | GPIO_MODER_MODE2_0;

  // vratit do AF (SPI1)
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE7)) | GPIO_MODER_MODE7_1;
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE5)) | GPIO_MODER_MODE5_1;
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE6)) | GPIO_MODER_MODE6_1;

  // nastavit CS pro AT25
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4)) | GPIO_MODER_MODE4_0;

  SUPPLY_ENABLE;

  uint32_t start = RTC_GetTicks();
  while ((RTC_GetTicks() - start) < 9);   // wait min 10ms for flash memory wakeup
}

void APP_SupplyOff()
{
  ADC->CCR &= ~ADC_CCR_VREFEN;     // disable VREFINT
  ADC->CCR &= ~ADC_CCR_TSEN;       // disable TEMP_INT

  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE2)) | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE2_1; // analog mode

  // tekl zde proud z PA7 (SPI MOSI), ktery se pres FLASH pamet objevil na napajeci vetvi, na ktere vzniklo napeti 2,8V
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE7)) | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE7_1; // analog mode
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE5)) | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE5_1; // analog mode
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE6)) | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE6_1; // analog mode

  // analog mode for CS pin
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE4)) | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1;
}


// STOP mode continues after 'wfi' instruction
// STANDBY mode continues from RESET vector and RAM is erased
void APP_SetLPmode(bool bStandby)
{
  // Adc_Disable();
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock
  PWR->CR |= PWR_CR_ULP;
  if (bStandby)
  {
    PWR->CR |= PWR_CR_PDDS;  // rozlisuje mody STANDBY | STOP
  }

  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  PWR->CR |= PWR_CR_CWUF;  // Clear Wakeup flag
  PWR->CR |= PWR_CR_LPSDSR;
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Set SLEEPDEEP bit of Cortex-M0 System Control Register

  __asm volatile ("wfi");

  SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
  PWR->CR &= ~PWR_CR_LPSDSR;
  SysTick->VAL = 0;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

//  Adc_Enable();
}

void APP_SaveTempOffset(int16_t nOffset)
{
  Adc_SetTempOffset(nOffset);

  // ulozit do EEPROM
  Eeprom_UnlockPELOCK();
  Eeprom_WriteUint32(EEPROM_TEMP_OFFSET, nOffset);
  Eeprom_LockNVM();
}

void APP_SaveInterval(uint32_t nInterval)
{
  g_nWakeUpInterval_s = nInterval;
  Eeprom_UnlockPELOCK();
  Eeprom_WriteUint32(EEPROM_INTERVAL_S, nInterval);
  Eeprom_LockNVM();
}

uint32_t APP_GetInterval_s(void)
{
  return g_nWakeUpInterval_s;
}

void APP_LogError(app_error_t e_error)
{
	g_eError = e_error;

	// ulozit chybu do EEPROM
  Eeprom_UnlockPELOCK();
  Eeprom_WriteUint32(EEPROM_ERROR, e_error);
  Eeprom_LockNVM();
}

bool App_LoadBackup()
{
  uint8_t nSize = sizeof(backup_t) / sizeof(uint32_t);
  uint32_t *pData = (uint32_t*)&g_backup;
  uint32_t nIndex = BACKUP_ADDR;

  while (nSize--)
  {
    *pData = RTC_ReadBackup(nIndex++);
    pData++;
  }

  nSize = sizeof (g_backup) - sizeof (g_backup.nCrc);
  uint32_t crc = App_CountCRC32HW((uint8_t*)&g_backup, nSize);
  if (crc != g_backup.nCrc)
  {
    return false;
  }

  return true;
}

void App_SaveBackup()
{
  uint8_t nSize = sizeof (g_backup) - sizeof (g_backup.nCrc);
  g_backup.nCrc = App_CountCRC32HW((uint8_t*)&g_backup, nSize);

  nSize = sizeof(backup_t) / sizeof(uint32_t);
  uint32_t *pData = (uint32_t*)&g_backup;
  uint32_t nIndex = BACKUP_ADDR;
  while (nSize--)
  {
    RTC_WriteBackup(nIndex++, *pData);
    pData++;
  }

}

void App_ClearBackup()
{
  g_backup.nSector = 0;
  g_backup.nSector = 0;
  App_SaveBackup();
}

uint32_t App_CountCRC32HW(uint8_t* pBuffer, uint16_t nSize)
{
  RCC->AHBENR |= RCC_AHBENR_CRCEN;
  CRC->CR = CRC_CR_RESET;
  for (uint16_t a = 0; a < nSize; a++)
  {
    CRC->DR = pBuffer[a];
  }

  uint32_t crc = (uint32_t) (CRC->DR);
  RCC->AHBENR &= ~RCC_AHBENR_CRCEN;
  return crc;
}

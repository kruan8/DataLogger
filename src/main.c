/*
 * main.c
 *
 *  Created on: 10. 8. 2016
 *  Author: Priesol Vladimir
 */

#include "stm32l0xx.h"
#include "rtc.h"
#include "App.h"
#include "usart.h"
#include "clock.h"
//
//#include "stm32l0xx_ll_gpio.h"

/*
 * MCU bezi z MSI oscilatoru a pri freq 2,1 Mhz ma spotrebu cca 440 uA.
 * Po zmene rozsahu napajeni na Range3 (PWR->CR) se spotreba snizila na 362 uA.
 * Pred merenim spotreby nestaci reset, ale je potreba prerusit napajeni,
 * jinak STOP mod ma spotrebu pres 100 uA (asi zustane napajeny DEBUG modul).
 *
 * Po uvedeni do STOP modu a odpojeni napajeni (PA2) od MCP9700 a G25D10,
 * se spotreba snizi na 1uA. Bylo potreba uvest do analogoveho vstupu i pin PA7
 * pres ktery tekl proud (asi PULLUP od SPI) a na PA2 se objevilo napeti 2,8 V.
 *
 * Merici cyklus trva v DEBUG konfiguraci (s vystupem výpisu na seriový port) cca 35 ms,
 * bez vypisu asi 6ms.
 *
 *
 * v0.1 - prvni verze
 * v0.2 - kalibrace nyni funguje jako ofset, ktery se uklada do EEPROM (CALxxx),
 *        ADC meri pomoci oversamplingu,
 *        oprava chyby cteni sektoru v 'APP_PrintRecords',
 *        pri inicializaci vynulovan ukazatel pozice USART bufferu (g_BufferInPos),
 *        oprava vymazu pameti (nepocital s ruznou velikosti FLASH pameti),
 *        vypis teplotniho ofsetu ve statusu + refresh ofsetu po kalibraci,
 *        timeout pred vstupem do mereni prodlouzen na 15s,
 *        snizeni hodnoty intervalu o 1s pred zapisem do RTC->WUTR,
 *
 *        9.1.2017: !!! projevuje se problem s mazanim pameti,
 *        po vycisteni napajecich kontaktu zavada odstranena, posilim napajeni pameti z pinu
 *        kapacitou 1M !!!
 *        Problem se vyskytuje nadale. Vypada to na nizke napajeci napeti pro pamet
 *        pri programovani (mazani).
 *        Nova verze PCB ma pro spinani napajeci vetve (pro flash + teplomer) tranzistor.
 *
 * v0.3 - Nova verze PCB s tranzistorem BSS84
 *
 * v0.4 - Po prostudovani datashetu CR2032 je jasne, za tato baterie nema sanci pokryt spotrebu flash pameti.
 *        CR2032 ma podle datasheetu povoleny odber 200 uA, flash memory vyzaduje 10mA.
 *        Napajeni zmeneno na 2x AAA baterii.
 *        Frekvence MCU snizena na 1MHz, doplneno logovani chyby.
 *        Nastaveni casovace komunikace po provedeni LIST.
 *        Do helpu pridan popis CAL prikazu.
 *        Odstranen vypis internal teplomeru.
 *        Pri inicializaci kontrola velikosti napìtí baterie.
 *
 * v0.5 - zmena low power modu STOP na STANDBY. Pokud je probuzeno z RTC, neinicializuje se USART.
 *        Pozice ve flash pameti je ulozeno do backup registru, aby se nemusela prochazet cela pamet.
 *        Odstraneno mereni teploty internim teplomerem.
 *        Spotøeba: beh (bez flash zapisu) 570 uA, standby 0,7 uA
 *
 * v0.6 - zmena rychlosti komunikace na 57600
 *
 * v0.7 - upravy pro Gaba:
 *         - pridana pamet GD25VQ21/2Mb
 *         - prodlouzen sleep interval na 20s
 *
 * Todo:Pro implementaci watchdogu se musi WD nastavit na nejdelsi dobu (32s), WD bude provadet reset.
 *        Vypnout preruseni RTC wakeup a po resetu provedenem pomoci WD, se zkontroluje, jestli je nastaven RTC_ISR->WUTF,
 *        coz signalizuje, ze vyprsel interval pro mereni. Pro mereni neni potreba inicializovat USART a asi ani RTC!
 *
 *
 */


int main(void)
{

  APP_Init();

  // RTC_Test();


  APP_UsartExec();

}



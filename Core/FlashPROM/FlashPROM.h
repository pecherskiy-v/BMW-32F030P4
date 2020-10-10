/*
 * FlashPROM.h
 *
 *  Created on: 30 дек. 2019 г.
 *      Author: dima
 */

#ifndef FLASHPROM_H_
#define FLASHPROM_H_

#include "main.h"
#include "string.h"
#include "stdio.h"

#define STARTADDR ((uint32_t)0x08003C00)         // адрес, с которого будет начинаться запись во флеш (с начала 15-ой страницы F030)
#define ENDMEMORY ((uint32_t)0x08003FFF)         // последняя ячейка флеша (F030)
#define PAGES 1                                  // количество страниц для очистки
#define BUFFSIZE 2                               // размер буфера для записи
#define DATAWIDTH 2                              // размерность данных буфера 16 бит - 2, 32 бита - 4
#define WIDTHWRITE FLASH_TYPEPROGRAM_HALFWORD    // длина слова (16 бит) для записи в функции HAL_FLASH_Program(...), если 32бита тогда FLASH_TYPEPROGRAM_WORD

typedef uint16_t myBuf_t;                         // либо uint32_t

void erase_flash(void);

uint32_t flash_search_address(uint32_t address, uint16_t cnt);

void write_to_flash(myBuf_t *buff);

void read_last_data_in_flash(myBuf_t *buff);

#endif /* FLASHPROM_H_ */

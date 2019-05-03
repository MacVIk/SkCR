/*
 * fw_update.h
 *
 *  Created on: 01.03.2013
 *      Author: smoker
 *
 *  Unit of firmware update functions
 */

#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Password to execute programming of higher sectors
#define REPROG_HS_PASSWORD1 0x4b09e375
#define REPROG_HS_PASSWORD2 0x816af042

//////////////////////////////////////////////////////////

// Address of flash start
#define FLASH_START_ADR 0x8000000

// Address of first sector of firmware buffer
#define FLASHBUF_PART1_ADR (FLASH_START_ADR + 0x40000)
// this sector
#define FLASHBUF_PART1_SECTOR (FLASH_Sector_6)

// Address of second sector of firmware buffer
#define FLASHBUF_PART2_ADR (FLASH_START_ADR + 0x60000)
// this sector
#define FLASHBUF_PART2_SECTOR (FLASH_Sector_7)

//////////////////////////////////////////////////////////

// Declare type of function to reprogram sectors
typedef void (*reprog_hs_func_type)(uint32_t,uint32_t, uint32_t, uint32_t);

// 99 = number of vector in ISR list
#define REPROG_HS_VECTOR ((99-1)*sizeof(long))
#define REPROG_HS_FUNC ((reprog_hs_func_type) ( *((uint32_t *) REPROG_HS_VECTOR) ) )
// calling: (*REPROG_HS_FUNC)(REPROG_HS_PASSWORD1,REPROG_HS_PASSWORD2, FLASHBUF_PART1_ADR, nwords);

////////////

// Passwords to reprogram
#define FLASHPROG_NEEDED_CONST (0x5a9a)
#define FLASHPROG_NEEDED2_CONST (0xb768)
extern uint16_t flashprog_needed;
extern uint16_t flashprog_needed2;

// counter to delay firmware update
extern uint8_t flashprog_delay_cnt;

extern uint8_t fw_checksum_part;
extern uint8_t fw_checksum_all;

////////////

void fw_flashprog_perform(void);// reprogram procedure

//bool fw_readflash32(uint8_t *dest);// reads from flash next 32 bytes
bool fw_writeflash32_buf(uint8_t *src, uint32_t d_ofs);//writes to flash next 32 bytes
void fw_writeflashadr(uint32_t d_adr);// saves address of read/write
bool fw_writeflash_start_delayed(void);// command to delayed reprogram

void fw_update_preinit(void);// module initialization

#ifdef __cplusplus
}
#endif

#endif /* FW_UPDATE_H_ */

/*
 * fw_update.c
 *
 *  Created on: 01.03.2013
 *      Author: smoker
 *
 *  Unit of firmware update functions
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_conf.h"
#include "fw_update.h"
#include "gen_utils.h"

// local copy of private constant in stm32f2xx_flash.c
#define SECTOR_MASK_LOC               ((uint32_t)0xFFFFFF07)
// local copy of private constant in stm32f2xx_iwdg.c
#define KR_KEY_RELOAD_LOC    ((uint16_t)0xAAAA)

static uint32_t flashprog_cur_adr; // текущий адрес (смещение от начала) во flash
static uint32_t flashprog_start_adr; // адрес (смещение от начала) во flash начала буфера прошивки
static uint32_t flashprog_end_adr; // адрес (смещение от начала) во flash окончания буфера прошивки

#define FLASHPROG_ADR_SETED_CONST 0x6b
uint8_t flashprog_adr_seted;

uint16_t flashprog_needed;
uint16_t flashprog_needed2;

// counter to delay firmware update
uint8_t flashprog_delay_cnt;

uint8_t fw_checksum_part;
uint8_t fw_checksum_all;

///////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// Copy of 'NVIC_SystemReset' in low sector
__attribute__ ((section(".fw_update_hs"))) static void NVIC_SystemReset_lowsect(
		void) {
	__DSB(); /* Ensure all outstanding memory accesses included
	 buffered write are completed before reset */
	SCB->AIRCR =
			((0x5FA << SCB_AIRCR_VECTKEY_Pos)
					| (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk)
					| SCB_AIRCR_SYSRESETREQ_Msk); /* Keep priority group unchanged */
	__DSB(); /* Ensure completion of memory access */
	while (1)
		; /* wait until reset */
}

/////////////////////////

// Copy of  'FLASH_GetStatus' in low sector
__attribute__ ((section(".fw_update_hs"))) static FLASH_Status FLASH_GetStatus_lowsect(
		void) {
	FLASH_Status flashstatus = FLASH_COMPLETE;

	if ((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) {
		flashstatus = FLASH_BUSY;
	} else {
		if ((FLASH->SR & FLASH_FLAG_WRPERR) != (uint32_t) 0x00) {
			flashstatus = FLASH_ERROR_WRP;
		} else {
			if ((FLASH->SR & (uint32_t) 0xEF) != (uint32_t) 0x00) {
				flashstatus = FLASH_ERROR_PROGRAM;
			} else {
				if ((FLASH->SR & FLASH_FLAG_OPERR) != (uint32_t) 0x00) {
					flashstatus = FLASH_ERROR_OPERATION;
				} else {
					flashstatus = FLASH_COMPLETE;
				}
			}
		}
	}
	/* Return the FLASH Status */
	return flashstatus;
}

/////////////////////////

// Copy of  'FLASH_WaitForLastOperation' in low sector
__attribute__ ((section(".fw_update_hs"))) static FLASH_Status FLASH_WaitForLastOperation_lowsect(
		void) {
	__IO
	FLASH_Status status = FLASH_COMPLETE;

	/* Check for the FLASH Status */
	status = FLASH_GetStatus_lowsect();

	/* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
	 Even if the FLASH operation fails, the BUSY flag will be reset and an error
	 flag will be set */
	while (status == FLASH_BUSY) {
		status = FLASH_GetStatus_lowsect();
	}
	/* Return the operation status */
	return status;
}

/////////////////////////

// Simplified copy of 'FLASH_EraseSector' in low sector
// VoltageRange= VoltageRange_3
__attribute__ ((section(".fw_update_hs"))) static FLASH_Status FLASH_EraseSector_simp_lowsect(
		uint32_t FLASH_Sector) {
	uint32_t tmp_psize = 0x0;
	FLASH_Status status = FLASH_COMPLETE;

	tmp_psize = FLASH_PSIZE_WORD; // VoltageRange= VoltageRange_3

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation_lowsect();

	if (status == FLASH_COMPLETE) {
		/* if the previous operation is completed, proceed to erase the sector */
		FLASH->CR &= CR_PSIZE_MASK;
		FLASH->CR |= tmp_psize;
		FLASH->CR &= SECTOR_MASK_LOC; // local copy of private constant in stm32f2xx_flash.c
		FLASH->CR |= FLASH_CR_SER | FLASH_Sector;
		FLASH->CR |= FLASH_CR_STRT;

		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation_lowsect();

		/* if the erase operation is completed, disable the SER Bit */FLASH->CR &=
				(~FLASH_CR_SER);
		FLASH->CR &= SECTOR_MASK_LOC;
	}
	/* Return the Erase Status */
	return status;
}

/////////////////////////

// Copy of  FLASH_ProgramWord in low sector
__attribute__ ((section(".fw_update_hs"))) static FLASH_Status FLASH_ProgramWord_lowsect(
		uint32_t Address, uint32_t Data) {
	FLASH_Status status = FLASH_COMPLETE;

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation_lowsect();

	if (status == FLASH_COMPLETE) {
		/* if the previous operation is completed, proceed to program the new data */
		FLASH->CR &= CR_PSIZE_MASK;
		FLASH->CR |= FLASH_PSIZE_WORD;
		FLASH->CR |= FLASH_CR_PG;

		*(__IO uint32_t*)Address = Data;

		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation_lowsect();

		/* if the program operation is completed, disable the PG Bit */
		FLASH->CR &= (~FLASH_CR_PG);
	}
	/* Return the Program Status */
	return status;
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

// copy data to higher sector flash
__attribute__ ((section(".fw_update_hs"))) static void copy_hs_flash(
		uint32_t buf_adr, uint32_t ofs, uint32_t size, uint32_t *pnwords) {
	uint32_t *src_ptr32;
	uint32_t dest_adr;
	uint32_t cnt;
	uint32_t v32;

	src_ptr32 = (uint32_t *) (buf_adr + ofs);
	dest_adr = (FLASH_START_ADR + ofs);
	cnt = 0;
	while (cnt < size) {
		v32 = *src_ptr32;
		if (FLASH_ProgramWord_lowsect(dest_adr, v32) == FLASH_COMPLETE) {
			dest_adr += 4;
			cnt += 4;
			src_ptr32++;

			(*pnwords)--;
			if ((*pnwords) == 0)
				break; // all data copied
		} else {
			NVIC_SystemReset_lowsect();
		}

		// Reload WatchDog counter
		IWDG->KR = KR_KEY_RELOAD_LOC;
	} //while
} // copy_hs_flash

//////////////////////////////////////////////////

// Reprogram higher sectors of firmware
// buf_adr - address of buffer of firmware in flash
// nwords - number of 32-bit words to rewrite
__attribute__ ((section(".fw_update_hs"))) void REPROG_HIGH_SECTORS(
		uint32_t write_fs_pwd1, uint32_t write_fs_pwd2, uint32_t buf_adr,
		uint32_t nwords) {
	if (write_fs_pwd1 != REPROG_HS_PASSWORD1)
		return; // bad password
	if (write_fs_pwd2 != REPROG_HS_PASSWORD2)
		return; // bad password

	// ------- copy sector 2 (16 Kbytes, address 0x8000) -------
	if (FLASH_EraseSector_simp_lowsect(FLASH_Sector_2) != FLASH_COMPLETE) { // error flash erase
		NVIC_SystemReset_lowsect();
	}
	copy_hs_flash(buf_adr, 0x8000, 0x4000, &nwords);

	// ------- copy sector 3 (16 Kbytes, address 0xC000) -------
	if (FLASH_EraseSector_simp_lowsect(FLASH_Sector_3) != FLASH_COMPLETE) { // error flash erase
		NVIC_SystemReset_lowsect();
	}
	copy_hs_flash(buf_adr, 0xC000, 0x4000, &nwords);

	// ------- copy sector 4 (64 Kbytes, address 0x10000) -------
	if (FLASH_EraseSector_simp_lowsect(FLASH_Sector_4) != FLASH_COMPLETE) { // error flash erase
		NVIC_SystemReset_lowsect();
	}
	copy_hs_flash(buf_adr, 0x10000, 0x10000, &nwords);

	// ------- copy sector 5 (128 Kbytes, address 0x20000) -------
	if (FLASH_EraseSector_simp_lowsect(FLASH_Sector_5) != FLASH_COMPLETE) { // error flash erase
		NVIC_SystemReset_lowsect();
	}
	copy_hs_flash(buf_adr, 0x20000, 0x20000, &nwords);

	// Reset. After reset new firmware should work !
	NVIC_SystemReset_lowsect();
} // REPROG_HIGH_SECTORS

////////////////////////////////////////////////////////////////////////

// reprogram procedure
void fw_flashprog_perform(void) {
	uint32_t *src_ptr32;
	uint32_t dest_adr;
	uint32_t cnt;
	uint32_t v32;

	// check passwords
	if (flashprog_needed != FLASHPROG_NEEDED_CONST)
		NVIC_SystemReset(); // bad password - execute reset
	if (flashprog_needed2 != FLASHPROG_NEEDED2_CONST)
		NVIC_SystemReset(); // bad password - execute reset

	if (flashprog_adr_seted != FLASHPROG_ADR_SETED_CONST)
		NVIC_SystemReset(); // no address setup - execute reset

	if (flashprog_start_adr != 0)
		NVIC_SystemReset(); // firmware must be from zero address

	if (flashprog_end_adr <= flashprog_start_adr)
		NVIC_SystemReset(); // zero address range

	if (flashprog_end_adr > 0x3ffff)
		NVIC_SystemReset(); // too large address range

	// DISABLE ALL INTERRUPTS
	__disable_irq();

	FLASH_ClearFlag(
			FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGAERR
					| FLASH_FLAG_WRPERR | FLASH_FLAG_OPERR | FLASH_FLAG_EOP); // clear error status
	/* Unlock the Flash to enable the flash control register access *************/
	FLASH_Unlock();

	if (flashprog_needed != FLASHPROG_NEEDED_CONST)
		NVIC_SystemReset(); // bad password - execute reset
	if (flashprog_needed2 != FLASHPROG_NEEDED2_CONST)
		NVIC_SystemReset(); // bad password - execute reset

	// if incorrect jumped here with, flash is not unlocked :)

	// ------- copy sector 0 (16 Kbytes) -------
	if (FLASH_EraseSector(FLASH_Sector_0, VoltageRange_3) != FLASH_COMPLETE) { //  error flash erase
		NVIC_SystemReset();
	}
	src_ptr32 = (uint32_t *) (FLASHBUF_PART1_ADR + 0x0000);
	dest_adr = (FLASH_START_ADR + 0x0000);
	cnt = 0;
	while (cnt < 0x4000) {
		v32 = *src_ptr32;
		if (FLASH_ProgramWord(dest_adr, v32) == FLASH_COMPLETE) {
			dest_adr += 4;
			cnt += 4;
			src_ptr32++;
		} else {
			NVIC_SystemReset();
		}

		// Reload WatchDog counter
		IWDG_ReloadCounter();
	} //while

	// ------- copy sector 1 (16 Kbytes) -------
	if (FLASH_EraseSector(FLASH_Sector_1, VoltageRange_3) != FLASH_COMPLETE) { //  error flash erase
		NVIC_SystemReset();
	}
	src_ptr32 = (uint32_t *) (FLASHBUF_PART1_ADR + 0x4000);
	dest_adr = (FLASH_START_ADR + 0x4000);
	cnt = 0;
	while (cnt < 0x4000) {
		v32 = *src_ptr32;
		if (FLASH_ProgramWord(dest_adr, v32) == FLASH_COMPLETE) {
			dest_adr += 4;
			cnt += 4;
			src_ptr32++;
		} else {
			NVIC_SystemReset();
		}

		// Reload WatchDog counter
		IWDG_ReloadCounter();
	} //while

	// -------  copy other sectors
	v32 = flashprog_end_adr - flashprog_start_adr; // number of bytes
	if (v32 < 0x8000)
		NVIC_SystemReset(); // less than 32K - already all copyed

	while ((v32 % 4) != 0)
		v32++; // align by 4
	v32 = (v32 - 0x8000) / 4; // number of 32-bit words

	// call procedure of reprogram higher sectors
	// it must be placed in already programmed part and it's address must be in interrupts vector table
	(*REPROG_HS_FUNC)(REPROG_HS_PASSWORD1, REPROG_HS_PASSWORD2,
			FLASHBUF_PART1_ADR, v32);

	NVIC_SystemReset();
} // fw_flashprog_perform

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/*
 // reads from flash next 32 bytes
 bool fw_readflash32(uint8_t *dest)
 {uint8_t i;
 uint8_t *ptr;

 if (flashprog_adr_seted!=FLASHPROG_ADR_SETED_CONST) return false;// no address set

 ptr= (uint8_t *) (FLASH_START_ADR+flashprog_cur_adr);
 for(i=0;i<32;i++)
 {
 dest[4+i]= *ptr;
 ptr++;
 }// for i

 dest[0]= BYTEPTR(flashprog_cur_adr)[0];
 dest[1]= BYTEPTR(flashprog_cur_adr)[1];
 dest[2]= BYTEPTR(flashprog_cur_adr)[2];
 dest[3]= BYTEPTR(flashprog_cur_adr)[3];// direct order of bytes (little endian)

 flashprog_cur_adr+= 32;

 return true;
 }// fw_readflash32
 */
////////////////////////////////////////////////////////////////////////

static bool flash_same32(uint8_t *src, uint8_t *dest) {
	uint8_t i;
	for (i = 0; i < 32; i++) {
		if (src[i] != dest[i])
			return false;
	}
	return true;
}

//writes to flash next 32 bytes
bool fw_writeflash32_buf(uint8_t *src, uint32_t d_ofs) {
	uint32_t adr;
	uint8_t *adr_ptr;
	uint32_t v32;
	uint8_t i;
	uint8_t src_ofs;
	uint8_t b0, b1, b2, b3;

	if (flashprog_adr_seted != FLASHPROG_ADR_SETED_CONST)
		return false; // no address set

	FLASH_ClearFlag(
			FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGAERR
					| FLASH_FLAG_WRPERR | FLASH_FLAG_OPERR | FLASH_FLAG_EOP); // clear error status
	/* Unlock the Flash to enable the flash control register access *************/
	FLASH_Unlock();

	flashprog_cur_adr = flashprog_start_adr + d_ofs;

	adr = flashprog_cur_adr + FLASHBUF_PART1_ADR;
	if (adr == FLASHBUF_PART1_ADR) { // start of sector - erase it
		if (FLASH_EraseSector(FLASHBUF_PART1_SECTOR, VoltageRange_3)
				!= FLASH_COMPLETE) { // error flash erase
			FLASH_Lock();
			return false;
		}
	}
	if (adr == FLASHBUF_PART2_ADR) { // start of sector - erase it
		if (FLASH_EraseSector(FLASHBUF_PART2_SECTOR, VoltageRange_3)
				!= FLASH_COMPLETE) { // error flash erase
			FLASH_Lock();
			return false;
		}
	}

	fw_checksum_part = 0;

	adr_ptr = (uint8_t *) adr;
	if (!flash_same32(src, adr_ptr)) {
		src_ofs = 0;
		for (i = 0; i < 8; i++) {
			b0 = src[src_ofs + 0];
			b1 = src[src_ofs + 1];
			b2 = src[src_ofs + 2];
			b3 = src[src_ofs + 3];

			fw_checksum_all ^= b0;
			fw_checksum_all ^= b1;
			fw_checksum_all ^= b2;
			fw_checksum_all ^= b3;

			fw_checksum_part += b0;
			fw_checksum_part += b1;
			fw_checksum_part += b2;
			fw_checksum_part += b3;

			BYTEPTR(v32)[0] = b0; // direct order of bytes (little endian)
			BYTEPTR(v32)[1] = b1;
			BYTEPTR(v32)[2] = b2;
			BYTEPTR(v32)[3] = b3;
			if (FLASH_ProgramWord(adr, v32) == FLASH_COMPLETE) {
				adr += 4;
				src_ofs += 4;
			} else { // error flash write
				FLASH_Lock();
				return false;
			}
		} //for i
	} //if !flash_same...

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock();

	if (!flash_same32(src, adr_ptr))
		return false; // not writed

	// shift address
	flashprog_cur_adr += 32;
	if (flashprog_cur_adr > flashprog_end_adr)
		flashprog_end_adr = flashprog_cur_adr;
	if (flashprog_end_adr > 0x3ffff)
		return false; // memory overflow

	return true;
} // fw_writeflash32_buf

////////////////////////////////////////////////////////////////////////

// saves address of read/write
void fw_writeflashadr(uint32_t d_adr) {
	flashprog_start_adr = d_adr;

	flashprog_cur_adr = flashprog_start_adr;
	flashprog_end_adr = flashprog_start_adr;

	fw_checksum_all = 0;

	// flag of address was saved
	flashprog_adr_seted = FLASHPROG_ADR_SETED_CONST;
} // fw_writeflashadr

////////////////////////////////////////////////////////////////////////

// command to delayed reprogram
bool fw_writeflash_start_delayed(void) {
	if (flashprog_adr_seted != FLASHPROG_ADR_SETED_CONST)
		return false; // address not set

	if (flashprog_end_adr <= flashprog_start_adr)
		return false; // addresses range iz zero

	if (flashprog_end_adr > 0x3ffff)
		return false; // addresses range too large

	if (flashprog_start_adr != 0)
		return false; // start must be zero

	// password to reprogram
	flashprog_needed = FLASHPROG_NEEDED_CONST;

	return true;
} // fw_writeflash_start_delayed

////////////////////////////////////////////////////////////////////////

// module initialization
void fw_update_preinit(void) {
	flashprog_adr_seted = 0;

	flashprog_needed = 0;
	flashprog_needed2 = 0;

	flashprog_delay_cnt = 0;
} // fw_update_preinit

#ifdef __cplusplus
}
#endif

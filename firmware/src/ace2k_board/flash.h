/* What: the only code in the application that erases or programs flash.
 * How: the signatures match struct ace2k_config_ops, so the config binding points at them. Every
 * call is checked against ace2k_guard_flash_allowed() before the controller is touched.
 * Each erase or program stalls the core with interrupts off. ace2k_flash_last_stall() is the
 * record of the last one: the running count, the timer at its start and end, and whether the
 * pending bit of the zero-cross line held an edge at its end. The zero-cross interrupt is its one
 * reader.
 * Depends on: the FMC and EXTI registers through CMSIS (board code, not host-compiled),
 * core/guard.h, heat/mains.h (the record's type). */
#ifndef ACE2K_BOARD_FLASH_H
#define ACE2K_BOARD_FLASH_H
#include <stdint.h>
#include "ace2k/heat/mains.h" // struct ace2k_mains_stall

/* Copies len bytes from the flash address addr. ctx is unused. */
void ace2k_flash_read(void *ctx, uint32_t addr, uint8_t *out, uint32_t len);
/* Erases the page at addr. Returns 0, -ACE2K_EREFUSED (the guard refuses the page) or
 * -ACE2K_EFLASH (the controller reports an error). ctx is unused. */
int ace2k_flash_erase_page(void *ctx, uint32_t addr);
/* Programs one 32-bit word at addr. Returns 0, -ACE2K_EREFUSED (addr not word-aligned, or refused
 * by the guard) or -ACE2K_EFLASH (the controller reports an error). ctx is unused. */
int ace2k_flash_program_word(void *ctx, uint32_t addr, uint32_t word);
/* The record is written with interrupts off, at the end of every erase and program. Read it whole
 * from an interrupt, where no operation can run under it, or with interrupts off. */
void ace2k_flash_last_stall(struct ace2k_mains_stall *out);

#endif

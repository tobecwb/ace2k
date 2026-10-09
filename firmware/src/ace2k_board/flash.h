/* What: the only code in the application that erases or programs flash.
 * How: signatures match struct ace2k_config_ops so the config binding points at them; every
 * call is checked against ace2k_guard_flash_allowed() before the controller is touched.
 * ace2k_flash_last_stall() is the record of the last erase or program — each stalls the core with
 * interrupts off: the running count, the timer at its start and end, and whether the zero-cross
 * line's pending bit held an edge at its end — for the zero-cross interrupt, the one reader.
 * Depends on: the FMC and EXTI registers through CMSIS (board code, not host-compiled),
 * ace2k/guard.h, ace2k/mains.h (the record's type). */
#ifndef ACE2K_BOARD_FLASH_H
#define ACE2K_BOARD_FLASH_H
#include <stdint.h>
#include "ace2k/heat/mains.h" // struct ace2k_mains_stall

void ace2k_flash_read(void *ctx, uint32_t addr, uint8_t *out, uint32_t len);
int ace2k_flash_erase_page(void *ctx, uint32_t addr);
int ace2k_flash_program_word(void *ctx, uint32_t addr, uint32_t word);
/* Written with interrupts off at the end of every erase and program; read whole from an interrupt
 * (no operation can run under it) or with interrupts off. */
void ace2k_flash_last_stall(struct ace2k_mains_stall *out);

#endif

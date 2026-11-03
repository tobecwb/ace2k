/* What: the system facts that a read-only image can state about itself:
 * - the cause of the last reset;
 * - whether the core runs on the PLL from the crystal;
 * - whether the independent watchdog is running (the low-speed oscillator it forces on) and
 *   configured as expected;
 * - the device identifier;
 * - the length and the CRC of the image itself.
 * How: plain register reads through CMSIS. The reset flags are read once and then cleared, so
 * each boot reports its own reset. The image length comes from the linker's symbols, because
 * the .data load image is the last thing in flash. The CRC-32 is the one that tools/mkimage.py
 * appends right after the image, before the 8-byte trailer.
 * Depends on: CMSIS, autoconf.h, core/util.h (board code, not host-compiled). */
#ifndef ACE2K_BOARD_SYSINFO_H
#define ACE2K_BOARD_SYSINFO_H
#include <stdbool.h>
#include <stdint.h>

#define ACE2K_SYSINFO_UID_LEN 12U

/* Returns an enum ace2k_reset_cause. The first call reads and clears the flags. Later calls
 * return the same value. */
uint8_t ace2k_sysinfo_reset_cause(void);
/* True when the core runs on the PLL and the PLL on the external crystal, which is ready. */
bool ace2k_sysinfo_clock_pll_hse(void);
/* True when LSIRDY is set, and PR / RLR hold what Klipper configures. Only the watchdog turns
 * that oscillator on. On this unit the bootloader arms the independent watchdog before it jumps to
 * the application. So the check attests a running watchdog with the expected prescaler and
 * reload, not that this image started it. */
bool ace2k_sysinfo_watchdog_armed(void);
/* Copies the 96-bit device identifier, in the order its bytes lie in the part. */
void ace2k_sysinfo_uid(uint8_t out[ACE2K_SYSINFO_UID_LEN]);
/* The length of the linked image in bytes, rounded up to 4. */
uint32_t ace2k_sysinfo_image_len(void);
/* True when the CRC-32 stored after the image matches the image. It reads the whole image, which
 * takes a few milliseconds, so health calls it once, at boot. */
bool ace2k_sysinfo_image_crc_ok(void);

#endif

/* What: the system facts a read-only image can state about itself — the cause of the last
 * reset, whether the core runs on the PLL from the crystal, whether the independent watchdog
 * is running (the low-speed oscillator it forces on) and configured as expected, the device
 * identifier, the image's own length and CRC.
 * How: plain register reads through CMSIS; the reset flags are read once and then cleared, so
 * each boot reports its own reset; the image length comes from the linker's symbols (the .data
 * load image is the last thing in flash); the CRC-32 is the one tools/mkimage.py appends right
 * after the image, before the 8-byte trailer.
 * Depends on: CMSIS, autoconf.h, ace2k/util.h (board code, not host-compiled). */
#ifndef ACE2K_BOARD_SYSINFO_H
#define ACE2K_BOARD_SYSINFO_H
#include <stdbool.h>
#include <stdint.h>

#define ACE2K_SYSINFO_UID_LEN 12U

/* enum ace2k_reset_cause.  The first call reads and clears the flags; later calls return the
 * same value. */
uint8_t ace2k_sysinfo_reset_cause(void);
bool ace2k_sysinfo_clock_pll_hse(void);
/* LSIRDY set (only the watchdog turns that oscillator on) and PR / RLR as Klipper configures
 * them.  On this unit the bootloader arms the independent watchdog before it jumps to the
 * application, so the check attests a running watchdog with the expected prescaler and reload,
 * not that this image started it. */
bool ace2k_sysinfo_watchdog_armed(void);
void ace2k_sysinfo_uid(uint8_t out[ACE2K_SYSINFO_UID_LEN]);
uint32_t ace2k_sysinfo_image_len(void);
bool ace2k_sysinfo_image_crc_ok(void); /* reads the whole image: a few milliseconds at boot */

#endif

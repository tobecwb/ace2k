// ace2k board system facts.
// - Reset flags: RCC_CSR. GD32F30x User Manual, RCU chapter: the reset source flags of the reset
//   source / clock register (STM32F1-compatible layout, CMSIS names). They are read once,
//   decoded, then cleared with RMVF; see ace2k_sysinfo_reset_cause().
// - Clock: SWS = PLL and PLLSRC = HSE, with HSERDY.
// - Watchdog: the ready flag of the internal low-speed oscillator is the evidence that it runs.
//   No prescaler or reload update may be in flight (status register idle). Then come the values
//   that Klipper's watchdog.c writes (prescaler /4, reload 0xFFF). It is the free watchdog timer
//   of the GD32F30x User Manual, with the same registers.
// - UID: the 96-bit device identifier at UID_BASE.
// - Image length: _data_flash + (_data_end - _data_start) - the application base, rounded up
//   to 4. The .data load image is the last section in flash (Klipper's armcm_link.lds.S). So this
//   is the length of the linked binary that tools/mkimage.py padded to 4 and checksummed.
#include "ace2k_board/sysinfo.h"
#include "ace2k/core/health.h" // enum ace2k_reset_cause
#include "ace2k/core/util.h"   // ace2k_crc32_update, ace2k_crc32_final, ace2k_get_u32_le
#include "autoconf.h"          // CONFIG_FLASH_APPLICATION_ADDRESS
#include "board/internal.h"    // RCC, IWDG, UID_BASE

// The watchdog values that Klipper's stm32/watchdog.c writes as literals: prescaler /4, and the
// full 12-bit reload. The chip glue (stm32/gd32f30x.c) arms the same values before Klipper does.
#define ACE2K_IWDG_PR_DIV4      0U
#define ACE2K_IWDG_RLR_EXPECTED 0x0FFFU
#define ACE2K_IWDG_SR_UPDATING  (IWDG_SR_PVU | IWDG_SR_RVU) /* FWDGT PUD / RUD */
#define ACE2K_IMAGE_ALIGN       4U
#define ACE2K_CRC_CHUNK         256U

// Klipper's linker script defines these three symbols, so they keep its names.
// NOLINTNEXTLINE(bugprone-reserved-identifier,readability-identifier-naming)
extern char _data_flash[], _data_start[], _data_end[];

static uint8_t ace2k_sysinfo_cause; /* enum ace2k_reset_cause, decoded once */
static bool ace2k_sysinfo_cause_read;

static uint8_t decode_reset_flags(uint32_t csr)
{
    if (csr & RCC_CSR_IWDGRSTF) {
        return ACE2K_RESET_WATCHDOG;
    }
    if (csr & RCC_CSR_WWDGRSTF) {
        return ACE2K_RESET_WINDOW_WATCHDOG;
    }
    if (csr & RCC_CSR_LPWRRSTF) {
        return ACE2K_RESET_LOW_POWER;
    }
    if (csr & RCC_CSR_SFTRSTF) {
        return ACE2K_RESET_SOFTWARE;
    }
    if (csr & RCC_CSR_PORRSTF) {
        return ACE2K_RESET_POWER_ON;
    }
    if (csr & RCC_CSR_PINRSTF) {
        return ACE2K_RESET_PIN;
    }
    return ACE2K_RESET_UNKNOWN;
}

// The flags are sticky until a power cycle or a write of RMVF. Left alone, every later boot would
// keep reporting the oldest reset: a software reset after a watchdog reset still shows the
// watchdog flag. So the first call decodes them, keeps the cause, and clears them. Each boot then
// reports its own reset. And the bootloader's own watchdog-flag test sees a fresh flag at the next
// reset.
uint8_t ace2k_sysinfo_reset_cause(void)
{
    if (!ace2k_sysinfo_cause_read) {
        ace2k_sysinfo_cause = decode_reset_flags(RCC->CSR);
        RCC->CSR |= RCC_CSR_RMVF;
        ace2k_sysinfo_cause_read = true;
    }
    return ace2k_sysinfo_cause;
}

bool ace2k_sysinfo_clock_pll_hse(void)
{
    return (bool)((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL &&
                  (RCC->CFGR & RCC_CFGR_PLLSRC) != 0U && (RCC->CR & RCC_CR_HSERDY) != 0U);
}

// Starting the independent watchdog forces the internal low-speed oscillator on (GD32F30x User
// Manual, FWDGT chapter; RCC LSIRDY). Nothing else in this image enables it. So LSIRDY set is
// the evidence that the watchdog is running.
// PR / RLR then say that it is configured as watchdog.c leaves it: /4, 0xFFF. Those are also the
// reset values. But the bootloader arms the watchdog with a longer timeout of its own, before it
// jumps to the application. So they do tell the two apart.
// And the status register must be idle. PUD / RUD still set long after boot mean that a
// prescaler or reload update never reached the watchdog's clock domain. PR / RLR then need
// not be the values in force. That clause is what caught a failure observed on the unit: PR 5,
// RLR 4000, both flags set, on every boot.
// It can also fail with PR / RLR already right. watchdog_init() writes the same values again
// without waiting. If the stream of reloads from its feed task kept that redundant update from
// completing, the flags would stay set over correct values. A failure that reads PR 0 /
// RLR 0xFFF with the flags set would point there, rather than to a lost write. Reading PR, RLR
// and SR on the bench (debug_read) tells the two apart.
// The check attests a running watchdog with the expected prescaler and reload, not that this
// image started it.
bool ace2k_sysinfo_watchdog_armed(void)
{
    return (bool)((RCC->CSR & RCC_CSR_LSIRDY) != 0U && (IWDG->SR & ACE2K_IWDG_SR_UPDATING) == 0U &&
                  IWDG->PR == ACE2K_IWDG_PR_DIV4 && IWDG->RLR == ACE2K_IWDG_RLR_EXPECTED);
}

void ace2k_sysinfo_uid(uint8_t out[ACE2K_SYSINFO_UID_LEN])
{
    const uint8_t *uid = (const uint8_t *)UID_BASE;
    for (uint32_t i = 0; i < ACE2K_SYSINFO_UID_LEN; i++) {
        out[i] = uid[i];
    }
}

uint32_t ace2k_sysinfo_image_len(void)
{
    uint32_t end = (uint32_t)_data_flash + ((uint32_t)_data_end - (uint32_t)_data_start);
    uint32_t len = end - CONFIG_FLASH_APPLICATION_ADDRESS;
    return (len + ACE2K_IMAGE_ALIGN - 1U) & ~(ACE2K_IMAGE_ALIGN - 1U);
}

bool ace2k_sysinfo_image_crc_ok(void)
{
    const uint8_t *base = (const uint8_t *)CONFIG_FLASH_APPLICATION_ADDRESS;
    uint32_t len = ace2k_sysinfo_image_len();
    uint32_t crc = ACE2K_CRC32_INIT;
    for (uint32_t off = 0; off < len; off += ACE2K_CRC_CHUNK) {
        uint32_t n = len - off < ACE2K_CRC_CHUNK ? len - off : ACE2K_CRC_CHUNK;
        crc = ace2k_crc32_update(crc, base + off, n);
    }
    crc = ace2k_crc32_final(crc);
    return crc == ace2k_get_u32_le(base + len); /* stored little-endian by tools/mkimage.py */
}

// Clock setup for the GigaDevice GD32F303 - an STM32F103-compatible part
// with its own clock tree: up to 120MHz, high-driver mode above 100MHz.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ, CONFIG_CLOCK_REF_FREQ
#include "internal.h" // enable_pclock

// The PLL setup below halves the crystal and multiplies it by a whole
// number.  It gives the right clock only with a 24MHz crystal, the one
// the ACE 2 Pro has.  Kconfig offers no other crystal for this part, and
// this check stops a build that gets one anyway.
#define GD32_CLOCK_REF_FREQ 24000000
#if CONFIG_CLOCK_REF_FREQ != GD32_CLOCK_REF_FREQ
#error "gd32f30x: only a 24 MHz crystal is supported"
#endif

// The project's naming and brace rules are off for this file.  It follows
// Klipper's own style, because it is meant for Klipper upstream.
// NOLINTBEGIN(readability-identifier-naming)
// NOLINTBEGIN(readability-braces-around-statements)

// High-driver mode of the power management unit (GD32F30x User Manual,
// PMU chapter).  The STM32F103 CMSIS header has no names for these bits.
// LDOVS[15:14] = 11 selects the LDO high mode.  The high-driver mode needs
// it, and software writes it while the main PLL is off.
#define GD32_PMU_CTL_LDOVS_HIGH (3U << 14U)
#define GD32_PMU_CTL_HDEN (1U << 16U)
#define GD32_PMU_CTL_HDS (1U << 17U)
#define GD32_PMU_CS_HDRF (1U << 16U)
#define GD32_PMU_CS_HDSRF (1U << 17U)
#define GD32_HIGH_DRIVER_MIN_FREQ 100000000

// Up to this core clock, both APB buses run at /2, as on the F1 at 72MHz.
#define GD32_APB_DIV2_MAX_FREQ 72000000

// Flash wait states (FMC chapter): 2 for 60 < HCLK <= 120MHz, on the
// high-density line this part belongs to.  The F1 code uses the same 2 at
// 72MHz.
#define GD32_FLASH_LATENCY 2U

// Free watchdog timer (GD32F30x User Manual, FWDGT chapter).  A write to
// the prescaler or reload register takes a few IRC40K periods to reach
// the low-speed clock domain.  Meanwhile the PUD / RUD flags of the
// status register read 1 (IWDG_SR PVU / RVU in the CMSIS names).  The
// manual says to wait for the flag to clear before writing.  Any key
// other than the unlock key locks the two registers again, so every write
// after a reload starts with a fresh unlock.
#define GD32_FWDGT_KEY_UNLOCK 0x5555
#define GD32_FWDGT_KEY_RELOAD 0xAAAA
#define GD32_FWDGT_KEY_START 0xCCCC
// Prescaler /4 and the full 12-bit reload: the values Klipper's
// stm32/watchdog.c writes, as literals, in watchdog_init().
#define GD32_FWDGT_PSC_DIV4 0
#define GD32_FWDGT_RELOAD 0x0FFF

// Bound on one wait for an update flag.  The early arm runs after
// SystemInit(), which switches the system clock to the internal 8MHz
// oscillator (IRC8M, the clock after reset in the RCU chapter of the
// GD32F30x User Manual).  So the bound is counted in cycles of that
// clock, not CONFIG_CLOCK_FREQ.
// The wait loop is five instructions: a status read on APB1, a shift, a
// branch, a decrement, a branch.  That is about 7-10 cycles, from flash
// at whatever wait states the bootloader left (none to three at 8MHz).
// 40 cycles is a generous ceiling for it.  So a timed-out wait lasts at
// most ~10ms, plus the few percent of the IRC8M tolerance.  It lasts at
// least 2000 * 5 cycles = 1.25ms.  That is several times one update: 5
// IRC40K periods, about 170us at the oscillator's slowest (30kHz),
// plus its start-up after a reset.
// A reload comes before each wait.  So the watchdog in force cannot
// expire in it, even at /4 with a reload of 4000 committed halfway
// (0.27s at the least).
#define GD32_FWDGT_WAIT_MAX_US 10000
#define GD32_IRC8M_MHZ 8
#define GD32_FWDGT_WAIT_LOOP_MAX_CYCLES 40
#define GD32_FWDGT_WAIT_ITERATIONS                                            \
    (GD32_FWDGT_WAIT_MAX_US * GD32_IRC8M_MHZ / GD32_FWDGT_WAIT_LOOP_MAX_CYCLES)

// Reload the watchdog, then wait, bounded, until none of the given update
// flags is set.  A time-out is not reported here.  The flags stay readable
// in the status register for any later check.
static void
fwdgt_reload_and_wait(uint32_t flags)
{
    IWDG->KR = GD32_FWDGT_KEY_RELOAD;
    for (uint32_t n = GD32_FWDGT_WAIT_ITERATIONS; n && (IWDG->SR & flags); n--)
        ;
}

void
clock_setup_gd32f30x(void)
{
    // Arm the independent watchdog first, with the values of Klipper's
    // watchdog.c: prescaler /4 and reload 0xFFF.  That is 273-546ms over
    // the IRC40K's 30-60kHz range.  A hang below then ends in a
    // watchdog reset, whatever watchdog the bootloader left running.  Such
    // a hang is a ready flag that never rises, or too few flash wait
    // states after the switch.  watchdog_init() arms it again later, and
    // its task feeds it.
    //
    // A bootloader may jump with its own prescaler or reload update still
    // in flight, and a write made during that update can be lost.  So each
    // write waits for its update flag (FWDGT chapter: wait before
    // writing).  After the start, the arm waits for both flags to clear.
    // Unless a wait timed out, no write here lands in an update in flight.
    // watchdog_init() writes the same values again without waiting.
    fwdgt_reload_and_wait(IWDG_SR_PVU);
    IWDG->KR = GD32_FWDGT_KEY_UNLOCK;
    IWDG->PR = GD32_FWDGT_PSC_DIV4;
    fwdgt_reload_and_wait(IWDG_SR_RVU);
    IWDG->KR = GD32_FWDGT_KEY_UNLOCK;
    IWDG->RLR = GD32_FWDGT_RELOAD;
    IWDG->KR = GD32_FWDGT_KEY_START;
    fwdgt_reload_and_wait(IWDG_SR_PVU | IWDG_SR_RVU);

    // Start from a known tree: the bootloader may hand over on a PLL,
    // and SystemInit() keeps the extended multiplier bits of CFGR
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;
    RCC->CFGR = 0;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
        ;
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);

    // External crystal
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    if (CONFIG_CLOCK_FREQ > GD32_HIGH_DRIVER_MIN_FREQ) {
        // LDO high mode first, while the PLL is off.  Then enable and
        // wait, then select and wait (user manual, "high-driver mode")
        enable_pclock(PWR_BASE);
        PWR->CR |= GD32_PMU_CTL_LDOVS_HIGH;
        PWR->CR |= GD32_PMU_CTL_HDEN;
        while (!(PWR->CSR & GD32_PMU_CS_HDRF))
            ;
        PWR->CR |= GD32_PMU_CTL_HDS;
        while (!(PWR->CSR & GD32_PMU_CS_HDSRF))
            ;
    }

    // PLL from HSE/2: 12MHz * 6 = 72MHz, 12MHz * 10 = 120MHz
    uint32_t mul = CONFIG_CLOCK_FREQ / (CONFIG_CLOCK_REF_FREQ / 2);
    uint32_t cfgr = ((1U << RCC_CFGR_PLLSRC_Pos) | RCC_CFGR_PLLXTPRE_HSE_DIV2
                     | ((mul - 2) << RCC_CFGR_PLLMULL_Pos));
    if (CONFIG_CLOCK_FREQ > GD32_APB_DIV2_MAX_FREQ)
        // Both APB buses at /4: the in-tree n32g45x precedent
        cfgr |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV4;
    else
        cfgr |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;
    cfgr |= RCC_CFGR_ADCPRE_DIV8;
    RCC->CFGR = cfgr;

    FLASH->ACR
        = ((GD32_FLASH_LATENCY << FLASH_ACR_LATENCY_Pos) | FLASH_ACR_PRFTBE);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;
    RCC->CFGR = cfgr | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
        ;
}

// NOLINTEND(readability-braces-around-statements)
// NOLINTEND(readability-identifier-naming)

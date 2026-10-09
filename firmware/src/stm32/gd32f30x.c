// Clock setup for the GigaDevice GD32F303 - an STM32F103-compatible part
// with its own clock tree: up to 120MHz, high-driver mode above 100MHz.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "internal.h" // enable_pclock

// High-driver mode of the power management unit (GD32F30x User Manual,
// PMU chapter); the STM32F103 CMSIS header has no names for these bits.
// LDOVS[15:14] = 11 selects the LDO high mode, a precondition of the
// high-driver mode, written by software while the main PLL is off.
#define GD32_PMU_CTL_LDOVS_HIGH (3 << 14)
#define GD32_PMU_CTL_HDEN (1 << 16)
#define GD32_PMU_CTL_HDS (1 << 17)
#define GD32_PMU_CS_HDRF (1 << 16)
#define GD32_PMU_CS_HDSRF (1 << 17)
#define GD32_HIGH_DRIVER_MIN_FREQ 100000000

// Flash wait states (FMC chapter): 2 for 60 < HCLK <= 120MHz on the
// high-density line this part belongs to; the F1 code uses the same 2 at
// 72MHz.  If the image faults at 120MHz, try 3 before anything else.
#define GD32_FLASH_LATENCY 2

// Free watchdog timer (GD32F30x User Manual, FWDGT chapter): a write to
// the prescaler or reload register is carried into the low-speed clock
// domain over a few IRC40K periods, while the PUD / RUD flags of the
// status register (IWDG_SR PVU / RVU in the CMSIS names) read 1; the
// manual says to wait for the flag to clear before writing.  Any key
// other than the unlock key locks the two registers again, so every write
// after a reload is preceded by a fresh unlock.
#define GD32_FWDGT_KEY_UNLOCK 0x5555
#define GD32_FWDGT_KEY_RELOAD 0xAAAA
#define GD32_FWDGT_KEY_START 0xCCCC

// Bound on one wait for an update flag.  The early arm runs after
// SystemInit(), which switches the system clock to the internal 8MHz
// oscillator (IRC8M, the clock the GD32F30x User Manual's RCU chapter
// gives after reset), so the bound is counted in cycles of that clock,
// not CONFIG_CLOCK_FREQ.  The wait loop is five instructions (a status
// read on APB1, a shift, a branch, a decrement, a branch): about 7-10
// cycles, from flash at whatever wait states the bootloader left (none
// to three at 8MHz), and 40 cycles is a generous ceiling for it.  Hence
// a timed-out wait lasts at most ~10ms (plus the few percent of the
// IRC8M's tolerance) and at least 2000 * 5 cycles = 1.25ms - several
// times one update (5 IRC40K periods, about 170us at the oscillator's
// slowest, 30kHz, plus its start-up after a reset).  Each wait is
// preceded by a reload, so the watchdog in force - even /4 with a reload
// of 4000 committed halfway, 0.27s at the least - cannot expire in it.
#define GD32_FWDGT_WAIT_MAX_US 10000
#define GD32_IRC8M_MHZ 8
#define GD32_FWDGT_WAIT_LOOP_MAX_CYCLES 40
#define GD32_FWDGT_WAIT_ITERATIONS                                            \
    (GD32_FWDGT_WAIT_MAX_US * GD32_IRC8M_MHZ / GD32_FWDGT_WAIT_LOOP_MAX_CYCLES)

// Reload the watchdog, then wait, bounded, until none of the given update
// flags is set.  A time-out is not reported here: the health check
// (ace2k_sysinfo_watchdog_armed) reads the flags again later.
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
    // Arm the independent watchdog first, exactly as Klipper's
    // watchdog.c does (prescaler /4, reload 0xFFF: 410-512ms), so that
    // a hang anywhere below - a ready flag that never rises, too few
    // flash wait states after the switch - ends in a watchdog reset into
    // the bootloader's recovery within half a second, without depending
    // on the longer watchdog the bootloader arms before the jump
    // (docs/hardware.md).  watchdog_init() re-arms it later and its task
    // feeds it.
    //
    // Observed on the bench (2026-09-27): after the bootloader's jump the
    // watchdog kept the bootloader's prescaler and reload with both
    // update flags set for good; the likely cause is a write made while
    // the bootloader's own update was still in flight.  So each write
    // waits for its flag (FWDGT chapter: wait before writing), and after
    // the start the arm waits for both flags to clear.  What this
    // guarantees: our writes are never made into an update in flight
    // unless a wait timed out.  What it does not: watchdog_init() writes
    // the same values again without waiting; if the final wait here timed
    // out, that write may land in an update in flight, and health bit 28
    // then reports the state.
    fwdgt_reload_and_wait(IWDG_SR_PVU);
    IWDG->KR = GD32_FWDGT_KEY_UNLOCK;
    IWDG->PR = 0;
    fwdgt_reload_and_wait(IWDG_SR_RVU);
    IWDG->KR = GD32_FWDGT_KEY_UNLOCK;
    IWDG->RLR = 0x0FFF;
    IWDG->KR = GD32_FWDGT_KEY_START;
    fwdgt_reload_and_wait(IWDG_SR_PVU | IWDG_SR_RVU);

    // Start from a known tree: the bootloader may hand over on a PLL,
    // and SystemInit() preserves the extended multiplier bits of CFGR
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
        // LDO high mode first (the PLL is off here), then enable, wait;
        // select, wait (user manual, "high-driver mode")
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
    uint32_t cfgr = ((1 << RCC_CFGR_PLLSRC_Pos) | RCC_CFGR_PLLXTPRE_HSE_DIV2
                     | ((mul - 2) << RCC_CFGR_PLLMULL_Pos));
    if (CONFIG_CLOCK_FREQ > 72000000)
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

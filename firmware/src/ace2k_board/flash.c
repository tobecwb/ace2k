// ace2k board flash: page erase and 32-bit word programming through the flash controller,
// refused outside the config page.  Code runs from the same flash, so the core stalls while
// the controller is busy (tens of ms for an erase): no interrupt, and so no timer, runs until
// the operation ends.  Each operation therefore holds interrupts off from unlock to lock and,
// before releasing them, declares the stall to Klipper's timer dispatcher (timer_note_stall,
// patches/README.md), which then catches the overdue timers up instead of shutting down with
// "Rescheduled timer in the past"; the board tick (tick.c) catches its own periods up on that
// pass and keeps now_ms honest.  The caller runs in task context with nothing time-critical
// pending.  Every operation also records itself, still with interrupts off: a running count, the
// timer at its start and at its end, and whether EXTI line 0's pending bit holds a zero-cross
// edge at its end — the edges that fell in the stall are lost but one, which the pending bit
// holds and the zero-cross interrupt takes, late, right after.  That interrupt is the one reader
// (ace2k_flash_last_stall): the edge it takes first after an operation that held one is the late
// edge, by fact, not by timing — the mains measurement and the heater learn of the stall from that
// edge alone.
// Register sequence and unlock keys: GD32F30x User Manual, FMC chapter (page erase, word
// program), EXTI chapter (the pending register); 2 KB pages programmed in 32-bit words is
// docs/hardware.md "Memory map".
#include "ace2k_board/flash.h"
#include "ace2k/core/guard.h" // ace2k_guard_flash_allowed
#include "ace2k/core/util.h"  // ACE2K_E*
#include "board/internal.h"   // FLASH, EXTI, timer_note_stall
#include "board/irq.h"        // irq_disable, irq_enable
#include "board/misc.h"       // timer_read_time

#define FLASH_UNLOCK_KEY1 0x45670123U
#define FLASH_UNLOCK_KEY2 0xCDEF89ABU
#define FLASH_ERRORS      (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)
#define ERASED_WORD       0xFFFFFFFFU

// The dispatcher's catch-up pass runs on the SysTick that stayed pending through the stall,
// right after interrupts come back; the grace only has to outlast that pass (a handful of
// timers, well under a millisecond), not the stall itself.
#define ACE2K_FLASH_STALL_GRACE_US 10000U

static volatile uint32_t ace2k_flash_stall_count;
static volatile uint32_t ace2k_flash_stall_start_time;
static volatile uint32_t ace2k_flash_stall_end_time;
static volatile bool ace2k_flash_stall_held;

// Interrupts go off, then the start is read.
static void stall_begin(void)
{
    irq_disable();
    ace2k_flash_stall_start_time = timer_read_time();
}

// Interrupts still off: the record is whole before any interrupt can run after the stall; the
// end and the pending bit are read last, just before interrupts come back (an edge after that
// read is taken microseconds late at most — on time).  Reading the bit clears nothing.
static void stall_end(void)
{
    timer_note_stall(ACE2K_FLASH_STALL_GRACE_US);
    ace2k_flash_stall_end_time = timer_read_time();
    ace2k_flash_stall_held = (EXTI->PR & EXTI_PR_PR0) != 0U;
    ace2k_flash_stall_count++;
    irq_enable();
}

void ace2k_flash_last_stall(struct ace2k_mains_stall *out)
{
    out->count = ace2k_flash_stall_count;
    out->start = ace2k_flash_stall_start_time;
    out->end = ace2k_flash_stall_end_time;
    out->held = ace2k_flash_stall_held;
}

static void unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_UNLOCK_KEY1;
        FLASH->KEYR = FLASH_UNLOCK_KEY2;
    }
}

static void lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

// Before an operation: wait for the controller, then clear whatever it still reports — the
// bootloader may leave PGERR or WRPRTERR set after its own programming — without a verdict.
// The flags clear on a written 1; writing one that is clear changes nothing.
static void wait_ready(void)
{
    while (FLASH->SR & FLASH_SR_BSY)
        ;
    FLASH->SR = FLASH_ERRORS | FLASH_SR_EOP;
}

// After an operation: wait for the controller and judge only what this operation raised.
static int wait_done(void)
{
    while (FLASH->SR & FLASH_SR_BSY)
        ;
    uint32_t sr = FLASH->SR;
    FLASH->SR = sr & (FLASH_ERRORS | FLASH_SR_EOP);
    return (sr & FLASH_ERRORS) ? -ACE2K_EFLASH : 0;
}

void ace2k_flash_read(void *ctx, uint32_t addr, uint8_t *out, uint32_t len)
{
    (void)ctx;
    const uint8_t *src = (const uint8_t *)addr;
    for (uint32_t i = 0; i < len; i++) {
        out[i] = src[i];
    }
}

int ace2k_flash_erase_page(void *ctx, uint32_t addr)
{
    (void)ctx;
    if (!ace2k_guard_flash_allowed(addr, ACE2K_GUARD_CONFIG_PAGE_SIZE)) {
        return -ACE2K_EREFUSED;
    }
    stall_begin();
    unlock();
    wait_ready();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT;
    int rc = wait_done();
    FLASH->CR &= ~FLASH_CR_PER;
    lock();
    stall_end();
    if (rc != 0) {
        return rc;
    }
    const volatile uint32_t *p = (const volatile uint32_t *)addr;
    for (uint32_t i = 0; i < ACE2K_GUARD_CONFIG_PAGE_SIZE / 4U; i++) {
        if (p[i] != ERASED_WORD) {
            return -ACE2K_EFLASH;
        }
    }
    return 0;
}

int ace2k_flash_program_word(void *ctx, uint32_t addr, uint32_t word)
{
    (void)ctx;
    if ((addr & 3U) != 0 || !ace2k_guard_flash_allowed(addr, 4U)) {
        return -ACE2K_EREFUSED;
    }
    stall_begin();
    unlock();
    wait_ready();
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint32_t *)addr = word;
    int rc = wait_done();
    FLASH->CR &= ~FLASH_CR_PG;
    lock();
    stall_end();
    // The store above is a program request to the controller, not a memory write: the readback
    // is the verdict, which cppcheck's memory model calls settled.
    // cppcheck-suppress knownConditionTrueFalse
    if (rc == 0 && *(volatile uint32_t *)addr != word) {
        rc = -ACE2K_EFLASH;
    }
    return rc;
}

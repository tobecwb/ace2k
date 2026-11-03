// ace2k transmit try: see tx.h. This is the one caller of console_try_sendf from the build patch.
// It is kept out of Klipper's generic command.c, so that a Klipper tree with another console
// still links.
#include "ace2k_board/tx.h"
#include <stdarg.h>           // va_list, va_start, va_end
#include "board/misc.h"       // console_try_sendf
#include "core/quiet.h"       // ace2k_quiet_drop
#include "ace2k_board/tick.h" // ace2k_tick_now_ms
#include "sched.h"            // DECL_SHUTDOWN, sched_is_shutdown

// When Klipper's shutdown began, and the quiet latch (core/quiet.h).
static volatile uint32_t ace2k_tx_shut_ms;
static bool ace2k_tx_quiet;

void ace2k_tx_shutdown(void)
{
    ace2k_tx_shut_ms = ace2k_tick_now_ms();
}
DECL_SHUTDOWN(ace2k_tx_shutdown);

bool ace2k_tx_try_sendf(const struct command_encoder *ce, ...)
{
    if (ace2k_quiet_drop(sched_is_shutdown() != 0, ace2k_tick_now_ms(), ace2k_tx_shut_ms,
                         &ace2k_tx_quiet)) {
        return true; // dropped: a stopped unit is quiet (core/quiet.h)
    }
    va_list args;
    va_start(args, ce);
    bool ok = console_try_sendf(ce, ACE2K_TX_RESPONSE_RESERVE_B, args) != 0;
    va_end(args);
    return ok;
}

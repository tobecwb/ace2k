// ace2k transmit try: see tx.h.  The one caller of the build patch's console_try_sendf, kept
// out of Klipper's generic command.c so that a Klipper tree with another console still links.
#include "core/tx.h"
#include <stdarg.h>     // va_list, va_start, va_end
#include "board/misc.h" // console_try_sendf

bool ace2k_tx_try_sendf(const struct command_encoder *ce, ...)
{
    va_list args;
    va_start(args, ce);
    bool ok = console_try_sendf(ce, ACE2K_TX_RESPONSE_RESERVE_B, args) != 0;
    va_end(args);
    return ok;
}

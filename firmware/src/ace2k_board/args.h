/* What: how a command handler takes a host argument. It checks the argument while it still has
 * all its 32 bits, so a value out of range shuts down and never wraps into a valid one. Klipper's
 * parser keeps every integer whole, even for %c and %hu. So lane=256 arrives as 256, and a plain
 * (uint8_t) cast would turn it into lane 0.
 * How: each macro takes the raw args[n] and a message, a string literal that names the command
 * and the argument ("ace2k_feed_start: lane out of range"). In range, the macro gives the value
 * in the narrow type. Out of range, it calls Klipper's shutdown with the message, which does not
 * return. These are macros because Klipper puts the message in the dictionary at compile time:
 * the message must reach _DECL_STATIC_STR as a literal, as it does in shutdown(). C leaves the
 * order of a call's arguments open, so a handler takes each argument into a local in turn, in the
 * order of the command's format. Then the first argument out of range is the one the message
 * names.
 * Depends on: Klipper's command.h and sched.h only. It knows no lane and no other core bound:
 * lane/lane_args.h builds the lane macros on top of it. It is binding code, not compiled on the
 * host, so it stays off the host tidy list, as ace2k_board/tx.h does.
 * tests/python/test_bindings_ranges.py checks that every handler narrows its arguments through
 * these macros or after its own check. */
#ifndef ACE2K_ARGS_H
#define ACE2K_ARGS_H
#include <stdbool.h>
#include <stdint.h>
#include "command.h" /* _DECL_STATIC_STR */
#include "sched.h"   /* sched_shutdown */

/* The argument when it is at most max_value; otherwise a shutdown with the message msg_id. */
static inline uint32_t ace2k_arg_max(uint32_t arg, uint32_t max_value, uint_fast8_t msg_id)
{
    if (arg > max_value) {
        sched_shutdown(msg_id);
    }
    return arg;
}

/* A byte from 0 to MAX. MAX is a constant of at most 255, checked when the code is compiled.
 * The check casts MAX to 32 bits first, so a negative MAX is a huge one and fails it too. */
#define ACE2K_ARG_U8_MAX(ARG, MAX, MSG)                                                            \
    ({                                                                                             \
        _Static_assert((uint32_t)(MAX) <= UINT8_MAX, "ACE2K_ARG_U8_MAX: MAX above 255");           \
        (uint8_t)ace2k_arg_max((ARG), (MAX), _DECL_STATIC_STR(MSG));                               \
    })
/* A byte: %c. */
#define ACE2K_ARG_U8(ARG, MSG) ACE2K_ARG_U8_MAX((ARG), UINT8_MAX, MSG)
/* A 16-bit value: %hu. */
#define ACE2K_ARG_U16(ARG, MSG) ((uint16_t)ace2k_arg_max((ARG), UINT16_MAX, _DECL_STATIC_STR(MSG)))
/* A flag: 0 or 1, as bool. */
#define ACE2K_ARG_FLAG(ARG, MSG) (ACE2K_ARG_U8_MAX((ARG), 1U, MSG) != 0U)

#endif

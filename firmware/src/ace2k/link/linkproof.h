/* What: the first-boot link proof — a freshly flashed image that nobody speaks to within a
 * window asks to be reset into the bootloader's recovery, so a valid-but-silent image is
 * recoverable over the wire; an image that has been spoken to once never asks again.
 * How: the binding calls ace2k_linkproof_init() with the loaded config record and the running
 * version string, ace2k_linkproof_tick() every tick (acts on ENTER_RECOVERY by halting), and
 * ace2k_linkproof_proven() when the host's first ace2k_version query arrives.
 * Contexts: `state` is a volatile single word written from two contexts, one transition each.
 * The tick runs in the interrupt and makes WAITING -> EXPIRED; ace2k_linkproof_proven() runs
 * in task context and makes WAITING -> PROVEN — but only after re-reading `state` once its
 * store returned: the store stalls the core, the tick that was pending through the stall runs
 * before the store returns, and a deadline that passed meanwhile wins (the proof is refused,
 * the halt the tick requested stands).  init() runs before either.
 * Depends on: <stdbool.h>, <stdint.h>, config.h (the record), util.h. */
#ifndef ACE2K_LINKPROOF_H
#define ACE2K_LINKPROOF_H
#include <stdbool.h>
#include <stdint.h>
#include "core/config.h"

/* How long an unproven image waits for the host before it gives itself up to recovery.  A human
 * is in the loop after a flash; a longer window costs only a longer wait when the image is
 * deaf.  initial value; adjust on the bench. */
#define ACE2K_LINKPROOF_WINDOW_MS 180000U

enum ace2k_linkproof_state {
    ACE2K_LINKPROOF_PROVEN,  /* the record names this image: no window runs */
    ACE2K_LINKPROOF_WAITING, /* the window runs */
    ACE2K_LINKPROOF_EXPIRED, /* the window ran out: recovery has been requested */
};

enum ace2k_linkproof_action {
    ACE2K_LINKPROOF_RUN,
    ACE2K_LINKPROOF_ENTER_RECOVERY,
};

/* store_proof persists "proven by `version`" and returns 0 or -ACE2K_E<NAME>. */
struct ace2k_linkproof_ops {
    int (*store_proof)(void *ctx, const char *version);
};

struct ace2k_linkproof {
    const struct ace2k_linkproof_ops *ops;
    void *ctx;
    const char *running_version;
    volatile enum ace2k_linkproof_state state; /* see "Contexts" above */
    uint32_t deadline_ms;
};

/* Decides PROVEN (stored flag set and stored version equal to running_version) or WAITING, and
 * arms the window from now_ms.  running_version must outlive self. */
void ace2k_linkproof_init(struct ace2k_linkproof *self, const struct ace2k_linkproof_ops *ops,
                          void *ctx, const struct ace2k_config_record *stored,
                          const char *running_version, uint32_t now_ms);

/* Advances the window.  ENTER_RECOVERY from the deadline on, every call, while unproven. */
enum ace2k_linkproof_action ace2k_linkproof_tick(struct ace2k_linkproof *self, uint32_t now_ms);

/* The host spoke.  Stores the proof once; 0 when proven (already or now), the store's error
 * when it failed (state unchanged), -ACE2K_EREFUSED after expiry — also when the window expired
 * during the store itself (state stays EXPIRED; the record on flash is harmless, the host did
 * speak to this image). */
int ace2k_linkproof_proven(struct ace2k_linkproof *self);

enum ace2k_linkproof_state ace2k_linkproof_state(const struct ace2k_linkproof *self);

/* Milliseconds left in the window; 0 when proven, expired or past the deadline. */
uint32_t ace2k_linkproof_remaining_ms(const struct ace2k_linkproof *self, uint32_t now_ms);

#endif

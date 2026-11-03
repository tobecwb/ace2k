/* What: the back-off of a store that failed on the flash: no retry before the wait has passed.
 * How: the binding owns one struct; the tick asks ace2k_backoff_allowed(), and the task reports
 * each store's result with the tick masked, so the tick never reads the struct half-written.
 * Depends on: core/util.h (ace2k_time_after). */
#ifndef ACE2K_BACKOFF_H
#define ACE2K_BACKOFF_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

/* The wait starts at MIN and doubles at each failure, up to MAX. A page that cannot be written
 * is then not erased a hundred times a second, and the same data is still tried again. */
#define ACE2K_BACKOFF_MIN_MS 1000U
#define ACE2K_BACKOFF_MAX_MS 60000U

struct ace2k_backoff {
    bool failed;       /* the last store failed on the flash */
    uint32_t retry_ms; /* no retry before this, while failed */
    uint32_t wait_ms;  /* the wait after the next failure */
};

#define ACE2K_BACKOFF_INIT { .failed = false, .retry_ms = 0U, .wait_ms = ACE2K_BACKOFF_MIN_MS }

/* True when a store may be tried at now_ms. */
static inline bool ace2k_backoff_allowed(const struct ace2k_backoff *b, uint32_t now_ms)
{
    if (!b->failed) {
        return true;
    }
    return ace2k_time_after(now_ms, b->retry_ms);
}

/* A store failed on the flash at now_ms. A refusal that is not a flash failure (a lease) is not
 * reported at all. */
static inline void ace2k_backoff_failed(struct ace2k_backoff *b, uint32_t now_ms)
{
    b->retry_ms = now_ms + b->wait_ms;
    b->failed = true;
    b->wait_ms = b->wait_ms > ACE2K_BACKOFF_MAX_MS / 2U ? ACE2K_BACKOFF_MAX_MS : b->wait_ms * 2U;
}

/* A store succeeded: the failure clears and the first wait comes back. */
static inline void ace2k_backoff_succeeded(struct ace2k_backoff *b)
{
    b->failed = false;
    b->wait_ms = ACE2K_BACKOFF_MIN_MS;
}

#endif

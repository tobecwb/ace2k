/* What: the link proof at first boot.  A freshly flashed image that nobody speaks to within a
 * window asks to be reset into the recovery of the bootloader.  So an image that is valid but
 * never hears from the host can still be recovered over the wire.  An image that the host has
 * spoken to once never asks again.
 * How: the binding calls ace2k_linkproof_init() with its live config record and the running version
 * string.  It calls ace2k_linkproof_tick() on every tick, with its sample of the host link, and
 * halts when the tick answers ENTER_RECOVERY.  It calls ace2k_linkproof_proven() when the
 * ace2k_version query of the host arrives.  The store may be refused as busy, because the heater
 * holds the flash.  Then the proof waits for a retry.  While ace2k_linkproof_retry_wanted() is
 * true, the tick wakes a task once the store may run, and the task calls ace2k_linkproof_retry().
 * The host spoke, so the window does not expire while that retry waits and the host keeps talking,
 * even past its deadline.  Once the link is lost past the deadline, the window expires, and stays
 * expired if the host comes back.
 * Contexts: `state` and `retry_pending` are volatile single words.  The tick runs in the
 * interrupt.  It moves WAITING to EXPIRED, and it only reads retry_pending.
 * ace2k_linkproof_proven() and ace2k_linkproof_retry() run in task context.  They write
 * retry_pending and move WAITING to PROVEN.  They store first, with the tick free.  The store
 * stalls the core, and a tick that was pending through the stall runs before the store returns.
 * Then they read `state` again and write PROVEN between hold_tick and release_tick, so no tick
 * runs between the read and the write.  A deadline that passed before that read wins.  The proof
 * is refused, and the halt that the tick requested stands.  A later tick finds PROVEN and asks for
 * nothing.  So a halt is never requested while the state is PROVEN.  The record, was_version and
 * was_flags are used by the task only, and no interrupt reads them.  init() runs before either.
 * Depends on: <stdbool.h>, <stdint.h>, config.h (the record), util.h. */
#ifndef ACE2K_LINKPROOF_H
#define ACE2K_LINKPROOF_H
#include <stdbool.h>
#include <stdint.h>
#include "core/config.h"

/* How long an unproven image waits for the host before it hands itself over to recovery.  The
 * flasher proves the image itself right after it writes it.  It asks for the version, as Klipper
 * does at connect.  So the window only has to last longer than the boot of an image that cannot
 * hear, plus the flasher's questions.  A unit flashed after another is proven by its own flash,
 * whatever Klipper does.  The value is 180 s, as it was at first.  It was 600 s while only Klipper
 * proved an image, because Klipper had to be started by hand within the window. */
#define ACE2K_LINKPROOF_WINDOW_MS 180000U

enum ace2k_linkproof_state {
    ACE2K_LINKPROOF_PROVEN,  /* the record names this image, so no window runs */
    ACE2K_LINKPROOF_WAITING, /* the window runs */
    ACE2K_LINKPROOF_EXPIRED, /* the window ran out, and recovery has been requested */
};

enum ace2k_linkproof_action {
    ACE2K_LINKPROOF_RUN,
    ACE2K_LINKPROOF_ENTER_RECOVERY,
};

/* store writes the config record to the flash, as the record is now.  It runs in task context.
 * It returns 0 on success.  It returns -ACE2K_EBUSY when the store is refused for now, and then
 * nothing is written.  Any other failure is another -ACE2K_E<NAME>.
 * hold_tick keeps the tick from running until release_tick.  A tick that comes in between runs at
 * the release.  The pair surrounds a few instructions only, and never nests.  The store never runs
 * inside it. */
struct ace2k_linkproof_ops {
    int (*store)(void *ctx);
    void (*hold_tick)(void *ctx);
    void (*release_tick)(void *ctx);
};

struct ace2k_linkproof {
    const struct ace2k_linkproof_ops *ops;
    void *ctx;
    struct ace2k_config_record *record; /* the live record that store writes */
    const char *running_version;
    volatile enum ace2k_linkproof_state state; /* see "Contexts" above */
    volatile bool retry_pending;               /* the host spoke, but the store was refused busy */
    uint32_t deadline_ms;
    char was_version[ACE2K_CONFIG_VERSION_LEN]; /* the record's version before a store */
    uint32_t was_flags;                         /* the record's flags before a store */
};

/* Sets up the proof and starts the window at now_ms.  The state is PROVEN when the record has its
 * proven flag set and its version equals running_version.  Otherwise it is WAITING.  record is the
 * live record that ops->store writes, and the proof goes into it.  record and running_version must
 * live longer than self. */
void ace2k_linkproof_init(struct ace2k_linkproof *self, const struct ace2k_linkproof_ops *ops,
                          void *ctx, struct ace2k_config_record *record,
                          const char *running_version, uint32_t now_ms);

/* Advances the window.  While the image is unproven, every call from the deadline on returns
 * ENTER_RECOVERY.  A proof refused busy holds that off while its retry waits and link_ok is true,
 * because the host spoke and keeps talking.  link_ok is the binding's judgement of the host link
 * (a valid frame within the link-loss time).  It counts only on the calls past the deadline, each
 * on its own: a link lost before the deadline and back by then holds as if it had never gone.
 * The first call past the deadline with link_ok false expires the window.  When the retry ends
 * without a proof (a flash failure), the next call past the deadline expires it too.  Expired is
 * final. */
enum ace2k_linkproof_action ace2k_linkproof_tick(struct ace2k_linkproof *self, uint32_t now_ms,
                                                 bool link_ok);

/* Records that the host spoke.  It writes the proof into the record and stores it, once.  The proof
 * is the running version and the proven flag.  It returns 0 when the image is proven, already or
 * now.  A failed store leaves the record and the state as they were, and returns the store's error.
 * On -ACE2K_EBUSY, a retry also waits.  It returns -ACE2K_EREFUSED after the window expired.  This
 * includes a window that expired during the store itself.  Then the state stays EXPIRED.  The proof
 * on the flash and in the record does no harm, because the host did speak to this image. */
int ace2k_linkproof_proven(struct ace2k_linkproof *self);

/* Returns true while a proof that was refused busy waits for its retry, and the window has not
 * expired.  That holds past the deadline too, while the host keeps talking, because the waiting
 * retry then keeps the window from expiring.  It may be called from any context. */
bool ace2k_linkproof_retry_wanted(const struct ace2k_linkproof *self);

/* Stores again a proof that was refused busy.  It works as ace2k_linkproof_proven() does, with
 * the same results.  It returns -ACE2K_EINVAL when no refused proof waits.  So a retry never
 * proves an image that the host never spoke to.  A flash failure ends the retries.  The next query
 * of the host tries again. */
int ace2k_linkproof_retry(struct ace2k_linkproof *self);

enum ace2k_linkproof_state ace2k_linkproof_state(const struct ace2k_linkproof *self);

/* Returns the milliseconds left in the window.  It returns 0 when the image is proven, expired or
 * past the deadline, including past the deadline with a retry that waits. */
uint32_t ace2k_linkproof_remaining_ms(const struct ace2k_linkproof *self, uint32_t now_ms);

#endif

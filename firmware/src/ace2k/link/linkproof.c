#include "link/linkproof.h"
#include "core/util.h"

static bool record_names(const struct ace2k_config_record *stored, const char *version)
{
    if ((stored->flags & ACE2K_CONFIG_FLAG_LINK_PROVEN) == 0) {
        return false;
    }
    return ace2k_str_equal(stored->proven_version, version);
}

/* Copies every byte of the field, so that a refused store puts back exactly what was there. */
static void copy_version(char *dst, const char *src)
{
    for (size_t i = 0; i < ACE2K_CONFIG_VERSION_LEN; i++) {
        dst[i] = src[i];
    }
}

/* The store writes the record as it is, so the proof must be in the record for the call.  If the
 * store fails, the record gets back what it had.  Otherwise the next store of another module would
 * write a proof that this store could not write.  Nothing else reads the record meanwhile.  Only
 * tasks use the record, and this runs in one task call.  The saved copy lives in self.  It is not
 * on the stack, because the tasks share the stack with every interrupt. */
static int store_proof(struct ace2k_linkproof *self)
{
    struct ace2k_config_record *rec = self->record;
    self->was_flags = rec->flags;
    copy_version(self->was_version, rec->proven_version);
    ace2k_str_copy(rec->proven_version, ACE2K_CONFIG_VERSION_LEN, self->running_version);
    rec->flags |= ACE2K_CONFIG_FLAG_LINK_PROVEN;
    int rc = self->ops->store(self->ctx);
    if (rc != 0) {
        copy_version(rec->proven_version, self->was_version);
        rec->flags = self->was_flags;
    }
    return rc;
}

void ace2k_linkproof_init(struct ace2k_linkproof *self, const struct ace2k_linkproof_ops *ops,
                          void *ctx, struct ace2k_config_record *record,
                          const char *running_version, uint32_t now_ms)
{
    self->ops = ops;
    self->ctx = ctx;
    self->record = record;
    self->running_version = running_version;
    self->state =
        record_names(record, running_version) ? ACE2K_LINKPROOF_PROVEN : ACE2K_LINKPROOF_WAITING;
    self->retry_pending = false;
    self->deadline_ms = now_ms + ACE2K_LINKPROOF_WINDOW_MS;
}

enum ace2k_linkproof_action ace2k_linkproof_tick(struct ace2k_linkproof *self, uint32_t now_ms,
                                                 bool link_ok)
{
    if (self->state == ACE2K_LINKPROOF_PROVEN) {
        return ACE2K_LINKPROOF_RUN;
    }
    /* A proof refused busy means the host spoke, so the window does not expire while it waits and
     * the host keeps talking.  A host gone silent past the deadline holds nothing. */
    if (self->state == ACE2K_LINKPROOF_WAITING && (!self->retry_pending || !link_ok) &&
        ace2k_time_after(now_ms, self->deadline_ms)) {
        self->state = ACE2K_LINKPROOF_EXPIRED;
    }
    return self->state == ACE2K_LINKPROOF_EXPIRED ? ACE2K_LINKPROOF_ENTER_RECOVERY
                                                  : ACE2K_LINKPROOF_RUN;
}

int ace2k_linkproof_proven(struct ace2k_linkproof *self)
{
    if (self->state == ACE2K_LINKPROOF_PROVEN) {
        return 0;
    }
    if (self->state == ACE2K_LINKPROOF_EXPIRED) {
        return -ACE2K_EREFUSED;
    }
    int rc = store_proof(self);
    if (rc != 0) {
        /* Only a busy refusal is retried, because the flash is free again soon.  A flash failure
         * is not retried here.  So a page that cannot be written is not erased again and again
         * by the unit: only the next ace2k_version tries again.  The host asks it once per
         * connect, and the flasher once per proof.
         * A refused proof that waits also keeps the window from expiring while the host keeps
         * talking (see the tick).  A failure ends that, so a window already past expires on the
         * next tick. */
        self->retry_pending = rc == -ACE2K_EBUSY;
        return rc;
    }
    /* The store stalled the core.  The tick that ran when the store ended may have expired the
     * window.  Then the halt is already on its way, so refuse, and leave the state to say so.  The
     * check and the write run with the tick held, so the tick cannot expire the window between
     * them.  A retry that waits is cleared there too.  Cleared before, it would let a tick past
     * the deadline expire the window that this store just proved. */
    self->ops->hold_tick(self->ctx);
    bool expired = self->state == ACE2K_LINKPROOF_EXPIRED;
    if (!expired) {
        self->state = ACE2K_LINKPROOF_PROVEN;
    }
    self->retry_pending = false;
    self->ops->release_tick(self->ctx);
    return expired ? -ACE2K_EREFUSED : 0;
}

bool ace2k_linkproof_retry_wanted(const struct ace2k_linkproof *self)
{
    if (!self->retry_pending) {
        return false;
    }
    return self->state == ACE2K_LINKPROOF_WAITING;
}

int ace2k_linkproof_retry(struct ace2k_linkproof *self)
{
    if (!self->retry_pending) {
        return -ACE2K_EINVAL;
    }
    return ace2k_linkproof_proven(self);
}

enum ace2k_linkproof_state ace2k_linkproof_state(const struct ace2k_linkproof *self)
{
    return self->state;
}

uint32_t ace2k_linkproof_remaining_ms(const struct ace2k_linkproof *self, uint32_t now_ms)
{
    if (self->state != ACE2K_LINKPROOF_WAITING || ace2k_time_after(now_ms, self->deadline_ms)) {
        return 0;
    }
    return self->deadline_ms - now_ms;
}

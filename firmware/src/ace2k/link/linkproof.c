#include "link/linkproof.h"
#include "core/util.h"

static bool record_names(const struct ace2k_config_record *stored, const char *version)
{
    if ((stored->flags & ACE2K_CONFIG_FLAG_LINK_PROVEN) == 0) {
        return false;
    }
    return ace2k_str_equal(stored->proven_version, version);
}

void ace2k_linkproof_init(struct ace2k_linkproof *self, const struct ace2k_linkproof_ops *ops,
                          void *ctx, const struct ace2k_config_record *stored,
                          const char *running_version, uint32_t now_ms)
{
    self->ops = ops;
    self->ctx = ctx;
    self->running_version = running_version;
    self->state =
        record_names(stored, running_version) ? ACE2K_LINKPROOF_PROVEN : ACE2K_LINKPROOF_WAITING;
    self->deadline_ms = now_ms + ACE2K_LINKPROOF_WINDOW_MS;
}

enum ace2k_linkproof_action ace2k_linkproof_tick(struct ace2k_linkproof *self, uint32_t now_ms)
{
    if (self->state == ACE2K_LINKPROOF_PROVEN) {
        return ACE2K_LINKPROOF_RUN;
    }
    if (self->state == ACE2K_LINKPROOF_WAITING && ace2k_time_after(now_ms, self->deadline_ms)) {
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
    int rc = self->ops->store_proof(self->ctx, self->running_version);
    if (rc != 0) {
        return rc;
    }
    /* The store stalled the core; the tick that ran when it ended may have expired the window.
     * The halt is then already on its way: refuse, and leave the state to say so. */
    if (self->state == ACE2K_LINKPROOF_EXPIRED) {
        return -ACE2K_EREFUSED;
    }
    self->state = ACE2K_LINKPROOF_PROVEN;
    return 0;
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

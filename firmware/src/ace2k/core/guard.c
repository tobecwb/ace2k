#include "core/guard.h"
#include "core/util.h"

void ace2k_guard_init(struct ace2k_guard *self)
{
    self->veto_count = 0;
    for (size_t i = 0; i < ACE2K_GUARD_VETO_SLOTS; i++) {
        self->veto_fn[i] = NULL;
        self->veto_ctx[i] = NULL;
    }
}

bool ace2k_guard_flash_allowed(uint32_t addr, uint32_t len)
{
    if (len == 0 || len > ACE2K_GUARD_CONFIG_PAGE_SIZE) {
        return false;
    }
    if (addr < ACE2K_GUARD_CONFIG_PAGE_ADDR) {
        return false;
    }
    /* Subtraction first: addr + len could wrap at the top of the address space. */
    return addr - ACE2K_GUARD_CONFIG_PAGE_ADDR <= ACE2K_GUARD_CONFIG_PAGE_SIZE - len;
}

int ace2k_guard_register_veto(struct ace2k_guard *self, ace2k_guard_veto_fn fn, void *ctx)
{
    if (fn == NULL) {
        return -ACE2K_EINVAL;
    }
    if (self->veto_count >= ACE2K_GUARD_VETO_SLOTS) {
        return -ACE2K_EFULL;
    }
    self->veto_fn[self->veto_count] = fn;
    self->veto_ctx[self->veto_count] = ctx;
    self->veto_count++;
    return 0;
}

bool ace2k_guard_bootloader_allowed(const struct ace2k_guard *self)
{
    bool allowed = true;
    for (size_t i = 0; i < self->veto_count; i++) {
        if (self->veto_fn[i](self->veto_ctx[i])) {
            allowed = false;
        }
    }
    return allowed;
}

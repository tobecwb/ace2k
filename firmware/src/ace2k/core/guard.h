/* What: the allow-lists that keep the image from touching what it must not — flash outside the
 * ace2k config page, and bootloader entry while a subsystem objects.
 * How: ace2k_guard_flash_allowed() is a pure predicate every flash routine calls before it
 * touches the controller; one struct ace2k_guard, owned by the guard binding, collects veto
 * callbacks and answers ace2k_guard_bootloader_allowed().
 * Depends on: <stdbool.h>, <stddef.h>, <stdint.h>, util.h. */
#ifndef ACE2K_GUARD_H
#define ACE2K_GUARD_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The one page ace2k may erase and program: the last 2 KB of the application slot
 * (docs/hardware.md, memory map). */
#define ACE2K_GUARD_CONFIG_PAGE_ADDR 0x08023800U
#define ACE2K_GUARD_CONFIG_PAGE_SIZE 2048U

/* The factory firmware's calibration pages, the two 2 KB pages immediately below the
 * application slot (docs/hardware.md "Memory map"): read by config, never written — the
 * allow-list below does not contain them, and test_guard proves it. */
#define ACE2K_GUARD_FACTORY_PAGE_PRIMARY_ADDR 0x08007000U
#define ACE2K_GUARD_FACTORY_PAGE_BACKUP_ADDR  0x08006800U
#define ACE2K_GUARD_FACTORY_PAGE_SIZE         2048U

#define ACE2K_GUARD_VETO_SLOTS 8

/* A veto returns true while its subsystem must not lose the firmware — a heater on, a lane
 * moving.  ctx is what the subsystem registered. */
typedef bool (*ace2k_guard_veto_fn)(void *ctx);

struct ace2k_guard {
    ace2k_guard_veto_fn veto_fn[ACE2K_GUARD_VETO_SLOTS];
    void *veto_ctx[ACE2K_GUARD_VETO_SLOTS];
    size_t veto_count;
};

/* Empties the veto registry. */
void ace2k_guard_init(struct ace2k_guard *self);

/* True when [addr, addr + len) lies entirely inside the config page and len is not zero.
 * Pure; safe from any context. */
bool ace2k_guard_flash_allowed(uint32_t addr, uint32_t len);

/* Adds a veto.  -ACE2K_EINVAL for a NULL callback, -ACE2K_EFULL when every slot is taken. */
int ace2k_guard_register_veto(struct ace2k_guard *self, ace2k_guard_veto_fn fn, void *ctx);

/* True when no registered veto objects.  Every veto is asked, in registration order. */
bool ace2k_guard_bootloader_allowed(const struct ace2k_guard *self);

#endif

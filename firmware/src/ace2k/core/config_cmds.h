/* What: the one config instance of the image, on the board's flash, and the unit's factory
 * calibration read from its pages.
 * How: any binding calls ace2k_config_binding() and gets the loaded record; the first call
 * loads it (no reliance on init order).  ace2k_config_binding_factory() reads the factory
 * pages the same way, once.  Task context only — a store blocks on the flash.  The command
 * ace2k_config_state publishes both states.  Every store goes through
 * ace2k_config_binding_store(), which a registered veto can refuse (the heater's lease).
 * Depends on: config.h, ace2k_board/flash.h (binding: Klipper-side code). */
#ifndef ACE2K_CONFIG_CMDS_H
#define ACE2K_CONFIG_CMDS_H
#include <stdbool.h>
#include "core/config.h"

struct ace2k_config *ace2k_config_binding(void);

/* The unit's factory calibration, read from its pages on the first call (task context). */
const struct ace2k_factory_calibration *ace2k_config_binding_factory(void);

/* A store veto: true while the flash must not be written — the heater's gate is leased
 * (a page erase stalls the core, the zero-cross interrupt
 * with it).  One slot. */
typedef bool (*ace2k_config_store_veto_fn)(void *ctx);

/* 0, -ACE2K_EINVAL (NULL), -ACE2K_EFULL (the slot is taken). */
int ace2k_config_binding_register_store_veto(ace2k_config_store_veto_fn fn, void *ctx);

/* The one way the image stores its record: the veto asked first (-ACE2K_EBUSY while it
 * objects) and the store marked in progress, both with interrupts masked; then
 * ace2k_config_store(), and the mark lifted.  Task context only. */
int ace2k_config_binding_store(void);

/* True from the veto's consent until the store returns: a lease must not begin (the dryer's rule 13). */
bool ace2k_config_binding_storing(void);

#endif

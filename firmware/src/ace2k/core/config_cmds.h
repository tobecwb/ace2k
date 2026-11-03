/* What: the image's one config instance, on the board's flash, and the factory
 * calibration of the unit, read from its pages.
 * How: any binding calls ace2k_config_binding() and gets the loaded record. The first call loads
 * it, so nothing relies on the init order. ace2k_config_binding_factory() reads the factory pages
 * the same way, once. Task context only, because a store blocks on the flash. The exception is
 * ace2k_config_binding_store_vetoed(), which any context may ask. The command ace2k_config_state
 * publishes both states. Every store goes through ace2k_config_binding_store(), which a registered
 * veto can refuse: the heater's lease.
 * Depends on: config.h, ace2k_board/flash.h (binding: Klipper-side code). */
#ifndef ACE2K_CONFIG_CMDS_H
#define ACE2K_CONFIG_CMDS_H
#include <stdbool.h>
#include "core/config.h"

/* The image's one config instance, loaded from the config page on first use. */
struct ace2k_config *ace2k_config_binding(void);

/* The unit's factory calibration, read from its pages on the first call (task context). */
const struct ace2k_factory_calibration *ace2k_config_binding_factory(void);

/* A store veto: true while the flash must not be written, because the heater's gate is
 * leased. A page erase stalls the core, and the zero-cross interrupt with it. One slot. */
typedef bool (*ace2k_config_store_veto_fn)(void *ctx);

/* Returns 0, -ACE2K_EINVAL (NULL), or -ACE2K_EFULL (the slot is taken). */
int ace2k_config_binding_register_store_veto(ace2k_config_store_veto_fn fn, void *ctx);

/* True while the registered veto objects, so a store now would be refused with -ACE2K_EBUSY.
 * Any context: the tick of the link proof asks it before it wakes its task for a store. */
bool ace2k_config_binding_store_vetoed(void);

/* The one way the image stores its record. With interrupts masked, it asks the veto first
 * (-ACE2K_EBUSY while it objects), and marks the store in progress. Then it runs
 * ace2k_config_store() and lifts the mark. Task context only. */
int ace2k_config_binding_store(void);

/* True from the veto's consent until the store returns. A lease must not begin then,
 * because the flash is written only with the gate idle. */
bool ace2k_config_binding_storing(void);

#endif

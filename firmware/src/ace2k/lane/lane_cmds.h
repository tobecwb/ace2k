/* What: the one lane instance of the image — the four filament encoders, the four motor tach
 * counters and, with CONFIG_ACE2K_MOTOR, the four motors behind ace2k_board/motor.c.
 * How: the tick samples link_ok from the serial driver once and feeds the core every 10 ms with
 * it; a task sends the periodic report; ace2k_lane_binding() hands the instance to the feed
 * binding and ace2k_lane_binding_link_ok() the tick's sample; ace2k_lane_binding_reset() zeroes
 * counters, refused while the lane moves or the registered reset veto objects — the feed binding
 * registers one, so this binding never looks upward at the feed.  The shutdown handler stops
 * every motor.  Compiled only with CONFIG_ACE2K_LANE; callers guard their calls with the same
 * flag.
 * Depends on: lane.h, ace2k_board/encoder.h, ace2k_board/fg.h, ace2k_board/motor.h,
 * ace2k_board/serial.h, ace2k_board/tick.h, guard_cmds.h, Klipper's sched and command. */
#ifndef ACE2K_LANE_CMDS_H
#define ACE2K_LANE_CMDS_H
#include <stdbool.h>
#include "lane/lane.h"

/* The instance, initialised on the first call (the feed binding calls it from its own init so
 * that the lane's tick slot precedes the feed's). */
struct ace2k_lane *ace2k_lane_binding(void);

/* The link_ok the last lane tick used.  For the feed tick, which runs in the same interrupt
 * right after the lane's: one sample per tick for both cores. */
bool ace2k_lane_binding_link_ok(void);

/* A counters-reset veto: true while the lane (or, for ACE2K_LANE_ALL, any lane) has state the
 * counters are the base of — the feed's odometers.  Called from command context under the
 * binding's mask with the lane the reset names, only ever 0..3 or ACE2K_LANE_ALL (the binding
 * refuses any other lane before asking); a plain read, no initialisation. */
typedef bool (*ace2k_lane_reset_veto_fn)(void *ctx, uint8_t lane);

/* Registers the one reset veto, at init.  -ACE2K_EINVAL for a NULL callback, -ACE2K_EFULL when
 * the slot is taken. */
int ace2k_lane_binding_register_reset_veto(ace2k_lane_reset_veto_fn fn, void *ctx);

/* 0; -ACE2K_EINVAL for a lane that is neither 0..3 nor ACE2K_LANE_ALL; -ACE2K_EREFUSED, nothing
 * changed, while the lane (for ACE2K_LANE_ALL, any lane) moves or the reset veto objects. */
int ace2k_lane_binding_reset(uint8_t lane);

#endif

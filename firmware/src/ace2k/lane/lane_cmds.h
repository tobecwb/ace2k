/* What: the one lane instance of the image.  It holds the four filament encoders and the four motor
 * tach counters.  With CONFIG_ACE2K_MOTOR it also holds the four motors behind ace2k_board/motor.c.
 * How: every 10 ms, the tick samples link_ok from the serial driver once and feeds the core with
 * it.  A task sends the periodic report.  ace2k_lane_binding() hands the instance to the feed
 * binding, and ace2k_lane_binding_link_ok() hands over the tick's sample.
 * ace2k_lane_binding_reset() zeroes counters.  The reset is refused while the lane moves, or while
 * the registered reset veto objects.  The feed binding registers that veto, so this binding never
 * looks upward at the feed.  The shutdown handler stops every motor.  This file is compiled only
 * with CONFIG_ACE2K_LANE, and callers guard their calls with the same flag.
 * Depends on: lane.h, ace2k_board/encoder.h, ace2k_board/fg.h, ace2k_board/motor.h,
 * ace2k_board/serial.h, ace2k_board/tick.h, guard_cmds.h, Klipper's sched and command. */
#ifndef ACE2K_LANE_CMDS_H
#define ACE2K_LANE_CMDS_H
#include <stdbool.h>
#include "lane/lane.h"

/* Returns the instance, initialised on the first call.  The feed binding calls it from its own
 * init, so that the lane's tick slot comes before the one of the feed. */
struct ace2k_lane *ace2k_lane_binding(void);

/* Returns the link_ok that the last lane tick used.  It is for the feed tick, which runs in the
 * same interrupt right after the lane tick.  So both cores use one sample per tick. */
bool ace2k_lane_binding_link_ok(void);

/* A veto on a counters reset.  It returns true while the lane has state that the counters are the
 * base of, such as the feed's odometers.  For ACE2K_LANE_ALL, it returns true when any lane has
 * such state.  It is called from command context under the binding's mask, with the lane that the
 * reset names.  That lane is only ever 0..3 or ACE2K_LANE_ALL, because the binding refuses any
 * other lane before asking.  It is a plain read, with no initialisation. */
typedef bool (*ace2k_lane_reset_veto_fn)(void *ctx, uint8_t lane);

/* Registers the one reset veto, at init.  It returns -ACE2K_EINVAL for a NULL callback, and
 * -ACE2K_EFULL when the slot is taken. */
int ace2k_lane_binding_register_reset_veto(ace2k_lane_reset_veto_fn fn, void *ctx);

/* Resets the counters of a lane.  It returns 0 on success.  It returns -ACE2K_EINVAL for a lane
 * that is neither 0..3 nor ACE2K_LANE_ALL.  It returns -ACE2K_EREFUSED, with nothing changed, while
 * the lane moves or the reset veto objects.  For ACE2K_LANE_ALL, any lane that moves refuses it. */
int ace2k_lane_binding_reset(uint8_t lane);

#endif

/* What: how a command handler takes a lane from the host.  A lane is 0..3, or 255 for every lane
 * where the command allows it.  A value out of range shuts down with the message of the handler, as
 * the macros of ace2k_board/args.h do.  It never wraps into a valid lane.
 * How: ACE2K_ARG_LANE is ACE2K_ARG_U8_MAX with the highest lane as its bound.
 * ACE2K_ARG_LANE_OR_ALL also takes ACE2K_LANE_ALL.  Each gives the lane as uint8_t.
 * Depends on: ace2k_board/args.h, core/util.h for the lane count, and lane/lane.h for
 * ACE2K_LANE_ALL.  It is binding code, not compiled on the host, so it stays off the host tidy
 * list, as ace2k_board/args.h does. */
#ifndef ACE2K_LANE_ARGS_H
#define ACE2K_LANE_ARGS_H
#include <stdint.h>
#include "ace2k_board/args.h" /* ACE2K_ARG_U8_MAX, _DECL_STATIC_STR, sched_shutdown */
#include "core/util.h"        /* ACE2K_LANE_COUNT */
#include "lane/lane.h"        /* ACE2K_LANE_ALL */

/* Returns the argument when it is a lane, 0..3, or ACE2K_LANE_ALL.  Any other value shuts down. */
static inline uint8_t ace2k_arg_lane_or_all(uint32_t arg, uint_fast8_t msg_id)
{
    if (arg >= ACE2K_LANE_COUNT && arg != ACE2K_LANE_ALL) {
        sched_shutdown(msg_id);
    }
    return (uint8_t)arg;
}

/* A lane: 0..3. */
#define ACE2K_ARG_LANE(ARG, MSG) ACE2K_ARG_U8_MAX((ARG), ACE2K_LANE_COUNT - 1U, MSG)
/* A lane, or ACE2K_LANE_ALL (255) for every lane. */
#define ACE2K_ARG_LANE_OR_ALL(ARG, MSG) ace2k_arg_lane_or_all((ARG), _DECL_STATIC_STR(MSG))

#endif

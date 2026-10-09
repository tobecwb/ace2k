/* What: the private seam between feed.c — the modes, the tick, the events —, feed_start.c — the
 * start command —, feed_compare.c — the motor × filament comparator —, feed_judge.c — the
 * verdicts from the buffer and the counters —, feed_assist.c — the assists —, feed_load.c — the
 * load and its tag search — and feed_snag.c — the tip snag's tolerance: what each needs from the
 * others.
 * How: included by those seven files only; the names carry the module's prefix because they are
 * not static, not because they are anyone's interface — the bindings and the tests use feed.h.
 * Depends on: <stdbool.h>, <stdint.h>, feed.h. */
#ifndef ACE2K_FEED_INTERNAL_H
#define ACE2K_FEED_INTERNAL_H
#include <stdbool.h>
#include <stdint.h>
#include "feed/feed.h"

/* The lane's bit of a per-lane input mask. */
static inline bool ace2k_feed_bit(uint8_t mask, uint8_t lane)
{
    return (((uint32_t)mask >> lane) & 1U) != 0;
}

/* feed.c: a mode the feed entered whose motor may run — every mode but idle and error. */
bool ace2k_feed_mode_runs(uint8_t mode);

/* feed.c: the lane enters a mode: the odometers' and the comparator's bases taken, the assist's
 * and the tail's state cleared.  seq: the start's, or 0 for a mode the tick enters on its own —
 * the retry key is the start's to record, and a mode of the unit's own must not touch it. */
void ace2k_feed_enter(struct ace2k_feed *self, uint8_t lane, uint8_t mode, uint8_t seq,
                      uint32_t now_ms);

/* feed.c: the lane stopped by the feed itself, its own stopped result dropped — not an outcome
 * to report. */
void ace2k_feed_stop_discard(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed.c: a notice — the event with the mode's odometers, the mode goes on. */
void ace2k_feed_notice(struct ace2k_feed *self, uint8_t lane, uint8_t kind);

/* feed.c: the mode ends: the lane stops, the event goes out with the mode it happened in and
 * the mode's odometers, the lane is idle. */
void ace2k_feed_finish(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms);

/* feed.c: an error the feed decides while the lane still moves (the feed's supervisors): the motor stopped first, then the event with the mode's odometers,
 * then error.  The lane's seq stays: the event carries the start that failed, as every later one
 * of the errored lane does until clear. */
void ace2k_feed_fail(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms);

/* feed_compare.c: the comparator's two bases taken afresh — at a mode's start, through a load's
 * grace, and at each of the assist's bursts. */
void ace2k_feed_compare_restart(struct ace2k_feed *self, uint8_t lane);

/* The comparator's verdict on a tick: which trigger tripped, if any.  The kind, and whether a
 * trip is an error at all, is the mode's to decide — the comparator reads counters, not the
 * buffer. */
enum ace2k_feed_trip {
    ACE2K_FEED_TRIP_NONE = 0,
    ACE2K_FEED_TRIP_STANDSTILL = 1, /* stall_check_um of motor since the strand last moved */
    ACE2K_FEED_TRIP_PARTIAL = 2,    /* slip_check_um of motor with the strand short by too much */
};

/* feed_compare.c: the comparator, once per lane per tick after the lane's result of the tick was
 * handled; ACE2K_FEED_TRIP_NONE for a lane in idle or error, or whose motor is not commanded, or
 * in a load's grace. */
enum ace2k_feed_trip ace2k_feed_compare(struct ace2k_feed *self, uint8_t lane);

/* feed_judge.c: this lane's buffer full — the shared pulled reading with this lane's own plunger
 * off its rest and not at its pushed end; a plunger at rest, or taut, is not the one at its
 * pulled end, so the full is another lane's and no verdict on this one. */
bool ace2k_feed_lane_full(const struct ace2k_feed_inputs *in, uint8_t lane);

/* feed_judge.c: this lane's buffer full while its mode pushes toward the head with the motor
 * running — the trigger of the tip snag's tolerance (feed_snag.c), stuck at once when the mode's
 * one tolerance is spent. */
bool ace2k_feed_full_ahead(const struct ace2k_feed *self, uint8_t lane,
                           const struct ace2k_feed_inputs *in);

/* feed_judge.c: a push toward the head that met what cannot move — a feed, a load's pull — ends
 * blocked: the lane idle, no error.  True when it did; false in any
 * other mode or phase, whose verdict is the caller's. */
bool ace2k_feed_block(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_judge.c: the comparator tripped — the kind from the buffer, or the tail past the drive. */
void ace2k_feed_on_trip(struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_trip trip,
                        const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_judge.c: the lane's deadline passed — true when it was the tail past the drive (a reverse
 * mode, the plunger at rest) and the mode goes on or waits; false when it is the timeout it
 * looks like, for the caller to fail. */
bool ace2k_feed_tail_after_deadline(struct ace2k_feed *self, uint8_t lane,
                                    const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_judge.c: the tail's budget, every tick of a mode whose tail_out is set. */
void ace2k_feed_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_assist.c: the insert fell in the follow — its tail begins: the flag and its base taken, a
 * take-up running stopped (the tail takes up nothing), the tail notice. */
void ace2k_feed_follow_tail_begin(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_judge.c: the follow's tail bound, every tick of a lane in its tail — true when
 * follow.tail_um of motor since the tail began ended the mode as tail_out. */
bool ace2k_feed_follow_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_assist.c: the assist commands — the forward assist, the reverse assist and the follow:
 * the speed within the lane's bounds, else bounds; the lane armed, and its first move started at
 * once when the last tick's readings ask for one (a burst on taut and no shared full, a take-up
 * on this lane's full — the follow either). */
enum ace2k_feed_refusal ace2k_feed_assist_start(struct ace2k_feed *self, uint8_t lane,
                                                enum ace2k_feed_cmd cmd, uint32_t speed_um_s,
                                                uint8_t seq, uint32_t now_ms);

/* feed_load.c: the load command — the parking distance and the speed the command's, else the
 * configured ones, within the lane's bounds; the settle begins. */
enum ace2k_feed_refusal ace2k_feed_load_start(struct ace2k_feed *self, uint8_t lane,
                                              uint32_t length_um, uint32_t speed_um_s, uint8_t seq,
                                              uint32_t now_ms);

/* feed_load.c: a load entered on the tick (the automatic load: seq 0). */
void ace2k_feed_begin_load(struct ace2k_feed *self, uint8_t lane, uint32_t park_um,
                           uint32_t speed_um_s, uint8_t seq, uint32_t now_ms);

/* feed_load.c: the load's step on the tick: the settle, the pause on a session, the search's
 * ends, the release of a pause. */
void ace2k_feed_load_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms);

/* feed_load.c: the load's running move ended done — true when the load goes on (the search, the
 * return, the resume after a reacquire), false when it is over (the caller emits loaded). */
bool ace2k_feed_load_move_done(struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_load.c: a start given to `lane` makes any other lane's search yield (the tag search's rule 4). */
void ace2k_feed_yield_others(struct ace2k_feed *self, uint8_t lane);

/* feed_assist.c: the autonomous behaviours' tick for one lane, after the lane's result and the
 * link were handled: the automatic load on the insert's rising edge (rose: the edge, on a lane
 * that was idle before this tick's result), then the mode's own step — the load's (feed_load.c),
 * the forward assist's bursts and its taut episode, the reverse assist's take-up, the follow's
 * bursts and take-ups with its taut episode.  Nothing starts with the link down. */
void ace2k_feed_auto_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          bool rose, uint32_t now_ms);

/* feed.c: the mode's odometers — the tach and the encoder since the mode started. */
void ace2k_feed_odometers(const struct ace2k_feed *self, uint8_t lane, uint32_t *motor_um,
                          int32_t *filament_um);

/* feed.c: a notice with odometers taken earlier — the snag's, from where its tip caught. */
void ace2k_feed_notice_at(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t motor_um,
                          int32_t filament_um);

/* feed_snag.c: a tolerance runs on the lane (any phase but none). */
bool ace2k_feed_snag_active(const struct ace2k_feed *self, uint8_t lane);

/* feed_snag.c: the tolerance's step on the tick, in place of the at-once stuck on this lane's
 * full: true when it owns this tick — a phase ran, began, or ended the mode — and the caller
 * asks nothing more (the comparator waits for the move it resumes). */
bool ace2k_feed_snag_step(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms);

/* feed_snag.c: a lane result during a tolerance — true when it was the tolerance's (the touch's
 * end, or a move that reached its length in the watch).  Never called for link_lost or
 * shutdown, which end the mode as they end any. */
bool ace2k_feed_snag_on_result(struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_lane_result *r, uint32_t now_ms);

/* feed_snag.c: a comparator trip toward the spool with the strand taut — true when it is the
 * tip catching the buffer's tube on its way back (a rollback or an unload, the insert present,
 * the strand back free_um in the mode, the tolerance unused): the touch forward has started. */
bool ace2k_feed_snag_reverse(struct ace2k_feed *self, uint8_t lane,
                             const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_load.c: the phase the snag interrupted — the pull or the tag search — goes on for what
 * is left of it: the snag's resume in a load. */
void ace2k_feed_load_resume_after_snag(struct ace2k_feed *self, uint8_t lane,
                                       const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_load.c: the tag search ends and returns to the parking point — no error:
 * found when the lane's tag is read on the tick's inputs, else on a
 * yield when one is pending (the tag search's rule 4), else on an obstacle.  The snag's end in the search: its
 * verdicts that are stuck elsewhere, and a yield that
 * arrives during the tolerance. */
void ace2k_feed_load_search_return(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

#endif

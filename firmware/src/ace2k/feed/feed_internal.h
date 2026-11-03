/* What: what the source files of the feed need from each other.  Each comment names the file that
 * has the function.
 * How: only those files include it.  The bindings and the tests use feed.h instead.
 * Depends on: <stdbool.h>, <stdint.h>, feed.h. */
#ifndef ACE2K_FEED_INTERNAL_H
#define ACE2K_FEED_INTERNAL_H
#include <stdbool.h>
#include <stdint.h>
#include "feed/feed.h"

/* The lane's bit in an input mask with one bit per lane. */
static inline bool ace2k_feed_bit(uint8_t mask, uint8_t lane)
{
    return (((uint32_t)mask >> lane) & 1U) != 0;
}

/* The motor's travel since a base taken from ace2k_lane_fg_raw(), in micrometres. */
static inline uint32_t ace2k_feed_motor_since_um(const struct ace2k_feed *self, uint8_t lane,
                                                 uint32_t fg_base_pulses)
{
    return ace2k_lane_fg_to_um(ace2k_lane_fg_raw(self->lane, lane) - fg_base_pulses);
}

/* The signed travel of the filament on the encoder since a base, in micrometres, + toward the
 * head. */
static inline int32_t ace2k_feed_strand_since_um(const struct ace2k_feed *self, uint8_t lane,
                                                 int32_t base_um)
{
    return ace2k_lane_encoder_um(self->lane, lane) - base_um;
}

/* The travel of the filament since a base along a direction, in micrometres.  It is negated for a
 * move toward the spool.  A filament that moves the other way reads as none. */
static inline uint32_t ace2k_feed_strand_along_um(const struct ace2k_feed *self, uint8_t lane,
                                                  int32_t base_um, bool reverse)
{
    int32_t travel = ace2k_feed_strand_since_um(self, lane, base_um);
    int32_t along = reverse ? -travel : travel;
    return along > 0 ? (uint32_t)along : 0;
}

/* A rollback or an unload: a mode whose command takes the filament back toward the spool. */
static inline bool ace2k_feed_mode_rewinds(uint8_t mode)
{
    return (bool)(mode == ACE2K_FEED_ROLLING_BACK || mode == ACE2K_FEED_UNLOADING);
}

/* feed.c: a mode the feed entered whose motor may run.  That is every mode but idle and error. */
bool ace2k_feed_mode_runs(uint8_t mode);

/* feed.c: the lane enters a mode.  The bases of the odometers and of the comparator are taken, and
 * the state of the assist and of the tail is cleared.  seq is the start's seq, or 0 for a mode that
 * the tick enters on its own.  Only a start records the retry key, and a mode of the unit itself
 * must not touch it. */
void ace2k_feed_enter(struct ace2k_feed *self, uint8_t lane, uint8_t mode, uint8_t seq,
                      uint32_t now_ms);

/* feed.c: the feed itself stops the lane, and drops the lane's stopped result, because that is not
 * an outcome to report. */
void ace2k_feed_stop_discard(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_events.c: puts an event onto the ring, with the lane's seq and the odometers given.  mode is
 * the lane's mode, except for the runout that ends an error, which names the mode that failed.  A
 * full ring counts the event as dropped. */
void ace2k_feed_push_event(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint8_t mode,
                           uint32_t motor_um, int32_t filament_um);

/* feed_events.c: an event with the mode's odometers read now.  It is a notice, and the mode goes
 * on, or a final event that the caller follows with the end of the mode. */
void ace2k_feed_notice(struct ace2k_feed *self, uint8_t lane, uint8_t kind);

/* feed.c: the mode ends.  The lane stops, the event goes out with the mode it happened in and the
 * odometers of the mode, and the lane is idle. */
void ace2k_feed_finish(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms);

/* feed.c: an error that the feed decides while the lane still moves (the feed's supervisors).  The
 * motor stops first, then the event goes out with the mode's odometers, then the lane is in error.
 * The lane's seq stays.  So the event carries the start that failed, as every later event of the
 * lane in error does until clear. */
void ace2k_feed_fail(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms);

/* feed_compare.c: takes the two bases of the comparator afresh.  This happens at the start of a
 * mode, through the grace of a load, and at each burst of the assist. */
void ace2k_feed_compare_restart(struct ace2k_feed *self, uint8_t lane);

/* The comparator's verdict on a tick: which trigger tripped, if any.  The mode decides the kind,
 * and whether a trip is an error at all, because the comparator reads counters, not the buffer. */
enum ace2k_feed_trip {
    ACE2K_FEED_TRIP_NONE = 0,
    ACE2K_FEED_TRIP_STANDSTILL = 1, /* stall_check_um of motor since the filament last moved */
    ACE2K_FEED_TRIP_PARTIAL = 2,    /* slip_check_um of motor with the filament short by too much */
};

/* feed_compare.c: the comparator, run once per lane per tick, after the tick's lane result was
 * handled.  It returns ACE2K_FEED_TRIP_NONE for a lane in idle or error, for a lane whose motor is
 * not commanded, and in the grace of a load. */
enum ace2k_feed_trip ace2k_feed_compare(struct ace2k_feed *self, uint8_t lane);

/* feed_judge.c: this lane's buffer is full.  That is the shared pulled reading, with this lane's
 * own plunger off its rest and not at its pushed end.  A plunger at rest, or taut, is not the one
 * at its pulled end.  Then the full belongs to another lane, and is no verdict on this one. */
bool ace2k_feed_lane_full(const struct ace2k_feed_inputs *in, uint8_t lane);

/* feed_judge.c: this lane's buffer is full while its mode pushes toward the head with the motor
 * running.  This triggers the tip snag's tolerance (feed_snag.c).  It is stuck at once when the
 * mode's one tolerance is spent.  With a free tip, it is the free phase at every full. */
bool ace2k_feed_full_ahead(const struct ace2k_feed *self, uint8_t lane,
                           const struct ace2k_feed_inputs *in);

/* feed_judge.c: the mode of the lane pushes toward the head, and its motor runs forward.  That is a
 * feed, or the pull of a load, or its tag search.  It is false once the push paused, ended or
 * turned back. */
bool ace2k_feed_pushing_ahead(const struct ace2k_feed *self, uint8_t lane);

/* feed_judge.c: a push toward the head that met something that cannot move ends blocked, with the
 * lane idle and no error.  The push is a feed, or the pull of a load.  It returns true when it did.
 * It returns false in any other mode or phase, and then the caller gives the verdict. */
bool ace2k_feed_block(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_judge.c: the comparator tripped.  The kind comes from the buffer, or the trip means the end
 * of the filament is past the drive. */
void ace2k_feed_on_trip(struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_trip trip,
                        const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_judge.c: the deadline of the lane passed, r being the move's result.  It returns true when
 * that was the end of the filament past the drive.  In the follow's tail, once the filament has
 * counted ACE2K_FEED_TAIL_ENC_UM since the tail began and stood still through the move while the
 * motor turned, the mode ends as tail_out.  In a reverse mode with the plunger at rest the mode
 * goes on or waits.  It returns false when it is the timeout that it looks like, for the caller to
 * fail. */
bool ace2k_feed_tail_after_deadline(struct ace2k_feed *self, uint8_t lane,
                                    const struct ace2k_lane_result *r,
                                    const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_judge.c: the tail's budget, every tick of a mode whose tail_out is set. */
void ace2k_feed_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_follow.c: the insert fell in the follow, so its tail begins.  The flag is set and its base
 * taken.  A take-up that runs is stopped, because the tail takes up nothing.  The tail notice goes
 * out. */
void ace2k_feed_follow_tail_begin(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_judge.c: the bound of the tail of the follow, every tick of a lane in its tail.  It returns
 * true when follow.tail_um of motor since the tail began ended the mode as tail_out. */
bool ace2k_feed_follow_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_assist.c: a move of the assist itself: a burst, a take-up, or a dose.  It is bounded, and
 * takes the bases of the comparator afresh.  lands is false for a move without the lane's landing,
 * a reverse dose.  It returns false when the lane refused the move because it still moves.  The
 * next tick asks again. */
bool ace2k_feed_assist_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                            uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms, bool lands);

/* feed_assist.c: a burst or a take-up.  It is ace2k_feed_assist_move(), counted in bursts. */
void ace2k_feed_assist_counted_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                                    uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms);

/* feed_assist.c: one forward burst, at most ACE2K_FEED_ASSIST_BURST_UM, at the armed speed. */
void ace2k_feed_assist_burst(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* feed_follow.c: the follow's step on the tick.  It handles the ends of its moves, the ledger and
 * the doses of the feed-forward, and its corrections on taut and on this lane's full.  It also runs
 * at the arming, on the last tick's readings. */
void ace2k_feed_follow_step(struct ace2k_feed *self, uint8_t lane,
                            const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_assist.c: the assist commands, which are the forward assist, the reverse assist and the
 * follow.  The speed must be within the lane's bounds, else the answer is bounds.  The lane is
 * armed.  Its first move starts at once when the readings of the last tick ask for one.  That is a
 * burst on taut with no shared full, or a take-up on this lane's full.  The follow takes either. */
enum ace2k_feed_refusal ace2k_feed_assist_start(struct ace2k_feed *self, uint8_t lane,
                                                enum ace2k_feed_cmd cmd, uint32_t speed_um_s,
                                                uint8_t seq, uint32_t now_ms);

/* feed_start.c: the refusals that come before every start, in their order: the lane, the link,
 * the error state, a busy lane, the insert sensors unknown, and no filament at the bay for a
 * command that needs one (all but the rollback).  ACE2K_FEED_ACCEPTED when none applies. */
enum ace2k_feed_refusal ace2k_feed_start_refusal(const struct ace2k_feed *self, uint8_t lane,
                                                 enum ace2k_feed_cmd cmd);

/* feed_stop.c: hands the stop that the host armed to the lane's next start, and clears it, whatever
 * the start's answer.  It returns none for a lane out of range. */
struct ace2k_feed_stop_on ace2k_feed_stop_take(struct ace2k_feed *self, uint8_t lane);

/* feed_stop.c: the step of the armed stop on the tick, ahead of the comparator, the tip snag and
 * the tails.  It returns true when it ended the mode, held or at rest.  It returns false at once
 * for a mode that is not armed. */
bool ace2k_feed_stop_step(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms);

/* feed_stop.c: a comparator trip on a move toward the spool armed for held, before
 * ace2k_feed_on_trip judges it.  It returns true when it ended the move held, with the plunger taut
 * and the filament still by the comparator's own measure.  It also returns true when it puts a
 * partial trip off to the next tick.  A filament that stopped only lately is not yet still by the
 * standstill's base.  It returns false for any other trip, because a filament that moves but slips
 * is for the comparator. */
bool ace2k_feed_stop_on_trip(struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_trip trip,
                             const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_load.c: the load command.  The parking distance and the speed are those of the command, else
 * the configured ones, within the lane's bounds.  The settle begins. */
enum ace2k_feed_refusal ace2k_feed_load_start(struct ace2k_feed *self, uint8_t lane,
                                              uint32_t length_um, uint32_t speed_um_s, uint8_t seq,
                                              uint32_t now_ms);

/* feed_load.c: a load from the bay is entered, either the automatic load (seq 0) or the load
 * command.  Its tip is free. */
void ace2k_feed_begin_load(struct ace2k_feed *self, uint8_t lane, uint32_t park_um,
                           uint32_t speed_um_s, uint8_t seq, uint32_t now_ms);

/* feed_load.c: the load's step on the tick: the settle, the pause on a session, the ends of the
 * search, and the release of a pause. */
void ace2k_feed_load_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms);

/* feed_load.c: the load's running move ended done.  It returns true when the load goes on: the
 * search, the return, or the resume after a reacquire.  It returns false when the load is over, and
 * then the caller emits loaded. */
bool ace2k_feed_load_move_done(struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_load.c: a start given to `lane` makes the search of any other lane yield (the tag search's
 * rule 4). */
void ace2k_feed_yield_others(struct ace2k_feed *self, uint8_t lane);

/* feed_assist.c: the tick of the autonomous behaviours for one lane, after the lane result and the
 * link were handled.  First the automatic load on the insert's rising edge.  rose is that edge, on
 * a lane that was idle before this tick's result.  Then the mode's own step: the load's step
 * (feed_load.c), the forward assist's bursts and its taut episode, the reverse assist's take-up,
 * and the bursts and take-ups of the follow with its taut episode.  Nothing starts with the link
 * down. */
void ace2k_feed_auto_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          bool rose, uint32_t now_ms);

/* feed_events.c: the mode's odometers, the tach and the encoder since the mode started. */
void ace2k_feed_odometers(const struct ace2k_feed *self, uint8_t lane, uint32_t *motor_um,
                          int32_t *filament_um);

/* feed_events.c: a notice with odometers taken earlier, such as those of the snag, from where its
 * tip caught. */
void ace2k_feed_notice_at(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t motor_um,
                          int32_t filament_um);

/* feed_snag.c: a tolerance owns the moves of the lane: the watch, the touch, the wait for rest.
 * The free phase never does, because it only judges.  So the tick of a load from the bay, a pause
 * for a read session or a yield, runs through it.  The watch of the read with motion holds the tick
 * of its load. */
bool ace2k_feed_snag_active(const struct ace2k_feed *self, uint8_t lane);

/* feed_snag.c: the tolerance's step on the tick, in place of the immediate stuck on this lane's
 * full.  It returns true when it owns this tick: a phase ran, began, or ended the mode.  Then the
 * caller asks nothing more, and the comparator waits for the move that it resumes.  The free phase
 * owns a tick only when it ended the push.  Otherwise it returns false, and the comparator judges
 * the tick too. */
bool ace2k_feed_snag_step(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms);

/* feed_snag.c: a lane result during a tolerance.  It returns true when the result belonged to the
 * tolerance.  That is the end of the touch, a move that reached its length in the watch, or a stall
 * or a deadline in the watch or the free phase of the search.  It is never called for link_lost or
 * shutdown, which end the mode as they end any mode. */
bool ace2k_feed_snag_on_result(struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_lane_result *r, uint32_t now_ms);

/* feed_snag.c: a comparator trip toward the spool with the filament taut.  It returns true when it
 * is the tip catching the buffer's tube on its way back, and then the forward touch has started.
 * That needs a rollback or an unload, the insert present, the filament back free_um in the mode,
 * and the tolerance unused. */
bool ace2k_feed_snag_reverse(struct ace2k_feed *self, uint8_t lane,
                             const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_load.c: the phase that the watch of the read with motion interrupted, its tag search, goes
 * on for what is left of it.  This is the snag's resume in a load.  A load from the bay never makes
 * the touch. */
void ace2k_feed_load_resume_after_snag(struct ace2k_feed *self, uint8_t lane,
                                       const struct ace2k_feed_inputs *in, uint32_t now_ms);

/* feed_load.c: the tag search ends and returns to the parking point, with no error.  It ends found
 * when the lane's tag is read on the tick's inputs.  Else it ends on a yield when one is pending
 * (the tag search's rule 4), else on an obstacle.  The watch, or the free phase, ends this way in
 * the search: their verdicts that are blocked or stuck elsewhere, a tag read by the time the
 * plunger is back at rest, and a yield that arrives during them. */
void ace2k_feed_load_search_return(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

#endif

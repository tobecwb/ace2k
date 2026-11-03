/* What: the dryer's automatic modes: the humidity mode (the cold box, the hold above the limit,
 * each own cycle's outcome, the failure count and the stop) and the print mode's bound cycle.
 * How: the binding owns one struct beside the dryer and ticks it right after ace2k_dryer_tick().
 * Depends on: <stdbool.h>, <stdint.h>, dryer.h, dryer_auto_store.h. */
#ifndef ACE2K_DRYER_AUTO_H
#define ACE2K_DRYER_AUTO_H
#include <stdbool.h>
#include <stdint.h>

#include "dryer/dryer.h"
#include "dryer/dryer_auto_store.h"

/* The bounds of the host's dry_humidity_limit and dry_max_failures (docs/protocol.md), so the
 * unit refuses what the host refuses. A limit of 0 % would start on any reading; a maximum of 0
 * would stop the mode before its first cycle. */
#define ACE2K_DRYER_AUTO_LIMIT_MIN_PCT 5U
#define ACE2K_DRYER_AUTO_LIMIT_MAX_PCT 95U
#define ACE2K_DRYER_AUTO_FAILURES_MIN  1U
#define ACE2K_DRYER_AUTO_FAILURES_MAX  20U
#define ACE2K_DRYER_AUTO_HOLD_MS       1800000U /* 30 min at or above the limit, the box cold */
/* The cold box: the dryer idle, and the chamber steady over the window: it neither fell nor rose
 * by COLD_BAND or more. On the recorded cool-downs this reads cold 1.9-3.9 h after the dryer went
 * idle (tests/data/dryer-modes/README.md). Initial values; re-tune on the bench. */
#define ACE2K_DRYER_AUTO_COLD_WINDOW_MS 1800000U
#define ACE2K_DRYER_AUTO_COLD_BAND_MC   500
#define ACE2K_DRYER_AUTO_SAMPLES        31U     /* the window's samples, both ends included */
#define ACE2K_DRYER_AUTO_LINK_MS        300000U /* 5 min of link down end a bound cycle */
/* Readings invalid this long empty the cold-box window and end the hold. A shorter glitch only
 * pauses them. Initial value; re-tune on the bench. */
#define ACE2K_DRYER_AUTO_INVALID_MS 60000U
/* The bench's overrides go no lower: at 1 s a window's 30 sample intervals (33 ms) are still
 * longer than the 10 ms tick. */
#define ACE2K_DRYER_AUTO_MIN_TIMING_MS 1000U
#define ACE2K_DRYER_AUTO_PCT10_PER_PCT 10U

/* The module decides like a user: it starts and stops the dryer through ace2k_dryer_start(),
 * ace2k_dryer_retarget() and ace2k_dryer_stop(), lowers an automatic cycle through
 * ace2k_dryer_lower_target(), and reads the dryer's state. */
struct ace2k_dryer_auto {
    struct ace2k_dryer *dryer;
    struct ace2k_dryer_auto_store *store; /* the working copy of the page's block */
    /* gen counts the changes that must reach the page; stored_gen is the last one the page
     * holds (ace2k_dryer_auto_stored) */
    uint32_t gen, stored_gen;
    /* timing: the defaults, or the bench's overrides (RAM only) */
    uint32_t hold_ms, window_ms, link_ms;
    /* the cold-box detector: a ring of chamber samples; ring_idx is the next slot to write (the
     * oldest once the ring is full), ring_n the samples in it */
    int32_t ring_mc[ACE2K_DRYER_AUTO_SAMPLES];
    uint8_t ring_idx, ring_n;
    uint32_t sample_next_ms; /* no sample before this, once the ring holds one */
    bool cold;
    /* the readings invalid with the dryer idle, since when (ACE2K_DRYER_AUTO_INVALID_MS) */
    bool invalid;
    uint32_t invalid_since_ms;
    /* the box at or above the limit while cold */
    bool above;
    uint32_t above_since_ms;
    /* the cycles */
    bool own;         /* the running cycle is the humidity mode's */
    bool bound;       /* the running cycle is bound to the host */
    bool outcome_due; /* an own cycle completed; its outcome waits for a cold box */
    bool link_down;
    uint32_t link_down_since_ms;
    uint8_t prev_state;  /* the dryer's state at the last tick, to see a cycle end */
    uint8_t insert_prev; /* the insert mask at the last tick, to see a strand enter */
    bool have_prev;      /* false until the first tick has set insert_prev */
    bool off_pending;    /* a manual stop turned the mode off; AUTO_OFF goes out on the next tick */
    bool stop_pending; /* the mode was changed with its cycle running; it stops on the next tick */
    /* the last valid RH read with the dryer idle, for the status; a cycle does not refresh it */
    uint16_t rh_pct10;
};

struct ace2k_dryer_auto_status {
    uint8_t mode, limit_pct, max_failures, failures, flags, profile_c;
    uint16_t profile_min;
    bool cold, own, bound;
    uint16_t rh_pct10; /* the last valid RH read with the dryer idle; not refreshed in a cycle */
    uint32_t above_s;
};

/* Binds the module to the dryer and to the store as the page gave it. A store out of range
 * (a mode above PRINT, a limit or a maximum outside its bounds, a profile outside the dryer's
 * range) is reset to the defaults field by field. Nothing runs from a reset: no own cycle, no
 * outcome pending. */
void ace2k_dryer_auto_init(struct ace2k_dryer_auto *self, struct ace2k_dryer *dryer,
                           struct ace2k_dryer_auto_store *store);
/* The user's choice. A change of mode, or HUMIDITY again, drops a pending outcome: it counts
 * neither way. The same mode again changes nothing else: no page write, and the hold runs on.
 * The exception is HUMIDITY again while stopped or with failures counted, the user's re-enable:
 * it clears the stop and the failures. A change of mode while an own or bound cycle runs stops
 * that cycle on the next tick. Returns 0, or -ACE2K_EINVAL for a mode out of range. */
int ace2k_dryer_auto_mode(struct ace2k_dryer_auto *self, uint8_t mode);
/* The host's numbers at every connect. A maximum at or below the failures already counted stops
 * the humidity mode on the next tick, with AUTO_STOPPED. Returns 0, or -ACE2K_EINVAL (nothing
 * changes). */
int ace2k_dryer_auto_limits(struct ace2k_dryer_auto *self, uint8_t limit_pct, uint8_t max_failures);
/* The host's profile. A temperature below the running own or bound cycle's target lowers that
 * target at once (ace2k_dryer_lower_target: the end stays, no event). It never raises a cycle and
 * never touches a manual one. Returns 0, or -ACE2K_EINVAL outside the dryer's range (nothing
 * changes). */
int ace2k_dryer_auto_profile(struct ace2k_dryer_auto *self, uint8_t target_c, uint16_t minutes);
/* The bench's overrides, RAM only. 0 keeps the default; a value below
 * ACE2K_DRYER_AUTO_MIN_TIMING_MS is raised to it. The host sends them at every connect, so only a
 * change restarts what it times: a new window empties the cold-box window, a new hold restarts
 * the hold, a new link time restarts nothing. */
void ace2k_dryer_auto_timing(struct ace2k_dryer_auto *self, uint32_t hold_ms, uint32_t window_ms,
                             uint32_t link_ms);
/* The same overrides in seconds, as the host sends them. Returns 0, or -ACE2K_EINVAL when a value
 * in milliseconds would not fit 32 bits (nothing changes). */
int ace2k_dryer_auto_timing_s(struct ace2k_dryer_auto *self, uint32_t hold_s, uint32_t window_s,
                              uint32_t link_s);
/* A start from the host, as ACEK_DRY sends it. Over an own or bound cycle in STARTING or
 * HEATING it retargets that cycle, which becomes manual; otherwise it is ace2k_dryer_start().
 * Once accepted, it drops a pending outcome: that counts neither way. */
enum ace2k_dryer_refusal ace2k_dryer_auto_manual_start(struct ace2k_dryer_auto *self,
                                                       uint8_t target_c, uint16_t minutes,
                                                       uint32_t now_ms);
/* A stop from the host, as ACEK_DRY_STOP sends it. An own cycle's stop turns the humidity mode
 * off (AUTO_OFF on the next tick). A bound cycle's stop leaves the mode. */
enum ace2k_dryer_refusal ace2k_dryer_auto_manual_stop(struct ace2k_dryer_auto *self,
                                                      uint32_t now_ms);
/* A start bound to the host (the print mode). Outside the print mode it is refused BUSY, and
 * nothing starts. */
enum ace2k_dryer_refusal ace2k_dryer_auto_start_bound(struct ace2k_dryer_auto *self,
                                                      uint8_t target_c, uint16_t minutes,
                                                      uint32_t now_ms);
/* The print mode's stop, at the print's end and when its cycle is orphaned. It stops the
 * running cycle only when it is bound to the host (STARTING or HEATING), as a bound cycle's
 * manual stop does: the mode stays. Over a manual or an own cycle, or with nothing running, it
 * answers OK and does nothing. So a stop decided from an old report never ends the user's
 * cycle. */
enum ace2k_dryer_refusal ace2k_dryer_auto_stop_bound(struct ace2k_dryer_auto *self,
                                                     uint32_t now_ms);
/* Every 10 ms, right after ace2k_dryer_tick(), with the same inputs. It emits through the
 * dryer's ring. With no host, a strand entering an empty lane lowers the stored profile, as the
 * dryer's spool rule lowers a running cycle. Every other call runs from command context, inside
 * the binding's irq bracket. */
void ace2k_dryer_auto_tick(struct ace2k_dryer_auto *self, const struct ace2k_dryer_inputs *in,
                           uint32_t now_ms);
/* The state for the host. While the readings are invalid, cold and above_s read 0; the cold box
 * and the hold inside are kept. */
void ace2k_dryer_auto_status(const struct ace2k_dryer_auto *self,
                             struct ace2k_dryer_auto_status *out, uint32_t now_ms);
/* The page handshake, as the dryer's log does it: a change that must reach the page bumps the
 * generation; dirty until the page has stored that generation. */
bool ace2k_dryer_auto_dirty(const struct ace2k_dryer_auto *self);
uint32_t ace2k_dryer_auto_generation(const struct ace2k_dryer_auto *self);
void ace2k_dryer_auto_stored(struct ace2k_dryer_auto *self, uint32_t generation);

#endif

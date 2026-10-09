/* What: the dryer's vent — when both exhaust flaps open during HEATING, and the humidity guard
 * that closes them again only if the chamber reads more than 1 g/m³ drier than the room.
 * How: one struct ace2k_dryer_vent per dryer; at STARTING ace2k_dryer_vent_room_pick() chooses the
 * room's reference — the reading's absolute humidity (ace2k_dryer_vent_sample_cg()), or the one
 * kept from a cycle that ended shortly before — and ace2k_dryer_vent_begin_cg() takes it;
 * ace2k_dryer_vent_heating() at the entry into HEATING, then ace2k_dryer_vent_step() every tick of
 * HEATING.  It samples the chamber on a ACE2K_DRYER_VENT_SAMPLE_MS grid from the entry (the first
 * sample at the entry) and decides only on a sample: from the first sample
 * ACE2K_DRYER_VENT_WINDOW_MS into HEATING it opens once, when
 * the chamber is within ACE2K_DRYER_VENT_NEAR_MC of the target or rose less than
 * ACE2K_DRYER_VENT_RISE_MC over the window — an invalid chamber sample decides nothing and restarts
 * the window, so "stopped rising" is judged over valid samples only; once open the guard compares
 * the chamber's absolute humidity with the room's.  Absolute humidity in 0.01 g/m³ (cg) from a
 * table of saturation vapour density and the RH.  No floats.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_DRYER_VENT_H
#define ACE2K_DRYER_VENT_H
#include <stdbool.h>
#include <stdint.h>

#include "core/util.h"

/* Shared by every dryer file (dryer_internal.h reaches them through dryer.h): millidegrees per
 * degree, and 100.0 % RH in 0.1 % — the full scale of the AHT20's reading (docs/hardware.md); a
 * reading above it is no reading (the binding's validity bound), and the vent clamps to it. */
#define ACE2K_DRYER_MC_PER_C      1000
#define ACE2K_DRYER_RH_FULL_PCT10 1000U

#define ACE2K_DRYER_VENT_SAMPLE_MS 30000U
#define ACE2K_DRYER_VENT_WINDOW_MS 300000U /* 5 min */
#define ACE2K_DRYER_VENT_SLOTS     ((ACE2K_DRYER_VENT_WINDOW_MS / ACE2K_DRYER_VENT_SAMPLE_MS) + 1U)
#define ACE2K_DRYER_VENT_NEAR_MC   3000 /* initial value; re-tune on the bench */
#define ACE2K_DRYER_VENT_RISE_MC   500  /* initial value; re-tune on the bench */
/* The guard: the condition held this long; it closes the flaps only when the chamber reads more
 * than CLOSE_MARGIN below the room's reference and reopens them at HYST above it; at most one move
 * per ACE2K_DRYER_VENT_GUARD_MIN_MS. */
#define ACE2K_DRYER_VENT_GUARD_HOLD_MS 60000U
#define ACE2K_DRYER_VENT_GUARD_HYST_CG 200U /* 2 g/m³ */
/* 1 g/m³: initial value (user decision 2026-10-01, bench: a 0.5 g/m³ dip closed the flaps);
 * re-tune on the bench */
#define ACE2K_DRYER_VENT_GUARD_CLOSE_MARGIN_CG 100U
#define ACE2K_DRYER_VENT_GUARD_MIN_MS          600000U
#define ACE2K_DRYER_VENT_AH_UNKNOWN            UINT32_MAX
/* The room's reference kept across a quick restart (a chamber still dry from the cycle that just
 * ended reads too low): reused when the last cycle ended less than ROOM_REUSE_MS ago and the
 * reference was taken less than ROOM_MAX_AGE_MS ago.  Initial values (user decision 2026-10-01);
 * re-tune on the bench. */
#define ACE2K_DRYER_VENT_ROOM_REUSE_MS   14400000U /* 4 h */
#define ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS 86400000U /* 24 h */

enum ace2k_dryer_vent_action {
    ACE2K_DRYER_VENT_HOLD = 0,
    ACE2K_DRYER_VENT_OPEN = 1,
    ACE2K_DRYER_VENT_CLOSE = 2,
};

struct ace2k_dryer_vent_sample {
    int32_t chamber_mc;
    uint16_t rh_pct10; /* 0.1 % */
    bool chamber_valid;
    bool rh_valid;
};

struct ace2k_dryer_vent {
    int32_t hist_mc[ACE2K_DRYER_VENT_SLOTS]; /* the chamber, one per sample, a ring */
    uint8_t hist_n, hist_next;
    uint32_t since_ms;       /* the entry into HEATING */
    uint32_t sample_next_ms; /* the next sample */
    uint32_t room_cg;        /* the reference, ACE2K_DRYER_VENT_AH_UNKNOWN when unread */
    bool vented;             /* opened once this cycle */
    bool open;               /* the flaps are (to be) open */
    bool cond;               /* the guard's condition held since cond_since_ms */
    uint32_t cond_since_ms;
    bool guard_moved;
    uint32_t guard_last_ms;
};

/* The room's reference the dryer keeps between cycles, in RAM only (a boot starts with none). */
struct ace2k_dryer_vent_room {
    bool kept;         /* a reference taken from a STARTING reading */
    uint32_t cg;       /* its absolute humidity */
    uint32_t taken_ms; /* when it was taken */
    bool ended;        /* a cycle ended since the boot */
    uint32_t ended_ms; /* when the last one ended */
};

/* Absolute humidity, 0.01 g/m³, of a reading: the chamber clamped to 0–89.999 °C, the RH to
 * ACE2K_DRYER_RH_FULL_PCT10. */
uint32_t ace2k_dryer_vent_ah_cg(int32_t chamber_mc, uint16_t rh_pct10);
/* A reading's absolute humidity, ACE2K_DRYER_VENT_AH_UNKNOWN when it is invalid. */
uint32_t ace2k_dryer_vent_sample_cg(const struct ace2k_dryer_vent_sample *s);
/* STARTING with the reference ace2k_dryer_vent_room_pick() chose; a new cycle. */
void ace2k_dryer_vent_begin_cg(struct ace2k_dryer_vent *v, uint32_t room_cg, uint32_t now_ms);
/* The reference for a start: the kept one when a cycle ended less than
 * ACE2K_DRYER_VENT_ROOM_REUSE_MS ago and it was taken less than ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS
 * ago; otherwise fresh_cg, kept and stamped now (an unknown one is not kept). */
uint32_t ace2k_dryer_vent_room_pick(struct ace2k_dryer_vent_room *room, uint32_t fresh_cg,
                                    uint32_t now_ms);
/* A cycle ended (its cool-down done, or its fault cleared). */
void ace2k_dryer_vent_room_ended(struct ace2k_dryer_vent_room *room, uint32_t now_ms);
/* Every tick: a reference ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS old is dropped, and an end
 * ACE2K_DRYER_VENT_ROOM_REUSE_MS old forgotten — long before the ms clock wraps (49.7 days), so an
 * old stamp never aliases back into either window. */
void ace2k_dryer_vent_room_expire(struct ace2k_dryer_vent_room *room, uint32_t now_ms);
/* The entry into HEATING: the sample grid starts here. */
void ace2k_dryer_vent_heating(struct ace2k_dryer_vent *v, uint32_t now_ms);
/* Every tick of HEATING; target_mc is the cycle's current target (after a lowering, the lowered
 * one).  OPEN or CLOSE: pulse both flaps that way; HOLD: nothing. */
enum ace2k_dryer_vent_action ace2k_dryer_vent_step(struct ace2k_dryer_vent *v, int32_t target_mc,
                                                   const struct ace2k_dryer_vent_sample *s,
                                                   uint32_t now_ms);

#endif

/* What: the dryer's vent. It decides when both exhaust flaps open during HEATING. It also
 * holds the humidity guard, which closes them again only if the chamber reads more than 1 g/m³
 * drier than the room.
 * How: there is one struct ace2k_dryer_vent per dryer.
 * - At STARTING, ace2k_dryer_vent_room_pick() chooses the room's reference. It is the
 *   reading's absolute humidity (ace2k_dryer_vent_sample_cg()), or the reference kept from a
 *   cycle that ended shortly before. ace2k_dryer_vent_begin_cg() takes it.
 * - At the entry into HEATING, call ace2k_dryer_vent_heating(). Then call
 *   ace2k_dryer_vent_step() every tick of HEATING.
 * The vent samples the chamber every ACE2K_DRYER_VENT_SAMPLE_MS from the entry. The first sample
 * is at the entry. It decides only on a sample. From the first sample at least
 * ACE2K_DRYER_VENT_WINDOW_MS into HEATING, it opens the flaps once. It opens them when the chamber
 * is within ACE2K_DRYER_VENT_NEAR_MC of the target, or rose less than ACE2K_DRYER_VENT_RISE_MC over
 * the window. An invalid chamber sample decides nothing and restarts the window. So "stopped
 * rising" is judged over valid samples only. Once the flaps are open, the guard compares the
 * chamber's absolute humidity with the room's. Absolute humidity is in 0.01 g/m³ (cg).
 * It comes from a table of saturation vapour density and the RH. No floats.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_DRYER_VENT_H
#define ACE2K_DRYER_VENT_H
#include <stdbool.h>
#include <stdint.h>

#include "core/util.h"

/* Shared by every dryer file. dryer_internal.h reaches them through dryer.h. The first is
 * millidegrees per degree. The second is 100.0 % RH in 0.1 %, the full scale of the AHT20 reading
 * (docs/hardware.md). The binding treats a reading above it as no reading, and the vent clamps to
 * it. */
#define ACE2K_DRYER_MC_PER_C      1000
#define ACE2K_DRYER_RH_FULL_PCT10 1000U

#define ACE2K_DRYER_VENT_SAMPLE_MS 30000U
#define ACE2K_DRYER_VENT_WINDOW_MS 300000U /* 5 min */
#define ACE2K_DRYER_VENT_SLOTS     ((ACE2K_DRYER_VENT_WINDOW_MS / ACE2K_DRYER_VENT_SAMPLE_MS) + 1U)
#define ACE2K_DRYER_VENT_NEAR_MC   3000 /* initial value; re-tune on the bench */
#define ACE2K_DRYER_VENT_RISE_MC   500  /* initial value; re-tune on the bench */
/* The guard. Its condition must hold this long before it acts. It closes the flaps only when the
 * chamber reads more than CLOSE_MARGIN below the room's reference. It reopens them at HYST
 * above the reference. It moves at most once per ACE2K_DRYER_VENT_GUARD_MIN_MS. */
#define ACE2K_DRYER_VENT_GUARD_HOLD_MS 60000U
#define ACE2K_DRYER_VENT_GUARD_HYST_CG 200U /* 2 g/m³ */
/* 1 g/m³, an initial value. With no margin, a dip of only 0.5 g/m³ closed the flaps. Re-tune on
 * the bench. */
#define ACE2K_DRYER_VENT_GUARD_CLOSE_MARGIN_CG 100U
#define ACE2K_DRYER_VENT_GUARD_MIN_MS          600000U
#define ACE2K_DRYER_VENT_AH_UNKNOWN            UINT32_MAX
/* The room's reference is kept across a quick restart. The reason: a chamber still dry from
 * the cycle that just ended reads too low. The kept reference is reused when the last cycle ended
 * less than ROOM_REUSE_MS ago, and the reference was taken less than ROOM_MAX_AGE_MS ago. Initial
 * values; re-tune on the bench. */
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
    int32_t hist_mc[ACE2K_DRYER_VENT_SLOTS]; /* the chamber, one value per sample, in a ring */
    uint8_t hist_n, hist_next;
    uint32_t since_ms;       /* the entry into HEATING */
    uint32_t sample_next_ms; /* the next sample */
    uint32_t room_cg;        /* the reference, ACE2K_DRYER_VENT_AH_UNKNOWN when unread */
    bool vented;             /* opened once this cycle */
    bool open;               /* the flaps are open, or about to open */
    bool cond;               /* the guard's condition held since cond_since_ms */
    uint32_t cond_since_ms;
    bool guard_moved;
    uint32_t guard_last_ms;
};

/* The room's reference that the dryer keeps between cycles. It lives in RAM only, so a boot
 * starts with none. */
struct ace2k_dryer_vent_room {
    bool kept;         /* a reference was taken from a STARTING reading */
    uint32_t cg;       /* its absolute humidity */
    uint32_t taken_ms; /* when it was taken */
    bool ended;        /* a cycle ended since the boot */
    uint32_t ended_ms; /* when the last one ended */
};

/* Returns the absolute humidity of a reading, in 0.01 g/m³. The chamber is clamped to
 * 0–89.999 °C, and the RH to ACE2K_DRYER_RH_FULL_PCT10. */
uint32_t ace2k_dryer_vent_ah_cg(int32_t chamber_mc, uint16_t rh_pct10);
/* Returns the absolute humidity of a reading, or ACE2K_DRYER_VENT_AH_UNKNOWN when it is invalid. */
uint32_t ace2k_dryer_vent_sample_cg(const struct ace2k_dryer_vent_sample *s);
/* Starts a new cycle at STARTING, with the reference that ace2k_dryer_vent_room_pick() chose. */
void ace2k_dryer_vent_begin_cg(struct ace2k_dryer_vent *v, uint32_t room_cg, uint32_t now_ms);
/* Returns the reference for a start. It is the kept one when a cycle ended less than
 * ACE2K_DRYER_VENT_ROOM_REUSE_MS ago and the kept one was taken less than
 * ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS ago. Otherwise it is fresh_cg, which is kept and stamped now.
 * An unknown fresh_cg is not kept. */
uint32_t ace2k_dryer_vent_room_pick(struct ace2k_dryer_vent_room *room, uint32_t fresh_cg,
                                    uint32_t now_ms);
/* Records that a cycle ended: its cool-down is done, or its fault was cleared. */
void ace2k_dryer_vent_room_ended(struct ace2k_dryer_vent_room *room, uint32_t now_ms);
/* Runs every tick. A reference ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS old is dropped. An end
 * ACE2K_DRYER_VENT_ROOM_REUSE_MS old is forgotten. Both happen long before the ms clock wraps
 * (49.7 days), so an old stamp never aliases back into either window. */
void ace2k_dryer_vent_room_expire(struct ace2k_dryer_vent_room *room, uint32_t now_ms);
/* Call at the entry into HEATING. The sampling starts here. */
void ace2k_dryer_vent_heating(struct ace2k_dryer_vent *v, uint32_t now_ms);
/* Call every tick of HEATING. target_mc is the cycle's current target; after a lowering, it
 * is the lowered one. On OPEN or CLOSE, pulse both flaps that way. On HOLD, do nothing. */
enum ace2k_dryer_vent_action ace2k_dryer_vent_step(struct ace2k_dryer_vent *v, int32_t target_mc,
                                                   const struct ace2k_dryer_vent_sample *s,
                                                   uint32_t now_ms);

#endif

/* The vent and the humidity guard (dryer_vent.h).  The table: saturation vapour density over
 * water, ρ = 216.7 · e / (273.15 + T) g/m³ with e = 6.112 · exp(17.67 T / (T + 243.5)) hPa — the
 * Magnus formula — in 0.01 g/m³, rounded, 0–90 °C. */
#include "dryer/dryer_vent.h"

#define VENT_TABLE_MAX_C 90

static const uint16_t ace2k_vent_sat_cg[VENT_TABLE_MAX_C + 1] = {
    485,   519,   556,   595,   636,   679,   726,   775,   826,   881,   939,   1000,  1065,
    1133,  1205,  1282,  1362,  1446,  1535,  1629,  1727,  1831,  1940,  2055,  2176,  2302,
    2435,  2574,  2721,  2874,  3035,  3203,  3380,  3564,  3758,  3960,  4172,  4393,  4624,
    4865,  5117,  5380,  5655,  5942,  6241,  6552,  6877,  7215,  7568,  7935,  8317,  8714,
    9128,  9558,  10005, 10470, 10953, 11454, 11975, 12516, 13077, 13659, 14263, 14889, 15538,
    16211, 16909, 17631, 18379, 19153, 19955, 20784, 21643, 22530, 23448, 24397, 25378, 26391,
    27438, 28520, 29636, 30789, 31979, 33206, 34473, 35780, 37127, 38516, 39948, 41423, 42943,
};

uint32_t ace2k_dryer_vent_ah_cg(int32_t chamber_mc, uint16_t rh_pct10)
{
    int32_t mc = ace2k_clamp_i32(chamber_mc, 0, (VENT_TABLE_MAX_C * ACE2K_DRYER_MC_PER_C) - 1);
    uint32_t i = (uint32_t)mc / (uint32_t)ACE2K_DRYER_MC_PER_C;
    uint32_t frac = (uint32_t)mc % (uint32_t)ACE2K_DRYER_MC_PER_C;
    uint32_t lo = ace2k_vent_sat_cg[i];
    uint32_t sat =
        lo + ((((uint32_t)ace2k_vent_sat_cg[i + 1U] - lo) * frac) / (uint32_t)ACE2K_DRYER_MC_PER_C);
    uint32_t rh = rh_pct10;
    if (rh > ACE2K_DRYER_RH_FULL_PCT10) {
        rh = ACE2K_DRYER_RH_FULL_PCT10; /* no air holds more than saturation */
    }
    return (sat * rh) / ACE2K_DRYER_RH_FULL_PCT10;
}

uint32_t ace2k_dryer_vent_sample_cg(const struct ace2k_dryer_vent_sample *s)
{
    if (!s->chamber_valid || !s->rh_valid) {
        return ACE2K_DRYER_VENT_AH_UNKNOWN;
    }
    return ace2k_dryer_vent_ah_cg(s->chamber_mc, s->rh_pct10);
}

void ace2k_dryer_vent_begin_cg(struct ace2k_dryer_vent *v, uint32_t room_cg, uint32_t now_ms)
{
    *v = (struct ace2k_dryer_vent){ 0 };
    v->room_cg = room_cg;
    v->since_ms = now_ms;
    v->sample_next_ms = now_ms;
}

static bool room_reusable(const struct ace2k_dryer_vent_room *room, uint32_t now_ms)
{
    if (!room->kept || !room->ended) {
        return false;
    }
    if (ace2k_time_since(now_ms, room->ended_ms) >= ACE2K_DRYER_VENT_ROOM_REUSE_MS) {
        return false;
    }
    return ace2k_time_since(now_ms, room->taken_ms) < ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS;
}

uint32_t ace2k_dryer_vent_room_pick(struct ace2k_dryer_vent_room *room, uint32_t fresh_cg,
                                    uint32_t now_ms)
{
    if (room_reusable(room, now_ms)) {
        return room->cg;
    }
    room->kept = false;
    if (fresh_cg != ACE2K_DRYER_VENT_AH_UNKNOWN) {
        room->kept = true;
        room->cg = fresh_cg;
        room->taken_ms = now_ms;
    }
    return fresh_cg;
}

void ace2k_dryer_vent_room_expire(struct ace2k_dryer_vent_room *room, uint32_t now_ms)
{
    if (room->kept &&
        ace2k_time_since(now_ms, room->taken_ms) >= ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS) {
        room->kept = false;
    }
    if (room->ended && ace2k_time_since(now_ms, room->ended_ms) >= ACE2K_DRYER_VENT_ROOM_REUSE_MS) {
        room->ended = false;
    }
}

void ace2k_dryer_vent_room_ended(struct ace2k_dryer_vent_room *room, uint32_t now_ms)
{
    room->ended = true;
    room->ended_ms = now_ms;
}

void ace2k_dryer_vent_heating(struct ace2k_dryer_vent *v, uint32_t now_ms)
{
    v->since_ms = now_ms;
    v->sample_next_ms = now_ms;
    v->hist_n = 0U;
    v->hist_next = 0U;
}

static void push(struct ace2k_dryer_vent *v, int32_t chamber_mc)
{
    v->hist_mc[v->hist_next] = chamber_mc;
    v->hist_next = (uint8_t)((v->hist_next + 1U) % ACE2K_DRYER_VENT_SLOTS);
    if (v->hist_n < ACE2K_DRYER_VENT_SLOTS) {
        v->hist_n++;
    }
}

static int32_t newest(const struct ace2k_dryer_vent *v)
{
    return v->hist_mc[(v->hist_next + ACE2K_DRYER_VENT_SLOTS - 1U) % ACE2K_DRYER_VENT_SLOTS];
}

/* The ring is full: the oldest slot is the one written next. */
static int32_t oldest(const struct ace2k_dryer_vent *v)
{
    return v->hist_mc[v->hist_next];
}

static bool open_due(const struct ace2k_dryer_vent *v, int32_t target_mc, uint32_t now_ms)
{
    if (v->hist_n == 0U || ace2k_time_since(now_ms, v->since_ms) < ACE2K_DRYER_VENT_WINDOW_MS) {
        return false;
    }
    if (newest(v) >= target_mc - ACE2K_DRYER_VENT_NEAR_MC) {
        return true;
    }
    if (v->hist_n < ACE2K_DRYER_VENT_SLOTS) {
        return false;
    }
    return newest(v) - oldest(v) < ACE2K_DRYER_VENT_RISE_MC;
}

/* The guard wants the flaps the other way: open and drier than the room by more than the close
 * margin (never, with a room below the margin), or closed and humid again by the hysteresis. */
static bool guard_wants(const struct ace2k_dryer_vent *v, uint32_t ah)
{
    if (v->open) {
        if (v->room_cg <= ACE2K_DRYER_VENT_GUARD_CLOSE_MARGIN_CG) {
            return false;
        }
        return ah < v->room_cg - ACE2K_DRYER_VENT_GUARD_CLOSE_MARGIN_CG;
    }
    return ah >= v->room_cg + ACE2K_DRYER_VENT_GUARD_HYST_CG;
}

static enum ace2k_dryer_vent_action guard(struct ace2k_dryer_vent *v,
                                          const struct ace2k_dryer_vent_sample *s, uint32_t now_ms)
{
    uint32_t ah = ace2k_dryer_vent_sample_cg(s);
    if (v->room_cg == ACE2K_DRYER_VENT_AH_UNKNOWN || ah == ACE2K_DRYER_VENT_AH_UNKNOWN) {
        /* frozen where it is, and the hold restarts: the condition must hold
         * ACE2K_DRYER_VENT_GUARD_HOLD_MS of valid readings, an invalid stretch counts for none */
        v->cond = false;
        return ACE2K_DRYER_VENT_HOLD;
    }
    if (!guard_wants(v, ah)) {
        v->cond = false;
        return ACE2K_DRYER_VENT_HOLD;
    }
    if (!v->cond) {
        v->cond = true;
        v->cond_since_ms = now_ms;
        return ACE2K_DRYER_VENT_HOLD;
    }
    if (ace2k_time_since(now_ms, v->cond_since_ms) < ACE2K_DRYER_VENT_GUARD_HOLD_MS) {
        return ACE2K_DRYER_VENT_HOLD;
    }
    if (v->guard_moved &&
        ace2k_time_since(now_ms, v->guard_last_ms) < ACE2K_DRYER_VENT_GUARD_MIN_MS) {
        return ACE2K_DRYER_VENT_HOLD;
    }
    v->cond = false;
    v->guard_moved = true;
    v->guard_last_ms = now_ms;
    if (v->open) {
        v->open = false;
        return ACE2K_DRYER_VENT_CLOSE;
    }
    v->open = true;
    return ACE2K_DRYER_VENT_OPEN;
}

enum ace2k_dryer_vent_action ace2k_dryer_vent_step(struct ace2k_dryer_vent *v, int32_t target_mc,
                                                   const struct ace2k_dryer_vent_sample *s,
                                                   uint32_t now_ms)
{
    if (!ace2k_time_after(now_ms, v->sample_next_ms)) {
        return ACE2K_DRYER_VENT_HOLD;
    }
    v->sample_next_ms += ACE2K_DRYER_VENT_SAMPLE_MS;
    if (v->vented) {
        return guard(v, s, now_ms);
    }
    if (!s->chamber_valid) {
        /* no decision on no reading, and the window restarts: "stopped rising" is judged over
         * ACE2K_DRYER_VENT_SLOTS valid samples on the grid, never across a gap */
        v->hist_n = 0U;
        v->hist_next = 0U;
        return ACE2K_DRYER_VENT_HOLD;
    }
    push(v, s->chamber_mc);
    if (!open_due(v, target_mc, now_ms)) {
        return ACE2K_DRYER_VENT_HOLD;
    }
    v->vented = true;
    v->open = true;
    return ACE2K_DRYER_VENT_OPEN;
}

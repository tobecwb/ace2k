/* The scheduler of the tie-break (rfid/watch_tiebreak.h).  It only chooses: the read with motion
 * that it asks for runs in the feed, and the watch attributes what follows the lane. */
#include "rfid/watch_tiebreak.h"
#include "rfid/watch_internal.h"

void ace2k_rfid_tiebreak_note(struct ace2k_rfid_tiebreak *tb, const struct ace2k_rfid_watch *w,
                              const struct ace2k_rfid_watch_inputs *in, uint32_t now_ms)
{
    uint8_t in_use = 0;
    (void)ace2k_rfid_watch_in_use(w, &in_use); /* 0 when not valid: every lane counts */
    /* Frozen sensors show no edge.  The edge is taken once they are known again. */
    uint8_t rise = 0;
    if (!in->insert_unknown) {
        rise = (uint8_t)(in->insert & (uint8_t)~tb->insert_last);
        tb->insert_last = in->insert;
    }
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        uint8_t reader = ace2k_rfid_reader_of(lane);
        if (ace2k_rfid_watch_bit(rise, lane)) {
            uint8_t pair = (uint8_t)(3U << ace2k_rfid_watch_first_lane(reader));
            tb->tried = (uint8_t)(tb->tried & (uint8_t)~pair);
            tb->t_still_ms[reader] = now_ms;
        }
        if (ace2k_rfid_watch_bit(in->moving, lane) && !ace2k_rfid_watch_bit(in_use, lane)) {
            tb->t_still_ms[reader] = now_ms;
        }
    }
}

/* The reader can take a tie-break: alive, no session or read on command on it, and its lanes not
 * in use still long enough. */
static bool reader_ready(const struct ace2k_rfid_tiebreak *tb, const struct ace2k_rfid_watch *w,
                         uint8_t reader, uint32_t now_ms)
{
    const struct ace2k_rfid_watch_reader *rd = &w->r[reader];
    uint8_t first = ace2k_rfid_watch_first_lane(reader);
    if (rd->dead || rd->session_open || w->l[first].probe || w->l[first + 1U].probe) {
        return false;
    }
    return ace2k_time_since(now_ms, tb->t_still_ms[reader]) >= ACE2K_RFID_TIEBREAK_QUIET_MS;
}

static bool lane_ready(const struct ace2k_rfid_tiebreak *tb, const struct ace2k_rfid_watch *w,
                       uint8_t lane, uint8_t idle_mask, uint8_t in_use)
{
    const struct ace2k_rfid_watch_lane *l = &w->l[lane];
    if (l->state != ACE2K_RFID_PENDING || l->by_hand || !ace2k_rfid_watch_bit(w->in.insert, lane)) {
        return false;
    }
    if (ace2k_rfid_watch_bit(in_use, lane) || ace2k_rfid_watch_bit(tb->tried, lane) ||
        !ace2k_rfid_watch_bit(idle_mask, lane)) {
        return false;
    }
    uint8_t n = ace2k_rfid_neighbour(lane);
    return (bool)(ace2k_rfid_watch_bit(idle_mask, n) || ace2k_rfid_watch_bit(in_use, n));
}

int8_t ace2k_rfid_tiebreak_pick(const struct ace2k_rfid_tiebreak *tb,
                                const struct ace2k_rfid_watch *w, uint8_t idle_mask,
                                uint32_t now_ms)
{
    uint8_t in_use = 0;
    if (!w->in.link_ok || w->in.insert_unknown || !ace2k_rfid_watch_in_use(w, &in_use)) {
        return -1; /* no move without the host's word */
    }
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (lane_ready(tb, w, lane, idle_mask, in_use) &&
            reader_ready(tb, w, ace2k_rfid_reader_of(lane), now_ms)) {
            return (int8_t)lane;
        }
    }
    return -1;
}

void ace2k_rfid_tiebreak_tried(struct ace2k_rfid_tiebreak *tb, uint8_t lane)
{
    if (lane < ACE2K_LANE_COUNT) {
        tb->tried = (uint8_t)(tb->tried | (1U << lane));
    }
}

void ace2k_rfid_tiebreak_restart(struct ace2k_rfid_tiebreak *tb, uint8_t lane)
{
    if (lane < ACE2K_LANE_COUNT) {
        tb->tried = (uint8_t)(tb->tried & (uint8_t)~(1U << lane));
    }
}

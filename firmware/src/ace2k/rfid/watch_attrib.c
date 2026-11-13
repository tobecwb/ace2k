/* The watch's attribution.  It decides which tag of the inventory of a reader belongs to which of
 * its two lanes, and answers a probe.  It was split from watch.c, and the seam is
 * watch_internal.h. */
#include "rfid/watch_internal.h"

bool ace2k_rfid_watch_in_list(const struct ace2k_iso14443a_tag *list, uint8_t n,
                              const struct ace2k_iso14443a_tag *t)
{
    for (uint8_t i = 0; i < n; i++) {
        if (ace2k_iso14443a_same_uid(&list[i], t)) {
            return true;
        }
    }
    return false;
}

/* The neighbour's UID (protocol.md, "Watching and attribution"): the UID attributed to the
 * neighbour in its insertion is never the UID of this lane, read, being read, or kept from a
 * session that ended, even when it comes back while only this lane moves.  A UID that the
 * neighbour only waits for, to open a session, is left out here on purpose: a UID the neighbour
 * took is already tracked.  note_seen tests it too, for rule 9. */
static bool owned_by_neighbour(const struct ace2k_rfid_watch *self, uint8_t lane,
                               const struct ace2k_iso14443a_tag *t)
{
    return ace2k_rfid_watch_holds(&self->l[ace2k_rfid_neighbour(lane)], t);
}

/* The record of a UID that left the field while the lane moved outside a load, or -1. */
static int gone_index(const struct ace2k_rfid_watch_reader *rd, uint8_t lane,
                      const struct ace2k_iso14443a_tag *t)
{
    for (uint8_t i = 0; i < rd->gone_count; i++) {
        if (rd->gone_lane[i] == lane && ace2k_iso14443a_same_uid(&rd->gone[i], t)) {
            return (int)i;
        }
    }
    return -1;
}

/* Passive watching: the UID left the field while the lane moved with its neighbour still, the
 * neighbour stayed still since, and the lane travelled cfg.passive_min_um to cfg.passive_max_um
 * before its return.  That is one full turn of the lane's spool.  A tag at the edge of the field
 * flickers back far sooner, and a return after more is not one turn. */
static bool returned_after_a_turn(const struct ace2k_rfid_watch *self, uint8_t lane,
                                  const struct ace2k_iso14443a_tag *t)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    int i = gone_index(rd, lane, t);
    if (i < 0) {
        return false;
    }
    return ace2k_rfid_watch_apart(self->in.travel_um[lane], rd->gone_at_um[i],
                                  self->cfg.passive_min_um, self->cfg.passive_max_um);
}

enum ace2k_rfid_claim {
    ACE2K_RFID_CLAIM_NONE,
    ACE2K_RFID_CLAIM_TAKE, /* the UID is the lane's */
    /* the UID is the lane's only once it follows the lane's reversals */
    ACE2K_RFID_CLAIM_CANDIDATE,
};

/* The attribution rules (protocol.md, "Watching and attribution") for a tag that the inventory
 * saw.  A tag is the lane's only when it follows the lane's motion.  A tag that only
 * enters the field proves nothing about the bay it came from: a hand hanging a spool on the
 * neighbour's bay brings one in too.
 * - The lane's own known UID is the lane's.
 * - With the neighbour read, a UID that enters while the lane moves is the lane's.  The
 *   neighbour's spool holds the neighbour's tag, so a new one can only be the lane's.  So it is
 *   with the neighbour set by hand and still: its strand never left, and its tag at rest does not
 *   enter.  A neighbour set by hand that moves (a print) may turn its tag in, so then the rules
 *   below apply.
 * - In the feed's load, a UID that enters while the lane moves is a candidate.  The feed reverses
 *   the lane, and the UID is the lane's once it follows (watch_confirm.c).  A UID that leaves
 *   while the lane moves is a candidate too (ace2k_rfid_watch_inventory_done).
 * - Outside a load, a UID is the lane's when it comes back after a full turn of the lane's spool,
 *   with the neighbour still from the leave to the return.  A leave while the neighbour moves is
 *   not recorded, and the neighbour moving drops the lane's records (ace2k_rfid_watch_drop_gone,
 *   from watch.c), so a record that is there was made and kept with the neighbour still. */
static enum ace2k_rfid_claim claims(const struct ace2k_rfid_watch *self, uint8_t lane,
                                    const struct ace2k_iso14443a_tag *t, bool entered)
{
    const struct ace2k_rfid_watch_inputs *in = &self->in;
    uint8_t n = ace2k_rfid_neighbour(lane);
    if (ace2k_rfid_watch_holds(&self->l[lane], t)) {
        return ACE2K_RFID_CLAIM_TAKE;
    }
    if (!entered || !ace2k_rfid_watch_bit(in->moving, lane)) {
        return ACE2K_RFID_CLAIM_NONE;
    }
    if (self->l[n].state == ACE2K_RFID_READ) {
        return ACE2K_RFID_CLAIM_TAKE;
    }
    if (self->l[n].by_hand && !ace2k_rfid_watch_bit(in->moving, n)) {
        /* The neighbour set by hand and still: its tag, at rest, does not enter. */
        return ACE2K_RFID_CLAIM_TAKE;
    }
    if (ace2k_rfid_watch_bit(in->loading, lane)) {
        return ACE2K_RFID_CLAIM_CANDIDATE;
    }
    bool n_still = (bool)!ace2k_rfid_watch_bit(in->moving, n);
    return n_still && returned_after_a_turn(self, lane, t) ? ACE2K_RFID_CLAIM_TAKE
                                                           : ACE2K_RFID_CLAIM_NONE;
}

void ace2k_rfid_watch_take(struct ace2k_rfid_watch *self, uint8_t lane,
                           const struct ace2k_iso14443a_tag *t, uint32_t now_ms)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (rd->session_open) {
        self->l[lane].want = true;
        self->l[lane].wanted = *t;
        return;
    }
    ace2k_rfid_watch_open_session(self, lane, t, now_ms);
}

static void attribute_lane(struct ace2k_rfid_watch *self, uint8_t lane,
                           const struct ace2k_iso14443a_out *o,
                           const bool entered[ACE2K_ISO_INVENTORY_MAX], uint32_t now_ms)
{
    for (uint8_t i = 0; i < o->tag_count; i++) {
        const struct ace2k_iso14443a_tag *t = &o->tags[i];
        if (owned_by_neighbour(self, lane, t)) {
            continue;
        }
        enum ace2k_rfid_claim c = claims(self, lane, t, entered[i]);
        if (c == ACE2K_RFID_CLAIM_TAKE) {
            ace2k_rfid_watch_take(self, lane, t, now_ms);
            return;
        }
        if (c == ACE2K_RFID_CLAIM_CANDIDATE) {
            (void)ace2k_rfid_watch_confirm_offer(self, lane, t);
        }
    }
}

/* In the feed's load, with the neighbour not read, a UID that left the field while the lane moved
 * is a candidate too: the lane's spool may have turned its tag away.  The UIDs that left are
 * offered in turn until one is accepted. */
static void offer_left(struct ace2k_rfid_watch *self, uint8_t lane,
                       const struct ace2k_iso14443a_tag *left, uint8_t left_count)
{
    const struct ace2k_rfid_watch_inputs *in = &self->in;
    if (!ace2k_rfid_watch_bit(in->loading, lane) || !ace2k_rfid_watch_bit(in->moving, lane) ||
        self->l[ace2k_rfid_neighbour(lane)].state == ACE2K_RFID_READ) {
        return;
    }
    for (uint8_t i = 0; i < left_count; i++) {
        if (!owned_by_neighbour(self, lane, &left[i]) &&
            !ace2k_rfid_watch_holds(&self->l[lane], &left[i]) &&
            ace2k_rfid_watch_confirm_offer(self, lane, &left[i])) {
            return;
        }
    }
}

/* A read on command.  It attributes no new tag: a tag goes to a lane only when that lane's motion
 * explains it, and a lane that has not moved stays pending.  It is decided at the
 * first inventory that completes the baseline, or at the next one after it, so one inventory that
 * missed a tag cannot answer alone.  The lane's own UID, attributed earlier by its motion, reopens
 * its session as an inventory would.  Any other UID that is not the neighbour's,
 * in this inventory or an earlier one of the baseline, answers ambiguous.  With none, the lane's
 * state is answered unchanged. */
static void probe_lane(struct ace2k_rfid_watch *self, uint8_t lane,
                       const struct ace2k_iso14443a_out *o, bool decide, uint32_t now_ms)
{
    struct ace2k_rfid_watch_lane *l = &self->l[lane];
    for (uint8_t i = 0; i < o->tag_count; i++) {
        const struct ace2k_iso14443a_tag *t = &o->tags[i];
        if (ace2k_rfid_watch_holds(l, t)) {
            if (decide && ace2k_rfid_watch_eligible(self, lane)) {
                l->probe = false;
                ace2k_rfid_watch_take(self, lane, t, now_ms);
                return;
            }
        } else if (!owned_by_neighbour(self, lane, t)) {
            l->probe_other = true;
        }
    }
    if (decide) {
        l->probe = false;
        ace2k_rfid_watch_push_state(self, lane,
                                    l->probe_other ? (uint8_t)ACE2K_RFID_KIND_AMBIGUOUS : l->state);
    }
}

/* Rule 9: a search that saw a UID it could not attribute does not end no_tag.  A UID the
 * neighbour holds or waits for is never this lane's, so it does not count.  It runs
 * after both lanes' attribution, so a UID the neighbour took in this inventory does not count. */
static void note_seen(struct ace2k_rfid_watch *self, uint8_t lane,
                      const struct ace2k_iso14443a_out *o)
{
    struct ace2k_rfid_watch_lane *l = &self->l[lane];
    const struct ace2k_rfid_watch_lane *n = &self->l[ace2k_rfid_neighbour(lane)];
    if (l->state != ACE2K_RFID_SEARCHING) {
        return;
    }
    for (uint8_t i = 0; i < o->tag_count; i++) {
        const struct ace2k_iso14443a_tag *t = &o->tags[i];
        bool wanted = (bool)(n->want && ace2k_iso14443a_same_uid(&n->wanted, t));
        if (!owned_by_neighbour(self, lane, t) && !wanted) {
            l->seen = true;
            return;
        }
    }
}

static void drop_gone_at(struct ace2k_rfid_watch_reader *rd, uint8_t i)
{
    rd->gone_count--;
    for (uint8_t k = i; k < rd->gone_count; k++) {
        rd->gone[k] = rd->gone[k + 1U];
        rd->gone_at_um[k] = rd->gone_at_um[k + 1U];
        rd->gone_lane[k] = rd->gone_lane[k + 1U];
    }
}

void ace2k_rfid_watch_drop_gone(struct ace2k_rfid_watch_reader *rd, uint8_t lane)
{
    for (uint8_t i = rd->gone_count; i-- > 0;) {
        if (rd->gone_lane[i] == lane) {
            drop_gone_at(rd, i);
        }
    }
}

/* A UID that comes back is measured once: its records go, and its next leave makes new ones.  A
 * return that is not taken (the neighbour moving) uses its record up too, so the lane needs
 * another full turn. */
static void drop_gone_uid(struct ace2k_rfid_watch_reader *rd, const struct ace2k_iso14443a_tag *t)
{
    for (uint8_t i = rd->gone_count; i-- > 0;) {
        if (ace2k_iso14443a_same_uid(&rd->gone[i], t)) {
            drop_gone_at(rd, i);
        }
    }
}

/* The record that a full table gives up for a new one.  A record of a UID that is in view again
 * waits for no return, so it goes first.  Otherwise the oldest goes.  t is the UID that leaves:
 * it is still tracked here, so its own records do not count as in view. */
static uint8_t gone_victim(const struct ace2k_rfid_watch_reader *rd,
                           const struct ace2k_iso14443a_tag *t)
{
    for (uint8_t i = 0; i < rd->gone_count; i++) {
        if (!ace2k_iso14443a_same_uid(&rd->gone[i], t) &&
            ace2k_rfid_watch_in_list(rd->tracked, rd->tracked_count, &rd->gone[i])) {
            return i;
        }
    }
    return 0;
}

/* A UID left the field.  For each lane of the reader that moves outside a load, with its
 * neighbour still, it is recorded with that lane's travel.  A newer leave replaces the older one.
 * With the table full, a record of a UID back in view goes first, then the oldest. */
static void note_gone(struct ace2k_rfid_watch *self, uint8_t reader,
                      const struct ace2k_iso14443a_tag *t)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    uint8_t first = ace2k_rfid_watch_first_lane(reader);
    for (uint8_t lane = first; lane < first + ACE2K_RFID_LANES_PER_READER; lane++) {
        if (!ace2k_rfid_watch_bit(self->in.moving, lane) ||
            ace2k_rfid_watch_bit(self->in.loading, lane) ||
            ace2k_rfid_watch_bit(self->in.moving, ace2k_rfid_neighbour(lane))) {
            continue;
        }
        int i = gone_index(rd, lane, t);
        if (i >= 0) {
            drop_gone_at(rd, (uint8_t)i);
        } else if (rd->gone_count == ACE2K_RFID_TRACKED_MAX) {
            drop_gone_at(rd, gone_victim(rd, t));
        }
        rd->gone[rd->gone_count] = *t;
        rd->gone_at_um[rd->gone_count] = self->in.travel_um[lane];
        rd->gone_lane[rd->gone_count] = lane;
        rd->gone_count++;
    }
}

static void untrack(struct ace2k_rfid_watch *self, uint8_t reader, uint8_t j)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    note_gone(self, reader, &rd->tracked[j]);
    rd->tracked_count--;
    rd->tracked[j] = rd->tracked[rd->tracked_count];
    rd->absent[j] = rd->absent[rd->tracked_count];
}

/* Every tracked UID is in view or inside its debounce window, so none is dropped for a new one.
 * With the table full, the new UID is not tracked: it does not enter in this inventory. */
static bool track(struct ace2k_rfid_watch_reader *rd, const struct ace2k_iso14443a_tag *t)
{
    if (rd->tracked_count == ACE2K_RFID_TRACKED_MAX) {
        return false;
    }
    rd->tracked[rd->tracked_count] = *t;
    rd->absent[rd->tracked_count] = 0;
    rd->tracked_count++;
    return true;
}

/* Brings the tracked UIDs up to this inventory.  A UID not tracked enters.  One that was seen is
 * tracked until it has been away ACE2K_RFID_DEBOUNCE_POLLS inventories in a row: then it left, and
 * re-enters only after that.  It returns the number of UIDs that left, listed in left.  The caller
 * takes nothing as entered, nor as left, in the first ACE2K_RFID_BASELINE_POLLS inventories: the
 * baseline. */
static uint8_t update_field(struct ace2k_rfid_watch *self, uint8_t reader,
                            const struct ace2k_iso14443a_out *o,
                            bool entered[ACE2K_ISO_INVENTORY_MAX],
                            struct ace2k_iso14443a_tag left[ACE2K_RFID_TRACKED_MAX])
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    uint8_t left_count = 0;
    for (uint8_t j = rd->tracked_count; j-- > 0;) {
        if (ace2k_rfid_watch_in_list(o->tags, o->tag_count, &rd->tracked[j])) {
            rd->absent[j] = 0;
        } else if (++rd->absent[j] >= ACE2K_RFID_DEBOUNCE_POLLS) {
            left[left_count++] = rd->tracked[j];
            untrack(self, reader, j);
        }
    }
    for (uint8_t i = 0; i < o->tag_count; i++) {
        entered[i] = false;
        if (!ace2k_rfid_watch_in_list(rd->tracked, rd->tracked_count, &o->tags[i])) {
            entered[i] = track(rd, &o->tags[i]);
        }
    }
    return left_count;
}

void ace2k_rfid_watch_inventory_done(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(self->iso, reader);
    /* Every finished inventory counts toward the baseline, so failed ones cannot stretch it. */
    bool in_baseline = rd->baseline_polls < ACE2K_RFID_BASELINE_POLLS;
    if (in_baseline) {
        rd->baseline_polls++;
    }
    if (o->result != ACE2K_ISO_OK) {
        return; /* a partial inventory changes nothing else */
    }
    bool entered[ACE2K_ISO_INVENTORY_MAX];
    struct ace2k_iso14443a_tag left[ACE2K_RFID_TRACKED_MAX];
    uint8_t left_count = update_field(self, reader, o, entered, left);
    if (in_baseline) {
        for (uint8_t i = 0; i < o->tag_count; i++) {
            entered[i] = false; /* the tags that were there at rest: none entered */
        }
        left_count = 0;
    }
    ace2k_rfid_watch_confirm_inventory(self, reader, now_ms);
    bool decide = rd->baseline_polls >= ACE2K_RFID_BASELINE_POLLS;
    uint8_t first = ace2k_rfid_watch_first_lane(reader);
    for (uint8_t lane = first; lane < first + ACE2K_RFID_LANES_PER_READER; lane++) {
        if (self->l[lane].probe) {
            probe_lane(self, lane, o, decide, now_ms);
        } else if (ace2k_rfid_watch_eligible(self, lane)) {
            attribute_lane(self, lane, o, entered, now_ms);
            offer_left(self, lane, left, left_count);
        }
    }
    for (uint8_t lane = first; lane < first + ACE2K_RFID_LANES_PER_READER; lane++) {
        if (ace2k_rfid_watch_eligible(self, lane)) {
            note_seen(self, lane, o);
        }
    }
    for (uint8_t i = 0; i < o->tag_count; i++) {
        if (entered[i]) {
            drop_gone_uid(rd, &o->tags[i]);
        }
    }
}

enum ace2k_rfid_read_refusal ace2k_rfid_watch_probe(struct ace2k_rfid_watch *self, uint8_t lane)
{
    if (lane < ACE2K_LANE_COUNT && self->in.insert_unknown) {
        return ACE2K_RFID_READ_BUSY; /* the insert sensors are unknown: it cannot be judged now */
    }
    if (lane >= ACE2K_LANE_COUNT || !ace2k_rfid_watch_bit(self->in.insert, lane)) {
        return ACE2K_RFID_READ_NO_FILAMENT;
    }
    if (!self->in.link_ok) {
        return ACE2K_RFID_READ_NO_LINK;
    }
    if (self->l[lane].state == ACE2K_RFID_READING || self->l[lane].probe) {
        return ACE2K_RFID_READ_BUSY;
    }
    struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (rd->dead) {
        ace2k_rfid_watch_push_state(self, lane,
                                    self->l[lane].state); /* accepted, answered at once */
        return ACE2K_RFID_READ_OK;
    }
    self->l[lane].probe = true;
    self->l[lane].probe_other = false;
    self->l[lane].t_probe_ms = self->now_ms;
    rd->suppressed = false;
    return ACE2K_RFID_READ_OK;
}

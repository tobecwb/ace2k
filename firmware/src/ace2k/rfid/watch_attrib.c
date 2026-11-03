/* The watch's attribution.  It decides which tag of the inventory of a reader belongs to which of
 * its two lanes, and answers a probe.  It was split from watch.c, and the seam is
 * watch_internal.h. */
#include "rfid/watch_internal.h"

static bool in_list(const struct ace2k_iso14443a_tag *list, uint8_t n,
                    const struct ace2k_iso14443a_tag *t)
{
    for (uint8_t i = 0; i < n; i++) {
        if (ace2k_iso14443a_same_uid(&list[i], t)) {
            return true;
        }
    }
    return false;
}

/* Attribution rule 1 (protocol.md, "Watching and attribution").  The UID attributed to the
 * neighbour in its insertion is never the UID of this lane.  This holds when that UID is read,
 * being read, or kept from a session that ended.  It holds even for a UID that enters while only
 * this lane moves.  Up to v0.12.0, this applied only to a tag read or being read, and the order of
 * the inventory decided the rest.  A read or reading lane always has its tag.  A UID that the
 * neighbour only waits for, to open a session, is left out here on purpose: attribution never
 * meets the difference, because a UID the neighbour took is already tracked.  note_seen tests it
 * too, for rule 9. */
static bool owned_by_neighbour(const struct ace2k_rfid_watch *self, uint8_t lane,
                               const struct ace2k_iso14443a_tag *t)
{
    const struct ace2k_rfid_watch_lane *n = &self->l[ace2k_rfid_neighbour(lane)];
    return (bool)(n->has_tag && ace2k_iso14443a_same_uid(&n->tag, t));
}

static bool own_uid(const struct ace2k_rfid_watch_lane *l, const struct ace2k_iso14443a_tag *t)
{
    return (bool)(l->has_tag && ace2k_iso14443a_same_uid(&l->tag, t));
}

/* Attribution rules 2, 4 and 5 (protocol.md, "Watching and attribution") for a tag that the
 * inventory saw: the lane's own known UID, then a UID that enters while the lane moves.  A tag is
 * the lane's only when its motion explains it.  Rule 3, removed in v0.13.0, gave a lane any tag
 * while its neighbour had no filament.  A spool can sit on that bay with no filament in. */
static bool claims(const struct ace2k_rfid_watch *self, uint8_t lane,
                   const struct ace2k_iso14443a_tag *t, bool entered)
{
    const struct ace2k_rfid_watch_inputs *in = &self->in;
    uint8_t n = ace2k_rfid_neighbour(lane);
    if (own_uid(&self->l[lane], t)) {
        return true;
    }
    if (!entered || !ace2k_rfid_watch_bit(in->moving, lane)) {
        return false;
    }
    if (!ace2k_rfid_watch_bit(in->moving, n) || self->l[n].state == ACE2K_RFID_READ) {
        return true;
    }
    return false;
}

static void take(struct ace2k_rfid_watch *self, uint8_t lane, const struct ace2k_iso14443a_tag *t,
                 uint32_t now_ms)
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
        if (!owned_by_neighbour(self, lane, t) && claims(self, lane, t, entered[i])) {
            take(self, lane, t, now_ms);
            return;
        }
    }
}

/* A read on command.  It attributes no new tag: a tag goes to a lane only when that lane's motion
 * explains it, and a lane that has not moved stays pending (since v0.13.0).  It is decided at the
 * first inventory that completes the baseline, or at the next one after it, so one inventory that
 * missed a tag cannot answer alone.  The lane's own UID, attributed earlier by its motion, reopens
 * its session as an inventory would (rule 2).  Any other UID that is not the neighbour's (rule 1),
 * in this inventory or an earlier one of the baseline, answers ambiguous.  With none, the lane's
 * state is answered unchanged. */
static void probe_lane(struct ace2k_rfid_watch *self, uint8_t lane,
                       const struct ace2k_iso14443a_out *o, bool decide, uint32_t now_ms)
{
    struct ace2k_rfid_watch_lane *l = &self->l[lane];
    for (uint8_t i = 0; i < o->tag_count; i++) {
        const struct ace2k_iso14443a_tag *t = &o->tags[i];
        if (own_uid(l, t)) {
            if (decide && ace2k_rfid_watch_eligible(self, lane)) {
                l->probe = false;
                take(self, lane, t, now_ms);
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
 * neighbour holds or waits for is never this lane's (rule 1), so it does not count.  It runs
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

static void untrack(struct ace2k_rfid_watch_reader *rd, uint8_t j)
{
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
 * tracked until it has been away ACE2K_RFID_DEBOUNCE_POLLS inventories in a row, and re-enters only
 * after that.  The caller takes nothing as entered in the first ACE2K_RFID_BASELINE_POLLS
 * inventories: the baseline. */
static void update_field(struct ace2k_rfid_watch_reader *rd, const struct ace2k_iso14443a_out *o,
                         bool entered[ACE2K_ISO_INVENTORY_MAX])
{
    for (uint8_t j = rd->tracked_count; j-- > 0;) {
        if (in_list(o->tags, o->tag_count, &rd->tracked[j])) {
            rd->absent[j] = 0;
        } else if (++rd->absent[j] >= ACE2K_RFID_DEBOUNCE_POLLS) {
            untrack(rd, j);
        }
    }
    for (uint8_t i = 0; i < o->tag_count; i++) {
        entered[i] = false;
        if (!in_list(rd->tracked, rd->tracked_count, &o->tags[i])) {
            entered[i] = track(rd, &o->tags[i]);
        }
    }
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
    update_field(rd, o, entered);
    if (in_baseline) {
        for (uint8_t i = 0; i < o->tag_count; i++) {
            entered[i] = false; /* the tags that were there at rest: none entered */
        }
    }
    bool decide = rd->baseline_polls >= ACE2K_RFID_BASELINE_POLLS;
    uint8_t first = ace2k_rfid_watch_first_lane(reader);
    for (uint8_t lane = first; lane < first + ACE2K_RFID_LANES_PER_READER; lane++) {
        if (self->l[lane].probe) {
            probe_lane(self, lane, o, decide, now_ms);
        } else if (ace2k_rfid_watch_eligible(self, lane)) {
            attribute_lane(self, lane, o, entered, now_ms);
        }
    }
    for (uint8_t lane = first; lane < first + ACE2K_RFID_LANES_PER_READER; lane++) {
        if (ace2k_rfid_watch_eligible(self, lane)) {
            note_seen(self, lane, o);
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

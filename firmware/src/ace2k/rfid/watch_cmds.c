// Binding of the watch: the readers' transceive over the reader
// instance, the iso14443a engine over the transceive, the watch over both.  The 10 ms tick only
// wakes the task and raises the report's due flag; the task runs one watch step — every SPI
// transfer of the readers happens there — publishes the feed's bits and sends what is
// due: the state report first, then the events, oldest first, each tried and held for the next
// tick when its frame does not fit (tx.h).  A malformed command (a lane or an argument out of
// range) is a host bug and takes Klipper's way out; a stale session is ignored.
#include "rfid/watch_cmds.h"
#include "rfid/iso14443a.h"
#include "rfid/reader_cmds.h"
#include "rfid/transceive.h"
#include "rfid/watch.h"
#include "lane/lane_cmds.h"     // ace2k_lane_binding: which lanes move
#include "lane/sensors_cmds.h"  // ace2k_sensors_binding: the insert sensors
#include "ace2k_board/serial.h" // ace2k_link_ok
#include "ace2k_board/tick.h"
#include "core/report.h"
#include "core/tx.h"    // ACE2K_SENDF
#include "autoconf.h"   // CONFIG_ACE2K_FEED
#include "board/irq.h"  // irq_save, irq_restore
#include "board/misc.h" // timer_from_us
#include "command.h"    // DECL_COMMAND, sendf, shutdown
#include "sched.h"      // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake
#if CONFIG_ACE2K_FEED
#include "feed/feed_cmds.h" // the loading and exhausted bits, the read with motion, the search's reach
#endif

// The phase (report.h): sensors 0, env 30, lane 50, rfid 60, mains 70, feed 80.
#define ACE2K_RFID_REPORT_PHASE_MS 60U
#define ACE2K_RFID_FIELD_BIT_A     0x01U
#define ACE2K_RFID_FIELD_BIT_B     0x02U
#define ACE2K_RFID_FAULT_BIT_A     0x04U
#define ACE2K_RFID_FAULT_BIT_B     0x08U
#define ACE2K_RFID_DEAD_BIT_A      0x10U
#define ACE2K_RFID_DEAD_BIT_B      0x20U
#define ACE2K_RFID_KEY_BYTES       6U
// The feed's bits, one word: a lane's bit in each nibble — the session's hold, the tag read, the
// tag lost.
#define ACE2K_RFID_FEED_HOLD_SHIFT 0U
#define ACE2K_RFID_FEED_READ_SHIFT 4U
#define ACE2K_RFID_FEED_LOST_SHIFT 8U
#define ACE2K_RFID_FEED_LANES_MASK 0x0FU

static struct ace2k_rfid_transceive ace2k_rfid_tx;
static struct ace2k_iso14443a ace2k_rfid_iso;
static struct ace2k_rfid_watch ace2k_rfid_watch_instance;
static struct task_wake ace2k_rfid_wake;
static struct ace2k_report ace2k_rfid_report;
static volatile bool ace2k_rfid_report_due;
// Written whole by the task, read once by the feed's tick: one aligned 16-bit store and one load,
// so the tick never sees a hold of this run with a read of the last (a torn triple).
static volatile uint16_t ace2k_rfid_feed_bits;
static bool ace2k_rfid_ready;

// The iso14443a engine's ops: the transceive of the reader instance.
static int iso_start(void *ctx, uint8_t reader, uint8_t cmd, const uint8_t *tx, uint8_t len,
                     uint8_t last_bits)
{
    return ace2k_rfid_transceive_start(ctx, reader, cmd, tx, len, last_bits);
}

static enum ace2k_rfid_poll iso_poll(void *ctx, uint8_t reader, struct ace2k_rfid_rx *rx)
{
    return ace2k_rfid_transceive_poll(ctx, reader, rx);
}

static void iso_crypto_off(void *ctx, uint8_t reader)
{
    ace2k_rfid_transceive_crypto_off(ctx, reader);
}

static const struct ace2k_iso14443a_ops ace2k_rfid_iso_ops = { .start = iso_start,
                                                               .poll = iso_poll,
                                                               .crypto_off = iso_crypto_off };

// The watch's reader control: the same transceive.
static int w_configure(void *ctx, uint8_t reader)
{
    return ace2k_rfid_transceive_configure(ctx, reader);
}

// The ops signature takes a plain void *ctx, shared with the callbacks that write through it.
// cppcheck-suppress constParameterCallback
static bool w_needs_configure(void *ctx, uint8_t reader)
{
    return ace2k_rfid_transceive_needs_configure(ctx, reader);
}

static void w_field_set(void *ctx, uint8_t reader, bool on)
{
    ace2k_rfid_transceive_field_set(ctx, reader, on);
}

static bool w_field_driven(void *ctx, uint8_t reader)
{
    return ace2k_rfid_transceive_field_driven(ctx, reader);
}

static const struct ace2k_rfid_watch_ops ace2k_rfid_watch_ops_board = {
    .configure = w_configure,
    .needs_configure = w_needs_configure,
    .field_set = w_field_set,
    .field_driven = w_field_driven,
};

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    if (ace2k_report_due(&ace2k_rfid_report, now_ms)) {
        ace2k_rfid_report_due = true;
    }
    sched_wake_task(&ace2k_rfid_wake); // one watch step per tick
}

// The feed's side of the inputs: which lanes are in a load, and whose last search ran its length.
// Without the feed, nothing loads.
static void feed_inputs(struct ace2k_rfid_watch_inputs *in)
{
#if CONFIG_ACE2K_FEED
    ace2k_feed_binding_rfid_bits(&in->loading, &in->exhausted);
#else
    in->loading = 0;
    in->exhausted = 0;
#endif
}

static void gather(struct ace2k_rfid_watch_inputs *in)
{
    const struct ace2k_sensors *s = ace2k_sensors_binding();
    const struct ace2k_lane *lane = ace2k_lane_binding();
    *in = (struct ace2k_rfid_watch_inputs){ .link_ok = ace2k_link_ok() };
    irqstatus_t flag = irq_save(); // the tick writes the lanes' moves and the debounced states
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (s->lane[i].insert) {
            in->insert |= (uint8_t)(1U << i);
        }
        if (ace2k_lane_is_moving(lane, i)) {
            in->moving |= (uint8_t)(1U << i);
        }
    }
    feed_inputs(in);
    irq_restore(flag);
}

static void publish(void)
{
    const struct ace2k_rfid_watch *w = &ace2k_rfid_watch_instance;
    uint32_t v = ((uint32_t)ace2k_rfid_watch_hold_bits(w) & ACE2K_RFID_FEED_LANES_MASK)
                 << ACE2K_RFID_FEED_HOLD_SHIFT;
    v |= ((uint32_t)ace2k_rfid_watch_read_bits(w) & ACE2K_RFID_FEED_LANES_MASK)
         << ACE2K_RFID_FEED_READ_SHIFT;
    v |= ((uint32_t)ace2k_rfid_watch_lost_bits(w) & ACE2K_RFID_FEED_LANES_MASK)
         << ACE2K_RFID_FEED_LOST_SHIFT;
    ace2k_rfid_feed_bits = (uint16_t)v;
}

static uint8_t field_mask(void)
{
    const struct ace2k_rfid_watch *w = &ace2k_rfid_watch_instance;
    uint32_t m = 0;
    m |= ace2k_rfid_watch_field(w, ACE2K_RFID_READER_A) ? ACE2K_RFID_FIELD_BIT_A : 0U;
    m |= ace2k_rfid_watch_field(w, ACE2K_RFID_READER_B) ? ACE2K_RFID_FIELD_BIT_B : 0U;
    m |= ace2k_rfid_watch_fault(w, ACE2K_RFID_READER_A) ? ACE2K_RFID_FAULT_BIT_A : 0U;
    m |= ace2k_rfid_watch_fault(w, ACE2K_RFID_READER_B) ? ACE2K_RFID_FAULT_BIT_B : 0U;
    m |= ace2k_rfid_watch_dead(w, ACE2K_RFID_READER_A) ? ACE2K_RFID_DEAD_BIT_A : 0U;
    m |= ace2k_rfid_watch_dead(w, ACE2K_RFID_READER_B) ? ACE2K_RFID_DEAD_BIT_B : 0U;
    return (uint8_t)m;
}

static bool report(void)
{
    uint8_t state[ACE2K_LANE_COUNT];
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        state[i] = ace2k_rfid_watch_state(&ace2k_rfid_watch_instance, i);
    }
    return ACE2K_SENDF("ace2k_rfid_state state=%*s field=%c", (uint8_t)sizeof state, state,
                       field_mask());
}

static bool send_event(const struct ace2k_rfid_event *e)
{
    switch (e->type) {
    case ACE2K_RFID_EV_TAG:
        return ACE2K_SENDF("ace2k_rfid_tag lane=%c session=%c uid=%*s atqa=%hu sak=%c", e->lane,
                           e->session, e->tag.uid_len, e->tag.uid, e->tag.atqa, e->tag.sak);
    case ACE2K_RFID_EV_DATA:
        return ACE2K_SENDF("ace2k_rfid_data lane=%c session=%c block=%c status=%c data=%*s",
                           e->lane, e->session, e->block, e->kind,
                           (uint8_t)(e->kind == ACE2K_RFID_DATA_OK ? ACE2K_ISO_BLOCK_BYTES : 0U),
                           e->data);
    case ACE2K_RFID_EV_STATE:
        return ACE2K_SENDF("ace2k_rfid_event lane=%c kind=%c session=%c", e->lane, e->kind,
                           e->session);
    default:
        return true; // no other event type exists
    }
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.
void ace2k_rfid_watch_task(void)
{
    if (!sched_check_wake(&ace2k_rfid_wake)) {
        return;
    }
    struct ace2k_rfid_watch_inputs in;
    gather(&in);
    ace2k_rfid_watch_step(&ace2k_rfid_watch_instance, ace2k_tick_now_ms(), &in);
    publish();
    if (ace2k_rfid_report_due) {
        (void)ace2k_report_try(&ace2k_rfid_report_due, report);
    }
    struct ace2k_rfid_event e;
    while (ace2k_rfid_watch_peek_event(&ace2k_rfid_watch_instance, &e)) {
        if (!send_event(&e)) {
            break; // the ring's tail waits for the next tick, the task's waker
        }
        ace2k_rfid_watch_drop_event(&ace2k_rfid_watch_instance);
    }
}
DECL_TASK(ace2k_rfid_watch_task);

static void ensure_ready(void)
{
    if (ace2k_rfid_ready) {
        return;
    }
    ace2k_rfid_transceive_init(&ace2k_rfid_tx, ace2k_rfid_reader_binding_mut());
    ace2k_iso14443a_init(&ace2k_rfid_iso, &ace2k_rfid_iso_ops, &ace2k_rfid_tx);
    ace2k_rfid_watch_init(&ace2k_rfid_watch_instance, &ace2k_rfid_iso, &ace2k_rfid_watch_ops_board,
                          &ace2k_rfid_tx);
#if CONFIG_ACE2K_FEED
    ace2k_feed_binding_search_set(ACE2K_FEED_SEARCH_DEFAULT_UM); // rfid_search_mm overrides
#endif
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_rfid_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_rfid_watch_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_rfid_watch_binding_init);

void ace2k_rfid_binding_feed_bits(uint8_t *hold, uint8_t *read, uint8_t *lost)
{
    uint32_t v = ace2k_rfid_feed_bits; // the one load
    *hold = (uint8_t)((v >> ACE2K_RFID_FEED_HOLD_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    *read = (uint8_t)((v >> ACE2K_RFID_FEED_READ_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    *lost = (uint8_t)((v >> ACE2K_RFID_FEED_LOST_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
}

bool ace2k_rfid_binding_reader_busy(uint8_t reader)
{
    return ace2k_rfid_ready && ace2k_rfid_watch_reader_busy(&ace2k_rfid_watch_instance, reader);
}

bool ace2k_rfid_binding_field_fault(uint8_t reader)
{
    if (!ace2k_rfid_ready) {
        return false;
    }
    const struct ace2k_rfid_watch *w = &ace2k_rfid_watch_instance;
    return ace2k_rfid_watch_fault(w, reader) || ace2k_rfid_watch_dead(w, reader);
}

void ace2k_rfid_binding_clear_dead(void)
{
    if (!ace2k_rfid_ready) {
        return;
    }
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        ace2k_rfid_watch_clear_dead(&ace2k_rfid_watch_instance, r);
    }
}

// A read with motion: the feed's search, forward up to its reach and back; without the feed,
// unsupported.  Busy while a session is open on the lane or a read on command is pending on it —
// checked before the feed's search start, so a refusal changes nothing: a search begun under an
// open session would never show the lane searching, and its outcome would be that session's.
static uint8_t read_with_motion(uint8_t lane)
{
#if CONFIG_ACE2K_FEED
    if (ace2k_rfid_watch_lane_busy(&ace2k_rfid_watch_instance, lane)) {
        return ACE2K_RFID_READ_BUSY;
    }
    switch (ace2k_feed_binding_search_start(lane)) {
    case ACE2K_FEED_ACCEPTED:
        // MOVE=1 is an explicit re-read: a read or no_tag lane forgets its tag (forget acts on
        // those two states only), or its search would run its whole reach past a tag the watch
        // no longer looks for; only after the feed accepted, so a refusal changes nothing.  The
        // feed's bits are re-published now, so its next tick does not see the stale read bit.
        ace2k_rfid_watch_forget(&ace2k_rfid_watch_instance, lane);
        publish();
        return ACE2K_RFID_READ_OK;
    case ACE2K_FEED_REFUSED_NO_FILAMENT:
        return ACE2K_RFID_READ_NO_FILAMENT;
    case ACE2K_FEED_REFUSED_NO_LINK:
        return ACE2K_RFID_READ_NO_LINK;
    case ACE2K_FEED_REFUSED_OTHER_LANE:
        return ACE2K_RFID_READ_OTHER_LANE_MOVING;
    case ACE2K_FEED_REFUSED_BOUNDS:
        return ACE2K_RFID_READ_UNSUPPORTED;
    default:
        return ACE2K_RFID_READ_BUSY; // busy, in error
    }
#else
    (void)lane;
    return ACE2K_RFID_READ_UNSUPPORTED;
#endif
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_read(uint32_t *args)
{
    ensure_ready();
    if (args[0] >= ACE2K_LANE_COUNT) {
        shutdown("ace2k_rfid_read: lane out of range");
    }
    if (args[1] > 1U) {
        shutdown("ace2k_rfid_read: move out of range");
    }
    uint8_t lane = (uint8_t)args[0];
    uint8_t reason = args[1] ? read_with_motion(lane)
                             : (uint8_t)ace2k_rfid_watch_probe(&ace2k_rfid_watch_instance, lane);
    sendf("ace2k_rfid_read_response lane=%c accepted=%c reason=%c", lane,
          reason == ACE2K_RFID_READ_OK ? 1 : 0, reason);
}
DECL_COMMAND(ace2k_rfid_cmd_read, "ace2k_rfid_read lane=%c move=%c");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_step(uint32_t *args)
{
    ensure_ready();
    uint8_t op = (uint8_t)args[2];
    uint8_t key_len = (uint8_t)args[5];
    const uint8_t *key = command_decode_ptr(args[6]);
    if (args[2] > ACE2K_RFID_OP_MIFARE_B) { // first: an unknown op is not a short MIFARE key
        shutdown("ace2k_rfid_step: op out of range");
    }
    if (op != ACE2K_RFID_OP_NTAG && key_len != ACE2K_RFID_KEY_BYTES) {
        shutdown("ace2k_rfid_step: a MIFARE step needs a six-byte key");
    }
    int rc = ace2k_rfid_watch_step_request(&ace2k_rfid_watch_instance, (uint8_t)args[0],
                                           (uint8_t)args[1], op, (uint8_t)args[3], (uint8_t)args[4],
                                           key, ace2k_tick_now_ms());
    if (rc == -ACE2K_EINVAL) {
        shutdown("ace2k_rfid_step: lane, op, sector, page or blocks out of bounds");
    }
    // -ACE2K_EREFUSED: no such session open, or a step already runs — a stale host, ignored
}
DECL_COMMAND(ace2k_rfid_cmd_step,
             "ace2k_rfid_step lane=%c session=%c op=%c arg=%c blocks=%c key=%*s");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_done(uint32_t *args)
{
    ensure_ready();
    if (args[0] >= ACE2K_LANE_COUNT || args[2] > 1U) {
        shutdown("ace2k_rfid_done: lane or result out of range");
    }
    (void)ace2k_rfid_watch_done(&ace2k_rfid_watch_instance, (uint8_t)args[0], (uint8_t)args[1],
                                args[2] != 0);
}
DECL_COMMAND(ace2k_rfid_cmd_done, "ace2k_rfid_done lane=%c session=%c result=%c");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_forget(uint32_t *args)
{
    ensure_ready();
    if (args[0] >= ACE2K_LANE_COUNT) {
        shutdown("ace2k_rfid_forget: lane out of range");
    }
    ace2k_rfid_watch_forget(&ace2k_rfid_watch_instance, (uint8_t)args[0]);
}
DECL_COMMAND(ace2k_rfid_cmd_forget, "ace2k_rfid_forget lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_lane_query(uint32_t *args)
{
    ensure_ready();
    if (args[0] >= ACE2K_LANE_COUNT) {
        shutdown("ace2k_rfid_lane_query: lane out of range");
    }
    uint8_t lane = (uint8_t)args[0];
    const struct ace2k_iso14443a_tag *t = ace2k_rfid_watch_tag(&ace2k_rfid_watch_instance, lane);
    uint8_t state = ace2k_rfid_watch_state(&ace2k_rfid_watch_instance, lane);
    // the UID of a lane that has one attributed: reading or read; none otherwise
    bool has = state == ACE2K_RFID_READING || state == ACE2K_RFID_READ;
    sendf("ace2k_rfid_lane_state lane=%c state=%c uid=%*s", lane, state,
          (uint8_t)(has ? t->uid_len : 0U), t->uid);
}
DECL_COMMAND(ace2k_rfid_cmd_lane_query, "ace2k_rfid_lane_query lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_query(uint32_t *args)
{
    ensure_ready();
    uint32_t rest_ms = args[0] / timer_from_us(1000U);
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_rfid_report, rest_ms, ace2k_tick_now_ms(), ACE2K_RFID_REPORT_PHASE_MS);
    ace2k_rfid_report_due = false;
    irq_restore(flag);
}
DECL_COMMAND(ace2k_rfid_cmd_query, "ace2k_rfid_query rest_ticks=%u");

#if CONFIG_ACE2K_FEED
DECL_CONSTANT("ACE2K_FEED_SEARCH_MAX_UM", ACE2K_FEED_SEARCH_MAX_UM);
// The host's default for rfid_search_mm when printer.cfg does not set it (the
// firmware's own): ace2k_feed.py's ACE_LOAD budget and ace2k_rfid.py's MOVE=1 wait read it.
DECL_CONSTANT("ACE2K_FEED_SEARCH_DEFAULT_UM", ACE2K_FEED_SEARCH_DEFAULT_UM);

// The search's reach from printer.cfg (rfid_search_mm), a config command: a change restarts
// the MCU, which starts from the default again.  Zero or past the ceiling is a host bug (rule 3).
// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_search_set(uint32_t *args)
{
    ensure_ready();
    if (args[0] == 0 || args[0] > ACE2K_FEED_SEARCH_MAX_UM) {
        shutdown("ace2k_rfid_search_set: search_um out of bounds");
    }
    ace2k_feed_binding_search_set(args[0]);
}
DECL_COMMAND(ace2k_rfid_cmd_search_set, "ace2k_rfid_search_set search_um=%u");
#endif

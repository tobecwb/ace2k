// Binding of the watch.  The transceive of the readers runs over the reader instance, the iso14443a
// engine runs over the transceive, and the watch runs over both.  The 10 ms tick only wakes the
// task and raises the report's due flag.  The task runs one watch step, and every SPI transfer of
// the readers happens there.  Then the task publishes the bits for the feed and sends what is due.
// The state report goes first, then the mask of the lanes set by hand when it is due, then the
// events, oldest first, then the traces of the confirmations, only once no event waits.  Each one
// is tried, and held for the next tick when its frame does not fit (tx.h).  A malformed command,
// with a lane or an argument out of range, is a bug in the host and takes the Klipper way out.  A
// stale session is ignored.
#include "rfid/watch_cmds.h"
#include "rfid/iso14443a.h" // ACE2K_ISO_KEY_BYTES
#include "rfid/reader_cmds.h"
#include "rfid/transceive.h"
#include "rfid/watch.h"
#include "rfid/watch_tiebreak.h"
#include "lane/lane_cmds.h"     // ace2k_lane_binding: which lanes move
#include "lane/sensors_cmds.h"  // ace2k_sensors_binding: the insert sensors
#include "ace2k_board/args.h"   // ACE2K_ARG_U8, ACE2K_ARG_U8_MAX, ACE2K_ARG_FLAG
#include "lane/lane_args.h"     // ACE2K_ARG_LANE
#include "ace2k_board/serial.h" // ace2k_link_ok
#include "ace2k_board/tick.h"
#include "core/report_cmds.h" // ace2k_report_query
#include "ace2k_board/tx.h"   // ACE2K_SENDF
#include "autoconf.h"         // CONFIG_ACE2K_FEED
#include "board/irq.h"        // irq_save, irq_restore
#include "command.h"          // DECL_COMMAND, sendf, shutdown
#include "sched.h"            // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake
#if CONFIG_ACE2K_FEED
// the loading and exhausted bits, the read with motion, the search's reach
#include "feed/feed_cmds.h"
#endif

// The phase (report.h): sensors 0, env 30, lane 50, rfid 60, mains 70, feed 80.
#define ACE2K_RFID_REPORT_PHASE_MS 60U
#define ACE2K_RFID_FIELD_BIT_A     0x01U
#define ACE2K_RFID_FIELD_BIT_B     0x02U
#define ACE2K_RFID_FAULT_BIT_A     0x04U
#define ACE2K_RFID_FAULT_BIT_B     0x08U
#define ACE2K_RFID_DEAD_BIT_A      0x10U
#define ACE2K_RFID_DEAD_BIT_B      0x20U
// The bits for the feed, in one word.  Each nibble holds one bit per lane, for the hold of the
// session, the read tag, the lost tag, the confirmation asked and its direction.  The next one
// holds the host's mask of the lanes in use, 0 while that mask is not valid, and the bit above
// it says whether it is valid.  The top nibble holds the epoch of each lane's confirmation.
#define ACE2K_RFID_FEED_HOLD_SHIFT          0U
#define ACE2K_RFID_FEED_READ_SHIFT          4U
#define ACE2K_RFID_FEED_LOST_SHIFT          8U
#define ACE2K_RFID_FEED_CONFIRM_SHIFT       12U
#define ACE2K_RFID_FEED_CONFIRM_REV_SHIFT   16U
#define ACE2K_RFID_FEED_IN_USE_SHIFT        20U
#define ACE2K_RFID_FEED_IN_USE_VALID        0x01000000U // bit 24
#define ACE2K_RFID_FEED_CONFIRM_EPOCH_SHIFT 28U
#define ACE2K_RFID_FEED_LANES_MASK          0x0FU

static struct ace2k_rfid_transceive ace2k_rfid_tx;
static struct ace2k_iso14443a ace2k_rfid_iso;
static struct ace2k_rfid_watch ace2k_rfid_watch_instance;
#if CONFIG_ACE2K_FEED
static struct ace2k_rfid_tiebreak ace2k_rfid_tb;
#endif
static struct task_wake ace2k_rfid_wake;
static struct ace2k_report ace2k_rfid_report;
static volatile bool ace2k_rfid_report_due;
// The task writes the word whole, and the tick of the feed reads it once.  It is one aligned 32-bit
// store and one load.  So the tick never sees a hold of this run with a read of the last run, which
// would be a torn set.
static volatile uint32_t ace2k_rfid_feed_bits;
// The lanes set by hand, one bit per lane.  The word above has no nibble left, so they go in a
// byte of their own.  The task writes it whole.  It changes only on a host command or a strand
// leaving.
static volatile uint8_t ace2k_rfid_by_hand_bits;
static bool ace2k_rfid_ready;

// The ops of the iso14443a engine: the transceive of the reader instance.
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

static const struct ace2k_iso14443a_ops iso_ops = {
    .start = iso_start,
    .poll = iso_poll,
    .crypto_off = iso_crypto_off,
};

// The control of the readers by the watch: the same transceive.
static int w_configure(void *ctx, uint8_t reader)
{
    return ace2k_rfid_transceive_configure(ctx, reader);
}

// The signature of the ops takes a plain void *ctx, shared with the callbacks that write through
// it.
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

static const struct ace2k_rfid_watch_ops watch_board_ops = {
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

// The inputs that come from the feed: which lanes are in a load, which ones had their last search
// run its whole length, which ones ran a confirmation leg out, and for which confirmation (its
// epoch).  Without the feed, nothing loads.
static void feed_inputs(struct ace2k_rfid_watch_inputs *in)
{
#if CONFIG_ACE2K_FEED
    ace2k_feed_binding_rfid_bits(&in->loading, &in->exhausted, &in->confirm_out,
                                 &in->confirm_out_epoch);
#else
    in->loading = 0;
    in->exhausted = 0;
    in->confirm_out = 0;
    in->confirm_out_epoch = 0;
#endif
}

static void gather(struct ace2k_rfid_watch_inputs *in)
{
    const struct ace2k_sensors *s = ace2k_sensors_binding();
    const struct ace2k_lane *lane = ace2k_lane_binding();
    *in = (struct ace2k_rfid_watch_inputs){ .link_ok = ace2k_link_ok() };
    irqstatus_t flag = irq_save(); // the tick writes the lane moves and the debounced states
    in->insert_unknown =
        (bool)!ace2k_sensors_insert_known(s); // the scan stopped: the bits are frozen
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (s->lane[i].insert) {
            in->insert |= (uint8_t)(1U << i);
        }
        if (ace2k_lane_is_moving(lane, i)) {
            in->moving |= (uint8_t)(1U << i);
        }
        if (ace2k_lane_is_reverse(lane, i)) {
            in->reverse |= (uint8_t)(1U << i);
        }
        in->travel_um[i] = ace2k_lane_encoder_um(lane, i);
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
    v |= ((uint32_t)ace2k_rfid_watch_confirm_bits(w) & ACE2K_RFID_FEED_LANES_MASK)
         << ACE2K_RFID_FEED_CONFIRM_SHIFT;
    v |= ((uint32_t)ace2k_rfid_watch_confirm_reverse_bits(w) & ACE2K_RFID_FEED_LANES_MASK)
         << ACE2K_RFID_FEED_CONFIRM_REV_SHIFT;
    v |= ((uint32_t)ace2k_rfid_watch_confirm_epoch_bits(w) & ACE2K_RFID_FEED_LANES_MASK)
         << ACE2K_RFID_FEED_CONFIRM_EPOCH_SHIFT;
    uint8_t in_use = 0;
    if (ace2k_rfid_watch_in_use(w, &in_use)) { // the mask 0 when not valid
        v |= ACE2K_RFID_FEED_IN_USE_VALID;
    }
    v |= ((uint32_t)in_use & ACE2K_RFID_FEED_LANES_MASK) << ACE2K_RFID_FEED_IN_USE_SHIFT;
    ace2k_rfid_feed_bits = v;
    ace2k_rfid_by_hand_bits =
        (uint8_t)(ace2k_rfid_watch_by_hand_bits(w) & ACE2K_RFID_FEED_LANES_MASK);
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

static void tiebreak(const struct ace2k_rfid_watch_inputs *in, uint32_t now_ms);

// The mask of the lanes set by hand last sent to the host.  0xff, no mask, sends it again: before
// the first, and after every state report.
static uint8_t ace2k_rfid_by_hand_told = 0xFFU;

// The mask of the lanes set by hand, with every state report and on any change.  A frame that does
// not fit is tried again on the next tick.
static void tell_by_hand(void)
{
    uint8_t mask = ace2k_rfid_watch_by_hand_bits(&ace2k_rfid_watch_instance);
    if (mask != ace2k_rfid_by_hand_told && ACE2K_SENDF("ace2k_rfid_by_hand_state mask=%c", mask)) {
        ace2k_rfid_by_hand_told = mask;
    }
}

// The traces of the confirmations, for the bench, oldest first.  A frame that does not fit is
// tried again on the next tick.
static void send_traces(void)
{
    struct ace2k_rfid_confirm_trace t;
    while (ace2k_rfid_watch_peek_trace(&ace2k_rfid_watch_instance, &t)) {
        if (!ACE2K_SENDF("ace2k_rfid_confirm_trace lane=%c what=%c travel_um=%i reverse=%c lost=%c",
                         t.lane, t.what, t.travel_um, (uint8_t)t.reverse, t.lost)) {
            break;
        }
        ace2k_rfid_watch_drop_trace(&ace2k_rfid_watch_instance);
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
    uint32_t now_ms = ace2k_tick_now_ms();
    ace2k_rfid_watch_step(&ace2k_rfid_watch_instance, now_ms, &in);
    tiebreak(&in, now_ms);
    publish();
    if (ace2k_rfid_report_due && ace2k_report_try(&ace2k_rfid_report_due, report)) {
        ace2k_rfid_by_hand_told = 0xFFU;
    }
    tell_by_hand();
    struct ace2k_rfid_event e;
    while (ace2k_rfid_watch_peek_event(&ace2k_rfid_watch_instance, &e)) {
        if (!send_event(&e)) {
            return; // the tail of the ring waits for the next tick, and the traces after it
        }
        ace2k_rfid_watch_drop_event(&ace2k_rfid_watch_instance);
    }
    send_traces();
}
DECL_TASK(ace2k_rfid_watch_task);

static void ensure_ready(void)
{
    if (ace2k_rfid_ready) {
        return;
    }
    ace2k_rfid_transceive_init(&ace2k_rfid_tx, ace2k_rfid_reader_binding_mut());
    ace2k_iso14443a_init(&ace2k_rfid_iso, &iso_ops, &ace2k_rfid_tx);
    ace2k_rfid_watch_init(&ace2k_rfid_watch_instance, &ace2k_rfid_iso, &watch_board_ops,
                          &ace2k_rfid_tx);
#if CONFIG_ACE2K_FEED
    // This sets the search's default reach, and rfid_search_mm overrides it.  The default is under
    // the limit, and feed.h asserts that.  The text is the same as that of the search command, so
    // the dictionary gains no entry for a line that cannot run.
    if (ace2k_feed_binding_search_set(ACE2K_FEED_SEARCH_DEFAULT_UM) != 0) {
        shutdown("ace2k_rfid_search_set: search_um out of bounds");
    }
    // The feed runs the legs of a confirmation with the watch's own settings.
    ace2k_feed_binding_confirm_cfg_set((uint32_t)ace2k_rfid_watch_instance.cfg.leg_max_um,
                                       (uint32_t)ace2k_rfid_watch_instance.cfg.speed_um_s);
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

void ace2k_rfid_binding_feed_bits(uint8_t *hold, uint8_t *read, uint8_t *lost, uint8_t *confirm,
                                  uint8_t *confirm_rev, uint8_t *confirm_epoch)
{
    uint32_t v = ace2k_rfid_feed_bits; // the one load
    *hold = (uint8_t)((v >> ACE2K_RFID_FEED_HOLD_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    *read = (uint8_t)((v >> ACE2K_RFID_FEED_READ_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    *lost = (uint8_t)((v >> ACE2K_RFID_FEED_LOST_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    *confirm = (uint8_t)((v >> ACE2K_RFID_FEED_CONFIRM_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    *confirm_rev = (uint8_t)((v >> ACE2K_RFID_FEED_CONFIRM_REV_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    *confirm_epoch =
        (uint8_t)((v >> ACE2K_RFID_FEED_CONFIRM_EPOCH_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
}

bool ace2k_rfid_binding_in_use(uint8_t *mask)
{
    uint32_t v = ace2k_rfid_feed_bits; // the one load
    *mask = (uint8_t)((v >> ACE2K_RFID_FEED_IN_USE_SHIFT) & ACE2K_RFID_FEED_LANES_MASK);
    return (bool)((v & ACE2K_RFID_FEED_IN_USE_VALID) != 0U);
}

uint8_t ace2k_rfid_binding_by_hand(void)
{
    return ace2k_rfid_by_hand_bits; // the one load
}

bool ace2k_rfid_binding_reader_busy(uint8_t reader)
{
    return (bool)(ace2k_rfid_ready &&
                  ace2k_rfid_watch_reader_busy(&ace2k_rfid_watch_instance, reader));
}

bool ace2k_rfid_binding_field_fault(uint8_t reader)
{
    if (!ace2k_rfid_ready) {
        return false;
    }
    const struct ace2k_rfid_watch *w = &ace2k_rfid_watch_instance;
    return (bool)(ace2k_rfid_watch_fault(w, reader) || ace2k_rfid_watch_dead(w, reader));
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

#if CONFIG_ACE2K_FEED
// The answer of a feed's start, as the answer of a read.
static enum ace2k_rfid_read_refusal read_refusal(enum ace2k_feed_refusal r)
{
    switch (r) {
    case ACE2K_FEED_ACCEPTED:
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
        // busy, in error, or the insert sensors unknown (the scan stopped): not now, try later
        return ACE2K_RFID_READ_BUSY;
    }
}

// Starts the feed's search for a read with motion.  ctx points to the read's seq, its retry key.
static enum ace2k_rfid_read_refusal search_start(void *ctx, uint8_t lane)
{
    return read_refusal(ace2k_feed_binding_search_start(lane, *(const uint8_t *)ctx));
}

#endif

// The tie-break (rfid/watch_tiebreak.h), after each watch step.  A lane the feed has idle with its
// motor stopped may be picked (the pick also rules out a session or a read on command on its
// reader).  The lane is tried once the feed accepted the start: a refused start is asked again at a
// later run.  Without the feed nothing moves on its own.
static void tiebreak(const struct ace2k_rfid_watch_inputs *in, uint32_t now_ms)
{
#if CONFIG_ACE2K_FEED
    const struct ace2k_rfid_watch *w = &ace2k_rfid_watch_instance;
    ace2k_rfid_tiebreak_note(&ace2k_rfid_tb, w, in, now_ms);
    uint8_t idle = (uint8_t)(ace2k_feed_binding_idle_bits() & (uint8_t)~in->moving);
    int8_t lane = ace2k_rfid_tiebreak_pick(&ace2k_rfid_tb, w, idle, now_ms);
    if (lane >= 0 && ace2k_feed_binding_tiebreak_start((uint8_t)lane) == ACE2K_FEED_ACCEPTED) {
        ace2k_rfid_tiebreak_tried(&ace2k_rfid_tb, (uint8_t)lane);
    }
#else
    (void)in;
    (void)now_ms;
#endif
}

// A read with motion runs the feed's search (ace2k_rfid_watch_read_with_motion).  Without the feed
// it is unsupported.  When a search starts, the bits for the feed are published again at once.  So
// the feed's next tick does not see the read bit that the lane just dropped.  The same read sent
// again (its seq, while its search runs or just after) is the transport's retry: it is answered
// accepted, and nothing else happens, before the watch forgets the lane's tag.
static uint8_t read_with_motion(uint8_t lane, uint8_t seq)
{
#if CONFIG_ACE2K_FEED
    if (ace2k_feed_binding_search_retry(lane, seq)) {
        return ACE2K_RFID_READ_OK;
    }
    enum ace2k_rfid_read_refusal r =
        ace2k_rfid_watch_read_with_motion(&ace2k_rfid_watch_instance, lane, search_start, &seq);
    if (r == ACE2K_RFID_READ_OK) {
        // The host's read is its request to start the lane's tie-break over.
        ace2k_rfid_tiebreak_restart(&ace2k_rfid_tb, lane);
        publish();
    }
    return (uint8_t)r;
#else
    (void)lane;
    (void)seq;
    return ACE2K_RFID_READ_UNSUPPORTED;
#endif
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_read(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_rfid_read: lane out of range");
    bool with_motion = ACE2K_ARG_FLAG(args[1], "ace2k_rfid_read: move out of range");
    // The host's key for a read with motion, 1..255; 0 is none (never a retry).  A read without
    // motion ignores it: sent twice, it only looks at the field twice.
    uint8_t seq = ACE2K_ARG_U8(args[2], "ace2k_rfid_read: seq out of range");
    uint8_t reason = with_motion
                         ? read_with_motion(lane, seq)
                         : (uint8_t)ace2k_rfid_watch_probe(&ace2k_rfid_watch_instance, lane);
    sendf("ace2k_rfid_read_response lane=%c accepted=%c reason=%c", lane,
          reason == ACE2K_RFID_READ_OK ? 1 : 0, reason);
}
DECL_COMMAND(ace2k_rfid_cmd_read, "ace2k_rfid_read lane=%c move=%c seq=%c");

// The arguments of ace2k_rfid_step, in the order of its format.  The key (%*s) takes two arguments,
// its length and then the encoded pointer.
enum ace2k_rfid_step_arg {
    ACE2K_RFID_STEP_ARG_LANE,
    ACE2K_RFID_STEP_ARG_SESSION,
    ACE2K_RFID_STEP_ARG_OP,
    ACE2K_RFID_STEP_ARG_ARG,
    ACE2K_RFID_STEP_ARG_BLOCKS,
    ACE2K_RFID_STEP_ARG_KEY_LEN,
    ACE2K_RFID_STEP_ARG_KEY,
};

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_step(uint32_t *args)
{
    ensure_ready();
    // Each argument is taken in the order of the format.  The core checks the lane, the page or
    // sector and the blocks against their bounds, and the session against the open one.  A value
    // above 255 must not wrap into those bounds.
    uint8_t lane = ACE2K_ARG_U8(args[ACE2K_RFID_STEP_ARG_LANE],
                                "ace2k_rfid_step: lane, op, sector, page or blocks out of bounds");
    uint8_t session =
        ACE2K_ARG_U8(args[ACE2K_RFID_STEP_ARG_SESSION], "ace2k_rfid_step: session out of range");
    uint8_t op = ACE2K_ARG_U8_MAX(args[ACE2K_RFID_STEP_ARG_OP], ACE2K_RFID_OP_MIFARE_B,
                                  "ace2k_rfid_step: op out of range");
    uint8_t arg = ACE2K_ARG_U8(args[ACE2K_RFID_STEP_ARG_ARG],
                               "ace2k_rfid_step: lane, op, sector, page or blocks out of bounds");
    uint8_t blocks =
        ACE2K_ARG_U8(args[ACE2K_RFID_STEP_ARG_BLOCKS],
                     "ace2k_rfid_step: lane, op, sector, page or blocks out of bounds");
    // The key comes last, as in the format.  The op is known by then, so an unknown op is never
    // taken for a short MIFARE key.  The length of the key is the byte count that Klipper gives for
    // the %*s, so it is compared whole.
    if (op != ACE2K_RFID_OP_NTAG && args[ACE2K_RFID_STEP_ARG_KEY_LEN] != ACE2K_ISO_KEY_BYTES) {
        shutdown("ace2k_rfid_step: a MIFARE step needs a six-byte key");
    }
    const uint8_t *key = command_decode_ptr(args[ACE2K_RFID_STEP_ARG_KEY]);
    int rc = ace2k_rfid_watch_step_request(&ace2k_rfid_watch_instance, lane, session, op, arg,
                                           blocks, key, ace2k_tick_now_ms());
    if (rc == -ACE2K_EINVAL) {
        shutdown("ace2k_rfid_step: lane, op, sector, page or blocks out of bounds");
    }
    // -ACE2K_EREFUSED: no such session, or a step already runs; a stale host, ignored
}
DECL_COMMAND(ace2k_rfid_cmd_step,
             "ace2k_rfid_step lane=%c session=%c op=%c arg=%c blocks=%c key=%*s");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_done(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_rfid_done: lane or result out of range");
    uint8_t session = ACE2K_ARG_U8(args[1], "ace2k_rfid_done: session out of range");
    bool was_read = ACE2K_ARG_FLAG(args[2], "ace2k_rfid_done: lane or result out of range");
    (void)ace2k_rfid_watch_done(&ace2k_rfid_watch_instance, lane, session, was_read);
}
DECL_COMMAND(ace2k_rfid_cmd_done, "ace2k_rfid_done lane=%c session=%c result=%c");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_forget(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_rfid_forget: lane out of range");
    ace2k_rfid_watch_host_forget(&ace2k_rfid_watch_instance, lane);
}
DECL_COMMAND(ace2k_rfid_cmd_forget, "ace2k_rfid_forget lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_lane_query(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_rfid_lane_query: lane out of range");
    const struct ace2k_iso14443a_tag *t = ace2k_rfid_watch_tag(&ace2k_rfid_watch_instance, lane);
    uint8_t state = ace2k_rfid_watch_state(&ace2k_rfid_watch_instance, lane);
    // the UID of a lane that has one attributed, reading or read; none otherwise
    bool has = (bool)(state == ACE2K_RFID_READING || state == ACE2K_RFID_READ);
    sendf("ace2k_rfid_lane_state lane=%c state=%c uid=%*s", lane, state,
          (uint8_t)(has ? t->uid_len : 0U), t->uid);
}
DECL_COMMAND(ace2k_rfid_cmd_lane_query, "ace2k_rfid_lane_query lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_query(uint32_t *args)
{
    ensure_ready();
    ace2k_report_query(&ace2k_rfid_report, &ace2k_rfid_report_due, args[0],
                       ACE2K_RFID_REPORT_PHASE_MS);
}
DECL_COMMAND(ace2k_rfid_cmd_query, "ace2k_rfid_query rest_ticks=%u");

// The host's mask of the lanes in use, a bit per lane: the lanes whose tip is in a head that
// prints, is paused or is otherwise in use.  The host sends it on every change and every second.
// It is published at once, so the feed's next tick sees a lane that just became in use.  A mask
// above the four lanes is a bug in the host.
// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_in_use(uint32_t *args)
{
    ensure_ready();
    uint8_t mask = ACE2K_ARG_U8_MAX(args[0], ACE2K_RFID_FEED_LANES_MASK,
                                    "ace2k_rfid_in_use: mask out of range");
    ace2k_rfid_watch_set_in_use(&ace2k_rfid_watch_instance, mask);
    publish();
}
DECL_COMMAND(ace2k_rfid_cmd_in_use, "ace2k_rfid_in_use mask=%c");

// The user named the lane's material by hand: no tag is attributed to it until its strand leaves.
// A lane with no filament ignores it.  ace2k_rfid_by_hand_state tells the host the marks.
// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_by_hand(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_rfid_by_hand: lane out of range");
    ace2k_rfid_watch_set_by_hand(&ace2k_rfid_watch_instance, lane);
}
DECL_COMMAND(ace2k_rfid_cmd_by_hand, "ace2k_rfid_by_hand lane=%c");

// The settings of the confirmation, sent by the host at connect and when the bench tunes them, in
// the order of enum ace2k_rfid_cfg_index.  A set with a value outside its bounds is ignored whole,
// and the refusal frame carries it back.  The host matches it with the set it sent last and logs
// it.  This one is not a shutdown: a tuning sent during a print must not stop it.  An accepted set
// reaches the feed too, which runs the legs.
static void confirm_set_refused(const uint32_t *v)
{
    sendf("ace2k_rfid_confirm_set_refused leg_max_um=%u speed_um_s=%u passive_min_um=%u"
          " follows=%c window_um=%u passive_max_um=%u",
          v[ACE2K_RFID_CFG_LEG], v[ACE2K_RFID_CFG_SPEED], v[ACE2K_RFID_CFG_PASSIVE_MIN],
          v[ACE2K_RFID_CFG_FOLLOWS], v[ACE2K_RFID_CFG_WINDOW], v[ACE2K_RFID_CFG_PASSIVE_MAX]);
}

// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_confirm_set(uint32_t *args)
{
    ensure_ready();
    if (ace2k_rfid_watch_confirm_set(&ace2k_rfid_watch_instance, args) != 0) {
        confirm_set_refused(args);
        return;
    }
#if CONFIG_ACE2K_FEED
    ace2k_feed_binding_confirm_cfg_set(args[ACE2K_RFID_CFG_LEG], args[ACE2K_RFID_CFG_SPEED]);
#endif
}
DECL_COMMAND(ace2k_rfid_cmd_confirm_set, "ace2k_rfid_confirm_set leg_max_um=%u speed_um_s=%u"
                                         " passive_min_um=%u follows=%c window_um=%u"
                                         " passive_max_um=%u");

#if CONFIG_ACE2K_FEED
// Klipper's DECL_CONSTANT encodes its value with signed literals inside the macro.  So the lint's
// signed-bitwise check is off for these declarations only.
// NOLINTBEGIN(bugprone-signed-bitwise)
DECL_CONSTANT("ACE2K_FEED_SEARCH_MAX_UM", ACE2K_FEED_SEARCH_MAX_UM);
// The default of the firmware for rfid_search_mm, which the host uses when printer.cfg does not set
// it.  The ACEK_LOAD budget of ace2k_feed.py and the MOVE=1 wait of ace2k_rfid.py read it.
DECL_CONSTANT("ACE2K_FEED_SEARCH_DEFAULT_UM", ACE2K_FEED_SEARCH_DEFAULT_UM);
// NOLINTEND(bugprone-signed-bitwise)

// The search's reach.  It is rfid_search_mm from printer.cfg, or the default.  It is sent as an
// init command at every connect.  A host older than v0.12.0 sends it as a config command, and only
// when the key is set.  Zero is a bug in the host, because the host never turns the search off.  A
// reach that the core refuses, past the limit (rule 3), is a bug in the host too.
// cppcheck-suppress constParameterPointer
void ace2k_rfid_cmd_search_set(uint32_t *args)
{
    ensure_ready();
    if (args[0] == 0 || ace2k_feed_binding_search_set(args[0]) != 0) {
        shutdown("ace2k_rfid_search_set: search_um out of bounds");
    }
}
DECL_COMMAND(ace2k_rfid_cmd_search_set, "ace2k_rfid_search_set search_um=%u");
#endif

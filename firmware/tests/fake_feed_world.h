/* What: the feed's test world, the real lane and feed cores on fake counters and lines.
 * How: world_init() primes both cores, and step() runs one 10 ms tick of each; header-only.
 * Depends on: feed/feed.h. */
#ifndef ACE2K_TEST_FAKE_FEED_WORLD_H
#define ACE2K_TEST_FAKE_FEED_WORLD_H
#include "feed/feed.h"

/* The world: the lane's fake counters and lines, the feed's inputs, one clock. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct world {
    uint16_t enc[ACE2K_LANE_COUNT];
    uint32_t fg[ACE2K_LANE_COUNT];
    uint8_t duty[ACE2K_LANE_COUNT];
    bool run[ACE2K_LANE_COUNT];
    bool reverse[ACE2K_LANE_COUNT];
    /* every forward start of the motor, even one stopped again within the same tick */
    uint32_t forward_starts[ACE2K_LANE_COUNT];
    struct ace2k_feed_inputs in;
    uint32_t now;
    struct ace2k_lane lane;
    struct ace2k_feed feed;
};

static inline uint16_t w_enc(void *ctx, uint8_t lane)
{
    return ((struct world *)ctx)->enc[lane];
}

static inline uint32_t w_fg(void *ctx, uint8_t lane)
{
    return ((struct world *)ctx)->fg[lane];
}

static inline void w_pwm(void *ctx, uint8_t lane, uint8_t duty_pct)
{
    ((struct world *)ctx)->duty[lane] = duty_pct;
}

static inline void w_run(void *ctx, uint8_t lane, bool on)
{
    struct world *w = ctx;
    w->run[lane] = on;
    if (on && !w->reverse[lane]) {
        w->forward_starts[lane]++;
    }
}

static inline void w_dir(void *ctx, uint8_t lane, bool reverse)
{
    ((struct world *)ctx)->reverse[lane] = reverse;
}

static const struct ace2k_lane_ops w_ops = {
    .encoder_read = w_enc,
    .fg_read = w_fg,
    .pwm_set = w_pwm,
    .run_set = w_run,
    .dir_set = w_dir,
};

/* Both cores primed at t = start_ms, the link up, filament in lane 1 (bit 0), every plunger at
 * rest.  A start a little below 2^32 puts the millisecond clock's wrap inside a test. */
static inline void world_init_at(struct world *w, uint32_t start_ms)
{
    *w = (struct world){ 0 };
    w->now = start_ms;
    w->in.link_ok = true;
    w->in.insert = 0x01;
    w->in.rest = 0x0F;
    ace2k_lane_init(&w->lane, &w_ops, w);
    ace2k_feed_init(&w->feed, &w->lane);
    w->feed.load.auto_load = false; /* the automatic load's own test arms it */
    ace2k_lane_tick(&w->lane, w->now, true);
    ace2k_feed_tick(&w->feed, w->now, &w->in);
}

/* The world at t = 10 ms. */
static inline void world_init(struct world *w)
{
    world_init_at(w, 10);
}

/* One 10 ms tick: lane 0's tach and encoder advance by the given amounts, both cores tick. */
static inline void step(struct world *w, uint32_t pulses, int counts)
{
    w->now += 10;
    w->fg[0] += pulses;
    w->enc[0] = (uint16_t)(w->enc[0] + counts);
    ace2k_lane_tick(&w->lane, w->now, w->in.link_ok);
    ace2k_feed_tick(&w->feed, w->now, &w->in);
}

static inline void steps(struct world *w, int n, uint32_t pulses, int counts)
{
    for (int i = 0; i < n; i++) {
        step(w, pulses, counts);
    }
}

/* n ticks with the tach at `pulses` a tick and the encoder advancing count_step (+1 forward, −1
 * in reverse) once every `every` ticks — on the every-th, 2·every-th … tick of the call: n / every
 * counts in all.  A move ends on the encoder (the length is the strand's travel), so this is what
 * finishes one: 4 pulses a tick is ≈ 32 mm/s of motor, a count every four ticks ≈ 31 mm/s of
 * strand — the few per cent the drive gear slips under a spool.  At 1234.2 µm a count, 10 mm
 * is 8 counts, 20 mm 16, 50 mm 41 and 100 mm 81. */
static inline void paced(struct world *w, int n, uint32_t pulses, int every, int count_step)
{
    for (int i = 0; i < n; i++) {
        step(w, pulses, ((i + 1) % every == 0) ? count_step : 0);
    }
}

/* n ticks with a tach pulse every `every` ticks and the strand still: a motor turning too slowly
 * for the standstill's 20 mm to come before a short move's deadline, yet never silent for the
 * lane's own second. */
static inline void crawl(struct world *w, int n, int every)
{
    for (int i = 0; i < n; i++) {
        step(w, ((i + 1) % every == 0) ? 1U : 0U, 0);
    }
}

static inline bool pop(struct world *w, struct ace2k_feed_event *e)
{
    return ace2k_feed_pop_event(&w->feed, e);
}

#endif

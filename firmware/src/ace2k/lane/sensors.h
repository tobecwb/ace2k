/* What: the lanes' insert sensors (analogue, a band per lane), the *empty* inputs (millivolts
 * only), the buffer Hall switches and the cutout input, debounced, with every edge as an event.
 * How: the binding owns one struct ace2k_sensors; it calls ace2k_sensors_tick() every 10 ms
 * with the last ADC pass and the raw digital levels; from a task it peeks the oldest event,
 * tries the frame, and drops the event once it is queued.  Insert: IIR α = 0.2 on the
 * millivolts, then the band's hysteresis — the reading falls when a strand presses the lever:
 * present below 30 % of the band, absent above 70 % (measured on the unit 2026-09-17) — then two
 * identical ticks (docs/hardware.md "Analogue inputs" — the factory firmware's rule); switches:
 * two identical ticks.  The first tick primes every state silently.  The event ring is
 * single-producer (the tick) / single-consumer (the task); each side publishes its index after
 * the payload, behind a compiler barrier.
 * Depends on: <stdbool.h>, <stddef.h>, <stdint.h>, adc_map.h, config.h (struct
 * ace2k_insert_band), util.h. */
#ifndef ACE2K_SENSORS_H
#define ACE2K_SENSORS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lane/adc_map.h"
#include "core/config.h"
#include "core/util.h"

/* Bits of `levels` in struct ace2k_sensors_inputs: 1 = the pin reads high. */
enum ace2k_sensors_input {
    ACE2K_SENSORS_IN_REST1 = 0, /* lanes 1–4 contiguous */
    ACE2K_SENSORS_IN_PUSHED1 = 4,
    ACE2K_SENSORS_IN_PULLED = 8,
    ACE2K_SENSORS_IN_CUTOUT = 9,
    ACE2K_SENSORS_IN_COUNT = 10,
};

/* Bits of ace2k_sensors_switches(): 1 = asserted. */
#define ACE2K_SENSORS_STATE_INSERT_SHIFT 0U
#define ACE2K_SENSORS_STATE_REST_SHIFT   4U
#define ACE2K_SENSORS_STATE_PUSHED_SHIFT 8U
#define ACE2K_SENSORS_STATE_PULLED_BIT   12U
#define ACE2K_SENSORS_STATE_CUTOUT_BIT   13U

enum ace2k_sensors_kind {
    ACE2K_SENSORS_KIND_INSERT = 0,
    ACE2K_SENSORS_KIND_REST = 1,
    ACE2K_SENSORS_KIND_PUSHED = 2,
    ACE2K_SENSORS_KIND_PULLED = 3, /* lane ACE2K_SENSORS_LANE_NONE */
    ACE2K_SENSORS_KIND_CUTOUT = 4, /* lane ACE2K_SENSORS_LANE_NONE */
};

#define ACE2K_SENSORS_LANE_NONE        255U
#define ACE2K_SENSORS_EVENT_RING       16U
#define ACE2K_SENSORS_IIR_DIV          5   /* α = 1/5 */
#define ACE2K_SENSORS_FILT_SHIFT       4U  /* the filter keeps millivolts × 16 */
#define ACE2K_SENSORS_TRIP_PRESENT_PCT 30U /* the reading falls to this point: present */
#define ACE2K_SENSORS_TRIP_ABSENT_PCT  70U /* rises to this point: absent */
#define ACE2K_SENSORS_FRESH_MS         50U
#define ACE2K_SENSORS_RAIL_LOW_MV      20U   /* initial value; adjust on the bench */
#define ACE2K_SENSORS_RAIL_HIGH_MV     3250U /* initial value; adjust on the bench */

struct ace2k_sensors_inputs {
    uint16_t mv[ACE2K_ADC_COUNT]; /* the last ADC pass, millivolts */
    uint32_t adc_age_ms;          /* age of that pass at this tick; ignored while !adc_valid */
    bool adc_valid;               /* false until the first pass exists */
    uint16_t levels;              /* enum ace2k_sensors_input bits, 1 = high */
};

struct ace2k_sensors_event {
    uint8_t lane; /* 0–3, or ACE2K_SENSORS_LANE_NONE */
    uint8_t kind; /* enum ace2k_sensors_kind */
    uint8_t level;
};

struct ace2k_sensors_lane {
    struct ace2k_insert_band band;
    int32_t insert_filt16; /* millivolts × 16 */
    bool insert_at_rail;   /* the last sample was at a rail: the filter is reseeded, not stepped */
    bool insert_raw;       /* the hysteresis output before the debounce */
    bool insert_last;
    bool insert;
    uint16_t insert_mv;
    uint16_t empty_mv;
    bool rest_last, rest;
    bool pushed_last, pushed;
    /* rest && pushed as of this tick, written once after both debounces.  The health
     * self-test reads it from task context, and the tick can flip the pair in one pass: a
     * lane moving rest → pushed would read as both for the width of one interrupt if the
     * reader combined the two flags itself.  One byte cannot tear. */
    bool rest_and_pushed;
};

struct ace2k_sensors {
    struct ace2k_sensors_lane lane[ACE2K_LANE_COUNT];
    bool pulled_last, pulled;
    bool cutout_last, cutout;
    uint16_t aux_mv;
    uint32_t adc_age_ms; /* the last pass's age; 0 while adc_valid is false (no pass yet) */
    bool adc_valid;
    bool primed;     /* the switches have seen their first tick */
    bool adc_primed; /* the insert filters have seen their first pass */
    struct ace2k_sensors_event ring[ACE2K_SENSORS_EVENT_RING];
    uint8_t head, tail;
    uint32_t dropped;
};

/* Every state cleared; the bands copied from bands (the factory read, or the defaults). */
void ace2k_sensors_init(struct ace2k_sensors *self,
                        const struct ace2k_insert_band bands[ACE2K_LANE_COUNT]);

/* 0, or -ACE2K_EINVAL for a lane out of range or low_mv >= high_mv.  Takes effect on the
 * next tick; the debounced state is not reset. */
int ace2k_sensors_set_band(struct ace2k_sensors *self, uint8_t lane, uint16_t low_mv,
                           uint16_t high_mv, enum ace2k_calibration_source source);

/* The lane's band in use; NULL for a lane out of range, as set_band refuses one. */
const struct ace2k_insert_band *ace2k_sensors_band(const struct ace2k_sensors *self, uint8_t lane);

/* One 10 ms tick.  Interrupt context: no blocking, no allocation. */
void ace2k_sensors_tick(struct ace2k_sensors *self, uint32_t now_ms,
                        const struct ace2k_sensors_inputs *in);

uint16_t ace2k_sensors_switches(const struct ace2k_sensors *self);

/* The oldest event, left in the ring: the binding tries to send it and pops it only once the
 * frame is queued, so a frame that does not fit waits at the ring's tail for the next tick.
 * The tick writes the head alone and never the slot at the tail (one slot stays free), so the
 * peek and the pop need no mask.  False when the ring is empty. */
bool ace2k_sensors_peek_event(const struct ace2k_sensors *self, struct ace2k_sensors_event *out);
/* The oldest event released without a copy — the binding's step once the peeked frame is queued.
 * Nothing when the ring is empty. */
void ace2k_sensors_drop_event(struct ace2k_sensors *self);
/* Oldest event first, copied and released (a peek and a drop); false when the ring is empty. */
bool ace2k_sensors_pop_event(struct ace2k_sensors *self, struct ace2k_sensors_event *out);

/* True while the ring holds an event; safe from any context. */
bool ace2k_sensors_has_events(const struct ace2k_sensors *self);

/* The last pass is at most ACE2K_SENSORS_FRESH_MS old. */
bool ace2k_sensors_fresh(const struct ace2k_sensors *self);

/* The lane's insert reading is at neither rail (ACE2K_SENSORS_RAIL_LOW_MV < mv <
 * ACE2K_SENSORS_RAIL_HIGH_MV); false with no pass or no such lane. */
bool ace2k_sensors_insert_connected(const struct ace2k_sensors *self, uint8_t lane);

/* Not both *rest* and *pushed* at once, as of the last tick (one byte read: safe from any
 * context); false for no such lane. */
bool ace2k_sensors_buffer_consistent(const struct ace2k_sensors *self, uint8_t lane);

#endif

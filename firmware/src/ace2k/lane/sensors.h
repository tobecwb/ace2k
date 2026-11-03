/* What: the insert sensors of the lanes, the *empty* inputs, the buffer Hall switches and the
 * cutout input.  Each is debounced, and every edge becomes an event.  An insert sensor is analogue,
 * with a band per lane.  The *empty* inputs are only reported in millivolts.
 * How: the binding owns one struct ace2k_sensors.  It calls ace2k_sensors_tick() every 10 ms with
 * the last ADC pass and the raw digital levels.  From a task it peeks the oldest event, tries the
 * frame, and drops the event once it is queued.
 * - Insert: an IIR filter with α = 0.2 on the millivolts, then the hysteresis of the band, then two
 *   identical ticks (docs/hardware.md "Analogue inputs", the rule of the factory firmware).  The
 *   reading falls when filament presses the lever.  So filament is present at or below 30 % of the
 *   band, and absent at or above 70 % (measured on the unit).
 * - Switches: two identical ticks.
 * The first tick primes every state without an event.  The event ring has one producer, the tick,
 * and one consumer, the task.  Each side publishes its index after the payload, behind a compiler
 * barrier.
 * - A pass older than ACE2K_SENSORS_ADC_STALE_MS (the scan stopped) says nothing about the
 *   inserts.  They are unknown (ace2k_sensors_insert_known): their filters and debounces stand
 *   still, and no insert event goes out.  The first fresh pass reseeds the filters, so it is not
 *   blended with the frozen one.  A real change since then makes its edge as always.  A flash
 *   write stalls the scan without stopping it, so for ACE2K_CATCHUP_SETTLE_MS after the tick's
 *   catch-up (util.h) a stale pass keeps the last verdict.  The auxiliary input is read from the
 *   same passes as the inserts.
 * Depends on: <stdbool.h>, <stddef.h>, <stdint.h>, adc_map.h (the channels, and the staleness
 * bound of core/adc_timing.h), calibration.h (struct ace2k_insert_band), util.h (the settle after
 * a catch-up). */
#ifndef ACE2K_SENSORS_H
#define ACE2K_SENSORS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lane/adc_map.h"
#include "core/calibration.h"
#include "core/util.h"

/* The bits of `levels` in struct ace2k_sensors_inputs.  1 means that the pin reads high. */
enum ace2k_sensors_input {
    ACE2K_SENSORS_IN_REST1 = 0, /* lanes 1–4 contiguous */
    ACE2K_SENSORS_IN_PUSHED1 = 4,
    ACE2K_SENSORS_IN_PULLED = 8,
    ACE2K_SENSORS_IN_CUTOUT = 9,
    ACE2K_SENSORS_IN_COUNT = 10,
};

/* The bits of ace2k_sensors_switches().  1 means asserted. */
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
#define ACE2K_SENSORS_IIR_DIV          5     /* initial value; re-tune on the bench (α = 1/5) */
#define ACE2K_SENSORS_FILT_SHIFT       4U    /* the filter keeps millivolts × 16 */
#define ACE2K_SENSORS_TRIP_PRESENT_PCT 30U   /* the reading falls to this point: present */
#define ACE2K_SENSORS_TRIP_ABSENT_PCT  70U   /* rises to this point: absent */
#define ACE2K_SENSORS_FRESH_MS         50U   /* initial value; re-tune on the bench */
#define ACE2K_SENSORS_RAIL_LOW_MV      20U   /* initial value; re-tune on the bench */
#define ACE2K_SENSORS_RAIL_HIGH_MV     3250U /* initial value; re-tune on the bench */
/* A pass older than this leaves the inserts unknown.  It is the bound past which env invalidates
 * the NTCs and VDDA.  core/adc_timing.h sets the one bound, so the two never disagree on a stopped
 * scan. */
#define ACE2K_SENSORS_ADC_STALE_MS ACE2K_ADC_STALE_MS
/* The period of the tick.  The binding checks that the board's tick agrees. */
#define ACE2K_SENSORS_TICK_PERIOD_MS 10U

struct ace2k_sensors_inputs {
    uint16_t mv[ACE2K_ADC_COUNT]; /* the last ADC pass, millivolts */
    uint32_t adc_age_ms;          /* age of that pass at this tick; ignored while !adc_valid */
    bool adc_valid;               /* false until the first pass exists */
    uint32_t now_ms;              /* the tick's clock; a late tick is a stall's catch-up */
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
    /* rest && pushed as of this tick, written once after both debounces.  The health self-test
     * reads it from task context.  The tick can flip the pair in one pass.  So if the reader
     * combined the two flags itself, a lane moving from rest to pushed would read as both for the
     * length of one interrupt.  One byte cannot tear. */
    bool rest_and_pushed;
};

struct ace2k_sensors {
    struct ace2k_sensors_lane lane[ACE2K_LANE_COUNT];
    bool pulled_last, pulled;
    bool cutout_last, cutout;
    uint16_t aux_mv;
    uint32_t adc_age_ms; /* age of the last pass; 0 while adc_valid is false (no pass yet) */
    bool adc_valid;
    bool primed;     /* the switches have seen their first tick */
    bool adc_primed; /* the insert filters have seen their first pass */
    /* The last pass is fresh enough to read the inserts (ACE2K_SENSORS_ADC_STALE_MS).  False
     * before the first pass.  One byte, written once per tick: safe from any context. */
    bool insert_known;
    bool insert_reseed; /* a stale pass was seen: the next fresh one reseeds the insert filters */
    struct ace2k_catchup catchup; /* a stall's catch-up holds the verdict for a moment */
    struct ace2k_sensors_event ring[ACE2K_SENSORS_EVENT_RING];
    volatile uint8_t head, tail; /* the tick writes head, and the task writes tail */
    uint32_t dropped;
};

/* Clears every state, and copies the bands from bands, which come from the factory read or the
 * defaults. */
void ace2k_sensors_init(struct ace2k_sensors *self,
                        const struct ace2k_insert_band bands[ACE2K_LANE_COUNT]);

/* Sets the band of a lane.  It returns 0, or -ACE2K_EINVAL for a lane out of range or for low_mv >=
 * high_mv.  The band takes effect on the next tick, and the debounced state is not reset. */
int ace2k_sensors_set_band(struct ace2k_sensors *self, uint8_t lane, uint16_t low_mv,
                           uint16_t high_mv, enum ace2k_calibration_source source);

/* Returns the band in use for the lane, or NULL for a lane out of range, as set_band refuses
 * one. */
const struct ace2k_insert_band *ace2k_sensors_band(const struct ace2k_sensors *self, uint8_t lane);

/* Runs one 10 ms tick, in interrupt context: no blocking, no allocation. */
void ace2k_sensors_tick(struct ace2k_sensors *self, const struct ace2k_sensors_inputs *in);

uint16_t ace2k_sensors_switches(const struct ace2k_sensors *self);

/* Copies the oldest event and leaves it in the ring.  The binding tries to send it, and pops it
 * only once the frame is queued.  So a frame that does not fit waits at the ring's tail for the
 * next tick.  The tick writes only the head, and never the slot at the tail, because one slot stays
 * free.  So the peek and the pop need no mask.  It returns false when the ring is empty. */
bool ace2k_sensors_peek_event(const struct ace2k_sensors *self, struct ace2k_sensors_event *out);
/* Releases the oldest event without a copy.  This is the binding's step once the peeked frame is
 * queued.  It does nothing when the ring is empty. */
void ace2k_sensors_drop_event(struct ace2k_sensors *self);
/* Copies and releases the oldest event, as a peek and a drop.  It returns false when the ring is
 * empty.  For the tests. */
bool ace2k_sensors_pop_event(struct ace2k_sensors *self, struct ace2k_sensors_event *out);

/* Returns true while the ring holds an event.  Safe from any context. */
bool ace2k_sensors_has_events(const struct ace2k_sensors *self);

/* Returns true when the insert readings say something: a pass exists and is at most
 * ACE2K_SENSORS_ADC_STALE_MS old.  While false, the lanes' insert states are their last ones,
 * frozen, and a caller must not act on them. */
bool ace2k_sensors_insert_known(const struct ace2k_sensors *self);

/* Returns true when the last pass is at most ACE2K_SENSORS_FRESH_MS old. */
bool ace2k_sensors_fresh(const struct ace2k_sensors *self);

/* Returns true when the lane's insert reading is at neither rail (ACE2K_SENSORS_RAIL_LOW_MV < mv <
 * ACE2K_SENSORS_RAIL_HIGH_MV).  It returns false with no pass, or for no such lane. */
bool ace2k_sensors_insert_connected(const struct ace2k_sensors *self, uint8_t lane);

/* Returns true when the lane is not *rest* and *pushed* at once, as of the last tick.  It reads one
 * byte, so it is safe from any context.  It returns false for no such lane. */
bool ace2k_sensors_buffer_consistent(const struct ace2k_sensors *self, uint8_t lane);

#endif

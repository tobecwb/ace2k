#include "lane/sensors.h"

static void push_event(struct ace2k_sensors *self, uint8_t lane, uint8_t kind, bool level)
{
    uint8_t next = (uint8_t)((self->head + 1U) % ACE2K_SENSORS_EVENT_RING);
    if (next == self->tail) {
        self->dropped++;
        return;
    }
    self->ring[self->head].lane = lane;
    self->ring[self->head].kind = kind;
    self->ring[self->head].level = level ? 1U : 0U;
    ACE2K_COMPILER_BARRIER(); /* payload before index */
    self->head = next;
}

/* Two identical samples move the state.  It returns true on a change.  Before priming, it seeds the
 * state without an event. */
static bool debounce(bool *state, bool *last, bool level, bool primed)
{
    bool changed = false;
    if (!primed) {
        *state = level;
    } else if (level == *last && level != *state) {
        *state = level;
        changed = true;
    }
    *last = level;
    return changed;
}

/* The level of the input.  It is widened to uint32_t first, because a uint16_t operand promotes to
 * int, and the lint refuses a bitwise operator on a signed operand. */
static bool level_of(const struct ace2k_sensors_inputs *in, unsigned bit)
{
    return (((uint32_t)in->levels >> bit) & 1U) != 0;
}

void ace2k_sensors_init(struct ace2k_sensors *self,
                        const struct ace2k_insert_band bands[ACE2K_LANE_COUNT])
{
    *self = (struct ace2k_sensors){ 0 };
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        self->lane[i].band = bands[i];
    }
}

int ace2k_sensors_set_band(struct ace2k_sensors *self, uint8_t lane, uint16_t low_mv,
                           uint16_t high_mv, enum ace2k_calibration_source source)
{
    if (lane >= ACE2K_LANE_COUNT || low_mv >= high_mv) {
        return -ACE2K_EINVAL;
    }
    self->lane[lane].band.low_mv = low_mv;
    self->lane[lane].band.high_mv = high_mv;
    self->lane[lane].band.source = source;
    return 0;
}

const struct ace2k_insert_band *ace2k_sensors_band(const struct ace2k_sensors *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return NULL;
    }
    return &self->lane[lane].band;
}

/* True at either end of the scale.  This is the one test behind the rail gate of the insert filter
 * and behind ace2k_sensors_insert_connected(). */
static bool at_rail(uint16_t mv)
{
    return (bool)(mv <= ACE2K_SENSORS_RAIL_LOW_MV || mv >= ACE2K_SENSORS_RAIL_HIGH_MV);
}

static void tick_insert(struct ace2k_sensors *self, uint8_t i,
                        const struct ace2k_sensors_inputs *in)
{
    struct ace2k_sensors_lane *l = &self->lane[i];
    uint16_t mv = in->mv[ACE2K_ADC_INSERT1 + i];
    int32_t mv16 = (int32_t)((uint32_t)mv << ACE2K_SENSORS_FILT_SHIFT);
    bool rail = at_rail(mv);
    l->insert_mv = mv;
    l->empty_mv = in->mv[ACE2K_ADC_EMPTY1 + i];
    /* A sample at a rail is not filament.  A disconnected lever board reads at one end of the
     * scale, and a shorted one at the other.  The low rail is inside the "present" region of every
     * band.  Such a lane reads absent for the tick, which is how the health mask says why.  The
     * filter is reseeded rather than stepped.  This happens at the rail, and once more on the first
     * sample back between the rails.  So the hysteresis restarts from the real value.  A filter
     * left integrating at the low rail would climb through the "present" band on the way back, and
     * report filament that is not there. */
    if (!self->adc_primed || self->insert_reseed || rail || l->insert_at_rail) {
        l->insert_filt16 = mv16;
    } else {
        l->insert_filt16 += (mv16 - l->insert_filt16) / ACE2K_SENSORS_IIR_DIV;
    }
    l->insert_at_rail = rail;
    uint32_t filt_mv = (uint32_t)l->insert_filt16 >> ACE2K_SENSORS_FILT_SHIFT;
    uint32_t band = (uint32_t)l->band.high_mv - l->band.low_mv;
    uint32_t present_mv = l->band.low_mv + (band * ACE2K_SENSORS_TRIP_PRESENT_PCT / ACE2K_PCT_FULL);
    uint32_t absent_mv = l->band.low_mv + (band * ACE2K_SENSORS_TRIP_ABSENT_PCT / ACE2K_PCT_FULL);
    /* Filament present is the low reading.  Filament presses the lever and the reading falls
     * (measured on the unit).  So filament is present at or below 30 % of
     * the band, and absent at or above 70 %. */
    if (filt_mv <= present_mv) {
        l->insert_raw = true;
    } else if (filt_mv >= absent_mv) {
        l->insert_raw = false;
    }
    if (rail) {
        l->insert_raw = false;
    }
    if (debounce(&l->insert, &l->insert_last, l->insert_raw, self->adc_primed)) {
        push_event(self, i, ACE2K_SENSORS_KIND_INSERT, l->insert);
    }
}

/* A stopped scan leaves the last pass in place with a growing age.  Its insert readings are the
 * frozen ones, so a spool that runs out meanwhile would never show.  The inserts go unknown
 * instead, and stand still until a fresh pass, which reseeds the filters.
 * A flash write stalls the core, and the scan with it, without stopping it.  The pass from before
 * the stall can pass the bound on the tick's catch-up, and for the two periods the next pass takes
 * (util.h, ace2k_catchup_settled).  For ACE2K_CATCHUP_SETTLE_MS after the catch-up, a stale pass
 * keeps the last verdict: known stays known, on the readings from just before the stall, and
 * unknown stays unknown.  It steps nothing.  A scan that is still stopped after it goes unknown.
 * Returns true when the pass was read, so the caller reads the rest of it too. */
static bool tick_inserts(struct ace2k_sensors *self, const struct ace2k_sensors_inputs *in)
{
    bool settled = ace2k_catchup_settled(&self->catchup, in->now_ms, ACE2K_SENSORS_TICK_PERIOD_MS);
    bool fresh_pass = (bool)(in->adc_valid && in->adc_age_ms <= ACE2K_SENSORS_ADC_STALE_MS);
    if (!fresh_pass && in->adc_valid && !settled) {
        return false;
    }
    self->insert_known = fresh_pass;
    if (!fresh_pass) {
        if (in->adc_valid) {
            self->insert_reseed = true;
        }
        return false;
    }
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        tick_insert(self, i, in);
    }
    self->adc_primed = true;
    self->insert_reseed = false;
    return true;
}

void ace2k_sensors_tick(struct ace2k_sensors *self, const struct ace2k_sensors_inputs *in)
{
    self->adc_valid = in->adc_valid;
    /* The age of the input means nothing without a pass.  So it is 0, and the report and the
     * freshness test never see an undefined value. */
    self->adc_age_ms = in->adc_valid ? in->adc_age_ms : 0;
    /* The auxiliary input is read with the inserts, from a pass they read.  So the report never
     * pairs it with insert readings of another pass. */
    if (tick_inserts(self, in)) {
        self->aux_mv = in->mv[ACE2K_ADC_AUX];
    }
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        struct ace2k_sensors_lane *l = &self->lane[i];
        /* active low: asserted when the pin reads 0 */
        if (debounce(&l->rest, &l->rest_last, (bool)!level_of(in, ACE2K_SENSORS_IN_REST1 + i),
                     self->primed)) {
            push_event(self, i, ACE2K_SENSORS_KIND_REST, l->rest);
        }
        if (debounce(&l->pushed, &l->pushed_last, (bool)!level_of(in, ACE2K_SENSORS_IN_PUSHED1 + i),
                     self->primed)) {
            push_event(self, i, ACE2K_SENSORS_KIND_PUSHED, l->pushed);
        }
        /* one store, after both debounces; the health self-test reads it (sensors.h) */
        l->rest_and_pushed = (bool)(l->rest && l->pushed);
    }
    if (debounce(&self->pulled, &self->pulled_last, (bool)!level_of(in, ACE2K_SENSORS_IN_PULLED),
                 self->primed)) {
        push_event(self, ACE2K_SENSORS_LANE_NONE, ACE2K_SENSORS_KIND_PULLED, self->pulled);
    }
    /* the cutout input idles low (docs/hardware.md); asserted means high */
    if (debounce(&self->cutout, &self->cutout_last, level_of(in, ACE2K_SENSORS_IN_CUTOUT),
                 self->primed)) {
        push_event(self, ACE2K_SENSORS_LANE_NONE, ACE2K_SENSORS_KIND_CUTOUT, self->cutout);
    }
    self->primed = true;
}

uint16_t ace2k_sensors_switches(const struct ace2k_sensors *self)
{
    uint16_t mask = 0;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        const struct ace2k_sensors_lane *l = &self->lane[i];
        mask |= (uint16_t)((l->insert ? 1U : 0U) << (ACE2K_SENSORS_STATE_INSERT_SHIFT + i));
        mask |= (uint16_t)((l->rest ? 1U : 0U) << (ACE2K_SENSORS_STATE_REST_SHIFT + i));
        mask |= (uint16_t)((l->pushed ? 1U : 0U) << (ACE2K_SENSORS_STATE_PUSHED_SHIFT + i));
    }
    mask |= (uint16_t)((self->pulled ? 1U : 0U) << ACE2K_SENSORS_STATE_PULLED_BIT);
    mask |= (uint16_t)((self->cutout ? 1U : 0U) << ACE2K_SENSORS_STATE_CUTOUT_BIT);
    return mask;
}

bool ace2k_sensors_peek_event(const struct ace2k_sensors *self, struct ace2k_sensors_event *out)
{
    if (self->tail == self->head) {
        return false;
    }
    *out = self->ring[self->tail];
    return true;
}

void ace2k_sensors_drop_event(struct ace2k_sensors *self)
{
    if (self->tail == self->head) {
        return;
    }
    ACE2K_COMPILER_BARRIER(); /* the copy, if any, before the release */
    self->tail = (uint8_t)((self->tail + 1U) % ACE2K_SENSORS_EVENT_RING);
}

bool ace2k_sensors_pop_event(struct ace2k_sensors *self, struct ace2k_sensors_event *out)
{
    if (!ace2k_sensors_peek_event(self, out)) {
        return false;
    }
    ace2k_sensors_drop_event(self);
    return true;
}

bool ace2k_sensors_has_events(const struct ace2k_sensors *self)
{
    return self->head != self->tail;
}

bool ace2k_sensors_insert_known(const struct ace2k_sensors *self)
{
    return self->insert_known;
}

bool ace2k_sensors_fresh(const struct ace2k_sensors *self)
{
    if (!self->adc_valid) {
        return false;
    }
    return self->adc_age_ms <= ACE2K_SENSORS_FRESH_MS;
}

bool ace2k_sensors_insert_connected(const struct ace2k_sensors *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT || !self->adc_valid) {
        return false;
    }
    if (at_rail(self->lane[lane].insert_mv)) {
        return false;
    }
    return true;
}

bool ace2k_sensors_buffer_consistent(const struct ace2k_sensors *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return false;
    }
    if (self->lane[lane].rest_and_pushed) {
        return false;
    }
    return true;
}

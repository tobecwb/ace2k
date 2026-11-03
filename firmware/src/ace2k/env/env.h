/* What: the environment readings, each with a valid bit: the two outlet NTCs in millidegrees,
 * the temperature and humidity of the chamber sensor, and VDDA from the internal reference.
 * How: the binding owns one struct ace2k_env.
 * - ace2k_env_tick() runs every 10 ms, in interrupt context. It takes the NTC millivolts, the raw
 *   reference and the age of the ADC pass they come from. It does a table lookup and a filter.
 * - ace2k_env_task() runs from a task. It talks I²C through ops.
 * The chamber sensor is an AHT20. One cycle per second: read the status, send `BE 08 00` if it is
 * uncalibrated, send `AC 33 00`, wait ≥ 80 ms, then read 7 bytes with a CRC-8 (Aosong
 * datasheet). No floats: the NTC curve is in env_table.h.
 * Depends on: <stdbool.h>, <stddef.h>, <stdint.h>, core/adc_timing.h (the pass's staleness bound);
 * env.c on env_table.h and util.h. */
#ifndef ACE2K_ENV_H
#define ACE2K_ENV_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "core/adc_timing.h"

#define ACE2K_ENV_NTC_MIN_MV        100U /* the valid window, docs/hardware.md */
#define ACE2K_ENV_NTC_MAX_MV        3200U
#define ACE2K_ENV_NTC_FILTER_DIV    8 /* α = 1/8 on the millivolts */
#define ACE2K_ENV_FILT_SHIFT        4U
#define ACE2K_ENV_VREFINT_MV        1200U /* GD32F30x datasheet, V_REFINT typical */
#define ACE2K_ENV_ADC_FULL_SCALE    4095U
#define ACE2K_ENV_VDDA_MIN_MV       3135U /* 3.3 V − 5 % */
#define ACE2K_ENV_VDDA_MAX_MV       3465U /* 3.3 V + 5 % */
#define ACE2K_ENV_AHT_ADDR          0x38U
#define ACE2K_ENV_AHT_PERIOD_MS     1000U
#define ACE2K_ENV_AHT_MEASURE_MS    80U
#define ACE2K_ENV_AHT_INIT_MS       10U
#define ACE2K_ENV_AHT_BUSY_RETRY_MS 10U
#define ACE2K_ENV_AHT_BUSY_RETRIES  3U
/* The one bound on the age of the chamber reading. A reading older than this counts as no
 * reading, for the valid bit here and for the dryer. The AHT is read once per
 * ACE2K_ENV_AHT_PERIOD_MS. After a failed read, the next good one comes a little over two periods
 * after the last good one: a period, then the measurement, then any busy retries. So a bound of
 * two periods dropped the bit for a moment on every lone I2C failure. Five periods ride out three
 * failed reads in a row. They ride out four when it is the status read that fails, because a
 * failed measurement read also costs the 80 ms measurement. */
#define ACE2K_ENV_CHAMBER_STALE_MS 5000U
#define ACE2K_ENV_AHT_FRAME_LEN    7U
#define ACE2K_ENV_AHT_STATUS_BUSY  0x80U
#define ACE2K_ENV_AHT_STATUS_CAL   0x08U
#define ACE2K_ENV_CHAMBER_MIN_MC   (-40000)
#define ACE2K_ENV_CHAMBER_MAX_MC   85000
#define ACE2K_ENV_RH_MAX_PCT10     1000U
/* The ADC pass's period and staleness bound (core/adc_timing.h), which the inserts share. */
#define ACE2K_ENV_ADC_SCAN_PERIOD_MS ACE2K_ADC_PASS_PERIOD_MS
#define ACE2K_ENV_ADC_STALE_MS       ACE2K_ADC_STALE_MS

enum ace2k_env_valid {
    ACE2K_ENV_VALID_NTC_LEFT = 0x1U,
    ACE2K_ENV_VALID_NTC_RIGHT = 0x2U,
    ACE2K_ENV_VALID_CHAMBER = 0x4U,
    ACE2K_ENV_VALID_VDDA = 0x8U,
};

enum ace2k_env_aht_phase {
    ACE2K_ENV_AHT_IDLE,      /* waiting for the next cycle */
    ACE2K_ENV_AHT_INIT_SENT, /* BE 08 00 sent, trigger after ACE2K_ENV_AHT_INIT_MS */
    ACE2K_ENV_AHT_TRIGGERED, /* AC 33 00 sent, read after ACE2K_ENV_AHT_MEASURE_MS */
};

/* Each returns 0 on success, and anything else on a bus failure (a NACK, a timeout). */
struct ace2k_env_ops {
    int (*i2c_write)(void *ctx, uint8_t addr, const uint8_t *data, uint8_t len);
    int (*i2c_read)(void *ctx, uint8_t addr, uint8_t *data, uint8_t len);
};

struct ace2k_env {
    const struct ace2k_env_ops *ops;
    void *ctx;
    int32_t ntc_filt16[2]; /* left, right: millivolts × 16 */
    bool ntc_primed;
    uint16_t ntc_mv[2];
    int32_t ptc_left_mc, ptc_right_mc;
    /* written by the task, read by the tick */
    volatile int32_t chamber_mc;
    volatile uint16_t chamber_rh_pct10;
    volatile uint32_t chamber_at_ms;
    volatile bool chamber_ever;
    uint16_t vdda_mv;
    uint8_t valid; /* enum ace2k_env_valid bits */
    enum ace2k_env_aht_phase aht_phase;
    volatile uint32_t aht_next_ms; /* written by the task, read by the tick */
    uint8_t aht_busy_retries;
    uint32_t aht_failures;
};

/* Returns the millidegrees for mv inside the window. Outside it, sets *valid false and returns
 * 0. */
int32_t ace2k_env_ntc_mc(uint16_t mv, bool *valid);

/* The CRC-8 of the AHT20: polynomial 0x31, initial 0xFF, no reflection. */
uint8_t ace2k_env_crc8(const uint8_t *data, size_t len);

/* Decodes a 7-byte frame. Returns false when the sensor is busy or when the CRC fails. */
bool ace2k_env_aht_decode(const uint8_t frame[ACE2K_ENV_AHT_FRAME_LEN], int32_t *mc,
                          uint16_t *rh_pct10);

/* Sets everything to zero and nothing valid. The ticks bring the NTCs and VDDA, and the task
 * brings the chamber. The task's first step is due at once. */
void ace2k_env_init(struct ace2k_env *self, const struct ace2k_env_ops *ops, void *ctx,
                    uint32_t now_ms);

/* Interrupt context. adc_age_ms is the age of the ADC pass that the readings come from. Past
 * ACE2K_ENV_ADC_STALE_MS, both NTC readings and VDDA are invalid and read 0, as for a broken
 * sensor. The next fresh pass starts the NTC filter again. vrefint_raw is the 12-bit reading of
 * the internal reference. */
void ace2k_env_tick(struct ace2k_env *self, uint32_t now_ms, uint32_t adc_age_ms,
                    uint16_t ntc_left_mv, uint16_t ntc_right_mv, uint16_t vrefint_raw);

/* True once the next deadline of the chamber state machine has passed. The binding's tick
 * wakes the task on it, rather than every 10 ms. It is one aligned read of a deadline that the task
 * writes, so it is safe from any context. */
bool ace2k_env_task_due(const struct ace2k_env *self, uint32_t now_ms);

/* Task context: the state machine of the chamber sensor. It runs one step when
 * ace2k_env_task_due() is true. Calling it early does nothing. */
void ace2k_env_task(struct ace2k_env *self, uint32_t now_ms);

/* True when the last chamber reading is inside −40…85 °C and 0…100 % RH. */
bool ace2k_env_chamber_plausible(const struct ace2k_env *self);

/* True for a chamber reading that is valid and no older than ACE2K_ENV_CHAMBER_STALE_MS. */
static inline bool ace2k_env_chamber_fresh(bool valid, uint32_t age_ms)
{
    return (bool)(valid && age_ms <= ACE2K_ENV_CHAMBER_STALE_MS);
}

/* True when the reading behind bit is valid. bit is one of enum ace2k_env_valid, passed as a
 * uint8_t, because an enum constant is a signed int in C, and the lint refuses one under &. */
bool ace2k_env_valid_bit(const struct ace2k_env *self, uint8_t bit);

#endif

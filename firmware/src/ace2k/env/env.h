/* What: the environment readings — the two outlet NTCs in millidegrees, the chamber sensor's
 * temperature and humidity, VDDA from the internal reference — with a valid bit for each.
 * How: the binding owns one struct ace2k_env; ace2k_env_tick() every 10 ms with the NTC
 * millivolts and the raw reference (interrupt context: a table lookup and a filter);
 * ace2k_env_task() from a task (it talks I²C through ops).  The chamber sensor is an AHT20:
 * status, `BE 08 00` if uncalibrated, `AC 33 00`, ≥ 80 ms, 7 bytes with a CRC-8 (Aosong
 * datasheet), one cycle per second.  No floats: the NTC curve is env_table.h.
 * Depends on: <stdbool.h>, <stddef.h>, <stdint.h>, env_table.h, util.h. */
#ifndef ACE2K_ENV_H
#define ACE2K_ENV_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
#define ACE2K_ENV_AHT_STALE_MS      2000U
#define ACE2K_ENV_AHT_FRAME_LEN     7U
#define ACE2K_ENV_AHT_STATUS_BUSY   0x80U
#define ACE2K_ENV_AHT_STATUS_CAL    0x08U
#define ACE2K_ENV_CHAMBER_MIN_MC    (-40000)
#define ACE2K_ENV_CHAMBER_MAX_MC    85000
#define ACE2K_ENV_RH_MAX_PCT10      1000U

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

/* 0 on success, anything else on a bus failure (a NACK, a timeout). */
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
    int32_t chamber_mc;
    uint16_t chamber_rh_pct10;
    uint32_t chamber_at_ms;
    bool chamber_ever;
    uint16_t vdda_mv;
    uint8_t valid; /* enum ace2k_env_valid bits */
    enum ace2k_env_aht_phase aht_phase;
    uint32_t aht_next_ms;
    uint8_t aht_busy_retries;
    uint32_t aht_failures;
};

/* Millidegrees for mv inside the window; *valid false (and 0 returned) outside it. */
int32_t ace2k_env_ntc_mc(uint16_t mv, bool *valid);

/* CRC-8, polynomial 0x31, initial 0xFF, no reflection — the AHT20's. */
uint8_t ace2k_env_crc8(const uint8_t *data, size_t len);

/* Decodes a 7-byte frame: false when busy or when the CRC fails. */
bool ace2k_env_aht_decode(const uint8_t frame[ACE2K_ENV_AHT_FRAME_LEN], int32_t *mc,
                          uint16_t *rh_pct10);

void ace2k_env_init(struct ace2k_env *self, const struct ace2k_env_ops *ops, void *ctx,
                    uint32_t now_ms);

/* Interrupt context.  vrefint_raw is the 12-bit reading of the internal reference. */
void ace2k_env_tick(struct ace2k_env *self, uint32_t now_ms, uint16_t ntc_left_mv,
                    uint16_t ntc_right_mv, uint16_t vrefint_raw);

/* True once the chamber state machine's next deadline has passed: the binding's tick wakes
 * the task on it rather than every 10 ms.  One aligned read of a deadline the task writes,
 * so it is safe from any context. */
bool ace2k_env_task_due(const struct ace2k_env *self, uint32_t now_ms);

/* Task context: the chamber sensor's state machine.  Runs one step when ace2k_env_task_due();
 * calling it early does nothing. */
void ace2k_env_task(struct ace2k_env *self, uint32_t now_ms);

/* The last chamber reading is inside −40…85 °C and 0…100 % RH. */
bool ace2k_env_chamber_plausible(const struct ace2k_env *self);

#endif

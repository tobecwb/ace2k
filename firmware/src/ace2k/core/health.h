/* What: the self-test — every check a read-only image can make, as two fault masks: `now`
 * (a set bit = the check fails right now) and `latched` (has failed since boot or since the
 * last clear).  Bits, in the order of docs/protocol.md "Health bits".
 * How: the binding gathers a struct ace2k_health_inputs from the other modules (and the board's
 * system facts), ace2k_health_evaluate() turns it into a mask, ace2k_health_apply() folds it in
 * — `full` at boot and on request (every bit), else periodic (the boot-only bits kept).
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_HEALTH_H
#define ACE2K_HEALTH_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

enum ace2k_health_bit {
    ACE2K_HEALTH_NTC_LEFT = 0,
    ACE2K_HEALTH_NTC_RIGHT = 1,
    ACE2K_HEALTH_CHAMBER = 2,
    ACE2K_HEALTH_CHAMBER_PLAUSIBLE = 3,
    ACE2K_HEALTH_READER_A = 4,
    ACE2K_HEALTH_READER_B = 5,
    ACE2K_HEALTH_ZEROCROSS = 6,
    ACE2K_HEALTH_MAINS_HZ = 7,
    ACE2K_HEALTH_CUTOUT = 8,
    ACE2K_HEALTH_INSERT1 = 9,  /* lanes 1–4 contiguous */
    ACE2K_HEALTH_BUFFER1 = 13, /* lanes 1–4 contiguous */
    ACE2K_HEALTH_ENCODER1 = 17,
    ACE2K_HEALTH_FG1 = 21,
    ACE2K_HEALTH_FRESH = 25,
    ACE2K_HEALTH_VDDA = 26,
    ACE2K_HEALTH_CLOCK = 27,
    ACE2K_HEALTH_WATCHDOG = 28,
    ACE2K_HEALTH_CONFIG_PAGE = 29,
    ACE2K_HEALTH_IMAGE_CRC = 30,
    ACE2K_HEALTH_BIT_COUNT = 31,
};

/* The shift count is cast: an enum constant is a signed int, and the lint refuses one as a
 * shift count. */
#define ACE2K_HEALTH_BIT(b) (1UL << (unsigned)(b))
#define ACE2K_HEALTH_BOOT_ONLY_MASK                                                                \
    ((0xFUL << (unsigned)ACE2K_HEALTH_ENCODER1) | (0xFUL << (unsigned)ACE2K_HEALTH_FG1) |          \
     ACE2K_HEALTH_BIT(ACE2K_HEALTH_CLOCK) | ACE2K_HEALTH_BIT(ACE2K_HEALTH_WATCHDOG) |              \
     ACE2K_HEALTH_BIT(ACE2K_HEALTH_CONFIG_PAGE) | ACE2K_HEALTH_BIT(ACE2K_HEALTH_IMAGE_CRC))

#define ACE2K_HEALTH_BOOT_WINDOW_MS       2000U
#define ACE2K_HEALTH_PERIOD_MS            1000U
#define ACE2K_HEALTH_ENCODER_QUIET_COUNTS 2

enum ace2k_reset_cause {
    ACE2K_RESET_UNKNOWN = 0,
    ACE2K_RESET_POWER_ON = 1,
    ACE2K_RESET_PIN = 2,
    ACE2K_RESET_SOFTWARE = 3,
    ACE2K_RESET_WATCHDOG = 4,
    ACE2K_RESET_WINDOW_WATCHDOG = 5,
    ACE2K_RESET_LOW_POWER = 6,
};

struct ace2k_health_inputs {
    bool ntc_left_valid, ntc_right_valid;
    bool chamber_valid, chamber_plausible;
    bool reader_ok[2];
    bool zerocross_present, mains_plausible;
    bool cutout_idle;
    bool insert_connected[ACE2K_LANE_COUNT];
    bool buffer_consistent[ACE2K_LANE_COUNT];
    bool buffer_check_enabled; /* the binding's ACE2K_HEALTH_BUFFER_CHECK_ENABLED: 1 since the
                                * bench of 2026-09-17 showed rest and pushed never overlap */
    int32_t encoder_delta[ACE2K_LANE_COUNT]; /* over the boot window */
    uint32_t fg_delta[ACE2K_LANE_COUNT];
    bool sensors_fresh;
    bool vdda_ok;
    bool clock_ok, watchdog_ok, config_page_ok, image_crc_ok;
};

struct ace2k_health {
    uint32_t now;
    uint32_t latched;
    uint32_t post_ms; /* when the last full evaluation ran */
    bool post_done;
    uint8_t reset_cause;
};

/* Pure: the fault mask of these inputs. */
uint32_t ace2k_health_evaluate(const struct ace2k_health_inputs *in);

void ace2k_health_init(struct ace2k_health *self, uint8_t reset_cause);

/* full: every bit replaced and post_ms = now_ms; periodic: the boot-only bits kept.  latched
 * accumulates either way. */
void ace2k_health_apply(struct ace2k_health *self, uint32_t mask, uint32_t now_ms, bool full);

/* latched := now. */
void ace2k_health_clear(struct ace2k_health *self);

#endif

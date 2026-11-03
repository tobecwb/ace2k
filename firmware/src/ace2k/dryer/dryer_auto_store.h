/* What: the part of the automatic modes that the config page keeps: the mode, the limit, the
 * failure count and its ceiling, the profile, and the stop.
 * How: types and defaults only. dryer_auto.h declares the functions that change it, and
 * core/config.c encodes it in a marked block of the page.
 * Depends on: <stdint.h>. */
#ifndef ACE2K_DRYER_AUTO_STORE_H
#define ACE2K_DRYER_AUTO_STORE_H
#include <stdint.h>

/* The values are the wire's (the ace2k_dryer_auto_mode command) and the page's. The store keeps
 * the mode in a uint8_t, so an image reads and writes the same byte. */
enum ace2k_dryer_auto_mode {
    ACE2K_DRYER_AUTO_MANUAL = 0,
    ACE2K_DRYER_AUTO_HUMIDITY = 1,
    ACE2K_DRYER_AUTO_PRINT = 2,
    ACE2K_DRYER_AUTO_MODES = 3, /* the count; not a mode */
};

#define ACE2K_DRYER_AUTO_STOPPED 0x1U /* the humidity mode stopped on its failures */

#define ACE2K_DRYER_AUTO_LIMIT_DEFAULT_PCT   40U
#define ACE2K_DRYER_AUTO_FAILURES_DEFAULT    4U
#define ACE2K_DRYER_AUTO_PROFILE_C_DEFAULT   45U  /* PLA, the safe default */
#define ACE2K_DRYER_AUTO_PROFILE_MIN_DEFAULT 360U /* 6 h */

struct ace2k_dryer_auto_store { /* 8 bytes on the page */
    uint8_t mode;               /* an enum ace2k_dryer_auto_mode */
    uint8_t limit_pct;          /* the RH limit, whole % */
    uint8_t max_failures;       /* failed cycles in a row that stop the humidity mode */
    uint8_t failures;           /* failed cycles in a row so far */
    uint8_t profile_c;          /* the temperature of an automatic cycle */
    uint8_t flags;              /* ACE2K_DRYER_AUTO_STOPPED */
    uint16_t profile_min;       /* the time of an automatic cycle */
};

static inline void ace2k_dryer_auto_store_defaults(struct ace2k_dryer_auto_store *s)
{
    s->mode = ACE2K_DRYER_AUTO_MANUAL;
    s->limit_pct = ACE2K_DRYER_AUTO_LIMIT_DEFAULT_PCT;
    s->max_failures = ACE2K_DRYER_AUTO_FAILURES_DEFAULT;
    s->failures = 0U;
    s->profile_c = ACE2K_DRYER_AUTO_PROFILE_C_DEFAULT;
    s->flags = 0U;
    s->profile_min = ACE2K_DRYER_AUTO_PROFILE_MIN_DEFAULT;
}

#endif

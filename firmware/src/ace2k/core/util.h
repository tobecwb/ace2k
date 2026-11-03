/* Shared helpers of the core.
 * What: the error codes, the lane count, the compiler barrier, the units in a second and in a
 * whole, wrap-safe time comparison, the settle after a stall's catch-up, the magnitude of an
 * int32_t, clamping, array size, CRC-16/CCITT, CRC-16/MCRF4XX, CRC-32, the little-endian integer
 * helpers, the wipe of a secret and bounded string helpers.
 * How: include it. Everything is `static inline` with no state of its own (the settle's state is
 * the caller's), so there is no util.c.
 * Depends on the C standard headers only. */
#ifndef ACE2K_UTIL_H
#define ACE2K_UTIL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ACE2K_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* The unit has four filament lanes; every per-lane array is sized by this. */
#define ACE2K_LANE_COUNT 4

/* A compiler-only barrier. The core runs on one in-order core, where an interrupt is atomic to the
 * task. So what must be prevented is the compiler moving the payload accesses of a ring across its
 * index store. A host test may define it first, to run an interrupt at each publication point of a
 * module it compiles into itself (tests/heat/test_heat_preempt.c). */
#ifndef ACE2K_COMPILER_BARRIER
#define ACE2K_COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")
#endif

/* Milliseconds in a second. A rate in units per second, times a period in ms, over this. */
#define ACE2K_MS_PER_S 1000U

/* The whole of a quantity in percent. A share in percent, times the whole, over this. */
#define ACE2K_PCT_FULL 100U

/* True once now has reached deadline, both on one wrapping clock. The firmware passes
 * milliseconds everywhere except the mains, which compares the timer ticks of its span clock. It
 * is correct across the clock's wrap (49 days for milliseconds), as long as the two are less
 * than 2^31 units apart. Every deadline in the firmware is. */
static inline bool ace2k_time_after(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

/* Milliseconds elapsed from from_ms to to_ms, wrap-safe. */
static inline uint32_t ace2k_time_since(uint32_t to_ms, uint32_t from_ms)
{
    return to_ms - from_ms;
}

/* A periodic tick that comes more than two periods after the last one is a catch-up after a
 * stall.  A flash erase or program holds the core, and every timer with it, for tens of ms.  The
 * ADC passes that the stall swallowed leave the age of the readings inflated for a moment, although
 * the scan never stopped: the first pass after the stall ends up to two periods after it.  So a
 * reader of that age waits ACE2K_CATCHUP_SETTLE_MS after the catch-up before it judges the age.
 * The health passes and the insert sensors share this one rule.  Initial value; re-tune on the
 * bench. */
#define ACE2K_CATCHUP_SETTLE_MS 50U

struct ace2k_catchup {
    uint32_t last_tick_ms; /* the last tick ace2k_catchup_settled() saw */
    uint32_t until_ms;     /* the age is not judged before this */
};

/* now_ms is the clock at init, so the first tick after it is not late. */
static inline void ace2k_catchup_init(struct ace2k_catchup *self, uint32_t now_ms)
{
    self->last_tick_ms = now_ms;
    self->until_ms = now_ms;
}

/* Every tick of period_ms.  Returns true when the age of the readings may be judged: no catch-up
 * in the last ACE2K_CATCHUP_SETTLE_MS. */
static inline bool ace2k_catchup_settled(struct ace2k_catchup *self, uint32_t now_ms,
                                         uint32_t period_ms)
{
    if (ace2k_time_since(now_ms, self->last_tick_ms) > 2U * period_ms) {
        self->until_ms = now_ms + ACE2K_CATCHUP_SETTLE_MS;
    }
    self->last_tick_ms = now_ms;
    return ace2k_time_after(now_ms, self->until_ms);
}

/* True when low <= value <= high: the bounds check of a setter. */
static inline bool ace2k_within_u32(uint32_t value, uint32_t low, uint32_t high)
{
    return (bool)(value >= low && value <= high);
}

/* The magnitude of v. It is a uint32_t, so INT32_MIN has one (2^31 fits no int32_t). */
static inline uint32_t ace2k_abs_i32(int32_t v)
{
    if (v < 0) {
        return 0U - (uint32_t)v;
    }
    return (uint32_t)v;
}

static inline int32_t ace2k_clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

/* Error codes. A function that can fail returns 0 on success and -ACE2K_E<NAME> otherwise. No
 * caller ignores one silently. */
enum ace2k_err {
    ACE2K_EOK = 0,
    ACE2K_EINVAL = 1,   /* an argument outside its bounds */
    ACE2K_EREFUSED = 2, /* a guard said no */
    ACE2K_EFLASH = 3,   /* the flash controller failed, or the readback differs */
    ACE2K_EFULL = 4,    /* a fixed-size registry has no free slot */
    ACE2K_EIO = 5,      /* a device on a bus did not answer as it must */
    ACE2K_EBUSY = 6,    /* another owner holds the resource, or its state forbids the change */
    ACE2K_ELATCHED = 7, /* a latched fault must be cleared first */
    ACE2K_EAGAIN = 8,   /* a transient condition said no: ask again */
};

#define ACE2K_CRC16_INIT 0xFFFFU
#define ACE2K_CRC16_POLY 0x1021U
#define ACE2K_CRC16_MSB  0x8000U

/* CRC-16/CCITT-FALSE over len bytes: polynomial 0x1021, initial 0xFFFF, no reflection, no final
 * xor. "123456789" gives 0x29B1. It is the checksum of the config record. */
static inline uint16_t ace2k_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = ACE2K_CRC16_INIT;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint32_t)data[i] << 8U);
        for (int n = 0; n < 8; n++) {
            /* Shift in 32 bits, because a uint16_t promotes to int, and the lint refuses a
             * bitwise operator on a signed operand. */
            uint32_t shifted = (uint32_t)crc << 1U;
            crc = (crc & ACE2K_CRC16_MSB) ? (uint16_t)(shifted ^ ACE2K_CRC16_POLY)
                                          : (uint16_t)shifted;
        }
    }
    return crc;
}

#define ACE2K_CRC16_MCRF4XX_INIT 0xFFFFU
#define ACE2K_CRC16_MCRF4XX_POLY 0x8408U /* 0x1021 reflected */

/* CRC-16/MCRF4XX, continued from crc: reflected polynomial 0x8408, initial 0xFFFF, no final xor.
 * "123456789" from 0xFFFF gives 0x6F91. It is the checksum of the factory calibration pages. */
static inline uint16_t ace2k_crc16_mcrf4xx_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int n = 0; n < 8; n++) {
            uint32_t shifted = (uint32_t)crc >> 1U;
            crc = (crc & 1U) ? (uint16_t)(shifted ^ ACE2K_CRC16_MCRF4XX_POLY) : (uint16_t)shifted;
        }
    }
    return crc;
}

static inline uint16_t ace2k_crc16_mcrf4xx(const uint8_t *data, size_t len)
{
    return ace2k_crc16_mcrf4xx_update(ACE2K_CRC16_MCRF4XX_INIT, data, len);
}

#define ACE2K_CRC32_INIT 0xFFFFFFFFU
#define ACE2K_CRC32_POLY 0xEDB88320U /* IEEE 802.3 reflected */

/* CRC-32 (IEEE, as zlib.crc32 computes it), continued from a running value. The value starts at
 * ACE2K_CRC32_INIT and is finished with ace2k_crc32_final(). "123456789" gives 0xCBF43926.
 * Bitwise, with no table, because the image check runs once at boot. */
static inline uint32_t ace2k_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int n = 0; n < 8; n++) {
            crc = (crc & 1U) ? (crc >> 1U) ^ ACE2K_CRC32_POLY : crc >> 1U;
        }
    }
    return crc;
}

static inline uint32_t ace2k_crc32_final(uint32_t crc)
{
    return crc ^ ACE2K_CRC32_INIT;
}

static inline uint32_t ace2k_crc32(const uint8_t *data, size_t len)
{
    return ace2k_crc32_final(ace2k_crc32_update(ACE2K_CRC32_INIT, data, len));
}

/* A uint16 in a byte buffer, low byte first. That is the layout of every record and report of the
 * firmware. The helpers widen to uint32_t first, because a uint8_t or uint16_t operand promotes to
 * int, and the lint refuses a bitwise operator on a signed operand. */
static inline uint16_t ace2k_get_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8U));
}

static inline void ace2k_put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)((uint32_t)v >> 8U);
}

#define ACE2K_BITS_PER_BYTE 8U

/* A uint32 in a byte buffer, low byte first: the record's magic and flags, the stored image
 * CRC, the report arrays. */
static inline uint32_t ace2k_get_u32_le(const uint8_t *p)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < sizeof v; i++) {
        v |= (uint32_t)p[i] << (i * ACE2K_BITS_PER_BYTE);
    }
    return v;
}

static inline void ace2k_put_u32_le(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < sizeof v; i++) {
        p[i] = (uint8_t)(v >> (i * ACE2K_BITS_PER_BYTE));
    }
}

/* Two's complement, as struct.unpack("<i") on the host reads it back. */
static inline void ace2k_put_i32_le(uint8_t *p, int32_t v)
{
    ace2k_put_u32_le(p, (uint32_t)v);
}

/* Zeroes n bytes at p through a volatile pointer, so the compiler keeps every store. A plain
 * zeroing of a local about to die, or of a struct never read again, is a dead store that
 * `-O2 -flto` removes. Every copy of a MIFARE key is wiped with it. */
static inline void ace2k_wipe(void *p, size_t n)
{
    volatile uint8_t *b = (volatile uint8_t *)p;
    for (size_t i = 0; i < n; i++) {
        b[i] = 0;
    }
}

/* True when two NUL-terminated strings are equal. The core includes no <string.h>. */
static inline bool ace2k_str_equal(const char *a, const char *b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

/* Copies src into dst[size], always NUL-terminating. A longer src is truncated. Returns the count
 * copied, without the NUL. size must be at least 1. */
static inline size_t ace2k_str_copy(char *dst, size_t size, const char *src)
{
    size_t n = 0;
    while (n + 1 < size && src[n]) {
        dst[n] = src[n];
        n++;
    }
    dst[n] = '\0';
    return n;
}

#endif

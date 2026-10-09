#include "env/env.h"
#include "env/env_table.h"
#include "core/util.h"

#define AHT_CMD_LEN        3U
#define AHT_RAW_BITS       20U
#define AHT_T_SCALE_MC     200000U /* raw / 2^20 × 200 °C − 50 °C, in millidegrees */
#define AHT_T_OFFSET_MC    50000
#define AHT_RH_SCALE_PCT10 1000U
#define CRC8_POLY          0x31U
#define CRC8_INIT          0xFFU
#define CRC8_MSB           0x80U
#define BYTE_MASK          0xFFU

/* The 7-byte answer: status, RH[19:12], RH[11:4], RH[3:0] | T[19:16], T[15:8], T[7:0], CRC
 * (Aosong AHT20 datasheet). */
#define AHT_FRAME_STATUS     0U
#define AHT_FRAME_RH_HI      1U
#define AHT_FRAME_RH_MID     2U
#define AHT_FRAME_RH_LO_T_HI 3U
#define AHT_FRAME_T_MID      4U
#define AHT_FRAME_T_LO       5U
#define AHT_FRAME_CRC        6U
#define AHT_RH_HI_SHIFT      12U
#define AHT_RH_MID_SHIFT     4U
#define AHT_RH_LO_SHIFT      4U /* the byte's upper nibble */
#define AHT_T_HI_SHIFT       16U
#define AHT_T_HI_MASK        0x0FU /* the byte's lower nibble */
#define AHT_T_MID_SHIFT      8U

static const uint8_t ace2k_aht_cmd_init[AHT_CMD_LEN] = { 0xBE, 0x08, 0x00 };
static const uint8_t ace2k_aht_cmd_trigger[AHT_CMD_LEN] = { 0xAC, 0x33, 0x00 };

int32_t ace2k_env_ntc_mc(uint16_t mv, bool *valid)
{
    if (mv < ACE2K_ENV_NTC_MIN_MV || mv > ACE2K_ENV_NTC_MAX_MV) {
        *valid = false;
        return 0;
    }
    *valid = true;
    uint32_t off = (uint32_t)mv - ACE2K_ENV_NTC_TABLE_MIN_MV;
    uint32_t idx = off / ACE2K_ENV_NTC_TABLE_STEP_MV;
    uint32_t frac = off % ACE2K_ENV_NTC_TABLE_STEP_MV;
    if (idx >= ACE2K_ENV_NTC_TABLE_LEN - 1U) {
        return ace2k_env_ntc_table_mc[ACE2K_ENV_NTC_TABLE_LEN - 1U];
    }
    int32_t a = ace2k_env_ntc_table_mc[idx];
    int32_t b = ace2k_env_ntc_table_mc[idx + 1U];
    return a + (((b - a) * (int32_t)frac) / (int32_t)ACE2K_ENV_NTC_TABLE_STEP_MV);
}

uint8_t ace2k_env_crc8(const uint8_t *data, size_t len)
{
    uint32_t crc = CRC8_INIT;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc =
                (crc & CRC8_MSB) ? ((crc << 1U) ^ CRC8_POLY) & BYTE_MASK : (crc << 1U) & BYTE_MASK;
        }
    }
    return (uint8_t)crc;
}

bool ace2k_env_aht_decode(const uint8_t frame[ACE2K_ENV_AHT_FRAME_LEN], int32_t *mc,
                          uint16_t *rh_pct10)
{
    if (frame[AHT_FRAME_STATUS] & ACE2K_ENV_AHT_STATUS_BUSY) {
        return false;
    }
    if (ace2k_env_crc8(frame, ACE2K_ENV_AHT_FRAME_LEN - 1U) != frame[AHT_FRAME_CRC]) {
        return false;
    }
    uint32_t rh_raw = ((uint32_t)frame[AHT_FRAME_RH_HI] << AHT_RH_HI_SHIFT) |
                      ((uint32_t)frame[AHT_FRAME_RH_MID] << AHT_RH_MID_SHIFT) |
                      ((uint32_t)frame[AHT_FRAME_RH_LO_T_HI] >> AHT_RH_LO_SHIFT);
    uint32_t t_raw = (((uint32_t)frame[AHT_FRAME_RH_LO_T_HI] & AHT_T_HI_MASK) << AHT_T_HI_SHIFT) |
                     ((uint32_t)frame[AHT_FRAME_T_MID] << AHT_T_MID_SHIFT) |
                     (uint32_t)frame[AHT_FRAME_T_LO];
    *rh_pct10 = (uint16_t)((rh_raw * AHT_RH_SCALE_PCT10) >> AHT_RAW_BITS);
    *mc = (int32_t)(((uint64_t)t_raw * AHT_T_SCALE_MC) >> AHT_RAW_BITS) - AHT_T_OFFSET_MC;
    return true;
}

void ace2k_env_init(struct ace2k_env *self, const struct ace2k_env_ops *ops, void *ctx,
                    uint32_t now_ms)
{
    *self = (struct ace2k_env){ 0 };
    self->ops = ops;
    self->ctx = ctx;
    self->aht_phase = ACE2K_ENV_AHT_IDLE;
    self->aht_next_ms = now_ms;
}

/* An enum constant is a signed int in C, and the lint refuses one under a bitwise operator;
 * the bit arrives here as a uint8_t. */
static void set_valid(struct ace2k_env *self, uint8_t bit, bool on)
{
    if (on) {
        self->valid |= bit;
    } else {
        self->valid &= (uint8_t)~bit;
    }
}

static void tick_ntc(struct ace2k_env *self, int side, uint16_t mv, int32_t *mc, uint8_t bit)
{
    int32_t mv16 = (int32_t)((uint32_t)mv << ACE2K_ENV_FILT_SHIFT);
    if (!self->ntc_primed) {
        self->ntc_filt16[side] = mv16;
    } else {
        self->ntc_filt16[side] += (mv16 - self->ntc_filt16[side]) / ACE2K_ENV_NTC_FILTER_DIV;
    }
    uint16_t filt_mv = (uint16_t)((uint32_t)self->ntc_filt16[side] >> ACE2K_ENV_FILT_SHIFT);
    self->ntc_mv[side] = filt_mv;
    bool valid = false;
    *mc = ace2k_env_ntc_mc(filt_mv, &valid);
    set_valid(self, bit, valid);
}

void ace2k_env_tick(struct ace2k_env *self, uint32_t now_ms, uint16_t ntc_left_mv,
                    uint16_t ntc_right_mv, uint16_t vrefint_raw)
{
    tick_ntc(self, 0, ntc_left_mv, &self->ptc_left_mc, ACE2K_ENV_VALID_NTC_LEFT);
    tick_ntc(self, 1, ntc_right_mv, &self->ptc_right_mc, ACE2K_ENV_VALID_NTC_RIGHT);
    self->ntc_primed = true;
    if (vrefint_raw == 0) {
        self->vdda_mv = 0;
    } else {
        /* A reference reading under 75 counts puts the quotient past a uint16_t: saturate,
         * so the window test below sees the ceiling and not a wrapped value. */
        uint32_t vdda_mv =
            ((uint32_t)ACE2K_ENV_VREFINT_MV * ACE2K_ENV_ADC_FULL_SCALE) / vrefint_raw;
        self->vdda_mv = vdda_mv > UINT16_MAX ? UINT16_MAX : (uint16_t)vdda_mv;
    }
    if (self->vdda_mv >= ACE2K_ENV_VDDA_MIN_MV && self->vdda_mv <= ACE2K_ENV_VDDA_MAX_MV) {
        set_valid(self, ACE2K_ENV_VALID_VDDA, true);
    } else {
        set_valid(self, ACE2K_ENV_VALID_VDDA, false);
    }
    if (self->chamber_ever &&
        ace2k_time_since(now_ms, self->chamber_at_ms) <= ACE2K_ENV_AHT_STALE_MS) {
        set_valid(self, ACE2K_ENV_VALID_CHAMBER, true);
    } else {
        set_valid(self, ACE2K_ENV_VALID_CHAMBER, false);
    }
}

static void aht_fail(struct ace2k_env *self, uint32_t now_ms)
{
    self->aht_failures++;
    self->aht_phase = ACE2K_ENV_AHT_IDLE;
    self->aht_next_ms = now_ms + ACE2K_ENV_AHT_PERIOD_MS;
}

static int aht_write(struct ace2k_env *self, const uint8_t cmd[AHT_CMD_LEN])
{
    return self->ops->i2c_write(self->ctx, ACE2K_ENV_AHT_ADDR, cmd, AHT_CMD_LEN);
}

static void aht_trigger(struct ace2k_env *self, uint32_t now_ms)
{
    if (aht_write(self, ace2k_aht_cmd_trigger) != 0) {
        aht_fail(self, now_ms);
        return;
    }
    self->aht_phase = ACE2K_ENV_AHT_TRIGGERED;
    self->aht_next_ms = now_ms + ACE2K_ENV_AHT_MEASURE_MS;
    self->aht_busy_retries = 0;
}

static void aht_idle(struct ace2k_env *self, uint32_t now_ms)
{
    uint8_t status = 0;
    if (self->ops->i2c_read(self->ctx, ACE2K_ENV_AHT_ADDR, &status, 1U) != 0) {
        aht_fail(self, now_ms);
        return;
    }
    if (!(status & ACE2K_ENV_AHT_STATUS_CAL)) {
        if (aht_write(self, ace2k_aht_cmd_init) != 0) {
            aht_fail(self, now_ms);
            return;
        }
        self->aht_phase = ACE2K_ENV_AHT_INIT_SENT;
        self->aht_next_ms = now_ms + ACE2K_ENV_AHT_INIT_MS;
        return;
    }
    aht_trigger(self, now_ms);
}

static void aht_triggered(struct ace2k_env *self, uint32_t now_ms)
{
    uint8_t frame[ACE2K_ENV_AHT_FRAME_LEN];
    if (self->ops->i2c_read(self->ctx, ACE2K_ENV_AHT_ADDR, frame, ACE2K_ENV_AHT_FRAME_LEN) != 0) {
        aht_fail(self, now_ms);
        return;
    }
    if (frame[AHT_FRAME_STATUS] & ACE2K_ENV_AHT_STATUS_BUSY) {
        if (++self->aht_busy_retries > ACE2K_ENV_AHT_BUSY_RETRIES) {
            aht_fail(self, now_ms);
            return;
        }
        self->aht_next_ms = now_ms + ACE2K_ENV_AHT_BUSY_RETRY_MS;
        return;
    }
    int32_t mc = 0;
    uint16_t rh = 0;
    if (!ace2k_env_aht_decode(frame, &mc, &rh)) {
        aht_fail(self, now_ms);
        return;
    }
    self->chamber_mc = mc;
    self->chamber_rh_pct10 = rh;
    self->chamber_at_ms = now_ms;
    self->chamber_ever = true;
    self->aht_phase = ACE2K_ENV_AHT_IDLE;
    self->aht_next_ms = now_ms + ACE2K_ENV_AHT_PERIOD_MS - ACE2K_ENV_AHT_MEASURE_MS;
}

bool ace2k_env_task_due(const struct ace2k_env *self, uint32_t now_ms)
{
    return ace2k_time_after(now_ms, self->aht_next_ms);
}

void ace2k_env_task(struct ace2k_env *self, uint32_t now_ms)
{
    if (!ace2k_env_task_due(self, now_ms)) {
        return;
    }
    switch (self->aht_phase) {
    case ACE2K_ENV_AHT_IDLE:
        aht_idle(self, now_ms);
        break;
    case ACE2K_ENV_AHT_INIT_SENT:
        aht_trigger(self, now_ms);
        break;
    case ACE2K_ENV_AHT_TRIGGERED:
        aht_triggered(self, now_ms);
        break;
    default:
        self->aht_phase = ACE2K_ENV_AHT_IDLE;
        break;
    }
}

bool ace2k_env_chamber_plausible(const struct ace2k_env *self)
{
    if (!self->chamber_ever) {
        return false;
    }
    if (self->chamber_mc < ACE2K_ENV_CHAMBER_MIN_MC ||
        self->chamber_mc > ACE2K_ENV_CHAMBER_MAX_MC) {
        return false;
    }
    return self->chamber_rh_pct10 <= ACE2K_ENV_RH_MAX_PCT10;
}

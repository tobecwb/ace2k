/* What: the dryer's air path — the two fans, switched together, and the two exhaust flaps —
 * as bounded outputs with an owner, and the thermal fan rule that no owner overrides.
 * How: the binding owns one struct ace2k_airflow over ops that write both fans, read the two fan
 * pins back and write a flap's two coils; ace2k_airflow_tick() every 10 ms, before heat and
 * dryer, applies rule 7 of the dryer (a fan never off while an outlet NTC reads above
 * 45 °C or is invalid; the rule releases them once both read below 42 °C), ends a manual run,
 * and ends and starts flap pulses.  The fans are commanded on while the owner asks or the rule
 * demands.  A flap moves by a pulse of ACE2K_FLAP_PULSE_MS on one coil (never above
 * ACE2K_FLAP_PULSE_MAX_MS), then both coils are released (the actuator is bistable and holds
 * its position unpowered, docs/hardware.md "Exhaust flaps"); one flap at a time — a pulse asked
 * while either flap's pulse runs waits for it; there is no position cache — every request
 * pulses.  The published position (ace2k_airflow_flap_pos) is where the last pulse that ran to
 * its end left the flap: unknown at boot, while a pulse on it runs or waits, and after a pulse a
 * shutdown cut short.
 * No tachometer exists: the fan read-back is the pin, never the rotation.  heat's binding sets
 * heat_holds every tick from ace2k_heat_active() — a lease, an odd half pending, a pulse not yet
 * seen over, or the gate reading high in any state (a gate stuck high is heat on, latched or
 * not).  While it holds, the fans stay commanded on whatever rule 7 or the owner decides
 * (rule 2): an explicit fan-off returns -ACE2K_EBUSY (a manual ACE_FAN off included), a MANUAL
 * owner does not expire, and an owner's release drops only the ownership.  Interrupt
 * context for the tick; the owner calls come from a task bracketed with irq_save by the binding.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_AIRFLOW_H
#define ACE2K_AIRFLOW_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

#define ACE2K_AIRFLOW_FAN_ON_MC        45000 /* rule 7: a fan never off above this */
#define ACE2K_AIRFLOW_FAN_OFF_MC       42000 /* hysteresis: the rule releases below this */
#define ACE2K_AIRFLOW_FAN_MANUAL_MAX_S 600U
/* Measured on the unit: 150 ms moved both flaps both ways; +50 ms margin; 20 of 20 moves per
 * flap at 200 ms. */
#define ACE2K_FLAP_PULSE_MS     200U
#define ACE2K_FLAP_PULSE_MAX_MS 400U

/* In the order of precedence: a flap request is never replaced by a weaker owner's.  The values
 * are the report's owner field (docs/protocol.md); 2 is not used.  No owner outranks the dryer
 * today: its callers keep their refusal handling as defence (dryer.c flaps_ask). */
enum ace2k_airflow_owner {
    ACE2K_AIRFLOW_OWNER_NONE = 0,
    ACE2K_AIRFLOW_OWNER_MANUAL = 1,
    ACE2K_AIRFLOW_OWNER_DRYER = 3,
};
enum ace2k_flap { ACE2K_FLAP_BOTTOM = 0, ACE2K_FLAP_REAR = 1, ACE2K_FLAP_COUNT = 2 };
enum ace2k_flap_pos { ACE2K_FLAP_UNKNOWN = 0, ACE2K_FLAP_OPEN = 1, ACE2K_FLAP_CLOSED = 2 };

/* A flap request: its direction and who asked it. */
struct ace2k_flap_request {
    bool set;
    bool open;
    enum ace2k_airflow_owner who;
};

struct ace2k_airflow_ops {
    void (*fan_write)(void *ctx, bool on); /* both fans */
    uint8_t (*fan_read)(void *ctx);        /* bit 0 left, bit 1 right: pin reads high */
    void (*flap_write)(void *ctx, enum ace2k_flap flap, bool open_coil, bool close_coil);
};

struct ace2k_airflow {
    const struct ace2k_airflow_ops *ops;
    void *ctx;
    enum ace2k_airflow_owner owner;
    bool owner_on;   /* what the owner asked */
    bool thermal_on; /* rule 7's demand */
    bool commanded;  /* what was last written: owner_on || thermal_on */
    uint32_t manual_until_ms;
    struct ace2k_flap_request flap_run[ACE2K_FLAP_COUNT];  /* the pulse on the coil now */
    struct ace2k_flap_request flap_wait[ACE2K_FLAP_COUNT]; /* the one waiting: depth 1 */
    uint32_t flap_until_ms[ACE2K_FLAP_COUNT];
    /* where the last pulse that ran to its end left the flap; written only then (and unknown by
     * a shutdown's cut) */
    enum ace2k_flap_pos flap_done[ACE2K_FLAP_COUNT];
    bool heat_holds;    /* ace2k_heat_active(), set by heat's binding every tick (rule 2) */
    bool measured_warm; /* the last tick: a valid outlet NTC at or above ACE2K_AIRFLOW_FAN_OFF_MC */
};

/* Owner NONE, fans off, both coils of both flaps released (each written once), positions
 * unknown. */
void ace2k_airflow_init(struct ace2k_airflow *self, const struct ace2k_airflow_ops *ops, void *ctx);

/* 0, -ACE2K_EBUSY (another owner holds the fans, or heat_holds and on == false),
 * -ACE2K_EINVAL (seconds 0 or above the maximum for MANUAL).  The dryer takes the fans
 * over from the manual owner; on == false releases the ownership; seconds is ignored for the
 * dryer, whose ownership has no end of its own. */
int ace2k_airflow_fans(struct ace2k_airflow *self, enum ace2k_airflow_owner who, bool on,
                       uint32_t seconds, uint32_t now_ms);

/* The owner who gives the fans up; ignored for any other owner.  While heat_holds the fans stay
 * commanded on all the same (rule 2), with no owner. */
void ace2k_airflow_release(struct ace2k_airflow *self, enum ace2k_airflow_owner who);

/* 0, -ACE2K_EBUSY (another owner than who holds the outputs; NONE/MANUAL may pulse only while
 * the owner is NONE or MANUAL), -ACE2K_EINVAL (no such flap).  A NONE or MANUAL pulse while a
 * pulse runs or waits: -ACE2K_EBUSY — it is never queued, so 0 means it runs now.  The two
 * flaps never pulse together: a dryer's pulse asked while either flap's pulse runs is
 * queued on its flap, depth 1 per flap, the latest request wins — but never over a stronger
 * owner's: while a stronger owner's pulse runs or waits on that flap, -ACE2K_EBUSY (enum order
 * is precedence); when a pulse ends, the other flap's waiting request starts first (neither flap
 * starves the other), else this flap's own. */
int ace2k_airflow_flap_pulse(struct ace2k_airflow *self, enum ace2k_airflow_owner who,
                             enum ace2k_flap flap, bool open, uint32_t now_ms);

/* Every 10 ms, before heat and dryer: rule 7 (invalid = hot), the manual expiry, the pulses. */
void ace2k_airflow_tick(struct ace2k_airflow *self, int32_t ntc_left_mc, int32_t ntc_right_mc,
                        bool left_valid, bool right_valid, uint32_t now_ms);

/* The Klipper shutdown: every owner released, every pulse ended with both coils released and the
 * waiting requests dropped; a flap whose pulse was cut is published unknown (it may have stopped
 * mid-travel), and it stays where it is until the next boot's close.  heat_holds is left as it
 * is — heat's binding recomputes it every tick, and a gate stuck high keeps holding the fans
 * through the shutdown; rule 7 stays in force too — the fans remain on while either demands them,
 * and the next ticks keep applying both. */
void ace2k_airflow_shutdown(struct ace2k_airflow *self);

/* The air path must keep running — the bootloader, which drives no fans, must not be entered:
 * the dryer owns the fans (from its start until its cool-down or fault has closed the flaps),
 * heat holds them (heat_holds), or rule 7 holds them on a measured reading (a valid outlet NTC
 * still at or above 42 °C).  Rule 7 holding them only because both NTCs read invalid does not
 * count: a unit with a broken sensor must stay recoverable over the wire.  Bounded by the
 * cool-down: the dryer hands the fans back within 10 min of its end, rule 7 lets go below 42 °C. */
bool ace2k_airflow_fans_required(const struct ace2k_airflow *self);

bool ace2k_airflow_fans_commanded(const struct ace2k_airflow *self);
/* The flap's published position: ACE2K_FLAP_UNKNOWN while a pulse on it runs or waits, else
 * where the last pulse that ran to its end left it (unknown if none did, or a shutdown cut it). */
enum ace2k_flap_pos ace2k_airflow_flap_pos(const struct ace2k_airflow *self, enum ace2k_flap flap);
/* Both positions as reported: bits 0–1 bottom, bits 2–3 rear (enum ace2k_flap_pos). */
uint8_t ace2k_airflow_flaps_report(const struct ace2k_airflow *self);
/* A request of who's on the flap is running or waiting: who's own pulses are not all over. */
bool ace2k_airflow_flap_pending(const struct ace2k_airflow *self, enum ace2k_flap flap,
                                enum ace2k_airflow_owner who);
uint8_t ace2k_airflow_fans_read(const struct ace2k_airflow *self); /* ops->fan_read */

#endif

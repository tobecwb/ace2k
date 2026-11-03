/* What: the dryer's air path: the two fans and the two exhaust flaps, as outputs with an
 * owner.
 * How: one struct ace2k_airflow, ticked every 10 ms before heat and dryer. Owners ask for the
 * outputs.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_AIRFLOW_H
#define ACE2K_AIRFLOW_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

/* The contract.
 *
 * The two fans are switched together. The outputs are bounded and have an owner. No owner
 * overrides the thermal fan rule. The binding owns one struct ace2k_airflow, on top of ops that
 * write both fans, read the two fan pins back, and write the two coils of a flap.
 *
 * ace2k_airflow_tick() runs every 10 ms, before heat and dryer. It applies the fan rule of the
 * dryer: a fan is never off while an outlet NTC reads above 45 °C or is invalid. The rule releases
 * the fans once both NTCs read below 42 °C. The tick also ends a manual run, and ends and starts
 * flap pulses. The fans are commanded on while the owner asks, or while the rule demands.
 *
 * The flaps. A flap moves by a pulse of ACE2K_FLAP_PULSE_MS on one coil, never above
 * ACE2K_FLAP_PULSE_MAX_MS. Then both coils are released. The actuator is bistable, and holds its
 * position unpowered (docs/hardware.md "Exhaust flaps"). One flap moves at a time: a pulse asked
 * while a pulse of either flap runs waits for it. There is no position cache, so every request
 * pulses. The published position (ace2k_airflow_flap_pos) is where the last pulse that ran to its
 * end left the flap. It is unknown at boot, while a pulse on the flap runs or waits, and after a
 * pulse that a shutdown cut short.
 *
 * The fan interlock. No tachometer exists: the fan read-back is the pin, never the rotation.
 * Heat's binding sets heat_holds every tick from ace2k_heat_active(). That covers a lease, an odd
 * half pending, a pulse not yet seen over, or the gate reading high in any state. A gate stuck high
 * is heat on, latched or not. While heat holds, the fans stay commanded on, whatever the fan rule
 * or the owner decides, because the gate never fires without both fans:
 * - an explicit fan-off returns -ACE2K_EBUSY, a manual ACEK_FAN off included;
 * - a MANUAL owner does not expire;
 * - the release of an owner drops only the ownership.
 *
 * The tick runs in interrupt context. The owner calls come from a task, which the binding brackets
 * with irq_save. */

#define ACE2K_AIRFLOW_FAN_ON_MC        45000 /* the fan rule: a fan is never off above this */
#define ACE2K_AIRFLOW_FAN_OFF_MC       42000 /* hysteresis: the rule releases below this */
#define ACE2K_AIRFLOW_FAN_MANUAL_MAX_S 600U  /* a design bound: a manual run ends by itself */
/* Measured on the unit: 150 ms moved both flaps both ways. The value adds 50 ms of margin. At
 * 200 ms, 20 of 20 moves per flap succeeded. */
#define ACE2K_FLAP_PULSE_MS     200U
#define ACE2K_FLAP_PULSE_MAX_MS 400U

/* The owners, in the order of precedence. A weaker owner never replaces the flap request of a
 * stronger one. The values are the report's owner field (docs/protocol.md); 2 is not used.
 * Today no owner outranks the dryer. Its callers keep their refusal handling as a defence
 * (flaps_ask in dryer.c). */
enum ace2k_airflow_owner {
    ACE2K_AIRFLOW_OWNER_NONE = 0,
    ACE2K_AIRFLOW_OWNER_MANUAL = 1,
    ACE2K_AIRFLOW_OWNER_DRYER = 3,
};
enum ace2k_flap { ACE2K_FLAP_BOTTOM = 0, ACE2K_FLAP_REAR = 1, ACE2K_FLAP_COUNT = 2 };
enum ace2k_flap_pos { ACE2K_FLAP_UNKNOWN = 0, ACE2K_FLAP_OPEN = 1, ACE2K_FLAP_CLOSED = 2 };

/* A flap request: its direction, and who asked for it. */
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
    bool thermal_on; /* the demand of the fan rule */
    bool commanded;  /* what was last written: owner_on || thermal_on */
    uint32_t manual_until_ms;
    struct ace2k_flap_request flap_run[ACE2K_FLAP_COUNT];  /* the pulse on the coil now */
    struct ace2k_flap_request flap_wait[ACE2K_FLAP_COUNT]; /* the one waiting: depth 1 */
    uint32_t flap_until_ms[ACE2K_FLAP_COUNT];
    /* Where the last pulse that ran to its end left the flap. It is written only then, and set to
     * unknown when a shutdown cuts a pulse. */
    enum ace2k_flap_pos flap_done[ACE2K_FLAP_COUNT];
    bool heat_holds;    /* ace2k_heat_active(), set every tick by heat's binding */
    bool measured_warm; /* the last tick: a valid outlet NTC at or above ACE2K_AIRFLOW_FAN_OFF_MC */
};

/* Sets owner NONE and the fans off. It releases both coils of both flaps, writing each once. The
 * positions are unknown. */
void ace2k_airflow_init(struct ace2k_airflow *self, const struct ace2k_airflow_ops *ops, void *ctx);

/* Asks for the fans on or off. Returns:
 * - 0 on success;
 * - -ACE2K_EINVAL: who is NONE; or, for MANUAL, seconds is 0 or above the maximum;
 * - -ACE2K_EBUSY: another owner holds the fans, or heat_holds is set and on is false.
 * The dryer takes the fans over from the manual owner. on == false releases the ownership. seconds
 * is ignored for the dryer, whose ownership has no end of its own. */
int ace2k_airflow_fans(struct ace2k_airflow *self, enum ace2k_airflow_owner who, bool on,
                       uint32_t seconds, uint32_t now_ms);

/* The owner who gives the fans up. It is ignored for any other owner. While heat_holds is set, the
 * fans stay commanded on all the same, with no owner, because the gate never fires without both
 * fans. */
void ace2k_airflow_release(struct ace2k_airflow *self, enum ace2k_airflow_owner who);

/* Asks for a flap pulse. Returns:
 * - 0 on success;
 * - -ACE2K_EBUSY: another owner than who holds the outputs. NONE and MANUAL may pulse only while
 *   the owner is NONE or MANUAL.
 * - -ACE2K_EINVAL: no such flap.
 * A NONE or MANUAL pulse while a pulse runs or waits gets -ACE2K_EBUSY. It is never queued, so 0
 * means that it runs now.
 * The two flaps never pulse together. A pulse of the dryer asked while a pulse of either flap runs
 * is queued on its flap: depth 1 per flap, and the latest request wins. But it never wins over a
 * stronger owner: while a pulse of a stronger owner runs or waits on that flap, the answer is
 * -ACE2K_EBUSY. The order of the enum is the precedence. When a pulse ends, the waiting request
 * of the other flap starts first, so neither flap starves the other. Else this flap's own waiting
 * request starts. */
int ace2k_airflow_flap_pulse(struct ace2k_airflow *self, enum ace2k_airflow_owner who,
                             enum ace2k_flap flap, bool open, uint32_t now_ms);

/* Heat's binding passes ace2k_heat_active() here every tick, and again after the hook of the
 * dryer. The heater never runs without the fans. So while holds is true, the fans stay commanded
 * on, whatever the owner or the temperature rule decides. This call only stores the value. The fan
 * output follows at the next owner call or tick. */
void ace2k_airflow_set_heat_holds(struct ace2k_airflow *self, bool holds);

/* Runs every 10 ms, before heat and dryer: the fan rule (an invalid NTC counts as hot), the expiry
 * of a manual run, and the pulses. */
void ace2k_airflow_tick(struct ace2k_airflow *self, int32_t ntc_left_mc, int32_t ntc_right_mc,
                        bool left_valid, bool right_valid, uint32_t now_ms);

/* The Klipper shutdown. Every owner is released. Every pulse ends with both coils released, and
 * the waiting requests are dropped. A flap whose pulse was cut is published as unknown, because it
 * may have stopped mid-travel. It stays where it is until the close at the next boot.
 * heat_holds is left as it is. Heat's binding recomputes it every tick, and a gate stuck high
 * keeps holding the fans through the shutdown. The fan rule stays in force too. The fans remain on
 * while either one demands them, and the next ticks keep applying both. */
void ace2k_airflow_shutdown(struct ace2k_airflow *self);

/* True when the air path must keep running. Then the bootloader must not be entered, because it
 * drives no fans. That is the case when:
 * - the dryer owns the fans, from its start until its cool-down or fault has closed the flaps;
 * - heat holds them (heat_holds);
 * - or the fan rule holds them on a measured reading: a valid outlet NTC still at or above 42 °C.
 * The fan rule holding them only because both NTCs read invalid does not count. A unit with a
 * broken sensor must stay recoverable over the wire. The cool-down bounds this: the dryer hands the
 * fans back within 10 min of its end, and the fan rule lets go below 42 °C. */
bool ace2k_airflow_fans_required(const struct ace2k_airflow *self);

/* The fans as last written: on while the owner asks, the fan rule demands, or heat holds them.
 * heat takes it as its fans_commanded input. */
bool ace2k_airflow_fans_commanded(const struct ace2k_airflow *self);
/* The flap's published position: ACE2K_FLAP_UNKNOWN while a pulse on it runs or waits. Else
 * it is where the last pulse that ran to its end left it. It is unknown if none did, or if a
 * shutdown cut it. */
enum ace2k_flap_pos ace2k_airflow_flap_pos(const struct ace2k_airflow *self, enum ace2k_flap flap);
/* Both positions as reported: bits 0–1 bottom, bits 2–3 rear (enum ace2k_flap_pos). */
uint8_t ace2k_airflow_flaps_report(const struct ace2k_airflow *self);
/* True when a request of who on the flap is running or waiting: the pulses of who are not all
 * over. */
bool ace2k_airflow_flap_pending(const struct ace2k_airflow *self, enum ace2k_flap flap,
                                enum ace2k_airflow_owner who);
uint8_t ace2k_airflow_fans_read(const struct ace2k_airflow *self); /* ops->fan_read */

#endif

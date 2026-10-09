# Hardware

This page holds the facts about the Anycubic ACE 2 Pro that the firmware relies on. Everything
here was measured on real units, in a teardown and in bench sessions. Nothing comes from the
factory firmware's internals. The page holds what the whole tree needs, and a figure that is a
measurement says so.

## MCU

- **GD32F303VCT6** (GigaDevice), LQFP100, Cortex-M4, **256 KB flash, 48 KB SRAM**. It is
  register-compatible with the STM32F103 family, but has its own clock tree.
- The part runs at **120 MHz** from the **24 MHz crystal**, with two flash wait states and both
  APB buses at 30 MHz. Above 100 MHz it needs the high-driver enable sequence of its
  power-management unit (`src/stm32/gd32f30x.c`). Verified on the unit: an hour on Klipper with
  the host's clock estimate within 0.004 % of 120 MHz, ten power cycles, and the restore drill.
  When `ACE2K_CLOCK_120M` is left out of the build, the same clock code runs the part at 72 MHz.
- No FPU use. Klipper never enables it: the build has no FPU flags, and the core uses integers
  with explicit units.

## Memory map

| Range | What | ace2k |
|---|---|---|
| below `0x08008000` | the unit's bootloader (reports `V1.0.2`) and its own data pages, including the per-unit sensor calibration | **never written** |
| `0x08008000` – `0x08024000` | application slot, 112 KB | the ace2k image; its last 2 KB page (`0x08023800`) is the ace2k config page |
| `0x08024000` and above | update staging used by the bootloader | never written by ace2k |

Flash is programmed in 32-bit words, in pages of 2 KB (observed on the bench, 2026-09-12).

**The config page does not survive a reflash** (observed on the unit). A record written by the
running image was gone after an upload through the bootloader's recovery. It was also gone after
an upload through the factory firmware's updater. The bootloader's update erases the whole
application slot, the last page included, not only the pages the new image covers. So nothing
ace2k stores there may assume it outlives a flash: every image starts with a blank page.

On every reset the bootloader validates the application and jumps to it. After a watchdog reset,
or if the application is invalid, it stays in a recovery mode instead. That mode accepts an
upload over the host link (see `flashing.md`). An application that does not feed the independent
watchdog within its first seconds is reset into recovery. So ace2k re-arms the watchdog at
~0.4 s before it touches the clock tree (`src/stm32/gd32f30x.c`). Klipper's `watchdog_init`
re-arms it again, and a scheduler task feeds it.

The bootloader arms the watchdog at /128 with a reload of 4000, a timeout of about 13 s.
**Observed** on the bench (2026-09-27) with one image, on every boot, power-on and software reset
alike:

- the prescaler read 5 (/128), the reload 4000 and the status register 3 (both update flags set);
- the values were unchanged a minute after boot, and health bit 28 reported it;
- the unit still reached recovery once it was no longer fed.

**Likely cause, to be confirmed on the bench:** the bootloader jumps with its own prescaler /
reload update still in flight. The image's write made during that update was lost, and the update
stayed pending. The early arm now reloads the watchdog first. Before each write it waits for the
matching update flag to clear, bounded to about 10 ms at most each (GD32F30x User Manual, free
watchdog timer: wait for PUD / RUD before writing the prescaler / reload). After the start it
waits for both flags to clear. Klipper's `watchdog_init` then writes the same values again without
waiting. That write lands on an idle watchdog unless the early arm's last wait timed out. Bit 28
reports either failure.

## Host link

The link is RS-485, half duplex. One differential pair is on the rear 6-pin port (`B`, VCC, `A`,
GND on pins 1–4). The 4-pin chain port carries the same pair passively, with no power.

- **UART: UART4 on PC10 (TX) / PC11 (RX)**. The factory firmware and the bootloader use
  230400 8N1. ace2k uses Klipper's default 250000.
- **Transceiver driver-enable: PA11**, high to transmit. The receiver-enable is on the same pin,
  so the unit does not hear its own bytes (measured 2026-09-12). Because of that, the unit yields
  the line: it raises PA11 only after 100 µs of silence since the last received byte
  (`protocol.md`, "The link is half-duplex").
- **USB conversion happens in the external cable**, not in the unit: a CH343 USB-to-serial
  converter (USB ID `1a86:55d3`) moulded into the cable's printer-end connector
  ([`cable.md`](cable.md)).

The transceiver is `U10` on the mainboard, a SOIC-8 from 3PEAK marked `T176A`. Its exact part
number was not resolved, and nothing depends on it. Its role was proven by continuity: its pin 6
(`A`) reaches pin 3 of the rear 6-pin port, and its pin 7 (`B`) reaches pin 1. It sits beside the
rear connector's header, with a pair of 22 Ω resistors in series on the line. The unit does not
terminate the bus: `A` and `B` read open to each other at the port. The termination is left to
the ends of the line.

## Lane LEDs

**PE13 / PE9 / PC15 / PE5**, lanes 1→4, **low = on** (measured 2026-09-12).

The other firmware on the unit use these patterns (observed). ace2k never reuses them:

- factory firmware, idle: all four on, steady;
- bootloader, normal boot: dark for 1–2 s;
- bootloader recovery: all four strobing at ~17 Hz.

ace2k's own patterns (after v0.11.0): steady means a state, and blinking asks you to look. Every
lane blinks on one shared clock, so lanes in the same state blink together.

| What | LEDs |
|---|---|
| lane empty | off |
| filament present — idle, assisting or following | on, steady |
| moving — feed, rollback, load, unload | blinking, 1 Hz |
| lane in error | blinking, 5 Hz |
| boot of a proven image, every lane empty | dark 300 ms (the insert sensors settle), then the intro: 1 → 4 → 1 twice, 150 ms a step, once |
| unproven image waiting for the host | chase 1 → 4, 250 ms a step, over every lane |

A loaded lane at rest looks like the factory firmware's idle. To tell them apart, look for the
intro at boot (an empty unit) or use `ACE_STATUS`.

## Analogue inputs

ADC1, 12-bit, 3.3 V reference: millivolts are `raw × 3300 / 4095`. The pins were measured
2026-09-12, with the MCU as the instrument. The two NTC sides were told apart 2026-09-13, with a
heat gun on each outlet, in both directions.

ace2k samples twelve channels: the ten inputs below, the auxiliary and the internal 1.2 V
reference (channel 17). One pass runs every 10 ms. The channels are spread 833 µs apart across
the period, not back to back. A pass run back to back held the peripheral long enough for
Klipper's own `analog_in` to fall behind and stall the host link (measured on the unit,
2026-09-17). The MCU's die temperature (channel 16) is left to Klipper's `temperature_mcu`, so the
two users of the ADC never share a channel.

| Input | Pin / channel | What is known |
|---|---|---|
| Outlet NTC **left** | PA5 / ch 5 | the original protocol's `ptc1_temp` |
| Outlet NTC **right** | PA4 / ch 4 | its `ptc2_temp` |
| Insert sensor, lanes 1–4 | PC5, PC4, PC3, PC2 / ch 15, 14, 13, 12 | an analogue position on the lever board at the slot entrance (below) |
| *Empty* inputs, lanes 1–4 | PA2, PA3, PB0, PB1 / ch 2, 3, 8, 9 | sampled by the factory firmware and never read by it. ace2k publishes the millivolts and judges nothing. Measured 2026-09-17: ≈ 0.8–1.0 V with nothing in the lane, and **no reaction to a filament** on any lane. What they measure is still unknown |
| Auxiliary | PC1 / ch 11 | sampled by the factory firmware, read by nothing. Thévenin ≈ 1.28 V / ≈ 15 kΩ at the pin (2026-09-12), ≈ 1.26–1.29 V through the ADC (2026-09-17). Identity unknown |
| Internal reference | channel 17 | 1.2 V typical; VDDA = 1200 mV × 4095 / raw. 3.256 V on the bench unit (2026-09-17/18), against a valid window of 3.3 V ± 5 % |

**Outlet NTCs.** 100 kΩ at 25 °C, B = 3950, 100 kΩ pull-up to 3.3 V. ace2k converts through a
generated table (`firmware/tools/ntc_table.py`, one entry every 25 mV, linear interpolation).
Each reading first passes a one-pole filter (α = 1/8). The filtered reading is **valid inside
100–3200 mV**, the factory firmware's window, ≈ −37 … +131 °C. Outside it, the `valid` bit clears
and the health bit trips. The value never becomes a NaN or a sentinel. At rest the two NTCs read
within 0.5 °C of each other (2026-09-13). After a cold boot they read within 0.24 °C of the
chamber sensor (2026-09-17).

**Insert sensors.** With no filament, the reading sits near the **high** value of the lane's
factory pair. A filament at the slot entrance pulls it below the **low** value: ≈ 1.2 V down to
≈ 0.3–0.4 V on the bench unit. When the filament is withdrawn, the reading returns to within a few
millivolts (2026-09-17, all four lanes). **Filament present is the low reading.**

ace2k applies a one-pole filter (α = 1/5) and a hysteresis on the lane's band. Filament is
present at or below 30 % of the band, and absent at or above 70 %. A change then needs two
identical 10 ms ticks. This is the factory firmware's rule, with the trips the right way up.

A sample at a rail is not filament. A rail is at or below 20 mV, or at or above 3250 mV (initial
values). A disconnected lever board reads at one end of the scale, a shorted one at the other, and
the low rail lies inside every band's "present" region. So such a lane reads absent, its filter is
reseeded from the next in-range sample, and the health bit says why.

The sensor reads the internal flap the filament pushes, not the external lever. A finger on the
lever did not move the reading. A 5 mm piece of filament at the mouth gave a mid-band value
(2026-09-17). The band per lane comes from the unit's own factory page ("Factory calibration
page", below), unless `printer.cfg` names another.

## Digital inputs

Measured 2026-09-12. The buffer Hall switches are **active low**. They are polled every 10 ms with
a pull-up (initial value; the factory firmware's input mode is not recorded). A state moves after
two identical ticks, and every debounced edge is an event.

| Switch | Lane 1 | Lane 2 | Lane 3 | Lane 4 |
|---|---|---|---|---|
| plunger at *rest* | PB4 | PB5 | PD7 | PE0 |
| plunger at its *pushed* end | PC12 | PD0 | PD1 | PD2 |

The four *pulled*-end switches are wired-OR onto one input, **PE1**: low = some plunger at its
pulled end, with no way to tell which lane.

Walked by hand on all four lanes (2026-09-17):

- at the pushed end, *pushed* alone;
- at the pulled end, *pulled* alone;
- at rest, *rest* alone;
- between the switches, none of the three.

Over 134 debounced events, **rest and pushed were never asserted together**: the previous switch
always releases before the next asserts. This is what enables health bits 13–16. *Rest* and
*pulled* can overlap for one tick on the way back from the pulled end (seen once, lane 4). The
rest switch releases early in the travel. The two end switches trip just short of the mechanical
stop.

**Thermal cutout fault input, PD15.** It reads 0 as a floating input and is left floating.
Asserted would be high. It was never seen asserted, not even through ≈ 245 s of driven heating
(2026-09-13), so the asserted level is inferred, not observed. Its companions are the triac gate
PC8 ("Heater", below) and PC9. PC9 is the pulse that resets the cutout's latch after an event.
**No ace2k image drives or configures PC9.** Every image with the lane or the heater only reserves
it, once, so that no host section can claim it (`protocol.md`, the dictionary's constants).

## Chamber sensor

An AHT20-class temperature / humidity sensor on **PE14 SCL / PE15 SDA**, address `0x38`. It is
bit-banged: no I²C peripheral is involved (measured 2026-09-12). ace2k drives it through Klipper's
software I²C at 100 kHz, from a task. The firmware owns the bus (no host oid), because the dryer
will need the reading with no host.

The transaction, from the datasheet (Aosong AHT20):

1. Read the status byte. If bit 3 (*calibrated*) is clear, send `BE 08 00`.
2. Trigger with `AC 33 00`.
3. Wait ≥ 80 ms.
4. Read 7 bytes: status, 20-bit humidity, 20-bit temperature, CRC-8 (polynomial `0x31`, initial
   `0xFF`).
5. Convert: RH % = raw / 2²⁰ × 100, °C = raw / 2²⁰ × 200 − 50.

One cycle runs per second. A busy answer is retried up to three times, 10 ms apart. A cycle older
than 2 s clears the `valid` bit. Plausible values: −40 … 85 °C and 0 … 100 % RH.

Measured 2026-09-17:

- after a cold boot, within 0.24 °C of the two outlet NTCs;
- after two hours of handling with the lid open, 2.3 °C above them (it sits where the operator's
  hands were);
- the breath test took the humidity from 65.6 % to a peak of 93.2 %, and back to 89.8 % after
  60 s, still falling.

## Heater

Two PTC heater modules, one per air duct (left and right), each on a finned aluminium heatsink.
**One triac on the mains switches both together**: there is no per-side control. At the teardown
(2026-09-10) the mainboard showed fan and NTC connectors per side, but no heater connector, and
one gate line leaving for the mains board.

A **`KW-C115C` bimetallic cutout at 115 °C** is clamped to the heatsink of the module opened at
the teardown, in series with one heater lead. Two things have not been measured: whether the
other module carries its own cutout, and how the two modules join. A tripped cutout is reported
on **PD15** ("Digital inputs"). The dryer's highest drive (65 °C asked, 73 °C at the outlet NTC)
sits 40 °C below it. The cutout is a last resort, never part of the control.

| Signal | Pin | Measured |
|---|---|---|
| Mains zero-cross | **PC0**, rising edge, one edge per half-cycle ("Counters") | 120 edges/s on 60 Hz mains, 118–122 through every heating run. A 7 ms lockout between edges rejected none on clean mains (2026-09-13). On 2026-09-28, at the moment the dryer took its first lease, the lockout rejected 2 edges and one 1 s window read 58.5 Hz (the next read 59.0–60.5): a transient dip the heater's mains bucket rides out. Every off-band window of the dryer bench (2026-09-28) spans a flash write of the dryer's log. Its erase and programming hold interrupts off, so all the edges in that time but one are lost. Since 2026-09-29 the frequency is measured from the edges' own times, over runs of edges that a write's late edge ends. The late edge is told by the input's pending bit, read as the write ends. So a write no longer reads as a dip, and a dip next to a write still does |
| Triac gate | **PC8**, **high = heating** | an external pull-down holds the gate off with the pin released (read 0 floating, 2026-09-12). Polarity proven 2026-09-13: 24 half-cycles at 10 % for 2 s moved both outlet NTCs |
| Cutout fault | **PD15** | never asserted through ≈ 245 s of heating (2026-09-13) |

**Firing.** Whole half-cycles, never phase angle. At a zero-cross edge the gate is raised for
6 ms and released. A half-cycle not fired carries no current. A duty of *D* % fires *D* of every
100 half-cycles, spread evenly and in whole cycles: both halves of a cycle or neither, so the load
draws no DC. Measured 2026-09-13 with a test image and a fan-first interlock: over five drives,
**8 590 half-cycles fired** in ≈ 245 s of heating. The count matched the duty exactly every time,
with no fault.

**Power.** A clamp meter on the mains lead (average-responding, not true-RMS) read 0.9–1.0 A at
35 % and 1.5 A peak at 50 %. That gives **≈ 2.7–3.0 A at full conduction, ≈ 340–380 W at
127 V**, with the PTCs still fairly cold (a PTC's current falls as it warms). 50 Hz / 220 V has
not been measured.

**Thermal response** (2026-09-13, unit closed and empty, fans on; four runs recorded on the unit):

| Duty | Right outlet NTC | Left outlet NTC | After the gate stopped | Chamber |
|---|---|---|---|---|
| 20 %, 120 s | 0.122 °C/s for a minute, then ≈ 0.06 °C/s — toward a plateau near 36–37 °C | 0.083–0.088 °C/s, then 0.057 | +0.2 °C | +2.2 °C, RH 60 → 57 % |
| 35 %, stopped at 50 °C (73 s) | 0.25 → 0.40 °C/s | 0.14 → 0.29 °C/s | +1.3 °C | +4.0 °C, RH 59 → 52 % |
| 50 %, stopped at 50 °C (40 s) | 0.22 → 0.70 → 0.81 °C/s | 0.095 → 0.435 → 0.469 °C/s | **+3.8 °C** | +3.0 °C |

- **Lag from the gate to the outlet NTC: 10–15 s.** The first 15 s of every run show only a
  fraction of the steady rate. The NTC peaks 10–16 s after the gate stops. So a controller that
  stops at its target reads up to 4 °C more later, and this overshoot grows with the duty.
- **Cool-down with the fans on:** 51 → 40 °C in ≈ 30 s, 54 → 40 °C in ≈ 44 s.
- **The right side heats faster than the left: ≈ 1.3–1.8×. This is a property of the unit**
  (settled 2026-09-27). The 2026-09-13 runs showed 1.5–1.7×, but they followed a heat gun that
  had warmed the right duct more. The 2026-09-27 runs were driven by ace2k on the same unit, cold
  for over an hour, flaps closed, with the chamber sensor 1.2–1.8 °C above both NTCs at rest.
  Each run gives two measures, from the CSVs:

  - the least-squares slope of each NTC 15–45 s after the first gate pulse (past the 10–15 s
    lag);
  - each NTC's rise from the row before the first pulse to its peak after the stop.

  | Run | Start, right / left | Slope 15–45 s, right / left | Ratio | Rise to peak, right / left | Ratio |
  |---|---|---|---|---|---|
  | 20 %, 120 s, cold | 24.2 / 24.7 °C | 0.160 / 0.107 °C/s | **1.50** | +12.0 / +9.3 °C | **1.28** |
  | 35 %, to 50 °C | 29.6 / 30.3 °C | 0.358 / 0.248 °C/s | **1.44** | +22.3 / +15.4 °C | **1.45** |
  | 50 %, to 50 °C | 32.3 / 33.8 °C | 0.737 / 0.443 °C/s | **1.67** | +23.0 / +12.6 °C | **1.83** |

  The 10 % / 2 s pulse (22 half-cycles) moved the NTCs +1.1 / +1.2 °C, too little to tell the
  sides apart. The 35 % and 50 % runs started warm, after a cool-down. The cold 20 % run alone
  gives 1.3–1.5×, and no run shows the left ahead. With one gate for both modules, the cause is
  physical (the modules, the ducts or the NTCs' placement), and it is not known.
- Not driven so far: a duty above 50 %, a run longer than 120 s, an outlet NTC above 55.3 °C (the
  2026-09-27 50 % run's overshoot).

## Fans

Two 24 V fans, one per duct: **PE12 = left, PE8 = right** (unit seen from the front),
**high = on**. External pull-downs keep a released pin off. Measured 2026-09-12 (each fan run
alone for ~5 s, airflow felt at its outlet) and again 2026-09-13 (a fan per side by ear). Their
current does not show on the clamp meter.

**There is no tachometer and no current sense.** The firmware can read back its own pin, but
never whether the fan turns. A stopped fan shows up, if at all, only in the temperatures.

## Exhaust flaps

Two flaps, one on the bottom and one on the rear of the case. Each is moved by a **bistable
electromagnetic actuator** behind a `TC118S` H-bridge. The actuator is a camera IR-cut filter
switcher, repurposed as a damper (identified at the teardown, 2026-09-10). A pulse on one of its
two inputs drives a flap. The flap then **holds its position unpowered**, across a power cycle
too. **There is no position readback.**

| Flap | Opens | Closes | Measured |
|---|---|---|---|
| Bottom | **PD4** high (PD3 low) | **PD3** high (PD4 low) | 2026-09-12: a 300–400 ms pulse on each input moved it each way, seen by the operator; 2026-09-27: 150 ms moved it each way, and 200 ms 20 of 20 times (10 closes, 10 opens) |
| Rear | **PD6** high (PD5 low) | **PD5** high (PD6 low) | 2026-09-12: likewise; 2026-09-27: likewise, 150 ms each way, 200 ms 20 of 20 |

Both inputs low release the coil (the bridge coasts). Both high is the bridge's brake, which
nothing needs and ace2k never writes.

**ace2k pulses for 200 ms** (`ACE2K_FLAP_PULSE_MS`). In a dryer bench run, 150 ms, the first value
tried, moved both flaps both ways. 200 ms keeps 50 ms of margin over it. At 200 ms each flap moved
20 of 20 times, counted by the operator. No pulse shorter than 150 ms has been tried.

## Motors

One 24 V motor per lane, with its driver inside the motor. The mainboard sees three signals per
lane and one tach output. Measured 2026-09-12 with the MCU as the instrument: each motor turned
alone, and the tach was counted on its own channel only.

| Lane | PWM (TIM2) | run (A) | direction (B) | tach (TIM4) |
|---|---|---|---|---|
| 1 | CH1 **PA15** | **PE10** | **PE11** | CH1 PB6 |
| 2 | CH2 **PB3** | **PB2** | **PE7** | CH2 PB7 |
| 3 | CH3 **PB10** | **PE6** | **PC13** | CH3 PB8 |
| 4 | CH4 **PB11** | **PE2** | **PE3** | CH4 PB9 |

**PWM: 30 kHz, active low.** The pin high means *stop*. The driver's own pull-up holds a floating
pin high. So reset, bootloader recovery and a hung MCU all leave a motor still (all four PWM pins
read 1 floating, motors still).

TIM2 reaches these pins through its full remap. ace2k drives the timer itself
(`src/ace2k_board/motor.c`), for this reason. Klipper's F1 pin setup rewrites the whole remap
register from a shadow of its own writes, and that shadow never saw the full remap. It does so
whenever it configures a pin for a function whose remap it emulates. That would put channels 1–2
back on PA0 / PA1, lane 4's encoder inputs, mid-move. So the firmware:

- reserves the pins a host section could reach that code through (`RESERVE_PINS_ace2k_remap`:
  PA13, PA14, PD5, PD6, PD8, PD9 — none of them driven by ace2k);
- reserves every other channel pin of TIM2 (in `RESERVE_PINS_ace2k_motor`), because a host
  `hardware_pwm` on one would reprogram the whole timer;
- re-checks the remap register every tick and before every duty write. This covers the
  bus-named host uses (`spi_bus`, `i2c_bus`), which never pass through pin reservation.

The timer runs at 30 kHz from its 60 MHz clock (an F1 timer on a prescaled bus runs at twice the
bus clock). It uses PWM mode 1 with the compare preloaded: 2 000 steps of duty.

**A = run**: 1 while a move runs, 0 at idle. **B = direction**: 0 toward the printer, 1 toward the
spool; it holds its last value at idle. A = 1 with the PWM driving turned that lane's motor, and
only that lane's. B = 0 turned the outlet gear forward, B = 1 backward. ace2k sets B, then A, then
the PWM, and never changes B while A is high.

**Duty → speed, unloaded** (no filament, no spool). 10 % did not move a motor (no tach pulse, no
sound). 20 % turns it at ≈ 180 tach pulses per second ≈ 15 mm/s (the four lanes 177–186
pulses/s, 2026-09-12). Measured on the bench on 2026-09-20, lane 1: the tach rate at each duty is
a straight line within ±2 %. This is the firmware's feed-forward table (`lane.c`):

| duty | 20 % | 30 % | 40 % | 50 % | 60 % | 70 % | 80 % | 90 % | 100 % |
|---|---|---|---|---|---|---|---|---|---|
| tach pulses/s | 190 | 307 | 423 | 538 | 649 | 762 | 880 | 992 | 1126 |
| mm/s | 15.4 | 24.9 | 34.3 | 43.6 | 52.6 | 61.7 | 71.3 | 80.3 | 91.2 |

**Under load:**

- 20 % started and sustained the motor under a 1 kg spool at 8.5 mm/s, and 25 % at 13.5 mm/s.
- A 100 mm closed-loop move at 15 mm/s completed under load at 20–28 % duty.
- At 100 % the speed was 79.4 mm/s under a well-wound ~950 g spool, and 76.9 mm/s under a badly
  wound 1 kg one.

The constants chosen from this:

- The duty floor stays **20 %**. Measured 2026-09-27 under a 1 kg spool: 20 % ran steadily at
  9.7 mm/s. 15 % ran at ≈ 4.7 mm/s, but not steadily at the tach's resolution. 10 % and 5 % did
  not start the motor, nor did 10 % unloaded.
- The speed floor is **9 mm/s**: the floor duty's loaded speed, rounded up.
- The ceiling is **70 mm/s**: 90 % of the well-wound spool's maximum (79.4 → 71.5 mm/s), rounded
  down to 5 mm/s. The badly wound spool's 76.9 mm/s leaves 9 % of headroom at it. Higher was
  preferred, without much overshoot.

With the initial straight-line feed-forward, the loaded closed loop converged too slowly: 100 mm
at 30 / 50 / 70 mm/s ended 6–11 % under the setpoint after 2–4 s. The measured table removes most
of the feed-forward error. The gains (`ACE2K_LANE_KP_DIV` 1500, `ACE2K_LANE_KI_DIV` 6000) answer
the load's 15–20 % speed drop within about a second.

**Observed on the bench, 2026-09-20:**

- Direction: `dir=0` (B low) turns the outlet gear toward the outlet. Confirmed with the motor
  running.
- A slip clutch sits between the motor and the spool-holder drive. A fingertip on the external
  gear stops it while the motor and the outlet gear go on. Unloaded, the external gear barely
  turns.
- The drive drags the encoder wheel even with no filament in the lane: ≈ 70 % of the motor's
  travel on lane 1, 0–2 counts on lanes 2–4. So an empty-lane move reads a filament travel. With
  a filament, the encoder tracks 92–101 % of the motor's travel.
- The buffer's three readings, with a filament:

  - *full*: the plunger at its pulled end (the lane pushed too much);
  - *rest*: the spring centred;
  - *taut*: the pushed end (the head pulls).

  Feeding into a blocked outlet fills the buffer (*pulled*) after ≈ 18 mm. Pulling the filament
  out at the outlet, once the slack is gone, reads *pushed*. A hand cannot hold the filament
  against the motor: what blocks the outlet is the end of the PTFE tube.
- Both assists keep the buffer at rest. This was decided on the unit:

  - The forward assist bursts on *taut* and stops at *rest*. A cycle that fed until *pulled* kept
    the filament compressed in the tube.
  - The reverse assist takes up only on this lane's *full*, and stops at *rest*. A creep at rest
    ran on with nothing holding the filament: 267 mm in seconds.
- In a feed and in a load's pull, this lane's *full* ended the move at once as `stuck`. Since
  v0.7.0 it is judged by the tip snag's tolerance first (`protocol.md`, "Feed events"); the tip
  snag is the thick tip described two items below.
- A reverse move whose filament stands still, with the plunger at rest, has the end of the
  filament past the drive gear. Only the spool can move it then. In a further unload test on
  lane 2, it took 54 mm of motor until the sensor cleared.
- A filament tip thicker than the filament can catch the entry of the tube the plunger carries. A
  printer without a filament cutter forms such a tip. It drags the plunger to its pulled end (or
  its pushed end, on the way back) with nothing holding the filament. A feed right after a load
  read *full* 41 mm in, with the filament following the motor 1:1. One touch of the plunger put it
  back at rest (motor 41.214 mm, filament 41.44 mm).
- The path from the drive's grip to the outlet measured **323 mm** on the development unit's
  lane 1. Method: 50 mm feeds from the point where the gear takes the filament, a 25 mm rollback,
  and the tip 1–2 mm out of the outlet at an encoder reading of 324 mm. The insert sensor sits a
  few millimetres before the gear. The load's parking distance defaults to 300 mm, which leaves
  the tip 23 mm inside the outlet.
- The position stop closes on the encoder. Its resolution is one count (≈ 1.2 mm) plus the 10 ms
  tick, and the target rounds to the nearest count: a 0.5 mm request moves one count. The motor
  coasts after the stop: 1–2 pulses at ≤ 50 mm/s, 8–9 (≈ 0.7 mm) at 90 mm/s, measured while the
  stop still counted the tach. That sits inside one count, so a cut from an interrupt is not
  needed.

**Tach (FG):** ≈ 12.35 pulses per millimetre at the outlet gear
(`ACE2K_LANE_FG_PULSES_PER_100MM` = 1235), no direction, counted by input capture (`fg.c`). The
tach drives a lane move's speed loop and its stall check. The move's length is the encoder's
travel ("Counters").

## Counters

Pins measured 2026-09-12.

**Filament encoders**: x4 quadrature, one advanced or general timer per lane. Lane 1 **TIM1
PA8/PA9**, lane 2 **TIM8 PC6/PC7**, lane 3 **TIM3 PA6/PA7**, lane 4 **TIM5 PA0/PA1**. ace2k runs
the timers in encoder mode (both edges of both inputs). They are free-running 16-bit counters,
extended to 32 bits by the 10 ms tick. The inputs are pulled up (a floating quadrature input
chatters), with no input filter (initial value). The scale is **1.2342 mm per count**, one
constant for all four lanes until a calibration measures them.

**Sign convention: positive toward the printer.** Settled with the motor on (2026-09-20): a
200 mm feed read +197.5 mm and the rollback −201.2 mm. The first reading by hand agreed (8 counts,
+9.873 mm, 2026-09-17, a filament entered toward the printer). The same runs close the ±5 % check
that the first, motor-off measurements left open. 197.5 mm read against ~195 mm on the ruler at
the default constant. After `ACE_CALIBRATE_ENCODER`, the development unit's lane 1 wheel read
1.2188 mm per count (0.1 % against the ruler). The wheels differ by a few per cent. That is what
the per-lane scale in `printer.cfg` is for (`laneN_encoder_scale`, accepted within ±20 % of the
default; `ace2k_lane_scale_set`).

The motor's travel and the filament's are not the same thing. On that lane, the motor's 200 mm
delivered ≈ 195 mm of filament under a 1 kg spool (≈ 3 % slip in the drive gear). So a lane
move's length is the encoder's travel. The position stop closes on the count, so a host that asks
for 100 mm gets 100 mm of filament, whatever the gear slips. The tach drives the speed loop.

**Motor tach (FG) outputs**: **TIM4 CH1–4 on PB6–PB9**, one pulse train per lane, no direction
(the rate per millimetre: [Motors](#motors)). ace2k counts them by input capture on the falling
edge (initial value; either edge counts), one interrupt per pulse, with the inputs pulled up. With
the motors unpowered, the count read **zero pulses on all four lanes** through every bench session
(2026-09-17/18).

**Mains zero-cross**: **PC0**, a driven signal read as a floating input. It has a rising edge once
per half-cycle, two per mains cycle: 120 per second on the bench's mains. ace2k routes it to EXTI
line 0 and records each edge's time. The frequency, in tenths of a hertz, is measured from the
edges' own times: an even number of half-periods over the time they took (until 2026-09-29 it was
the edge count of the last 1 s window divided by two; the Heater table above says why that
changed). It is *present* while an edge arrived within the last 100 ms. Measured
**60.0 Hz, present**, throughout (2026-09-17/18). Plausible values are 50 or 60 Hz ± 1.

## RFID readers

RC522-class parts on **SPI2: PB13 SCK / PB14 MISO / PB15 MOSI**, mode 0, measured 2026-09-13.
ace2k clocks it at 1 MHz.

- Reader **A**: NSS **PB12**, RST **PD13**, IRQ **PD14**. It serves lanes 3–4.
- Reader **B**: NSS **PD10**, RST **PD12**, IRQ **PD11**. It serves lanes 1–2.

Each pair of lanes shares one antenna. The version register (`0x37`) reads **`0x18`** on both.
That is a clone value; genuine parts read `0x91` / `0x92`. Measured 2026-09-13, and again
2026-09-18 through ace2k's own soft reset and read.

RST low = reset. An external pull-up keeps the readers running when the pin floats. ace2k drives
RST **high only**: a reader is reset through its command register, never by the pin. The IRQ
lines are wired and unused.

- **Register access:** address byte `(reg << 1) & 0x7E`, bit 7 set for a read (NXP MFRC522
  datasheet).
- **Soft reset:** command register (`0x01`) ← `0x0F`, then poll its bit 4 (*PowerDown*) every
  millisecond. It clears well inside one millisecond. The unit gives up after 150 ms (the public
  MFRC522 library's 3 × 50 ms budget, kept as the bound).

Nothing in the images before v0.4.0 turned the field on. Neither did the bench's probe image,
built without `CONFIG_ACE2K_RFID_READ` (the flag check proved it while that image was built).

**The field** (since v0.4.0; NXP MFRC522 datasheet). The antenna drivers are `TxControlReg`
(`0x14`) bits 0–1, *Tx1RFEn* and *Tx2RFEn*. The field is on with both set, and off with both
cleared. Every switch-off is **read back**. Either bit still set means the field is on: a fault
the unit latches and reports (`protocol.md`, "Tag states"). The field is on only while one of the
reader's lanes needs it. The two readers' fields are independent.

**The configuration.** A soft reset clears it. ace2k applies it before the field first comes on,
and again after any soft reset (the health probe's), as the public MFRC522 library applies it:

- `TModeReg` (`0x2A`) ← `0x80`: the timer starts at the end of every transmission (*TAuto*);
- `TPrescalerReg` (`0x2B`) ← `0xA9`: the timer runs at 13.56 MHz / 339 ≈ 40 kHz, 25 µs per
  count;
- the reload (`0x2C`/`0x2D`) ← 1000: **25 ms for a tag's answer**;
- `TxASKReg` (`0x15`) ← `0x40`: 100 % ASK modulation;
- `ModeReg` (`0x11`) ← `0x3D`: the CRC preset `0x6363` of ISO 14443-A.

If a configuration fails (its soft reset did not complete), it is retried no sooner than 1 s
later. After three failures in a row, the unit gives the reader up until the next health run.

**The exchange** never waits on the tag. The unit loads the FIFO, starts `Transceive` (`0x0C`) or
`MFAuthent` (`0x0E`), and returns. The task then polls `ComIrqReg` (`0x04`) on its later runs,
until one of three things happens: the tag answered, the reader's 25 ms timer expired, or
`ErrorReg` (`0x06`) flagged a protocol, parity, collision or buffer error.

A MIFARE authentication succeeded when `Status2Reg` (`0x08`) shows *MFCrypto1On*. That bit is
cleared again at the end of every MIFARE job, read or failed. An exchange still unanswered after
ten task runs (100 ms, four times the reader's own timer) ends its job in error. One exchange runs
per reader at a time; the two readers run in parallel. The image writes four reader commands:
`Idle`, `Transceive`, `MFAuthent`, `SoftReset`. The tag's **CRC_A is computed in software**
(ISO/IEC 14443-3, Annex B), so `CalcCRC` is never used.

**Read-only, by the first byte.** Every frame to a tag leaves through one function. That function
checks the frame's first byte against the protocol's activation and read commands:

- `REQA` `0x26`, `WUPA` `0x52`;
- the anticollision / `SELECT` codes `0x93` / `0x95` / `0x97`;
- `HLTA` `0x50`, `READ` `0x30`;
- the MIFARE authentications `0x60` (key A) / `0x61` (key B).

It refuses any other byte and leaves the reader untouched. So no write (`0xA0`, `0xA2`), no value
operation (`0xB0`, `0xC0`–`0xC2`), and no password, signature or lock command can reach a tag. No
host command carries a raw frame: the host names an operation, and the firmware builds it
(`protocol.md`, "RFID sessions"). A MIFARE key stays in the unit only from the step that carries
it to the start of its authentication. It is wiped then.

**The tags on the spools.** Measured 2026-09-13 through a bench image that drove the readers from
the host. A tag parked in a bay is seen by that bay's reader, and never by the other pair's.
Three spool tags read end to end, among them a MIFARE Classic 1K and an NTAG213. A Snapmaker
spool's tag sits next to the hub, **out of the bay antenna's reach**. It read only with the tag
taken out and held at the antenna.

**What the first reads showed** (2026-09-27), with no motion, through ace2k's own readers. Each
tag decoded to its label at the first try, and the field was off after every read:

- an **Anycubic** spool's **NTAG213**, on the reader of bays 1–2;
- a **Bambu Lab** spool's **MIFARE Classic 1K**, on the reader of bays 3–4;
- a **Snapmaker** tag's **MIFARE Classic 1K**, held at the antenna of bays 1–2.

The **Elegoo** and **Creality** spools on hand carry **no tag**. Nothing answered in 675 and
2 629 reads while each was turned in front of the antenna. So neither brand's format has been
checked against a real tag, and both ship disabled in `config/ace2k_tags.cfg`.

**The spool's geometry** (2026-09-27; lane 1, the reader of bays 1–2, the filament fed at 30 and
9 mm/s). A spool's tag passes the antenna once per revolution. The filament per revolution is
π × the winding's diameter:

- **≈ 519 mm** on an Anycubic spool of ≈ 800 g (winding Ø ≈ 165 mm);
- **≈ 432 mm** on a Bambu Lab spool of ≈ 600 g (Ø ≈ 137.5 mm).

The **window** is the filament travelled while the tag answers:

- **102–116 mm** for the Anycubic NTAG213, with a short flicker at each edge around a solid core
  of ≈ 60 mm;
- **45–56 mm** for the Bambu Lab MIFARE Classic 1K, with no flicker.

Every passage was seen at a 50 ms inventory period, at both speeds. The flanges measured
194–200 mm across five brands. So one revolution of any spool that fits takes **at most
π × 200 mm ≈ 628 mm** of filament. A stop sent on a tag's first answer at 30 mm/s ran **1.2 mm**
past it (one encoder count), and the tag stayed in the field.

## Factory calibration page

The factory firmware keeps a 1064-byte configuration record in the two 2 KB pages immediately
below the application slot:

- a **primary** copy at **`0x08007000`**;
- a **backup** at **`0x08006800`**.

Each copy is validated by a CRC-16 over the record, stored little-endian at offset `0x7FC` of the
page. The CRC uses initial `0xFFFF`, reflected polynomial `0x8408` and no final XOR:
CRC-16/MCRF4XX, the same CRC as the factory firmware's wire frames.

The four insert-sensor pairs are the first records. For lane *s* (0–3), they are two `uint16`
millivolt values, at record offsets `0x02 + 8s` and `0x04 + 8s`. ace2k takes the lane's band as
`low = min`, `high = max`. A pair whose values differ by 99 mV or less is degenerate, and the
built-in defaults **600 / 1100 mV** apply. This is the factory firmware's own rule. The factory
firmware reports these pairs over its protocol as `GET_MOTOR_STATUS`.

**Read at every boot, never written.** The firmware checks the primary's CRC, then the backup's,
then falls back to the defaults. It says which one it used (`ace2k_config_state`,
`ace2k_sensors_thresholds`, `ACE_STATUS`). The pages are read through plain memory reads. The
flash allow-list refuses both pages to the erase and program routines, and a host test proves it.

Verified 2026-09-16 on the development unit's own page copy: the stored checksum reproduces with
exactly that algorithm, and the four pairs equal what the factory firmware reports. Verified again
on the unit 2026-09-17 with the firmware running: `source = factory` on all four lanes, and the
bands equal to the unit's calibration report, pair for pair.

A `printer.cfg` override (`laneN_insert_mv`) wins over the page for that lane. The values of a
real unit never enter this tree (`decisions/0010-factory-calibration-read-at-boot.md`).

## Debug header `P1`

Silkscreened below the MCU: `RST · CLK · SWDIO · GND · 3V3 · TX · RX`. No debugger has been
attached to it yet. ace2k keeps SWD enabled (Klipper releases only the JTAG pins on an F1-class
part). If the running firmware has disabled SWD, attach under reset. `TX`/`RX` are believed to be
**USART3 on PD8/PD9**, but this has not been confirmed with an adapter yet.

## What the unit has (inventory)

- **Four filament lanes.** Each lane has:
  - a 24 V motor carrying its own driver (PWM, run/enable, direction, a tach output);
  - a quadrature filament encoder;
  - an analogue insert sensor, and a second analogue input (named *empty* by the original
    protocol);
  - three Hall switches on the buffer plunger (rest, pushed end, pulled end).

  The four pulled-end switches are wired-OR onto one input.
- **A dryer**: two PTC heaters on one triac (mains, with a zero-cross input and a 115 °C thermal
  cutout in series wired to a fault input), two outlet NTCs, a chamber temperature/humidity
  sensor, two fans, two exhaust flaps. Measured heater power ≈ 340–380 W at 127 V.
- **Two RC522-class RFID readers**, one per pair of lanes, sharing one antenna per pair.
- **Four lane LEDs.**

# Klipper patch

One patch, applied to the pinned Klipper commit (`../klipper.pin`) by `make build` as a local
commit that the build removes again. It never leaves this repository and must stay small
(target: under 100 added lines — the `+` lines of the diff, not its length).

`0001-gd32f303-and-ace2k-hooks.patch` (83 lines added, 17 replaced, 10 files — the count is
`git apply --numstat`'s, blank lines included):

- `src/generic/armcm_timer.c`: `timer_note_stall(grace_us)`, next to `timer_repeat_until`. A
  flash page erase or word program stalls the core (code runs from the flash being written), so
  no timer fires until it ends; the first dispatch after it finds Klipper's own 100 ms periodic
  timer overdue and would shut down with "Rescheduled timer in the past". The caller declares
  the stall before re-enabling interrupts and the dispatcher catches the overdue timers up within
  the grace window instead.
- `src/stm32/Kconfig`: the `MACH_GD32F303` processor entry (selects the STM32F1 family:
  register-compatible), its CMSIS header (`stm32f103xe`), the clock defaults — `CLOCK_FREQ`
  120000000 when `ACE2K_CLOCK_120M` is set, else 72000000 — flash 256 KB, RAM 48 KB; the serial
  choice `STM32_SERIAL_UART4_PC11_PC10_RS485` (UART4 PC11/PC10 with the RS-485 driver-enable on
  PA11, `depends on MACH_GD32F303`, selects `SERIAL`); `source "src/ace2k/Kconfig"`.
- `src/stm32/Makefile`: `-mcpu=cortex-m4` for the part; `src-$(CONFIG_MACH_GD32F303) +=
  stm32/gd32f30x.c`, our clock file (`make build` symlinks it into `klipper/src/stm32/`);
  `serial-src-$(CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485) := ace2k_board/serial.c`, which
  swaps our serial driver in for `stm32/serial.c` when that choice is taken;
  `-include src/ace2k/Makefile`.
- `src/stm32/internal.h`: the declarations of `clock_setup_gd32f30x()` and `timer_note_stall()`.
- `src/stm32/stm32f1.c`: `armcm_main()` calls `clock_setup_gd32f30x()` for `MACH_GD32F303`
  (a branch ahead of the n32g45x one — the single replaced line); `get_pclock_frequency()`
  returns `CONFIG_CLOCK_FREQ / 4` for the part above 72 MHz, where both APB buses run at /4 as
  they do on the n32g45x.
- `src/stm32/adc.c`: the `ADC_VREFINT` pseudo-pin (channel 17 on the F1 family), next to
  `ADC_TEMPERATURE`, so the internal reference is sampled like any other channel. Klipper's
  `gpio_adc_setup()` configures every pin it does not know as a pseudo-pin as an analogue GPIO,
  which for a pseudo-pin ends in `shutdown("Not a valid pin")`; the test that keeps
  `ADC_TEMPERATURE` out of that path now covers `ADC_VREFINT` too (the one replaced line there).
- `src/i2c_software.[ch]`: `i2c_software_setup()` factored out of `command_i2c_set_sw_bus` and
  exported, so a module of the MCU can own a bit-banged bus without a host oid (the chamber
  sensor must work with no host).
- `src/generic/serial_irq.c`, `src/generic/misc.h`: `console_try_sendf(ce, reserve, args)` — the
  body of `console_sendf()`, returning 0 at its two drop points (no room for the message; the
  encoded frame does not fit what is free) or when the frame would leave fewer than `reserve`
  bytes free behind it — the cheap check before the compaction counts the reserve too, so a
  frame that can never leave it free returns before anything is moved or encoded, and the check
  after the encode measures the free space from the compacted buffer's start — and 1 once the
  frame is queued; `console_sendf()` is a call to it with a reserve of 0 that drops the result.  Klipper drops a frame that does not fit its 96-byte
  transmit buffer silently; the ace2k bindings send their reports and events through the try
  variant (`src/ace2k/tx.[ch]`, where the varargs wrapper lives so that Klipper's generic
  `command.c` stays untouched and a tree with another console still links) and hold a frame
  that did not fit for their next tick, keeping room for a command response behind theirs.

## Regenerating it

Edit inside `../klipper` on the pristine pin, then `git -C ../klipper diff > 0001-….patch` and
`git -C ../klipper checkout -- .`. Check with `git -C ../klipper apply --check ../patches/0001-….patch`.
Write the patch out **before** building: `make build` starts by resetting the submodule to the
pin, and refuses to do so while `../klipper` carries uncommitted edits (`FORCE=1` discards them).

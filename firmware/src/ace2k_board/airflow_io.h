/* What: the dryer's air-path outputs on the board — both fans on PE12 (left) and PE8 (right),
 * active high with external pull-downs; the bottom flap's coils on PD4 (open) / PD3 (close) and
 * the rear flap's on PD6 (open) / PD5 (close), both low = released (docs/hardware.md "Fans",
 * "Exhaust flaps", measured 2026-09-12).
 * How: ace2k_airflow_io_init() once: every pin an output driven low; the writes are plain GPIO;
 * ace2k_airflow_io_fans_read() reads the two fan pins' input register — the pin, not the fan's
 * rotation (there is no tachometer).  A flap write drives the coil going low first.
 * Depends on: Klipper's gpio (board code, not host-compiled). */
#ifndef ACE2K_BOARD_AIRFLOW_IO_H
#define ACE2K_BOARD_AIRFLOW_IO_H
#include <stdbool.h>
#include <stdint.h>

#define ACE2K_AIRFLOW_IO_FLAPS 2U

void ace2k_airflow_io_init(void);
void ace2k_airflow_io_fans(bool on);
/* bit 0 the left fan's pin reads high, bit 1 the right fan's. */
uint8_t ace2k_airflow_io_fans_read(void);
/* flap 0 bottom, 1 rear; out of range ignored. */
void ace2k_airflow_io_flap(uint8_t flap, bool open_coil, bool close_coil);

#endif

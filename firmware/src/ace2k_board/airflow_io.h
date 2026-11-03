/* What: the dryer's air-path outputs on the board (docs/hardware.md "Fans", "Exhaust flaps",
 * measured on the unit):
 * - both fans, on PE12 (left) and PE8 (right), active high with external pull-downs;
 * - the coils of the bottom flap on PD4 (open) / PD3 (close), and those of the rear flap on
 *   PD6 (open) / PD5 (close). Both low = released.
 * How: call ace2k_airflow_io_init() once; it makes every pin an output driven low. The writes are
 * plain GPIO. ace2k_airflow_io_fans_read() reads the input register of the two fan pins: the pin,
 * not the fan's rotation, because there is no tachometer. A flap write drives the coil that
 * goes low first.
 * Depends on: Klipper's gpio (board code, not host-compiled). */
#ifndef ACE2K_BOARD_AIRFLOW_IO_H
#define ACE2K_BOARD_AIRFLOW_IO_H
#include <stdbool.h>
#include <stdint.h>

#define ACE2K_AIRFLOW_IO_FLAPS 2U

/* Once, at init: makes every fan and coil pin an output driven low. */
void ace2k_airflow_io_init(void);
/* Switches both fans on or off together. */
void ace2k_airflow_io_fans(bool on);
/* Bit 0: the pin of the left fan reads high. Bit 1: the pin of the right fan reads high. */
uint8_t ace2k_airflow_io_fans_read(void);
/* flap is 0 for the bottom, 1 for the rear. A flap out of range is ignored. */
void ace2k_airflow_io_flap(uint8_t flap, bool open_coil, bool close_coil);

#endif

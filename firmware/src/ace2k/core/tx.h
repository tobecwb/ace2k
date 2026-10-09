/* What: how a binding task sends a report or an event — through Klipper's transmit buffer,
 * told whether the frame was queued, so that a frame that did not fit is held for the next tick
 * instead of being dropped, and with room kept behind it for a command response.
 * How: ACE2K_SENDF(FMT, args...) is Klipper's sendf over ace2k_tx_try_sendf(), which runs the
 * build patch's console_try_sendf(): 0 when the encoded frame does not fit what is free in the
 * 96-byte buffer — Klipper decides on the frame's real length — or would leave fewer than
 * ACE2K_TX_RESPONSE_RESERVE_B free behind it.  The macro evaluates to bool: true, queued;
 * false, nothing was sent, try again next tick.  Task context, as sendf; no interrupt in this
 * image sends (the bindings send from tasks, Klipper's shutdown report from its main loop), so
 * the try needs no re-entrancy guard of its own.
 * The reserve: the responses to host commands go out through plain sendf and are dropped
 * against a full buffer; the bindings' frames could fill the buffer to the brim on their tick.
 * 24 B keeps room for the largest steady-state response with its acknowledgement — the clock
 * response (12 B framed) and the ack (5 B), a health response (14 B typical, 23 at most) and the
 * ack, a feed start response (10 B) and the ack, Klipper's stats (up to 22 B) — each alone; the
 * connect-time responses (the version, 25 B; the thresholds, 30 B; the identify chunks) are not
 * covered and rely on the transport's query retry, as does a lost start response — the start
 * is idempotent on that retry (feed.h).
 * Depends on: Klipper's command.h and board/misc.h (console_try_sendf, in the console this
 * firmware builds with: ace2k's Kconfig ties the motor and the feed to the RS-485 serial
 * console) — binding code, not host-compiled (this header leaves the tidy list as
 * ace2k_board/serial.h does). */
#ifndef ACE2K_TX_H
#define ACE2K_TX_H
#include <stdbool.h>
#include "command.h" /* struct command_encoder, _DECL_ENCODER */

/* The room a binding's frame leaves behind it, for a command response and its ack. */
#define ACE2K_TX_RESPONSE_RESERVE_B 24U

/* Klipper's sendf as a try: true when the frame was queued with the reserve still free. */
bool ace2k_tx_try_sendf(const struct command_encoder *ce, ...);

#define ACE2K_SENDF(FMT, args...) ace2k_tx_try_sendf(_DECL_ENCODER(FMT), ##args)

#endif

// Binding of the half-duplex turnaround.  It answers the read-only query of the counters, and the
// query and the clear of the longest silence between two valid frames.  The
// turnaround itself runs inside the serial driver.  ace2k_board/serial.c owns the instance and its
// timer, and decides when the line is raised.  This file only reports what the turnaround counted.
#include "ace2k_board/serial.h" // ace2k_serial_link_*, ACE2K_LINK_IDLE_US, ACE2K_LINK_LOST_MS
#include "command.h"            // DECL_COMMAND, sendf

// Klipper calls every DECL_COMMAND handler by name from generated code, so it is not static.
void ace2k_link_cmd_query(uint32_t *args)
{
    (void)args;
    uint32_t deferred = 0;
    uint32_t rearmed = 0;
    ace2k_serial_link_counters(&deferred, &rearmed);
    sendf("ace2k_link_state deferred=%u rearmed=%u idle_us=%u", deferred, rearmed,
          ACE2K_LINK_IDLE_US);
}
DECL_COMMAND(ace2k_link_cmd_query, "ace2k_link_query");

// The longest silence since boot or the last ace2k_link_gap_clear, beside the window that judges
// it, so the host needs no constant to compare the two.
void ace2k_link_cmd_gap_query(uint32_t *args)
{
    (void)args;
    sendf("ace2k_link_gap_state max_ms=%u lost_ms=%u", ace2k_serial_link_gap_ms(),
          ACE2K_LINK_LOST_MS);
}
DECL_COMMAND(ace2k_link_cmd_gap_query, "ace2k_link_gap_query");

void ace2k_link_cmd_gap_clear(uint32_t *args)
{
    (void)args;
    ace2k_serial_link_gap_clear();
}
DECL_COMMAND(ace2k_link_cmd_gap_clear, "ace2k_link_gap_clear");

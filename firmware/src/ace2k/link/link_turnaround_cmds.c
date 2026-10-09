// Binding of the half-duplex turnaround: the read-only query of its counters.  The turnaround
// itself runs inside the serial driver (ace2k_board/serial.c owns the instance and its timer:
// it decides when the line is raised); this file only reports what it counted.
#include "ace2k_board/serial.h" // ace2k_serial_link_counters, ACE2K_LINK_IDLE_US
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

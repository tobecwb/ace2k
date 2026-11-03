// Binding of the readers to SPI2 (PB13/PB14/PB15, mode 0), with NSS and RST as plain outputs
// (docs/hardware.md "RFID readers").  It runs in task context only.  RST is set high at init, and
// this image never writes it again.  A reader is reset through CommandReg, never through its RST
// pin.
#include "rfid/reader.h"
#include "rfid/reader_cmds.h"
#include "ace2k_board/args.h" // ACE2K_ARG_U8_MAX
#include "ace2k_board/pins.h"
#include "autoconf.h"            // CONFIG_ACE2K_RFID_READ
#include "board/gpio.h"          // gpio_out_setup/write, spi_setup/prepare/transfer
#include "command.h"             // DECL_COMMAND, sendf, DECL_CONSTANT_STR
#include "generic/armcm_timer.h" // udelay
#if CONFIG_ACE2K_RFID_READ
#include "rfid/watch_cmds.h" // a reader in use is not probed
#endif

DECL_CONSTANT_STR("RESERVE_PINS_ace2k_rfid", "PB13,PB14,PB15,PB12,PD10,PD13,PD12");

#define ACE2K_RFID_SPI_BUS  0U /* spi2_PB14_PB15_PB13 in Klipper's enumeration */
#define ACE2K_RFID_SPI_MODE 0U
#define ACE2K_RFID_SPI_RATE 1000000U

static const uint8_t nss_pin[ACE2K_RFID_READER_COUNT] = {
    ACE2K_PIN_RFID_A_NSS,
    ACE2K_PIN_RFID_B_NSS,
};
static const uint8_t rst_pin[ACE2K_RFID_READER_COUNT] = {
    ACE2K_PIN_RFID_A_RST,
    ACE2K_PIN_RFID_B_RST,
};
static struct gpio_out ace2k_rfid_nss[ACE2K_RFID_READER_COUNT];
static struct gpio_out ace2k_rfid_rst[ACE2K_RFID_READER_COUNT];
static struct spi_config ace2k_rfid_spi;
static struct ace2k_rfid_reader ace2k_rfid_reader_instance;
static bool ace2k_rfid_reader_ready;

static void nss(void *ctx, uint8_t reader, bool selected)
{
    (void)ctx;
    gpio_out_write(ace2k_rfid_nss[reader], selected ? 0 : 1); /* active low */
}

static void rst(void *ctx, uint8_t reader, bool running)
{
    (void)ctx;
    gpio_out_write(ace2k_rfid_rst[reader], running ? 1 : 0);
}

static void spi_xfer(void *ctx, uint8_t *buf, uint8_t len)
{
    (void)ctx;
    spi_prepare(ace2k_rfid_spi);
    spi_transfer(ace2k_rfid_spi, 1, len, buf);
}

static void delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    udelay(us);
}

static const struct ace2k_rfid_reader_ops board_ops = {
    .nss = nss,
    .rst = rst,
    .spi_xfer = spi_xfer,
    .delay_us = delay_us,
};

static void ensure_ready(void)
{
    if (ace2k_rfid_reader_ready) {
        return;
    }
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        ace2k_rfid_nss[r] = gpio_out_setup(nss_pin[r], 1);
        ace2k_rfid_rst[r] = gpio_out_setup(rst_pin[r], 1);
    }
    ace2k_rfid_spi = spi_setup(ACE2K_RFID_SPI_BUS, ACE2K_RFID_SPI_MODE, ACE2K_RFID_SPI_RATE);
    ace2k_rfid_reader_init(&ace2k_rfid_reader_instance, &board_ops, NULL);
    ace2k_rfid_reader_ready = true;
}

const struct ace2k_rfid_reader *ace2k_rfid_reader_binding(void)
{
    ensure_ready();
    return &ace2k_rfid_reader_instance;
}

struct ace2k_rfid_reader *ace2k_rfid_reader_binding_mut(void)
{
    ensure_ready();
    return &ace2k_rfid_reader_instance;
}

// A reader that the watch uses is left alone.  The watch uses it while it wants the field on, or
// while a job runs on it.  The soft reset of a probe would cut the field in the middle of a read.
// The last version of the reader stands.  Both paths that probe a reader come here, the health
// self-test and the query command, so they follow one rule.
int ace2k_rfid_reader_binding_probe(uint8_t reader)
{
    ensure_ready();
#if CONFIG_ACE2K_RFID_READ
    if (ace2k_rfid_binding_reader_busy(reader)) {
        return -ACE2K_EBUSY;
    }
#endif
    return ace2k_rfid_reader_probe(&ace2k_rfid_reader_instance, reader);
}

// The reply carries the outcome, and the host decides.  Version 0 means that the reset never
// completed.  Any other value but the expected one means an unexpected part.  A reader that the
// watch uses is not reset, and the reply carries the version read at its last soft reset.  A reader
// outside 0..1 is a bug in the host, not a condition of the reader.  Klipper's way to refuse a bad
// command is a shutdown with a message.  Klipper's generated dispatch declares every handler as
// void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_rfid_reader_cmd_query(uint32_t *args)
{
    uint8_t reader = ACE2K_ARG_U8_MAX(args[0], ACE2K_RFID_READER_COUNT - 1U,
                                      "ace2k_rfid_reader_query: reader out of range");
    (void)ace2k_rfid_reader_binding_probe(reader); /* a busy reader keeps its last version */
    sendf("ace2k_rfid_reader_state reader=%c version=%c", reader,
          ace2k_rfid_reader_instance.version[reader]);
}
DECL_COMMAND(ace2k_rfid_reader_cmd_query, "ace2k_rfid_reader_query reader=%c");

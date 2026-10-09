// Binding of the readers to SPI2 (PB13/PB14/PB15, mode 0) with NSS and RST as plain outputs
// (docs/hardware.md "RFID readers").  Task context only.  RST is set high at init and never
// written again in this image (a reader is reset through CommandReg, never through its RST pin).
#include "rfid/reader.h"
#include "rfid/reader_cmds.h"
#include "ace2k_board/pins.h"
#include "board/gpio.h"          // gpio_out_setup/write, spi_setup/prepare/transfer
#include "command.h"             // DECL_COMMAND, sendf, shutdown, DECL_CONSTANT_STR
#include "generic/armcm_timer.h" // udelay
#include "sched.h"               // sched_shutdown, behind shutdown()

DECL_CONSTANT_STR("RESERVE_PINS_ace2k_rfid", "PB13,PB14,PB15,PB12,PD10,PD13,PD12");

#define ACE2K_RFID_SPI_BUS  0U /* spi2_PB14_PB15_PB13 in Klipper's enumeration */
#define ACE2K_RFID_SPI_MODE 0U
#define ACE2K_RFID_SPI_RATE 1000000U

static const uint8_t ace2k_rfid_nss_pin[ACE2K_RFID_READER_COUNT] = { ACE2K_PIN_RFID_A_NSS,
                                                                     ACE2K_PIN_RFID_B_NSS };
static const uint8_t ace2k_rfid_rst_pin[ACE2K_RFID_READER_COUNT] = { ACE2K_PIN_RFID_A_RST,
                                                                     ACE2K_PIN_RFID_B_RST };
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

static const struct ace2k_rfid_reader_ops ace2k_rfid_board_ops = {
    .nss = nss, .rst = rst, .spi_xfer = spi_xfer, .delay_us = delay_us
};

static void ensure_ready(void)
{
    if (ace2k_rfid_reader_ready) {
        return;
    }
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        ace2k_rfid_nss[r] = gpio_out_setup(ace2k_rfid_nss_pin[r], 1);
        ace2k_rfid_rst[r] = gpio_out_setup(ace2k_rfid_rst_pin[r], 1);
    }
    ace2k_rfid_spi = spi_setup(ACE2K_RFID_SPI_BUS, ACE2K_RFID_SPI_MODE, ACE2K_RFID_SPI_RATE);
    ace2k_rfid_reader_init(&ace2k_rfid_reader_instance, &ace2k_rfid_board_ops, NULL);
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

int ace2k_rfid_reader_binding_probe(uint8_t reader)
{
    ensure_ready();
    return ace2k_rfid_reader_probe(&ace2k_rfid_reader_instance, reader);
}

// The reply carries the outcome: version 0 = the reset never completed, any other value but the
// expected one = an unexpected part; the host decides.  A reader outside 0..1 is a host bug,
// not a reader condition: Klipper's idiom for a bad command is a shutdown with a message.
// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_rfid_reader_cmd_query(uint32_t *args)
{
    uint8_t reader = (uint8_t)args[0];
    if (reader >= ACE2K_RFID_READER_COUNT) {
        shutdown("ace2k_rfid_reader_query: reader out of range");
    }
    (void)ace2k_rfid_reader_binding_probe(reader);
    // shutdown() does not return; cppcheck does not see through Klipper's static-string macro
    // to the noreturn declaration behind it, so it carries reader == 2 down to this index.
    // cppcheck-suppress arrayIndexOutOfBoundsCond
    sendf("ace2k_rfid_reader_state reader=%c version=%c", reader,
          ace2k_rfid_reader_instance.version[reader]);
}
DECL_COMMAND(ace2k_rfid_reader_cmd_query, "ace2k_rfid_reader_query reader=%c");

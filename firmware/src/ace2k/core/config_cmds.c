// Binding of the config record to the board flash, and of the factory calibration pages (read
// once, never written).  No policy here.
#include "core/config_cmds.h"
#include "ace2k_board/flash.h"
#include "board/irq.h" // irq_save, irq_restore
#include "command.h"   // DECL_COMMAND, sendf

static const struct ace2k_config_ops ace2k_config_board_ops = {
    .flash_read = ace2k_flash_read,
    .flash_erase_page = ace2k_flash_erase_page,
    .flash_program_word = ace2k_flash_program_word,
};

static struct ace2k_config ace2k_config_instance;
static bool ace2k_config_loaded;

struct ace2k_config *ace2k_config_binding(void)
{
    if (!ace2k_config_loaded) {
        ace2k_config_init(&ace2k_config_instance, &ace2k_config_board_ops, NULL);
        ace2k_config_loaded = true;
    }
    return &ace2k_config_instance;
}

static ace2k_config_store_veto_fn ace2k_config_store_veto;
static void *ace2k_config_store_veto_ctx;

int ace2k_config_binding_register_store_veto(ace2k_config_store_veto_fn fn, void *ctx)
{
    if (fn == NULL) {
        return -ACE2K_EINVAL;
    }
    if (ace2k_config_store_veto != NULL) {
        return -ACE2K_EFULL;
    }
    ace2k_config_store_veto_ctx = ctx;
    ace2k_config_store_veto = fn;
    return 0;
}

static volatile bool ace2k_config_storing;

// The dryer's rule 13 as one step: the veto asked and the store marked in progress under one interrupt
// bracket, so no tick can begin a lease between the two; the heat binding reads the mark at every
// tick and refuses a new lease until the store returns.
int ace2k_config_binding_store(void)
{
    struct ace2k_config *cfg = ace2k_config_binding(); // loaded before the bracket
    irqstatus_t flag = irq_save();
    bool vetoed =
        ace2k_config_store_veto != NULL && ace2k_config_store_veto(ace2k_config_store_veto_ctx);
    if (!vetoed) {
        ace2k_config_storing = true;
    }
    irq_restore(flag);
    if (vetoed) {
        return -ACE2K_EBUSY;
    }
    int rc = ace2k_config_store(cfg);
    ace2k_config_storing = false;
    return rc;
}

bool ace2k_config_binding_storing(void)
{
    return ace2k_config_storing;
}

static struct ace2k_factory_calibration ace2k_config_factory;
static bool ace2k_config_factory_loaded;

const struct ace2k_factory_calibration *ace2k_config_binding_factory(void)
{
    if (!ace2k_config_factory_loaded) {
        ace2k_config_factory_read(&ace2k_config_board_ops, NULL, &ace2k_config_factory);
        ace2k_config_factory_loaded = true;
    }
    return &ace2k_config_factory;
}

// page: 0 valid, 1 blank, 2 corrupt — the enum's order; calibration_source: 0 default, 1 factory.
void ace2k_config_cmd_state(uint32_t *args)
{
    (void)args;
    const struct ace2k_config *cfg = ace2k_config_binding();
    const struct ace2k_factory_calibration *fac = ace2k_config_binding_factory();
    sendf("ace2k_config_state_response page=%c calibration_source=%c", (uint8_t)cfg->source,
          (uint8_t)fac->source);
}
DECL_COMMAND(ace2k_config_cmd_state, "ace2k_config_state");

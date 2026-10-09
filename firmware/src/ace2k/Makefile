# ace2k sources.  Included from src/stm32/Makefile by the ace2k patch (one line).
# ACE2K_VERSION comes from firmware/Makefile on the command line.
ACE2K_VERSION ?= 0.0.0-dev

dirs-y += src/ace2k src/ace2k_board
dirs-y += src/ace2k/core src/ace2k/link src/ace2k/lane src/ace2k/feed src/ace2k/heat src/ace2k/dryer
dirs-y += src/ace2k/env src/ace2k/rfid
src-$(CONFIG_ACE2K) += ace2k_board/tick.c ace2k_board/flash.c ace2k/core/tx.c
src-$(CONFIG_ACE2K) += ace2k/core/guard.c ace2k/core/guard_cmds.c ace2k/core/config.c ace2k/core/config_cmds.c
src-$(CONFIG_ACE2K) += ace2k/link/linkproof.c ace2k/link/linkproof_cmds.c
src-$(CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485) += ace2k/link/link_turnaround.c
src-$(CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485) += ace2k/link/link_turnaround_cmds.c
src-$(CONFIG_ACE2K_LED) += ace2k/lane/led.c ace2k/lane/led_cmds.c
src-$(CONFIG_ACE2K_SENSORS) += ace2k_board/adc_scan.c ace2k/lane/sensors.c ace2k/lane/sensors_cmds.c
src-$(CONFIG_ACE2K_ENV) += ace2k/env/env.c ace2k/env/env_cmds.c
src-$(CONFIG_ACE2K_LANE) += ace2k_board/encoder.c ace2k_board/fg.c ace2k/lane/lane.c ace2k/lane/lane_cmds.c
src-$(CONFIG_ACE2K_MOTOR) += ace2k_board/motor.c
src-$(CONFIG_ACE2K_FEED) += ace2k/feed/feed.c ace2k/feed/feed_start.c ace2k/feed/feed_compare.c ace2k/feed/feed_judge.c
src-$(CONFIG_ACE2K_FEED) += ace2k/feed/feed_assist.c ace2k/feed/feed_load.c ace2k/feed/feed_snag.c
src-$(CONFIG_ACE2K_FEED) += ace2k/feed/feed_cmds.c
src-$(CONFIG_ACE2K_MAINS) += ace2k_board/zerocross.c ace2k/heat/mains.c ace2k/heat/mains_cmds.c
src-$(CONFIG_ACE2K_HEAT) += ace2k_board/airflow_io.c ace2k/heat/airflow.c ace2k/heat/airflow_cmds.c
src-$(CONFIG_ACE2K_HEAT) += ace2k_board/gate.c ace2k/heat/heat.c ace2k/heat/heat_cmds.c
src-$(CONFIG_ACE2K_DRYER) += ace2k/dryer/dryer.c ace2k/dryer/dryer_ctl.c ace2k/dryer/dryer_protect.c
src-$(CONFIG_ACE2K_DRYER) += ace2k/dryer/dryer_vent.c ace2k/dryer/dryer_log.c ace2k/dryer/dryer_cmds.c
src-$(CONFIG_ACE2K_RFID) += ace2k/rfid/reader.c ace2k/rfid/reader_cmds.c
src-$(CONFIG_ACE2K_RFID_READ) += ace2k/rfid/transceive.c ace2k/rfid/iso14443a.c ace2k/rfid/watch.c
src-$(CONFIG_ACE2K_RFID_READ) += ace2k/rfid/watch_cmds.c
src-$(CONFIG_ACE2K_HEALTH) += ace2k_board/sysinfo.c ace2k/core/health.c ace2k/core/health_cmds.c

# Our own objects only: the version define stays out of Klipper's compile lines, and our code is
# held to a stricter standard than the tree around it.  -Isrc/ace2k because the sources include
# one another subsystem-qualified ("core/util.h"), resolved from src/ace2k, as the host tests
# compile them.  Klipper's DECL_CTR_INT (every
# integer DECL_CONSTANT) stores its request text without the terminator by design; GCC 15's
# -Wextra objects to that, and Klipper's own build is -Wall only, so that one warning is off.
ACE2K_CFLAGS := -Isrc/ace2k -Wextra -Wundef -Werror -Wno-unterminated-string-initialization
$(OUT)src/ace2k/%.o: CFLAGS += -DACE2K_VERSION=\"$(ACE2K_VERSION)\" $(ACE2K_CFLAGS)
$(OUT)src/ace2k_board/%.o: CFLAGS += $(ACE2K_CFLAGS)

# The version reaches the image as a define on the compile line, not as a source, so make alone
# never sees it change: an incremental build after a `-dirty` one kept the old string in
# linkproof_cmds.o and the image answered ace2k_version with the previous build's name.  The
# stamp holds the version; its rule runs on every build (a phony prerequisite of its own, not
# borrowed from Klipper's Makefile) and rewrites the file only when the content differs — at
# recipe time, so `make -n`, menuconfig and clean touch nothing — and every object whose CFLAGS
# carry the define depends on it.
ACE2K_VERSION_STAMP := $(OUT)ace2k-version.stamp
.PHONY: FORCE
FORCE:
$(ACE2K_VERSION_STAMP): FORCE
	$(Q)mkdir -p $(OUT)
	$(Q)printf '%s' '$(ACE2K_VERSION)' | cmp -s - $@ 2>/dev/null || printf '%s' '$(ACE2K_VERSION)' > $@
$(patsubst %.c,$(OUT)src/%.o,$(filter ace2k/%,$(src-y))): $(ACE2K_VERSION_STAMP)

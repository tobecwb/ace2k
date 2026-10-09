# ace2k firmware build — wraps the Klipper build.
#
#   make build     reset the submodule to the pin, apply patches/ as ONE deterministic local
#                  commit (a dirty Klipper tree would stamp hostname+time into the dictionary),
#                  overlay src/ into klipper/src/, configure from config/, build, wrap the image
#                  for the unit's bootloader, reset the submodule again.
#   make klipper-reset  put the submodule back on the pin (after an interrupted build); refuses
#                  to discard uncommitted edits in klipper/ unless FORCE=1
#   make flag-check  build the release and prove what it must and must not carry (made from
#                  config/ace2k.config, the version constant, PC9 never drivable, the dryer, no
#                  bench commands)
#   make test      host tests of the core (cc) and of the tools (pytest)
#   make coverage  line coverage of the core under the host tests (built with clang)
#   make lint      formatter check, clang-tidy, cppcheck, ruff
#   make lint-selftest  clang-tidy on a fixture written to fail, proving the rules bite
#   make doctor    the toolchain check
#   make hooks     install the pre-commit hook (format-check on the staged tree) in the
#                  enclosing repository; replaces only a hook it wrote itself
#
# Edit only under src/, config/, patches/, tests/, tools/.  klipper/ is disposable.
# Requires the Arm GNU Toolchain (newlib); see scripts/doctor.sh.  GNU make 3.81 compatible.

ACE2K_ROOT    := $(abspath ..)
KLIPPER       := $(abspath klipper)
KLIPPER_PIN   := $(shell cat klipper.pin)
PATCHES       := $(abspath $(wildcard patches/*.patch))
CONFIG        := $(abspath config/ace2k.config)
BUILD         := $(abspath build)
PYTHON        ?= python3
ACE2K_VERSION ?= $(shell git -C $(ACE2K_ROOT) describe --tags --match 'v[0-9]*' --always --dirty 2>/dev/null | sed 's/^v//')
ACE2K_MAX_BYTES ?= 106496

# The Arm GNU Toolchain cask installs outside PATH on macOS; prefer it, else PATH.
ACE2K_CROSS ?= $(firstword $(patsubst %gcc,%,$(wildcard /Applications/ArmGNUToolchain/*/arm-none-eabi/bin/arm-none-eabi-gcc)) arm-none-eabi-)
CROSS_CC    := $(ACE2K_CROSS)gcc
TOOLBIN     := $(BUILD)/bin

# Sub-makes get the version explicitly: while the temporary commit exists the parent tree is
# "dirty" and re-evaluating `git describe` would change the name mid-build.
SUB          := $(MAKE) --no-print-directory ACE2K_VERSION=$(ACE2K_VERSION)
KLIPPER_MAKE := $(MAKE) -C $(KLIPPER) CROSS_PREFIX=$(ACE2K_CROSS) CPP="$(CROSS_CC) -E" \
                ACE2K_VERSION=$(ACE2K_VERSION)

# Deterministic identity for the temporary patch commit: same pin + same patch = same sha.
COMMIT_ENV := GIT_AUTHOR_DATE=2000-01-01T00:00:00Z GIT_COMMITTER_DATE=2000-01-01T00:00:00Z
COMMIT_ID  := -c user.name=ace2k-build -c user.email=build@ace2k.invalid -c commit.gpgsign=false

.PHONY: all build klipper-reset patch overlay config klipper-image doctor clean

all: build

# A failed step must not leave the submodule on the temporary commit (the superproject would show
# it as modified, a careless `git add -A` would record a sha that exists nowhere, and the next
# build would be named `-dirty`): the steps after the first reset run as one shell list whose
# failure resets the submodule before reporting.  An interrupted build is put back by
# `make klipper-reset`.
build:
	$(SUB) klipper-reset
	@$(SUB) patch && $(SUB) overlay && $(SUB) config && $(SUB) klipper-image && $(SUB) image || { \
	    echo "build: failed; resetting the submodule to the pin" >&2; $(SUB) klipper-reset FORCE=1; exit 1; }
	$(SUB) klipper-reset
	@echo "build: $(IMAGE)"

# Back to the pin, dropping the temporary patch commit.  Untracked files (the overlay symlinks,
# out/, .config) survive a hard reset.  Uncommitted changes to tracked files would not — they are
# someone's work in progress (see patches/README.md), so the reset refuses unless FORCE=1.
klipper-reset:
	@test -f $(KLIPPER)/Makefile || { echo "klipper submodule missing: git submodule update --init"; exit 1; }
	@if [ -z "$(FORCE)" ] && ! (git -C $(KLIPPER) diff --quiet && git -C $(KLIPPER) diff --cached --quiet); then \
	    echo "klipper-reset: klipper/ has uncommitted changes; turn them into a patch (patches/README.md) or discard them, or run with FORCE=1" >&2; exit 1; fi
	git -C $(KLIPPER) reset -q --hard $(KLIPPER_PIN)

patch:
	@for p in $(PATCHES); do git -C $(KLIPPER) apply --check $$p || exit 1; git -C $(KLIPPER) apply $$p; done
	$(COMMIT_ENV) git -C $(KLIPPER) $(COMMIT_ID) commit -q -am "ace2k build patch (temporary; applied and removed by firmware/Makefile)"

# Symlink our source directories into the Klipper tree and hide them from its git status.
overlay:
	@for d in ace2k ace2k_board; do \
	    if [ -d src/$$d ]; then ln -sfn $(abspath src/$$d) $(KLIPPER)/src/$$d; fi; done
	@if [ -f src/stm32/gd32f30x.c ]; then ln -sf $(abspath src/stm32/gd32f30x.c) $(KLIPPER)/src/stm32/gd32f30x.c; fi
	@excl=$$(git -C $(KLIPPER) rev-parse --path-format=absolute --git-path info/exclude); \
	    for e in src/ace2k src/ace2k_board src/stm32/gd32f30x.c; do \
	        grep -qx "$$e" $$excl 2>/dev/null || echo "$$e" >> $$excl; done

# olddefconfig regenerates Klipper's out/board link while parsing, i.e. from the answer file
# before it is completed; redo the link from the completed .config (make 3.81 compares mtimes
# in whole seconds and would not notice the newer .config).
config:
	cp $(CONFIG) $(KLIPPER)/.config
	$(KLIPPER_MAKE) olddefconfig
	$(KLIPPER_MAKE) create-board-link

# Klipper's scripts/check-gcc.sh calls a bare `readelf`; give it the cross one, by its resolved
# path (a bare prefix such as `arm-none-eabi-` has no directory part and the link would dangle).
klipper-image:
	mkdir -p $(TOOLBIN)
	@readelf="$$(command -v $(ACE2K_CROSS)readelf)"; \
	    [ -n "$$readelf" ] || { echo "klipper-image: $(ACE2K_CROSS)readelf not found; see scripts/doctor.sh"; exit 1; }; \
	    ln -sf "$$readelf" $(TOOLBIN)/readelf
	PATH="$(TOOLBIN):$$PATH" $(KLIPPER_MAKE)

IMAGE := $(BUILD)/ace2k-$(ACE2K_VERSION).bin

.PHONY: image test-py

image:
	mkdir -p $(BUILD)
	$(PYTHON) tools/mkimage.py $(KLIPPER)/out/klipper.bin $(IMAGE) --max-bytes $(ACE2K_MAX_BYTES)
	cp $(KLIPPER)/out/klipper.elf $(BUILD)/ace2k-$(ACE2K_VERSION).elf
	cp $(KLIPPER)/out/klipper.dict $(BUILD)/ace2k-$(ACE2K_VERSION).dict
	$(ACE2K_CROSS)size $(KLIPPER)/out/klipper.elf

.PHONY: flag-check
FLAG_ELF     := $(BUILD)/ace2k-$(ACE2K_VERSION).elf
FLAG_DICT    := $(BUILD)/ace2k-$(ACE2K_VERSION).dict
FLAG_RELEASE := $(abspath config/ace2k.config)
# The checks on the release image.  flag-check depends on build, so `make flag-check` builds the
# image it checks and `make build flag-check` builds it once.  The build must have been made from
# config/ace2k.config: every CONFIG_ACE2K* line of it in klipper/.config, and no other
# CONFIG_ACE2K*=y there.  The image's .elf and .dict must exist under the build's version, and
# the dictionary must carry the ACE2K_VERSION constant with that same value — the constant is
# present and matches the image's name.  By the dryer's safety rules, PC9 (the cutout's
# latch reset) appears in the dictionary only inside the value of a RESERVE_PINS_* constant —
# reserved at most once, so the host cannot drive it — and in no source under src/ outside such a
# pin-list string (comments stripped by the compiler's preprocessor, which keeps every #define of
# both arms of an #if and would warn of a redefinition: -w); the gate's header and pin are
# confined to the gate and its binding.  Then: ace2k_flap_pulse is the two-argument command (the
# pulse length is the compiled one), the dryer is present, and no retired bench command is in the
# dictionary (no ace2k_heat_* name, no ace2k_lane_ move/stop/speed_set/duty/motion, no
# ace2k_rfid_trace).
flag-check: build
	@test -f $(FLAG_ELF) && test -f $(FLAG_DICT) || \
	    { echo "flag-check: the build left no $(FLAG_ELF) or $(FLAG_DICT)"; exit 1; }
	@want=$$(grep -E '^CONFIG_ACE2K[A-Z0-9_]*=' $(FLAG_RELEASE) | LC_ALL=C sort); \
	    have=$$(grep -E '^CONFIG_ACE2K[A-Z0-9_]*=y' $(KLIPPER)/.config | LC_ALL=C sort); \
	    missing=$$(printf '%s\n' "$$want" | while read -r l; do \
	        grep -qxF "$$l" $(KLIPPER)/.config || echo "$$l"; done); \
	    if [ -n "$$missing" ] || [ "$$want" != "$$have" ]; then \
	        echo "flag-check: the build was not made from config/ace2k.config"; exit 1; fi
	@grep -q '"ACE2K_VERSION": *"$(ACE2K_VERSION)"' $(FLAG_DICT) || \
	    { echo "flag-check: the release dictionary lacks ACE2K_VERSION"; exit 1; }
	@d=$(FLAG_DICT); \
	    if sed -E 's/"RESERVE_PINS_[A-Za-z0-9_]*": *"[^"]*"//g' $$d | grep -E -q 'PC9([^0-9]|$$)'; then \
	        echo "flag-check: PC9 outside a pin reservation in the release dictionary"; exit 1; fi; \
	    n=$$(grep -E -o '"RESERVE_PINS_[A-Za-z0-9_]*": *"[^"]*"' $$d | grep -E -c 'PC9([^0-9]|$$)' || true); \
	    if [ "$$n" -gt 1 ]; then echo "flag-check: PC9 reserved $$n times in the release dictionary"; exit 1; fi; \
	    for f in $$(find src -name '*.[ch]' | LC_ALL=C sort); do \
	        code=$$($(CROSS_CC) -w -fpreprocessed -dD -E -P -x c $$f) || exit 1; \
	        if printf '%s\n' "$$code" | grep -E -q "(ACE2K_PIN|GPIO)\( *'C' *, *9 *\)"; then \
	            echo "flag-check: PC9 named as a pin in $$f"; exit 1; fi; \
	        bad=$$(printf '%s\n' "$$code" | grep -E 'PC9([^0-9]|$$)' | \
	            grep -E -v '^[[:space:]]*(DECL_CONSTANT_STR\("RESERVE_PINS_[A-Za-z0-9_]+",[[:space:]]*)?"[A-Z0-9,]*PC9[A-Z0-9,]*"' || true); \
	        if [ -n "$$bad" ]; then echo "flag-check: PC9 in $$f outside a pin reservation:"; echo "$$bad"; exit 1; fi; \
	        if printf '%s\n' "$$code" | grep -E -q 'PC9([^0-9]|$$)' && \
	            ! printf '%s\n' "$$code" | grep -q 'DECL_CONSTANT_STR("RESERVE_PINS_'; then \
	            echo "flag-check: PC9 in $$f, which declares no pin reservation"; exit 1; fi; done; \
	    inc=$$(grep -rl '#include "ace2k_board/gate.h"' src/ace2k src/ace2k_board | LC_ALL=C sort | tr '\n' ' '); \
	    if [ "$$inc" != "src/ace2k/heat/heat_cmds.c src/ace2k_board/gate.c " ]; then \
	        echo "flag-check: gate.h included by: $$inc"; exit 1; fi; \
	    pin=$$(grep -rl 'ACE2K_PIN_TRIAC_GATE' src | LC_ALL=C sort | tr '\n' ' '); \
	    if [ "$$pin" != "src/ace2k_board/gate.c src/ace2k_board/pins.h " ]; then \
	        echo "flag-check: the triac pin named in: $$pin"; exit 1; fi; \
	    echo "flag-check: PC9 nowhere"
	@d=$(FLAG_DICT); \
	    grep -q '"ace2k_flap_pulse flap=%c open=%c"' $$d || \
	        { echo "flag-check: the release's ace2k_flap_pulse is not the two-argument command"; exit 1; }; \
	    grep -q '"ace2k_dryer_start target_c=%c minutes=%hu"' $$d || \
	        { echo "flag-check: the release lacks the dryer"; exit 1; }; \
	    retired='"ace2k_(heat_[a-z_]*|lane_(move|stop|speed_set|duty|motion)[a-z_]*|rfid_trace)[" ]'; \
	    names=$$(grep -E -o "$$retired" $$d || true); \
	    if [ -n "$$names" ]; then \
	        echo "flag-check: retired bench commands in the release:"; echo "$$names"; exit 1; fi; \
	    echo "flag-check: release OK"

test-py:
	$(PYTHON) -m pytest -q tests/python

doctor:
	@sh ../scripts/doctor.sh

.PHONY: hooks
# Replaces only a hook this target wrote before (scripts/pre-commit carries the exact marker
# "ace2k pre-commit (public)"); any other hook, including one that runs more checks, is kept.
hooks:
	@hook="$$(git rev-parse --git-path hooks)/pre-commit" && prefix="$$(git rev-parse --show-prefix)" && \
	    if [ -e "$$hook" ] && ! grep -qF 'ace2k pre-commit (public)' "$$hook"; then \
	        echo "hooks: $$hook exists and was not written by make hooks; not overwriting"; exit 1; fi && \
	    sed "s|^prefix=ACE2K_FIRMWARE_PREFIX$$|prefix=$${prefix%/}|" ../scripts/pre-commit > "$$hook" && chmod +x "$$hook" && \
	    echo "hooks: installed $$hook"

clean:
	rm -rf $(BUILD) $(TESTDIR)
	-$(SUB) klipper-reset
	rm -rf $(KLIPPER)/out $(KLIPPER)/.config $(KLIPPER)/.config.old

TESTDIR   := tests/build
# core/tx.c is the binding layer's (it calls Klipper's console), not the core's: kept off the host build
CORE_SRC  := $(filter-out %_cmds.c src/ace2k/core/tx.c, $(wildcard src/ace2k/*/*.c))
TEST_SRC  := $(wildcard tests/*/test_*.c)
TEST_BIN  := $(patsubst tests/%.c,$(TESTDIR)/%,$(TEST_SRC))
HOSTCC    ?= cc
HOSTFLAGS := -std=c11 -g -O0 -Wall -Wextra -Werror -Wshadow -Wstrict-prototypes -Wmissing-prototypes \
             -Wdouble-promotion -Wundef -Wvla -Isrc -Isrc/ace2k -Itests

.PHONY: test test-c

$(TESTDIR):
	mkdir -p $@

$(TESTDIR)/%: tests/%.c $(CORE_SRC) $(wildcard src/ace2k/*/*.h src/ace2k_board/*.h) \
              $(wildcard tests/*.h) | $(TESTDIR)
	@mkdir -p $(@D)
	$(HOSTCC) $(HOSTFLAGS) $< $(CORE_SRC) -o $@ -lm

test-c: $(TEST_BIN)
	@for t in $(TEST_BIN); do ./$$t || exit 1; done

test: test-c test-py

# The coverage flags are clang's, so the coverage build always uses clang (the host cc is gcc on
# most Linux systems).  On macOS Apple clang's own llvm-cov (via xcrun) must read the profiles;
# on Linux the distribution's llvm-cov pairs with clang from the same packages.
COVCC         ?= clang
LLVM_COV      ?= $(shell if command -v xcrun >/dev/null 2>&1; then echo "xcrun llvm-cov"; else echo llvm-cov; fi)
LLVM_PROFDATA ?= $(shell if command -v xcrun >/dev/null 2>&1; then echo "xcrun llvm-profdata"; else echo llvm-profdata; fi)
COVFLAGS      := -fprofile-instr-generate -fcoverage-mapping

.PHONY: coverage
coverage: | $(TESTDIR)
	@bins=""; for t in $(TEST_SRC); do n=$$(basename $$t .c); \
	    $(COVCC) $(HOSTFLAGS) $(COVFLAGS) $$t $(CORE_SRC) -o $(TESTDIR)/cov_$$n -lm || exit 1; \
	    LLVM_PROFILE_FILE=$(TESTDIR)/cov_$$n.profraw ./$(TESTDIR)/cov_$$n >/dev/null || exit 1; \
	    bins="$$bins $(TESTDIR)/cov_$$n"; done; \
	[ -n "$$bins" ] || { echo "coverage: no tests"; exit 1; }; \
	$(LLVM_PROFDATA) merge -sparse $(TESTDIR)/cov_*.profraw -o $(TESTDIR)/coverage.profdata; \
	set -- $$bins; first=$$1; shift; objs=""; for b in "$$@"; do objs="$$objs -object $$b"; done; \
	$(LLVM_COV) report $$first $$objs -instr-profile=$(TESTDIR)/coverage.profdata \
	    -ignore-filename-regex='tests/' src/ace2k

LLVM_BIN     := $(shell brew --prefix llvm 2>/dev/null)/bin
CLANG_FORMAT ?= $(shell command -v clang-format 2>/dev/null || echo $(LLVM_BIN)/clang-format)
FORMAT_SRC   := $(filter-out tests/lint-fixtures/%, $(wildcard src/ace2k/*/*.c src/ace2k/*/*.h \
                  src/ace2k_board/*.c src/ace2k_board/*.h src/stm32/*.c tests/*/*.c tests/*.h))
PY_SRC       := firmware/tools firmware/tests/python klippy scripts

.PHONY: format format-check

format:
	$(CLANG_FORMAT) -i $(FORMAT_SRC)
	cd .. && ruff format $(PY_SRC)

format-check:
	@$(CLANG_FORMAT) --dry-run -Werror $(FORMAT_SRC) && echo "clang-format: OK"
	@cd .. && ruff format --check $(PY_SRC)
	@echo "format-check: OK"

CLANG_TIDY   ?= $(shell command -v clang-tidy 2>/dev/null || echo $(LLVM_BIN)/clang-tidy)
TIDY_SRC     := $(CORE_SRC) $(TEST_SRC)
# The board headers are pure declarations (standard headers only) that tests/lane/test_pins.c
# includes, so they are checked the same way; a header that needs a Klipper header cannot be
# listed here — serial.h includes autoconf.h for the link_ok helper, tx.h command.h for the
# encoder — and the cross build and cppcheck check those.
TIDY_HDR     := $(filter-out src/ace2k_board/serial.h src/ace2k/core/tx.h, \
                  $(wildcard src/ace2k/*/*.h src/ace2k_board/*.h))
TIDY_FLAGS   := -- -std=c11 -I$(abspath src) -I$(abspath src/ace2k) -I$(abspath tests)
CPPCHECK_SRC := $(wildcard src/ace2k/*/*.c src/ace2k_board/*.c src/stm32/*.c)

.PHONY: lint tidy cppcheck ruff lint-selftest

# A translation unit carries its own directory's configuration into every header it includes, so
# the core headers are also checked on their own: reached only from tests/, they would otherwise
# get the tests' relaxed rules.
tidy:
	$(CLANG_TIDY) --quiet $(TIDY_SRC) $(TIDY_FLAGS)
	$(CLANG_TIDY) --quiet $(TIDY_HDR) $(TIDY_FLAGS) -x c
	@echo "clang-tidy: OK"

cppcheck:
	cppcheck --quiet --error-exitcode=1 --enable=warning,style,performance,portability \
	    --inline-suppr --std=c11 --suppress=missingIncludeSystem --suppress=unmatchedSuppression \
	    -I src -I src/ace2k -I klipper/src -I klipper/out $(CPPCHECK_SRC)
	@echo "cppcheck: OK"

ruff:
	cd .. && ruff check $(PY_SRC)
	@echo "ruff: OK"

lint: format-check tidy cppcheck ruff
	@echo "lint: OK"

# The tidy configuration must actually bite: run it on the fixture and require the three
# findings the fixture was written for.  The project config is named explicitly because the
# fixture lives under tests/, whose own .clang-tidy relaxes the magic-number rule.
lint-selftest:
	@out=$$($(CLANG_TIDY) --quiet --config-file=.clang-tidy tests/lint-fixtures/bad_style.c -- -std=c11 2>&1); \
	    for check in readability-magic-numbers readability-identifier-naming readability-function-cognitive-complexity; do \
	        echo "$$out" | grep -q "$$check" || { echo "lint-selftest: $$check did not fire"; echo "$$out"; exit 1; }; done; \
	    echo "lint-selftest: OK (tidy reported $$(echo "$$out" | grep -c 'error:') findings on the fixture)"

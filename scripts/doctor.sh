#!/bin/sh
# Check the host toolchain that the ace2k build, tests and lints need. Exit with 1 if anything is
# missing.
# Klipper needs a cross toolchain with newlib. The file sched.c includes <setjmp.h>, and the link
# uses libc_nano. The Homebrew formula arm-none-eabi-gcc comes without newlib. The Arm GNU
# Toolchain cask (gcc-arm-embedded) comes with it.
set -u
status=0
llvm_bin=$(brew --prefix llvm 2>/dev/null)/bin
PATH="$PATH:$llvm_bin"

report() { # arguments: status name detail
    printf '  %-8s %-18s %s\n' "$1" "$2" "$3"
}
need() { # arguments: name command hint
    if command -v "$2" >/dev/null 2>&1; then report ok "$1" "$(command -v "$2")"
    else report MISSING "$1" "install: $3"; status=1; fi
}

echo "ace2k doctor"

# Cross compiler. Prefer the Arm cask and fall back to PATH. Newlib is required either way.
# To detect newlib, ask the compiler itself to find <setjmp.h>. This works with any layout of the
# distribution.
cross=$(ls -d /Applications/ArmGNUToolchain/*/arm-none-eabi/bin/arm-none-eabi-gcc 2>/dev/null | head -1)
[ -n "$cross" ] || cross=$(command -v arm-none-eabi-gcc 2>/dev/null)
if [ -z "$cross" ]; then
    report MISSING arm-none-eabi-gcc "brew install --cask gcc-arm-embedded"; status=1
elif ! echo '#include <setjmp.h>' | "$cross" -E -x c - >/dev/null 2>&1; then
    report MISSING arm-none-eabi-gcc "$cross has no newlib (no setjmp.h): brew install --cask gcc-arm-embedded (Debian/Ubuntu: apt install libnewlib-arm-none-eabi)"; status=1
else
    report ok arm-none-eabi-gcc "$cross ($("$cross" -dumpversion), newlib present)"
fi

need "host cc" cc "Xcode command line tools (macOS) / build-essential (Linux)"
need python3 python3 "brew install python"
need clang-format clang-format "brew install llvm  (keg-only: found via brew --prefix llvm)"
need clang-tidy clang-tidy "brew install llvm"
need cppcheck cppcheck "brew install cppcheck"
need ruff ruff "brew install ruff"
# Check pytest the way `make test-py` runs it, with `python3 -m pytest`. So the module must import
# from python3.
if pytest_version=$(python3 -c 'import pytest; print(pytest.__version__)' 2>/dev/null); then
    report ok pytest "python3 -m pytest ($pytest_version)"
else
    report MISSING pytest "install it into that python3 (Debian/Ubuntu: apt install python3-pytest; elsewhere: python3 -m pip install pytest)"; status=1
fi

# The Klipper submodule, if it is present. Report its commit and whether it matches the pin.
here=$(cd "$(dirname "$0")/.." && pwd)
if [ -f "$here/firmware/klipper.pin" ]; then
    pin=$(cat "$here/firmware/klipper.pin")
    if [ -f "$here/firmware/klipper/Makefile" ]; then
        head=$(git -C "$here/firmware/klipper" rev-parse HEAD 2>/dev/null)
        if [ "$head" = "$pin" ]; then report ok klipper "$pin"
        else report WARN klipper "HEAD $head differs from the pin $pin — make build resets it; if you moved the pin, git add firmware/klipper"; fi
    else
        report MISSING klipper "git submodule update --init  (from the repository root)"; status=1
    fi
fi

[ $status -eq 0 ] && echo "doctor: OK" || echo "doctor: something is missing"
exit $status

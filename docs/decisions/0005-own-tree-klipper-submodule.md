# 0005 — Own tree with Klipper as a pinned submodule and one patch

Date: 2026-09-14 · Status: accepted

## Context
Klipper has no plugin mechanism for `src/`. A new processor needs an entry in `src/stm32/Kconfig`
and a `-mcpu` line in its Makefile, and our sources need a `src-y` hook. Some change to Klipper's
own files is unavoidable.

## Decision
The repository contains only our files. Klipper is a git submodule pinned to a commit
(`firmware/klipper.pin`). `make build` does this:
1. resets the submodule to the pin;
2. applies one small patch (`firmware/patches/`, target under 100 lines) as a deterministic
   temporary commit;
3. symlinks `firmware/src/…` into `klipper/src/`;
4. configures from `firmware/config/ace2k.config` and builds;
5. wraps the image for the unit's bootloader (`firmware/tools/mkimage.py`);
6. resets the submodule again.

## Consequences
- Readers see 100 % our code.
- Updating Klipper means moving the pin and re-checking the patch.
- The temporary commit keeps Klipper's version string clean. A dirty tree makes Klipper stamp the
  build host's name and time into the dictionary.
- Nothing under `firmware/klipper/` is ever edited by hand.

## Alternatives rejected
A full fork of Klipper as the project repository: 99 % foreign code, "ours" only visible as a
diff, and a rebase per upstream update.

# 0008 — Credits and sourcing policy

Date: 2026-09-14 · Status: accepted

## Context
The project rests on published work by named people and on measurements made on real units. A
firmware whose constants and pin maps cannot be traced to a source is not reviewable. A project
that does not credit the work it builds on does not deserve contributors.

## Decision
Everything in this tree (code, comments, docs) cites only measurements and published sources:
- every physical constant names where it was measured (`docs/hardware.md`);
- every protocol fact points at [hakimio](https://github.com/hakimio)'s published schema or at
  observed behaviour of the original firmware;
- every tag layout or key names the project it came from.

Credits go to `docs/credits.md` and to the header of every file that derives from a source. A tag
layout or a key is a fact and needs a credit. Code ported from a project needs a
GPL-3.0-compatible licence and keeps its original header. Values that are initial guesses are
marked as such and re-measured on the bench.

## Consequences
- Constants that cannot yet be traced to a measurement enter as marked initial values, with an
  open item in the module's documentation.
- Fixtures use synthetic tag UIDs.
- Per-unit identifiers never enter the repository.
- `docs/credits.md` is reviewed with every pull request that adds a source.

## Alternatives rejected
- Credits scattered in file headers only: easy to lose.
- Unsourced constants with a "works for me" note: unreviewable.

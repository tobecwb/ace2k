"""The flasher's run state and the steps every route shares: its waits, Klipper's stop and
start, the plan and its question, the opening of the unit's port, and the calibration file (saved
before a write to the factory firmware, and again after it).

route.py (the way into recovery) and verify.py (the check after the write) build on FlasherBase,
and flasher.py's Flasher joins them with the command line and the flow.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from . import host  # host.port_holders, looked up at the call: one place for the tests to patch
from .host import RESTORE_COMMAND, SERVICE_TIMEOUT_S
from .wire import (
    RefusedError,
    Seen,
    Unit,
    calibration_files,
    calibration_text,
    count_units,
    parse_calibration,
    read_calibration,
    read_uid,
)

if TYPE_CHECKING:
    from collections.abc import Callable
    from typing import Any, Protocol

    from .host import MoonrakerProto, Service
    from .image import Image
    from .wire import ClockProto, LinkProto

    class OpenLinkProto(Protocol):
        """Opens the unit's port at a baud rate: SerialLink, or a simulated host in a test."""

        def __call__(self, port: str, baud: int = ...) -> LinkProto: ...


# --- Timing: starting values.  The timeouts of single exchanges with the unit are in wire.py. --
VERDICT_WAIT_S = 15.0  # recovery copies the image, then starts it
VERDICT_TRIES = 5
# After ACEK_RESTORE_STOCK: Klipper leaves `ready` about 4-6 s after the unit resets
RESTORE_WAIT_S = 15.0
REFUSAL_SETTLE_S = (
    6.0  # after a refusal (HTTP error or `!!` line) while ready, a reset may follow in 4-6 s
)
STOP_WAIT_S = 15.0  # after Klipper is stopped: its process lets go of the port
ACE2K_WAIT_S = 90.0  # after Klipper is started: ace2k reports its version
FACTORY_WAIT_S = 60.0
# the factory firmware that took an update resets about 2 s after the finish
OLD_FIRMWARE_WAIT_S = 15.0
# The number of polls in a row with no answer that means the old firmware is gone.  That takes
# about 3 s, less than the time the bootloader needs to copy the image and start the new
# firmware.
SILENT_POLLS = 2
STATE_POLL_S = 0.5
PROVE_WAIT_S = 30.0  # asking the new image for its version (the link proof), retries included


def close_quietly(link: LinkProto) -> None:
    """Close the unit's port.

    A port that fails to close is already gone, because the converter in the cable disappeared.
    Nothing is left to do with it, and the flow goes on.
    """
    with contextlib.suppress(OSError):
        link.close()


# --- The flow's shared state and steps ------------------------------------------------------
RESET_NOTE = (
    "  {name} had already reset the unit into the bootloader's recovery (it stays "
    "there and accepts an update).  {klipper} until "
    "ace2k is written again: split the chain or check the port, then run the tool again "
    "(recovery is identified as such)."
)
# Over the cable, until recovery answers: the unit may not have reset
CABLE_NOTE = (
    "  {name} was sent, but the unit may still run ace2k: a unit that turns busy withdraws it, "
    "and its answer can be lost.  Nothing was written.  Look at the LEDs (all four strobing "
    "fast is recovery) and run the tool again: it identifies either."
)
# The console may lag behind the script's answer.  This is how long the tool looks for the
# unknown-command line.
UNKNOWN_SETTLE_S = 2.0


@dataclass
class Deps:
    """Everything the flow uses from the system.  A test passes simulated ones."""

    open_link: OpenLinkProto
    moonraker: MoonrakerProto
    clock: ClockProto
    run: Callable[..., subprocess.CompletedProcess[Any]]  # subprocess.run
    which: Callable[[str], str | None]  # shutil.which
    proc: Path
    by_id: Path
    initd: Path
    ask: Callable[[str], str]  # input
    out: Callable[[str], None]  # print
    now: Callable[[], datetime.datetime]  # datetime.datetime.now


class FlasherBase:
    """One run of the tool: its state, and the steps that the routes and the checks share."""

    port: str  # the unit's port, once the flow has resolved it
    image: Image  # the ace2k image being checked, kept so the messages can name its waiting time

    def __init__(self, args: argparse.Namespace, deps: Deps) -> None:
        self.args, self.d = args, deps
        self.out = deps.out
        self.stopped = False  # this run stopped Klipper
        self.restart_ok = True  # Klipper may be started again at exit
        self.service: Service | None = None
        self.link: LinkProto | None = None  # the unit's port, while it is open
        self.update_started = False
        self.written = False  # the update was written whole and its finish sent
        # What ran on the unit when the update started: the factory firmware or recovery
        self.target: Seen | None = None
        self.start_tried = False  # the flow already tried a start and reported its result
        self.reset = False  # the port was opened after the restore reset the unit
        # touched: something was done.  Klipper was stopped, the restore command was sent, or the
        # update started.  Until then, a refusal exits with EXIT_UNCHANGED.
        # unit_touched: only the last two, the ones that change the unit.  Until then, a run
        # where nothing answers exits with EXIT_NO_ANSWER.
        self.touched = False
        self.unit_touched = False
        # The name of the restore command: _restore_choice sets it from Klipper's command list,
        # and an old host module may change it to the old name.
        self.restore_name = RESTORE_COMMAND
        self.unit_name: str | None = None  # the [ace2k <name>] of the unit, once Klipper says
        self.by_cable = False  # ace2k_bootloader_enter was sent; recovery has not answered yet

    def _wait(self, seconds: float, done: Callable[[], bool]) -> bool:
        """Poll done until it is true or the time runs out.  Return whether it became true."""
        deadline = self.d.clock.now() + seconds
        while True:
            if done():
                return True
            if self.d.clock.now() >= deadline:
                return False
            self.d.clock.sleep(STATE_POLL_S)

    def _stop_klipper(self, port: str, after_restore: bool = False) -> None:
        """Stop Klipper's service, then wait until nothing holds port open.

        after_restore is True when the restore command already reset the unit.  A stop that
        fails, hangs or is interrupted is refused, with what to do next.
        """
        assert self.service is not None  # every route that stops Klipper refuses without one
        cmd = shlex.join(self.service.stop)
        try:
            rc = getattr(self.d.run(self.service.stop, timeout=SERVICE_TIMEOUT_S), "returncode", 1)
            how = f"exit {rc}"
        except OSError as e:
            rc, how = 1, str(e)
        except subprocess.TimeoutExpired:
            # The command ran part of the way.  Klipper's state is unknown, so this is a failure.
            self.touched = True
            tail = self._unknown_stop_tail(after_restore, hung=True)
            raise RefusedError(
                f"`{cmd}` did not finish within {SERVICE_TIMEOUT_S:.0f} s and was ended: "
                f"Klipper may or may not be stopped.{tail}"
            ) from None
        except KeyboardInterrupt:  # Ctrl-C: systemctl may already have passed the stop on
            self.touched = True
            tail = self._unknown_stop_tail(after_restore, hung=False)
            raise RefusedError(
                f"`{cmd}` was interrupted: Klipper may or may not be stopped.{tail}"
            ) from None
        if rc != 0:
            tail = "." + self._reset_note() if after_restore else "; nothing changed"
            raise RefusedError(f"`{cmd}` failed ({how}): Klipper was not stopped{tail}")
        self.stopped = self.touched = True
        if not self._wait(STOP_WAIT_S, lambda: not host.port_holders(port, self.d.proc)):
            raise RefusedError(f"Klipper was stopped but {port} is still held open")

    def _unknown_stop_tail(self, after_restore: bool, hung: bool) -> str:
        """Return what to do after a stop command that did not finish.  Klipper may or may not
        run.

        hung is True when the command was ended after the time limit, and False when it was
        interrupted with Ctrl-C.
        """
        assert self.service is not None  # only a stop this run tried ends here
        if after_restore:
            return (
                f"  {self.restore_name} had already reset the unit into the bootloader's recovery "
                "(it stays there and accepts an update).  Stop Klipper by hand, then run the tool "
                "again: it finds recovery and writes the image."
            )
        start = f"  If Klipper does not run now, start it with `{shlex.join(self.service.start)}`."
        if not hung:
            return start[:-1] + ", then run the tool again."
        stop = shlex.join(self.service.stop)
        return f"{start}  Find out why `{stop}` hangs before you run the tool again."

    def _start_klipper(self) -> bool:
        """Start Klipper's service.  Return False, and say what to do, when the start fails."""
        assert self.service is not None  # Klipper is started only after this run stopped it
        cmd = shlex.join(self.service.start)
        try:
            rc = getattr(self.d.run(self.service.start, timeout=SERVICE_TIMEOUT_S), "returncode", 1)
            how = f"exit {rc}"
        except OSError as e:
            rc, how = 1, str(e)
        except subprocess.TimeoutExpired:  # it may still start, so say so and what to do
            self.out(
                f"Klipper may not have started (`{cmd}` did not finish within "
                f"{SERVICE_TIMEOUT_S:.0f} s): run it again or check Klipper."
            )
            return False
        if rc != 0:
            self.out(f"Klipper did NOT start ({how}): run {cmd}")
            return False
        self.stopped = False
        return True

    def _reset_note(self) -> str:
        """Return the note added to a refusal after the restore reset the unit.

        The note says that Klipper is being started again only when this run stopped it.  Over
        the cable, until recovery answers, it says the unit may still run ace2k instead.
        """
        if self.by_cable:
            return CABLE_NOTE.format(name=self.restore_name)
        klipper = (
            "Klipper is being started again but will not connect"
            if self.stopped and self.restart_ok
            else "Klipper will not connect"
        )
        return RESET_NOTE.format(name=self.restore_name, klipper=klipper)

    def _plan(self, image: Image, port: str, found: str, steps: list[str]) -> None:
        """Print what was found and the steps to come, then ask for confirmation.

        With --yes it does not ask.  Any answer but y or yes is refused, and nothing changes.
        """
        self.out(f"Unit on {port}: {found}")
        self.out(
            f"Image: {image.kind} {image.version}, {len(image.data)} bytes, "
            f"CRC 0x{image.crc:04X}, announced as {image.announce!r}"
        )
        self.out("Steps:")
        for s in steps:
            self.out(f"  - {s}")
        if self.args.yes:
            return
        try:
            answer = self.d.ask("Proceed? [y/N] ")
        except EOFError:
            answer = ""
        if answer.strip().lower() not in ("y", "yes"):
            raise RefusedError("Not confirmed; nothing changed.")

    def _open_unit(self, port: str, reset: bool = False) -> Unit:
        """Open the unit's port.

        reset is True when the restore command already reset the unit.  Then a refusal says
        that the unit is in recovery.
        """
        if host.port_holders(port, self.d.proc):
            why = f"{port} is held open by another program"
            raise RefusedError(
                why + "." + self._reset_note() if reset else why + "; nothing changed"
            )
        self.reset = reset
        try:
            self.link = self.d.open_link(port)
        except RefusedError as e:
            if not reset:
                raise
            raise RefusedError(f"{e}." + self._reset_note()) from None
        return Unit(self.link, self.d.clock)

    def _one_unit(self, unit: Unit, port: str, reset: bool = False) -> None:
        """Refuse unless exactly one unit answers DISCOVER on port.

        reset is True when the restore already reset the unit, so the refusal says so.
        """
        n = count_units(unit)
        if n > 1:
            why = (
                f"{n} units answered on {port}: this tool flashes one unit per port "
                "(a daisy chain is not supported)"
            )
        elif n == 0:
            why = f"No unit answered DISCOVER on {port}"
        else:
            return
        if reset:
            raise RefusedError(why + "." + self._reset_note())
        raise RefusedError(why + "; nothing changed")

    def _after_steps(self, image: Image, tool_stops: bool) -> list[str]:
        """Return the plan's last steps, the ones after the write.

        tool_stops is True when this run stops Klipper itself.
        """
        start = f" ({shlex.join(self.service.start)})" if self.service else ""
        if image.kind == "factory":
            return [
                "check the factory firmware answers with that version; compare calibration",
                "Klipper is left stopped afterwards (the factory firmware does not speak its "
                f"protocol); start it later with{start or ' your start command'}",
            ]
        if tool_stops:
            return [f"start Klipper{start}; check it reports ace2k {image.version}"]
        if self.args.config_follows:
            return ["the caller starts Klipper, which checks the new image; this run does not wait"]
        return [
            "you start (or restart) Klipper when asked; the tool checks it reports ace2k "
            f"{image.version}"
        ]

    def _save_calibration(self, unit: Unit, compare: bool) -> None:
        """Read the unit's calibration and save it to a new file in --calibration-dir.

        With compare, also compare it with the newest earlier file of the same unit.  A file
        that cannot be written is refused.
        """
        directory = Path(self.args.calibration_dir)
        uid = read_uid(unit)
        raw, pairs = read_calibration(unit)
        earlier = calibration_files(directory, uid)
        stamp = self.d.now().strftime("%Y%m%d-%H%M%S")
        path = directory / f"ace-calibration-{uid}-{stamp}.txt"
        try:
            directory.mkdir(parents=True, exist_ok=True)
            path.write_text(calibration_text(raw, pairs))
        except OSError as e:
            raise RefusedError(
                f"the calibration could not be written to {directory}: {e.strerror or e}"
            ) from e
        self.out(f"Calibration saved: {path}")
        if not compare:
            return
        if not earlier:
            self.out(f"No earlier calibration file of this unit in {directory}: not compared.")
            return
        self._compare_calibration(path, pairs, earlier)

    def _compare_calibration(
        self, path: Path, pairs: list[tuple[int, int]], earlier: list[Path]
    ) -> None:
        """Compare pairs with the newest earlier file that can be read.

        A file that cannot be read gives a warning, and the next older file is tried.
        """
        for file in reversed(earlier):
            try:
                before = parse_calibration(file.read_text())
            except UnicodeDecodeError:
                self.out(f"Warning: {file} could not be read (not a text file).")
                continue
            except (OSError, RefusedError) as e:
                reason = e.strerror if isinstance(e, OSError) and e.strerror else str(e)
                self.out(f"Warning: {file} could not be read ({reason}).")
                continue
            differ = [i for i, (x, y) in enumerate(zip(before, pairs)) if x != y]
            if len(before) == len(pairs) and not differ:
                self.out(f"Calibration intact (same as {file.name}).")
            else:
                self.out(f"WARNING: the calibration differs from {file.name} in pairs {differ}")
            return
        self.out(
            f"Warning: the calibration was not compared: compare {path} with an earlier file by "
            "hand."
        )

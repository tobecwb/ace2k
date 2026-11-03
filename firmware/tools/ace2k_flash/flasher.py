"""ace2k_flash: flash an ACE 2 Pro with an ace2k image or the factory image.

    python3 ace2k_flash.pyz IMAGE.bin [--port PORT] [--version V] [--yes]
    python3 ace2k_flash.pyz --identify --port PORT

It runs on the Klipper host: Linux, Python 3.9 or later, and pyserial, which Klipper already
needs.  Before anything changes, the tool finds out what runs on the unit: the factory firmware,
ace2k under Klipper, or the bootloader's recovery.  At the end it checks that the new firmware
answers with the new version.  The user guide is docs/flashing.md.

The update over the original Anycubic protocol is three steps: the announce, the 64-byte chunks,
and the finish.  hakimio worked out this sequence and published it first
(https://gist.github.com/hakimio/39c71fa7174e699c6470b7c79323b189).  This tool is an independent
implementation of it.

Exit status:
  0  flashed and verified
  1  failed, or refused after something was done (Klipper stopped, the restore command sent, the
     update started): the message says what state the unit and Klipper are in
  2  flashed, but not verifiable here.  When this run stopped Klipper and could not start it
     again, the status stays 2 and the last line says `Klipper did NOT start`
  3  refused, and nothing was changed: at most read-only queries reached the unit, no command
     that changes the unit or Klipper ran, and Klipper was neither stopped nor started (a
     usage error too)
  4  nothing answered on the port that this flasher can use: not the factory firmware, not
     recovery, and no ace2k that this flasher can send to recovery.  Nothing that changes the
     unit was sent.  Klipper may have been stopped and started again (when it was not ready and
     used this port; if that start fails: 1).  The unit may be off, or run an ace2k image older
     than ace2k_bootloader_enter
  5  --identify only: the unit answered but cannot blink (recovery, or an ace2k image older than
     ace2k_identify).  Nothing was sent that changes the unit
Ctrl-C: 3 before any change, 1 after.
"""

from __future__ import annotations

import argparse
import datetime
import fnmatch
import os
import shlex
import shutil
import subprocess
import sys
import urllib.parse
from pathlib import Path
from typing import Literal, NoReturn, cast

from . import host  # host.port_holders, looked up at the call: one place for the tests to patch
from .base import Deps, close_quietly
from .blink import blink
from .host import (
    ADAPTER_GLOB,
    RESTORE_COMMAND,
    RESTORE_COMMAND_OLD,
    KlipperView,
    Moonraker,
    candidate_ports,
    find_service,
    is_klippy,
    klipper_view,
    same_device,
)
from .image import Image, load_image
from .route import RecoveryRoute
from .verify import Verify
from .wire import (
    EXIT_FAILED,
    EXIT_NO_ANSWER,
    EXIT_UNCHANGED,
    Clock,
    LinkError,
    NothingAnsweredError,
    RefusedError,
    Seen,
    SerialLink,
    Unit,
    UpdateError,
    identify,
    update,
)


class Flasher(RecoveryRoute, Verify):
    """One run of the tool: choose the route, write the image, and check the result."""

    def _resolve_port(self, view: KlipperView) -> str:
        """Return the unit's port: --port, else the one in Klipper's config.

        A --port that is not an adapter of the kind in the unit's cable gets one warning line,
        and is still used.  When neither gives one port, it is refused.  The refusal lists the
        units in the config, or the adapters of the kind in the unit's cable.
        """
        if self.args.port:
            port = cast(str, self.args.port)
            # A port that does not exist is refused next, with no warning.
            if Path(port).exists() and not self._units_adapter(port):
                self.out(
                    f"Warning: {port} does not look like the unit's adapter "
                    f"({self.d.by_id}/{ADAPTER_GLOB}). Never give the port of a printer head."
                )
            return port
        if view.ace2k_port:
            return view.ace2k_port
        if view.ace2k_units:
            lines = ["No port given, and Klipper's config has several units:"]
            for name, p in view.ace2k_units.items():
                lines.append(f"  [ace2k {name}] on {p or 'an MCU with no serial'}")
            lines.append("Pass the port of the one to flash: --port <its serial>")
            raise RefusedError("\n".join(lines))
        lines = ["No port given and no [ace2k] MCU in Klipper's config to take it from."]
        ports = candidate_ports(self.d.by_id)
        if ports:
            lines.append("Adapters of the kind in the unit's cable:")
            for p in ports:
                holders = host.port_holders(p, self.d.proc) if self.d.proc.is_dir() else []
                note = "  (held open by Klipper)" if any(is_klippy(c) for _, c in holders) else ""
                lines.append(f"  {p}{note}")
        else:
            lines.append(f"No adapter of the kind in the unit's cable under {self.d.by_id}.")
        lines.append("Pass the unit's port: --port /dev/serial/by-id/usb-1a86_...")
        raise RefusedError("\n".join(lines))

    def _units_adapter(self, port: str) -> bool:
        """True when port is an adapter of the kind in the unit's cable, by name or by link.

        The name is the one under by_id.  Any other path counts when it leads to the same device
        as one of those adapters, as /dev/ttyACM0 may.
        """
        path = Path(port)
        if path.parent == self.d.by_id and fnmatch.fnmatchcase(path.name, ADAPTER_GLOB):
            return True
        real = os.path.realpath(port)
        return any(os.path.realpath(p) == real for p in candidate_ports(self.d.by_id))

    def _restore_choice(self) -> bool:
        """Choose the name of the restore command, and set it as self.restore_name.

        The name is the one Klipper's command list holds: the current name, else the old name of
        a host module up to v0.12.0-rc.1.  It returns True when the list was read.

        When the list holds neither name, it is refused before any question, and nothing is
        sent.  When the list cannot be read, it takes the current name unchecked and returns
        False.
        """
        listed = self.d.moonraker.gcode_commands()
        if listed is None:
            self.restore_name = RESTORE_COMMAND
            return False
        if RESTORE_COMMAND in listed:
            self.restore_name = RESTORE_COMMAND
        elif RESTORE_COMMAND_OLD in listed:
            self.restore_name = RESTORE_COMMAND_OLD
        else:
            raise RefusedError(
                f"Klipper knows neither {RESTORE_COMMAND} nor {RESTORE_COMMAND_OLD}: is the "
                "[ace2k] host module installed and loaded?  Nothing changed."
            )
        return True

    def run(self) -> int:
        """Run the flow and return its exit status.

        When this run stopped Klipper and could not start it again, the status is EXIT_FAILED,
        whatever it was before.
        """
        status = EXIT_FAILED
        try:
            status = self._outcome()
        finally:
            if self.link is not None:
                close_quietly(self.link)
            if not self._leave_klipper():
                status = EXIT_FAILED
        return status

    def _outcome(self) -> int:
        """Return the flow's exit status.  Every refusal and failure is printed."""
        try:
            return self._run()
        except RefusedError as e:
            self.out(str(e))
            if isinstance(e, NothingAnsweredError) and not self.unit_touched:
                return EXIT_NO_ANSWER
            unchanged = not self.touched if e.unchanged is None else e.unchanged
            return EXIT_UNCHANGED if unchanged else EXIT_FAILED
        except UpdateError as e:
            self.out(f"The update failed: {e}.")
            self._name_a_holder()
            self.out(self._retry(held=e.held))
            return EXIT_FAILED
        except OSError as e:
            return self._system_error(e)
        except KeyboardInterrupt:
            return self._interrupted()

    def _system_error(self, e: OSError) -> int:
        """Report an OSError and return the exit status.

        An error of the serial link is named as one.  Any other error is printed with its own
        text, which names what failed.
        """
        if isinstance(e, LinkError):
            self.out(f"Serial link error: {e}")
            if self.update_started:
                self.out(self._next_step())
        else:
            self.out(f"Error: {e}")
        return EXIT_FAILED if self.touched else EXIT_UNCHANGED

    def _interrupted(self) -> int:
        """Report a Ctrl-C and say what it interrupted.

        Before any change, it counts as a refusal.  After a change, it counts as a failure.
        """
        if not self.touched:
            self.out("Interrupted; nothing changed.")
            return EXIT_UNCHANGED
        self.out("Interrupted.")
        if self.update_started:
            self.out(self._next_step())
        elif self.reset or self.by_cable:  # the unit in recovery, or ace2k_bootloader_enter sent
            self.out(self._reset_note().strip())
        elif self.unit_touched:  # the restore was sent, but the reset was not seen yet
            self.out(
                "The restore command was sent: the unit may be in the bootloader's recovery (the "
                "four LEDs strobing fast); run the tool again, and it identifies recovery."
            )
        return EXIT_FAILED

    def _name_a_holder(self) -> None:
        """After a failed write, name another program that opened the port during the write.

        The check before the write cannot see a program that opens the port later.
        """
        try:
            holders = host.port_holders(self.port, self.d.proc)
        except RefusedError:  # no /proc: nothing to name
            return
        others = [h for h in holders if h[0] != os.getpid()]
        if others:
            pid, cmd = others[0]
            self.out(
                f"Another program opened {self.port} during the write ({cmd}, pid {pid}): it is "
                "the likely cause."
            )

    def _next_step(self) -> str:
        """Return what to do when the run is cut short after the update started.

        An update cut short is simply run again.  After a whole write, only the check was cut
        short.
        """
        if self.written:
            return (
                "The image was written, but checking it was cut short. Running the same command "
                "again writes the image again: after the plan and the question, or at once with "
                "--yes."
            )
        return self._retry()

    def _retry(self, held: bool = False) -> str:
        """Return what to do after a failed update.

        Recovery accepts a new update at once.  The factory firmware keeps an update it did not
        finish, and refuses the next one until it restarts.

        held is True when the first chunk was refused with the generic failure code.  The most
        likely cause is that the unit already holds an unfinished update.
        """

        if self.target is None or self.target.kind != "factory":
            return "The unit accepts a new update: run the same command again."
        if held:
            return (
                "The factory firmware probably holds an unfinished update from an earlier run: "
                "power-cycle the unit, then run the same command again."
            )
        return (
            "The factory firmware may still hold this update and refuse a new one: power-cycle "
            "the unit, then run the same command again."
        )

    def _leave_klipper(self) -> bool:
        """Start Klipper again if this run stopped it, or say why it stays stopped.

        It returns False when that start fails.  When the flow already tried a start, that start
        was reported there, and the flow's own status stands.
        """
        if not self.stopped or self.start_tried:
            return True
        assert self.service is not None  # this run stopped Klipper with it
        if self.restart_ok:
            self.out(f"Starting Klipper again: {shlex.join(self.service.start)}")
            return self._start_klipper()
        self.out("Klipper is left STOPPED: this run targets the factory firmware, which does not")
        if self.target is not None and self.target.kind == "factory":  # not in recovery
            self.out("speak Klipper's protocol.")
        else:
            self.out("speak Klipper's protocol (if the update failed, the unit is in recovery).")

        self.out("Remove [mcu ace2k] and [ace2k] from the config, including any #*# [ace2k]")
        self.out("section in the SAVE_CONFIG block at the end of printer.cfg, then start Klipper:")
        self.out(f"  {shlex.join(self.service.start)}")
        return True

    def _run(self) -> int:
        """Load the image, check the host, choose the route to the unit, and flash."""
        a, d = self.args, self.d
        if a.identify:
            return blink(a.port, d)
        image = load_image(Path(a.image), a.version)
        view = klipper_view(d.moonraker, a.port)
        port = self.port = self._resolve_port(view)
        self.unit_name = view.ace2k_unit  # for [ace2k <name>], the restore adds UNIT=<name>
        self._preflight(view, port)
        self.service = find_service(a.klipper_stop, a.klipper_start, d.which, d.run, d.initd)
        flash = {
            "restore": self._flash_by_restore,
            "not_ready": self._flash_after_stop,
            "free": self._flash_free_port,
        }[self._route(view, port)]
        return flash(image, view, port)

    @staticmethod
    def _preflight(view: KlipperView, port: str) -> None:
        """Refuse when the port does not exist, the print state is unknown, or a print runs."""
        if not Path(port).exists():
            raise RefusedError(f"{port} does not exist; nothing changed")
        if view.printing is None:
            raise RefusedError(
                "Klipper is ready but its print state could not be read; nothing changed"
            )
        if view.printing:
            raise RefusedError("A print is running or paused; nothing changed")

    def _route(self, view: KlipperView, port: str) -> Literal["restore", "not_ready", "free"]:
        """How to reach the unit on port.

        "restore": ace2k runs under a ready Klipper, which sends the restore command.
        "not_ready": Klipper is not ready and uses the port, so it is stopped first.
        "free": nothing holds the port.
        A port that another program holds open is refused. So is a port that Klipper holds
        when Klipper is not to be stopped.
        """
        if view.state == "ready" and view.ace2k_version and same_device(view.ace2k_port, port):
            return "restore"
        holders = host.port_holders(port, self.d.proc)
        stop_first = self._stop_first(view, port, holders)
        if holders and (not all(is_klippy(c) for _, c in holders) or not stop_first):
            listed = "; ".join(f"pid {p}: {c}" for p, c in holders)
            raise RefusedError(f"{port} is held open ({listed}); nothing changed")
        return "not_ready" if stop_first else "free"

    @staticmethod
    def _stop_first(view: KlipperView, port: str, holders: list[tuple[int, str]]) -> bool:
        """Return True when Klipper is not ready and uses the port, so it must be stopped first."""
        # Klipper's own config decides, not who holds the port at this instant.  A Klipper that
        # is not ready closes the port after a failed identify, and opens it again every few
        # seconds.
        configured_here = view.configured_on(port)
        not_ready = view.state in ("startup", "error", "shutdown")
        return bool(
            (holders and view.state not in (None, "ready")) or (not_ready and configured_here)
        )

    def _flash_by_restore(self, image: Image, view: KlipperView, port: str) -> int:
        """Flash a unit that runs ace2k under a ready Klipper.

        The restore command resets the unit into recovery.  Then Klipper is stopped and the image
        is written.
        """
        if self.service is None:
            raise RefusedError(
                "ace2k runs under Klipper, but this tool does not know how to stop Klipper "
                "here: pass --klipper-stop and --klipper-start; nothing changed"
            )
        checked = self._restore_choice()
        unchecked = "" if checked else ", unchecked: the command list could not be read"
        self._plan(
            image,
            port,
            f"ace2k {view.ace2k_version}, under Klipper",
            [
                f"{self.restore_name}{unchecked} (the unit resets into the bootloader's recovery)",
                f"stop Klipper: {shlex.join(self.service.stop)}",
                f"write {image.kind} {image.version}",
                *self._after_steps(image, tool_stops=True),
            ],
        )
        unit, seen = self._enter_recovery(port, checked)
        return self._flash(unit, image, view, seen)

    def _flash_after_stop(self, image: Image, view: KlipperView, port: str) -> int:
        """Flash when Klipper is not ready and uses the port.

        Klipper is stopped first.  Then the unit is identified and the image is written.
        """
        if self.service is None:
            raise RefusedError(
                f"Klipper is running and not ready ({view.state}) and may hold {port}, but "
                "this tool does not know how to stop it here: pass --klipper-stop and "
                "--klipper-start; nothing changed"
            )
        self._plan(
            image,
            port,
            f"Klipper is not ready ({view.state}) and uses this port",
            [
                f"stop Klipper: {shlex.join(self.service.stop)}",
                "identify the unit; stop if it is neither the factory firmware nor recovery",
                f"write {image.kind} {image.version}",
                *self._after_steps(image, tool_stops=True),
            ],
        )
        self._stop_klipper(port)
        unit = self._open_unit(port)
        seen = identify(unit)
        if seen is None:
            raise NothingAnsweredError(
                f"Nothing on {port} answered as the factory firmware or recovery.  If the "
                "unit runs ace2k, get Klipper ready (FIRMWARE_RESTART) and run this again."
            )
        self._one_unit(unit, port)
        return self._flash_identified(unit, image, view, seen, confirmed=True)

    def _flash_free_port(self, image: Image, view: KlipperView, port: str) -> int:
        """Flash when nothing holds the port: identify the unit and write the image.

        A unit that answers neither as the factory firmware nor as recovery may run ace2k with no
        Klipper connected.  Then it is sent into recovery over its cable.
        """
        unit = self._open_unit(port)
        seen = identify(unit)
        if seen is None:
            close_quietly(unit.link)
            self.link = None
            unit, seen = self._enter_recovery_by_cable(image, port)
            return self._flash(unit, image, view, seen)
        self._one_unit(unit, port)
        return self._flash_identified(unit, image, view, seen, confirmed=False)

    def _flash_identified(
        self, unit: Unit, image: Image, view: KlipperView, seen: Seen, confirmed: bool
    ) -> int:
        """Flash a unit that runs the factory firmware or recovery.

        confirmed is True when the user already confirmed the plan.  Otherwise the plan is shown
        and confirmed here.  The calibration of a unit that runs the factory firmware is saved
        before the write.
        """
        found = (
            f"factory firmware {seen.version}"
            if seen.kind == "factory"
            else f"bootloader recovery (boot {seen.boot_version}, last upload {seen.version!r})"
        )
        if not confirmed:
            steps: list[str] = []
            if seen.kind == "factory":
                steps.append(f"save the calibration to {self.args.calibration_dir}")
            steps += [
                f"write {image.kind} {image.version}",
                *self._after_steps(image, tool_stops=False),
            ]
            self._plan(image, self.port, found, steps)
        else:
            self.out(f"Identified: {found}")
        if seen.kind == "factory":
            self._save_calibration(unit, compare=False)
        return self._flash(unit, image, view, seen)

    def _flash(self, unit: Unit, image: Image, view: KlipperView, target: Seen) -> int:
        """Write image to the unit, which runs target, then check the result.

        target is the factory firmware or recovery.
        """
        self._still_alone()
        if image.kind == "factory":
            self.restart_ok = False
        self.target = target
        self.update_started = self.touched = self.unit_touched = True
        self.out(f"Writing {image.kind} {image.version} ...")
        update(unit, image, self.out, self.d.clock)
        self.written = True
        self.out("Written.")
        if image.kind == "factory":
            return self._verify_factory(unit, image)
        return self._verify_ace2k(unit, image, view)

    def _still_alone(self) -> None:
        """Refuse when another program opened the port after this tool did.

        A Klipper that is not ready opens the port again every few seconds.  Each time it opens
        the port, it changes the port's speed.
        """
        others = [h for h in host.port_holders(self.port, self.d.proc) if h[0] != os.getpid()]
        if not others:
            return
        pid, cmd = others[0]
        why = (
            f"{self.port} was opened by another program after this tool opened it "
            f"({cmd}, pid {pid})"
        )
        if self.reset:
            raise RefusedError(why + "." + self._reset_note())
        raise RefusedError(why + ("; nothing was written" if self.stopped else "; nothing changed"))


class _Parser(argparse.ArgumentParser):
    """An argument parser whose usage error exits with EXIT_UNCHANGED.

    A usage error means "refused, nothing changed".  argparse's own status 2 would mean "flashed,
    but not verified".
    """

    def error(self, message: str) -> NoReturn:
        self.print_usage(sys.stderr)
        self.exit(EXIT_UNCHANGED, f"{self.prog}: error: {message}\n")


def main(argv: list[str] | None = None) -> int:
    """Parse the command line, run the flow, and return the exit status."""
    ap = _Parser(
        # The file's own name, as every Python version before 3.14 prints it.  Python 3.14
        # would print "python3.14 ace2k_flash.pyz" for a zip.
        prog=os.path.basename(sys.argv[0]),
        description=__doc__.split("\n")[0],
        epilog=__doc__[__doc__.index("Exit status:") :],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "image",
        nargs="?",
        help="the image: an ace2k .bin, or the factory .bin (not with --identify)",
    )
    ap.add_argument(
        "--identify",
        action="store_true",
        help="blink the unit on --port and exit; no image, nothing is written",
    )
    ap.add_argument("--port", help="the unit's serial port (default: from Klipper's config)")
    ap.add_argument("--version", help="the image's version, when the image does not carry it")
    ap.add_argument(
        "--moonraker",
        default="http://localhost:7125",
        help="Moonraker's URL (it starts with http:// or https://)",
    )
    ap.add_argument("--klipper-stop", help="the command that stops Klipper on this host")
    ap.add_argument("--klipper-start", help="the command that starts Klipper on this host")
    ap.add_argument("--calibration-dir", default=".", help="where calibration files go")
    ap.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    ap.add_argument(
        "--config-follows",
        action="store_true",
        help="the caller writes [ace2k] and starts Klipper after this run (an installer)",
    )
    args = ap.parse_args(argv)
    if args.identify and (args.image or not args.port):
        ap.error("--identify takes --port and no image")
    if not args.identify and not args.image:
        ap.error("the following arguments are required: image")
    url = urllib.parse.urlsplit(args.moonraker)
    if url.scheme not in ("http", "https") or not url.netloc:
        ap.error(
            f"--moonraker {args.moonraker}: give the URL with http:// or https:// and a host, "
            "such as http://localhost:7125"
        )
    deps = Deps(
        open_link=SerialLink,
        moonraker=Moonraker(args.moonraker),
        clock=Clock(),
        run=subprocess.run,
        which=shutil.which,
        proc=Path("/proc"),
        by_id=Path("/dev/serial/by-id"),
        initd=Path("/etc/init.d/S60klipper"),
        ask=input,
        out=print,
        now=datetime.datetime.now,
    )
    return Flasher(args, deps).run()

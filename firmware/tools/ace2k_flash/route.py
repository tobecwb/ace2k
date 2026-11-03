"""The flasher's way into the bootloader's recovery for a unit that runs ace2k.

Under a ready Klipper, the restore command resets the unit (with a fall-back to its old name, and
a wait for the reset).  With no Klipper connected, ace2k_bootloader_enter goes over the unit's own
cable.  Either way, the port is then opened again and recovery must answer.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .base import REFUSAL_SETTLE_S, RESTORE_WAIT_S, UNKNOWN_SETTLE_S, FlasherBase, close_quietly
from .host import RestoreRequest
from .klipper import ACE2K_BAUD, BOOTLOADER_ENTER, enter_recovery
from .wire import RefusedError, Seen, Unit, identify

if TYPE_CHECKING:
    from .image import Image


class RecoveryRoute(FlasherBase):
    """The steps that take a unit running ace2k into recovery."""

    def _enter_recovery(self, port: str, checked: bool) -> tuple[Unit, Seen]:
        """Send the restore command and wait for the reset.

        The name is self.restore_name, set by _restore_choice.  checked is False when the
        command list could not be read, so the name was not checked.  It returns the unit and
        what recovery answered.
        """
        unit = self.unit_name
        req = RestoreRequest(
            self.d.moonraker, self.restore_name, f" UNIT={unit}" if unit else "", checked
        )
        req.mark_console()
        self.touched = self.unit_touched = True
        req.send()
        if not checked:
            self._fall_back_if_unknown(req)
        self._await_reset(req)
        return self._open_recovery(port)

    def _enter_recovery_by_cable(self, image: Image, port: str) -> tuple[Unit, Seen]:
        """Send an ace2k unit with no Klipper into recovery over its own cable.

        enter_recovery refuses a unit without ace2k_bootloader_enter, and a veto.  An accepted
        command can still be withdrawn, and an answer can be lost, so recovery is looked for.
        """
        link = self.d.open_link(port, ACE2K_BAUD)
        try:
            enter_recovery(link, self.d.clock, lambda v: self._plan_by_cable(image, v), port)
        finally:
            close_quietly(link)
        self.d.clock.sleep(RESTORE_WAIT_S / 3)  # the reset comes 50 ms after the answer
        return self._find_recovery(port)

    def _plan_by_cable(self, image: Image, version: str) -> None:
        """Show the plan for an ace2k unit reached over its cable, and get it confirmed."""
        self._plan(
            image,
            self.port,
            f"ace2k {version}, over its cable (no Klipper connected)",
            [
                f"{BOOTLOADER_ENTER} (the unit resets into the bootloader's recovery)",
                f"write {image.kind} {image.version}",
                *self._after_steps(image, tool_stops=False),
            ],
        )
        self.restore_name = BOOTLOADER_ENTER  # the refusals after the reset name it
        self.touched = self.unit_touched = self.by_cable = True  # the command goes next

    def _fall_back_if_unknown(self, req: RestoreRequest) -> None:
        """Send the old name when Klipper did not know the new one.

        This happens only when the name was not checked against the command list.
        """
        if req.answer is not True or req.mark is None:
            return
        # An old host module answers the new name with an info line only.  The console store
        # may lag behind the answer, so the tool looks for a short time.  A reset ends the wait,
        # as it does on a new host.
        self._wait(UNKNOWN_SETTLE_S, lambda: req.unknown() or req.left_ready())
        if not req.left_ready() and req.unknown():
            req.fall_back_to_old_name()
            self.restore_name = req.name

    def _await_reset(self, req: RestoreRequest) -> None:
        """Wait for the unit's reset to take Klipper out of ready.  Refuse when it does not."""
        error = req.error
        if error is not None and (not error.ran or error.is_refusal):
            self._after_http_error(req)
        else:
            self._after_answer(req)

    def _after_http_error(self, req: RestoreRequest) -> None:
        """Handle a command that came back with an error from the module, or did not run."""
        error = req.error
        assert error is not None  # _await_reset calls this only for an error answer
        if not error.ran:
            raise RefusedError(
                f"{req.name} did not run: {error.first_line}; nothing changed.{req.unlisted}",
                unchanged=True,
            )
        # The answer decides: it is a refusal, which Klipper printed as `!! <first line>`.  A
        # reset may still follow, so the tool waits a moment, then refuses with that line.
        if not self._wait(REFUSAL_SETTLE_S, req.left_ready):
            raise req.refused(f"!! {error.first_line}")

    def _after_answer(self, req: RestoreRequest) -> None:
        """Wait for the reset after a command that ran without a refusal, or got no answer."""
        # Klipper takes a few seconds to notice a reset.  A refusal on the console ends the wait
        # early, but only after a script that ran without error.  With no answer, the refusal
        # may belong to another client, so the wait runs to its end.  A slow reset still counts.
        if req.answer is True:
            self._wait(RESTORE_WAIT_S, lambda: req.left_ready() or bool(req.new_refusals()))
            if not req.left_ready() and req.new_refusals():
                self._wait(REFUSAL_SETTLE_S, req.left_ready)
        else:
            self._wait(RESTORE_WAIT_S, req.left_ready)
        if not req.left_ready():
            self._no_reset(req)

    def _no_reset(self, req: RestoreRequest) -> None:
        """Refuse because Klipper still reports ready after the wait.  Say what is known."""
        error = req.error
        fresh = [] if error is not None else req.new_refusals()
        if fresh:
            raise req.refused(fresh[-1])
        where = (
            "Moonraker does not answer"
            if req.mr.state() is None
            else f"Klipper still reports ready {RESTORE_WAIT_S:.0f} s after {req.name}"
        )
        if error is not None:
            what = f"{req.name} failed: {error.first_line}.  {where}"
        elif req.answer is None:
            what = f"{req.name} got no answer: it may not have reached Moonraker.  {where}"
        else:
            what = f"{where} and no refusal was printed"
        raise RefusedError(
            f"{what}: the unit may or may not have reset.  Nothing was stopped.  Look at "
            "the LEDs — all four strobing fast is recovery — and run the tool again (it "
            f"identifies recovery on its own).{req.unlisted}"
        )

    def _open_recovery(self, port: str) -> tuple[Unit, Seen]:
        """Stop Klipper, open the port, and check that one unit answers, in recovery."""
        self._stop_klipper(port, after_restore=True)
        return self._find_recovery(port)

    def _find_recovery(self, port: str) -> tuple[Unit, Seen]:
        """Open the port after the reset, and check that one unit answers, in recovery."""
        unit = self._open_unit(port, reset=True)
        seen = identify(unit)
        if seen is None:
            raise RefusedError(
                f"Expected the bootloader's recovery on {port}, found nothing." + self._reset_note()
            )
        if seen.kind != "recovery":  # the factory firmware, so the reset did not reach this unit
            raise RefusedError(
                f"Expected the bootloader's recovery on {port}, found the factory firmware: "
                "nothing was written.  The unit runs the factory firmware, which Klipper does not "
                "connect to: run the tool again, and it identifies the factory firmware and writes "
                "the image."
            )
        self.out(f"Recovery answers (boot {seen.boot_version}).")
        self.by_cable = False  # recovery is seen: the reset note holds from here on
        self._one_unit(unit, port, reset=True)
        return unit, seen

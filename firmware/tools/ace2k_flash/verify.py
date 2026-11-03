"""The flasher's check after the write.

For an ace2k image: the bootloader lets go, the image proves its link, and Klipper reports the new
version (or the run says why that cannot be checked here).  For the factory image: the old
firmware lets go, the new one answers with its version, and the calibration is saved again and
compared.
"""

from __future__ import annotations

import shlex
from typing import TYPE_CHECKING

from .base import (
    ACE2K_WAIT_S,
    FACTORY_WAIT_S,
    OLD_FIRMWARE_WAIT_S,
    PROVE_WAIT_S,
    SILENT_POLLS,
    VERDICT_TRIES,
    VERDICT_WAIT_S,
    FlasherBase,
    close_quietly,
)
from .host import klipper_view, same_device
from .klipper import ACE2K_BAUD, KlipperUnit
from .wire import (
    CMD_GET_INFO,
    EXIT_OK,
    EXIT_UNVERIFIED,
    IDENTIFY_TIMEOUT_S,
    LinkError,
    RefusedError,
    Seen,
    Unit,
    classify,
)

if TYPE_CHECKING:
    from .host import KlipperView
    from .image import Image
    from .wire import Json


class Verify(FlasherBase):
    """The steps that check the result after the write."""

    def _verify_ace2k(self, unit: Unit, image: Image, view: KlipperView) -> int:
        """Check that the ace2k image runs and that Klipper reports it.

        The bootloader must let go of the unit.  Then the image is asked for its version, and
        Klipper must report it.  When Klipper cannot be asked about this port, it returns
        EXIT_UNVERIFIED.
        """
        self.image = image  # kept so the messages can name its waiting time
        self._bootloader_let_go(unit)
        assert self.link is not None  # _open_unit opened it for the update
        close_quietly(self.link)
        self.link = None
        proven = self._prove(image)
        here = view.configured_on(self.port)
        # The caller (an installer) stopped Klipper, writes the config and starts it.  Otherwise
        # Klipper stopped by hand cannot have its config read, but it can be started and asked.
        follows = self.args.config_follows and not self.stopped
        if view.state is None or follows or not (here or view.state == "disconnected"):
            return self._unverifiable(view, proven, image)
        self._ask_for_klipper(proven)
        return self._await_report(image, proven)

    def _bootloader_let_go(self, unit: Unit) -> None:
        """Wait for the bootloader to copy the image and start it.

        If the factory firmware or the bootloader still answers afterwards, the image was not
        taken, and this is refused.
        """
        self.out(
            f"Waiting {VERDICT_WAIT_S:.0f} s for the bootloader to copy the image and start it..."
        )
        self.d.clock.sleep(VERDICT_WAIT_S)
        for _ in range(VERDICT_TRIES):
            frame = unit.ask(CMD_GET_INFO, timeout=IDENTIFY_TIMEOUT_S)
            seen = None if frame is None else classify(frame)
            if seen is not None:
                if seen.kind == "factory":
                    # The factory firmware that took the update keeps it until it restarts
                    took = self.target is not None and self.target.kind == "factory"
                    raise RefusedError(
                        "The factory firmware answers after the update: the image was not taken.  "
                        + (
                            "Power-cycle the unit, then check the image and run the command again."
                            if took
                            else "Check the image, then run the command again."
                        )
                    )
                raise RefusedError(
                    "The bootloader still answers after the update: the image was refused and "
                    "the unit is in recovery.  Check the image, then run the command again."
                )

    def _unverifiable(self, view: KlipperView, proven: bool, image: Image) -> int:
        """Say why ace2k cannot be checked from here, and what to do.  Return EXIT_UNVERIFIED."""
        self.out("The bootloader let go of the unit; ace2k cannot be checked from here:")
        if self.stopped:
            return self._unverifiable_after_stop(proven)
        return self._unverifiable_untouched(view, proven, image)

    def _unverifiable_after_stop(self, proven: bool) -> int:
        """This run stopped Klipper: start it again and say how to add the unit to its config."""
        self.start_tried = True
        if not self._start_klipper():
            return EXIT_UNVERIFIED
        self.out("Klipper was started again with its current config, which has no [ace2k]")
        self.out("on this port: ace2k cannot be checked from here.")
        for line in self._config_lines(proven, "restart Klipper"):
            self.out(line)
        return EXIT_UNVERIFIED

    def _unverifiable_untouched(self, view: KlipperView, proven: bool, image: Image) -> int:
        """This run did not stop Klipper: say what is missing, and how to add the unit.

        With --config-follows, the caller adds the unit to Klipper's config when it is missing,
        and starts Klipper.
        """
        here = view.configured_on(self.port)
        if view.state is None:
            self.out("Moonraker does not answer here.")
        elif self.args.config_follows:
            missing = not here and view.state != "disconnected"
            self.out(
                (
                    "Klipper's config has no [ace2k] on this port yet: the caller adds it and "
                    "starts Klipper"
                    if missing
                    else "Klipper checks the new image when it starts: the caller starts Klipper"
                )
                + ("." if proven else f", within the new image's {self._window(image)}.")
            )
            return EXIT_UNVERIFIED
        else:
            self.out("Klipper's config has no [ace2k] on this port.")
        for line in self._config_lines(proven, "start Klipper"):
            self.out(line)
        return EXIT_UNVERIFIED

    def _ask_for_klipper(self, proven: bool) -> None:
        """Start Klipper if this run stopped it.

        Otherwise ask the user to start or restart Klipper, unless it is already ready.
        """
        window = "" if proven else self._window_note()
        if self.stopped:
            assert self.service is not None  # this run stopped Klipper with it
            self.out(f"Starting Klipper: {shlex.join(self.service.start)}")
            self.start_tried = True  # a failed start is not tried again when the run ends
            if not self._start_klipper():
                raise RefusedError("Klipper did not start; the unit waits for it in ace2k")
        elif (live := klipper_view(self.d.moonraker, self.port).state) == "disconnected":
            self.out(
                "Start Klipper now. Waiting up to "
                f"{ACE2K_WAIT_S:.0f} s for it to report the new version{window}."
            )
        elif live != "ready":
            how = f", or `{shlex.join(self.service.start)}`" if self.service else ""
            self.out(
                f"Restart Klipper (FIRMWARE_RESTART{how}). Waiting up to "
                f"{ACE2K_WAIT_S:.0f} s for it to report the new version{window}."
            )

    def _await_report(self, image: Image, proven: bool) -> int:
        """Wait for Klipper to report the new version on this port.

        It returns EXIT_OK.  It refuses when the wait runs out.
        """
        mr = self.d.moonraker

        def reported() -> bool:
            v = klipper_view(mr, self.port)
            if not (v.ace2k_version and same_device(v.ace2k_port, self.port)):
                return False
            return image.has_version(v.ace2k_version)

        if self._wait(ACE2K_WAIT_S, reported):
            self.out(f"Klipper reports ace2k {image.version}: done.")
            return EXIT_OK
        raise RefusedError(
            f"Klipper did not report ace2k {image.version} within {ACE2K_WAIT_S:.0f} s: check "
            "ACEK_STATUS and klippy.log" + ("" if proven else self._window_note())
        )

    def _prove(self, image: Image) -> bool:
        """Ask the new image for its version over its own link, as Klipper asks when it connects.

        The answer proves the image's link, and then the image never falls back into recovery.
        It returns True when the image answered and stored its proof.  It returns False when the
        port cannot be opened, because another program took it meanwhile.  Then Klipper's own
        connect proves the image.  It refuses when the image does not answer within
        PROVE_WAIT_S, or answers another version.
        """
        self.out("Asking the new image for its version (its link proof) ...")
        try:
            link = self.d.open_link(self.port, ACE2K_BAUD)
        except RefusedError as e:
            self.out(f"{e}: Klipper's connect proves the image instead.")
            return False
        dictionary: Json | None = None
        version: str | None = None
        try:
            unit = KlipperUnit(link, self.d.clock)
            deadline = self.d.clock.now() + PROVE_WAIT_S
            while True:
                if dictionary is None:  # read once; only the questions after it are repeated
                    dictionary = unit.dictionary(deadline)
                answer = unit.version(dictionary) if dictionary else None
                if answer is not None:
                    version = answer
                    if not image.has_version(version):
                        raise RefusedError(
                            f"The new image answers version {version}, not {image.version}: "
                            "check the image and run the command again."
                        )
                    assert dictionary is not None  # an answer needs a dictionary
                    if unit.proven(dictionary):
                        break
                if self.d.clock.now() >= deadline:
                    why = "does not answer" if version is None else "did not store its proof"
                    raise RefusedError(
                        f"The new image {why} on {self.port} within {PROVE_WAIT_S:.0f} s.  It "
                        f"goes back to recovery by itself within {self._window(image)} of its "
                        "start (the four LEDs strobing fast); then run this command again: it "
                        "finds recovery and writes the image again."
                    )
                self.d.clock.sleep(1.0)
        finally:
            close_quietly(link)
        self.out(f"ace2k {version} answers and stored its proof: it stays.")
        return True

    @staticmethod
    def _window(image: Image) -> str:
        """Return how long a newly written image waits for a host, as its dictionary says.

        An image built before that constant existed waits 180 s.  Some test images from just
        before the constant waited 600 s.
        """
        if image.window_s is not None:
            return f"{image.window_s} s"
        return "180 s (600 s for some images)"

    def _window_note(self) -> str:
        """Return the note that says how long the new image waits for a host."""
        return (
            " (a newly written image goes back to recovery if no host talks to it within "
            f"{self._window(self.image)})"
        )

    def _config_lines(self, proven: bool, start: str) -> list[str]:
        """Return the lines that tell how to add the unit to Klipper's config.

        proven is True when the image stored its link proof.  start names what to do to Klipper
        afterwards.
        """
        if proven:
            return [
                "The image is proven and stays: add [mcu ace2k] and [ace2k] (docs/flashing.md)",
                f"and {start}.",
            ]
        window = self._window(self.image)
        return [
            "A NEWLY WRITTEN IMAGE GOES BACK TO RECOVERY IF NO HOST TALKS TO IT",
            f"WITHIN {window}: add [mcu ace2k] and [ace2k] (docs/flashing.md) and",
            f"{start} within that window.",
            "If that window passes, the unit stays in recovery: finish the config,",
            "restart Klipper, and run this command again.",
        ]

    def _old_firmware_let_go(self, unit: Unit, old: Seen) -> None:
        """Wait for the factory firmware that took the update to stop answering.

        Before it resets, the old firmware checks the staged image and writes the request to the
        bootloader.  So for about 2 s it still answers with its own version.  An answer from
        recovery, or with another version, means it let go at once.  Silence counts only when
        SILENT_POLLS polls in a row get no answer, because a single reply can be lost.  It is
        refused when the old firmware still answers after OLD_FIRMWARE_WAIT_S.
        """
        silent = [0]

        def let_go() -> bool:
            frame = unit.ask(CMD_GET_INFO, timeout=IDENTIFY_TIMEOUT_S)
            if frame is None:
                silent[0] += 1
                return silent[0] >= SILENT_POLLS
            silent[0] = 0
            seen = classify(frame)
            return seen is not None and (seen.kind != "factory" or seen.version != old.version)

        self.out("Waiting for the old factory firmware to stop answering...")
        if not self._wait(OLD_FIRMWARE_WAIT_S, let_go):
            raise RefusedError(
                f"The old factory firmware ({old.version}) still answers "
                f"{OLD_FIRMWARE_WAIT_S:.0f} s after the update: it did not take the image.  "
                "Power-cycle the unit, then run the same command again."
            )

    def _verify_factory(self, unit: Unit, image: Image) -> int:
        """Check that the factory firmware answers with the image's version.

        Then save the calibration again and compare it with an earlier file.  A calibration that
        cannot be read after the update is only a warning.
        """
        if self.target is not None and self.target.kind == "factory":
            self._old_firmware_let_go(unit, self.target)
        want = image.version.lstrip("Vv")
        last: list[Seen | None] = [None]

        def answered() -> bool:
            frame = unit.ask(CMD_GET_INFO, timeout=IDENTIFY_TIMEOUT_S)
            last[0] = None if frame is None else classify(frame)
            return last[0] is not None and last[0].kind == "factory"

        self.out(f"Waiting for the factory firmware to start (up to {FACTORY_WAIT_S:.0f} s)...")
        if not self._wait(FACTORY_WAIT_S, answered):
            what = "recovery still answers" if last[0] else "nothing answers"
            raise RefusedError(f"The factory firmware did not answer within the wait ({what})")
        seen = last[0]
        assert seen is not None  # the wait ended with an answer from the factory firmware
        got = seen.version
        if got.lstrip("Vv") != want:
            raise RefusedError(f"The factory firmware answers with {got}, not {image.version}")
        self.out(f"The factory firmware answers: {got}.")
        try:  # the flash is verified, so a calibration that cannot be read is only a warning
            self._save_calibration(unit, compare=True)
        except (RefusedError, LinkError) as e:
            self.out(f"Warning: the calibration could not be read after the update: {e}")
        return EXIT_OK

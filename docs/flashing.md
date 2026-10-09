# Flashing

`firmware/tools/ace2k_flash.py` flashes the unit with one command on the Klipper host. It works in
every direction:

- from the factory firmware to ace2k;
- from ace2k to a newer ace2k;
- from ace2k back to the factory firmware;
- out of the bootloader's recovery mode.

The tool speaks the original Anycubic protocol at 230400 baud. It reaches the unit's RS-485 host
port through the USB adapter inside the unit's own cable. [hakimio](https://github.com/hakimio)
worked out this update sequence and published it first. The manual procedure below still uses his
updater (<https://gist.github.com/hakimio/39c71fa7174e699c6470b7c79323b189>).

You can always go back to the factory firmware, with the same tool. See
[Back to the factory firmware](#back-to-the-factory-firmware) below.

## Before you start

1. **The USB cable.** The unit has no USB port of its own. Build the cable first
   ([`cable.md`](cable.md)). Then, with the unit powered, check that the host sees it:
   `lsusb` lists `1a86:55d3`, and `ls /dev/serial/by-id/` shows
   `usb-1a86_USB_Single_Serial_<serial>-if00`. Nothing below works until it does.
2. **The factory image.** Get it and keep it before the first flash. You will want it if you ever
   go back. [Back to the factory firmware](#back-to-the-factory-firmware) says where to get it.
3. **A Klipper host** you can log into, as a user allowed to stop and start Klipper.

## With `ace2k_flash`

### First install

Your unit still runs the factory firmware. Set up Klipper first. The tool can then find the unit,
flash it and check the result in one go.

Below, `<user>@<host>` is how you log into your Klipper host over SSH. Klipper is usually in
`~/klipper` on the host. If yours is somewhere else, adjust the commands.

**On your computer:**

1. **Get the release.** Clone this repository at the release's tag, and download the release's
   image into it:

       git clone --branch vX.Y.Z https://github.com/tobecwb/ace2k
       cd ace2k
       curl -LO https://github.com/tobecwb/ace2k/releases/download/vX.Y.Z/ace2k-X.Y.Z.bin

2. **Copy everything to the Klipper host.** The host module goes into Klipper. The tool and the
   image go into one directory, `~/ace2k-flash`. The tool also saves its calibration copies there.

       scp klippy/extras/ace2k*.py <user>@<host>:klipper/klippy/extras/
       scp -r klippy/extras/ace2k_tags <user>@<host>:klipper/klippy/extras/
       ssh <user>@<host> mkdir -p ace2k-flash
       scp firmware/tools/ace2k_flash.py ace2k-X.Y.Z.bin <user>@<host>:ace2k-flash/

**On Windows:**

- Windows Terminal / PowerShell already has `ssh`, `scp` and `curl`.
- `git` comes with [Git for Windows](https://git-scm.com/download/win).
- There is no `ssh-copy-id`. Without a key, `ssh` and `scp` simply ask for the host's password.
- If you prefer graphical tools: [WinSCP](https://winscp.net/) copies the files of step 2, and
  [PuTTY](https://www.putty.org/) opens the SSH session for the steps below. The release page
  offers the source as a zip and the image as a file.

**On the Klipper host** (`ssh <user>@<host>`):

3. **Find the unit's USB path.** Turn the unit on and plug its cable into the host. Then:

       ls /dev/serial/by-id/

   Look for `usb-1a86_USB_Single_Serial_<serial>-if00` and write the whole name down.

4. **Add the unit to `printer.cfg`.** Open it and add these two sections, with your path from
   step 3:

       [mcu ace2k]
       serial: /dev/serial/by-id/usb-1a86_USB_Single_Serial_<serial>-if00
       baud: 250000
       restart_method: command

       [ace2k]

   That is all the flash needs. Save the file.

5. **Restart Klipper.** It now shows an error about `mcu ace2k`. This is expected: the unit still
   speaks the factory protocol, so Klipper cannot talk to it yet.

6. **Run the tool:**

       cd ~/ace2k-flash
       python3 ace2k_flash.py ace2k-X.Y.Z.bin

   The tool finds the unit from `printer.cfg`. It shows what it will do and asks
   `Proceed? [y/N]`. Answer `y`. Then the tool:

   1. Saves a copy of the unit's factory calibration to
      `ace-calibration-<unit id>-<date-time>.txt` in `~/ace2k-flash`. If you ever go back to the
      factory firmware, the tool compares the unit with this file. Keep the file, and copy it to
      your computer too.
   2. Stops Klipper.
   3. Writes the image (about 20 s).
   4. Starts Klipper again and checks the version.

   It ends with `Klipper reports ace2k X.Y.Z: done.`

7. **Check.** Klipper is ready. In the console, `ACE_STATUS` starts with `ace2k X.Y.Z, link
   proven: yes`. The lane LEDs show each lane's state: on for a loaded lane, off for an empty one.

If anything stops the tool, it tells you what state the unit and Klipper are in. Running the same
command again is safe.

### Updating to a newer ace2k

1. Get the new release. Copy its host module and image to the Klipper host, as in steps 1 and 2
   above. The tool restarts Klipper after the flash, and that loads the new host module.
2. Stop the dryer and any lane that moves. Do not update during a print. The tool refuses in both
   cases, and then changes nothing.
3. On the Klipper host:

       cd ~/ace2k-flash
       python3 ace2k_flash.py ace2k-X.Y.Z.bin

   Answer `y`. The unit resets into the bootloader: all four LEDs strobe fast. The tool writes the
   new image and starts Klipper again. Klipper must report the new version: `Klipper reports ace2k
   X.Y.Z: done.`

### How the tool works

The tool runs on the Klipper host, as a user allowed to stop and start Klipper. That is root, or
another user with `--klipper-stop "sudo systemctl stop klipper"` and `--klipper-start "sudo
systemctl start klipper"`. Give those two options together. Every option is in
[`commands.md`](commands.md#ace2k_flashpy).

What it does, in order:

1. **Reads the image.**
   - An ace2k image carries its version since the release after `v0.11.0`. An older ace2k image
     needs `--version`. The factory image needs `--version` too.
   - The tool derives the version string it announces to the bootloader from the version: the
     first 11 characters.
   - At the end, the tool recognises an older ace2k image by its own version string. The firmware
     embeds that string. So the tool accepts whatever version Klipper reports, when that exact
     string occurs in the image. Example: `--version 0.11` for an image that reports `0.11.0`.
2. **Finds the port.** It takes `--port`. Without it, it takes the `serial` of the MCU that
   `[ace2k]` uses in Klipper's config. It reads that config through Moonraker (`--moonraker`,
   default `http://localhost:7125`). With neither, it lists the adapters of the kind in the unit's
   cable, and stops.
3. **Identifies the unit.** It acts only on what it identified:
   - ace2k: Klipper reports it on that port.
   - The factory firmware, or the bootloader's recovery: they answer on the port.
   - A first install: Klipper is running but not ready, with `[ace2k]` on that port, because the
     unit still runs the factory firmware. Stopping Klipper is then part of the plan, whether or
     not Klipper holds the port at that moment.

   Anything else ends the run with nothing changed. For example: no answer; a print running or
   paused; Klipper's print state cannot be read; more than one unit answering on the port. The
   tool never opens a port another program holds.

   More than one unit on a port is a daisy chain. An update reaches every unit at address 0, so
   the tool flashes one unit per port. Split the chain first, or move the unit to its own cable.
4. **Shows the plan and asks** `Proceed? [y/N]`. `--yes` skips the question.
5. **Keeps the calibration**, when the unit runs the factory firmware. The unit's id and its
   `GET_MOTOR_STATUS` go to `ace-calibration-<unit id>-<date-time>.txt` in `--calibration-dir`
   (default: the current directory).
6. **From ace2k, puts the unit in recovery and stops Klipper.**
   - It sends `ACE_RESTORE_STOCK`. The console refuses it while the dryer heats or a lane moves.
     The tool then shows the console's reason and changes nothing.
   - It stops Klipper. With `--klipper-stop` / `--klipper-start` given, it uses them: they take
     precedence over the detection. Otherwise it uses `systemctl` when a `klipper` unit exists,
     else `/etc/init.d/S60klipper` when it exists. When neither exists, you must give
     `--klipper-stop` / `--klipper-start`.
   - It waits up to 15 s for Klipper to notice the reset.
7. **Writes the image** (about 20 s for ace2k, 30 s for the factory image).
8. **Checks the result.**
   - **ace2k:** the bootloader no longer answers. Klipper is started again and must report the new
     version on that port. With no `[ace2k]` in the config, the tool cannot check this, and says
     so. A newly written image goes back to recovery if no host talks to it within 180 s. So
     configure Klipper (see [First install](#first-install)) and start it within that window. If
     the window passes, finish the config, restart Klipper and run the tool again.
   - **Factory firmware:** it answers with the version given. The tool reads its calibration again
     and compares it with the newest earlier file of the same unit in `--calibration-dir`. It
     reports "calibration intact", or the pairs that differ. With no earlier file, it skips the
     comparison and says so. Klipper is then left stopped. Before you start it, remove
     `[mcu ace2k]` and `[ace2k]` from the config. That includes any `#*# [ace2k]` section in the
     SAVE_CONFIG block at the end of `printer.cfg`: Klipper refuses to start while that block still
     declares `[ace2k]`.

The tool's exit status:

| Status | Meaning |
|---|---|
| 0 | Flashed and verified. |
| 1 | Refused or failed. This includes an ace2k image that was written but whose version Klipper did not report in time. |
| 2 | Flashed, but it cannot be checked here: no `[ace2k]` on that port in Klipper's config, or Moonraker does not answer. |

On status 1, the message says what state the unit and Klipper are in. To retry a failed update,
run the same command again: recovery accepts a new update.

What a refusal means depends on when it happens:

- **The console refuses `ACE_RESTORE_STOCK`** (the dryer heats, a lane moves). Nothing changed.
- **A refusal after the unit reset.** For example: a second unit found on the port, an answer that
  is not recovery's, a failed Klipper stop, a program holding the port. The unit stays in the
  bootloader's recovery, which is persistent and accepts an update. The tool says so. Klipper will
  not connect until ace2k is written again: run the tool again.
- **Klipper still reports ready after the wait, and no refusal was printed.** The unit may or may
  not have reset. Nothing was stopped. Look at the LEDs: all four strobing fast is recovery. Then
  run the tool again. It identifies recovery on its own.

## Back to the factory firmware

Going back is safe. If an attempt fails, run the same command again to recover. ace2k never writes
the unit's factory calibration pages, and the bootloader's recovery mode accepts a new update.

**You need the factory image before you start.** It is Anycubic's, and it is not distributed here.

- [hakimio](https://github.com/hakimio) published where to get it: Anycubic's own update package
  for the ACE 2 Pro, version 1.1.31, with its password and an already extracted `.bin`. It is in a
  comment of the "ACE Pro 2 Compatibility?" issue of SnapAce (a driver for the first ACE Pro):
  <https://github.com/BlackFrogKok/SnapAce/issues/7#issuecomment-4424875984>.
- Anycubic's API lists that package as `1.1.31`, 46 930 bytes, MD5
  `f7968b5148b77fd0569bb8e138ad8a74`. Check the MD5 before you use it.
- His updater gist (<https://gist.github.com/hakimio/39c71fa7174e699c6470b7c79323b189>) extracts
  the `.bin` from the package.
- Have the `.bin` on the Klipper host first.

1. **Check that your calibration copy exists.** When `ace2k_flash` flashed the unit away from the
   factory firmware, it saved the unit's id and its `GET_MOTOR_STATUS` pairs to
   `ace-calibration-<unit id>-<date-time>.txt`. The file is in `~/ace2k-flash` on the Klipper host,
   or in the `--calibration-dir` you gave. Keep that file: the restore compares against it. If you
   have none, the restore still works. The pages were never touched, and the check at step 5 is
   simply skipped.
2. **Stop anything that heats or moves.** The restore is refused while the dryer heats or a lane
   moves, and a refusal from the console changes nothing. Stop the dryer and any feed, and end any
   print.
3. **Run the tool.** Put the factory image in `~/ace2k-flash` on the Klipper host, next to the
   tool and your calibration copy. Run the tool there, with the image's version:

       cd ~/ace2k-flash
       python3 ace2k_flash.py ACE2_V1.1.31.bin --version 1.1.31

   It shows the plan and asks `Proceed? [y/N]`. Answer `y`. The tool then sends
   `ACE_RESTORE_STOCK` itself, stops Klipper and writes the image (about 30 s).
4. **Watch the LEDs.** After the reset, all four lane LEDs strobe fast while the image is written.
   This is the bootloader's recovery mode: persistent and safe. Then the unit boots the factory
   firmware.
5. **Check the result.** The tool should report the version you gave (`1.1.31`). With a copy from
   step 1, it should also report `calibration intact`. If it lists pairs that differ, keep the
   output and stop there. Do not flash again.
6. **Remove ace2k from Klipper's config before you start Klipper.** The tool leaves Klipper
   stopped. Delete `[mcu ace2k]` and `[ace2k]` from `printer.cfg`. Also delete any `#*# [ace2k]`
   section in the SAVE_CONFIG block at the end of the file: Klipper will not start while that
   block still declares `[ace2k]`. Then start Klipper.

If the unit does not answer at any point, nothing is lost. You do not need to power-cycle it first,
and you can simply run the same command again. See
[If the unit does not answer after a flash](#if-the-unit-does-not-answer-after-a-flash) below.

## The manual procedure

These steps are what the tool does, done by hand. Use them when the tool cannot run on your host.
Every flow on this page has been run on a real unit, and the durations given are the ones measured
there. Below, `PORT` is the USB adapter's serial device, and the updater is hakimio's
`ace2-ota-update.py`.

### Before anything: keep the unit's calibration

The factory firmware answers `GET_MOTOR_STATUS` with 16 pairs of numbers. They are the unit's own
sensor calibration. Read them with hakimio's shell
(<https://gist.github.com/hakimio/551915aa02b7e248721bed672ad46e0b>), and keep the output for each
unit.

You need it to check a restore. ace2k never writes those flash pages, so a restore must reproduce
them byte for byte. Comparing with your copy is how you check that it did.

You do not need to copy anything into `printer.cfg` for the unit to work. The firmware reads the
same values from the unit's own page at every boot (`hardware.md`, "Factory calibration page"),
and uses them for the insert sensors. `ACE_STATUS` shows the bands in use and where they came from.

### Factory firmware → ace2k

    python3 ace2-ota-update.py PORT ace2k-X.Y.Z.bin --version ace2k-X.Y.Z

The version string you announce to the factory updater is limited to 11 characters (`ace2k-1.2.3`
fits). A longer one gets no answer to the upgrade announcement: a silent refusal (a 12-character
string was refused this way). The image already carries the 8-byte trailer the bootloader checks:
`firmware/tools/mkimage.py` appended it.

The upload takes about 20 s, and the unit resets into ace2k at the end. The lane LEDs go dark for
a moment, then chase: one lane at a time, 1 → 4. The updater then polls for the new version over
the original protocol for 30 s, and prints a timeout warning. That is expected: the unit now speaks
Klipper's protocol, not the original one.

Then add the unit to `printer.cfg` and restart Klipper:

    [mcu ace2k]
    serial: /dev/serial/by-id/<the unit's USB adapter>
    baud: 250000
    restart_method: command

    [ace2k]

You need both sections:

- `[ace2k]` sends `ace2k_version` when Klipper identifies the MCU. That is the firmware's proof
  that the link works. Without it, a freshly flashed unit resets into recovery after 180 s
  (`protocol.md`, "The link proof").
- `restart_method: command` makes `FIRMWARE_RESTART` a plain reset of the unit. The unit goes back
  into ace2k through the bootloader's validation, never into recovery.

On a bench with no printer, make the unit the primary MCU. Klipper's `[printer]` section insists on
the two limits even with no kinematics:

    [mcu]
    serial: /dev/serial/by-id/<the unit's USB adapter>
    baud: 250000
    restart_method: command

    [ace2k]
    mcu: mcu

    [printer]
    kinematics: none
    max_velocity: 1
    max_accel: 1

Without Klipper at all, Klipper's own `klippy/console.py PORT` connects to the unit. Type
`ace2k_version` there: that proves the link just the same.

### ace2k → factory firmware

1. Send `ACE_RESTORE_STOCK` from Klipper. On a bench without Klipper running, connect with
   Klipper's `klippy/console.py PORT` instead, and send the firmware command
   `ace2k_bootloader_enter`. That is a low-level command of the unit; it does not exist in
   Klipper's console. The command is refused while the dryer is heating or a lane moves, so stop
   those first. The unit answers first. It stops feeding its watchdog 50 ms later and resets into
   the bootloader's recovery mode, which is persistent. All four lane LEDs strobe within a second.
2. Stop Klipper (`sudo service klipper stop`). Otherwise it reports the MCU as lost and holds the
   port.
3. `python3 ace2-ota-update.py PORT <factory image>.bin --version 1.1.31 --force` (about 30 s).
   - `--version` is the factory image's own version. Change it for a later image.
   - `--force` bypasses the updater's same-version skip. It is needed when the factory firmware is
     already running, and harmless otherwise.
   - In recovery, the updater's first lines report `boot_version=V1.0.2` with `version=V0.0.0`, or
     with the version string of the last upload the bootloader received. That is the sign you are
     in the right place.
   - The factory image is Anycubic's and is not distributed here. Where to get it is in
     [Back to the factory firmware](#back-to-the-factory-firmware) above.
4. Read `GET_MOTOR_STATUS` again and compare it with the copy you kept. If they are identical, the
   unit is back to its factory state.

Before you start Klipper again, remove `[mcu ace2k]` and `[ace2k]` from `printer.cfg`. Also remove
any `#*# [ace2k]` section in the SAVE_CONFIG block at the end of that file (for example a saved
encoder scale). Klipper will not start while that block still declares `[ace2k]`.

### ace2k → a newer ace2k

Follow the same path as the restore, with the new image instead of the factory one:

1. Send `ACE_RESTORE_STOCK`, or the firmware command `ace2k_bootloader_enter` from `console.py`.
2. Stop Klipper.
3. Run `python3 ace2-ota-update.py PORT ace2k-X.Y.Z.bin --version ace2k-X.Y.Z --force` (about
   20 s). `--force` is harmless here. It is needed when the version string you announce is the one
   the bootloader still reports, as in a rebuild under the same version: the updater skips only an
   identical announced string.

The new image boots unproven. The reflash erases the config page, and the proof is bound to the
version that made it anyway. The image proves itself on the first `ace2k_version`. On the LEDs you
see:

1. the chase, until Klipper is started again (or `console.py` queries the version);
2. then the intro sweep, when every lane is empty;
3. then the lanes' own states.

**The updater's verify step never ends here, and that is expected.** Once every chunk is written,
the updater prints `[verify] Waiting for ACE to reboot and report version …`. Then it prints
`(no response yet)` lines forever. It asks for the version in the original Anycubic protocol, and
an ace2k image answers only the Klipper host. It is not a failure. What to do:

1. Watch the chunk count reach its total and the `[verify]` lines start.
2. Press Ctrl+C to end the updater.
3. Start Klipper. Do it within the 180 s an unproven image waits for the host (see "If the unit
   does not answer after a flash", below).
4. The LEDs are the real confirmation: the chase until Klipper connects, then the intro sweep
   (when every lane is empty). `ACE_STATUS` names the new version.

### If the unit does not answer after a flash

The bootloader validates the application on every reset. An incomplete or invalid image, or a
watchdog reset, puts it into recovery mode: the same mode as step 1 of "ace2k → factory firmware"
above. The updater works there. The four LEDs strobing is that mode.

In that mode, run step 3 of "ace2k → factory firmware" as it is, without a power cycle. The
bootloader re-validates the image on every power cycle, and boots it when it is valid. So a valid
image that never answers would run its 180 s window once more before it comes back to the strobe.

A valid image that runs but never answers on the host link recovers itself. If no host talks to an
ace2k image within 180 s of boot, the image keeps the chase going until then. Then it stops feeding
its watchdog and resets into recovery on its own (`protocol.md`, "The link proof"). A test build
that never answered did so after 181 s.

The three LED states, so you can tell them apart:

- **Chase:** one lane at a time, 1 → 4, 250 ms each. An unproven image is waiting for the host,
  and the window runs.
- **Intro sweep:** 300 ms dark, then 1 → 4 → 1 twice, 150 ms a step, about 2 s. It plays once at
  boot (or when the image is proven), and only while every lane is empty. The image has been
  proven and runs. It never resets itself for lack of a host. After the sweep, each LED shows its
  lane ([Lane LEDs](hardware.md#lane-leds)).
- **All four strobing fast** (~17 Hz): the bootloader's recovery. Step 3 of "ace2k → factory
  firmware" works.

One case still needs the debug header `P1`: an image that answers nothing, *and* feeds its
watchdog, *and* has been proven. No ace2k image does this, because the proof is stored only after
an answer.

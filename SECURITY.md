# Security and safety

ace2k drives a mains heater, two fans and four motors. A bug that lets the heater run when it
should not is a safety problem, not only a software bug. Report it privately, as described below.

## If a unit is misbehaving now

Switch the unit off at the mains and unplug it. Do not wait for a reply here.

## What to report privately

Report any of these in private, not as a public issue:

- the heater on without both fans, or heating past its limits;
- the heater or a motor still running after the host link is lost, after a STOP or after an error;
- a dryer fault (the thermal cutout, an open or shorted thermistor) that does not stop heating;
- a write to flash outside the ace2k configuration page;
- any way to make the unit unrecoverable through its serial port;
- any frame or command that does any of the above.

A fault you can only reproduce on a modified unit or with a modified image is still worth
reporting. Say what was modified.

## How to report

Use **Security → Report a vulnerability** on this repository. That opens a private advisory that
only the maintainer can see. Include:

- the ace2k version (the release tag, or `git describe` for a build from source);
- the Klipper version on the host;
- the steps, the commands sent and the Klipper log;
- what the unit did, and what you expected.

This is a one-maintainer project. Expect a first reply within a week. A fix for a confirmed safety
problem comes before any other work.

## Supported versions

Only the latest release gets fixes. Before you report, check that the problem is still there on
the latest release.

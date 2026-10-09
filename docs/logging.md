# Logging a long run

Two small scripts in `scripts/` record what the unit does over hours, and turn the record into a
readable report. They were written for the project's own long-run test (the burn-in). They are
just as useful when something goes wrong on your printer: a log of the hours before a problem says
far more than a description of it.

## `scripts/ace2k_logger.py` — the recorder

The recorder runs on the Klipper host, next to Klipper and Moonraker. It reads everything through
Moonraker's HTTP API. It **never opens the unit's port and sends no G-code**, so it cannot disturb
a print.

Every 10 s it appends one CSV row to `ace2k-YYYY-MM-DD.csv`. The row holds:

- the print state;
- the unit's version and health;
- temperatures and humidity;
- each lane's state and counters;
- the dryer;
- Klipper's own link statistics for the unit's MCU.

It also writes every new `ace2k` console line to `ace2k-events-YYYY-MM-DD.jsonl`. You get one
pair of files per day. Nothing it writes names the unit, the printer or a print.

It needs only the host's own Python 3 (3.7 or later, standard library). To start it:

    mkdir -p ~/ace2k-log
    python3 scripts/ace2k_logger.py --out ~/ace2k-log

Options:

- `--url` points it at another Moonraker (default `http://localhost:7125`).
- `--period` changes the interval, in seconds (default 10).

Leave it running in the background: `nohup … &`, a service, or your host's start-up hooks. On the
Snapmaker U1, the adapter's install guide shows how to start it with every Klipper start:
[The burn-in logger (optional)](https://github.com/tobecwb/ace2k-u1/blob/main/docs/install.md#the-burn-in-logger-optional).

## `scripts/ace2k_burnin_report.py` — the report

The report runs anywhere with Python 3, on a copy of the logger's directory:

    python3 scripts/ace2k_burnin_report.py ~/ace2k-log > report.md

It writes a Markdown summary with:

- the totals: hours printing, drying cycles;
- one row per print, with the lanes that worked in it;
- one row per day;
- the console events left to look at.

It also scores the run against the project's burn-in targets: at least 20 hours of printing in at
least five prints, every lane used, and two drying cycles, one of them during a print. For a run
of your own, read those as a scale, not as a pass mark.

## Reporting a problem

Attach the report and, if you can, the day's CSV and events files. They carry no identifier of
your unit or printer. Also add:

- what you saw: the time, the lane, the LEDs;
- the output of `ACE_STATUS` and `ACE_HEALTH` from the console.

Before you post anything else (`klippy.log`, your `printer.cfg`), remove your unit's USB serial
number from it. It appears in the
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_<serial>-if00` path.

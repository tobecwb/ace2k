# The USB cable

The ACE 2 Pro has no USB port. It talks to the host over RS-485, on the rear 6-pin port
([`hardware.md`](hardware.md), "Host link"). The USB-to-serial converter is inside Anycubic's own
cable, in the large molded connector at the printer end. It is a CH343, with USB ID `1a86:55d3`.

To plug the unit into a Klipper host, keep that cable and give its printer end a USB plug.

The multiACE project uses the same cable. Its cable guide has photos and more detail:
<https://postapocalyptic-diy.com/ace-pro-1-2-cable-guide/>. Note that multiACE itself does not
work with ace2k today. multiACE drives the unit's factory firmware, while ace2k replaces that
firmware and makes the unit a Klipper MCU.

## What you need

- **Anycubic's ACE 2 Pro cable for the Kobra 3 / Kobra 3 Max / Kobra S1.** It must be the genuine
  one, with the converter molded into its 4-pin end. The chip *is* the USB device: a cable without
  it never shows up as a USB device, whatever the wiring.
- A **female 2x2 Molex-style connector** that mates with the cable's 4-pin end. Or cut that end
  off and use its wires.
- A **USB-A plug** you can wire: a solderless screw-terminal plug, or a cut USB cable.
- A multimeter.

Plug the cable's 6-pin end into the unit's 6-pin host port, as usual. Do not use the 4-pin chain
port: it carries no power, and the converter is powered by the unit.

## Wiring

The positions on the cable's 4-pin end are numbered 1–4, as in the common pinout drawing of
this connector, with the latch marked:

| Position | Signal | USB-A |
|---|---|---|
| 1 | D− | D− |
| 2 | D+ | D+ |
| 3 | not used | — |
| 4 | GND | GND |

Leave the USB **VCC (+5 V) unconnected**. The converter takes its power from the unit, not from
the host.

**Identify the pins by measuring them, not from a drawing.** Drawings online show either the
plug's face or the socket's, and one is the mirror of the other. Power the unit and leave the USB
side unconnected. On the unit described here, the measurements were:

- **GND** is the only position with continuity straight through the cable to the unit's side.
- **D+** sits at about 3.3 V against GND. This is the converter's pull-up that announces a
  full-speed device.
- **D−** sits at about 0 V.
- The fourth position is the unused one.

The wire colours of a pigtail are not a standard. On the cable built here they were yellow GND,
red D+, black D− and white unused. Measure yours.

**Check the USB plug too.** One 5-way screw-terminal USB-A plug had `S / + / D− / D+ / −`
printed, but its print was mirrored: the real order was `S / − / D+ / D− / +`. Before you wire it,
plug the empty connector into a computer's USB port and measure. +5 V is between the real VCC and
GND terminals. Wire to the measured terminals, not to the print.

## Check that it works

Power the unit, plug the USB end into the host, then run:

    lsusb | grep -i 1a86:55d3
    ls /dev/serial/by-id/

You should see:

- a QinHeng device `1a86:55d3`;
- `usb-1a86_USB_Single_Serial_<serial>-if00`. That whole path is what `serial:` takes in
  `[mcu ace2k]`.

Filter `lsusb` by that ID. A search for another ID finds nothing, and looks like a dead cable.

If you see nothing at all in `lsusb`, and nothing in `dmesg -w` while you plug it in, the chip is
not powered or has no ground. Check that the unit is on, that the 6-pin end is in the host port,
and check the GND wire.

## Without Anycubic's cable — not tested

Without the genuine ACE 2 Pro cable for the Kobra 3 / Kobra 3 Max / Kobra S1, a generic
USB-to-RS-485 adapter on the unit's 6-pin host port may work. The unit's link is plain RS-485.
**This has not been tested with ace2k**, and you may need to change the setup or the code. What is
known:

- The 6-pin host port carries `B`, VCC, `A` and GND on pins 1–4 ([`hardware.md`](hardware.md),
  "Host link"). The adapter takes `A`, `B` and GND. The port's VCC exists to power the converter in
  Anycubic's cable. An adapter powered from USB does not need it.
- The link is half duplex. The adapter must switch direction on its own (most USB-RS-485 adapters
  do). It must run at 250000 baud for ace2k, and at 230400 for the factory firmware and the
  bootloader, which the flasher uses.
- In `serial:`, Klipper takes the adapter's own `/dev/serial/by-id/` path. When `ace2k_flash.py`
  cannot find the port, it lists only adapters of the kind in Anycubic's cable. With another
  adapter, give it `--port`, or put the path in Klipper's config first.

Use one cable per unit. ace2k drives one unit per serial port
([`decisions/0002-one-unit-per-serial-port.md`](decisions/0002-one-unit-per-serial-port.md)).

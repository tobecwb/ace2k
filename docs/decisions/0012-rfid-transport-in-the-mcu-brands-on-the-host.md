# 0012 — RFID: the transport in the MCU, the brands on the host

Date: 2026-09-26 · Status: accepted · Amends: 0004 (its RFID part)

## Context
ADR 0004 put "the RFID pipeline with every format decoder" in the MCU, so that a tag would be read
and decoded with no host. Two things came out of specifying the RFID feature:
- New spool brands keep appearing, each with its own key derivation and layout. A user must be
  able to add one without a new firmware and a reflash.
- Nothing the unit does on its own needs a decoded record. The host arms the dryer's setpoint (a
  tag never starts the heater). The insert sequence only needs to know that a tag was found and
  read. The bytes can wait for the host.

What cannot leave the MCU is the timing:
- stopping a lane when its tag is in front of the antenna;
- telling which of two lanes on one antenna a tag belongs to;
- running the ISO 14443-A exchanges within the tag's time in the field.

## Decision
The MCU owns the transport:
- the readers' field;
- the tag protocol: activation, anticollision, selection, MIFARE authentication with a key it is
  handed, reads of blocks and pages;
- the watch of the lanes, which turns a field on only while needed;
- the attribution of a tag to a lane, by that lane's motion;
- the load's search for the tag, and a bounded read session.

It never writes to a tag, and it carries no brand's key and no brand's layout.

The host owns the brands. One Python module per brand derives the keys from the UID and decodes
the bytes into one record shape. Every brand parameter lives in `config/ace2k_tags.cfg`, each with
the source it was published in. A tag no module recognises still counts, by its UID.

## Consequences
- A new brand is a host file, one registry line and one configuration section. The firmware does
  not change.
- No tag is decoded while the host is down. A session with no host answering ends within half a
  second, and the lane waits for the next move.
- The public firmware carries no key material.
- The wire carries the tag's raw bytes, 16 at a time, and the host's steps. A read of a whole
  NTAG213 or of the Bambu record is a handful of round trips.

## Alternatives rejected
- Every decoder in the MCU, one file per format (ADR 0004 as written): a reflash for every new
  brand, and the keys in the public image.
- Everything on the host over a raw SPI tunnel: the timing of the search and of the attribution
  would ride on the host's scheduling, and the link would carry every register access.

"""Run the flasher: `python3 -m ace2k_flash`, and the entry of ace2k_flash.pyz."""

from .flasher import main


def run() -> None:
    """Run the command line and exit with its status (flasher.py lists the statuses)."""
    raise SystemExit(main())


if __name__ == "__main__":
    run()

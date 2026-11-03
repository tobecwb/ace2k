"""make host-bundle: the files ace2k-u1's install.sh takes from an ace2k release."""

from __future__ import annotations

import subprocess
import tarfile
from pathlib import Path

from test_ace2k_flash_help import help_as, run_help

FIRMWARE = Path(__file__).resolve().parents[2]


def test_the_host_bundle_holds_the_host_files_and_nothing_else(tmp_path: Path) -> None:
    build = tmp_path / "build"
    subprocess.run(
        ["make", "-C", str(FIRMWARE), "host-bundle", "ACE2K_VERSION=9.9.9-test", f"BUILD={build}"],
        check=True,
        capture_output=True,
    )
    with tarfile.open(build / "ace2k-host-9.9.9-test.tar.gz") as tar:
        names = {m.name for m in tar.getmembers() if m.isfile()}
        flasher = tar.extractfile("firmware/tools/ace2k_flash.pyz")
        assert flasher is not None
        pyz = tmp_path / "ace2k_flash.pyz"
        pyz.write_bytes(flasher.read())
    extras = {n for n in names if n.startswith("klippy/extras/")}
    assert "klippy/extras/ace2k.py" in extras
    assert "klippy/extras/ace2k_feed.py" in extras
    assert "klippy/extras/ace2k_tags/__init__.py" in extras
    assert all(n.endswith(".py") for n in extras)
    assert names - extras == {
        "config/ace2k.cfg",
        "config/ace2k_tags.cfg",
        "firmware/tools/ace2k_flash.pyz",
        "LICENSE",
    }
    assert not any("__pycache__" in n or "/._" in n or n.endswith(".DS_Store") for n in names)
    # the flasher in the bundle is the one file, and it runs
    result = run_help(pyz)
    assert result.returncode == 0, result.stderr
    assert result.stdout == help_as("ace2k_flash.pyz")


def test_the_release_job_attaches_the_host_bundle() -> None:
    ci = (FIRMWARE.parent / ".github" / "workflows" / "ci.yml").read_text()
    release = ci[ci.index("  release:") :]
    assert "make -C firmware host-bundle" in release
    assert "ace2k-host-*.tar.gz" in release.split("SHA256SUMS")[0]  # in the sha256sum line
    assert "firmware/build/ace2k-host-*.tar.gz" in release

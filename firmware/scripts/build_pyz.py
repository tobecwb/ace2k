"""Build build/ace2k_flash.pyz: the flasher package as one runnable file, byte-reproducible.

    python3 scripts/build_pyz.py tools/ace2k_flash build/ace2k_flash.pyz

The file is a zip archive behind a `#!/usr/bin/env python3` line, as `python3 -m zipapp` makes
one.  It differs from zipapp's in one way: the entries are sorted and carry a fixed time and
mode, so the same source always gives the same bytes.  The entries are stored, not compressed.
"""

from __future__ import annotations

import sys
import zipfile
from pathlib import Path

FIXED_TIME = (2000, 1, 1, 0, 0, 0)


def build(package: Path, out: Path) -> None:
    """Write package's .py files into out, sorted, with fixed times, and a shebang."""
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp")
    with tmp.open("wb") as raw:
        raw.write(b"#!/usr/bin/env python3\n")
        with zipfile.ZipFile(raw, "w", compression=zipfile.ZIP_STORED) as zf:
            for path in sorted(package.rglob("*.py")):
                name = path.relative_to(package.parent).as_posix()
                info = zipfile.ZipInfo(name, FIXED_TIME)
                info.external_attr = 0o644 << 16
                zf.writestr(info, path.read_bytes())
            main = zipfile.ZipInfo("__main__.py", FIXED_TIME)
            main.external_attr = 0o644 << 16
            zf.writestr(main, b"from ace2k_flash.__main__ import run\nrun()\n")
    tmp.chmod(0o755)
    tmp.replace(out)


if __name__ == "__main__":
    build(Path(sys.argv[1]), Path(sys.argv[2]))

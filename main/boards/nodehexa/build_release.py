#!/usr/bin/env python3
"""Build and package the Chinese NodeHexa release with 你好小智.

Run after sourcing ESP-IDF export.sh. Uses the canonical upstream build entry
point, which resets configuration and validates the selected WakeNet model.
"""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]


def main() -> None:
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/build.py"),
            "nodehexa",
            "--name", "nodehexa",
            "--language", "zh-CN",
            "--wake-word", "nihaoxiaozhi",
            "--zip",
        ],
        cwd=ROOT,
        check=True,
    )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Build NodeHexa with the existing MultiNet custom wake-word implementation.

The upstream CLI currently selects WakeNet models only. Reuse its configuration,
validation, build and packaging functions without changing shared build behavior.
Run after sourcing ESP-IDF export.sh. This script never flashes a device.
"""

import importlib.util
import json
import os
from pathlib import Path


BOARD_DIR = Path(__file__).resolve().parent
ROOT = BOARD_DIR.parents[2]
PROFILE = BOARD_DIR / "wake_word_hai_xiaoling.defaults"


def main() -> None:
    os.chdir(ROOT)
    spec = importlib.util.spec_from_file_location("xiaozhi_build", ROOT / "scripts/build.py")
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)
    # Fail before changing local build state if the SDK environment is missing.
    build._get_idf_command()

    config = json.loads((BOARD_DIR / "config.json").read_text(encoding="utf-8"))
    if config["target"] != "esp32s3" or config["type"] != "nodehexa":
        raise ValueError("This wake-word profile requires the NodeHexa ESP32-S3 BSP")
    variant = next(item for item in config["builds"] if item["name"] == "nodehexa")
    options = list(variant["sdkconfig_append"])
    options.extend(
        line.strip()
        for line in PROFILE.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )
    board_symbol = build._resolve_board_config(
        "nodehexa", "esp32s3", options, variant_name="nodehexa"
    )
    if board_symbol != "CONFIG_BOARD_TYPE_NODEHEXA":
        raise ValueError("Unexpected board selection")

    print("Building NodeHexa: MultiNet6 Chinese, 嗨小灵 (hai xiao ling)", flush=True)
    os.environ.pop("IDF_TARGET", None)
    build._prepare_target("esp32s3", False)
    build._configure_build("esp32s3", options, "nodehexa", False)
    build._validate_configured_options(options, "NodeHexa 嗨小灵 profile")
    configured = set(Path("sdkconfig").read_text(encoding="utf-8").splitlines())
    for option in options:
        if option.partition("=")[2] not in {"y", "n"} and option not in configured:
            raise ValueError(f"Kconfig rejected the wake-word setting: {option}")
    build._run_idf("build")
    build.merge_bin()
    # Keep the OTA board identity unchanged; distinguish the local release filename.
    build.zip_bin("nodehexa-hai-xiaoling", build.get_project_version())


if __name__ == "__main__":
    main()

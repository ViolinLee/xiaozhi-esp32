"""Verify the NodeHexa release profile and production action aliases on the host."""

import importlib.util
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / "main/boards/nodehexa"


class NodeHexaReleaseTests(unittest.TestCase):
    def test_release_entry_selects_standard_chinese_wake_word(self):
        spec = importlib.util.spec_from_file_location("nodehexa_release", BOARD / "build_release.py")
        release = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(release)
        with patch.object(release.subprocess, "run") as run:
            release.main()
        run.assert_called_once_with(
            [sys.executable, str(ROOT / "scripts/build.py"), "nodehexa",
             "--name", "nodehexa", "--language", "zh-CN",
             "--wake-word", "nihaoxiaozhi", "--zip"],
            cwd=ROOT, check=True,
        )

    def test_performance_aliases_keep_canonical_uart_names(self):
        compiler = shutil.which("c++")
        if compiler is None:
            self.skipTest("requires a host C++ compiler")
        driver = r'''
#include <cassert>
#include "nodehexa_actions.h"
int main() {
    using nodehexa_actions::Performance;
    const char* names[] = {"showtime", "freestyle", "beatsway", "attack", "greeting", "curiosity"};
    for (const auto* name : names) assert(Performance(name) == name);
    assert(Performance("威慑攻击") == "attack");
    assert(Performance("Threat_Display") == "attack");
    assert(Performance("迎宾致意") == "greeting");
    assert(Performance("Welcome-Ritual") == "greeting");
    assert(Performance("好奇探察") == "curiosity");
    assert(Performance("Curious Explorer") == "curiosity");
    assert(Performance("forward").empty());
    assert(Performance("unknown").empty());
    assert(nodehexa_actions::Motion("attack").empty());
    assert(nodehexa_actions::Motion("停止") == "standby");
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            source = path / "actions.cc"
            binary = path / "actions"
            source.write_text(driver)
            subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I", str(BOARD), str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()

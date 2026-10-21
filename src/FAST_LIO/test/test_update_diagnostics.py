#!/usr/bin/env python3
"""Compile and exercise the production IKFoM update with/without diagnostics."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class UpdateDiagnosticsTest(unittest.TestCase):
    def test_observation_and_invalid_updates(self):
        cache = (ROOT.parents[1] / 'build/CMakeCache.txt').read_text()
        compiler = re.search(r'^CMAKE_CXX_COMPILER:[^=]+=(.+)$', cache, re.M).group(1)
        with tempfile.TemporaryDirectory() as tmp:
            executable = str(Path(tmp) / 'test_update_diagnostics')
            subprocess.run([compiler, '-std=c++14', '-O1', '-fopenmp',
                            '-I/usr/include/eigen3', '-I'+str(ROOT/'include'),
                            str(ROOT/'test/test_update_diagnostics.cpp'),
                            '-o', executable], check=True)
            subprocess.run([executable], check=True)

if __name__ == '__main__':
    unittest.main()

"""Device health versus range validity; Windows: run in a VS developer prompt."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ToFHealthTests(unittest.TestCase):
    def test_real_obstacle_classifier_and_range_encoding(self):
        compiler = shutil.which('cl') or shutil.which('gcc') or shutil.which('clang')
        if compiler is None:
            self.skipTest('A native cl, gcc or clang C compiler is required')
        source = (ROOT / 'firmware/sensor_pico/src/tof.c').read_text(encoding='utf-8')
        # Compile the production wire helpers; no hardware/I2C calls are needed.
        helpers = ['#include "tof.h"']
        for name in ('tof_result_code', 'tof_distance_get', 'tof_distance_set'):
            match = re.search(r'(?:uint16_t|void) ' + name + r'\([^)]*\)\s*\{.*?\n\}', source, re.S)
            self.assertIsNotNone(match, name)
            helpers.append(match.group())
        build_dir = ROOT / 'build'
        build_dir.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=build_dir, prefix='tof-health-') as temp:
            temp = Path(temp)
            helper_file = temp / 'tof_helpers.c'
            helper_file.write_text('\n'.join(helpers), encoding='utf-8')
            executable = temp / 'test_tof_health.exe'
            sources = [
                str(ROOT / 'tests/test_obstacle.c'),
                str(ROOT / 'firmware/sensor_pico/src/obstacle.c'),
                str(helper_file),
            ]
            includes = str(ROOT / 'firmware/sensor_pico/include')
            if Path(compiler).name.lower() == 'cl.exe':
                command = [compiler, '/nologo', '/std:c11', '/W4', '/WX', '/I', includes,
                           *sources, '/Fo:' + str(temp) + '/', '/Fe:' + str(executable)]
            else:
                command = [compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                           '-I', includes, *sources, '-o', str(executable)]
            compile_result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            self.assertEqual(compile_result.returncode, 0, compile_result.stdout + compile_result.stderr)
            run_result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=20)
            self.assertEqual(run_result.returncode, 0, run_result.stdout + run_result.stderr)
            self.assertIn('Obstacle detector/fault tests passed', run_result.stdout)


if __name__ == '__main__':
    unittest.main()

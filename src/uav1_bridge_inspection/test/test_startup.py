#!/usr/bin/env python3
"""解析真实 launch，截获 Terminator 配置，不启动飞控或传感器。"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

from configobj import ConfigObj


class StartupTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='bridge_startup_test_')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.script = Path(__file__).resolve().parents[3] / 'shfiles/start_uav1_bridge_inspection.sh'
        self.capture = self.root / 'layout.conf'
        # 使用真实 roslaunch --nodes，仅替代最后打开窗口的命令。
        fake = self.root / 'terminator'
        fake.write_text('''#!/usr/bin/python3
import os, shutil, sys
source = next(arg.split('=', 1)[1] for arg in sys.argv if arg.startswith('--config='))
shutil.copyfile(source, os.environ['BRIDGE_TEST_CAPTURE'])
''')
        fake.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.root) + os.pathsep + os.environ['PATH'],
                        BRIDGE_TEST_CAPTURE=str(self.capture))

    def run_script(self, *args):
        return subprocess.run(['bash', str(self.script), *args], env=self.env,
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=30, check=True)

    def test_no_arguments_opens_six_panes(self):
        self.run_script()
        config = ConfigObj(str(self.capture), encoding='utf-8')
        self.assertEqual(config['profiles']['default']['exit_action'], 'hold')
        layout = config['layouts']['uav1_bridge']
        terminals = [v for v in layout.values() if v['type'] == 'Terminal']
        self.assertEqual(len(terminals), 6)
        expected = ['mavros', 'mid360', 'fastlio', 'pose', 'camera', 'mission']
        for terminal, pane in zip(terminals, expected):
            self.assertEqual(shlex.split(terminal['command']),
                             ['bash', str(self.script), '--pane', pane])
        for parent, children in [('root', ['top', 'bottom']), ('top', ['pane0', 'top_right']),
                                 ('top_right', ['pane1', 'pane2']),
                                 ('bottom', ['pane3', 'bottom_right']),
                                 ('bottom_right', ['pane4', 'pane5'])]:
            actual = sorted((key for key, v in layout.items() if v['parent'] == parent),
                            key=lambda key: int(layout[key]['order']))
            self.assertEqual(actual, children)

    def test_overrides_and_path_with_spaces(self):
        route = self.root / 'route with spaces.yaml'
        route.write_text('waypoints: [[1.0, 0.0, 2.0]]\n')
        args = ['ground_z:=-0.2', 'landing_x:=12.5', 'landing_y:=-1.0',
                'route_file:=' + str(route)]
        self.run_script(*args)
        layout = ConfigObj(str(self.capture), encoding='utf-8')['layouts']['uav1_bridge']
        for terminal in (v for v in layout.values() if v['type'] == 'Terminal'):
            self.assertEqual(shlex.split(terminal['command'])[4:], args)

    def test_help_does_not_open_windows(self):
        result = self.run_script('--help')
        self.assertIn('六分屏', result.stdout)
        self.assertFalse(self.capture.exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""Run with python3 -m unittest discover -s src/cooperative_mapping -v."""
import ast
import hashlib
import importlib
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
from scipy import ndimage as ndi

from core import Grid, agreement, inverse, merge, must_yield, register, separation_mask, transform
from prepare import patch_explorer, prepare


def scene(seed=42):
    import cv2
    rng = np.random.default_rng(seed)
    a = np.zeros((500, 650), dtype=np.int8)
    cv2.rectangle(a, (20, 20), (625, 475), 100, 3)
    for _ in range(65):
        x, y = rng.integers([45, 45], [580, 425])
        w, h = rng.integers(5, 35, size=2)
        cv2.rectangle(a, (x, y), (x+w, y+h), 100, 2)
        # Asymmetric tails provide distinct corners rather than identical boxes.
        cv2.line(a, (x+w, y+h), (x+w+int(w/2), y+h+int(h/3)), 100, 2)
    return Grid(a, 0.05, -5.0, -4.0)


def reframe(grid, pose, yaw=0.0):
    """Independent inverse-sampled snapshot in another coordinate frame."""
    # Large enough for arbitrary source offset/rotation in these tests.
    rows, cols = np.mgrid[:850, :850]
    template = Grid(np.zeros((850, 850), dtype=np.int8), 0.05, -12.0, -12.0, yaw)
    points = transform(template.points(rows.ravel(), cols.ravel()), pose)
    cells = grid.cells(points)
    valid = ((cells[:, 0] >= 0) & (cells[:, 1] >= 0)
             & (cells[:, 0] < grid.data.shape[0]) & (cells[:, 1] < grid.data.shape[1]))
    data = np.full(rows.size, -1, dtype=np.int8)
    j = np.flatnonzero(valid)
    data[j] = grid.data[cells[j, 0], cells[j, 1]]
    return Grid(data.reshape(rows.shape), *template.geometry)


class CoreTests(unittest.TestCase):
    def test_frame_roundtrip(self):
        pose = (3.2, -1.4, 1.2)
        p = np.array([[0, 0], [2.4, -1.7], [-8.1, 4.3]])
        np.testing.assert_allclose(transform(transform(p, pose), inverse(pose)), p, atol=1e-12)

    def test_packet_and_invalid_payloads(self):
        g = Grid(np.array([[-1, 0, 100]], np.int8), 0.05, -2.0, 3.0, 0.4)
        packet = g.packet()
        np.testing.assert_array_equal(Grid.unpack(packet).data, g.data)
        packet['shape'] = [1, 2]
        with self.assertRaises(ValueError):
            Grid.unpack(packet)
        packet = g.packet()
        packet['revision'] = 'bad'
        with self.assertRaises(ValueError):
            Grid.unpack(packet)

    def test_rotated_origin_cells(self):
        g = Grid(np.zeros((20, 25), np.int8), 0.05, -4, 2, math.pi/3)
        r, c = np.mgrid[:20, :25]
        cells = g.cells(g.points(r.ravel(), c.ravel()))
        np.testing.assert_array_equal(cells, np.column_stack([r.ravel(), c.ravel()]))

    def test_merge_unknown_and_obstacle_precedence(self):
        a = Grid(np.array([[0, -1], [100, 0]], np.int8), 0.05, 0, 0)
        b = Grid(np.array([[100, 0], [0, -1]], np.int8), 0.05, 0, 0)
        result = merge(a, b, (0, 0, 0))
        np.testing.assert_array_equal(result.data, [[100, 0], [100, 0]])
        np.testing.assert_array_equal(merge(b, a, (0, 0, 0)).data, result.data)
        self.assertEqual(result.packet()['revision'], merge(a, b, (0, 0, 0)).packet()['revision'])

    def test_merge_offset_and_no_feedback(self):
        a = Grid(np.zeros((3, 3), np.int8), 0.1, 0, 0)
        b = Grid(np.full((3, 3), 100, np.int8), 0.1, 0, 0)
        result = merge(a, b, (1, 0, 0))
        self.assertGreaterEqual(result.data.shape[1], 13)
        self.assertEqual(result.data[0, 5], -1)
        self.assertEqual(result.data[0, 10], 100)
        # A later raw snapshot can remove old obstacles (no accumulated feedback).
        b2 = Grid(np.zeros((3, 3), np.int8), 0.1, 0, 0)
        self.assertEqual(merge(a, b2, (1, 0, 0)).data[0, 10], 0)

    def test_registration_arbitrary_start(self):
        target = scene()
        expected = (1.7, -2.1, 0.52)
        source = reframe(target, expected, yaw=-0.2)
        found = register(target, source)
        self.assertIsNotNone(found)
        pose, quality = found
        self.assertLess(math.hypot(pose[0]-expected[0], pose[1]-expected[1]), 0.12)
        self.assertLess(abs(pose[2]-expected[2]), 0.03)
        self.assertGreater(quality['wall_agreement'], 0.8)
        self.assertIsNotNone(agreement(target, source, pose))
        self.assertIsNone(agreement(target, source, (10, 10, 0)))

    def test_registration_refuses_unrelated_and_empty_maps(self):
        self.assertIsNone(register(scene(42), scene(17)))
        empty = Grid(np.full((100, 100), -1, np.int8), 0.05, 0, 0)
        self.assertIsNone(register(scene(), empty))
        self.assertIsNone(agreement(scene(), empty, (0, 0, 0)))

    def test_registration_partial_overlap_and_unknown_boundaries(self):
        full = scene()
        expected = (-1.1, 2.3, -0.74)
        source = reframe(full, expected)
        a = full.data.copy()
        a[:, 440:] = -1
        target = Grid(a, *full.geometry)
        rr, cc = np.mgrid[:source.data.shape[0], :source.data.shape[1]]
        common_points = transform(source.points(rr.ravel(), cc.ravel()), expected)
        b = source.data.copy().ravel()
        b[common_points[:, 0] < 3.0] = -1
        source = Grid(b.reshape(source.data.shape), *source.geometry)
        found = register(target, source)
        self.assertIsNotNone(found)
        pose, quality = found
        self.assertLess(math.hypot(pose[0]-expected[0], pose[1]-expected[1]), 0.15)
        self.assertLess(abs(pose[2]-expected[2]), 0.03)

    def test_plain_corridor_is_not_enough_for_alignment(self):
        a = np.zeros((100, 700), dtype=np.int8)
        a[5:8, :] = a[92:95, :] = 100
        grid = Grid(a, 0.05, 0, 0)
        self.assertIsNone(register(grid, grid))

    def test_three_meter_reservation_and_tie_break(self):
        mask = np.ones((100, 100), bool)
        filtered = separation_mask(mask, (0.1, -5, -5, 0), [0, 0, 0])
        rows, cols = np.nonzero(filtered)
        p = Grid(np.zeros((1, 1), np.int8), 0.1, -5, -5).points(rows, cols)
        self.assertTrue((np.linalg.norm(p, axis=1) >= 3.0).all())
        self.assertTrue(must_yield('swarm2', 'swarm1', [0, 0], [2.99, 0]))
        self.assertFalse(must_yield('swarm1', 'swarm2', [0, 0], [2.99, 0]))
        self.assertFalse(must_yield('swarm2', 'swarm1', [0, 0], [3, 0]))


class IntegrationPreparationTests(unittest.TestCase):
    def source(self):
        # Tiny representative explorer validates AST placement independently of ROS.
        return '''
from __future__ import annotations
class Other: pass
def main():
    class Explorer(Node):
        def __init__(self):
            super().__init__('explorer')
            self.map_sub = self.create_subscription(Grid, '/map', self.on_map, qos)
            self.timer = self.create_timer(.25, self.step)
        def step(self):
            self.goal = choose_goal(mask, self.geometry, 0, 0, [], .5)
            self.future = self.navigate.send_goal_async(request)
        def finish(self, now, msg, reached=False): pass
        def no_frontier(self, now, x, y): self.start_home(now, x, y)
'''

    def test_preserves_return_home_and_wraps_navigation(self):
        code = patch_explorer(self.source())
        compile(code, 'patched', 'exec')
        self.assertIn('self.cooperation.send_goal', code)
        self.assertIn('self.cooperation.before_step', code)
        self.assertIn('self.cooperation.release()', code)
        self.assertIn('self.cooperation.peer_exploring()', code)
        self.assertIn('self.start_home(now, x, y)', code)
        self.assertIn('/swarm/merged_map', code)
        self.assertLess(code.index('from __future__'), code.index('from cooperative_hooks'))
        with self.assertRaises(ValueError):
            patch_explorer(code)

    def test_unknown_explorer_refused(self):
        with self.assertRaises(ValueError):
            patch_explorer('print("not the project explorer")')

    def test_runtime_copy_does_not_change_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            src = root/'src/wheel_ekf_demo'
            src.mkdir(parents=True)
            names = ['nav2.launch.py', 'mapping.launch.py', 'frontier_explorer.py',
                     'robot_demo.py', 'hardware.py', 'demo_core.py', 'ekf.yaml',
                     'nav2_params_swarm1.yaml', 'manual_nav.xml', 'exploration_nav.xml', 'web_map.py']
            for name in names:
                (src/name).write_text('# nav2_params\n')
            (src/'nav2.launch.py').write_text('from pathlib import Path\nFOLDER = Path(__file__).resolve().parent\n# nav2_params\n')
            (src/'frontier_explorer.py').write_text(self.source())
            (src/'web_map.py').write_text("TOPIC = '/map'\n")
            (src/'nav2_params_swarm1.yaml').write_text('''
global_costmap:
  global_costmap:
    ros__parameters:
      static_layer: {map_topic: /map}
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: ~/Documents/swarm-mono/src/wheel_ekf_demo/manual_nav.xml
''')
            before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in src.iterdir()}
            folder = prepare(root, 'swarm1')
            after = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in src.iterdir()}
            self.assertEqual(before, after)
            self.assertIn('/swarm/merged_map', (folder/'nav2_params_swarm1.yaml').read_text())
            self.assertIn(str(folder/'manual_nav.xml'), (folder/'nav2_params_swarm1.yaml').read_text())
            self.assertIn('/swarm/merged_map', (folder/'web_map.py').read_text())


class ReservationLifecycleTests(unittest.TestCase):
    def setUp(self):
        qos = types.ModuleType('rclpy.qos')
        qos.QoSProfile = lambda **kwargs: kwargs
        qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
        msg = types.ModuleType('std_msgs.msg')
        msg.String = lambda **kwargs: types.SimpleNamespace(**kwargs)
        self.modules = patch.dict(sys.modules, {'rclpy': types.ModuleType('rclpy'),
            'rclpy.qos': qos, 'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': msg})
        self.modules.start()
        sys.modules.pop('cooperative_hooks', None)
        self.hooks = importlib.import_module('cooperative_hooks')
        self.env = patch.dict(os.environ, SWARM_ROBOT_ID='swarm2', SWARM_RUN_ID='run2')
        self.env.start()
        self.messages = []
        self.node = types.SimpleNamespace(
            create_publisher=lambda *a: types.SimpleNamespace(publish=self.messages.append),
            create_subscription=lambda *a: None, create_timer=lambda *a: None,
            get_logger=lambda: types.SimpleNamespace(info=lambda *a: None))
        self.coop = self.hooks.Cooperation(self.node)

    def tearDown(self):
        self.env.stop()
        sys.modules.pop('cooperative_hooks', None)
        self.modules.stop()

    def receive(self, ready=True, exploring=True):
        import json
        self.coop.on_peer(types.SimpleNamespace(data=json.dumps(dict(
            robot='swarm1', ready=ready, goal=[0, 0, 0], kind='explore', exploring=exploring))))

    def test_unaligned_or_stale_peer_never_filters_geometry(self):
        mask = np.ones((10, 10), bool)
        self.receive(ready=False)
        self.assertTrue(self.coop.peer_exploring())
        np.testing.assert_array_equal(self.coop.filter(mask, (0.1, 0, 0, 0))[0], mask)
        self.receive()
        self.coop.peer_at -= 4.0
        self.assertFalse(self.coop.peer_exploring())
        np.testing.assert_array_equal(self.coop.filter(mask, (0.1, 0, 0, 0))[0], mask)

    def test_cancel_once_only_after_goal_handle_then_release(self):
        import json
        self.receive()
        self.coop.goal, self.coop.kind = [1, 0, 0], 'explore'
        cancels = []
        explorer = types.SimpleNamespace(phase='nav_handle', _goal_handle=None, cancel_since=None)
        self.coop.before_step(explorer)
        self.assertFalse(cancels)
        explorer.phase = 'nav_result'
        explorer._goal_handle = types.SimpleNamespace(cancel_goal_async=lambda: cancels.append(1))
        self.coop.before_step(explorer)
        self.coop.before_step(explorer)
        self.assertEqual(len(cancels), 1)
        self.coop.release()
        self.assertIsNone(json.loads(self.messages[-1].data)['goal'])

    def test_home_goal_does_not_yield_as_frontier(self):
        self.receive()
        self.coop.goal, self.coop.kind = [1, 0, 0], 'home'
        explorer = types.SimpleNamespace(phase='nav_result', _goal_handle=None, cancel_since=None)
        self.coop.before_step(explorer)
        self.assertIsNone(explorer.cancel_since)


if __name__ == '__main__':
    unittest.main()

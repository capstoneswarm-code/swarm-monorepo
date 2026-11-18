#!/usr/bin/env python3
"""Run from ~/Documents/swarm-mono with navigation and the viewer stopped.

Adds a per-hardware-run ID, browser trail resets, and 35 cm exploration goals.
Preserves calibrated hardware settings, speed limits, and collision protection.
"""

import ast
from datetime import datetime
from pathlib import Path
import shutil
import tempfile
import yaml


def replace_once(code, old, new):
    count = code.count(old)
    if count != 1:
        raise ValueError(f"Expected one match, found {count}: {old!r}; nothing changed")
    return code.replace(old, new, 1)


def patch_bridge(code):
    if '_navigation_run_id_pub' in code:
        return code
    code = replace_once(code, '    from rclpy.node import Node\n',
        '    from rclpy.node import Node\n'
        '    from std_msgs.msg import String\n'
        '    from uuid import uuid4\n'
        '    from rclpy.qos import DurabilityPolicy, QoSProfile\n')
    classes = [n for n in ast.walk(ast.parse(code)) if isinstance(n, ast.ClassDef) and n.name == 'Bridge']
    if len(classes) != 1:
        raise ValueError('Expected one Bridge class; nothing changed')
    init = next(n for n in classes[0].body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    supers = [n for n in init.body if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call)
              and isinstance(n.value.func, ast.Attribute) and n.value.func.attr == '__init__']
    if not supers:
        raise ValueError('Bridge node initialization not found; nothing changed')
    lines = code.splitlines(keepends=True)
    lines.insert(supers[0].end_lineno,
        '            self._navigation_run_id = str(uuid4())\n'
        '            self._navigation_run_id_pub = self.create_publisher(\n'
        '                String, "/navigation/run_id",\n'
        '                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))\n'
        '            self._navigation_run_timer = self.create_timer(1.0, lambda:\n'
        '                self._navigation_run_id_pub.publish(String(data=self._navigation_run_id)))\n'
        '            self._navigation_run_id_pub.publish(String(data=self._navigation_run_id))\n')
    return ''.join(lines)


def patch_viewer(code):
    if 'self.run_sub =' in code:
        return code
    code = replace_once(code, 'import zlib\n', 'import zlib\nfrom uuid import uuid4\n')
    code = replace_once(code, 'let image=null, meta=null, busy=false, trail=[];',
        'let image=null, meta=null, busy=false, trail=[];\nlet runId=null, viewerId=null;')
    anchor = "  const state=await (await fetch('/state',{cache:'no-store'})).json();"
    code = replace_once(code, anchor, anchor + '''
  if(state.run_id!==runId || state.viewer_id!==viewerId){
   trail=[];
   if(state.viewer_id!==viewerId){meta=null;image=null;}
   runId=state.run_id;viewerId=state.viewer_id;
  }''')
    code = replace_once(code, "        self.status = ''\n",
        "        self.status = ''\n        self.run_id = ''\n        self.viewer_id = str(uuid4())\n")
    code = replace_once(code,
        'pose=self.pose, goal=self.goal, status=self.status), allow_nan=False)',
        'pose=self.pose, goal=self.goal, status=self.status,\n'
        '                run_id=self.run_id, viewer_id=self.viewer_id), allow_nan=False)')
    anchor = "            self.status_sub = self.create_subscription(String, '/exploration/status', self.on_status, display_qos)"
    code = replace_once(code, anchor, anchor + '\n'
        '            self.pose_after_ns = 0\n'
        "            self.run_sub = self.create_subscription(String, '/navigation/run_id', self.on_run, display_qos)")
    code = replace_once(code, '        def on_status(self, msg):\n', '''        def on_run(self, msg):
            with shared.lock:
                if shared.run_id == msg.data:
                    return
                shared.run_id = msg.data
                shared.pose = shared.goal = None
                shared.status = 'New run: waiting for fresh pose'
            self.pose_after_ns = self.get_clock().now().nanoseconds

        def on_status(self, msg):
''')
    anchor = '                stamp = tf.header.stamp.sec*10**9 + tf.header.stamp.nanosec'
    code = replace_once(code, anchor, anchor + '\n'
        '                if stamp < self.pose_after_ns:\n'
        "                    raise ValueError('Waiting for pose from the new run')")
    return code


def make_updates(folder):
    bridge = folder/'robot_demo.py'
    viewer = folder/'web_map.py'
    params = folder/'nav2_params.yaml'
    explorer = folder/'frontier_explorer.py'
    tree = folder/'exploration_nav.xml'
    config = yaml.safe_load(params.read_text())
    controller = config['controller_server']['ros__parameters']
    if ('explore_goal_checker' not in controller['goal_checker_plugins']
            or 'goal_checker_id="explore_goal_checker"' not in tree.read_text()):
        raise ValueError('Install navigation_flow_update first; nothing changed')
    checker = controller['explore_goal_checker']
    checker.update(xy_goal_tolerance=0.35, yaw_goal_tolerance=3.141592653589793, stateful=True)
    follow = controller['FollowPath']
    speed = float(follow['desired_linear_vel'])
    if speed <= 0.50:
        # Do not shorten high-speed braking approaches with this update.
        follow['approach_velocity_scaling_dist'] = 0.60
    follow['min_approach_linear_velocity'] = min(0.12, speed)
    code = explorer.read_text()
    old = 'eligible = np.isfinite(distance) & (distance >= 0.30)'
    new = 'eligible = np.isfinite(distance) & (distance >= 0.50)'
    if old in code:
        code = replace_once(code, old, new)
    elif new not in code:
        raise ValueError('Unexpected frontier eligibility code; nothing changed')
    updates = {
        bridge: patch_bridge(bridge.read_text()),
        viewer: patch_viewer(viewer.read_text()),
        explorer: code,
        params: yaml.safe_dump(config, sort_keys=False)}
    for path, content in updates.items():
        if path.suffix == '.py':
            compile(content, str(path), 'exec')
    return updates


def main():
    workspace = Path.home()/'Documents/swarm-mono'
    if Path.cwd().resolve() != workspace.resolve():
        raise SystemExit('Run from ~/Documents/swarm-mono')
    try:
        updates = make_updates(workspace/'src/wheel_ekf_demo')
    except (OSError, ValueError, KeyError, SyntaxError, StopIteration) as error:
        raise SystemExit(f'Not applied: {error}')
    parent = workspace/'nav_tuning_backups'
    parent.mkdir(exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d-history-goals-%H%M%S-'), dir=parent))
    for path in updates:
        shutil.copy2(path, backup/path.name)
    try:
        for path, content in updates.items():
            path.write_text(content)
    except OSError:
        for path in updates:
            shutil.copy2(backup/path.name, path)
        raise
    print(f'Installed. Backup: {backup}')
    print('New hardware runs reset the browser trail, even with explore:=false.')
    print('Exploration goals complete within 0.35 m with no final heading requirement.')
    print('Restart navigation and the viewer; reload the browser once after installing.')
    print('Speed, collision protection, calibration, and manual goal tolerances preserved.')


if __name__ == '__main__':
    main()

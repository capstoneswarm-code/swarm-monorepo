#!/usr/bin/env python3
"""Start cooperative mapping using a separate runtime copy of the working stack."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from uuid import uuid4

from prepare import prepare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=['swarm1', 'swarm2'], default=socket.gethostname().split('.')[0])
    parser.add_argument('--radius', type=float, required=True)
    parser.add_argument('--hardware-config', type=Path, required=True)
    parser.add_argument('--peer-url', default='http://127.0.0.1:8769')
    parser.add_argument('--peer-ssh', help='USER@PEER_ADDRESS; automatically maintain the SSH tunnel')
    parser.add_argument('--no-explore', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    root = Path.home()/'Documents/swarm-mono'
    if Path.cwd().resolve() != root.resolve():
        parser.error('Run from ~/Documents/swarm-mono')
    if args.robot not in ('swarm1', 'swarm2'):
        parser.error('Set hostname to swarm1/swarm2 or pass --robot')
    if socket.gethostname().split('.')[0] != args.robot:
        parser.error('--robot must match this Pi hostname (the copied launch selects YAML by hostname)')
    if not 0.03 <= args.radius <= 1.0:
        parser.error('Supply the measured robot radius in meters')
    hardware = args.hardware_config.expanduser().resolve()
    if not hardware.is_file():
        parser.error(f'Hardware configuration is missing: {hardware}')
    home = Path(__file__).resolve().parent
    lock = (home/'cooperative.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('Cooperative runner already active on this robot')
    folder = prepare(root, args.robot)
    print(f'Prepared {folder}; original wheel_ekf_demo was not changed.', flush=True)
    if args.prepare_only:
        return
    import cv2  # Check BEFORE starting hardware.
    if not hasattr(cv2, 'SIFT_create'):
        raise RuntimeError('OpenCV lacks SIFT; install Ubuntu python3-opencv')
    import rclpy
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='79' if args.robot == 'swarm1' else '78',
               SWARM_ROBOT_ID=args.robot, SWARM_RUN_ID=str(uuid4()), PYTHONUNBUFFERED='1')
    env['PYTHONPATH'] = str(home)+os.pathsep+env.get('PYTHONPATH', '')
    children = []
    shutting_down = False

    def interrupted(*_):
        nonlocal shutting_down
        shutting_down = True

    for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
        signal.signal(getattr(signal, name), interrupted)
    try:
        if args.peer_ssh:
            key = Path.home()/'.ssh/swarm_peer_ed25519'
            if not key.is_file():
                raise RuntimeError(f'Missing {key}; follow the peer SSH-key setup first')
            children.append(subprocess.Popen(['bash', str(home/'tunnel.sh'), args.peer_ssh],
                env=env, start_new_session=True))
        peer_process = subprocess.Popen([sys.executable, '-u', str(home/'peer_node.py'),
            '--robot', args.robot, '--peer-url', args.peer_url], env=env,
            start_new_session=True, preexec_fn=lambda: os.nice(10))
        children.append(peer_process)
        time.sleep(0.5)
        if peer_process.poll() is not None:
            raise RuntimeError('Peer node could not start; hardware was not launched')
        children.append(subprocess.Popen(['ros2', 'launch', str(folder/'nav2.launch.py'),
            f'robot_radius:={args.radius}', f'hardware_config:={hardware}',
            'explore:='+('false' if args.no_explore else 'true')], env=env, start_new_session=True))
        while not shutting_down:
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f'Stack process exited ({child.returncode}); stopping cooperative run')
            time.sleep(0.2)
    finally:
        # Stop the Nav2/hardware process group first, then the peer node.
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()


if __name__ == '__main__':
    main()

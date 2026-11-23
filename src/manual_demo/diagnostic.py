#!/usr/bin/env python3
"""Short low-speed spin-diagnosis test using YOUR existing drivers and EKF.

Stop other motor/Nav2 programs. Run from ~/Documents/swarm-mono.
Default: tests move the robot automatically. --no-drive does stationary checks.
No project files, board flash settings, or gyro range are changed by this script.
"""
import argparse
import ast
import csv
from datetime import datetime
import hashlib
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import statistics
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from contextlib import ExitStack, closing


class TestFault(RuntimeError):
    pass


# Outgoing wheel targets are bounded independently of the manual-demo speed.
MAX_TEST_WHEEL_SPEED = .14


def motion_checks(results):
    failures=[]
    for name, sign in (('forward',1),('reverse',-1)):
        result=results.get(name,{})
        if any(sign*(result.get(side+'_mean') or 0)<.015 for side in ('left','right')):
            failures.append(name+': both wheels did not sustain the requested direction')
    for name, sign in (('positive_yaw_CCW',1),('negative_yaw_CW',-1)):
        result=results.get(name,{})
        if any(sign*(result.get(key+'_mean') or 0)<.08 for key in ('wheel_w','gyro_w','ekf_w')):
            failures.append(name+': wheel/IMU/EKF rotation was missing, too small, or opposite')
    return failures


def metrics(rows):
    selected=[r for r in rows if r['phase_time']>=.20]
    if not selected:selected=rows
    out={'samples':len(selected)}
    for key in ('cmd_left','cmd_right','left','right','wheel_v','wheel_w','gyro_w','raw_body_w','ekf_v','ekf_w'):
        values=[r[key] for r in selected if r.get(key) is not None and math.isfinite(r[key])]
        out[key+'_mean']=statistics.mean(values) if values else None
        out[key+'_peak']=max(map(abs,values)) if values else None
    for side in ('left','right'):
        out[side+'_moving_fraction']=sum(abs(r[side])>=.015 for r in selected)/max(1,len(selected))
    out['gyro_saturated_samples']=sum(bool(r.get('gyro_saturated')) for r in selected)
    return out


def conclusions(results):
    notes=[]
    for name, result in results.items():
        if name.startswith('settle') or name=='stationary':continue
        w,g=result.get('wheel_w_mean'),result.get('gyro_w_mean')
        if w is not None and g is not None and abs(w)>.12 and abs(g)>.08:
            if w*g<0:
                notes.append(f'{name}: encoder and IMU yaw signs disagree; inspect wheel mapping/signs and IMU mount, without guessing which one.')
            elif not .5 <= abs(w/g) <= 2.0:
                notes.append(f'{name}: yaw magnitudes differ by {abs(w/g):.2f}x; check track width, speed scales and gyro conversion/range.')
        if name in ('positive_yaw_CCW','negative_yaw_CW'):
            sign=1 if name=='positive_yaw_CCW' else -1
            if w is not None and g is not None and sign*w<-.12 and sign*g<-.08:
                notes.append(f'{name}: both sensors report yaw OPPOSITE the positive/negative body command; commanded steering convention needs physical direction verification.')
            if w is not None and g is not None and max(abs(w),abs(g))<.08:
                notes.append(f'{name}: little rotation detected; possible motor deadband, wrong channels or command delivery failure.')
        if result['gyro_saturated_samples']:
            notes.append(f'{name}: raw gyro approached its ADC limit (saturation).')
        if name in ('forward','reverse'):
            sign=1 if name=='forward' else -1
            v=result.get('wheel_v_mean')
            if v is not None and sign*v<-.01:
                notes.append(f'{name}: reported forward velocity has the opposite sign to the body command.')
        raw=result.get('raw_body_w_mean')
        if raw is not None and g is not None and abs(raw)>.1 and abs(raw-g)>.1+abs(raw)*.20:
            notes.append(f'{name}: existing IMU driver output differs from raw-register conversion; verify GYRO_CONFIG and software sensitivity together.')
        ekf=result.get('ekf_w_mean')
        if ekf is not None and w is not None and g is not None and w*g>0 and abs(w)>.12 and abs(g)>.12 and ekf*w<0:
            notes.append(f'{name}: EKF yaw sign disagrees with BOTH inputs; inspect EKF topic configuration and duplicate publishers.')
    return notes


def port_users(port):
    """Read process FD device identities; never kill another process."""
    device=os.stat(port).st_rdev
    users=[]
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name)==os.getpid():continue
        try:
            for fd in (process/'fd').iterdir():
                try:
                    if fd.stat().st_rdev==device:
                        users.append((int(process.name),(process/'comm').read_text().strip()))
                        break
                except OSError:pass
        except (OSError,PermissionError):pass
    return users


def source_ros_if_needed():
    if importlib.util.find_spec('rclpy') is not None:return
    setup=Path('/opt/ros/kilted/setup.bash')
    if setup.is_file() and not os.environ.get('SWARM_DIAG_SOURCED'):
        env=dict(os.environ,SWARM_DIAG_SOURCED='1')
        os.execve('/bin/bash',['bash','-c',
            'source /opt/ros/kilted/setup.bash && exec /usr/bin/python3 "$@"',
            'robot-diagnostics',str(Path(__file__).resolve()),*sys.argv[1:]],env)
    raise TestFault('rclpy unavailable: install/source ROS2 Kilted and use /usr/bin/python3')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path('src/wheel_ekf_demo/hardware.yaml'))
    parser.add_argument('--domain-id',type=int,default={'swarm1':79,'swarm2':78}.get(socket.gethostname().split('.')[0],79))
    parser.add_argument('--no-drive',action='store_true',help='Stationary pipeline checks only')
    parser.add_argument('--no-mapping',action='store_true',help='Do not launch the existing LiDAR/SLAM launch')
    args=parser.parse_args()
    source_ros_if_needed()
    os.environ['ROS_DOMAIN_ID']=str(args.domain_id)
    root=Path.cwd().resolve()
    config_path=args.config.expanduser().resolve()
    if not config_path.is_file():raise TestFault(f'Configuration missing: {config_path}')
    folder=config_path.parent
    for name in ('hardware.py','demo_core.py','robot_demo.py','ekf.yaml'):
        if not (folder/name).is_file():raise TestFault(f'Missing {folder/name}; choose the config beside the autonomous drivers')
    sys.path.insert(0,str(folder))
    import yaml
    config=yaml.safe_load(config_path.read_text())
    core=importlib.import_module('demo_core')
    hw=importlib.import_module('hardware')
    demo=importlib.import_module('robot_demo')
    core.validate_config(dict(config,speed_m_s=.04,key_timeout_s=.8))
    if not .05<=config['track_width_m']<=.5:
        raise TestFault('Track width outside 0.05..0.5 m; inspect the small-chassis configuration first')
    if not Path(config['serial_port']).exists():raise TestFault(f'Motor port missing: {config["serial_port"]}')
    owners=port_users(config['serial_port'])
    if owners:raise TestFault(f'Motor port already open by {owners}; stop those programs first')
    report_dir=root/'diagnostics'/f'{datetime.now():%Y%m%d-%H%M%S}-{socket.gethostname()}'
    report_dir.mkdir(parents=True,exist_ok=False)
    report={'host':socket.gethostname(),'domain':args.domain_id,'config_file':str(config_path),
        'config':config,'source_sha256':{name:hashlib.sha256((folder/name).read_bytes()).hexdigest()
        for name in ('hardware.py','demo_core.py','robot_demo.py','ekf.yaml')},
        'results':{},'notes':[], 'scope':'hardware/command/sensor/EKF checks; no Nav2 planning or ground-truth calibration test'}
    other=root/'src/manual_demo/hardware.yaml'
    if other.is_file() and other.resolve()!=config_path:
        alternative=yaml.safe_load(other.read_text())
        keys=('left_motor','right_motor','left_command_sign','right_command_sign','left_encoder_sign',
              'right_encoder_sign','track_width_m','left_speed_scale','right_speed_scale','imu_mount_rpy_deg')
        differences={key:{'tested':config.get(key),'manual':alternative.get(key)}
                     for key in keys if config.get(key)!=alternative.get(key)}
        report['manual_config_differences']=differences
        if differences:report['notes'].append('Manual/autonomous configuration differences: '+', '.join(differences))
    print(f'Configuration: {config_path}\nDomain: {args.domain_id}\nReport: {report_dir}',flush=True)
    print('Short automatic low-speed tests; Ctrl+C stops. Keep the physical power switch reachable.',flush=True)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry,OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from ament_index_python.packages import get_package_prefix
    from smbus import SMBus
    from tf2_ros import Buffer,TransformListener,TransformException

    def interrupt(*_):raise KeyboardInterrupt
    for name in ('SIGINT','SIGTERM','SIGHUP'):signal.signal(getattr(signal,name),interrupt)
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    stack=ExitStack();children=[];board=None;watchdog=None;watchdog_stop=threading.Event()
    armed=threading.Event();heartbeat=[time.monotonic()];failure=[];write_lock=threading.RLock()
    observer=rclpy.create_node('robot_diagnostics');bridge=None
    executor=SingleThreadedExecutor();executor.add_node(observer)
    last_ekf=[None,-math.inf];last_scan=[None,-math.inf];last_map=[None,-math.inf]
    raw_history=deque(maxlen=1500);last_raw=[None];last_raw_at=[-math.inf]
    latest_gyro=[(0.,0.,0.)];raw_bias=(0.,0.,0.);sensitivity=131.0
    rows=[];latest_wheels=[None];wheel_at=[-math.inf]
    observer.create_subscription(Odometry,'/hardware_demo/odometry/filtered',
        lambda m:last_ekf.__setitem__(slice(None),[m,time.monotonic()]),qos_profile_sensor_data)
    observer.create_subscription(LaserScan,'/scan',lambda m:last_scan.__setitem__(slice(None),[m,time.monotonic()]),qos_profile_sensor_data)
    observer.create_subscription(OccupancyGrid,'/map',lambda m:last_map.__setitem__(slice(None),[m,time.monotonic()]),qos_profile_sensor_data)
    tf=Buffer();listener=TransformListener(tf,observer)
    pub=observer.create_publisher(TwistStamped,demo.NAV_CMD_TOPIC,10)
    csv_file=stack.enter_context((report_dir/'samples.csv').open('w',newline=''))
    fields=['phase','phase_time','time','cmd_left','cmd_right','left','right','wheel_v','wheel_w',
            'gyro_w','raw_body_w','ekf_v','ekf_w','gyro_saturated','raw_gx','raw_gy','raw_gz']
    writer=csv.DictWriter(csv_file,fieldnames=fields);writer.writeheader()

    def spin():
        for _ in range(5):executor.spin_once(timeout_sec=0)

    def query(data):
        board.write(data);deadline=time.monotonic()+.7;reply=b''
        while time.monotonic()<deadline:
            size=board.port.in_waiting
            if size:reply+=board.port.read(min(size,4096))
            time.sleep(.01)
        return reply.decode('ascii',errors='replace')

    class CaptureBus:
        def __init__(self,bus):self.bus=SMBus(bus)
        def __getattr__(self,name):return getattr(self.bus,name)
        def read_i2c_block_data(self,address,register,count):
            data=self.bus.read_i2c_block_data(address,register,count)
            if register==0x3B and len(data)==14:
                sample=struct.unpack('>7h',bytes(data))
                raw=sample[4:7];last_raw[0]=raw;last_raw_at[0]=time.monotonic()
                raw_history.append((time.monotonic(),sample))
            return data

    def stop_motors():
        armed.clear()
        if board is not None:
            try:board.stop()
            except Exception:pass

    def watch():
        while not watchdog_stop.wait(.025):
            if armed.is_set() and time.monotonic()-heartbeat[0]>.20:
                failure.append('Main-loop watchdog expired (>0.20 s); motors stopped')
                stop_motors()

    def phase(name,mode,a,b,duration,ready_required=True):
        print(f'\n{name}: {mode} targets {a:+.3f}, {b:+.3f}; {duration:.2f} s',flush=True)
        start=time.monotonic();next_imu=next_write=next_pub=next_sample=start
        moving=bool(a or b)
        selected=[];previous_cmd=(0.,0.)
        heartbeat[0]=start
        if moving:armed.set()
        try:
            while time.monotonic()-start<duration:
                now=time.monotonic();elapsed=now-start;heartbeat[0]=now
                if failure:raise TestFault(failure[0])
                if any(p.poll() is not None for label,p in children if label=='ekf'):
                    raise TestFault('Diagnostic EKF exited; inspect the report ekf.log')
                frames=board.speeds()
                if frames:
                    latest_wheels[0]=frames[-1];wheel_at[0]=now;bridge.publish_wheels(frames[-1],now)
                if now>=next_imu:
                    latest_gyro[0]=imu.body_gyro();bridge.publish_gyro(latest_gyro[0],imu.gyro_std)
                    next_imu=now+.02
                if ready_required and (now-wheel_at[0]>.35 or now-last_raw_at[0]>.15
                        or now-last_ekf[1]>.5 or bridge.last_filtered is None
                        or now-bridge.last_filtered>.5):
                    raise TestFault('Wheel, IMU, or EKF data became stale; stopped')
                if ready_required and any(observer.count_publishers(t)>1 for t in
                        ('/hardware_demo/wheel_odom','/hardware_demo/imu/data_raw','/hardware_demo/odometry/filtered',demo.NAV_CMD_TOPIC)):
                    raise TestFault('Unexpected second publisher appeared; stopped')
                ramp=min(1.,elapsed/.15) if moving else 0.
                if mode=='twist' and now>=next_pub:
                    msg=TwistStamped();msg.header.stamp=observer.get_clock().now().to_msg()
                    msg.header.frame_id='base_link';msg.twist.linear.x=a*ramp;msg.twist.angular.z=b*ramp
                    pub.publish(msg);next_pub=now+.04
                spin()
                target=bridge.nav_velocity.targets(now) if mode=='twist' else (a*ramp,b*ramp)
                if not moving:target=(0.,0.)
                scale=max(1.,max(map(abs,target))/MAX_TEST_WHEEL_SPEED);target=tuple(v/scale for v in target)
                if latest_wheels[0] is not None:
                    left,right=core.wheel_speeds(latest_wheels[0],config)
                    if moving and (max(abs(left),abs(right))>.25 or abs(latest_gyro[0][2])>2.5):
                        report['abort_sample']=dict(phase=name,left=left,right=right,gyro=latest_gyro[0],raw=last_raw[0])
                        raise TestFault(f'Overspeed: L/R={left:+.3f}/{right:+.3f} m/s, gyro={latest_gyro[0][2]:+.3f} rad/s; stopped')
                    if moving and last_raw[0] and max(map(abs,last_raw[0]))>=32100:
                        raise TestFault('Gyro raw counts approached saturation; stopped')
                if now>=next_write:
                    if not moving:board.stop()
                    else:board.command(*target)
                    next_write=now+.04
                if now>=next_sample and latest_wheels[0] is not None:
                    left,right=core.wheel_speeds(latest_wheels[0],config)
                    velocity,omega=core.body_velocity(left,right,config['track_width_m'])
                    raw=last_raw[0] or (0,0,0)
                    raw_body=core.rotate_sensor(tuple(math.radians((v-bias)/sensitivity)
                        for v,bias in zip(raw,raw_bias)),config['imu_mount_rpy_deg'])
                    ekf=last_ekf[0]
                    row=dict(phase=name,phase_time=elapsed,time=now,cmd_left=target[0],cmd_right=target[1],
                        left=left,right=right,wheel_v=velocity,wheel_w=omega,gyro_w=latest_gyro[0][2],
                        raw_body_w=raw_body[2],ekf_v=ekf.twist.twist.linear.x if ekf else None,
                        ekf_w=ekf.twist.twist.angular.z if ekf else None,
                        gyro_saturated=max(map(abs,raw))>=32100,raw_gx=raw[0],raw_gy=raw[1],raw_gz=raw[2])
                    rows.append(row);selected.append(row);writer.writerow(row);next_sample=now+.02
                time.sleep(.003)
        finally:
            stop_motors();csv_file.flush()
            if selected:
                result=metrics(selected);report['results'][name]=result
                print(f"  ACTUAL outgoing wheel targets peak L/R={result['cmd_left_peak']:.3f}/{result['cmd_right_peak']:.3f} m/s",flush=True)
                print(f"  wheel yaw={result['wheel_w_mean']:+.3f}, IMU yaw={result['gyro_w_mean']:+.3f}, "
                      f"EKF yaw={result['ekf_w_mean'] if result['ekf_w_mean'] is not None else 'missing'} rad/s",flush=True)
        if moving and mode=='twist' and selected:
            if max(report['results'][name]['cmd_left_peak'],report['results'][name]['cmd_right_peak'])<.001:
                raise TestFault('ROS body commands did not reach the existing wheel-command converter')

    try:
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:spin();time.sleep(.02)
        existing={t:observer.count_publishers(t) for t in ('/hardware_demo/wheel_odom','/hardware_demo/imu/data_raw','/hardware_demo/odometry/filtered',demo.NAV_CMD_TOPIC)}
        # Our command publisher is expected; all sensor publishers must be absent.
        if any(existing[t] for t in existing if t!=demo.NAV_CMD_TOPIC) or existing[demo.NAV_CMD_TOPIC]>1:
            raise TestFault(f'Other control/sensor stack is active: {existing}; stop it first')
        node_names={name for name,ns in observer.get_node_names_and_namespaces()}
        if node_names & {'controller_server','bt_navigator','cooperative_motion_gate'}:
            raise TestFault('Nav2/cooperative controller still running; stop its launch first')
        board=stack.enter_context(hw.MotorBoard(config))
        original_write=board.write
        def write(data):
            with write_lock:
                if failure and (data.startswith(b'$spd:') or
                        (data.startswith(b'$pwm:') and data!=b'$pwm:0,0,0,0#')):
                    raise TestFault('Watchdog fault is latched; refusing further motion commands')
                return original_write(data)
        board.write=write
        watchdog=threading.Thread(target=watch,daemon=True);watchdog.start()
        report['board_flash']=query(b'$read_flash#');report['board_battery']=query(b'$read_vol#')
        print(report['board_flash'],report['board_battery'],flush=True)
        match=re.search(r'Battery:([\d.]+)V',report['board_battery'])
        if match and float(match[1])<6:
            raise TestFault('Battery report below 6 V; check the motor supply for this chassis before driving')
        with closing(SMBus(config['i2c_bus'])) as bus:
            report['imu_before']={'who_am_i':bus.read_byte_data(config['imu_address'],0x75),
                'gyro_config':bus.read_byte_data(config['imu_address'],0x1B)}
        imu=stack.enter_context(hw.MPU6050(config,bus_factory=CaptureBus))
        gyro_config=imu.bus.read_byte_data(config['imu_address'],0x1B)
        fs=(gyro_config>>3)&3;sensitivity=(131.,65.5,32.8,16.4)[fs]
        report['gyro']={'register':gyro_config,'full_scale_deg_s':(250,500,1000,2000)[fs],
                        'sensitivity_lsb_per_deg_s':sensitivity}
        print('Keep STILL for the existing driver gyro calibration.',flush=True)
        imu.calibrate()
        samples=list(raw_history)
        raw_bias=tuple(statistics.mean(s[1][4+i] for s in samples) for i in range(3))
        accel=tuple(statistics.mean(s[1][i]/16384 for s in samples) for i in range(3))
        body_gravity=core.rotate_sensor(accel,config['imu_mount_rpy_deg'])
        report['stationary_body_accel_g']=body_gravity
        if body_gravity[2]<.75 or max(map(abs,body_gravity[:2]))>.30:
            report['notes'].append('Stationary accelerometer is not approximately body +Z; verify IMU mounting rotation (assuming the existing ±2g accelerometer configuration).')
        print('Gyro full scale:',report['gyro']['full_scale_deg_s'],'deg/s; body gravity:',body_gravity,flush=True)
        # Test the project's actual command conversion without moving first.
        converter=demo.NavVelocity(config['track_width_m'])
        converter.update(0.,0.,0.,.5);positive=converter.targets(.01)
        converter.update(0.,1.,.04,0.);stale=converter.targets(.01)
        converter.update(0.,0.,.04,0.);expired=converter.targets(10.)
        report['command_unit_checks']={'positive_yaw_wheel_targets':positive,'stale_targets':stale,'expired_targets':expired}
        if not (positive[0]<0<positive[1] and stale==(0.,0.) and expired==(0.,0.)):
            raise TestFault('Existing body-command converter failed direction/freshness tests')
        bridge=demo.make_node(config,nav2=True);executor.add_node(bridge)
        executable=Path(get_package_prefix('robot_localization'))/'lib/robot_localization/ekf_node'
        ekf_log=stack.enter_context((report_dir/'ekf.log').open('w'))
        ekf=subprocess.Popen([str(executable),'--ros-args','-r','__node:=ekf_filter_node',
            '-r','__ns:=/hardware_demo','--params-file',str(folder/'ekf.yaml')],
            stdout=ekf_log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(('ekf',ekf))
        board.start_telemetry()
        phase('startup','direct',0.,0.,5.,ready_required=False)
        if (time.monotonic()-wheel_at[0]>.35 or time.monotonic()-last_ekf[1]>.5
                or bridge.last_filtered is None or time.monotonic()-bridge.last_filtered>.5):
            raise TestFault('No fresh wheel/EKF output after startup; inspect ekf.log')
        mapping=folder/'mapping.launch.py'
        if not args.no_mapping and mapping.is_file() and not observer.count_publishers('/scan') and 'slam_toolbox' not in node_names:
            log=stack.enter_context((report_dir/'mapping.log').open('w'))
            child=subprocess.Popen(['ros2','launch',str(mapping)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            children.append(('mapping',child))
            phase('mapping_startup','direct',0.,0.,7.)
        phase('stationary','direct',0.,0.,2.)
        if not args.no_drive:
            thresholds={}
            # These are empirical start thresholds for short pulses, not calibration.
            for side in ('left','right'):
                for speed in (.04,.06,.08,.10,.12,.14):
                    name=f'{side}_sweep_{speed:.2f}'
                    phase(name,'direct',speed if side=='left' else 0.,
                          speed if side=='right' else 0.,1.2)
                    phase('settle_'+name,'direct',0.,0.,1.)
                    result=report['results'][name]
                    measured=result[side+'_mean']
                    if measured < -.015:
                        raise TestFault(f'{side}: corrected encoder feedback is negative for a forward wheel target; verify physical direction and channel/sign configuration')
                    other_side='right' if side=='left' else 'left'
                    if abs(result[other_side+'_mean'])>.02:
                        raise TestFault(f'{side}: uncommanded wheel reported motion; inspect channel mapping and physical wheel motion')
                    if measured>=.015 and result[side+'_moving_fraction']>=.6:
                        thresholds[side]=speed
                        print(f'{side}: sustained encoder motion first detected at {speed:.2f} m/s target',flush=True)
                        break
                report['wheel_start_targets_m_s']=dict(thresholds)
                if side not in thresholds:
                    raise TestFault(f'{side}: no sustained encoder motion up to {MAX_TEST_WHEEL_SPEED:.2f} m/s; dynamic checks failed. Check physical motion, motor power, channel mapping and the exact serial commands before increasing limits')
            speed=min(MAX_TEST_WHEEL_SPEED,max(.08,max(thresholds.values())+.02))
            yaw=min(2.,2*speed/config['track_width_m'])
            converter.update(0.,0.,0.,yaw)
            turn_targets=converter.targets(.01)
            report['command_unit_checks']['tested_yaw_request']=yaw
            report['command_unit_checks']['converted_turn_targets']=turn_targets
            if any(abs(value)<thresholds[side] for side,value in zip(('left','right'),turn_targets)):
                report['notes'].append(f'Body converter limits turn targets to {turn_targets}, below one or both measured wheel start targets {thresholds}; low-speed steering may stall. Existing converter limits were not changed.')
            for name,a,b in (('forward',speed,0.),('reverse',-speed,0.),
                             ('positive_yaw_CCW',0.,yaw),('negative_yaw_CW',0.,-yaw)):
                phase(name,'twist',a,b,1.2)
                phase('settle_'+name,'direct',0.,0.,1.)
        phase('final_stop_observation','direct',0.,0.,1.5)
        report['command_unit_checks']['live_bridge_expired_targets']=bridge.nav_velocity.targets(time.monotonic())
        scan=last_scan[0];mapping_msg=last_map[0]
        report['lidar']={'received':scan is not None,'age_s':time.monotonic()-last_scan[1] if scan else None}
        if scan:
            report['lidar'].update(range_min=scan.range_min,range_max=scan.range_max,
                infinite=sum(math.isinf(v) for v in scan.ranges),rays=len(scan.ranges),
                near_max=sum(math.isfinite(v) and abs(v-scan.range_max)<.02 for v in scan.ranges))
        report['map']={'received':mapping_msg is not None,'width':mapping_msg.info.width if mapping_msg else None,
                       'height':mapping_msg.info.height if mapping_msg else None}
        if not scan:report['notes'].append('LiDAR data was unavailable; inspect mapping.log if the script launched mapping.')
        if not mapping_msg:report['notes'].append('No map received; the LiDAR/SLAM map check did not pass.')
        for frame in ('odom','map'):
            try:
                trans=tf.lookup_transform(frame,'base_link',rclpy.time.Time())
                report['tf_'+frame]={'available':True,'stamp_sec':trans.header.stamp.sec}
            except TransformException as error:report['tf_'+frame]={'available':False,'reason':str(error)}
        report['motion_failures']=motion_checks(report['results']) if not args.no_drive else []
        report['motion_checks_passed']=not report['motion_failures'] if not args.no_drive else None
        report['completed']=True
    except BaseException as error:
        stop_motors();report['completed']=False;report['error']=f'{type(error).__name__}: {error}'
        print('\nSTOPPED:',report['error'],flush=True)
    finally:
        for name in ('SIGINT','SIGTERM','SIGHUP'):signal.signal(getattr(signal,name),signal.SIG_IGN)
        stop_motors();watchdog_stop.set()
        if watchdog is not None:watchdog.join(timeout=1)
        for label,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=2)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        if bridge is not None:executor.remove_node(bridge);bridge.destroy_node()
        executor.remove_node(observer);observer.destroy_node();executor.shutdown();rclpy.try_shutdown()
        stack.close()
        if not report.get('completed'):
            report['motion_checks_passed']=False if not args.no_drive else None
        report['notes'].extend(report.get('motion_failures',[]))
        report['notes'].extend(conclusions(report['results']))
        if report.get('gyro',{}).get('full_scale_deg_s')==250:
            report['notes'].append('Gyro is configured for ±250 deg/s (±4.36 rad/s). Faster turns can saturate it; this low-speed test does not deliberately force saturation.')
        if report.get('completed'):
            report['notes'].append('Automatic sequence completed; this does not verify physical forward/left direction, wheel distance scale, or Nav2 path tracking without external observation.')
        (report_dir/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        phase_lines=[]
        def number(value):return 'missing' if value is None else f'{value:+.3f}'
        for name,result in report['results'].items():
            if name.startswith('settle') or name in ('startup','mapping_startup','final_stop_observation'):continue
            phase_lines.append(name+': wheel L/R='+number(result['left_mean'])+'/'+number(result['right_mean'])+
                ' m/s; yaw wheel/IMU/EKF='+number(result['wheel_w_mean'])+'/'+number(result['gyro_w_mean'])+'/'+number(result['ekf_w_mean'])+' rad/s')
        summary='\n'.join(['ROBOT DIAGNOSTIC SUMMARY',f'Sequence completed: {report.get("completed",False)}',
            f'Motion checks passed: {report.get("motion_checks_passed")} (None = not tested)',
            report.get('error',''),*phase_lines,*report['notes'],f'Full report: {report_dir/"report.json"}'])+'\n'
        (report_dir/'summary.txt').write_text(summary)
        print('\n'+summary,flush=True)
    return 0 if report.get('completed') and report.get('motion_checks_passed') is not False else 1


if __name__=='__main__':
    try:sys.exit(main())
    except (TestFault,KeyboardInterrupt) as error:
        print(f'Stopped before test: {error}',file=sys.stderr);sys.exit(1)

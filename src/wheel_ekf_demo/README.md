### How to run

1. Open up 2 terminals on the RPi

2. Run the following in each terminal:
```bash
source /opt/ros/kilted/setup.bash
export ROS_DOMAIN_ID=78
cd ~/Documents/swarm-mono/
```

3. In one terminal run `python3 src/wheel_ekf_demo/robot_demo.py --config src/wheel_ekf_demo/hardware.yaml` to start manual motor control and sensor data fusion

4. In the other terminal run `ros2 launch "$PWD/src/wheel_ekf_demo/mapping.launch.py"` to start slam-toolbox and map publishing

5. On your local machine run the following to start the visualization script:
```bash
ssh -x -t -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8765:127.0.0.1:8765 swarm2 \
  'source /opt/ros/kilted/setup.bash && ROS_DOMAIN_ID=78 python3 ~/Documents/swarm-mono/src/wheel_ekf_demo/web_map.py'
```

6. Connect to `http://127.0.0.1:8765/` in a browser to see results

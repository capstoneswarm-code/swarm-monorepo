### To run demo:
```bash
bash src/wheel_ekf_demo/start_nav2.sh robot_radius:=0.14   hardware_config:="$PWD/src/wheel_ekf_demo/hardware.yaml"
```

### Visualization:

On your local machine run the following to start the visualization script:
```bash
ssh -x -t -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8765:127.0.0.1:8765 swarm2 \
  'source /opt/ros/kilted/setup.bash && ROS_DOMAIN_ID=78 python3 ~/Documents/swarm-mono/src/manual_demo/web_map.py'
```

Connect to `http://127.0.0.1:8765/` in a browser to see results

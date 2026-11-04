# Build natively for the host architecture: amd64 on Intel/AMD, arm64 on Apple Silicon.
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        dbus-x11 \
        libgl1-mesa-dri \
        mesa-utils \
        novnc \
        openbox \
        python3-colcon-common-extensions \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-minimal-tb3-sim \
        ros-jazzy-navigation2 \
        ros-jazzy-ros-gz \
        ros-jazzy-rosbag2 \
        ros-jazzy-rviz2 \
        ros-jazzy-slam-toolbox \
        ros-jazzy-teleop-twist-keyboard \
        websockify \
        x11-utils \
        x11vnc \
        xterm \
        xvfb \
    && mkdir -p /opt/swarm \
    && dpkg-query -W > /opt/swarm/build-packages.txt \
    && rm -rf /var/lib/apt/lists/*

COPY docker/sim/swarm-* /usr/local/bin/
RUN chmod +x /usr/local/bin/swarm-*

ENV DISPLAY=:99 \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /workspace
EXPOSE 6080

# Keep the ROS image's entrypoint, which sources the ROS environment.
CMD ["dbus-run-session", "--", "swarm-desktop"]

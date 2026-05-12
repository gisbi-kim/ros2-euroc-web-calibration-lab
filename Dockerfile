# ROS2 Jazzy + FastAPI browser lab for camera calibration.
# The container expects a prebuilt ROS2 bag mounted at BAG_PATH.

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

ENV BAG_PATH=/external/euroc/MH_01_easy_ros2_ready
ENV SAVED_DIR=/data/saved
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    python3-pip \
    python3-yaml \
    python3-numpy \
    python3-opencv \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-py \
    ros-jazzy-rosbag2-storage-sqlite3 \
    ros-jazzy-rosbag2-transport \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --no-cache-dir \
    fastapi \
    "uvicorn[standard]" \
    aiofiles \
    pydantic \
    gdown

WORKDIR /ros2_ws
COPY ros2_ws/src /ros2_ws/src
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

COPY scripts /opt/euroc_web_lab/scripts
RUN chmod +x /opt/euroc_web_lab/scripts/*.sh \
    && mkdir -p /data/saved

EXPOSE 8080
HEALTHCHECK --interval=10s --timeout=3s --retries=5 CMD /opt/euroc_web_lab/scripts/healthcheck.sh

ENTRYPOINT ["/opt/euroc_web_lab/scripts/entrypoint.sh"]
CMD ["demo"]

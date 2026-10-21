# ROS 2 Humble for the Go2 EDU's Jetson (decision D1).
#
# The EDU ships JetPack 5.1.1 -> Ubuntu 20.04 -> ROS 2 Foxy, which has been
# end-of-life since June 2023 and carries a Nav2 predating the current
# collision monitor and smoother. This container gets a supported Nav2 on
# the robot without touching the stock software or risking the JetPack
# install.
#
# arm64 for the Orin. Everything here is CPU-side: FAST-LIO2, Nav2 and the
# bridge need no CUDA, so this stays on the plain ros:humble base rather
# than an l4t image. That keeps it buildable and testable on any arm64 host.
#
#   docker build -t go2-survey:humble docker/
#   bash docker/run.sh
#
# --network host is REQUIRED, not a convenience: DDS discovery needs
# multicast on the robot's L2 segment, and bridged networking silently
# yields zero topics. See docker/run.sh.

FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# --- system deps -------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      libeigen3-dev libpcl-dev libyaml-cpp-dev libboost-all-dev \
      iw iproute2 iputils-ping tcpdump net-tools \
      python3-pip python3-colcon-common-extensions \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-pcl-ros \
      ros-humble-navigation2 ros-humble-nav2-bringup \
      ros-humble-robot-state-publisher ros-humble-xacro \
      ros-humble-tf2-ros ros-humble-tf2-tools \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-realsense2-camera \
      ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Post-processing only; the survey node itself writes plain CSV.
RUN pip3 install --no-cache-dir numpy scipy matplotlib

# --- Livox SDK2 --------------------------------------------------------------
# Prerequisite for livox_ros_driver2. Pinned rather than tracking a moving
# branch: a silent upstream change here would surface as a LiDAR that no
# longer connects, which is an awful thing to debug mid-survey.
#
# The SDK and the driver below must be a MATCHED pair. v1.2.5 is too old for
# the 1.2.6 driver: the driver references kLivoxLidarTypeMid360s, which the SDK
# only gained in v1.3.x. Bump both together, never one alone.
ARG LIVOX_SDK2_REF=v1.3.1
RUN git clone --depth 1 --branch ${LIVOX_SDK2_REF} \
      https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build \
    && cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" \
    && cmake --install /tmp/Livox-SDK2/build \
    && ldconfig \
    && rm -rf /tmp/Livox-SDK2

# --- workspace ---------------------------------------------------------------
ENV WS=/opt/go2_ws
WORKDIR ${WS}/src

# livox_ros_driver2 uses its own build script rather than plain colcon.
# Pinned to the 1.2.6 RELEASE: its main branch references double-echo SDK
# symbols not present in any Livox-SDK2 release, so main only builds against
# SDK master. 1.2.6 pairs with SDK2 v1.3.1 above.
RUN git clone --depth 1 --branch 1.2.6 https://github.com/Livox-SDK/livox_ros_driver2.git

# FAST-LIO2 with its ikd-Tree submodule. The ROS2 branch is REQUIRED: the
# default branch is ROS 1 only (find_package(catkin)) and cannot build on
# Humble.
RUN git clone --depth 1 --branch ROS2 --recursive https://github.com/hku-mars/FAST_LIO.git

WORKDIR ${WS}
# fast_lio's ROS2 branch does find_package(livox_ros_driver2 REQUIRED), so the
# driver's install must be sourced before fast_lio is configured.
#
# NO --symlink-install here: livox's build.sh runs a full-workspace colcon
# build, so fast_lio is already installed (regular copy) by the time this runs.
# --symlink-install would then fail trying to replace that directory with a
# symlink ("existing path cannot be removed: Is a directory"). We never edit
# fast_lio's source, so symlink install buys nothing anyway.
RUN source /opt/ros/humble/setup.bash \
    && (cd src/livox_ros_driver2 && ./build.sh humble) \
    && source install/setup.bash \
    && colcon build \
         --packages-select fast_lio \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Unitree message types -------------------------------------------------
# unitree_api (Request on /api/sport/request) and unitree_go (SportModeState
# etc.), which go2_survey_bridge needs to drive the robot. Built here for
# Humble from Unitree's official repo rather than borrowed from the host:
# the host's are built for Foxy, and ROS message packages are not compatible
# across distros. Only the two message packages are built -- not the rest of
# unitree_ros2's workspace. Pinned for the same reason as Livox-SDK2.
# (Same approach as dancher00/go2-nav2-wifi's Dockerfile.lidar3d.)
ARG UNITREE_ROS2_REF=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git /tmp/unitree_ros2 \
    && git -C /tmp/unitree_ros2 checkout --quiet ${UNITREE_ROS2_REF} \
    && mkdir -p /tmp/unitree_msgs_ws/src \
    && cp -r /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api \
             /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go \
             /tmp/unitree_msgs_ws/src/ \
    && source /opt/ros/humble/setup.bash \
    && cd /tmp/unitree_msgs_ws \
    && colcon build --merge-install --install-base /opt/unitree_msgs \
         --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /tmp/unitree_ros2 /tmp/unitree_msgs_ws

# This repo is bind-mounted at runtime (see run.sh) rather than baked in, so
# editing a node does not mean rebuilding the image.
ENV GO2_SURVEY_WS=/workspace

# CycloneDDS is what unitree_ros2 speaks. CYCLONEDDS_URI is set at RUN time,
# not here: it names a network interface that only exists on the robot.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

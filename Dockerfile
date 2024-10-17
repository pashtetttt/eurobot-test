FROM ros:humble-ros-core

ENV ROS_DISTRO humble
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src /ros2_ws/src

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

CMD ["ros2", "launch", "turtlesim_path_planner", "path_planner_launch.py"]


FROM osrf/ros:jazzy-desktop

ENV GZ_VERSION=harmonic

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-localization \
    libcgal-dev \
    libfftw3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src/ src/

RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]

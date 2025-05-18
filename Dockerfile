FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    xvfb \
    wmctrl \
    x11-utils \
    libfltk1.3-dev \
    ros-jazzy-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
    matplotlib \
    opencv-python \
    progressbar2 \
    Pillow

RUN [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rm /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

ENV ROS_WS=/root/tb3_stage_explore_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

COPY src/ ${ROS_WS}/src/

RUN apt update && apt install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt update

    
RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select openslam_gmapping"
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && source ${ROS_WS}/install/setup.bash && colcon build --symlink-install"
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY"

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

ENV DISPLAY=:1
ENV screen=0
ENV resolution=800x600x24
RUN echo "Xvfb ${DISPLAY} -screen ${screen} ${resolution} &" >> ~/.bashrc

COPY launch.sh /root/launch.sh
RUN chmod +x /root/launch.sh

WORKDIR ${ROS_WS}/src/tb3_stage_explore
ENTRYPOINT ["/bin/bash", "-i", "/root/launch.sh"]

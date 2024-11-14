FROM abarekatain/ros_noetic_nvidia:2024

USER root

# Update CMake
RUN cd /tmp && \
    wget https://github.com/Kitware/CMake/releases/download/v3.25.1/cmake-3.25.1-linux-x86_64.sh && \
    chmod +x cmake-3.25.1-linux-x86_64.sh && \
    ./cmake-3.25.1-linux-x86_64.sh --prefix=/usr/local --exclude-subdir --skip-license && \
    rm ./cmake-3.25.1-linux-x86_64.sh

# Install vcstools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get update && \
    apt-get install python3-vcstool

RUN apt-get install -y ros-noetic-ros-control ros-noetic-ros-controllers

USER $USERNAME

WORKDIR $CATKIN_WS_PATH

RUN cd $CATKIN_WS_PATH/src && \
    wstool init . && \
    wstool merge -t . https://raw.githubusercontent.com/moveit/moveit/master/moveit.rosinstall && \
    wstool remove  moveit_tutorials && \
    wstool remove  panda_moveit_config && \
    wstool update -t . && \
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro noetic && \
    cd $HOME/catkin_ws/ && \
    catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    catkin build

USER root 

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /*entrypoint.sh

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-actionlib-tools

COPY ./requirements.txt /requirements.txt
RUN /usr/bin/python -m pip install --upgrade pip

RUN pip install --ignore-installed psutil
RUN pip install --upgrade -r /requirements.txt

# Update and install specific packages
RUN apt-get update && apt-get install -y \
    ros-noetic-abb-robot-msgs \
    ros-noetic-abb-egm-msgs \
    ros-noetic-libfranka \
    ros-noetic-abb-rapid-msgs \
    ros-noetic-abb-rapid-sm-addin-msgs \
    ros-noetic-boost-sml \
    && rm -rf /var/lib/apt/lists/*

# Clean up apt cache to reduce image size
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR $HOME
ENTRYPOINT ["/entrypoint.sh"]
FROM nvcr.io/nvidia/tensorrt:21.06-py3 as base

ARG CATKIN_WS_NAME=catkin_ws

ENV TZ=Europe/Rome

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y install tzdata\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y &&\
    # Install build tools, build dependencies and python
    apt-get install -y \
        build-essential \
        software-properties-common \
        libssl-dev libffi-dev \
        ssh \
      	gcc \
      	g++ \
      	gdb \
      	clang \
      	cmake \
      	rsync \
      	tar \
        git \
        wget \
        unzip \
        yasm \
        pkg-config \
        libswscale-dev \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libavformat-dev \
        libpq-dev \
        libxine2-dev \
        libglew-dev \
        libtiff5-dev \
        zlib1g-dev \
        libjpeg-dev \
        libavcodec-dev \
        libavformat-dev \
        libavutil-dev \
        libpostproc-dev \
        libswscale-dev \
        libeigen3-dev \
        libtbb-dev \
        libgtk2.0-dev \
        pkg-config \
        python3-dev \
	    python3-pip \
        python3-venv \
	    qt5-default \
    && rm -rf /var/lib/apt/lists/*


# Install ROS NOETIC
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV LANG=C.UTF-8

ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=noetic

RUN apt update && apt install -y --no-install-recommends \
	ros-noetic-desktop-full \
	python3-rosdep \
	python3-rosinstall \
	python3-vcstools \
	python3-catkin-tools \
	python3-osrf-pycommon\
	python3-rosinstall-generator \
	python3-wstool \
    libgl1-mesa-glx libgl1-mesa-dri mesa-utils xdg-user-dirs \
    libfontconfig1-dev libfreetype6-dev libx11-dev \
    libx11-xcb-dev \
    libxext-dev \
    libxfixes-dev \
    libxi-dev \
    libxrender-dev \
    libxcb1-dev \
    libxcb-glx0-dev \
    libxcb-keysyms1-dev \
    libxcb-image0-dev \
    libxcb-shm0-dev \
    libxcb-icccm4-dev \
    libxcb-sync0-dev \
    libxcb-xfixes0-dev \
    libxcb-shape0-dev \
    libxcb-randr0-dev \
    libxcb-render-util0-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    libxinerama-dev \
    libqt5opengl5-dev \
    tmux \
    iputils-ping \
    iproute2 \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

RUN apt update \
    && apt install -y \
    bash-completion  \
    ros-noetic-plotjuggler-ros  \
    ros-noetic-robot-localization \
    ros-noetic-imu-tools \
    libdw-dev \
    python3-tk \
    htop \
    && rm -rf /var/lib/apt/lists/*

ENV USERNAME abrk
ENV PASSWD pwd
ENV SHELL /bin/bash
ENV HOME /home/${USERNAME}
ENV CATKIN_WS_PATH ${HOME}/${CATKIN_WS_NAME}
ENV PATH $HOME/bin/:$HOME/.local/bin${PATH:+:${PATH}}
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# https://serverfault.com/questions/98842/useradd-with-755-permissions
RUN useradd -m -u 1000 -U -p ${PASSWD} -s ${SHELL} -G video -K UMASK=0022 ${USERNAME}

RUN usermod -a -G sudo ${USERNAME}  && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN usermod --password $(openssl passwd -1 ${PASSWD}) ${USERNAME}

RUN usermod --password $(openssl passwd -1 ${PASSWD}) root

#https://github.com/NVIDIA/nvidia-docker/issues/1438
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN PYTHON_BIN=$(which python) && unlink $PYTHON_BIN && ln -s $(which python3) $PYTHON_BIN


#https://stackoverflow.com/questions/34630571/docker-env-variables-not-set-while-log-via-shell
RUN env | grep _ >> /etc/environment
RUN mkdir /root/.ssh
RUN env | grep _ >> /root/.ssh/environment

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /*entrypoint.sh

USER $USERNAME

RUN xdg-user-dirs-update --force

RUN mkdir -p /tmp/runtime-ros

RUN echo 'export XDG_RUNTIME_DIR=/tmp/runtime-ros' >> ~/.bashrc

RUN mkdir -p $CATKIN_WS_PATH/src

WORKDIR $CATKIN_WS_PATH

RUN catkin init \
    && catkin config --extend /opt/ros/noetic/

RUN echo 'test -e /opt/ros/noetic/setup.bash && source /opt/ros/noetic/setup.bash' >> ~/.bashrc
#RUN echo 'test -e /opt/ros/noetic/setup.bash && source /opt/ros/noetic/setup.bash' >> ~/.bash_profile
#RUN echo 'test -e /opt/ros/noetic/setup.bash && source /opt/ros/noetic/setup.bash' >> ~/.profile

RUN echo 'test -e '$CATKIN_WS_PATH'/devel/setup.bash && source '$CATKIN_WS_PATH'/devel/setup.bash' >> ~/.bashrc
#RUN echo 'test -e '$CATKIN_WS_PATH'/devel/setup.bash && source '$CATKIN_WS_PATH'/devel/setup.bash' >> ~/.bash_profile
#RUN echo 'test -e '$CATKIN_WS_PATH'/devel/setup.bash && source '$CATKIN_WS_PATH'/devel/setup.bash' >> ~/.profile

RUN mkdir ${HOME}/.ssh
RUN env | grep _ >> ${HOME}/.ssh/environment

ENTRYPOINT ["/ros_entrypoint.sh"]

# SSH and ROS master port exposed
EXPOSE 2222 11301 11111 8889 8890

USER root

# Setup ssh remote toolchain for clion and other IDEs
# example: docker run -d --cap-add sys_ptrace -p127.0.0.1:2222:22 --name <container_name> <image_name>
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
    echo 'Port 2222'; \
    echo 'PermitUserEnvironment yes;' \
  ) > /etc/ssh/sshd_config_remote_dev \
   && mkdir -p /run/sshd

USER $USERNAME
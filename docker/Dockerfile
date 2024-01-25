# ARGs to be set before the FROM command
ARG UBUNTU_RELEASE_YEAR=22
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=0
ARG CUDA_PATCH=0

# Start building the image from the nvidia/cuda image
FROM nvidia/cuda:${CUDA_MAJOR}.${CUDA_MINOR}.${CUDA_PATCH}-devel-ubuntu${UBUNTU_RELEASE_YEAR}.04

# ARGs to be set after the FROM command (ARGs that appear before the FROM statement don't exist anymore afterwards)
ARG UBUNTU_RELEASE_YEAR
ARG CUDA_MAJOR
ARG CUDA_MINOR
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=0
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS2_DIST=humble

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

# Disable apt-get warnings
RUN apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*

ENV TZ=Europe/Paris

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libopencv-dev libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

############ Install ROS2 ############

# Set and Check Locale
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y locales && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8 && \
  locale  # verify settings && \
  rm -rf /var/lib/apt/lists/*

# Setup Sources
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y software-properties-common && \
  add-apt-repository universe && \
  apt-get install -y curl && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  rm -rf /var/lib/apt/lists/*

# Install ROS 2 Base packages and Python dependencies
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y \
  ros-${ROS2_DIST}-ros-base \
  ros-${ROS2_DIST}-ament-cmake-clang-format \
  ros-${ROS2_DIST}-image-transport \
  ros-${ROS2_DIST}-image-transport-plugins \
  ros-${ROS2_DIST}-diagnostic-updater \
  ros-${ROS2_DIST}-xacro \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools && \
  pip3 install \
  argcomplete \
  numpy \
  empy \
  lark  && \
  apt-get update || true && \
  apt-get install --no-install-recommends -y \
  python3-colcon-common-extensions && \
  rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup colcon argcomplete and cd for ROS2
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
  echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc

# Setup git autocompletion and colored prompt
RUN curl -o ~/.git-prompt.sh https://raw.githubusercontent.com/git/git/master/contrib/completion/git-prompt.sh && \
  echo "source ~/.git-prompt.sh" >> ~/.bashrc && \
  echo "export GIT_PS1_DESCRIBE_STYLE='contains'" >> ~/.bashrc && \
  echo "export GIT_PS1_SHOWCOLORHINTS='y'" >> ~/.bashrc && \
  echo "export GIT_PS1_SHOWDIRTYSTATE='y'" >> ~/.bashrc && \
  echo "export GIT_PS1_SHOWSTASHSTATE='y'" >> ~/.bashrc && \
  echo "export GIT_PS1_SHOWUNTRACKEDFILES='y'" >> ~/.bashrc && \
  echo "export GIT_PS1_SHOWUPSTREAM='auto'" >> ~/.bashrc && \
  echo "export PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$(__git_ps1)\[\033[00m\]$ '" >> ~/.bashrc

# Customize the .bashrc file adding functions and aliases
RUN echo 'function cd {\n\tbuiltin cd "$@" && ls -F\n}' >> ~/.bashrc && \
  echo "alias ll='ls -alF'" >> ~/.bashrc && \
  echo "alias la='ls -A'" >> ~/.bashrc && \
  echo "alias l='ls -CF'" >> ~/.bashrc && \
  echo "alias open='xdg-open .'" >> ~/.bashrc && \
  echo "alias gitdiff_commit='git difftool HEAD origin/HEAD'" >> ~/.bashrc && \
  echo "alias gitdiff_lastpush='git difftool HEAD^ HEAD'" >> ~/.bashrc && \
  echo "alias gitdiff_unstaged='git difftool'" >> ~/.bashrc && \
  echo "alias cl='clear'" >> ~/.bashrc && \
  echo "alias cb='catkin build -cs'" && \
  echo "alias cct='catkin clean --this'" >> ~/.bashrc && \
  echo "alias cbt='catkin bt -cs'" >> ~/.bashrc && \
  echo "alias sb='source ~/.bashrc'" >> ~/.bashrc && \
  echo "alias eb='code ~/.bashrc'" >> ~/.bashrc

# Install and setup the ZED SDK
RUN echo "CUDA Version $CUDA_VERSION" > /usr/local/cuda/version.txt
RUN /bin/bash -c "apt-get update -y || true && \
  apt-get install --no-install-recommends dialog bash-completion lsb-release wget less udev sudo  build-essential cmake zstd python3 python3-pip libpng-dev libgomp1 -y && \
  python3 -m pip install --upgrade pip; python3 -m pip install numpy opencv-python-headless && \
  wget -q -O ZED_SDK_Linux_Ubuntu.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}${CUDA_MINOR}/ubuntu${UBUNTU_RELEASE_YEAR} && \
  chmod +x ZED_SDK_Linux_Ubuntu.run && \
  ./ZED_SDK_Linux_Ubuntu.run -- silent skip_tools skip_cuda && \
  ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
  rm ZED_SDK_Linux_Ubuntu.run && \
  rm -rf /var/lib/apt/lists/*"

# Install the ZED ROS2 Wrapper
ENV ROS_DISTRO ${ROS2_DIST}

WORKDIR /root/perception_ws/src

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
  apt-get update -y || true && \
  git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
  git clone --recursive https://github.com/stereolabs/zed-ros2-examples.git && \
  cd .. && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' " && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /root/rosbridge_ws/src

# Install ROS1 and build from source


# Export ROS1 and ROS2 install paths
ENV ROS1_INSTALL_PATH ~/ros_catkin_ws/install_isolated
ENV ROS2_INSTALL_PATH /opt/ros/$ROS_DISTRO

# Download and build the ROS1 bridge (https://github.com/ros2/ros1_bridge)
RUN /bin/bash -c "apt-get update -y || true && \
  git clone --recursive https://github.com/ros2/ros1_bridge.git && \
  cd .. && \
  source $ROS1_INSTALL_PATH/setup.bash && \
  source $ROS2_INSTALL_PATH/setup.bash && \
  colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"
  
WORKDIR /root/perception_ws

# Setup environment variables
COPY ros_entrypoint.sh /sbin/ros_entrypoint.sh
RUN sudo chmod 755 /sbin/ros_entrypoint.sh

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]
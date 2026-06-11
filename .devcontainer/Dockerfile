ARG CUDA_VERSION=12.4.1
ARG OS_VERSION=22.04

FROM nvidia/cuda:${CUDA_VERSION}-base-ubuntu${OS_VERSION} AS build

RUN echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80retries

### lerobot dependencies ###
# Define Python version argument
ARG PYTHON_VERSION=3.10

# Configure environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    MUJOCO_GL=egl \
    PATH=/lerobot/.venv/bin:$PATH \
    CUDA_VISIBLE_DEVICES=0 \
    TEST_TYPE=single_gpu \
    DEVICE=cuda

# Install Python, system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common build-essential git curl \
    libglib2.0-0 libgl1-mesa-glx libegl1-mesa libvulkan1 mesa-vulkan-drivers ffmpeg \
    libxcb-cursor0 \
    libusb-1.0-0-dev speech-dispatcher libgeos-dev portaudio19-dev \
    cmake pkg-config ninja-build \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       python${PYTHON_VERSION} \
       python${PYTHON_VERSION}-venv \
       python${PYTHON_VERSION}-dev \
    && curl -LsSf https://astral.sh/uv/install.sh | sh \
    && mv /root/.local/bin/uv /usr/local/bin/uv 


### ros ###
# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y --no-install-recommends tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS Apt sources
RUN curl -L -s -o /tmp/ros2-apt-source.deb https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.$(lsb_release -cs)_all.deb \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble

# Install ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    python3-argcomplete \
    python3-rosdep \
  && rm -rf /var/lib/apt/lists/*

# Expose the nvidia driver to allow opengl
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1
ENV DEBIAN_FRONTEND=

RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-humble-ament-* \
  vim \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python-is-python3

RUN pip install pyrealsense2

# install moveit
# https://moveit.ai/install-moveit2/binary/
# install realsense
# https://github.com/realsenseai/realsense-ros
# ros-humble-usb-cam
RUN apt update && apt install -y \
    ros-humble-moveit \
    ros-humble-realsense2-* \
    ros-humble-usb-cam \
    ros-humble-topic-tools


FROM build AS dev 

# change user 
ARG USERNAME=ubuntu 
ARG USER_UID=1000 
ARG USER_GID=$USER_UID 
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \ 
    groupadd --gid $USER_GID $USERNAME && \ 
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \ 
    fi 
# Add sudo support for the non-root user 
RUN apt-get update && \ 
    apt-get install -y sudo && \ 
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \ 
    chmod 0440 /etc/sudoers.d/$USERNAME 
# Switch from root to user 
USER $USERNAME 

# Add user to video group to allow access to webcam 
RUN sudo usermod --append --groups video,plugdev $USERNAME 

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    vim \
    nano \
    less \
    tmux

RUN echo "export PATH=\"$HOME/.local/bin:$PATH\"" >> /home/ubuntu/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc  && \
    echo "source /home/ubuntu/techin517/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /home/ubuntu/.bashrc && \
    echo "export HF_USER=ubuntu" >> /home/ubuntu/.bashrc && \
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-ubuntu" >> /home/ubuntu/.bashrc

# Default shell
CMD ["/bin/bash"]

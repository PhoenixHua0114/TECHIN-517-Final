sudo apt-get install -y \
    ros-humble-usb-cam \
    ros-humble-realsense2-* \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-apriltag-ros \
    ros-humble-topic-tools

sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -y


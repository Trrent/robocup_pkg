FROM ros:noetic
SHELL ["/bin/bash", "-ci"]
ENV DEBIAN_FRONTEND noninteractive

ENV ROS_DISTRO=noetic
ENV DESTIN=/catkin_ws
WORKDIR $DESTIN

RUN mkdir -p $DESTIN/src && cd $DESTIN && echo "source /opt/ros/${ROS_DISTRO}/setup.bash">~/.bashrc
RUN catkin_make && echo "source $DESTIN/devel/setup.bash">~/.bashrc
RUN apt update && apt install -y libeigen3-dev ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-control-msgs ros-${ROS_DISTRO}-pr2-msgs \
    ros-${ROS_DISTRO}-angles ros-${ROS_DISTRO}-control-toolbox ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-kdl-parser libcppunit-dev ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-gazebo-ros
RUN apt install -y ros-${ROS_DISTRO}-xacro  ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-effort-controllers \ 
    ros-${ROS_DISTRO}-position-controllers \ 
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-controller


COPY . $DESTIN/src
RUN catkin_make
# CMD [""]

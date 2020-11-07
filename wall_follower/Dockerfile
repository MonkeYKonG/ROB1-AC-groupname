FROM osrf/ros2:devel

RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update=locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN apt-get update
RUN apt-get install -y \
    curl \
    python3-colcon-common-extensions \
    libgazebo11-dev \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-cartographer \
    ros-foxy-cartographer-ros \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup

WORKDIR /code

# RUN curl https://raw.githubusercontent.com/MonkeYKonG/ROB1-AC-groupname/master/project.repos > project.repos
# RUN vcs import < project.repos
# RUN colcon build

CMD ls /opt
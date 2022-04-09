FROM introlab3it/rtabmap_ros:noetic

# set a directory for the app
WORKDIR /usr/src/app

# install dependencies
RUN sh -c '. /etc/lsb-release && echo "deb http://packages.ros.org.ros.informatik.uni-freiburg.de/ros/ubuntu $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-get -qq update \
    && apt-get -qq install -y --no-install-recommends \
        bash \
        zip \
        git \
        default-jre \
        gnupg \
        gnupg2 \
        gnupg1 \
        curl \
        flex \
        bison \
        ros-noetic-rosbridge-* \
        build-essential \
        cmake \
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
        libopenjp2-7-dev \
        libavformat-dev \
        libpq-dev \
        htop \
        vim \
        nano \
        tmux \
        build-essential \
        libbz2-dev \
        libxml2-dev \
        libzip-dev \
        libboost-all-dev \
        lua5.2 \
        liblua5.2-dev \
        ros-noetic-smach* 

RUN mkdir /home/l3x
WORKDIR /home/l3x

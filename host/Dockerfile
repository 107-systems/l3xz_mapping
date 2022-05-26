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
        net-tools \
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
        ros-noetic-smach* \
        chrony \
        python3-pip

COPY ./chrony.conf /etc/chrony

RUN pip3 install geopy
RUN pip3 install utm
RUN pip3 install requests
RUN pip3 install gpxpy
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN mkdir /home/log
RUN mkdir /home/l3xz
WORKDIR /home/l3xz

ENTRYPOINT ["/entrypoint.sh"]

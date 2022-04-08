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

# Build OpenCV
ENV OPENCV_VERSION="4.5.1"

RUN wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -O opencv.zip \
    && unzip -qq opencv.zip -d /opt \
    && rm -rf opencv.zip \
    && cmake \
        -D BUILD_TIFF=ON \
        -D BUILD_opencv_java=OFF \
        -D WITH_CUDA=OFF \
        -D WITH_OPENGL=OFF \
        -D WITH_OPENCL=OFF \
        -D WITH_IPP=OFF \
        -D WITH_TBB=ON \
        -D WITH_EIGEN=ON \
        -D WITH_V4L=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D CMAKE_BUILD_TYPE=RELEASE \
      	-D CMAKE_BUILD_TYPE=RELEASE \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_PC_FILE_NAME=opencv.pc \
       /opt/opencv-${OPENCV_VERSION} \
    && make -j$(nproc) install \
    && rm -rf /opt/build/* \
    && rm -rf /opt/opencv-${OPENCV_VERSION} \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq autoremove \
    && apt-get -qq clean

RUN mkdir /home/l3x
WORKDIR /home/l3x

# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM osrf/ros:humble-desktop-full

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
    
# Install PCL & Eigen & essential libraries
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        cppcheck \
        gdb \
        git \
        sudo \
        vim \
        wget \
        tmux \
        curl \
        less \
        htop \
        libsm6 libxext6 libgl1-mesa-glx libxrender-dev \ 
        curl \
 && apt-get clean

RUN export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean
 
RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip git python3-pip
RUN apt-get install -y ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-pcl-ros

RUN apt-get install -y libceres-dev 
    

RUN apt-get install -y python-tk python3-pip

# Python 3.
RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir numpy scipy matplotlib ipython opencv-python 
RUN pip3 install --no-cache-dir pandas visdom scikit-image scikit-learn numba Pillow
RUN pip3 install --no-cache-dir pyyaml pycryptodome bagpy Cython

RUN apt-get -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# RUN sudo rosdep init
RUN rosdep update
RUN pip3 install -U catkin_tools gym
RUN apt-get -y install ros-humble-grid-map-msgs ros-humble-grid-map

# Install livox SDK
WORKDIR /root/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /root/Livox-SDK2
RUN mkdir build
WORKDIR /root/Livox-SDK2/build
RUN cmake .. && make -j2 && make install

WORKDIR /root/
RUN git clone http://github.com/strasdat/Sophus.git \
    && cd Sophus && git checkout 97e7161 \
    && mkdir build && cd build && cmake .. -DBUILD_TESTS=OFF \
    && make -j8 && sudo make install
WORKDIR /root/
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && git checkout 4abef92 &&\
    mkdir build && \
    cd build && \
    cmake  \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
           .. && \
    make -j6 && \
    sudo make install


RUN /bin/bash -c '. /opt/ros/humble/setup.bash;'

RUN pip install rerun-sdk
RUN pip install numpy==1.24.3 matplotlib==3.7.1
RUN pip install --upgrade packaging
RUN pip3 install torch --ignore-installed


WORKDIR /root/ros2_ws

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]


























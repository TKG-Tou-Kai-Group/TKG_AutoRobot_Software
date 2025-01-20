FROM nvidia/cuda:12.3.1-devel-ubuntu22.04
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN apt -y -qq update && apt -y -qq install software-properties-common mesa-utils libglu1-mesa-dev\
  freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev \
  libao-dev libmpg123-dev libasio-dev && \
  add-apt-repository universe && \
  apt -y -qq update && apt install curl -y && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  apt -y -qq update && \
  apt -y -qq upgrade && \
  apt -y -qq install ros-humble-desktop
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

RUN set -x && \
  apt-get update -y -qq && \
  apt-get install -y -qq \
    ros-humble-pluginlib ros-humble-urdf ros-humble-yaml-cpp-vendor && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq \
    libyaml-cpp-dev \
    vim v4l-utils exfat-* \
    openssh-server \
    terminator dbus-x11 git \
    python3-pip && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

# for ros2
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    ninja-build \
    python3-colcon-ros \
    qtcreator \
    libusb-dev libusb-1.0-0-dev && \
    apt-get install -y -q \
    libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl \
    gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev libopen3d-dev libfmt-dev libunwind-dev libgoogle-glog-dev liblua5.2-dev \
    && apt-get clean

RUN set -x && \
  pip3 install -U pip && \
  pip3 install pyserial numpy scipy

# for opencv with cuda
RUN cd /opt && \
  git clone https://github.com/opencv/opencv.git && \
  git clone https://github.com/opencv/opencv_contrib.git && \
  cd opencv; git checkout -b 4.5.0 && \
  cd ../opencv_contrib; git checkout -b 4.5.0 && \
  cd ../opencv && mkdir build && cd build/ && \
  CUDA_PATH="/usr/local/cuda-11" CFLAGS="-I/usr/local/cuda-11/include" LDFLAGS="-L/usr/local/cuda-11/lib64" \
  cmake .. -D CMAKE_BUILD_TYPE=RELEASE \
  -D BUILD_opencv_world=OFF \
  -D BUILD_opencv_apps=ON \
  -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
  -D OpenBLAS_INCLUDE_DIR=/usr/include/x86_64-linux-gnu \
  -D OpenBLAS_LIB=/usr/lib/x86_64-linux-gnu/openblas-pthread/libopenblas.so \
  -D WITH_PYTHON=ON \
  -D BUILD_opencv_python2=OFF \
  -D BUILD_opencv_python3=ON \
  -D PYTHON_DEFAULT_EXECUTABLE=python3 \
  -D WITH_MKL=ON \
  -D WITH_CUDA=ON \
  -D WITH_GSTREAMER=ON \
  -D OPENCV_GAPI_GSTREAMER=ON \
  -D CUDA_FAST_MATH=ON \
  -D WITH_CUBLAS=ON \
  -D WITH_CUDNN=ON \
  -D WITH_NVCUVID=OFF \
  -D OPENCV_DNN_CUDA=OFF \
  -D BUILD_opencv_cudaimgproc=OFF \
  -D BUILD_EXAMPLES=ON \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_TESTS=ON \
  -D INSTALL_C_EXAMPLES=ON \
  -D INSTALL_BIN_EXAMPLES=ON \
  -D INSTALL_PYTHON_EXAMPLES=ON && \
  make -j8 2>&1 |tee make.log && \
  make install && ldconfig  


# for realsense
RUN cd /opt && \
  git clone https://github.com/IntelRealSense/librealsense.git -b v2.54.1 && \
  cd librealsense && \
  #./scripts/patch-realsense-ubuntu-L4T.sh  && \
  mkdir build && cd build && \
  cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=true -DCMAKE_BUILD_TYPE=release && \
  make -j8 && \
  make install

RUN apt update && apt install -qq -y ros-humble-realsense2-* ros-humble-laser-proc
RUN pip3 install pyusb
RUN echo 'source /root/colcon_ws/install/setup.sh' >> ~/.bashrc

COPY config/99-realsense-libusb.rules /etc/udev/rules.d/
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENTRYPOINT ["/bin/bash"]

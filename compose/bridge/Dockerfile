ARG PARENT
FROM ${PARENT}

RUN apt-get update --fix-missing
RUN apt-get install -y libzmq3-dev \
                       git \
                       build-essential \
                       autoconf \
                       libtool \
                       libeigen3-dev \
                       cmake \
                       vim \
                       ros-${ROS_DISTRO}-ackermann-msgs \
                       ros-${ROS_DISTRO}-map-server \
                       ros-${ROS_DISTRO}-genpy \
                       ros-${ROS_DISTRO}-rviz

RUN cp -r /usr/include/eigen3/Eigen /usr/include

RUN git clone https://github.com/protocolbuffers/protobuf.git && \
    cd protobuf && \
    git checkout tags/v3.8.0 && \
    git submodule update --init --recursive && \
    ./autogen.sh && \
    ./configure && \
    make -j8 && \
    make install && \
    ldconfig && \
    make clean && \
    cd .. && \
    rm -r protobuf

RUN apt install -y python3-pip && pip3 install numpy

# Install F1Tenth GYM
#RUN git clone -b cpp_backend_archive --depth=1 https://github.com/f1tenth/f1tenth_gym /f1tenth_gym
#RUN cd f1tenth_gym && mkdir build && cd build && cmake .. && make
#RUN cd f1tenth_gym && cp ./build/sim_requests_pb2.py ./gym/ && pip install -e gym/
COPY ./gym /f1tenth/gym
RUN pip install -e /f1tenth/gym/

# Install F1Tenth GYM Bridge
RUN mkdir -p /catkin_ws/src
COPY ./f1tenth_gym_ros /catkin_ws/src/f1tenth_gym_ros
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"

ENV F1TENTH_EXEC_DIR=/f1tenth_gym/build/

COPY ./compose/bridge/start.sh /start.sh
RUN chmod +x /start.sh
CMD ["/start.sh"]
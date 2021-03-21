FROM osrf/ros:kinetic-desktop-full

ARG SSH_PRIVATE_KEY


LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
#ADD non_ros_dependencies non_ros_dependencies
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list

ENV ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
ENV ROS_ROOT=/opt/ros/kinetic/share/ros
ENV ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/catkin_ws/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks
ENV ROS_MASTER_URI=http://localhost:11311
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/catkin_ws/devel/lib:/root/catkin_ws/devel/lib/x86_64-linux-gnu:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/ros/kinetic/lib
ENV CATKIN_TEST_RESULTS_DIR=/root/catkin_ws/build/test_results
ENV CPATH=$CPATH:/root/catkin_ws/devel/include:/opt/ros/kinetic/include
ENV ROS_TEST_RESULTS_DIR=/root/catkin_ws/build/test_results
ENV PATH=$PATH:/root/catkin_ws/devel/bin:/opt/ros/kinetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV ROSLISP_PACKAGE_DIRECTORIES=/root/catkin_ws/devel/share/common-lisp
ENV ROS_DISTRO=kinetic
ENV PYTHONPATH=$PYTHONPATH:/root/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages
ENV PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/root/catkin_ws/devel/lib/pkgconfig:/root/catkin_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/kinetic/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/kinetic/lib/pkgconfig
ENV CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/root/catkin_ws/devel:/opt/ros/kinetic

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility


RUN apt-get update && apt-get install -y \
        ros-kinetic-gazebo-msgs \
        ros-kinetic-gazebo-ros \
        ssh \
        ros-kinetic-gazebo-dev
RUN apt-get install -y \
        ros-kinetic-gazebo-ros-pkgs \
        ros-kinetic-ros-control \
        ros-kinetic-ros-controllers \
        ros-kinetic-rosserial-client \
        ros-kinetic-rosserial-arduino \
        ros-kinetic-rosserial-server \
        ros-kinetic-rosserial-msgs \
        ros-kinetic-rosserial-python \
        ros-kinetic-ros-control \
        ros-kinetic-ros-controllers \
        ros-kinetic-joy \
        ros-kinetic-geographic-msgs \
    && apt-get clean



RUN mkdir /root/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && chmod 400 /root/.ssh/id_rsa

RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

WORKDIR /tmp
RUN git clone git@github.com:smilykoch/roberto_robot.git
RUN rm /root/.ssh/id_rsa

WORKDIR /catkin_ws
RUN cp -r /tmp/roberto_robot/src .
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash \
          && catkin_make --pkg roberto_msgs \
          && source /catkin_ws/devel/setup.bash'


COPY src/ ./src
RUN /bin/bash -c 'catkin_make'

CMD ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && source /catkin_ws/devel/setup.bash && echo $GAZEBO_MODEL_PATH && roslaunch roberto_gazebo roberto_line_test.launch"]

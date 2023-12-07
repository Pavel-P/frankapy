FROM ros:noetic-ros-core
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gnome-terminal \
    dbus-x11 \
    build-essential \
    gcc \
    g++ \
    sshpass \
    python3 \
    python3-pip \
    python3-rosdep \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    && rm -rf /var/lib/apt/lists/*

RUN echo "export ROS_MASTER_URI='http://192.168.1.243:11311'" >> /etc/bash.bashrc 
RUN echo "export ROS_IP='192.168.1.243'" >> /etc/bash.bashrc 
RUN ln -s /bin/python3 /bin/python

WORKDIR /opt/frankapy
COPY . .

RUN pip3 install --upgrade numpy
RUN pip3 install -q -e .
RUN cd catkin_ws && source /ros_entrypoint.sh && catkin_make
RUN echo "source /opt/frankapy/catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

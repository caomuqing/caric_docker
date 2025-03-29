# Use osrf/ros:noetic-desktop-full as the base image
FROM xuxinhang007/caric:v1

# Update and install additional dependencies
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-turtlebot3 \
    python3-rosdep \
    git && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Set up a catkin workspace
#RUN mkdir -p /root/catkin_ws/src


RUN sudo apt-get update && \
	apt-get install -y python3-wstool python3-catkin-tools python3-empy \
                     protobuf-compiler libgoogle-glog-dev \
                     ros-noetic-control-toolbox \
                     ros-noetic-octomap-msgs \
                     ros-noetic-octomap-ros \
                     ros-noetic-mavros \
                     ros-noetic-mavros-msgs \
                     ros-noetic-rviz-visual-tools \
                     ros-noetic-gazebo-plugins \
                     python-is-python3 \
                     python3-pip \
                     ros-noetic-octomap-server \
                     libarmadillo-dev \
                     libelf-dev \
                     libdw-dev 

RUN pip3 install scipy

RUN mkdir -p /root/library && \
    cd /root/library && \
    wget https://github.com/stevengj/nlopt/archive/v2.7.1.tar.gz && \
    tar -xzf v2.7.1.tar.gz && \
    cd nlopt-2.7.1 && mkdir build && cd build && cmake .. && \
    make && make install

RUN sudo apt update && sudo apt install locales && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# RUN sudo apt install -y software-properties-common
# RUN sudo add-apt-repository universe
# RUN sudo apt update && sudo apt install curl -y
# RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# RUN sudo apt update
# RUN sudo apt install -y ros-foxy-desktop python3-argcomplete ros-foxy-ros1-bridge 


    
# Clone additional ROS packages from GitHub (replace these URLs with packages you need)
# Set up the catkin workspace and clone additional packages
# RUN mkdir -p /root/ws_caric/src && \
#     cd /root/ws_caric/src && \
#     git clone https://github.com/ntu-aris/caric_mission && \
#     git clone https://github.com/ntu-aris/rotors_simulator && \
#     git clone https://github.com/ntu-aris/velodyne_simulator && \
#     git clone https://github.com/ntu-aris/unicon && \
#     git clone https://github.com/ntu-aris/traj_gennav

# RUN cd /root/ws_caric/
WORKDIR /root/ws_caric
# RUN touch /root/ws_caric/src/rotors_simulator/rotors_hil_interface/CATKIN_IGNORE
#RUN catkin build traj_gennav
#RUN catkin build

# Install package dependencies
# RUN apt-get update && \
#     rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build traj_gennav"
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Source the workspace setup in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

COPY fastdds.xml /root/fastdds.xml

# Set the default command to launch a bash shell
CMD ["bash"]

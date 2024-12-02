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
#RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the default command to launch a bash shell
CMD ["bash"]

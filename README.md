# caric_docker
### **docker for running Cooperative Aerial Robot Inspection Challenge (CARIC)**

Preliminary: make sure you have docker installed: [instruction](https://docs.docker.com/engine/install/ubuntu/).

The main CARIC components (simulator and controller) are run in a ROS1 docker container, and we provide the ROS1-ROS2 bridge to allow users to develop their algorithm in ROS2 dockers. If you want to develop in ROS1, you only need to run the ros1 container.

## Run CARIC in a ROS1 docker container and develop in ROS1
Clone this package and build docker:
```bash
git clone https://github.com/caomuqing/caric_docker
cd caric_docker
docker compose build ros_caric
```
If the build succeeds, run the following command to run the docker container and build the ros workspace:

```bash
docker compose up ros_caric
```
After building the ros package, do not close the terminal; open a new terminal. Run the following command to enter the container:

```bash
xhost + #allowing allow docker access to screen
docker exec -it ros_caric_container_1 bash # open a container terminal
```
To run the baseline method:
```bash
source devel/setup.bash
roslaunch caric_baseline run.launch scenario:=mbs #other scenarios: hangar, crane
```
You should see rviz opening up and the drones start moving. You can develop your solution by creating new packages in the workspace `ws_caric/src`. An example package `ws_caric/src/caric_baseline` is provided.

Remember to run `docker compose down` to shut down the containers when you finish running.


## Run CARIC in a ROS1 container and develop your solution in ROS2 container
Clone this package and build docker:
```bash
git clone https://github.com/caomuqing/caric_docker
cd caric_docker
docker compose build
```
If the build succeeds, run the following command to run the docker container and build the ros workspace:

```bash
docker compose up
```
After building the ros package, do not close the terminal; open a new terminal. Run the following command to enter the container:

```bash
xhost + #allowing allow docker access to screen
docker exec -it ros_caric_container_1 bash # open a container terminal
```
To run the baseline method:
```bash
source devel/setup.bash
roslaunch caric_baseline run.launch scenario:=mbs
or roslaunch caric_mission run_mbs.launch
```

Open a new terminal and go into the ros_bridge container
```bash
docker exec -it ros_caric_bridge bash
rosparam load bridge.param #load the ros1-ros2 bridge parameter
ros2 run ros1_bridge parameter_bridge #run ros1_bridge
```

Open a new terminal of ros_bridge container and run ros2 domain bridge:
```bash
docker exec -it ros_caric_bridge bash
ros2 run domain_bridge domain_bridge domain_bridge.yaml
```

In the new docker ros_caric_drone (ROS_DOMAIN_ID=1), you should be able to get the ros2 topic of drone status:
```bash
docker exec -it ros_caric_drone bash
ros2 topic echo /sentosa/gimbal
```

The file `domain_bridge.yaml` governs the topics to be transferred across domains in ROS2.

The file `bridge.param` governs the topics to be bridged from ROS1 to ROS2.

Remember to run `docker compose down` to shut down the containers when you finish running.

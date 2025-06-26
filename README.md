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

```
Open a new terminal and go into the ros_bridge container
```bash
docker exec -it ros_caric_bridge bash
rosparam load bridge.param #load the ros1-ros2 bridge parameter
ros2 run ros1_bridge parameter_bridge #run ros1_bridge
```

The file `bridge.param` controls which topics are bridged from ROS1 to ROS2.

> **Note:** You can adjust message transmission between domains by editing the configuration files (such as `bridge.param`).

### Configuration Adjustment

Each node is assigned a domain ID using the `TARGET_TO_DOMAIN` mapping:

<p align="center">

| Domain ID | Node                   |
|:---------:|:----------------------:|
| 0         | Central domain         |
| 1         | Jurong UAV             |
| 2         | Raffles UAV            |
| 3         | Sentosa UAV            |
| 4         | Changi UAV             |
| 5         | Nanyang UAV            |
| 99        | Ground Control Station |

</p>

You can modify the configuration files in `caric_mission_ros2/config` to adjust:
- Message routing rules between domains
- Communication parameters and frequency
- Message reception strategies for each UAV node

Customize these parameters to suit your multi-domain communication needs.

## Run PPCom (Peer-to-Peer Communication) System

The PPCom system enables distributed communication between multiple drone nodes across different ROS2 domains.

For detailed instructions, see the [caric_mission_ros2 README](caric_mission_ros2/README.md).

### View ROS2 Topic List

To view topics in a specific ROS2 domain, set the `ROS_DOMAIN_ID` environment variable.  
For example:
```bash
docker exec -it ros_caric_drone bash
export ROS_DOMAIN_ID=1
ros2 topic list
```
> **Notice:** The topics you see here are processed through the Line-of-Sight (LOS) communication mechanism.

Set `ROS_DOMAIN_ID` to the desired domain number to see all ROS2 topics in that domain. This is useful for debugging cross-domain communication and topic mapping.

The file `bridge.param` governs the topics to be bridged from ROS1 to ROS2.

Remember to run `docker compose down` to shut down the containers when you finish running.

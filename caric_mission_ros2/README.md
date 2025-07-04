# PPCom (Peer-to-Peer Communication) System

The PPCom system enables distributed communication between multiple drone nodes across different ROS2 domains. Follow these steps to run the complete system:

### Step 1: Run CARIC in ROS1 Container
First, start the ROS1 container and run the baseline or mission launch files:

```bash
xhost + # allowing docker access to screen
docker exec -it ros_caric_container_1 bash # open a container terminal
```

To run the baseline method:
```bash
source devel/setup.bash
roslaunch caric_baseline run.launch scenario:=mbs
# or
roslaunch caric_mission run_mbs.launch
```

### Step 2: Run ROS1-ROS2 Bridge
Open a new terminal and start the bridge container:

```bash
docker exec -it ros_caric_bridge bash
rosparam load bridge.param # load the ros1-ros2 bridge parameter
ros2 run ros1_bridge parameter_bridge # run ros1_bridge
```

### Step 3: Run PPCom Router in Domain 0
Open a new terminal and enter the ROS2 container in domain 0 to run the PPCom router:
```bash
docker exec -it ros_caric_drone bash
ros2 launch caric_mission_ros2 multi_bridge.launch.py
```
### Step 4: Run Talker Nodes (Only for testing ping_message)

This step is only for testing the ping/message functionality between nodes.

Open separate terminals for each drone talker node. Each node should run in the ROS2 container:

For Sentosa talker:
```bash
docker exec -it ros_caric_drone bash
ros2 run caric_mission_ros2 ppcom_sentosa_talker.py
```

For Raffles talker:
```bash
docker exec -it ros_caric_drone bash
ros2 run caric_mission_ros2 ppcom_raffles_talker.py
```

For Jurong talker:
```bash
docker exec -it ros_caric_drone bash
ros2 run caric_mission_ros2 ppcom_jurong_talker.py
```

For Nanyang talker:
```bash
docker exec -it ros_caric_drone bash
ros2 run caric_mission_ros2 ppcom_nanyang_talker.py
```

For Changi talker:
```bash
docker exec -it ros_caric_drone bash
ros2 run caric_mission_ros2 ppcom_changi_talker.py
```

### Expected Output
After running the talker nodes, you should see message sending and receiving status in the terminal output:
- **Green text**: Successfully received messages from other nodes
- **Red text**: No messages received from specific nodes
- **Blue text**: Sending status indicators

Each talker will periodically send ping messages and display the communication status with other nodes in the network.

Remember to run `docker compose down` to shut down all containers when you finish running.
# Launch the drones
source ~/workspace/ws_caric_racer/devel/setup.bash
(
    sleep 5;
    wait;
    
    (
        rosservice call /sentosa/traj_gennav/readfile;
        rosservice call /sentosa/traj_gennav/execute_path;
    ) &
    (
        rosservice call /changi/traj_gennav/readfile;
        rosservice call /changi/traj_gennav/execute_path;
    ) &
    (
        rosservice call /nanyang/traj_gennav/readfile;
        rosservice call /nanyang/traj_gennav/execute_path;
    ) 
) & \
roslaunch caric_mission demo_paths_inspector.launch

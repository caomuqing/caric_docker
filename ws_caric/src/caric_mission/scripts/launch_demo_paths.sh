# Build the workspace
catkin build;

# Launch the drones

(
    sleep 5;
    wait;
    
    (
        rosservice call /jurong/traj_gennav/readfile;
        rosservice call /jurong/traj_gennav/execute_path;
    ) &

    (
        rosservice call /raffles/traj_gennav/readfile;
        rosservice call /raffles/traj_gennav/execute_path;
    ) &

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
roslaunch caric_mission demo_paths.launch
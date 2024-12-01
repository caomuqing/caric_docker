
# Read the trajectory
rosservice call /jurong/traj_gennav/readfile
# rosservice call /raffles/traj_gennav/readfile
# rosservice call /sentosa/traj_gennav/readfile
# rosservice call /changi/traj_gennav/readfile
# rosservice call /nanyang/traj_gennav/readfile

# Launch the drone, one after another
rosservice call /jurong/traj_gennav/execute_path && sleep 3;
# rosservice call /raffles/traj_gennav/execute_path && sleep 3;
# rosservice call /sentosa/traj_gennav/execute_path && sleep 3;
# rosservice call /changi/traj_gennav/execute_path && sleep 3;
# rosservice call /nanyang/traj_gennav/execute_path && sleep 3;

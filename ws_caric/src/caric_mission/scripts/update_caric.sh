#!/bin/bash

source ../../../devel/setup.bash
roscd caric_mission && git pull;
roscd rotors_simulator && git pull;
roscd unicon && git pull;
roscd traj_gennav && git pull;
roscd velodyne_simulator && git pull;
roscd caric_mission/scripts
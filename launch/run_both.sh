#!/bin/bash

rosbag play /home/nick/catkin_ws/smartbases/ft_carson_data/20201117_185358_2.bag &

roslaunch velo_combo dangers.launch &

roslaunch velo_combo dangers_right.launch &
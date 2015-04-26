#!/bin/bash


rm -r ~/catkin_ws/src/$1
cp -r ~/HKUST-ML3-14/$1 ~/catkin_ws/src/$1
<<<<<<< HEAD
cd ~/catkin_ws
catkin_make
=======
catkin_make --source ~/catkin_ws/src
>>>>>>> master

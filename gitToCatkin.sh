#!/bin/bash


rm -r ~/catkin_ws/src/*
cp -r ~/HKUST-ML3-14/$1 ~/catkin_ws/src/$1
cd ~/catkin_ws
catkin_make

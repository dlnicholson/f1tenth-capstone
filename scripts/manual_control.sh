#!/bin/bash

#begin teleop.launch for f1tenth

set -e

if [ $1 = "" ]
then
	echo "Pass folder";
	exit;
else
	cd $1
	cd f1tenth_ws

	catkin_make
	source devel/setup.bash
	roslaunch racecar teleop.launch
fi

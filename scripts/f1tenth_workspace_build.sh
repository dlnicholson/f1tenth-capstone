#!/bin/bash

#build workspace for f1tenth

#pass folder to install into

set -e

if [ $1 = "" ]
then
	echo "Pass folder";
	exit;

else
	cd $1
	git clone https://github.com/f1tenth/f1tenth_system
	
	mkdir -p f1tenth_ws/src
	cp -r f1tenth_system f1tenth_ws/src/

	sudo apt-get install

	sudo apt-get install ros-melodic-driver-base

	cd f1tenth_ws
	find . -name "*.py" -exec chmod +x {} \;


fi



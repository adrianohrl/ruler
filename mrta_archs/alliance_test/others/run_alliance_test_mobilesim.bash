#!/bin/bash

USAGE="Usage: run_alliance_test_mobilesim <map name>

  Author: Adriano Henrique Rossette Leite
  Version: 1.0.0
  Description: This script runs multiple Adept Pioneer 3 DX 
               robots in the MobileSim simulator for Multi 
               Robot System (MRS) application simulations.
"

if [[ ${#} = 0 ]]; then
	echo "${USAGE}"
	exit
fi

MAP_PATH="others/map/${1}.map"
ROS_PKG="alliance_test"
NUMBER_ROBOTS=3

./run_mobilesim.bash -m ${MAP_PATH} -p ${ROS_PKG} -n ${NUMBER_ROBOTS}
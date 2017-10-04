# Application Simulation

These tools launch the simulator [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) for MRS applications.

Usage:

    run_alliance_test_mobilesim <map name>

    run_mobilesim [--help | -h] 
                  [--map | -m <map path>] 
                  [--num-robots | -n <number of robots>]
                  [--ros-pkg | -p <ROS package name>]

-------------------

# run_alliance_test_mobilesim

This tool simplifies the usage of the run_mobilesim tool for the ROS package alliance_test.

So, in order to use these tools, firstly enter the following command to a terminal:

	roscd alliance_test/others
	chmod +x run_alliance_test_mobilesim run_mobilesim

And then, just enter:

	./run_alliance_test_mobilesim.bash map1

-------------------

# run_mobilesim

This script runs multiple Adept Pioneer 3 DX robots in the MobileSim simulator for Multi Robot System (MRS) application simulations.

Firstly, check if the /usr/local/MobileSim/PioneerRobotModels.world.inc file contains the following definitions:

	# This is a quick hack to have several differently 
	# colored Pioneers in the same simulation:
	define red_p3dx p3dx (color "red")
	define black_p3dx p3dx (color "black")
	define blue_p3dx p3dx (color "blue")
	define violet_p3dx p3dx (color "violet")
	define green_p3dx p3dx (color "green")
	define orange_p3dx p3dx (color "orange")
	define magenta_p3dx p3dx (color "magenta")
	define cyan_p3dx p3dx (color "cyan")
	define purple_p3dx p3dx (color "purple")
	define yellow_p3dx p3dx (color "yellow")
	define pink_p3dx p3dx (color "pink")

-------------------

# Creating maps

Use the application [Mapper3-Basic](http://robots.mobilerobots.com/wiki/Mapper3Basic) to create/edit maps in order to define the space to simulate in the simulator [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) according to the desired application.